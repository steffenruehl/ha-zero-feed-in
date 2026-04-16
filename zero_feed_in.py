"""
Zero Feed-In PI Controller for Zendure SolarFlow 2400 AC+

Bidirectional PI controller that keeps the grid meter at ~0 W:
  - Solar surplus detected → charge battery (absorb surplus)
  - Solar deficit detected → discharge battery (cover demand)
  - No solar surplus       → NEVER charge from grid

Key concepts:

  solar_surplus = -last_sf_limit - grid_power
    Estimates how much PV exceeds house load regardless of
    what the SolarFlow is currently doing.

  Operating mode (Schmitt trigger with hysteresis):
    Determines whether we're in CHARGING or DISCHARGING regime.
    Prevents flapping when surplus oscillates near zero.

  Asymmetric targets:
    DISCHARGING: target = +30W (small grid draw OK as safety buffer)
    CHARGING:    target =   0W (absorb all surplus exactly)

  grid_power remains the process variable for the PI controller.

Installation:
  1. Install AppDaemon addon in Home Assistant
  2. Copy this file to config/appdaemon/apps/zero_feed_in.py
  3. Configure apps.yaml (see apps.yaml example file)
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum, auto

import appdaemon.plugins.hass.hassapi as hass


# ═══════════════════════════════════════════════════════════
#  Constants
# ═══════════════════════════════════════════════════════════

DIRECTION_THRESHOLD_W = 10
"""Minimum absolute watts to classify relay state as charge/discharge."""

ROUNDING_STEP_W = 10
"""Zendure accepts limits in multiples of this value."""

EMERGENCY_SAFETY_MARGIN_W = 50
"""Extra watts subtracted during emergency curtailment."""

UNAVAILABLE_STATES = {None, "unknown", "unavailable"}
"""HA entity states indicating a sensor is offline."""

DEFAULT_SENSOR_PREFIX = "sensor.zfi"
"""Prefix for HA entities published by this controller."""

AC_MODE_INPUT = "Input mode"
"""AC mode select value for charging."""

AC_MODE_OUTPUT = "Output mode"
"""AC mode select value for discharging."""


# ═══════════════════════════════════════════════════════════
#  Data types
# ═══════════════════════════════════════════════════════════


class OperatingMode(Enum):
    """High-level regime: are we trying to charge or discharge?

    Selected by Schmitt trigger based on solar surplus.
    Determines which target the PI controller tracks.
    """

    CHARGING = auto()
    DISCHARGING = auto()


class RelayDirection(Enum):
    """Physical relay state of the SolarFlow.

    Used for direction-change lockout to prevent relay chatter.
    Separate from OperatingMode — the relay can be IDLE while
    the mode is still CHARGING (e.g. during lockout or deadband).
    """

    CHARGE = auto()
    IDLE = auto()
    DISCHARGE = auto()


@dataclass
class Config:
    """Typed configuration loaded from apps.yaml."""

    # Sensors (read-only: grid power and battery level)
    grid_sensor: str
    soc_sensor: str

    # Actuators (write-only: commands sent to device)
    output_entity: str
    """number.* entity for setOutputLimit — discharge power (W).
    Only written to, never read. Current limit tracked in ControllerState."""
    input_entity: str
    """number.* entity for setInputLimit — AC charge power (W).
    Only written to, never read."""
    ac_mode_entity: str
    """select.* entity for acMode — must be set to 'Input mode' before
    charging and 'Output mode' before discharging."""

    # Optional switches (input_boolean) to enable/disable directions
    charge_switch: str | None = None
    """input_boolean that enables charging.  When 'off', the controller
    idles instead of charging.  None = charging always allowed."""
    discharge_switch: str | None = None
    """input_boolean that enables discharging.  When 'off', the controller
    idles instead of discharging.  None = discharging always allowed."""

    # Controller tuning
    discharge_target_w: float = 30.0
    charge_target_w: float = 0.0
    kp_up: float = 0.3
    """Proportional gain for increasing output (error > 0, grid drawing).

    Lower than kp_down to ramp up cautiously and avoid overshoot
    into feed-in territory.  The integral will handle steady-state.
    """
    kp_down: float = 0.8
    """Proportional gain for decreasing output (error < 0, feeding in).

    Higher than kp_up to cut power aggressively when we're feeding
    in — battery energy is precious, don't waste it on the grid.
    """
    ki_up: float = 0.03
    """Integral gain for increasing output.  Slower to prevent
    overshoot when ramping up discharge."""
    ki_down: float = 0.08
    """Integral gain for decreasing output.  Faster to quickly
    reduce discharge when grid power drops."""
    deadband_w: float = 25.0
    interval_s: int = 5

    # Power limits
    max_discharge_w: float = 800.0
    max_charge_w: float = 2400.0
    min_soc_pct: float = 10.0
    max_soc_pct: float = 100.0

    # Mode switching (Schmitt trigger)
    mode_hysteresis_w: float = 50.0
    charge_confirm_s: float = 15.0
    """Seconds surplus must stay above hysteresis before switching
    to CHARGING.  Prevents transient feed-in spikes (e.g. when
    a load turns off mid-discharge) from triggering a relay switch.
    Entering DISCHARGING is instant — demand should be covered quickly."""

    # Protection
    max_feed_in_w: float = 800.0
    emergency_kp_mult: float = 4.0
    direction_lockout_s: float = 5.0

    # Debug
    dry_run: bool = True
    sensor_prefix: str = DEFAULT_SENSOR_PREFIX

    @classmethod
    def from_args(cls, args: dict) -> Config:
        return cls(
            grid_sensor=args["grid_power_sensor"],
            soc_sensor=args["solarflow_soc_sensor"],
            output_entity=args["output_limit_entity"],
            input_entity=args["input_limit_entity"],
            ac_mode_entity=args["ac_mode_entity"],
            charge_switch=args.get("charge_switch"),
            discharge_switch=args.get("discharge_switch"),
            discharge_target_w=float(
                args.get("target_grid_power", cls.discharge_target_w)
            ),
            charge_target_w=float(
                args.get("charge_target_power", cls.charge_target_w)
            ),
            kp_up=float(args.get("kp_up", args.get("kp", cls.kp_up))),
            kp_down=float(args.get("kp_down", args.get("kp", cls.kp_down))),
            ki_up=float(args.get("ki_up", args.get("ki", cls.ki_up))),
            ki_down=float(args.get("ki_down", args.get("ki", cls.ki_down))),
            deadband_w=float(args.get("deadband", cls.deadband_w)),
            interval_s=int(args.get("interval", cls.interval_s)),
            max_discharge_w=float(args.get("max_output", cls.max_discharge_w)),
            max_charge_w=float(args.get("max_charge", cls.max_charge_w)),
            min_soc_pct=float(args.get("min_soc", cls.min_soc_pct)),
            max_soc_pct=float(args.get("max_soc", cls.max_soc_pct)),
            mode_hysteresis_w=float(
                args.get("mode_hysteresis", cls.mode_hysteresis_w)
            ),
            charge_confirm_s=float(
                args.get("charge_confirm", cls.charge_confirm_s)
            ),
            max_feed_in_w=float(args.get("max_feed_in", cls.max_feed_in_w)),
            emergency_kp_mult=float(
                args.get("emergency_kp_multiplier", cls.emergency_kp_mult)
            ),
            direction_lockout_s=float(
                args.get("direction_lockout", cls.direction_lockout_s)
            ),
            dry_run=bool(args.get("dry_run", cls.dry_run)),
            sensor_prefix=args.get("sensor_prefix", cls.sensor_prefix),
        )


@dataclass
class SensorReading:
    """Snapshot of all sensor values for one control cycle.

    Only contains values read from external sensors.
    The current limit comes from ControllerState (last_computed_w
    for PI, last_sent_w for surplus estimation).
    """

    grid_power_w: float  # positive = grid draw, negative = feed-in
    soc_pct: float


@dataclass
class ControlOutput:
    """Result of one control cycle."""

    discharge_limit_w: int  # power to feed into house, >= 0
    charge_limit_w: int  # power to charge from AC, >= 0
    raw_limit_w: float  # internal signed value (+ discharge, - charge)
    p_term: float
    i_term: float
    reason: str

    @staticmethod
    def from_raw(raw: float, p_term: float, i_term: float, reason: str):
        return ControlOutput(
            discharge_limit_w=max(0, int(raw)),
            charge_limit_w=max(0, int(-raw)),
            raw_limit_w=raw,
            p_term=p_term,
            i_term=i_term,
            reason=reason,
        )

    @staticmethod
    def idle(p_term: float, i_term: float, reason: str):
        return ControlOutput.from_raw(0.0, p_term, i_term, reason)


@dataclass
class ControllerState:
    """Mutable state persisted across control cycles."""

    # PI state
    integral: float = 0.0
    last_computed_w: float = 0.0
    """What the PI last computed. Used as base for next PI step."""
    last_sent_w: float = 0.0
    """What was actually sent to the device. Used for surplus estimation.
    In dry run this stays at 0 — reflecting that the device does nothing."""
    last_sent_discharge_w: int = -1
    """Last discharge limit actually sent. -1 = never sent (forces first send)."""
    last_sent_charge_w: int = -1
    """Last charge limit actually sent. -1 = never sent (forces first send)."""

    # Operating mode (Schmitt trigger output)
    mode: OperatingMode = OperatingMode.DISCHARGING

    # Relay direction tracking (timestamp only — actual direction
    # is always read from the ac_mode entity)
    last_relay_change_t: float = field(default_factory=time.monotonic)

    # Mode switch confirmation: timestamp when surplus first crossed
    # the charge threshold, or None if not pending
    charge_pending_since: float | None = None


# ═══════════════════════════════════════════════════════════
#  PI Controller (pure computation)
# ═══════════════════════════════════════════════════════════


class PIController:
    """PI controller with asymmetric gains and anti-windup back-calculation."""

    def __init__(
        self,
        kp_up: float,
        kp_down: float,
        ki_up: float,
        ki_down: float,
        output_min: float,
        output_max: float,
    ):
        self.kp_up = kp_up
        self.kp_down = kp_down
        self.ki_up = ki_up
        self.ki_down = ki_down
        self.output_min = output_min
        self.output_max = output_max

    def update(
        self,
        error: float,
        integral: float,
        dt: float,
    ) -> tuple[float, float, float]:
        """
        Compute one PI step with asymmetric gains.

        Selects kp/ki based on error sign:
          error > 0 → grid drawing → ramp up (cautious)
          error < 0 → feeding in   → ramp down (aggressive)

        Anti-windup: if output would be clamped, back-calculate
        the integral so it stays consistent with the clamped output.
        This prevents the integral from growing while saturated.

        Returns:
            (output, p_term, new_integral)
        """
        if error >= 0:
            kp, ki = self.kp_up, self.ki_up
        else:
            kp, ki = self.kp_down, self.ki_down

        p_term = kp * error
        new_integral = integral + ki * error * dt

        # Compute unclamped output
        output = p_term + new_integral

        # Anti-windup back-calculation: if output exceeds limits,
        # adjust integral so output == limit.  This prevents the
        # integral from winding up while the output is saturated.
        if output > self.output_max:
            new_integral = self.output_max - p_term
            output = self.output_max
        elif output < self.output_min:
            new_integral = self.output_min - p_term
            output = self.output_min

        return output, p_term, new_integral


# ═══════════════════════════════════════════════════════════
#  Main AppDaemon App
# ═══════════════════════════════════════════════════════════


class ZeroFeedIn(hass.Hass):
    """Bidirectional zero feed-in controller for Zendure SolarFlow 2400 AC+."""

    def initialize(self):
        self.cfg = Config.from_args(self.args)
        self.state = ControllerState()
        self.pi = PIController(
            kp_up=self.cfg.kp_up,
            kp_down=self.cfg.kp_down,
            ki_up=self.cfg.ki_up,
            ki_down=self.cfg.ki_down,
            output_min=-self.cfg.max_charge_w,
            output_max=self.cfg.max_discharge_w,
        )
        self._last_p = 0.0
        self._last_i = 0.0

        self._seed_state_from_device()
        self.run_every(self._on_tick, "now", self.cfg.interval_s)

        self.log(
            f"Started | mode={self.state.mode.name} "
            f"discharge_target={self.cfg.discharge_target_w}W "
            f"charge_target={self.cfg.charge_target_w}W "
            f"hysteresis={self.cfg.mode_hysteresis_w}W "
            f"Kp={self.cfg.kp_up}/{self.cfg.kp_down} "
            f"Ki={self.cfg.ki_up}/{self.cfg.ki_down} "
            f"dry_run={self.cfg.dry_run}"
        )
        self._check_entities()

    def _seed_state_from_device(self):
        """Read current device state to avoid unnecessary commands on startup.

        Seeds the PI state and relay direction from the device's current
        output/input limits and AC mode.  This prevents the controller
        from resetting the device to 0W on the first tick.
        """
        out_raw = self._read_float(self.cfg.output_entity)
        in_raw = self._read_float(self.cfg.input_entity)

        out_w = int(out_raw) if out_raw is not None else 0
        in_w = int(in_raw) if in_raw is not None else 0

        # Determine signed limit: positive = discharge, negative = charge
        if out_w > 0:
            current_w = float(out_w)
        elif in_w > 0:
            current_w = float(-in_w)
        else:
            current_w = 0.0

        # Seed PI so first deadband tick returns the device's current output
        self.state.last_computed_w = current_w
        self.state.last_sent_w = current_w
        # When the device was in steady state, output ≈ integral
        self.state.integral = current_w

        # Track what the device is already doing to skip redundant sends
        self.state.last_sent_discharge_w = out_w
        self.state.last_sent_charge_w = in_w

        self.log(
            f"Seeded from device: limit={current_w}W "
            f"out={out_w}W in={in_w}W "
            f"ac_mode={self.get_state(self.cfg.ac_mode_entity)}"
        )

    def _check_entities(self):
        """Log the current state of all configured entities at startup."""
        entities = {
            "grid_sensor": self.cfg.grid_sensor,
            "soc_sensor": self.cfg.soc_sensor,
            "output_entity": self.cfg.output_entity,
            "input_entity": self.cfg.input_entity,
            "ac_mode_entity": self.cfg.ac_mode_entity,
        }
        if self.cfg.charge_switch:
            entities["charge_switch"] = self.cfg.charge_switch
        if self.cfg.discharge_switch:
            entities["discharge_switch"] = self.cfg.discharge_switch
        for label, entity_id in entities.items():
            raw = self.get_state(entity_id)
            if raw in UNAVAILABLE_STATES:
                self.log(f"  ✗ {label}: {entity_id} → {raw!r}", level="WARNING")
            else:
                self.log(f"  ✓ {label}: {entity_id} → {raw}")

    # ─── Tick ─────────────────────────────────────────────

    def _on_tick(self, _kwargs):
        reading = self._read_sensors()
        if reading is None:
            return

        surplus = self._estimate_solar_surplus(reading)
        output = self._compute(reading, surplus)
        self._update_state(output)
        self._log_output(output, reading, surplus)
        self._publish_ha_sensors(output, reading, surplus)

        if not self.cfg.dry_run:
            self._send_limits(output)

    # ─── Sensor reading ──────────────────────────────────

    def _read_float(self, entity: str) -> float | None:
        raw = self.get_state(entity)
        if raw in UNAVAILABLE_STATES:
            self.log(f"  {entity} → {raw!r}", level="DEBUG")
            return None
        try:
            return float(raw)
        except (ValueError, TypeError):
            self.log(f"  {entity} → {raw!r} (not a number)", level="WARNING")
            return None

    def _read_sensors(self) -> SensorReading | None:
        grid = self._read_float(self.cfg.grid_sensor)
        soc = self._read_float(self.cfg.soc_sensor)

        unavailable = []
        if grid is None:
            unavailable.append(f"grid ({self.cfg.grid_sensor})")
        if soc is None:
            unavailable.append(f"soc ({self.cfg.soc_sensor})")

        if unavailable:
            self.log(
                f"Sensor unavailable: {', '.join(unavailable)}, skipping cycle",
                level="WARNING",
            )
            return None

        return SensorReading(grid_power_w=grid, soc_pct=soc)

    # ─── Surplus & mode ──────────────────────────────────

    def _estimate_solar_surplus(self, r: SensorReading) -> float:
        """Estimate PV production minus house load.

            surplus = pv - house_load = -sf_signed - grid
            where sf_signed = last_sent (+ discharge, - charge)

        Uses last_sent_w (what the device is actually doing),
        NOT last_computed_w (what the PI calculated). In dry run,
        last_sent_w stays at 0 — correctly reflecting that the
        device isn't doing anything.
        """
        return -self.state.last_sent_w - r.grid_power_w

    def _update_operating_mode(self, surplus: float):
        """Schmitt trigger: update mode with hysteresis to prevent flapping.

                          surplus > +hysteresis (sustained)
        DISCHARGING ─────────────────────────────────▸ CHARGING
                    ◂─────────────────────────────────
                          surplus < -hysteresis (instant)

        Entering CHARGING requires surplus to stay above the threshold
        for charge_confirm_s seconds.  Entering DISCHARGING is instant
        to cover demand quickly.
        """
        h = self.cfg.mode_hysteresis_w
        old_mode = self.state.mode

        if self.state.mode == OperatingMode.DISCHARGING:
            if surplus > h:
                # Surplus above threshold — start or continue confirmation
                now = time.monotonic()
                if self.state.charge_pending_since is None:
                    self.state.charge_pending_since = now
                    self.log(
                        f"Charge pending: surplus={surplus:.0f}W > "
                        f"hysteresis={h:.0f}W, confirming for "
                        f"{self.cfg.charge_confirm_s:.0f}s"
                    )
                elif now - self.state.charge_pending_since >= self.cfg.charge_confirm_s:
                    self.state.mode = OperatingMode.CHARGING
                    self.state.charge_pending_since = None
            else:
                # Surplus dropped below threshold — cancel pending
                if self.state.charge_pending_since is not None:
                    self.log("Charge pending cancelled: surplus dropped")
                    self.state.charge_pending_since = None
        else:
            # CHARGING → DISCHARGING: instant
            if surplus < -h:
                self.state.mode = OperatingMode.DISCHARGING

        if self.state.mode != old_mode:
            self.state.integral = 0.0
            self.state.last_computed_w = 0.0
            self.state.charge_pending_since = None
            self.log(
                f"Mode {old_mode.name} → {self.state.mode.name}: "
                f"integral and last_computed reset"
            )

    def _target_for_mode(self) -> float:
        """Select PI target based on current operating mode."""
        if self.state.mode == OperatingMode.CHARGING:
            return self.cfg.charge_target_w
        return self.cfg.discharge_target_w

    # ─── Computation pipeline ────────────────────────────

    def _compute(self, r: SensorReading, surplus: float) -> ControlOutput:
        emergency = self._check_emergency(r)
        if emergency is not None:
            return emergency

        self._update_operating_mode(surplus)
        target = self._target_for_mode()

        error = r.grid_power_w - target
        raw_limit = self._run_pi(error)

        guarded = self._apply_guards(raw_limit, r, surplus)
        if guarded is not None:
            return guarded

        locked = self._check_relay_lockout(raw_limit)
        if locked is not None:
            return locked

        clamped = self._clamp_and_round(raw_limit, surplus)
        reason = f"{'Charge' if clamped < 0 else 'Discharge'} ({self.state.mode.name})"
        return ControlOutput.from_raw(clamped, self._last_p, self._last_i, reason)

    def _check_emergency(self, r: SensorReading) -> ControlOutput | None:
        """Aggressive curtailment when feed-in exceeds legal limit."""
        feed_in = max(0.0, -r.grid_power_w)
        if feed_in <= self.cfg.max_feed_in_w:
            return None

        excess = feed_in - self.cfg.max_feed_in_w
        forced = max(0.0, self.state.last_sent_w - excess - EMERGENCY_SAFETY_MARGIN_W)
        forced = self._round_to_step(forced)
        self.state.integral = 0.0
        return ControlOutput.from_raw(forced, 0.0, 0.0, "EMERGENCY")

    def _run_pi(self, error: float) -> float:
        """Execute PI step in position form. Freezes in deadband.

        Position form: output = Kp * error + integral.
        NOT incremental — the proportional term responds to the
        current error directly, preventing accumulation during
        device lag.
        """
        if abs(error) <= self.cfg.deadband_w:
            self._last_p = 0.0
            self._last_i = self.state.integral
            return self.state.last_computed_w

        output, self._last_p, self._last_i = self.pi.update(
            error=error,
            integral=self.state.integral,
            dt=self.cfg.interval_s,
        )
        self.state.integral = self._last_i
        return output

    def _apply_guards(
        self, raw_limit: float, r: SensorReading, surplus: float
    ) -> ControlOutput | None:
        """Check switches, SOC limits, and grid-charge protection."""
        # Direction switches (input_boolean)
        if raw_limit > 0 and not self._is_switch_on(self.cfg.discharge_switch):
            return ControlOutput.idle(
                self._last_p, self._last_i, "Discharge disabled"
            )
        if raw_limit < 0 and not self._is_switch_on(self.cfg.charge_switch):
            return ControlOutput.idle(
                self._last_p, self._last_i, "Charge disabled"
            )

        # SOC limits
        if raw_limit > 0 and r.soc_pct <= self.cfg.min_soc_pct:
            return ControlOutput.idle(self._last_p, self._last_i, "SOC too low")

        if raw_limit < 0:
            if surplus <= 0:
                return ControlOutput.idle(
                    self._last_p, self._last_i, "No surplus, charge blocked"
                )
            if r.soc_pct >= self.cfg.max_soc_pct:
                return ControlOutput.idle(self._last_p, self._last_i, "SOC full")

        return None

    def _is_switch_on(self, entity: str | None) -> bool:
        """Check if a switch entity is on.  None means always on."""
        if entity is None:
            return True
        return self.get_state(entity) == "on"

    def _check_relay_lockout(self, raw_limit: float) -> ControlOutput | None:
        """Enforce minimum time between relay direction changes.

        Reads the actual AC mode from the entity to determine
        the current relay direction.  While locked, the integral
        is frozen to prevent windup during the slow AC mode transition.
        """
        desired = self._classify_relay(raw_limit)
        current = self._read_current_relay()

        if desired == current or desired == RelayDirection.IDLE:
            return None

        elapsed = time.monotonic() - self.state.last_relay_change_t
        if elapsed >= self.cfg.direction_lockout_s:
            return None

        # Freeze integral during lockout — the device is switching
        # AC mode and can't respond, so accumulating error is harmful.
        self.state.integral = self._last_i

        return ControlOutput.from_raw(
            self.state.last_computed_w,
            self._last_p,
            self._last_i,
            f"Relay locked ({elapsed:.0f}/{self.cfg.direction_lockout_s:.0f}s)",
        )

    # ─── Clamping ────────────────────────────────────────

    def _clamp_and_round(self, raw: float, surplus: float) -> float:
        """Clamp to safe range and round to device step size.

        For charging: caps at estimated solar surplus so the
        device never draws from the grid to charge.
        """
        if raw > 0:
            clamped = min(raw, self.cfg.max_discharge_w)
        elif raw < 0:
            max_safe_charge = max(0.0, surplus)
            clamped = max(raw, -self.cfg.max_charge_w, -max_safe_charge)
        else:
            clamped = 0.0
        return self._round_to_step(clamped)

    @staticmethod
    def _round_to_step(value: float) -> float:
        return round(value / ROUNDING_STEP_W) * ROUNDING_STEP_W

    @staticmethod
    def _classify_relay(limit: float) -> RelayDirection:
        """Classify a signed watt value into a relay direction."""
        if limit > DIRECTION_THRESHOLD_W:
            return RelayDirection.DISCHARGE
        if limit < -DIRECTION_THRESHOLD_W:
            return RelayDirection.CHARGE
        return RelayDirection.IDLE

    def _read_current_relay(self) -> RelayDirection:
        """Derive current relay direction from the AC mode entity."""
        ac_mode = self.get_state(self.cfg.ac_mode_entity)
        if ac_mode == AC_MODE_OUTPUT:
            return RelayDirection.DISCHARGE
        if ac_mode == AC_MODE_INPUT:
            return RelayDirection.CHARGE
        return RelayDirection.IDLE

    # ─── State update ────────────────────────────────────

    def _update_state(self, output: ControlOutput):
        self.state.last_computed_w = output.raw_limit_w

    # ─── Output ──────────────────────────────────────────

    def _log_output(self, output: ControlOutput, reading: SensorReading, surplus: float):
        mode = "DRY" if self.cfg.dry_run else "LIVE"
        if output.raw_limit_w >= 0:
            power_str = f"out={output.discharge_limit_w}W"
        else:
            power_str = f"in={output.charge_limit_w}W"

        self.log(
            f"{mode} | grid={reading.grid_power_w:.0f}W "
            f"surplus={surplus:.0f}W "
            f"soc={reading.soc_pct:.0f}% "
            f"mode={self.state.mode.name} "
            f"{power_str} | "
            f"P={output.p_term:.0f} I={output.i_term:.0f} | "
            f"{output.reason}"
        )

    def _send_limits(self, output: ControlOutput):
        # Set AC mode before sending power limits so the device
        # is in the correct direction before receiving a nonzero value.
        # When output is IDLE (0W), leave AC mode as-is — either
        # direction is fine and switching would cause a needless relay click.
        new_relay = self._classify_relay(output.raw_limit_w)
        if new_relay == RelayDirection.DISCHARGE:
            self._set_ac_mode(AC_MODE_OUTPUT)
        elif new_relay == RelayDirection.CHARGE:
            self._set_ac_mode(AC_MODE_INPUT)

        # Only send power limits when they actually changed to avoid
        # unnecessary device traffic and relay wear.
        if output.discharge_limit_w != self.state.last_sent_discharge_w:
            self.call_service(
                "number/set_value",
                entity_id=self.cfg.output_entity,
                value=output.discharge_limit_w,
            )
            self.state.last_sent_discharge_w = output.discharge_limit_w

        if output.charge_limit_w != self.state.last_sent_charge_w:
            self.call_service(
                "number/set_value",
                entity_id=self.cfg.input_entity,
                value=output.charge_limit_w,
            )
            self.state.last_sent_charge_w = output.charge_limit_w

        self.state.last_sent_w = output.raw_limit_w

    def _set_ac_mode(self, mode: str):
        """Set the SolarFlow AC mode if it differs from current."""
        current = self.get_state(self.cfg.ac_mode_entity)
        if current == mode:
            return
        self.call_service(
            "select/select_option",
            entity_id=self.cfg.ac_mode_entity,
            option=mode,
        )
        self.state.last_relay_change_t = time.monotonic()
        self.log(f"AC mode: {current} → {mode}")

    # ─── HA sensor publishing ────────────────────────────

    def _set_sensor(
        self,
        name: str,
        value,
        unit: str | None = None,
        icon: str | None = None,
    ):
        """Create or update a HA sensor entity via set_state."""
        entity_id = f"{self.cfg.sensor_prefix}_{name}"
        attrs = {"friendly_name": f"ZFI {name.replace('_', ' ').title()}"}
        if unit:
            attrs["unit_of_measurement"] = unit
            attrs["state_class"] = "measurement"
        if icon:
            attrs["icon"] = icon
        try:
            self.set_state(entity_id, state=str(value), attributes=attrs, replace=True)
        except Exception as exc:
            self.log(
                f"set_state failed for {entity_id}: {exc}",
                level="WARNING",
            )

    def _publish_ha_sensors(
        self,
        output: ControlOutput,
        reading: SensorReading,
        surplus: float,
    ):
        """Publish controller internals as HA sensors for dashboards."""

        # Operating regime
        self._set_sensor(
            "mode",
            self.state.mode.name.lower(),
            icon="mdi:swap-vertical",
        )
        self._set_sensor(
            "relay",
            self._read_current_relay().name.lower(),
            icon="mdi:electric-switch",
        )

        # Physical estimates
        self._set_sensor("surplus", round(surplus), "W", "mdi:solar-power")
        self._set_sensor(
            "device_output",
            round(self.state.last_sent_w),
            "W",
            "mdi:transmission-tower-export",
        )

        # PI internals
        self._set_sensor("p_term", round(output.p_term), "W", "mdi:alpha-p-box")
        self._set_sensor("i_term", round(output.i_term), "W", "mdi:alpha-i-box")
        self._set_sensor(
            "integral", round(self.state.integral), "W", "mdi:sigma"
        )
        self._set_sensor(
            "target",
            round(self._target_for_mode()),
            "W",
            "mdi:target",
        )
        self._set_sensor(
            "error",
            round(reading.grid_power_w - self._target_for_mode()),
            "W",
            "mdi:delta",
        )

        # Output
        self._set_sensor(
            "discharge_limit",
            output.discharge_limit_w,
            "W",
            "mdi:battery-arrow-down",
        )
        self._set_sensor(
            "charge_limit",
            output.charge_limit_w,
            "W",
            "mdi:battery-arrow-up",
        )
        self._set_sensor("reason", output.reason, icon="mdi:information-outline")
