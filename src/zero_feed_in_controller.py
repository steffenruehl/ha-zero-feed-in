"""
Zero Feed-In Controller – Device-Agnostic PI Controller

Bidirectional PI controller that keeps the grid meter at ~0 W.
Publishes a signed desired-power value to a HA sensor entity.
A separate device driver reads that sensor and translates it
into hardware-specific commands.

Output convention (sensor.zfi_desired_power):
    positive = discharge (feed into house)
    negative = charge (absorb surplus into battery)
"""

from __future__ import annotations

import time
from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum, auto
from typing import TYPE_CHECKING

from src.csv_logger import CsvLogger

if TYPE_CHECKING:
    import appdaemon.plugins.hass.hassapi as hass


# ═══════════════════════════════════════════════════════════
#  Constants
# ═══════════════════════════════════════════════════════════

UNAVAILABLE_STATES = {None, "unknown", "unavailable"}
"""HA entity states indicating a sensor is offline."""

DEFAULT_SENSOR_PREFIX = "sensor.zfi"
"""Prefix for HA entities published by this controller."""

CONTROLLER_CSV_COLUMNS = [
    "grid_w", "soc_pct", "battery_power_w", "pv_power_w",
    "surplus_w", "mode", "desired_power_w",
    "p_term", "d_term", "i_term", "ff_pv_term",
    "integral", "target_w", "error_w", "reason",
]
"""Column names for controller CSV log (excluding timestamp)."""


# ═══════════════════════════════════════════════════════════
#  Data types
# ═══════════════════════════════════════════════════════════


class OperatingMode(Enum):
    """Which direction the controller is currently regulating.

    DISCHARGING — house is consuming more than PV produces; battery
        feeds the deficit.  Target is ``discharge_target_w`` (default 30 W).
    CHARGING — PV surplus exceeds house load; excess is absorbed into
        the battery.  Target is ``charge_target_w`` (default 0 W).

    Transitions use a Schmitt trigger with hysteresis to avoid relay
    chatter.  See ``ControlLogic._update_operating_mode``.
    """

    CHARGING = auto()
    DISCHARGING = auto()


@dataclass
class Config:
    """Typed, validated configuration loaded from ``apps.yaml``.

    Covers three domains:

    * **Sensor entity IDs** — grid power, SOC, battery power.
    * **Controller tuning** — PI gains, deadband, targets, limits.
    * **Protection** — max feed-in, emergency multiplier.

    Constructed via the ``from_args`` classmethod which maps the
    free-form ``args`` dict from AppDaemon into typed fields with
    sensible defaults.
    """

    # Sensors
    grid_sensor: str
    """HA entity ID for grid power sensor (W). Positive = importing."""
    soc_sensor: str
    """HA entity ID for battery state-of-charge sensor (%)."""
    battery_power_sensor: str
    """HA entity ID for battery power sensor. Sign: +discharge / -charge."""

    # Battery sensor mode: "signed" or "unsigned"
    #   signed   – sensor already has correct sign (+discharge/-charge)
    #   unsigned – sensor is always positive; sign derived from ac_mode_entity
    battery_sensor_mode: str = "signed"
    ac_mode_entity: str | None = None
    """Required when battery_sensor_mode='unsigned'. AC mode select entity
    whose state ('Input mode'/'Output mode') determines charge/discharge sign."""

    # Optional switches (input_boolean) to enable/disable directions
    charge_switch: str | None = None
    discharge_switch: str | None = None

    # Controller tuning
    discharge_target_w: float = 30.0
    charge_target_w: float = 0.0
    kp_up: float = 0.3
    kp_down: float = 0.8
    ki_up: float = 0.03
    ki_down: float = 0.08
    deadband_w: float = 25.0
    interval_s: int = 5

    # D-term
    kd: float = 0.3
    """Derivative gain on grid power delta."""
    kd_deadband_w: float = 30.0
    """Ignore grid changes smaller than this (noise filter)."""

    # PV feed-forward
    pv_sensor: str = ""
    """HA entity for PV power (W). Empty = disabled."""
    ff_pv_gain: float = 0.6
    """Fraction of PV change applied as correction. 0 = disabled."""
    ff_pv_deadband_w: float = 20.0
    """Ignore PV changes below this (MPPT noise filter)."""

    # Slew rate limiter
    slew_rate_w_per_s: float = 0.0
    """Maximum output change rate (W/s). 0 = disabled. Measure from step response."""

    # Power limits
    max_discharge_w: float = 800.0
    max_charge_w: float = 2400.0
    min_soc_pct: float = 10.0
    max_soc_pct: float = 100.0

    # Mode switching (Schmitt trigger)
    mode_hysteresis_w: float = 50.0
    charge_confirm_s: float = 15.0

    # Protection
    max_feed_in_w: float = 800.0
    emergency_kp_mult: float = 4.0

    # Debug
    dry_run: bool = True
    debug: bool = False
    """If True, publish internal PI sensors (p_term, i_term, integral, etc.)."""
    sensor_prefix: str = DEFAULT_SENSOR_PREFIX

    # File logging
    log_dir: str = ""
    """Directory for CSV log files. Empty = file logging disabled."""

    @classmethod
    def from_args(cls, args: dict[str, object]) -> Config:
        """Create a ``Config`` from the raw ``args`` dict provided by AppDaemon.

        Maps apps.yaml keys (e.g. ``target_grid_power``) to dataclass
        fields (``discharge_target_w``).  Falls back to class-level
        defaults for any omitted key.
        """
        return cls(
            grid_sensor=args["grid_power_sensor"],
            soc_sensor=args["soc_sensor"],
            battery_power_sensor=args["battery_power_sensor"],
            battery_sensor_mode=args.get(
                "battery_sensor_mode", cls.battery_sensor_mode
            ),
            ac_mode_entity=args.get("ac_mode_entity"),
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
            kd=float(args.get("kd", cls.kd)),
            kd_deadband_w=float(args.get("kd_deadband", cls.kd_deadband_w)),
            pv_sensor=args.get("pv_sensor", cls.pv_sensor),
            ff_pv_gain=float(args.get("ff_pv_gain", cls.ff_pv_gain)),
            ff_pv_deadband_w=float(
                args.get("ff_pv_deadband", cls.ff_pv_deadband_w)
            ),
            slew_rate_w_per_s=float(
                args.get("slew_rate", cls.slew_rate_w_per_s)
            ),
            dry_run=bool(args.get("dry_run", cls.dry_run)),
            debug=bool(args.get("debug", cls.debug)),
            sensor_prefix=args.get("sensor_prefix", cls.sensor_prefix),
            log_dir=args.get("log_dir", cls.log_dir),
        )


@dataclass
class Measurement:
    """Snapshot of all external inputs needed for one control cycle.

    Assembled by the HA adapter from sensor readings and switch states.
    Passed into ``ControlLogic.compute()`` as a single immutable bundle
    so the control logic never touches HA directly.

    Power sign convention (project-wide):
        positive = discharge (feed into house)
        negative = charge   (absorb surplus into battery)
    """

    grid_power_w: float
    """Grid power in watts.  Positive = importing from grid."""
    soc_pct: float
    """Battery state of charge in percent (0–100)."""
    battery_power_w: float
    """Actual battery power (W).  +discharge / -charge."""
    pv_power_w: float | None = None
    """PV production (W). Always >= 0. None if sensor not configured."""
    discharge_enabled: bool = True
    """Whether the user's discharge switch (``input_boolean``) is on."""
    charge_enabled: bool = True
    """Whether the user's charge switch (``input_boolean``) is on."""


@dataclass
class ControlOutput:
    """Result of a single control cycle.

    Carries the desired power setpoint together with the PI terms
    and a human-readable reason string for logging / HA sensor
    publishing.
    """

    desired_power_w: float
    """Signed desired power (W).  +discharge / -charge."""
    p_term: float
    """Proportional term of the PI output (W)."""
    d_term: float
    """Derivative term of the PID output (W)."""
    i_term: float
    """Integral term of the PI output (W)."""
    ff_pv_term: float
    """PV feed-forward term (W)."""
    reason: str
    """Human-readable decision reason (e.g. 'EMERGENCY', 'SOC too low')."""

    @staticmethod
    def from_raw(
        raw: float,
        p_term: float,
        d_term: float,
        i_term: float,
        ff_pv_term: float,
        reason: str,
    ) -> ControlOutput:
        """Construct from a raw (possibly clamped) power value."""
        return ControlOutput(
            desired_power_w=raw,
            p_term=p_term,
            d_term=d_term,
            i_term=i_term,
            ff_pv_term=ff_pv_term,
            reason=reason,
        )

    @staticmethod
    def idle(
        p_term: float, d_term: float, i_term: float, ff_pv_term: float, reason: str,
    ) -> ControlOutput:
        """Shorthand for a zero-power (idle) output with a guard reason."""
        return ControlOutput.from_raw(0.0, p_term, d_term, i_term, ff_pv_term, reason)


@dataclass
class ControllerState:
    """Mutable state carried across control cycles by ``ControlLogic``.

    Reset semantics:
        * ``integral`` is reset to 0 on mode transitions.
        * ``last_computed_w`` is set by each cycle's output (not
          explicitly reset on mode change).
        * ``charge_pending_since`` is set when a charge-mode candidate
          is first detected and cleared on confirmation or cancellation.
    """

    integral: float = 0.0
    """PI integral accumulator (W).  Committed only on normal output."""
    last_computed_w: float = 0.0
    """Most recent output power (W).  Used as hold value in deadband."""
    mode: OperatingMode = OperatingMode.DISCHARGING
    """Current operating mode (Schmitt trigger output)."""
    charge_pending_since: float | None = None
    """Monotonic timestamp when charge-mode candidacy started, or None."""
    previous_grid_w: float | None = None
    """Grid power from previous cycle (W). Used for D-term computation."""
    previous_pv_w: float | None = None
    """PV power from previous cycle (W). Used for PV feed-forward."""


EMERGENCY_SAFETY_MARGIN_W = 50


# ═══════════════════════════════════════════════════════════
#  PI Controller (pure computation)
# ═══════════════════════════════════════════════════════════


class PIController:
    """PI controller with asymmetric gains and anti-windup back-calculation.

    Asymmetric gains allow different response speeds for increasing
    vs. decreasing power (e.g. ramp up slowly, ramp down fast).

    Anti-windup uses *back-calculation*: when the output saturates at
    ``output_min`` or ``output_max``, the integral term is
    back-calculated so that ``p_term + integral == saturated_output``.
    This prevents integral build-up during sustained saturation.
    """

    def __init__(
        self,
        kp_up: float,
        kp_down: float,
        ki_up: float,
        ki_down: float,
        output_min: float,
        output_max: float,
    ) -> None:
        self.kp_up: float = kp_up
        self.kp_down: float = kp_down
        self.ki_up: float = ki_up
        self.ki_down: float = ki_down
        self.output_min: float = output_min
        self.output_max: float = output_max

    def update(
        self,
        error: float,
        integral: float,
        dt: float,
    ) -> tuple[float, float, float]:
        """Compute one PI step.

        Args:
            error: Regulation error (W).  Positive = grid importing.
            integral: Current integral accumulator value (W).
            dt: Time since last update (s).

        Returns:
            ``(output, p_term, new_integral)`` — the clamped output
            power, the proportional contribution, and the
            (possibly back-calculated) new integral value.
        """
        if error >= 0:
            kp, ki = self.kp_up, self.ki_up
        else:
            kp, ki = self.kp_down, self.ki_down

        p_term = kp * error
        new_integral = integral + ki * error * dt

        output = p_term + new_integral

        if output > self.output_max:
            new_integral = self.output_max - p_term
            output = self.output_max
        elif output < self.output_min:
            new_integral = self.output_min - p_term
            output = self.output_min

        return output, p_term, new_integral


# ═══════════════════════════════════════════════════════════
#  Control Logic (pure computation, no HA dependencies)
# ═══════════════════════════════════════════════════════════


class ControlLogic:
    """Device/HA-agnostic control logic.  Fully testable without mocks.

    Owns the PI controller instance, the ``ControllerState``, and all
    decision logic (emergency protection, mode switching, guards,
    surplus clamping).  Receives a ``Measurement`` each cycle and
    returns a ``ControlOutput``.

    The optional *log* callback is used for human-readable status
    messages.  If omitted, messages are silently discarded.
    """

    def __init__(self, cfg: Config, log: Callable[[str], None] | None = None) -> None:
        self.cfg = cfg
        self.state = ControllerState()
        self.pi = PIController(
            kp_up=cfg.kp_up,
            kp_down=cfg.kp_down,
            ki_up=cfg.ki_up,
            ki_down=cfg.ki_down,
            output_min=-cfg.max_charge_w,
            output_max=cfg.max_discharge_w,
        )
        self._last_p = 0.0
        self._last_d = 0.0
        self._last_i = 0.0
        self._last_ff_pv = 0.0
        self._log = log or (lambda msg: None)

    def seed(self, battery_power_w: float | None) -> None:
        """Seed PI from current battery power to avoid step on startup."""
        current_w = battery_power_w if battery_power_w is not None else 0.0
        self.state.last_computed_w = current_w
        self.state.integral = current_w
        if current_w < 0:
            self.state.mode = OperatingMode.CHARGING
        self._log(f"Seeded from battery_power: {current_w:.0f}W")

    @staticmethod
    def estimate_surplus(m: Measurement) -> float:
        """Estimate PV minus house load.

        Energy balance at AC bus:
            PV + battery_power + grid_power = house_load
            surplus = PV - house = -battery_power - grid_power
        """
        return -m.battery_power_w - m.grid_power_w

    def compute(self, m: Measurement, now: float | None = None) -> ControlOutput:
        """Run one full control cycle and update internal state.

        Pipeline::

            emergency check → mode update → PID + FF → slew → guards → clamp

        Args:
            m: Current sensor snapshot.
            now: Monotonic timestamp (seconds).  Defaults to
                ``time.monotonic()`` when omitted.

        Returns:
            A ``ControlOutput`` with the desired power, PID/FF terms,
            and the decision reason.
        """
        if now is None:
            now = time.monotonic()

        surplus = self.estimate_surplus(m)

        emergency = self._check_emergency(m)
        if emergency is not None:
            self.state.last_computed_w = emergency.desired_power_w
            self._update_previous(m)
            return emergency

        self._update_operating_mode(surplus, now)
        target = self.target_for_mode()

        error = m.grid_power_w - target
        pid_out, p_term, d_term, new_integral = self._run_pid(error, m.grid_power_w)
        self._last_p = p_term
        self._last_d = d_term

        ff_pv = self._compute_pv_feed_forward(m.pv_power_w)
        self._last_ff_pv = ff_pv

        combined = pid_out + ff_pv
        slew_limited = self._apply_slew_limit(combined)

        guarded = self._apply_guards(slew_limited, m, surplus)
        if guarded is not None:
            # Don't commit integral — freeze PI while guard blocks output
            self._last_i = self.state.integral
            self.state.last_computed_w = guarded.desired_power_w
            self._update_previous(m)
            return guarded

        # Commit PI state
        self.state.integral = new_integral
        self._last_i = new_integral

        clamped = self._clamp(slew_limited, surplus)

        # Anti-windup: back-calculate integral when surplus/power clamp active
        if clamped != slew_limited:
            self.state.integral = clamped - p_term
            self._last_i = self.state.integral

        reason = f"{'Charge' if clamped < 0 else 'Discharge'} ({self.state.mode.name})"
        output = ControlOutput.from_raw(
            clamped, self._last_p, self._last_d, self._last_i,
            self._last_ff_pv, reason,
        )
        self.state.last_computed_w = output.desired_power_w
        self._update_previous(m)
        return output

    def _update_previous(self, m: Measurement) -> None:
        """Update previous-cycle state for D-term and PV feed-forward."""
        self.state.previous_grid_w = m.grid_power_w
        self.state.previous_pv_w = m.pv_power_w

    def _check_emergency(self, m: Measurement) -> ControlOutput | None:
        """Detect excessive grid feed-in and force a reduced setpoint.

        If instantaneous feed-in exceeds ``max_feed_in_w``, the output
        is slashed by the excess plus a safety margin.  The integral
        is back-calculated so that the PI resumes smoothly.

        Returns:
            An ``EMERGENCY`` ``ControlOutput`` if triggered, else ``None``.
        """
        feed_in = max(0.0, -m.grid_power_w)
        if feed_in <= self.cfg.max_feed_in_w:
            return None

        excess = feed_in - self.cfg.max_feed_in_w
        forced = max(0.0, self.state.last_computed_w - excess - EMERGENCY_SAFETY_MARGIN_W)
        # Back-calculate integral so PI resumes smoothly (p_term = 0)
        self.state.integral = forced
        return ControlOutput.from_raw(forced, 0.0, 0.0, forced, 0.0, "EMERGENCY")

    def _run_pid(
        self, error: float, grid_power: float,
    ) -> tuple[float, float, float, float]:
        """Compute PID output without committing state.

        If the error is within the deadband, the PI is frozen but the
        D-term can still fire on large grid deltas (load steps).

        Returns:
            ``(output, p_term, d_term, new_integral)`` — caller decides
            whether to commit ``new_integral`` to ``self.state``.
        """
        d_term = self._compute_d_term(grid_power)

        if abs(error) <= self.cfg.deadband_w:
            # Even in deadband, D can fire (large load step)
            if d_term == 0:
                return self.state.last_computed_w, 0.0, 0.0, self.state.integral
            # D-only correction while PI is frozen
            return (
                self.state.last_computed_w + d_term,
                0.0,
                d_term,
                self.state.integral,
            )

        pi_output, p_term, new_integral = self.pi.update(
            error=error,
            integral=self.state.integral,
            dt=self.cfg.interval_s,
        )
        return pi_output + d_term, p_term, d_term, new_integral

    def _compute_d_term(self, grid_power: float) -> float:
        """Compute the derivative term from grid power change.

        Reacts to rate-of-change of the grid signal — load steps
        produce a large instantaneous delta that D catches immediately.
        """
        prev = self.state.previous_grid_w
        if prev is None:
            return 0.0

        delta = grid_power - prev
        if abs(delta) < self.cfg.kd_deadband_w:
            return 0.0

        return self.cfg.kd * delta

    def _compute_pv_feed_forward(self, pv_power: float | None) -> float:
        """True feed-forward from PV production changes.

        PV drops → positive term (need more discharge / less charge).
        PV rises → negative term (can charge more / discharge less).
        """
        if pv_power is None:
            return 0.0

        prev = self.state.previous_pv_w
        if prev is None:
            return 0.0

        delta = pv_power - prev
        if abs(delta) < self.cfg.ff_pv_deadband_w:
            return 0.0

        return -self.cfg.ff_pv_gain * delta

    def _apply_slew_limit(self, raw: float) -> float:
        """Limit output change rate to match device ramp capability.

        Prevents PI integral windup when commanding steps larger than the
        device can deliver in one cycle.
        """
        if self.cfg.slew_rate_w_per_s <= 0:
            return raw

        max_delta = self.cfg.slew_rate_w_per_s * self.cfg.interval_s
        delta = raw - self.state.last_computed_w

        if abs(delta) <= max_delta:
            return raw

        sign = 1.0 if delta > 0 else -1.0
        return self.state.last_computed_w + sign * max_delta

    def _update_operating_mode(self, surplus: float, now: float) -> None:
        """Schmitt trigger with charge-confirmation delay.

        DISCHARGING → CHARGING:
            Requires surplus to stay above ``+hysteresis`` for at least
            ``charge_confirm_s`` consecutive seconds.
        CHARGING → DISCHARGING:
            Instant when surplus drops below ``-hysteresis``.

        On every mode transition the integral accumulator is reset to
        zero so the PI starts fresh in the new regime.
        """
        h = self.cfg.mode_hysteresis_w
        old_mode = self.state.mode

        if self.state.mode == OperatingMode.DISCHARGING:
            if surplus > h:
                if self.state.charge_pending_since is None:
                    self.state.charge_pending_since = now
                    self._log(
                        f"Charge pending: surplus={surplus:.0f}W > "
                        f"hysteresis={h:.0f}W, confirming for "
                        f"{self.cfg.charge_confirm_s:.0f}s"
                    )
                elif now - self.state.charge_pending_since >= self.cfg.charge_confirm_s:
                    self.state.mode = OperatingMode.CHARGING
                    self.state.charge_pending_since = None
            else:
                if self.state.charge_pending_since is not None:
                    self._log("Charge pending cancelled: surplus dropped")
                    self.state.charge_pending_since = None
        else:
            if surplus < -h:
                self.state.mode = OperatingMode.DISCHARGING

        if self.state.mode != old_mode:
            self.state.charge_pending_since = None
            old_integral = self.state.integral
            self.state.integral = 0.0
            self._log(
                f"Mode {old_mode.name} → {self.state.mode.name} "
                f"(integral reset: {old_integral:.0f} → 0)"
            )

    def target_for_mode(self) -> float:
        """Return the grid-power target (W) for the current operating mode.

        DISCHARGING: ``discharge_target_w`` (default 30 W — small grid draw
        as safety buffer).
        CHARGING: ``charge_target_w`` (default 0 W — absorb all surplus).
        """
        if self.state.mode == OperatingMode.CHARGING:
            return self.cfg.charge_target_w
        return self.cfg.discharge_target_w

    def _apply_guards(
        self, raw_limit: float, m: Measurement, surplus: float
    ) -> ControlOutput | None:
        """Check safety conditions that override PI output.

        Guards are evaluated *before* the integral is committed, so the
        PI state freezes while any guard blocks the output.

        Returns:
            An idle ``ControlOutput`` if a guard fires, else ``None``.
        """
        if raw_limit > 0 and not m.discharge_enabled:
            return ControlOutput.idle(
                self._last_p, self._last_d, self._last_i,
                self._last_ff_pv, "Discharge disabled",
            )
        if raw_limit < 0 and not m.charge_enabled:
            return ControlOutput.idle(
                self._last_p, self._last_d, self._last_i,
                self._last_ff_pv, "Charge disabled",
            )

        if raw_limit > 0 and m.soc_pct <= self.cfg.min_soc_pct:
            return ControlOutput.idle(
                self._last_p, self._last_d, self._last_i,
                self._last_ff_pv, "SOC too low",
            )

        if raw_limit < 0:
            if surplus <= 0:
                return ControlOutput.idle(
                    self._last_p, self._last_d, self._last_i,
                    self._last_ff_pv, "No surplus, charge blocked",
                )
            if m.soc_pct >= self.cfg.max_soc_pct:
                return ControlOutput.idle(
                    self._last_p, self._last_d, self._last_i,
                    self._last_ff_pv, "SOC full",
                )

        return None

    def _clamp(self, raw: float, surplus: float) -> float:
        """Enforce hard power limits and surplus cap.

        * Discharge is capped at ``max_discharge_w``.
        * Charge is capped at both ``max_charge_w`` and the available
          solar surplus (whichever is smaller), preventing the battery
          from pulling power from the grid.

        Returns:
            Clamped power value (W).
        """
        if raw > 0:
            return min(raw, self.cfg.max_discharge_w)
        elif raw < 0:
            max_safe_charge = max(0.0, surplus)
            return max(raw, -self.cfg.max_charge_w, -max_safe_charge)
        return 0.0


# ═══════════════════════════════════════════════════════════
#  Main AppDaemon App (thin HA adapter)
# ═══════════════════════════════════════════════════════════

try:
    import appdaemon.plugins.hass.hassapi as hass

    _HASS_BASE = hass.Hass
except ImportError:
    _HASS_BASE = object  # type: ignore[assignment,misc]


class ZeroFeedInController(_HASS_BASE):
    """Thin Home Assistant / AppDaemon adapter.

    Responsibilities:

    * Read HA sensor entities and assemble a ``Measurement``.
    * Delegate computation to ``ControlLogic``.
    * Publish results back to HA as ``sensor.zfi_*`` entities.

    All domain logic lives in ``ControlLogic`` — this class only
    bridges the gap between AppDaemon callbacks and pure Python.
    """

    cfg: Config
    logic: ControlLogic

    def initialize(self) -> None:
        self.cfg = Config.from_args(self.args)
        self.logic = ControlLogic(self.cfg, log=self.log)

        # CSV file logger (disabled when log_dir is empty)
        self._csv: CsvLogger | None = None
        if self.cfg.log_dir:
            self._csv = CsvLogger(
                self.cfg.log_dir,
                "zfi_controller",
                CONTROLLER_CSV_COLUMNS,
            )
            self.log(f"CSV logging to {self.cfg.log_dir}/zfi_controller_*.csv")

        bp = self._read_float(self.cfg.battery_power_sensor)
        self.logic.seed(bp)

        self.run_every(self._on_tick, "now", self.cfg.interval_s)

        self.log(
            f"Started | mode={self.logic.state.mode.name} "
            f"battery_sensor_mode={self.cfg.battery_sensor_mode} "
            f"discharge_target={self.cfg.discharge_target_w}W "
            f"charge_target={self.cfg.charge_target_w}W "
            f"hysteresis={self.cfg.mode_hysteresis_w}W "
            f"Kp={self.cfg.kp_up}/{self.cfg.kp_down} "
            f"Ki={self.cfg.ki_up}/{self.cfg.ki_down} "
            f"Kd={self.cfg.kd} "
            f"ff_pv_gain={self.cfg.ff_pv_gain} "
            f"slew_rate={self.cfg.slew_rate_w_per_s} "
            f"dry_run={self.cfg.dry_run}"
        )

        if self.cfg.battery_sensor_mode == "unsigned" and not self.cfg.ac_mode_entity:
            self.log(
                "battery_sensor_mode='unsigned' requires ac_mode_entity",
                level="ERROR",
            )

        self._check_entities()

    def _check_entities(self) -> None:
        """Log availability of every configured HA entity at startup."""
        entities = {
            "grid_sensor": self.cfg.grid_sensor,
            "soc_sensor": self.cfg.soc_sensor,
            "battery_power": self.cfg.battery_power_sensor,
        }
        if self.cfg.ac_mode_entity:
            entities["ac_mode"] = self.cfg.ac_mode_entity
        if self.cfg.charge_switch:
            entities["charge_switch"] = self.cfg.charge_switch
        if self.cfg.discharge_switch:
            entities["discharge_switch"] = self.cfg.discharge_switch
        if self.cfg.pv_sensor:
            entities["pv_sensor"] = self.cfg.pv_sensor
        for label, entity_id in entities.items():
            raw = self.get_state(entity_id)
            if raw in UNAVAILABLE_STATES:
                self.log(f"  ✗ {label}: {entity_id} → {raw!r}", level="WARNING")
            else:
                self.log(f"  ✓ {label}: {entity_id} → {raw}")

    # ─── Tick ─────────────────────────────────────────────

    def _on_tick(self, _kwargs: dict) -> None:
        """Called every ``interval_s`` seconds by AppDaemon's scheduler."""
        m = self._read_measurement()
        if m is None:
            return

        output = self.logic.compute(m)
        surplus = ControlLogic.estimate_surplus(m)
        self._log_output(output, m, surplus)
        self._publish_ha_sensors(output, m, surplus)
        self._log_csv(output, m, surplus)

    # ─── Sensor reading ──────────────────────────────────

    def _read_float(self, entity: str) -> float | None:
        """Read an HA entity's state as a float, returning None on failure."""
        raw: str | None = self.get_state(entity)
        if raw in UNAVAILABLE_STATES:
            return None
        try:
            return float(raw)
        except (ValueError, TypeError):
            self.log(f"  {entity} → {raw!r} (not a number)", level="WARNING")
            return None

    def _read_measurement(self) -> Measurement | None:
        """Assemble a ``Measurement`` from HA sensor states.

        Returns ``None`` (and logs a warning) when any required sensor
        is unavailable, causing the tick to be skipped.
        """
        grid = self._read_float(self.cfg.grid_sensor)
        soc = self._read_float(self.cfg.soc_sensor)
        bp = self._read_float(self.cfg.battery_power_sensor)

        unavailable = []
        if grid is None:
            unavailable.append(f"grid ({self.cfg.grid_sensor})")
        if soc is None:
            unavailable.append(f"soc ({self.cfg.soc_sensor})")
        if bp is None:
            unavailable.append(f"battery_power ({self.cfg.battery_power_sensor})")

        if unavailable:
            self.log(
                f"Sensor unavailable: {', '.join(unavailable)}, skipping cycle",
                level="WARNING",
            )
            return None

        # Derive signed battery power from sensor mode
        if self.cfg.battery_sensor_mode == "unsigned":
            # Unsigned sensor: always positive, sign from AC mode entity
            ac_mode = self.get_state(self.cfg.ac_mode_entity)
            if ac_mode == "Input mode":
                battery_power = -abs(bp)   # charging → negative
            else:
                battery_power = abs(bp)    # discharging → positive
        else:
            # Signed sensor: +discharge/-charge (no transformation)
            battery_power = bp

        # Read optional PV sensor
        pv = self._read_float(self.cfg.pv_sensor) if self.cfg.pv_sensor else None

        return Measurement(
            grid_power_w=grid,
            soc_pct=soc,
            battery_power_w=battery_power,
            pv_power_w=pv,
            discharge_enabled=self._is_switch_on(self.cfg.discharge_switch),
            charge_enabled=self._is_switch_on(self.cfg.charge_switch),
        )

    def _is_switch_on(self, entity: str | None) -> bool:
        """Return True if the switch entity is on, or if no entity is configured."""
        if entity is None:
            return True
        return self.get_state(entity) == "on"

    # ─── Output ──────────────────────────────────────────

    def _log_output(
        self, output: ControlOutput, m: Measurement, surplus: float
    ) -> None:
        """Emit a single-line summary of the control cycle to the AppDaemon log."""
        mode = "DRY" if self.cfg.dry_run else "LIVE"
        power = output.desired_power_w
        if power >= 0:
            power_str = f"discharge={power:.0f}W"
        else:
            power_str = f"charge={-power:.0f}W"

        self.log(
            f"{mode} | grid={m.grid_power_w:.0f}W "
            f"surplus={surplus:.0f}W "
            f"soc={m.soc_pct:.0f}% "
            f"mode={self.logic.state.mode.name} "
            f"{power_str} | "
            f"P={output.p_term:.0f} I={output.i_term:.0f} "
            f"D={output.d_term:.0f} FF={output.ff_pv_term:.0f} | "
            f"{output.reason}"
        )

    def _log_csv(
        self, output: ControlOutput, m: Measurement, surplus: float,
    ) -> None:
        """Append one row to the CSV log file (if file logging is enabled)."""
        if self._csv is None:
            return
        target = self.logic.target_for_mode()
        self._csv.log_row({
            "grid_w": round(m.grid_power_w),
            "soc_pct": round(m.soc_pct, 1),
            "battery_power_w": round(m.battery_power_w),
            "pv_power_w": round(m.pv_power_w) if m.pv_power_w is not None else "",
            "surplus_w": round(surplus),
            "mode": self.logic.state.mode.name,
            "desired_power_w": round(output.desired_power_w),
            "p_term": round(output.p_term),
            "d_term": round(output.d_term),
            "i_term": round(output.i_term),
            "ff_pv_term": round(output.ff_pv_term),
            "integral": round(self.logic.state.integral),
            "target_w": round(target),
            "error_w": round(m.grid_power_w - target),
            "reason": output.reason,
        })

    # ─── HA sensor publishing ────────────────────────────

    def _set_sensor(
        self,
        name: str,
        value: object,
        unit: str | None = None,
        icon: str | None = None,
    ) -> None:
        """Create or update a ``sensor.zfi_<name>`` entity in HA."""
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
        m: Measurement,
        surplus: float,
    ) -> None:
        """Push all ``sensor.zfi_*`` entities to HA for dashboards and the driver."""
        # Desired power (the main output for the driver)
        self._set_sensor(
            "desired_power",
            round(output.desired_power_w),
            "W",
            "mdi:transmission-tower",
        )

        # Operating regime
        self._set_sensor(
            "mode",
            self.logic.state.mode.name.lower(),
            icon="mdi:swap-vertical",
        )

        if not self.cfg.debug:
            return

        # ── Debug-only sensors below ─────────────────────

        # Physical estimates
        self._set_sensor("surplus", round(surplus), "W", "mdi:solar-power")
        self._set_sensor(
            "battery_power",
            round(m.battery_power_w),
            "W",
            "mdi:battery-charging",
        )

        # PI internals
        self._set_sensor("p_term", round(output.p_term), "W", "mdi:alpha-p-box")
        self._set_sensor("i_term", round(output.i_term), "W", "mdi:alpha-i-box")
        self._set_sensor("d_term", round(output.d_term), "W", "mdi:delta")
        self._set_sensor(
            "ff_pv", round(output.ff_pv_term), "W", "mdi:solar-power-variant",
        )
        self._set_sensor(
            "integral", round(self.logic.state.integral), "W", "mdi:sigma"
        )
        self._set_sensor(
            "target",
            round(self.logic.target_for_mode()),
            "W",
            "mdi:target",
        )
        self._set_sensor(
            "error",
            round(m.grid_power_w - self.logic.target_for_mode()),
            "W",
            "mdi:delta",
        )
        self._set_sensor(
            "pv_power",
            round(m.pv_power_w or 0),
            "W",
            "mdi:solar-panel",
        )
        self._set_sensor("reason", output.reason, icon="mdi:information-outline")
