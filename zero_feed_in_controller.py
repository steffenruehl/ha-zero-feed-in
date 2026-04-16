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
from dataclasses import dataclass
from enum import Enum, auto

import appdaemon.plugins.hass.hassapi as hass


# ═══════════════════════════════════════════════════════════
#  Constants
# ═══════════════════════════════════════════════════════════

UNAVAILABLE_STATES = {None, "unknown", "unavailable"}
"""HA entity states indicating a sensor is offline."""

DEFAULT_SENSOR_PREFIX = "sensor.zfi"
"""Prefix for HA entities published by this controller."""


# ═══════════════════════════════════════════════════════════
#  Data types
# ═══════════════════════════════════════════════════════════


class OperatingMode(Enum):
    CHARGING = auto()
    DISCHARGING = auto()


@dataclass
class Config:
    """Typed configuration loaded from apps.yaml."""

    # Sensors
    grid_sensor: str
    soc_sensor: str
    battery_power_sensor: str
    """Sensor reporting battery power with sign: +discharge/-charge."""

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
    sensor_prefix: str = DEFAULT_SENSOR_PREFIX

    @classmethod
    def from_args(cls, args: dict) -> Config:
        return cls(
            grid_sensor=args["grid_power_sensor"],
            soc_sensor=args["soc_sensor"],
            battery_power_sensor=args["battery_power_sensor"],
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
            dry_run=bool(args.get("dry_run", cls.dry_run)),
            sensor_prefix=args.get("sensor_prefix", cls.sensor_prefix),
        )


@dataclass
class SensorReading:
    grid_power_w: float
    soc_pct: float
    battery_power_w: float
    """Actual battery power in controller convention: +discharge, -charge."""


@dataclass
class ControlOutput:
    desired_power_w: float
    """Signed desired power: +discharge, -charge."""
    p_term: float
    i_term: float
    reason: str

    @staticmethod
    def from_raw(raw: float, p_term: float, i_term: float, reason: str):
        return ControlOutput(
            desired_power_w=raw,
            p_term=p_term,
            i_term=i_term,
            reason=reason,
        )

    @staticmethod
    def idle(p_term: float, i_term: float, reason: str):
        return ControlOutput.from_raw(0.0, p_term, i_term, reason)


@dataclass
class ControllerState:
    integral: float = 0.0
    last_computed_w: float = 0.0
    mode: OperatingMode = OperatingMode.DISCHARGING
    charge_pending_since: float | None = None


EMERGENCY_SAFETY_MARGIN_W = 50


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
#  Main AppDaemon App
# ═══════════════════════════════════════════════════════════


class ZeroFeedInController(hass.Hass):
    """Device-agnostic zero feed-in PI controller.

    Reads grid power, SOC, and battery power sensors.
    Publishes a signed desired-power value to sensor.zfi_desired_power.
    """

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

        self._seed_state()
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

    def _seed_state(self):
        """Seed PI from current battery power sensor to avoid step on startup."""
        bp = self._read_float(self.cfg.battery_power_sensor)
        if bp is not None:
            current_w = bp
        else:
            current_w = 0.0

        self.state.last_computed_w = current_w
        self.state.integral = current_w

        if current_w < 0:
            self.state.mode = OperatingMode.CHARGING

        self.log(f"Seeded from battery_power: {current_w:.0f}W")

    def _check_entities(self):
        entities = {
            "grid_sensor": self.cfg.grid_sensor,
            "soc_sensor": self.cfg.soc_sensor,
            "battery_power": self.cfg.battery_power_sensor,
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

    # ─── Sensor reading ──────────────────────────────────

    def _read_float(self, entity: str) -> float | None:
        raw = self.get_state(entity)
        if raw in UNAVAILABLE_STATES:
            return None
        try:
            return float(raw)
        except (ValueError, TypeError):
            self.log(f"  {entity} → {raw!r} (not a number)", level="WARNING")
            return None

    def _read_sensors(self) -> SensorReading | None:
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

        # Battery sensor convention: +discharge/-charge
        # Controller convention:     +discharge/-charge (same, no negation)
        return SensorReading(
            grid_power_w=grid,
            soc_pct=soc,
            battery_power_w=bp,
        )

    # ─── Surplus & mode ──────────────────────────────────

    def _estimate_solar_surplus(self, r: SensorReading) -> float:
        """Estimate PV minus house load using actual battery power.

        Energy balance at AC bus:
            PV + battery_power + grid_power = house_load
            surplus = PV - house = -battery_power - grid_power
        """
        return -r.battery_power_w - r.grid_power_w

    def _update_operating_mode(self, surplus: float):
        h = self.cfg.mode_hysteresis_w
        old_mode = self.state.mode

        if self.state.mode == OperatingMode.DISCHARGING:
            if surplus > h:
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
                if self.state.charge_pending_since is not None:
                    self.log("Charge pending cancelled: surplus dropped")
                    self.state.charge_pending_since = None
        else:
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

        clamped = self._clamp(raw_limit, surplus)
        reason = f"{'Charge' if clamped < 0 else 'Discharge'} ({self.state.mode.name})"
        return ControlOutput.from_raw(clamped, self._last_p, self._last_i, reason)

    def _check_emergency(self, r: SensorReading) -> ControlOutput | None:
        feed_in = max(0.0, -r.grid_power_w)
        if feed_in <= self.cfg.max_feed_in_w:
            return None

        excess = feed_in - self.cfg.max_feed_in_w
        forced = max(0.0, self.state.last_computed_w - excess - EMERGENCY_SAFETY_MARGIN_W)
        self.state.integral = 0.0
        return ControlOutput.from_raw(forced, 0.0, 0.0, "EMERGENCY")

    def _run_pi(self, error: float) -> float:
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
        if raw_limit > 0 and not self._is_switch_on(self.cfg.discharge_switch):
            return ControlOutput.idle(
                self._last_p, self._last_i, "Discharge disabled"
            )
        if raw_limit < 0 and not self._is_switch_on(self.cfg.charge_switch):
            return ControlOutput.idle(
                self._last_p, self._last_i, "Charge disabled"
            )

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
        if entity is None:
            return True
        return self.get_state(entity) == "on"

    def _clamp(self, raw: float, surplus: float) -> float:
        if raw > 0:
            clamped = min(raw, self.cfg.max_discharge_w)
        elif raw < 0:
            max_safe_charge = max(0.0, surplus)
            clamped = max(raw, -self.cfg.max_charge_w, -max_safe_charge)
        else:
            clamped = 0.0
        return clamped

    # ─── State update ────────────────────────────────────

    def _update_state(self, output: ControlOutput):
        self.state.last_computed_w = output.desired_power_w

    # ─── Output ──────────────────────────────────────────

    def _log_output(self, output: ControlOutput, reading: SensorReading, surplus: float):
        mode = "DRY" if self.cfg.dry_run else "LIVE"
        power = output.desired_power_w
        if power >= 0:
            power_str = f"discharge={power:.0f}W"
        else:
            power_str = f"charge={-power:.0f}W"

        self.log(
            f"{mode} | grid={reading.grid_power_w:.0f}W "
            f"surplus={surplus:.0f}W "
            f"soc={reading.soc_pct:.0f}% "
            f"mode={self.state.mode.name} "
            f"{power_str} | "
            f"P={output.p_term:.0f} I={output.i_term:.0f} | "
            f"{output.reason}"
        )

    # ─── HA sensor publishing ────────────────────────────

    def _set_sensor(
        self,
        name: str,
        value,
        unit: str | None = None,
        icon: str | None = None,
    ):
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
            self.state.mode.name.lower(),
            icon="mdi:swap-vertical",
        )

        # Physical estimates
        self._set_sensor("surplus", round(surplus), "W", "mdi:solar-power")
        self._set_sensor(
            "battery_power",
            round(reading.battery_power_w),
            "W",
            "mdi:battery-charging",
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
        self._set_sensor("reason", output.reason, icon="mdi:information-outline")
