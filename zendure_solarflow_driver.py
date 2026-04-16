"""
Zendure SolarFlow Driver – Translates desired power to device commands.

Reads sensor.zfi_desired_power (published by the controller) and
maps it to outputLimit, inputLimit, and acMode commands for the
Zendure SolarFlow 2400 AC+.

Handles:
  - AC mode switching (Input mode / Output mode)
  - Relay direction lockout (configurable, prevents relay chatter)
  - Power rounding to device step size (10 W)
  - Redundant-send suppression
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from enum import Enum, auto

import appdaemon.plugins.hass.hassapi as hass


# ═══════════════════════════════════════════════════════════
#  Constants
# ═══════════════════════════════════════════════════════════

DIRECTION_THRESHOLD_W = 10
ROUNDING_STEP_W = 10
UNAVAILABLE_STATES = {None, "unknown", "unavailable"}

AC_MODE_INPUT = "Input mode"
AC_MODE_OUTPUT = "Output mode"
AC_MODE_RETRY_S = 30
"""Re-send AC mode command if device hasn't confirmed after this many seconds."""



class RelayDirection(Enum):
    CHARGE = auto()
    IDLE = auto()
    DISCHARGE = auto()


@dataclass
class Config:
    desired_power_sensor: str
    output_entity: str
    input_entity: str
    ac_mode_entity: str
    interval_s: int = 2
    direction_lockout_s: float = 5.0
    relay_filter_enabled: bool = True
    dry_run: bool = True
    sensor_prefix: str = "sensor.zfi"

    @classmethod
    def from_args(cls, args: dict) -> Config:
        return cls(
            desired_power_sensor=args["desired_power_sensor"],
            output_entity=args["output_limit_entity"],
            input_entity=args["input_limit_entity"],
            ac_mode_entity=args["ac_mode_entity"],
            interval_s=int(args.get("interval", cls.interval_s)),
            direction_lockout_s=float(
                args.get("direction_lockout", cls.direction_lockout_s)
            ),
            relay_filter_enabled=bool(
                args.get("relay_filter_enabled", cls.relay_filter_enabled)
            ),
            dry_run=bool(args.get("dry_run", cls.dry_run)),
            sensor_prefix=args.get("sensor_prefix", cls.sensor_prefix),
        )


@dataclass
class DriverState:
    last_sent_discharge_w: int = -1
    last_sent_charge_w: int = -1
    last_relay_change_t: float = 0.0
    """Monotonic time of the last AC mode change.  Initialised to 0 so
    the first direction change is never blocked by lockout."""
    last_set_relay: RelayDirection = RelayDirection.IDLE
    """Track what WE last commanded — the HA entity gets overwritten
    by MQTT device reports and cannot be trusted for lockout."""


class ZendureSolarFlowDriver(hass.Hass):
    """Translates signed desired-power into Zendure SolarFlow commands."""

    def initialize(self):
        self.cfg = Config.from_args(self.args)
        self.driver_state = DriverState()

        self._seed_from_device()
        self.run_every(self._on_tick, "now", self.cfg.interval_s)

        self.log(
            f"Started | desired_power={self.cfg.desired_power_sensor} "
            f"relay_filter={self.cfg.relay_filter_enabled} "
            f"lockout={self.cfg.direction_lockout_s}s "
            f"dry_run={self.cfg.dry_run}"
        )

    def _seed_from_device(self):
        """Read current device limits to avoid redundant sends on startup."""
        out_raw = self._read_float(self.cfg.output_entity)
        in_raw = self._read_float(self.cfg.input_entity)
        self.driver_state.last_sent_discharge_w = (
            int(out_raw) if out_raw is not None else -1
        )
        self.driver_state.last_sent_charge_w = (
            int(in_raw) if in_raw is not None else -1
        )

        # Seed relay tracking from actual device state
        ac_mode = self.get_state(self.cfg.ac_mode_entity)
        if ac_mode == AC_MODE_OUTPUT:
            self.driver_state.last_set_relay = RelayDirection.DISCHARGE
        elif ac_mode == AC_MODE_INPUT:
            self.driver_state.last_set_relay = RelayDirection.CHARGE

        self.log(
            f"Seeded: out={self.driver_state.last_sent_discharge_w}W "
            f"in={self.driver_state.last_sent_charge_w}W "
            f"ac_mode={ac_mode} "
            f"relay={self.driver_state.last_set_relay.name}"
        )

    def _read_float(self, entity: str) -> float | None:
        raw = self.get_state(entity)
        if raw in UNAVAILABLE_STATES:
            return None
        try:
            return float(raw)
        except (ValueError, TypeError):
            return None

    # ─── Tick ─────────────────────────────────────────────

    def _on_tick(self, _kwargs):
        desired = self._read_float(self.cfg.desired_power_sensor)
        if desired is None:
            self.log(
                f"desired_power unavailable ({self.cfg.desired_power_sensor}), skipping",
                level="WARNING",
            )
            return

        # Relay lockout: hold previous commands while direction is switching
        if self.cfg.relay_filter_enabled and self._is_relay_locked(desired):
            elapsed = time.monotonic() - self.driver_state.last_relay_change_t
            self.log(
                f"Locked | desired={desired:.0f}W "
                f"relay={self.driver_state.last_set_relay.name} "
                f"({elapsed:.1f}/{self.cfg.direction_lockout_s:.0f}s)"
            )
            return

        # Round to device step
        discharge_w = max(0, self._round_to_step(desired))
        charge_w = max(0, self._round_to_step(-desired))

        mode = "DRY" if self.cfg.dry_run else "LIVE"
        if desired >= 0:
            power_str = f"out={discharge_w}W"
        else:
            power_str = f"in={charge_w}W"
        device_mode = self.get_state(self.cfg.ac_mode_entity)
        self.log(
            f"{mode} | desired={desired:.0f}W {power_str} "
            f"relay={self.driver_state.last_set_relay.name} "
            f"device={device_mode}"
        )

        if not self.cfg.dry_run:
            self._send_limits(desired, discharge_w, charge_w)

        # Publish what the driver actually sent
        self._set_sensor(
            "device_output",
            discharge_w if desired >= 0 else -charge_w,
            "W",
            "mdi:transmission-tower-export",
        )
        self._set_sensor(
            "discharge_limit", discharge_w, "W", "mdi:battery-arrow-down"
        )
        self._set_sensor(
            "charge_limit", charge_w, "W", "mdi:battery-arrow-up"
        )
        self._set_sensor(
            "relay",
            self._read_current_relay().name.lower(),
            icon="mdi:electric-switch",
        )

    # ─── Relay lockout ────────────────────────────────────

    def _is_relay_locked(self, desired: float) -> bool:
        """True if a direction change is needed but still in lockout.

        Uses the driver's own tracking (last_set_relay) instead of the
        HA entity, because the Zendure MQTT integration overwrites the
        entity with the device's current state before the relay has
        physically switched (10-15 s latency).
        """
        desired_dir = self._classify_relay(desired)
        if desired_dir == self.driver_state.last_set_relay or desired_dir == RelayDirection.IDLE:
            return False
        elapsed = time.monotonic() - self.driver_state.last_relay_change_t
        return elapsed < self.cfg.direction_lockout_s

    @staticmethod
    def _classify_relay(limit: float) -> RelayDirection:
        if limit > DIRECTION_THRESHOLD_W:
            return RelayDirection.DISCHARGE
        if limit < -DIRECTION_THRESHOLD_W:
            return RelayDirection.CHARGE
        return RelayDirection.IDLE

    def _read_current_relay(self) -> RelayDirection:
        ac_mode = self.get_state(self.cfg.ac_mode_entity)
        if ac_mode == AC_MODE_OUTPUT:
            return RelayDirection.DISCHARGE
        if ac_mode == AC_MODE_INPUT:
            return RelayDirection.CHARGE
        return RelayDirection.IDLE

    # ─── Send ─────────────────────────────────────────────

    def _send_limits(self, desired: float, discharge_w: int, charge_w: int):
        # Set AC mode before power limits
        new_relay = self._classify_relay(desired)
        if new_relay == RelayDirection.DISCHARGE:
            self._set_ac_mode(AC_MODE_OUTPUT)
        elif new_relay == RelayDirection.CHARGE:
            self._set_ac_mode(AC_MODE_INPUT)

        if discharge_w != self.driver_state.last_sent_discharge_w:
            self.call_service(
                "number/set_value",
                entity_id=self.cfg.output_entity,
                value=discharge_w,
            )
            self.driver_state.last_sent_discharge_w = discharge_w

        if charge_w != self.driver_state.last_sent_charge_w:
            self.call_service(
                "number/set_value",
                entity_id=self.cfg.input_entity,
                value=charge_w,
            )
            self.driver_state.last_sent_charge_w = charge_w

    def _set_ac_mode(self, mode: str):
        """Send AC mode command only on intent change or after retry timeout."""
        new_relay = (
            RelayDirection.CHARGE if mode == AC_MODE_INPUT
            else RelayDirection.DISCHARGE
        )

        # Intent unchanged and within retry window — wait for device
        elapsed = time.monotonic() - self.driver_state.last_relay_change_t
        if new_relay == self.driver_state.last_set_relay and elapsed < AC_MODE_RETRY_S:
            return

        # First send or retry
        self.driver_state.last_set_relay = new_relay
        self.driver_state.last_relay_change_t = time.monotonic()
        self.call_service(
            "select/select_option",
            entity_id=self.cfg.ac_mode_entity,
            option=mode,
        )
        tag = "retry" if elapsed >= AC_MODE_RETRY_S else "set"
        self.log(f"AC mode {tag}: {mode}")

    # ─── Helpers ──────────────────────────────────────────

    @staticmethod
    def _round_to_step(value: float) -> int:
        return int(round(value / ROUNDING_STEP_W) * ROUNDING_STEP_W)

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
