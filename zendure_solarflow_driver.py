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
class AdaptiveLockout:
    """Integrates |power| × dt to decide when relay lockout expires.

    Instead of using instantaneous power to scale a fixed duration,
    each tick contributes ``|power| × dt`` to an accumulator.  The
    lockout expires when the accumulated energy reaches::

        threshold = full_power_w × base_lockout_s

    Examples with full_power_w=200, base=30 s (threshold = 6000 W·s):
        200 W sustained → 30 s
        100 W sustained → 60 s
        50 W sustained  → 120 s
        Variable power  → each slice contributes proportionally

    Capped at ``max_multiplier × base`` elapsed time so the lockout
    stays finite even when power is near zero.
    """

    full_power_w: float = 200.0
    max_multiplier: float = 10.0
    _accumulated_ws: float = 0.0
    _last_tick_t: float = 0.0

    def reset(self):
        """Reset accumulator (call on direction change or relay switch)."""
        self._accumulated_ws = 0.0
        self._last_tick_t = 0.0

    def tick(self, power_w: float, now: float) -> None:
        """Accumulate a time slice of |power| × dt."""
        if self._last_tick_t > 0.0:
            dt = now - self._last_tick_t
            if dt > 0:
                self._accumulated_ws += abs(power_w) * dt
        self._last_tick_t = now

    def is_settled(self, base_s: float) -> bool:
        """True if accumulated energy has reached the threshold."""
        threshold_ws = self.full_power_w * base_s
        return self._accumulated_ws >= threshold_ws

    @property
    def progress(self) -> float:
        """Fraction of threshold reached (0.0 → 1.0+)."""
        return self._accumulated_ws

    def threshold(self, base_s: float) -> float:
        return self.full_power_w * base_s


@dataclass
class Config:
    desired_power_sensor: str
    output_entity: str
    input_entity: str
    ac_mode_entity: str
    interval_s: int = 2
    direction_lockout_s: float = 5.0
    relay_filter_enabled: bool = True
    adaptive_lockout: AdaptiveLockout = None  # type: ignore[assignment]
    dry_run: bool = True
    sensor_prefix: str = "sensor.zfi"

    def __post_init__(self):
        if self.adaptive_lockout is None:
            self.adaptive_lockout = AdaptiveLockout()

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
            adaptive_lockout=AdaptiveLockout(
                full_power_w=float(args.get("adaptive_lockout_ref_w", 200.0)),
                max_multiplier=float(args.get("adaptive_lockout_max_mult", 10.0)),
            ),
            dry_run=bool(args.get("dry_run", cls.dry_run)),
            sensor_prefix=args.get("sensor_prefix", cls.sensor_prefix),
        )


RELAY_SAFETY_TIMEOUT_S = 300.0
"""Never lock the relay for more than this many seconds (failsafe)."""

MIN_ACTIVE_POWER_W = 10
"""Minimum power in CHARGING/DISCHARGING states.  Below this the relay
may switch, so we clamp to this floor while in an active state."""


# ═══════════════════════════════════════════════════════════
#  Relay State Machine
# ═══════════════════════════════════════════════════════════

class RelayState(Enum):
    """Logical relay state (not the physical device state)."""
    IDLE = auto()
    CHARGING = auto()
    DISCHARGING = auto()


class RelayStateMachine:
    """Guards relay transitions so the physical relay doesn't chatter.

    Three states: IDLE (power=0), CHARGING (≥10 W charge),
    DISCHARGING (≥10 W discharge).

    Transition rules:
      - → CHARGING:    adaptive energy integrator must reach threshold
      - → DISCHARGING: adaptive energy integrator must reach threshold
      - → IDLE:        fixed ``idle_lockout_s`` must elapse

    Each transition candidate gets its own AdaptiveLockout / timer.
    If the requested state changes while a transition is pending the
    accumulator resets, so only sustained intent triggers a switch.

    The machine also publishes its state and transition progress to
    HA sensors via a callback.
    """

    def __init__(
        self,
        idle_lockout_s: float,
        charge_lockout: AdaptiveLockout,
        discharge_lockout: AdaptiveLockout,
        base_lockout_s: float,
        log_fn=None,
        publish_fn=None,
    ):
        self.idle_lockout_s = idle_lockout_s
        self.charge_lockout = charge_lockout
        self.discharge_lockout = discharge_lockout
        self.base_lockout_s = base_lockout_s
        self._log = log_fn
        self._publish = publish_fn

        self.state = RelayState.IDLE
        self._pending: RelayState | None = None
        self._idle_pending_since: float = 0.0

    def seed(self, direction: RelayDirection):
        """Set initial state from device without transition guards."""
        if direction == RelayDirection.CHARGE:
            self.state = RelayState.CHARGING
        elif direction == RelayDirection.DISCHARGE:
            self.state = RelayState.DISCHARGING
        else:
            self.state = RelayState.IDLE

    def update(self, desired_w: float, now: float) -> float:
        """Process a desired power value.  Returns the allowed power.

        The returned power respects the current state constraints:
          - IDLE → always 0
          - CHARGING → clamp to [-max, -MIN_ACTIVE_POWER_W]
          - DISCHARGING → clamp to [+MIN_ACTIVE_POWER_W, +max]

        Internally drives transition logic.
        """
        target = self._classify(desired_w)

        if target == self.state:
            # Already in the right state — reset any pending transition
            self._clear_pending()
            return self._clamp(desired_w)

        # A different state is requested — check transition eligibility
        if target != self._pending:
            # New target — start fresh
            self._set_pending(target, now)

        allowed = self._check_transition(target, desired_w, now)
        return allowed

    def publish(self):
        """Push current state and transition progress to HA sensors."""
        if not self._publish:
            return
        self._publish(
            "relay_sm_state",
            self.state.name.lower(),
            icon="mdi:state-machine",
        )
        # Publish transition progress for each possible target
        if self._pending == RelayState.CHARGING:
            al = self.charge_lockout
            pct = min(100, al.progress / max(1, al.threshold(self.base_lockout_s)) * 100)
            self._publish("relay_sm_charge_pct", f"{pct:.0f}", "%", "mdi:battery-charging")
        else:
            self._publish("relay_sm_charge_pct", "0", "%", "mdi:battery-charging")

        if self._pending == RelayState.DISCHARGING:
            al = self.discharge_lockout
            pct = min(100, al.progress / max(1, al.threshold(self.base_lockout_s)) * 100)
            self._publish("relay_sm_discharge_pct", f"{pct:.0f}", "%", "mdi:battery-arrow-down")
        else:
            self._publish("relay_sm_discharge_pct", "0", "%", "mdi:battery-arrow-down")

        if self._pending == RelayState.IDLE:
            elapsed = time.monotonic() - self._idle_pending_since if self._idle_pending_since > 0 else 0.0
            pct = min(100, elapsed / max(1, self.idle_lockout_s) * 100)
            self._publish("relay_sm_idle_pct", f"{pct:.0f}", "%", "mdi:sleep")
        else:
            self._publish("relay_sm_idle_pct", "0", "%", "mdi:sleep")

        self._publish(
            "relay_sm_pending",
            self._pending.name.lower() if self._pending else "none",
            icon="mdi:timer-sand",
        )

    # ── Internal ──────────────────────────────────────────

    def _check_transition(self, target: RelayState, desired_w: float, now: float) -> float:
        """Check if the pending transition can fire.  Returns allowed power."""
        can_switch = False

        if target == RelayState.IDLE:
            elapsed = now - self._idle_pending_since if self._idle_pending_since > 0 else 0.0
            can_switch = elapsed >= self.idle_lockout_s
        elif target == RelayState.CHARGING:
            self.charge_lockout.tick(desired_w, now)
            can_switch = self.charge_lockout.is_settled(self.base_lockout_s)
        elif target == RelayState.DISCHARGING:
            self.discharge_lockout.tick(desired_w, now)
            can_switch = self.discharge_lockout.is_settled(self.base_lockout_s)

        # Safety cap
        if not can_switch and self._pending_elapsed(now) >= RELAY_SAFETY_TIMEOUT_S:
            can_switch = True

        if can_switch:
            old = self.state
            self.state = target
            self._clear_pending()
            if self._log:
                self._log(f"SM transition: {old.name} → {target.name}")
            return self._clamp(desired_w)

        # Not yet — stay in current state
        return self._clamp_current(desired_w)

    def _clamp(self, desired_w: float) -> float:
        """Clamp power for the *current* (just-transitioned) state."""
        return self._clamp_for_state(self.state, desired_w)

    def _clamp_current(self, desired_w: float) -> float:
        """Clamp power to current state while transition is pending."""
        return self._clamp_for_state(self.state, desired_w)

    @staticmethod
    def _clamp_for_state(state: RelayState, desired_w: float) -> float:
        if state == RelayState.IDLE:
            return 0.0
        if state == RelayState.CHARGING:
            # Charging: desired is negative; floor at -MIN_ACTIVE_POWER_W
            return min(-MIN_ACTIVE_POWER_W, desired_w)
        if state == RelayState.DISCHARGING:
            # Discharging: desired is positive; floor at +MIN_ACTIVE_POWER_W
            return max(MIN_ACTIVE_POWER_W, desired_w)
        return 0.0

    @staticmethod
    def _classify(desired_w: float) -> RelayState:
        if desired_w > DIRECTION_THRESHOLD_W:
            return RelayState.DISCHARGING
        if desired_w < -DIRECTION_THRESHOLD_W:
            return RelayState.CHARGING
        return RelayState.IDLE

    def _set_pending(self, target: RelayState, now: float):
        self._pending = target
        self.charge_lockout.reset()
        self.discharge_lockout.reset()
        self._idle_pending_since = now if target == RelayState.IDLE else 0.0

    def _clear_pending(self):
        self._pending = None
        self._idle_pending_since = 0.0

    def _pending_elapsed(self, now: float) -> float:
        if self._pending == RelayState.IDLE and self._idle_pending_since > 0:
            return now - self._idle_pending_since
        # For adaptive lockouts, use last_tick time as proxy
        if self._pending == RelayState.CHARGING and self.charge_lockout._last_tick_t > 0:
            return now - (self.charge_lockout._last_tick_t - (
                self.charge_lockout._accumulated_ws / max(1, self.charge_lockout.full_power_w)
            ))
        if self._pending == RelayState.DISCHARGING and self.discharge_lockout._last_tick_t > 0:
            return now - (self.discharge_lockout._last_tick_t - (
                self.discharge_lockout._accumulated_ws / max(1, self.discharge_lockout.full_power_w)
            ))
        return 0.0


@dataclass
class DriverState:
    last_sent_discharge_w: int = -1
    last_sent_charge_w: int = -1
    last_relay_change_t: float = 0.0
    """Monotonic time of the last AC mode command."""
    last_set_relay: RelayDirection = RelayDirection.IDLE
    """Track what WE last commanded — the HA entity gets overwritten
    by MQTT device reports and cannot be trusted for lockout."""


class ZendureSolarFlowDriver(hass.Hass):
    """Translates signed desired-power into Zendure SolarFlow commands."""

    def initialize(self):
        self.cfg = Config.from_args(self.args)
        self.driver_state = DriverState()

        self.relay_sm = RelayStateMachine(
            idle_lockout_s=self.cfg.direction_lockout_s,
            charge_lockout=AdaptiveLockout(
                full_power_w=self.cfg.adaptive_lockout.full_power_w,
                max_multiplier=self.cfg.adaptive_lockout.max_multiplier,
            ),
            discharge_lockout=AdaptiveLockout(
                full_power_w=self.cfg.adaptive_lockout.full_power_w,
                max_multiplier=self.cfg.adaptive_lockout.max_multiplier,
            ),
            base_lockout_s=self.cfg.direction_lockout_s,
            log_fn=self.log,
            publish_fn=self._set_sensor,
        )

        self._seed_from_device()
        self.run_every(self._on_tick, "now", self.cfg.interval_s)

        self.log(
            f"Started | desired_power={self.cfg.desired_power_sensor} "
            f"relay_filter={self.cfg.relay_filter_enabled} "
            f"lockout={self.cfg.direction_lockout_s}s "
            f"adaptive_ref={self.cfg.adaptive_lockout.full_power_w}W "
            f"adaptive_max={self.cfg.adaptive_lockout.max_multiplier}x "
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

        # Seed state machine from device
        self.relay_sm.seed(self.driver_state.last_set_relay)

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

        now = time.monotonic()

        # State machine decides allowed power
        if self.cfg.relay_filter_enabled:
            allowed = self.relay_sm.update(desired, now)
        else:
            allowed = desired

        if allowed != desired:
            self.log(
                f"SM clamp | wanted={desired:.0f}W allowed={allowed:.0f}W "
                f"state={self.relay_sm.state.name} "
                f"pending={self.relay_sm._pending.name if self.relay_sm._pending else 'none'}"
            )

        # Round to device step
        discharge_w = max(0, self._round_to_step(allowed))
        charge_w = max(0, self._round_to_step(-allowed))

        mode = "DRY" if self.cfg.dry_run else "LIVE"
        if allowed >= 0:
            power_str = f"out={discharge_w}W"
        else:
            power_str = f"in={charge_w}W"
        device_mode = self.get_state(self.cfg.ac_mode_entity)
        self.log(
            f"{mode} | desired={desired:.0f}W allowed={allowed:.0f}W {power_str} "
            f"sm={self.relay_sm.state.name} "
            f"device={device_mode}"
        )

        if not self.cfg.dry_run:
            self._send_limits(allowed, discharge_w, charge_w)

        # Publish what the driver actually sent
        self._set_sensor(
            "device_output",
            discharge_w if allowed >= 0 else -charge_w,
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

        # Publish state machine sensors
        self.relay_sm.publish()

    def _read_current_relay(self) -> RelayDirection:
        ac_mode = self.get_state(self.cfg.ac_mode_entity)
        if ac_mode == AC_MODE_OUTPUT:
            return RelayDirection.DISCHARGE
        if ac_mode == AC_MODE_INPUT:
            return RelayDirection.CHARGE
        return RelayDirection.IDLE

    # ─── Send ─────────────────────────────────────────────

    def _send_limits(self, desired: float, discharge_w: int, charge_w: int):
        # Set AC mode based on state machine state
        sm_state = self.relay_sm.state
        if sm_state == RelayState.DISCHARGING:
            self._set_ac_mode(AC_MODE_OUTPUT)
        elif sm_state == RelayState.CHARGING:
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
