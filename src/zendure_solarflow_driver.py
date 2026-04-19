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
from typing import TYPE_CHECKING

try:
    from src.csv_logger import CsvLogger
except ModuleNotFoundError:
    from csv_logger import CsvLogger  # type: ignore[no-redef]

if TYPE_CHECKING:
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

MIN_ACTIVE_POWER_W = 25
"""Default minimum power in CHARGING/DISCHARGING states (W).  Below this
the relay may switch, so we clamp to this floor while in an active state."""

RELAY_SAFETY_TIMEOUT_S = 600.0
"""Never lock the relay for more than this many seconds (failsafe)."""

RELAY_SWITCH_DELAY_S = 8.0
"""Holdoff after an SM transition: keep relay_locked=true for this long
to let the physical relay finish switching."""



class RelayDirection(Enum):
    """Logical direction the AC relay should be set to.

    The Zendure SolarFlow 2400 AC+ has a physical relay that switches
    between Input mode (charging) and Output mode (discharging).
    IDLE means no power flow is requested.
    """

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
    """

    full_power_w: float = 200.0
    _accumulated_ws: float = 0.0
    _last_tick_t: float = 0.0

    def reset(self) -> None:
        """Reset accumulator (call on direction change or relay switch)."""
        self._accumulated_ws = 0.0
        self._last_tick_t = 0.0

    def pause(self) -> None:
        """Pause accumulation without resetting progress.

        Clears the last-tick timestamp so that when accumulation resumes,
        the elapsed gap is not counted.  Call on accumulators whose target
        is not currently active.
        """
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
        """Accumulated energy (W·s)."""
        return self._accumulated_ws

    def threshold(self, base_s: float) -> float:
        """Energy threshold (W·s) that must be accumulated before the lockout expires."""
        return self.full_power_w * base_s


@dataclass
class Config:
    """Typed configuration for the Zendure SolarFlow driver.

    Loaded from the ``zendure_solarflow_driver`` section of ``apps.yaml``
    via the ``from_args`` classmethod.
    """

    desired_power_sensor: str
    """HA entity ID for the controller's desired-power output."""
    output_entity: str
    """HA number entity for ``outputLimit`` (discharge power, W)."""
    input_entity: str
    """HA number entity for ``inputLimit`` (charge power, W)."""
    ac_mode_entity: str
    """HA select entity for AC mode ('Input mode' / 'Output mode')."""
    interval_s: int = 2
    """Poll interval in seconds (should be faster than the controller)."""
    direction_lockout_s: float = 5.0
    """Base lockout duration before allowing a relay direction change (s).""" 
    relay_sm_enabled: bool = True
    """If False, bypass the relay state machine entirely."""
    adaptive_lockout_ref_w: float = 200.0
    """Reference power for full-speed adaptive lockout (W)."""
    min_active_power_w: float = MIN_ACTIVE_POWER_W
    """Minimum power floor in active relay states (W). Below this the
    device may idle, so the SM clamps to at least this value."""
    dry_run: bool = True
    """If True, compute but do not send commands to the device."""
    debug: bool = False
    """If True, publish relay state machine internals to HA sensors."""
    sensor_prefix: str = "sensor.zfi"
    """Prefix for HA sensor entities published by this driver."""

    # File logging
    log_dir: str = ""
    """Directory for CSV log files. Empty = file logging disabled."""

    @classmethod
    def from_args(cls, args: dict[str, object]) -> Config:
        """Map free-form AppDaemon ``args`` to typed ``Config`` fields."""
        return cls(
            desired_power_sensor=args["desired_power_sensor"],
            output_entity=args["output_limit_entity"],
            input_entity=args["input_limit_entity"],
            ac_mode_entity=args["ac_mode_entity"],
            interval_s=int(args.get("interval", cls.interval_s)),
            direction_lockout_s=float(
                args.get("direction_lockout", cls.direction_lockout_s)
            ),
            relay_sm_enabled=bool(
                args.get("relay_sm_enabled", cls.relay_sm_enabled)
            ),
            adaptive_lockout_ref_w=float(
                args.get("adaptive_lockout_ref_w", cls.adaptive_lockout_ref_w)
            ),
            min_active_power_w=float(
                args.get("min_active_power_w", cls.min_active_power_w)
            ),
            dry_run=bool(args.get("dry_run", cls.dry_run)),
            debug=bool(args.get("debug", cls.debug)),
            sensor_prefix=args.get("sensor_prefix", cls.sensor_prefix),
            log_dir=args.get("log_dir", cls.log_dir),
        )


DRIVER_CSV_COLUMNS = [
    "desired_w", "allowed_w", "discharge_limit_w", "charge_limit_w",
    "sm_state", "sm_target", "device_mode",
]
"""Column names for driver CSV log (excluding timestamp)."""


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

    Three states: IDLE (power=0), CHARGING (≥min_active_power_w charge),
    DISCHARGING (≥min_active_power_w discharge).

    Transition rules:
      - → CHARGING:    adaptive energy integrator must reach threshold
      - → DISCHARGING: adaptive energy integrator must reach threshold
      - → IDLE:        accumulated time wanting IDLE must reach ``idle_lockout_s``

    Each non-current state tracks its own transition progress
    independently.  Switching between two non-current targets does
    **not** reset the other's accumulator, so oscillating desired
    power still makes progress toward whichever transition
    accumulates enough first.

    Accumulators are only reset when:
      - The desired state matches the current state (stable), or
      - An actual state transition fires.

    The machine also publishes its state and transition progress to
    HA sensors via a callback.
    """

    def __init__(
        self,
        idle_lockout_s: float,
        charge_lockout: AdaptiveLockout,
        discharge_lockout: AdaptiveLockout,
        base_lockout_s: float,
        min_active_power_w: float = MIN_ACTIVE_POWER_W,
        log_fn=None,
        publish_fn=None,
    ):
        self.idle_lockout_s = idle_lockout_s
        self.charge_lockout = charge_lockout
        self.discharge_lockout = discharge_lockout
        self.base_lockout_s = base_lockout_s
        self.min_active_power_w = min_active_power_w
        self._log = log_fn
        self._publish = publish_fn

        self.state = RelayState.IDLE
        self._last_target: RelayState | None = None
        """Most recently requested non-current state (for publish display)."""
        self._idle_accumulated_s: float = 0.0
        """Accumulated seconds wanting IDLE (only ticked when target is IDLE)."""
        self._idle_last_tick_t: float = 0.0
        """Timestamp of the last idle tick (0.0 = no previous tick)."""
        self._departure_since: float = 0.0
        """Wall-clock time of first tick wanting to leave current state (for safety timeout)."""

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
          - CHARGING → clamp to [-max, -min_active_power_w]
          - DISCHARGING → clamp to [+min_active_power_w, +max]

        Each non-current state's accumulator is ticked independently;
        switching targets does not reset the other accumulators.
        """
        target = self._classify(desired_w)

        if target == self.state:
            # Stable in current state — reset all transition trackers
            self._reset_all_transitions()
            return self._clamp(desired_w)

        # Track wall-clock departure time for safety timeout
        if self._departure_since == 0.0:
            self._departure_since = now

        self._last_target = target

        # Tick only the target's accumulator; pause others' tick clocks
        self._tick_target(target, desired_w, now)

        # Check if this target's accumulator has settled
        can_switch = self._is_ready(target)

        # Safety cap — never block longer than RELAY_SAFETY_TIMEOUT_S
        if not can_switch and (now - self._departure_since) >= RELAY_SAFETY_TIMEOUT_S:
            can_switch = True

        if can_switch:
            old = self.state
            self.state = target
            self._reset_all_transitions()
            if self._log:
                self._log(f"SM transition: {old.name} → {target.name}")
            return self._clamp(desired_w)

        # Not yet — stay in current state
        return self._clamp(desired_w)

    def publish(self):
        """Push current state and transition progress to HA sensors."""
        if not self._publish:
            return
        self._publish(
            "relay_sm_state",
            self.state.name.lower(),
            icon="mdi:state-machine",
        )
        self._publish(
            "relay_sm_pending",
            self._last_target.name.lower() if self._last_target else "none",
            icon="mdi:timer-sand",
        )

        # Per-direction transition progress (always shown independently)
        al = self.charge_lockout
        threshold = al.threshold(self.base_lockout_s)
        pct = min(100, al.progress / max(1, threshold) * 100)
        self._publish("relay_sm_charge_pct", f"{pct:.0f}", "%", "mdi:battery-charging")

        al = self.discharge_lockout
        threshold = al.threshold(self.base_lockout_s)
        pct = min(100, al.progress / max(1, threshold) * 100)
        self._publish("relay_sm_discharge_pct", f"{pct:.0f}", "%", "mdi:battery-arrow-down")

        pct = min(100, self._idle_accumulated_s / max(1, self.idle_lockout_s) * 100)
        self._publish("relay_sm_idle_pct", f"{pct:.0f}", "%", "mdi:sleep")

        # Unified lockout progress — for the most recently targeted state
        unified_pct, accumulated_ws, threshold_ws = self._lockout_progress()
        self._publish(
            "relay_sm_lockout_pct", f"{unified_pct:.0f}", "%", "mdi:timer-sand",
        )
        self._publish(
            "relay_sm_accumulated_ws",
            f"{accumulated_ws:.0f}",
            "W·s",
            "mdi:sigma",
        )
        self._publish(
            "relay_sm_threshold_ws",
            f"{threshold_ws:.0f}",
            "W·s",
            "mdi:target",
        )

    def _lockout_progress(self) -> tuple[float, float, float]:
        """Return (pct, accumulated, threshold) for the most recently targeted state.

        Returns (0, 0, 0) when no transition is pending.
        """
        if self._last_target is None:
            return 0.0, 0.0, 0.0

        if self._last_target == RelayState.IDLE:
            pct = min(100, self._idle_accumulated_s / max(1, self.idle_lockout_s) * 100)
            return pct, self._idle_accumulated_s, self.idle_lockout_s

        al = (
            self.charge_lockout
            if self._last_target == RelayState.CHARGING
            else self.discharge_lockout
        )
        threshold = al.threshold(self.base_lockout_s)
        pct = min(100, al.progress / max(1, threshold) * 100)
        return pct, al.progress, threshold

    # ── Internal ──────────────────────────────────────────

    def _tick_target(self, target: RelayState, desired_w: float, now: float) -> None:
        """Tick the accumulator for *target* and pause the other two.

        Each non-active accumulator is paused so that an idle gap does not
        inflate its progress when it becomes active again.
        """
        if target == RelayState.IDLE:
            self._tick_idle(now)
            self.charge_lockout.pause()
            self.discharge_lockout.pause()
        elif target == RelayState.CHARGING:
            self.charge_lockout.tick(desired_w, now)
            self.discharge_lockout.pause()
            self._idle_last_tick_t = 0.0
        elif target == RelayState.DISCHARGING:
            self.discharge_lockout.tick(desired_w, now)
            self.charge_lockout.pause()
            self._idle_last_tick_t = 0.0

    def _tick_idle(self, now: float) -> None:
        """Accumulate elapsed time for the IDLE transition."""
        if self._idle_last_tick_t > 0.0:
            dt = now - self._idle_last_tick_t
            if dt > 0:
                self._idle_accumulated_s += dt
        self._idle_last_tick_t = now

    def _is_ready(self, target: RelayState) -> bool:
        """Return True if *target*'s accumulator has reached its threshold."""
        if target == RelayState.IDLE:
            return self._idle_accumulated_s >= self.idle_lockout_s
        if target == RelayState.CHARGING:
            return self.charge_lockout.is_settled(self.base_lockout_s)
        if target == RelayState.DISCHARGING:
            return self.discharge_lockout.is_settled(self.base_lockout_s)
        return False

    def _reset_all_transitions(self) -> None:
        """Reset every transition accumulator (on state change or stable match)."""
        self.charge_lockout.reset()
        self.discharge_lockout.reset()
        self._idle_accumulated_s = 0.0
        self._idle_last_tick_t = 0.0
        self._departure_since = 0.0
        self._last_target = None

    def _clamp(self, desired_w: float) -> float:
        """Clamp power to the current state's constraints."""
        return self._clamp_for_state(self.state, desired_w, self.min_active_power_w)

    @staticmethod
    def _clamp_for_state(state: RelayState, desired_w: float, min_power_w: float = MIN_ACTIVE_POWER_W) -> float:
        """Clamp *desired_w* to the floor for *state*."""
        if state == RelayState.IDLE:
            return 0.0
        if state == RelayState.CHARGING:
            return min(-min_power_w, desired_w)
        if state == RelayState.DISCHARGING:
            return max(min_power_w, desired_w)
        return 0.0

    @staticmethod
    def _classify(desired_w: float) -> RelayState:
        if desired_w > DIRECTION_THRESHOLD_W:
            return RelayState.DISCHARGING
        if desired_w < -DIRECTION_THRESHOLD_W:
            return RelayState.CHARGING
        return RelayState.IDLE




@dataclass
class DriverState:
    """Mutable state tracking what the driver has sent to the device.

    Used for redundant-send suppression and relay intent tracking.
    The HA entity state cannot be trusted because the Zendure MQTT
    integration overwrites it faster than the physical relay switches.
    """

    last_sent_discharge_w: int = -1
    """Last ``outputLimit`` value sent (W), or -1 if none sent yet."""
    last_sent_charge_w: int = -1
    """Last ``inputLimit`` value sent (W), or -1 if none sent yet."""
    last_relay_change_t: float = 0.0
    """Monotonic time of the last AC mode command."""
    last_set_relay: RelayDirection = RelayDirection.IDLE
    """Track what WE last commanded — the HA entity gets overwritten
    by MQTT device reports and cannot be trusted for lockout."""


try:
    import appdaemon.plugins.hass.hassapi as hass

    _HASS_BASE = hass.Hass
except ImportError:
    _HASS_BASE = object  # type: ignore[assignment,misc]


class ZendureSolarFlowDriver(_HASS_BASE):
    """Translates signed desired-power into Zendure SolarFlow commands.

    Polls ``sensor.zfi_desired_power`` (published by the controller)
    every ``interval_s`` seconds and maps it to:

    * ``outputLimit`` — discharge power (W)
    * ``inputLimit`` — charge power (W)
    * ``acMode`` — relay direction ('Input mode' / 'Output mode')

    An optional relay state machine prevents relay chatter by
    gating direction changes behind an adaptive energy integrator.
    """

    cfg: Config
    driver_state: DriverState
    relay_sm: RelayStateMachine

    def initialize(self) -> None:
        self.cfg = Config.from_args(self.args)
        self.driver_state = DriverState()
        self._last_sm_transition_t: float = -RELAY_SWITCH_DELAY_S

        # CSV file logger (disabled when log_dir is empty)
        self._csv: CsvLogger | None = None
        if self.cfg.log_dir:
            self._csv = CsvLogger(
                self.cfg.log_dir,
                "zfi_driver",
                DRIVER_CSV_COLUMNS,
            )
            self.log(f"CSV logging to {self.cfg.log_dir}/zfi_driver_*.csv")

        self.relay_sm = RelayStateMachine(
            idle_lockout_s=self.cfg.direction_lockout_s,
            charge_lockout=AdaptiveLockout(
                full_power_w=self.cfg.adaptive_lockout_ref_w,
            ),
            discharge_lockout=AdaptiveLockout(
                full_power_w=self.cfg.adaptive_lockout_ref_w,
            ),
            base_lockout_s=self.cfg.direction_lockout_s,
            min_active_power_w=self.cfg.min_active_power_w,
            log_fn=self.log,
            publish_fn=self._set_sensor,
        )

        self._seed_from_device()
        self.run_every(self._on_tick, "now", self.cfg.interval_s)

        self.log(
            f"Started | desired_power={self.cfg.desired_power_sensor} "
            f"relay_sm={self.cfg.relay_sm_enabled} "
            f"lockout={self.cfg.direction_lockout_s}s "
            f"adaptive_ref={self.cfg.adaptive_lockout_ref_w}W "
            f"dry_run={self.cfg.dry_run}"
        )

    def _seed_from_device(self) -> None:
        """Read current device limits and AC mode to avoid redundant sends on startup."""
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

    def _on_tick(self, _kwargs: dict) -> None:
        """Called every ``interval_s`` seconds: read desired power, apply SM, send."""
        desired = self._read_float(self.cfg.desired_power_sensor)
        if desired is None:
            self.log(
                f"desired_power unavailable ({self.cfg.desired_power_sensor}), skipping",
                level="WARNING",
            )
            return

        now = time.monotonic()

        # State machine decides allowed power
        if self.cfg.relay_sm_enabled:
            prev_state = self.relay_sm.state
            allowed = self.relay_sm.update(desired, now)
            if self.relay_sm.state != prev_state:
                self._last_sm_transition_t = now
        else:
            allowed = desired

        if allowed != desired:
            self.log(
                f"SM clamp | wanted={desired:.0f}W allowed={allowed:.0f}W "
                f"state={self.relay_sm.state.name} "
                f"target={self.relay_sm._last_target.name if self.relay_sm._last_target else 'none'}"
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
        # Signal to controller that the SM is clamping output or relay
        # is still physically switching after an SM transition.
        sm_clamping = allowed != desired
        relay_switching = (now - self._last_sm_transition_t) < RELAY_SWITCH_DELAY_S
        relay_locked = sm_clamping or relay_switching
        self._set_sensor(
            "relay_locked",
            str(relay_locked).lower(),
            icon="mdi:lock" if relay_locked else "mdi:lock-open",
        )

        # Publish state machine sensors (debug only)
        if self.cfg.debug:
            self.relay_sm.publish()

        # CSV file log
        if self._csv is not None:
            self._csv.log_row({
                "desired_w": round(desired),
                "allowed_w": round(allowed),
                "discharge_limit_w": discharge_w,
                "charge_limit_w": charge_w,
                "sm_state": self.relay_sm.state.name,
                "sm_target": (
                    self.relay_sm._last_target.name
                    if self.relay_sm._last_target else "none"
                ),
                "device_mode": device_mode,
            })

    def _read_current_relay(self) -> RelayDirection:
        """Read the device's current AC mode entity and return as a ``RelayDirection``."""
        ac_mode = self.get_state(self.cfg.ac_mode_entity)
        if ac_mode == AC_MODE_OUTPUT:
            return RelayDirection.DISCHARGE
        if ac_mode == AC_MODE_INPUT:
            return RelayDirection.CHARGE
        return RelayDirection.IDLE

    # ─── Send ─────────────────────────────────────────────

    def _send_limits(self, desired: float, discharge_w: int, charge_w: int) -> None:
        """Send AC mode and power limit commands to the device.

        AC mode is set first (based on the relay state machine state),
        then ``outputLimit`` and ``inputLimit`` are sent only when their
        values have changed from the last-sent value.
        """
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

    def _set_ac_mode(self, mode: str) -> None:
        """Send AC mode command only on intent change or after retry timeout.

        Uses the driver's own intent tracking (``last_set_relay``) rather
        than the HA entity state, because the MQTT integration overwrites
        the entity faster than the physical relay switches (10–15 s).
        """
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
        """Round a power value to the nearest device step (10 W)."""
        return int(round(value / ROUNDING_STEP_W) * ROUNDING_STEP_W)

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
