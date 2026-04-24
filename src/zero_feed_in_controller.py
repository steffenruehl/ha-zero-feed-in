"""
Zero Feed-In Controller – Event-Based Direct Calculation

Bidirectional controller that keeps the grid meter at ~0 W.
Publishes a signed desired-power value to a HA sensor entity.
A separate device driver reads that sensor and translates it
into hardware-specific commands.

Output convention (sensor.zfi_desired_power):
    positive = discharge (feed into house)
    negative = charge (absorb surplus into battery)
"""

from __future__ import annotations

import json
import os
import time
from collections.abc import Callable
from dataclasses import dataclass
from datetime import datetime, timezone
from enum import Enum, auto
from typing import TYPE_CHECKING, Any

try:
    from .csv_logger import CsvLogger
except ImportError:
    from csv_logger import CsvLogger  # type: ignore[no-redef]

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
    "grid_w", "soc_pct", "battery_power_w",
    "surplus_w", "mode", "desired_power_w",
    "error_w", "target_w",
    "drift_acc", "muting",
    "reason",
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

    Covers two domains:

    * **Sensor entity IDs** — grid power, SOC, battery power.
    * **Controller tuning** — ki, hysteresis, muting, targets, limits.

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

    # Optional switches (input_boolean) to enable/disable directions
    charge_switch: str | None = None
    discharge_switch: str | None = None

    # Dynamic min SOC from forecast automation
    dynamic_min_soc_entity: str | None = None
    """``input_number`` entity that carries the forecast-adjusted min SOC (%).

    Read every cycle.  When unavailable, falls back to the static
    ``min_soc_pct`` from apps.yaml.  The dynamic value is clamped to
    ``[min_soc_pct, max_soc_pct]`` so it can never violate hard limits.
    """

    # Controller tuning
    discharge_target_w: float = 30.0
    """Grid power target when discharging (W). Small positive = slight grid draw."""
    charge_target_w: float = 0.0
    """Grid power target when charging (W). Zero = absorb all surplus."""
    ki: float = 1.0
    """Gain applied to error. 1.0 = full correction in one step."""
    hysteresis_w: float = 15.0
    """Suppress commands when |change| <= this (W). Also drift threshold."""
    muting_s: float = 8.0
    """Seconds to ignore sensor updates after sending a command."""

    # Power limits
    max_discharge_w: float = 800.0
    max_charge_w: float = 2400.0
    min_soc_pct: float = 10.0
    max_soc_pct: float = 100.0

    # Mode switching (Schmitt trigger)
    mode_hysteresis_w: float = 50.0
    charge_confirm_s: float = 15.0

    # Debug
    dry_run: bool = True
    debug: bool = False
    """If True, publish internal controller sensors (drift, muting, etc.)."""
    sensor_prefix: str = DEFAULT_SENSOR_PREFIX

    # Heartbeat
    heartbeat_mqtt_topic: str = ""
    """MQTT topic for heartbeat publishing.  When set, the controller publishes
    an ISO-8601 UTC timestamp on every callback for external monitoring
    (e.g. by an ESP fallback controller).  Empty = disabled."""

    # File logging
    log_dir: str = ""
    """Directory for CSV log files. Empty = file logging disabled."""

    @classmethod
    def from_args(cls, args: dict[str, Any]) -> Config:
        """Create a ``Config`` from the raw ``args`` dict provided by AppDaemon.

        Maps apps.yaml keys (e.g. ``target_grid_power``) to dataclass
        fields (``discharge_target_w``).  Falls back to class-level
        defaults for any omitted key.
        """
        return cls(
            grid_sensor=args["grid_power_sensor"],
            soc_sensor=args["soc_sensor"],
            battery_power_sensor=args["battery_power_sensor"],
            charge_switch=args.get("charge_switch"),
            discharge_switch=args.get("discharge_switch"),
            dynamic_min_soc_entity=args.get("dynamic_min_soc_entity"),
            discharge_target_w=float(
                args.get("target_grid_power", cls.discharge_target_w)
            ),
            charge_target_w=float(
                args.get("charge_target_power", cls.charge_target_w)
            ),
            ki=float(args.get("ki", cls.ki)),
            hysteresis_w=float(args.get("hysteresis", cls.hysteresis_w)),
            muting_s=float(args.get("muting", cls.muting_s)),
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
            dry_run=bool(args.get("dry_run", cls.dry_run)),
            debug=bool(args.get("debug", cls.debug)),
            sensor_prefix=args.get("sensor_prefix", cls.sensor_prefix),
            heartbeat_mqtt_topic=args.get(
                "heartbeat_mqtt_topic", cls.heartbeat_mqtt_topic
            ),
            log_dir=args.get("log_dir", cls.log_dir),
        )


@dataclass
class Measurement:
    """Snapshot of all external inputs needed for one control evaluation.

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
    discharge_enabled: bool = True
    """Whether the user's discharge switch (``input_boolean``) is on."""
    charge_enabled: bool = True
    """Whether the user's charge switch (``input_boolean``) is on."""
    dynamic_min_soc_pct: float | None = None
    """Dynamic minimum SOC (%) from ``sensor.zfi_dynamic_min_soc``.

    When set, overrides ``Config.min_soc_pct`` for the SOC-too-low guard.
    Clamped to ``[Config.min_soc_pct, Config.max_soc_pct]`` so the
    dynamic value can never violate the hard safety limits from apps.yaml.
    ``None`` means no dynamic override — use ``Config.min_soc_pct``.
    """


@dataclass
class ControlOutput:
    """Result of a single control evaluation.

    Carries the desired power setpoint together with a human-readable
    reason string for logging / HA sensor publishing.
    """

    desired_power_w: float
    """Signed desired power (W).  +discharge / -charge."""
    reason: str
    """Human-readable decision reason (e.g. 'SOC too low', 'Drift correction')."""

    @staticmethod
    def idle(reason: str) -> ControlOutput:
        """Shorthand for a zero-power (idle) output with a guard reason."""
        return ControlOutput(desired_power_w=0.0, reason=reason)


@dataclass
class ControllerState:
    """Mutable state carried across control evaluations by ``ControlLogic``.

    Reset semantics:
        * ``last_sent_w`` is updated on each send (or seeded at startup).
        * ``drift_acc`` is reset on every send and on guard blocks.
        * ``charge_pending_since`` is set when a charge-mode candidate
          is first detected and cleared on confirmation or cancellation.
    """

    last_sent_w: float = 0.0
    """What was last sent to the device. Base for next calculation."""
    last_command_t: float = 0.0
    """Monotonic timestamp of last command. Muting reference."""
    drift_acc: float = 0.0
    """Accumulated small errors for drift correction (W)."""
    mode: OperatingMode = OperatingMode.DISCHARGING
    """Current operating mode (Schmitt trigger output)."""
    charge_pending_since: float | None = None
    """Monotonic timestamp when charge-mode candidacy started, or None."""


# ═══════════════════════════════════════════════════════════
#  Control Logic (pure computation, no HA dependencies)
# ═══════════════════════════════════════════════════════════


class ControlLogic:
    """Device/HA-agnostic control logic.  Fully testable without mocks.

    Uses direct calculation with muting and drift accumulation.
    Receives a ``Measurement`` each evaluation and
    returns a ``ControlOutput`` or ``None`` (muted / no action needed).

    The optional *log* callback is used for human-readable status
    messages.  If omitted, messages are silently discarded.
    """

    def __init__(self, cfg: Config, log: Callable[[str], None] | None = None) -> None:
        self.cfg = cfg
        self.state = ControllerState()
        self._log = log or (lambda msg: None)

    def seed(self, battery_power_w: float | None) -> None:
        """Seed last_sent from current battery power to avoid step on startup."""
        current_w = battery_power_w if battery_power_w is not None else 0.0
        self.state.last_sent_w = current_w
        if current_w < 0:
            self.state.mode = OperatingMode.CHARGING
        self._log(f"Seeded from battery_power: {current_w:.0f}W")

    def state_snapshot(self) -> dict[str, Any]:
        """Return a dict of state fields suitable for JSON persistence."""
        return {
            "last_sent_w": self.state.last_sent_w,
            "mode": self.state.mode.name,
            "drift_acc": self.state.drift_acc,
        }

    def restore_from_snapshot(self, data: dict[str, Any]) -> bool:
        """Restore internal state from a snapshot dict.

        Returns True on success, False if the data is invalid or
        incomplete.  On failure the state is left unchanged.
        """
        try:
            last_sent_w = float(data["last_sent_w"])
            mode = OperatingMode[data["mode"]]
            drift_acc = float(data.get("drift_acc", 0.0))
        except (KeyError, ValueError, TypeError):
            return False
        self.state.last_sent_w = last_sent_w
        self.state.mode = mode
        self.state.drift_acc = drift_acc
        return True

    @staticmethod
    def estimate_surplus(m: Measurement) -> float:
        """Estimate PV minus house load.

        Energy balance at AC bus:
            PV + battery_power + grid_power = house_load
            surplus = PV - house = -battery_power - grid_power
        """
        return -m.battery_power_w - m.grid_power_w

    def compute(self, m: Measurement, now: float | None = None) -> ControlOutput | None:
        """Evaluate and possibly emit a new setpoint.

        Returns ``None`` if muted or no action needed.

        Pipeline::

            muting check -> mode update -> direct calc -> guards -> clamp

        Args:
            m: Current sensor snapshot.
            now: Monotonic timestamp (seconds).  Defaults to
                ``time.monotonic()`` when omitted.

        Returns:
            A ``ControlOutput`` with the desired power and reason,
            or ``None`` if muted / below hysteresis.
        """
        if now is None:
            now = time.monotonic()

        surplus = self.estimate_surplus(m)

        # -- Muting: wait for device to react --------------
        if now - self.state.last_command_t < self.cfg.muting_s:
            return None

        # -- Surplus & mode --------------------------------
        self._update_operating_mode(surplus, now)
        target = self.target_for_mode()

        # -- Direct calculation ----------------------------
        error = m.grid_power_w - target
        correction = self.cfg.ki * error
        new_limit = self.state.last_sent_w + correction

        # -- Large change: immediate correction ------------
        if abs(correction) > self.cfg.hysteresis_w:
            guarded = self._apply_guards(new_limit, m)
            if guarded is not None:
                self.state.drift_acc = 0.0
                return guarded

            clamped = self._clamp(new_limit, surplus)

            old_sent = self.state.last_sent_w
            if not self.cfg.dry_run:
                self.state.last_sent_w = clamped
                self.state.last_command_t = now
            self.state.drift_acc = 0.0

            direction = "Charge" if clamped < 0 else "Discharge"
            delta = clamped - old_sent
            reason = f"{direction} (direct, Δ={delta:+.0f}W)"
            return ControlOutput(desired_power_w=clamped, reason=reason)

        # -- Small change: drift accumulator ---------------
        if not self.cfg.dry_run:
            self.state.drift_acc += correction

        if abs(self.state.drift_acc) > self.cfg.hysteresis_w:
            drift_limit = self.state.last_sent_w + self.state.drift_acc

            guarded = self._apply_guards(drift_limit, m)
            if guarded is not None:
                self.state.drift_acc = 0.0
                return guarded

            clamped = self._clamp(drift_limit, surplus)

            acc = self.state.drift_acc
            if not self.cfg.dry_run:
                self.state.last_sent_w = clamped
                self.state.last_command_t = now
            self.state.drift_acc = 0.0

            reason = f"Drift correction ({acc:+.0f}W accumulated)"
            return ControlOutput(desired_power_w=clamped, reason=reason)

        # -- Nothing to do ---------------------------------
        return None

    def _update_operating_mode(self, surplus: float, now: float) -> None:
        """Schmitt trigger with charge-confirmation delay.

        DISCHARGING -> CHARGING:
            Requires surplus to stay above ``+hysteresis`` for at least
            ``charge_confirm_s`` consecutive seconds.
        CHARGING -> DISCHARGING:
            Instant when surplus drops below ``-hysteresis``.

        On every mode transition the drift accumulator is reset.
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
            self.state.drift_acc = 0.0
            self._log(
                f"Mode {old_mode.name} -> {self.state.mode.name}"
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
        self, raw_limit: float, m: Measurement,
    ) -> ControlOutput | None:
        """Check safety conditions that override the computed output.

        Guards are evaluated before sending.  If a guard fires, the
        command is suppressed and the drift accumulator is reset.

        Returns:
            An idle ``ControlOutput`` if a guard fires, else ``None``.
        """
        if raw_limit > 0 and not m.discharge_enabled:
            return ControlOutput.idle("Discharge disabled")
        if raw_limit < 0 and not m.charge_enabled:
            return ControlOutput.idle("Charge disabled")

        effective_min_soc = self.cfg.min_soc_pct
        if m.dynamic_min_soc_pct is not None:
            effective_min_soc = max(
                self.cfg.min_soc_pct,
                min(m.dynamic_min_soc_pct, self.cfg.max_soc_pct),
            )
        if raw_limit > 0 and m.soc_pct <= effective_min_soc:
            return ControlOutput.idle("SOC too low")

        if raw_limit < 0:
            if m.soc_pct >= self.cfg.max_soc_pct:
                return ControlOutput.idle("SOC full")

        return None

    def _clamp(self, raw: float, surplus: float) -> float:
        """Enforce hard power limits and surplus cap.

        * Discharge is capped at ``max_discharge_w``.
        * Charge is capped at both ``max_charge_w`` and the measured
          surplus (whichever is smaller), preventing the battery
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

    * React to grid sensor changes (event-driven).
    * Read HA sensor entities and assemble a ``Measurement``.
    * Delegate computation to ``ControlLogic``.
    * Publish results back to HA as ``sensor.zfi_*`` entities.

    All domain logic lives in ``ControlLogic`` — this class only
    bridges the gap between AppDaemon callbacks and pure Python.
    """

    cfg: Config
    logic: ControlLogic

    def initialize(self) -> None:
        """Initialize the controller — called by AppDaemon on startup."""
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

        # State persistence
        _run_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "run")
        os.makedirs(_run_dir, exist_ok=True)
        self._state_file: str = self.args.get(
            "state_file",
            os.path.join(_run_dir, "zfi_controller_state.json"),
        )
        if self._restore_state():
            self.log(
                f"Restored state from {self._state_file}: "
                f"mode={self.logic.state.mode.name} "
                f"last_sent={self.logic.state.last_sent_w:.1f}W"
            )
        else:
            self.log("No saved state found, using battery_power seed")

        # Event-driven: react to grid sensor changes
        self.listen_state(
            self._on_grid_change,
            self.cfg.grid_sensor,
        )

        # Periodic safety check (in case sensor stops updating)
        self.run_every(self._safety_tick, "now+30", 30)

        self.log(
            f"Started | mode={self.logic.state.mode.name} "
            f"discharge_target={self.cfg.discharge_target_w}W "
            f"charge_target={self.cfg.charge_target_w}W "
            f"ki={self.cfg.ki} "
            f"hysteresis={self.cfg.hysteresis_w}W "
            f"muting={self.cfg.muting_s}s "
            f"dry_run={self.cfg.dry_run}"
        )

        self._check_entities()

    def terminate(self) -> None:
        """Called by AppDaemon on shutdown — save state for next startup."""
        self._save_state()

    # --- State persistence --------------------------------

    def _save_state(self) -> None:
        """Atomically write controller state to JSON file."""
        try:
            tmp = self._state_file + ".tmp"
            with open(tmp, "w") as f:
                json.dump(self.logic.state_snapshot(), f)
            os.replace(tmp, self._state_file)
        except OSError as exc:
            self.log(f"Failed to save state: {exc}", level="WARNING")

    def _restore_state(self) -> bool:
        """Try to restore controller state from JSON file.

        Returns True if state was successfully restored.
        """
        try:
            with open(self._state_file) as f:
                data = json.load(f)
            return self.logic.restore_from_snapshot(data)
        except (FileNotFoundError, json.JSONDecodeError, OSError):
            return False

    def _check_entities(self) -> None:
        """Log availability of every configured HA entity at startup."""
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
                self.log(f"  WARN {label}: {entity_id} -> {raw!r}", level="WARNING")
            else:
                self.log(f"  OK {label}: {entity_id} -> {raw}")

    # --- Event-driven callbacks ----------------------------

    def _on_grid_change(self, entity: str, attribute: str, old: str, new: str, kwargs: dict) -> None:
        """Called by AppDaemon when the grid sensor state changes."""
        if new in UNAVAILABLE_STATES:
            return

        try:
            grid_w = float(new)
        except (ValueError, TypeError):
            return

        m = self._read_measurement(grid_w)
        if m is None:
            return

        output = self.logic.compute(m)
        if output is None:
            return  # muted or no action

        surplus = ControlLogic.estimate_surplus(m)
        self._log_output(output, m, surplus)
        self._publish_ha_sensors(output, m, surplus)
        self._publish_heartbeat()
        self._log_csv(output, m, surplus)
        self._save_state()

    def _safety_tick(self, _kwargs: dict) -> None:
        """Periodic check: if grid sensor hasn't updated in 60s, go safe."""
        last = self.get_state(self.cfg.grid_sensor, attribute="last_updated")
        if last is None:
            return
        try:
            from dateutil.parser import parse
            age = (datetime.now(timezone.utc) - parse(last)).total_seconds()
        except (ImportError, ValueError):
            return
        if age > 60:
            self.log("Grid sensor stale (>60s), setting safe state", level="WARNING")
            self._publish_safe_state()

    def _publish_safe_state(self) -> None:
        """Publish a zero-power safe state when the sensor is stale."""
        output = ControlOutput(desired_power_w=0.0, reason="Safe (sensor stale)")
        m_dummy = Measurement(grid_power_w=0.0, soc_pct=50.0, battery_power_w=0.0)
        self._publish_ha_sensors(output, m_dummy, 0.0)

    # --- Sensor reading ------------------------------------

    def _read_float(self, entity: str) -> float | None:
        """Read an HA entity's state as a float, returning None on failure."""
        raw: str | None = self.get_state(entity)
        if raw in UNAVAILABLE_STATES:
            return None
        try:
            return float(raw)  # type: ignore[arg-type]
        except (ValueError, TypeError):
            self.log(f"  {entity} -> {raw!r} (not a number)", level="WARNING")
            return None

    def _read_measurement(self, grid_w: float) -> Measurement | None:
        """Assemble a ``Measurement`` from HA sensor states.

        The grid power is passed in directly from the event callback.
        Returns ``None`` (and logs a warning) when any required sensor
        is unavailable.
        """
        soc = self._read_float(self.cfg.soc_sensor)
        bp = self._read_float(self.cfg.battery_power_sensor)

        unavailable = []
        if soc is None:
            unavailable.append(f"soc ({self.cfg.soc_sensor})")
        if bp is None:
            unavailable.append(f"battery_power ({self.cfg.battery_power_sensor})")

        if unavailable:
            self.log(
                f"Sensor unavailable: {', '.join(unavailable)}, skipping",
                level="WARNING",
            )
            return None

        assert soc is not None  # guaranteed by check above
        assert bp is not None

        return Measurement(
            grid_power_w=grid_w,
            soc_pct=soc,
            battery_power_w=bp,
            discharge_enabled=self._is_switch_on(self.cfg.discharge_switch),
            charge_enabled=self._is_switch_on(self.cfg.charge_switch),
            dynamic_min_soc_pct=self._read_float(self.cfg.dynamic_min_soc_entity)
            if self.cfg.dynamic_min_soc_entity
            else None,
        )

    def _is_switch_on(self, entity: str | None) -> bool:
        """Return True if the switch entity is on, or if no entity is configured."""
        if entity is None:
            return True
        return self.get_state(entity) == "on"

    # --- Output --------------------------------------------

    def _log_output(
        self, output: ControlOutput, m: Measurement, surplus: float,
    ) -> None:
        """Emit a single-line summary to the AppDaemon log."""
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
            f"drift={self.logic.state.drift_acc:.0f} | "
            f"{output.reason}"
        )

    def _log_csv(
        self, output: ControlOutput, m: Measurement, surplus: float,
    ) -> None:
        """Append one row to the CSV log file (if file logging is enabled)."""
        if self._csv is None:
            return
        target = self.logic.target_for_mode()
        now = time.monotonic()
        muting_remaining = max(
            0.0,
            self.cfg.muting_s - (now - self.logic.state.last_command_t),
        )
        self._csv.log_row({
            "grid_w": round(m.grid_power_w),
            "soc_pct": round(m.soc_pct, 1),
            "battery_power_w": round(m.battery_power_w),
            "surplus_w": round(surplus),
            "mode": self.logic.state.mode.name,
            "desired_power_w": round(output.desired_power_w),
            "error_w": round(m.grid_power_w - target),
            "target_w": round(target),
            "drift_acc": round(self.logic.state.drift_acc),
            "muting": round(muting_remaining, 1),
            "reason": output.reason,
        })

    # --- HA sensor publishing ------------------------------

    def _set_sensor(
        self,
        name: str,
        value: object,
        unit: str | None = None,
        icon: str | None = None,
    ) -> None:
        """Create or update a ``sensor.zfi_<name>`` entity in HA.

        Includes a ``last_published`` ISO timestamp in the attributes so
        that HA always sees a state change and advances ``last_updated``.
        """
        entity_id = f"{self.cfg.sensor_prefix}_{name}"
        attrs = {"friendly_name": f"ZFI {name.replace('_', ' ').title()}"}
        if unit:
            attrs["unit_of_measurement"] = unit
            attrs["state_class"] = "measurement"
        if icon:
            attrs["icon"] = icon
        attrs["last_published"] = datetime.now(timezone.utc).isoformat()
        try:
            self.set_state(entity_id, state=str(value), attributes=attrs, replace=True)
        except Exception as exc:
            self.log(
                f"set_state failed for {entity_id}: {exc}",
                level="WARNING",
            )

    def _publish_heartbeat(self) -> None:
        """Publish an MQTT heartbeat timestamp for external monitoring."""
        topic = self.cfg.heartbeat_mqtt_topic
        if not topic:
            return
        try:
            self.call_service(
                "mqtt/publish",
                topic=topic,
                payload=datetime.now(timezone.utc).isoformat(),
                retain=False,
            )
        except Exception as exc:  # noqa: BLE001
            self.log(f"Heartbeat publish failed: {exc}", level="WARNING")

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

        # Measurement inputs (required for operations dashboard)
        self._set_sensor(
            "grid_power",
            round(m.grid_power_w),
            "W",
            "mdi:transmission-tower",
        )
        self._set_sensor(
            "soc",
            round(m.soc_pct),
            "%",
            "mdi:battery",
        )

        if not self.cfg.debug:
            return

        # -- Debug-only sensors below ----------------------

        # Physical estimates
        self._set_sensor("surplus", round(surplus), "W", "mdi:solar-power")
        self._set_sensor(
            "battery_power",
            round(m.battery_power_w),
            "W",
            "mdi:battery-charging",
        )

        # Controller internals
        target_w = self.logic.target_for_mode()
        self._set_sensor("target", round(target_w), "W", "mdi:target")
        self._set_sensor(
            "error", round(m.grid_power_w - target_w), "W", "mdi:delta",
        )
        self._set_sensor(
            "drift_acc", round(self.logic.state.drift_acc), "W", "mdi:sigma",
        )

        now = time.monotonic()
        muting_remaining = max(
            0.0,
            self.cfg.muting_s - (now - self.logic.state.last_command_t),
        )
        self._set_sensor(
            "muting", 1 if muting_remaining > 0 else 0, icon="mdi:timer-sand",
        )
        self._set_sensor(
            "muting_remaining", round(muting_remaining, 1), "s", "mdi:timer-sand",
        )
        self._set_sensor("reason", output.reason, icon="mdi:information-outline")

        # Dynamic min SOC (shows effective value after clamping)
        if m.dynamic_min_soc_pct is not None:
            effective = max(
                self.cfg.min_soc_pct,
                min(m.dynamic_min_soc_pct, self.cfg.max_soc_pct),
            )
        else:
            effective = self.cfg.min_soc_pct
        self._set_sensor(
            "effective_min_soc", round(effective), "%", "mdi:battery-low",
        )
