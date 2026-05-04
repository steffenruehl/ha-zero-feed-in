"""
Pulse-Load Filter — Baseline mitigation for periodic pulse loads.

Subscribes to a detector's ``active`` entity (e.g. from
``pulse_load_detector``) and the raw grid sensor.  When the detector
signals a pulse load, this filter outputs a stable baseline instead
of the oscillating grid signal.

Three stages once activated:
  1. **Settle delay** — pause battery and wait ~15 s for the inverter
     to actually ramp down.  Samples during this phase are discarded.
  2. **Measurement** — collect grid samples for ~60 s with the battery
     truly silent, then ``min(grid)`` → exact baseline.
  3. **I-controller drift tracking** — correct baseline every 60 s to
     track slow house-load changes.

Output convention:
  - When inactive: passes raw grid value through (debug entity only).
  - When active, measuring: outputs 0 W (battery paused).
  - When active, baseline ready: outputs +baseline (desired discharge
    power, same sign convention as the controller).

When not in dry-run mode, the filter writes directly to the
``desired_power_entity`` (default ``sensor.zfi_desired_power``),
taking over from the controller.  The controller must be configured
with the same ``pulse_load_active_entity`` so it yields control.

Published entities:
  - ``filtered_power_entity`` — debug output (always written)
  - ``desired_power_entity`` — driver input (written when active + not dry_run)
  - ``{sensor_prefix}_measuring`` — 1 during measurement pause, 0 otherwise
  - ``{sensor_prefix}_baseline``  — current baseline estimate (W)
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    import appdaemon.plugins.hass.hassapi as hass

try:
    import appdaemon.plugins.hass.hassapi as hass

    _HASS_BASE = hass.Hass
except ImportError:
    _HASS_BASE = object  # type: ignore[assignment,misc]


# ═══════════════════════════════════════════════════════════
#  Constants
# ═══════════════════════════════════════════════════════════

UNAVAILABLE_STATES = {None, "unknown", "unavailable"}
"""HA entity states indicating a sensor is offline."""

DEFAULT_FILTERED_POWER_ENTITY = "sensor.zfi_plf_filtered_grid_power"
"""Default entity ID for the filtered grid power output."""

DEFAULT_DESIRED_POWER_ENTITY = "sensor.zfi_desired_power"
"""Default entity ID for the desired battery power (driver input)."""

DEFAULT_SENSOR_PREFIX = "sensor.zfi_plf"
"""Prefix for HA status/debug entities published by this filter."""


# ═══════════════════════════════════════════════════════════
#  Configuration
# ═══════════════════════════════════════════════════════════


@dataclass
class FilterConfig:
    """Typed configuration for the pulse-load filter (mitigation).

    Loaded from ``apps.yaml`` via ``from_args()``.
    """

    # Input sensors
    grid_sensor: str
    """HA entity ID for raw grid power sensor (W)."""
    active_entity: str
    """HA entity ID published by the detector (``1`` = active)."""

    # Measurement pause
    settle_duration_s: float = 15.0
    """Settle delay before collecting baseline samples (s).

    After the battery-pause command is issued, the inverter takes
    10–15 s to actually ramp down.  Samples during this phase are
    discarded to avoid corrupting the baseline with residual
    battery output.
    """
    measurement_duration_s: float = 60.0
    """How long to collect baseline samples after settle (s)."""

    # I-controller drift tracking
    drift_target_w: float = 30.0
    """Desired grid import during oven-OFF phases (W)."""
    drift_ki: float = 0.3
    """I-controller gain per observation cycle."""
    drift_cycle_s: float = 60.0
    """Observation window for I-controller correction (s)."""

    # Output
    filtered_power_entity: str = DEFAULT_FILTERED_POWER_ENTITY
    """Entity ID for the filtered grid power output (W). Debug/dashboard use."""
    desired_power_entity: str = DEFAULT_DESIRED_POWER_ENTITY
    """Entity ID for the desired battery power output (W).

    Written when active and not dry_run.  Same semantics as the
    controller output: +discharge / -charge.  The driver reads this.
    """
    sensor_prefix: str = DEFAULT_SENSOR_PREFIX
    """Prefix for status/debug HA entities (measuring, baseline)."""
    dry_run: bool = True
    """When True, publish sensors but don't pause battery."""
    debug: bool = False
    """Publish extra debug sensors."""

    @classmethod
    def from_args(cls, args: dict[str, Any]) -> FilterConfig:
        """Create a ``FilterConfig`` from the AppDaemon ``args`` dict."""
        return cls(
            grid_sensor=args["grid_power_sensor"],
            active_entity=args["active_entity"],
            settle_duration_s=float(
                args.get("settle_duration_s", cls.settle_duration_s)
            ),
            measurement_duration_s=float(
                args.get("measurement_duration_s", cls.measurement_duration_s)
            ),
            drift_target_w=float(
                args.get("drift_target_w", cls.drift_target_w)
            ),
            drift_ki=float(args.get("drift_ki", cls.drift_ki)),
            drift_cycle_s=float(args.get("drift_cycle_s", cls.drift_cycle_s)),
            filtered_power_entity=args.get(
                "filtered_power_entity", cls.filtered_power_entity
            ),
            desired_power_entity=args.get(
                "desired_power_entity", cls.desired_power_entity
            ),
            sensor_prefix=args.get("sensor_prefix", cls.sensor_prefix),
            dry_run=bool(args.get("dry_run", cls.dry_run)),
            debug=bool(args.get("debug", cls.debug)),
        )


# ═══════════════════════════════════════════════════════════
#  Baseline Estimator (measurement pause + I-controller)
# ═══════════════════════════════════════════════════════════


class BaselineEstimator:
    """Estimates the true house-load baseline during pulse-load filtering.

    Three phases:
    1. **Settle** — battery pause command issued; inverter ramps down
       over 10–15 s.  Grid samples are discarded.
    2. **Measurement** — battery is truly silent; ``min(grid)`` over one
       full oven cycle gives the exact baseline.
    3. **Drift tracking** — an I-controller adjusts the baseline every
       ``drift_cycle_s`` based on ``min(grid) - target``.
    """

    def __init__(self, cfg: FilterConfig) -> None:
        self._cfg = cfg
        self.measuring: bool = False
        """True during the measurement pause phase."""
        self.baseline: float | None = None
        """Current baseline estimate (W). None before first measurement."""
        self._measure_start_t: float = 0.0
        """Monotonic time when measurement pause began."""
        self._measure_samples: list[float] = []
        """Grid samples collected during measurement pause."""
        self._drift_cycle_start_t: float = 0.0
        """Start of current I-controller observation window."""
        self._drift_cycle_min: float = float("inf")
        """Minimum grid value in current drift cycle."""

    def start_measurement(self, now: float) -> None:
        """Begin the measurement pause phase.

        Args:
            now: Monotonic timestamp (seconds).
        """
        self.measuring = True
        self._measure_start_t = now
        self._measure_samples = []

    def update(self, grid_w: float, now: float) -> float | None:
        """Process one grid sample, return baseline if available.

        During settle: discards samples while inverter ramps down.
        During measurement: collects samples, completes when duration
        expires.  After measurement: runs I-controller drift correction.

        Args:
            grid_w: Current grid power reading (W).
            now: Monotonic timestamp (seconds).

        Returns:
            Current baseline estimate, or ``None`` if still measuring.
        """
        if self.measuring:
            elapsed = now - self._measure_start_t
            settle = self._cfg.settle_duration_s
            # Only collect samples after the settle phase
            if elapsed >= settle:
                self._measure_samples.append(grid_w)
            if elapsed >= settle + self._cfg.measurement_duration_s:
                self._complete_measurement(now)
            return self.baseline

        if self.baseline is None:
            return None

        # Asymmetric drift correction
        #
        # Negative grid = export = battery definitely overshooting.
        # This is unambiguous (oven pulses only cause positive spikes),
        # so correct immediately with full error.
        #
        # Positive grid = import = could be increased house load OR an
        # oven-ON spike.  Use min(grid) over a full drift cycle to
        # filter out pulse-load spikes before correcting upward.
        if grid_w < 0:
            error = grid_w - self._cfg.drift_target_w
            self.baseline += error
            # Reset cycle so we don't double-count this sample
            self._drift_cycle_start_t = now
            self._drift_cycle_min = float("inf")
        else:
            self._drift_cycle_min = min(self._drift_cycle_min, grid_w)
            elapsed = now - self._drift_cycle_start_t
            if elapsed >= self._cfg.drift_cycle_s:
                error = self._drift_cycle_min - self._cfg.drift_target_w
                self.baseline += self._cfg.drift_ki * error
                # Reset cycle
                self._drift_cycle_start_t = now
                self._drift_cycle_min = float("inf")

        return self.baseline

    def reset(self) -> None:
        """Reset estimator state (called on filter deactivation)."""
        self.measuring = False
        self.baseline = None
        self._measure_samples = []
        self._drift_cycle_start_t = 0.0
        self._drift_cycle_min = float("inf")

    def _complete_measurement(self, now: float) -> None:
        """Finalize the measurement pause and set the initial baseline.

        Args:
            now: Monotonic timestamp (seconds).
        """
        self.measuring = False
        if self._measure_samples:
            self.baseline = min(self._measure_samples)
        else:
            self.baseline = self._cfg.drift_target_w
        # Start drift tracking from here
        self._drift_cycle_start_t = now
        self._drift_cycle_min = float("inf")


# ═══════════════════════════════════════════════════════════
#  Filter Logic (pure, no HA dependency)
# ═══════════════════════════════════════════════════════════


class PulseLoadFilterLogic:
    """Stateful filter driven by an external active signal.

    Pure computation — no HA or AppDaemon dependency.  The HA adapter
    feeds samples and the active flag, and reads the filtered output.
    """

    def __init__(self, cfg: FilterConfig) -> None:
        self.cfg = cfg
        self.estimator = BaselineEstimator(cfg)
        self._was_active: bool = False
        """Previous active state for edge detection."""

    @property
    def active(self) -> bool:
        """Whether the filter is currently engaged (last known state)."""
        return self._was_active

    @property
    def measuring(self) -> bool:
        """Whether the filter is in measurement-pause phase."""
        return self.estimator.measuring

    @property
    def baseline(self) -> float | None:
        """Current baseline estimate, or None if not yet measured."""
        return self.estimator.baseline

    def update(self, grid_w: float, now: float, active: bool) -> float:
        """Process one grid sample and return the desired battery power.

        Output semantics (same as controller: +discharge / -charge):
          - **Inactive**: returns raw ``grid_w`` (pass-through for debug).
          - **Active, measuring**: returns ``0.0`` (battery paused).
          - **Active, baseline ready**: returns ``+baseline`` (discharge
            at baseline rate to cover house load).

        Args:
            grid_w: Raw grid power reading (W).
            now: Monotonic timestamp (seconds).
            active: Whether the detector considers a pulse load present.

        Returns:
            Desired battery power when active, or raw grid pass-through
            when inactive.
        """
        was_active = self._was_active

        # Rising edge: just activated → start measurement pause
        if active and not was_active:
            self.estimator.start_measurement(now)
            self._was_active = active
            return 0.0  # pause battery during measurement

        # Falling edge: deactivated → reset estimator
        if not active and was_active:
            self.estimator.reset()

        self._was_active = active

        if not active:
            return grid_w

        # Active: update estimator and return desired battery power
        baseline = self.estimator.update(grid_w, now)
        if baseline is not None:
            return baseline  # +baseline = discharge at house-load rate
        # Still measuring, no baseline yet — keep battery paused
        return 0.0


# ═══════════════════════════════════════════════════════════
#  AppDaemon HA Adapter
# ═══════════════════════════════════════════════════════════


class PulseLoadFilter(_HASS_BASE):
    """AppDaemon app: baseline mitigation for periodic pulse loads.

    Subscribes to a detector's active entity and the raw grid sensor.
    When active, outputs a stable baseline via ``filtered_power_entity``.
    """

    cfg: FilterConfig
    logic: PulseLoadFilterLogic

    def initialize(self) -> None:
        """Called by AppDaemon on startup."""
        self.cfg = FilterConfig.from_args(self.args)
        self.logic = PulseLoadFilterLogic(self.cfg)
        self._monotonic_offset = time.monotonic()
        self._detector_active: bool = False
        """Last known detector state from the active entity."""

        # Subscribe to grid sensor for value updates
        self.listen_state(
            self._on_grid_change,
            self.cfg.grid_sensor,
        )
        # Subscribe to detector active entity for state changes
        self.listen_state(
            self._on_active_change,
            self.cfg.active_entity,
        )
        self.log(
            "PulseLoadFilter initialized (active_entity=%s, "
            "desired_power_entity=%s, dry_run=%s)",
            self.cfg.active_entity,
            self.cfg.desired_power_entity,
            self.cfg.dry_run,
        )

    def _on_active_change(
        self,
        entity: str,
        attribute: str,
        old: str,
        new: str,
        kwargs: dict[str, Any],
    ) -> None:
        """Called by AppDaemon when the detector active entity changes."""
        if new in UNAVAILABLE_STATES:
            return
        self._detector_active = new == "1"

    def _on_grid_change(
        self,
        entity: str,
        attribute: str,
        old: str,
        new: str,
        kwargs: dict[str, Any],
    ) -> None:
        """Called by AppDaemon when the grid sensor state changes."""
        if new in UNAVAILABLE_STATES:
            return
        try:
            grid_w = float(new)
        except (ValueError, TypeError):
            return

        now = time.monotonic() - self._monotonic_offset
        was_measuring = self.logic.measuring

        filtered = self.logic.update(grid_w, now, self._detector_active)

        # Publish filtered output (debug/dashboard — always written)
        self._set_entity(
            self.cfg.filtered_power_entity,
            round(filtered, 1),
            unit="W",
        )

        # Write desired_power when active and not dry_run
        if self._detector_active and not self.cfg.dry_run:
            self._set_entity(
                self.cfg.desired_power_entity,
                round(filtered),
                unit="W",
            )

        # Publish state sensors
        self._set_sensor("measuring", int(self.logic.measuring))
        if self.logic.baseline is not None:
            self._set_sensor("baseline", round(self.logic.baseline, 1), unit="W")

        # Log state transitions
        if self.logic.measuring and not was_measuring:
            self.log("Measurement pause started")
        elif not self.logic.measuring and was_measuring:
            self.log(
                "Measurement complete — baseline=%.1f W",
                self.logic.baseline,
            )

    def _set_entity(
        self,
        entity_id: str,
        value: object,
        unit: str | None = None,
    ) -> None:
        """Create or update a HA entity by full entity ID."""
        friendly = entity_id.split(".", 1)[-1].replace("_", " ").title()
        attrs: dict[str, Any] = {"friendly_name": friendly}
        if unit:
            attrs["unit_of_measurement"] = unit
            attrs["state_class"] = "measurement"
        attrs["last_published"] = datetime.now(timezone.utc).isoformat()
        try:
            self.set_state(entity_id, state=str(value), attributes=attrs, replace=True)
        except Exception as exc:  # noqa: BLE001
            self.log(
                "set_state failed for %s: %s",
                entity_id,
                exc,
                level="WARNING",
            )

    def _set_sensor(
        self,
        name: str,
        value: object,
        unit: str | None = None,
    ) -> None:
        """Create or update a ``{sensor_prefix}_{name}`` entity in HA."""
        self._set_entity(
            f"{self.cfg.sensor_prefix}_{name}", value, unit=unit
        )
