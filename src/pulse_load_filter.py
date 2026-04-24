"""
Pulse-Load Filter — Baseline mitigation for periodic pulse loads.

Subscribes to a detector's ``active`` entity (e.g. from
``pulse_load_detector``) and the raw grid sensor.  When the detector
signals a pulse load, this filter outputs a stable baseline instead
of the oscillating grid signal.

Two stages once activated:
  1. **Measurement pause** — pause battery for ~60 s, measure
     ``min(grid)`` → exact baseline with no inverter-loss offset.
  2. **I-controller drift tracking** — correct baseline every 60 s to
     track slow house-load changes.

Output convention:
  - When inactive: passes raw grid value through transparently.
  - When active: outputs the estimated baseline.

Published entities:
  - ``filtered_power_entity`` — the main output (configurable entity ID)
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
    measurement_duration_s: float = 60.0
    """How long to pause battery and measure baseline (s)."""

    # I-controller drift tracking
    drift_target_w: float = 30.0
    """Desired grid import during oven-OFF phases (W)."""
    drift_ki: float = 0.3
    """I-controller gain per observation cycle."""
    drift_cycle_s: float = 60.0
    """Observation window for I-controller correction (s)."""

    # Output
    filtered_power_entity: str = DEFAULT_FILTERED_POWER_ENTITY
    """Entity ID for the filtered grid power output (W)."""
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
            sensor_prefix=args.get("sensor_prefix", cls.sensor_prefix),
            dry_run=bool(args.get("dry_run", cls.dry_run)),
            debug=bool(args.get("debug", cls.debug)),
        )


# ═══════════════════════════════════════════════════════════
#  Baseline Estimator (measurement pause + I-controller)
# ═══════════════════════════════════════════════════════════


class BaselineEstimator:
    """Estimates the true house-load baseline during pulse-load filtering.

    Two phases:
    1. **Measurement pause** — battery is paused, ``min(grid)`` over one
       full oven cycle gives the exact baseline.
    2. **Drift tracking** — an I-controller adjusts the baseline every
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

        During measurement: collects samples, completes when duration
        expires.  After measurement: runs I-controller drift correction.

        Args:
            grid_w: Current grid power reading (W).
            now: Monotonic timestamp (seconds).

        Returns:
            Current baseline estimate, or ``None`` if still measuring.
        """
        if self.measuring:
            self._measure_samples.append(grid_w)
            elapsed = now - self._measure_start_t
            if elapsed >= self._cfg.measurement_duration_s:
                self._complete_measurement(now)
            return self.baseline

        if self.baseline is None:
            return None

        # I-controller drift tracking
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
        """Process one grid sample and return the filtered output.

        Args:
            grid_w: Raw grid power reading (W).
            now: Monotonic timestamp (seconds).
            active: Whether the detector considers a pulse load present.

        Returns:
            Filtered grid power — either the raw value (inactive) or
            the estimated baseline (active).
        """
        was_active = self._was_active

        # Rising edge: just activated → start measurement pause
        # Don't feed the activation sample to the estimator — the
        # battery hasn't been paused yet, so this sample is from the
        # old (chasing) state.
        if active and not was_active:
            self.estimator.start_measurement(now)
            self._was_active = active
            return grid_w

        # Falling edge: deactivated → reset estimator
        if not active and was_active:
            self.estimator.reset()

        self._was_active = active

        if not active:
            return grid_w

        # Active: update estimator and return baseline (or raw if measuring)
        baseline = self.estimator.update(grid_w, now)
        if baseline is not None:
            return baseline
        # Still measuring, no baseline yet — pass through raw
        return grid_w


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
            "PulseLoadFilter initialized (active_entity=%s, dry_run=%s)",
            self.cfg.active_entity,
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

        # Publish filtered output
        self._set_entity(
            self.cfg.filtered_power_entity,
            round(filtered, 1),
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
        # TODO: battery pause mechanism for non-dry-run mode.
        # The controller should read the detector's active entity and
        # zero its output during measurement.  For now, dry_run only.

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
