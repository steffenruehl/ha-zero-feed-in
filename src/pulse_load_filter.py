"""
Pulse-Load Filter — Removes periodic pulse loads from the grid sensor.

Standalone AppDaemon app that sits between the raw grid sensor and the
controller.  During oven-style duty-cycling (e.g. 15 s ON / 45 s OFF),
it outputs a stable baseline instead of the raw oscillating signal.

Three stages:
  1. **Sign-flip detection** — grid rapidly flips between large import
     and large export → controller is futilely chasing oscillations.
  2. **Measurement pause** — pause battery for ~60 s, measure
     ``min(grid)`` → exact baseline with no inverter-loss offset.
  3. **I-controller drift tracking** — correct baseline every 60 s to
     track slow house-load changes.

Output convention:
  - When inactive: passes raw grid value through transparently.
  - When active: outputs the estimated baseline.

Published entities (``sensor.zfi_plf_*``):
  - ``filtered_grid_power`` — the output value (raw or baseline)
  - ``active``              — 1 when filter is engaged, 0 otherwise
  - ``measuring``           — 1 during measurement pause, 0 otherwise
  - ``baseline``            — current baseline estimate (W)
  - ``flip_count``          — sign flips in the activation window
"""

from __future__ import annotations

import time
from collections import deque
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

DEFAULT_SENSOR_PREFIX = "sensor.zfi_plf"
"""Prefix for HA entities published by this filter."""


# ═══════════════════════════════════════════════════════════
#  Configuration
# ═══════════════════════════════════════════════════════════


@dataclass
class FilterConfig:
    """Typed configuration for the pulse-load filter.

    Loaded from ``apps.yaml`` via ``from_args()``.
    """

    # Sensor
    grid_sensor: str
    """HA entity ID for raw grid power sensor (W)."""

    # Sign-flip detection
    flip_thresh_w: float = 500.0
    """Minimum grid magnitude for each leg of a sign flip (W)."""
    flip_window_s: float = 20.0
    """Max time between positive and negative legs of a flip (s)."""
    activate_count: int = 2
    """Number of sign flips needed to activate the filter."""
    activate_window_s: float = 120.0
    """Sliding window for counting sign flips (s)."""
    deactivate_quiet_s: float = 120.0
    """Deactivate when no |grid| > flip_thresh_w for this long (s)."""

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
    desired_power_entity: str = "sensor.zfi_desired_power"
    """Entity the filter writes to pause/resume the battery."""
    sensor_prefix: str = DEFAULT_SENSOR_PREFIX
    """Prefix for published HA entities."""
    dry_run: bool = True
    """When True, publish sensors but don't override desired power."""
    debug: bool = False
    """Publish extra debug sensors (flip_count, etc.)."""

    @classmethod
    def from_args(cls, args: dict[str, Any]) -> FilterConfig:
        """Create a ``FilterConfig`` from the AppDaemon ``args`` dict."""
        return cls(
            grid_sensor=args["grid_power_sensor"],
            flip_thresh_w=float(args.get("flip_thresh_w", cls.flip_thresh_w)),
            flip_window_s=float(args.get("flip_window_s", cls.flip_window_s)),
            activate_count=int(args.get("activate_count", cls.activate_count)),
            activate_window_s=float(
                args.get("activate_window_s", cls.activate_window_s)
            ),
            deactivate_quiet_s=float(
                args.get("deactivate_quiet_s", cls.deactivate_quiet_s)
            ),
            measurement_duration_s=float(
                args.get("measurement_duration_s", cls.measurement_duration_s)
            ),
            drift_target_w=float(
                args.get("drift_target_w", cls.drift_target_w)
            ),
            drift_ki=float(args.get("drift_ki", cls.drift_ki)),
            drift_cycle_s=float(args.get("drift_cycle_s", cls.drift_cycle_s)),
            desired_power_entity=args.get(
                "desired_power_entity", cls.desired_power_entity
            ),
            sensor_prefix=args.get("sensor_prefix", cls.sensor_prefix),
            dry_run=bool(args.get("dry_run", cls.dry_run)),
            debug=bool(args.get("debug", cls.debug)),
        )


# ═══════════════════════════════════════════════════════════
#  Sign-Flip Detector
# ═══════════════════════════════════════════════════════════


class SignFlipDetector:
    """Detects futile controller response via grid power sign flips.

    A sign flip is registered when, within ``flip_window_s``, both a
    sample with ``grid > +flip_thresh_w`` and one with
    ``grid < -flip_thresh_w`` are observed.

    Activation: ``>= activate_count`` flips within ``activate_window_s``.
    Deactivation: no ``|grid| > flip_thresh_w`` for ``deactivate_quiet_s``.
    """

    def __init__(self, cfg: FilterConfig) -> None:
        self._cfg = cfg
        self._recent: deque[tuple[float, float]] = deque()
        """Recent ``(timestamp, grid_w)`` samples within the flip window."""
        self._flip_times: deque[float] = deque()
        """Timestamps of detected sign flips."""
        self._last_flip_check_t: float = -cfg.flip_window_s - 1.0
        """Prevents double-counting the same flip event."""
        self._last_large_t: float = 0.0
        """Last time |grid| exceeded the flip threshold."""
        self.active: bool = False
        """Whether the detector considers the filter should be active."""

    @property
    def flip_count(self) -> int:
        """Number of sign flips in the current activation window."""
        return len(self._flip_times)

    def update(self, grid_w: float, now: float) -> bool:
        """Process one grid sample, return whether detector is active.

        Args:
            grid_w: Current grid power reading (W).
            now: Monotonic timestamp (seconds).

        Returns:
            ``True`` if the filter should be active.
        """
        cfg = self._cfg

        # Track large grid excursions for deactivation
        if abs(grid_w) > cfg.flip_thresh_w:
            self._last_large_t = now

        # Maintain recent-sample window
        self._recent.append((now, grid_w))
        cutoff = now - cfg.flip_window_s
        while self._recent and self._recent[0][0] < cutoff:
            self._recent.popleft()

        # Check for sign flip in current window
        has_pos = any(g > cfg.flip_thresh_w for _, g in self._recent)
        has_neg = any(g < -cfg.flip_thresh_w for _, g in self._recent)
        is_flip = has_pos and has_neg

        # Register flip (debounce: one per flip_window_s)
        if is_flip and (now - self._last_flip_check_t) > cfg.flip_window_s:
            self._flip_times.append(now)
            self._last_flip_check_t = now

        # Prune old flips
        flip_cutoff = now - cfg.activate_window_s
        while self._flip_times and self._flip_times[0] < flip_cutoff:
            self._flip_times.popleft()

        # State transitions
        if not self.active:
            if len(self._flip_times) >= cfg.activate_count:
                self.active = True
        else:
            quiet_duration = now - self._last_large_t
            if self._last_large_t > 0 and quiet_duration > cfg.deactivate_quiet_s:
                self.active = False
                self._flip_times.clear()
                self._last_flip_check_t = 0.0

        return self.active


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
    """Stateful filter combining sign-flip detection and baseline estimation.

    Pure computation — no HA or AppDaemon dependency.  The HA adapter
    feeds samples in and reads the output.
    """

    def __init__(self, cfg: FilterConfig) -> None:
        self.cfg = cfg
        self.detector = SignFlipDetector(cfg)
        self.estimator = BaselineEstimator(cfg)
        self._was_active: bool = False
        """Previous active state for edge detection."""

    @property
    def active(self) -> bool:
        """Whether the filter is currently engaged."""
        return self.detector.active

    @property
    def measuring(self) -> bool:
        """Whether the filter is in measurement-pause phase."""
        return self.estimator.measuring

    @property
    def baseline(self) -> float | None:
        """Current baseline estimate, or None if not yet measured."""
        return self.estimator.baseline

    @property
    def flip_count(self) -> int:
        """Number of sign flips in the activation window."""
        return self.detector.flip_count

    def update(self, grid_w: float, now: float) -> float:
        """Process one grid sample and return the filtered output.

        Args:
            grid_w: Raw grid power reading (W).
            now: Monotonic timestamp (seconds).

        Returns:
            Filtered grid power — either the raw value (inactive) or
            the estimated baseline (active).
        """
        was_active = self._was_active
        is_active = self.detector.update(grid_w, now)

        # Rising edge: just activated → start measurement pause
        # Don't feed the activation sample to the estimator — the
        # battery hasn't been paused yet, so this sample is from the
        # old (chasing) state.
        if is_active and not was_active:
            self.estimator.start_measurement(now)
            self._was_active = is_active
            return grid_w

        # Falling edge: deactivated → reset estimator
        if not is_active and was_active:
            self.estimator.reset()

        self._was_active = is_active

        if not is_active:
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
    """AppDaemon app: filters periodic pulse loads from the grid sensor.

    Subscribes to the raw grid sensor, runs ``PulseLoadFilterLogic``,
    and publishes the filtered output as ``sensor.zfi_plf_filtered_grid_power``.
    In dry-run mode (default), does not pause the battery — only publishes
    sensor entities for dashboard observation.
    """

    cfg: FilterConfig
    logic: PulseLoadFilterLogic

    def initialize(self) -> None:
        """Called by AppDaemon on startup."""
        self.cfg = FilterConfig.from_args(self.args)
        self.logic = PulseLoadFilterLogic(self.cfg)
        self._monotonic_offset = time.monotonic()

        self.listen_state(
            self._on_grid_change,
            self.cfg.grid_sensor,
        )
        self.log("PulseLoadFilter initialized (dry_run=%s)", self.cfg.dry_run)

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

        filtered = self.logic.update(grid_w, now)

        # Publish filtered output
        self._set_sensor("filtered_grid_power", round(filtered, 1), unit="W")

        # Publish state sensors
        self._set_sensor("active", int(self.logic.active))
        self._set_sensor("measuring", int(self.logic.measuring))
        if self.logic.baseline is not None:
            self._set_sensor("baseline", round(self.logic.baseline, 1), unit="W")

        if self.cfg.debug:
            self._set_sensor("flip_count", self.logic.flip_count)

        # Battery pause control (measurement phase)
        if not self.cfg.dry_run:
            if self.logic.measuring and not was_measuring:
                self.log("Measurement pause started — pausing battery")
                self._pause_battery()
            elif not self.logic.measuring and was_measuring:
                self.log(
                    "Measurement complete — baseline=%.1f W",
                    self.logic.baseline,
                )

    def _pause_battery(self) -> None:
        """Request the controller to output zero power during measurement.

        Sets the desired power entity to 0 so the battery stops
        discharging while we measure the true baseline.
        """
        self.set_state(
            self.cfg.desired_power_entity,
            state="0",
            attributes={
                "friendly_name": "ZFI Desired Power",
                "unit_of_measurement": "W",
                "pulse_load_filter_override": True,
            },
            replace=True,
        )

    def _set_sensor(
        self,
        name: str,
        value: object,
        unit: str | None = None,
    ) -> None:
        """Create or update a ``sensor.zfi_plf_<name>`` entity in HA."""
        entity_id = f"{self.cfg.sensor_prefix}_{name}"
        attrs: dict[str, Any] = {
            "friendly_name": f"ZFI PLF {name.replace('_', ' ').title()}",
        }
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
