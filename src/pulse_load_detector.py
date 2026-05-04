"""
Pulse-Load Detector — Detects periodic pulse loads on the grid sensor.

Standalone AppDaemon app that monitors the raw grid sensor for signs of
futile controller chasing caused by duty-cycling appliances (e.g. oven
15 s ON / 45 s OFF).  Detection is based on **sign-flip analysis**: the
grid rapidly alternates between large import and large export.

This module only *detects* — it does not mitigate.  A separate mitigation
module (e.g. ``pulse_load_filter``) subscribes to the active entity and
implements the chosen reaction strategy.

Published entities (``{sensor_prefix}_*``):
  - ``active``       — 1 when pulse load detected, 0 otherwise
  - ``flip_count``   — sign flips in the activation window (debug)
  - ``energy_ratio`` — grid export / battery discharge energy ratio (debug)
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

try:
    from .csv_logger import CsvLogger
except ImportError:
    from csv_logger import CsvLogger  # type: ignore[no-redef]


# ═══════════════════════════════════════════════════════════
#  Constants
# ═══════════════════════════════════════════════════════════

UNAVAILABLE_STATES = {None, "unknown", "unavailable"}
"""HA entity states indicating a sensor is offline."""

DEFAULT_SENSOR_PREFIX = "sensor.zfi_pld"
"""Prefix for HA entities published by this detector."""


# ═══════════════════════════════════════════════════════════
#  Configuration
# ═══════════════════════════════════════════════════════════


@dataclass
class DetectorConfig:
    """Typed configuration for the pulse-load detector.

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
    """Number of sign flips needed to activate."""
    activate_window_s: float = 120.0
    """Sliding window for counting sign flips (s)."""
    deactivate_quiet_s: float = 120.0
    """Deactivate when no |grid| > flip_thresh_w for this long (s)."""

    # Energy-ratio detection
    battery_power_sensor: str | None = None
    """HA entity ID for battery power sensor (W). +discharge / -charge.
    When set, activation requires both sign flips AND energy ratio."""
    energy_window_s: float = 120.0
    """Sliding window for energy integration (s)."""
    energy_ratio_thresh: float = 0.3
    """Minimum grid_export / battery_discharge ratio to activate (0–1)."""

    # Output
    sensor_prefix: str = DEFAULT_SENSOR_PREFIX
    """Prefix for published HA entities."""
    debug: bool = False
    """Publish extra debug sensors (flip_count, energy_ratio)."""
    log_dir: str = ""
    """Directory for CSV logs. Empty string disables logging."""

    @classmethod
    def from_args(cls, args: dict[str, Any]) -> DetectorConfig:
        """Create a ``DetectorConfig`` from the AppDaemon ``args`` dict."""
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
            battery_power_sensor=args.get(
                "battery_power_sensor", cls.battery_power_sensor
            ),
            energy_window_s=float(
                args.get("energy_window_s", cls.energy_window_s)
            ),
            energy_ratio_thresh=float(
                args.get("energy_ratio_thresh", cls.energy_ratio_thresh)
            ),
            sensor_prefix=args.get("sensor_prefix", cls.sensor_prefix),
            debug=bool(args.get("debug", cls.debug)),
            log_dir=str(args.get("log_dir", cls.log_dir)),
        )


# ═══════════════════════════════════════════════════════════
#  Constants — energy integration
# ═══════════════════════════════════════════════════════════

MIN_DISCHARGE_WS = 1.0
"""Minimum battery discharge energy (W·s) to avoid division by zero."""

DETECTOR_CSV_COLUMNS = [
    "grid_w",
    "battery_w",
    "grid_export_ws",
    "battery_discharge_ws",
    "energy_ratio",
    "flip_count",
    "active",
]
"""Column names for the detector CSV log."""


# ═══════════════════════════════════════════════════════════
#  Sign-Flip Detector (pure logic)
# ═══════════════════════════════════════════════════════════


class SignFlipDetector:
    """Detects futile controller response via grid power sign flips.

    A sign flip is registered when, within ``flip_window_s``, both a
    sample with ``grid > +flip_thresh_w`` and one with
    ``grid < -flip_thresh_w`` are observed.

    Activation requires:
      - ``>= activate_count`` flips within ``activate_window_s``, **and**
      - (if battery sensor configured) ``energy_ratio >= energy_ratio_thresh``
        within ``energy_window_s``.

    Deactivation: no ``|grid| > flip_thresh_w`` for ``deactivate_quiet_s``.

    Energy ratio = grid_export_energy / battery_discharge_energy, computed
    via trapezoidal integration over a sliding window.  A high ratio means
    a large fraction of battery discharge is being wasted as grid export
    (the hallmark of futile controller chasing).
    """

    def __init__(self, cfg: DetectorConfig) -> None:
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

        # Energy tracking
        self._energy_samples: deque[tuple[float, float, float]] = deque()
        """``(timestamp, grid_w, battery_w)`` samples for energy integration."""
        self._has_energy: bool = cfg.battery_power_sensor is not None
        """Whether energy-ratio check is enabled."""
        self.grid_export_ws: float = 0.0
        """Cumulative grid export energy in the window (W·s)."""
        self.battery_discharge_ws: float = 0.0
        """Cumulative battery discharge energy in the window (W·s)."""

    @property
    def flip_count(self) -> int:
        """Number of sign flips in the current activation window."""
        return len(self._flip_times)

    @property
    def energy_ratio(self) -> float:
        """Grid export / battery discharge energy ratio (0–1+).

        Returns 0.0 when battery discharge is negligible.
        """
        if self.battery_discharge_ws < MIN_DISCHARGE_WS:
            return 0.0
        return self.grid_export_ws / self.battery_discharge_ws

    def _update_energy(self, grid_w: float, battery_w: float, now: float) -> None:
        """Add a sample and recompute energy integrals over the window.

        Uses trapezoidal integration for accuracy with irregularly-spaced
        samples.  Only the positive parts contribute: ``max(0, -grid_w)``
        for export, ``max(0, battery_w)`` for discharge.
        """
        self._energy_samples.append((now, grid_w, battery_w))

        # Prune samples outside the energy window
        cutoff = now - self._cfg.energy_window_s
        while self._energy_samples and self._energy_samples[0][0] < cutoff:
            self._energy_samples.popleft()

        # Recompute integrals via trapezoidal rule
        export_ws = 0.0
        discharge_ws = 0.0
        samples = self._energy_samples
        for i in range(1, len(samples)):
            t0, g0, b0 = samples[i - 1]
            t1, g1, b1 = samples[i]
            dt = t1 - t0
            if dt <= 0:
                continue
            # Trapezoidal: average of endpoints × dt
            export_ws += 0.5 * (max(0.0, -g0) + max(0.0, -g1)) * dt
            discharge_ws += 0.5 * (max(0.0, b0) + max(0.0, b1)) * dt

        self.grid_export_ws = export_ws
        self.battery_discharge_ws = discharge_ws

    def update(
        self, grid_w: float, now: float, battery_w: float | None = None
    ) -> bool:
        """Process one grid sample, return whether detector is active.

        Args:
            grid_w: Current grid power reading (W).  Positive = import.
            now: Monotonic timestamp (seconds).
            battery_w: Current battery power (W).  +discharge / -charge.
                       ``None`` disables energy-ratio check for this sample.

        Returns:
            ``True`` if a pulse load is detected.
        """
        cfg = self._cfg

        # Energy tracking (when battery sensor available)
        if battery_w is not None and self._has_energy:
            self._update_energy(grid_w, battery_w, now)

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
            flips_ok = len(self._flip_times) >= cfg.activate_count
            energy_ok = (
                self.energy_ratio >= cfg.energy_ratio_thresh
                if self._has_energy
                else True  # no battery sensor → flip-only
            )
            if flips_ok and energy_ok:
                self.active = True
        else:
            quiet_duration = now - self._last_large_t
            if self._last_large_t > 0 and quiet_duration > cfg.deactivate_quiet_s:
                self.active = False
                self._flip_times.clear()
                self._last_flip_check_t = -cfg.flip_window_s - 1.0

        return self.active


# ═══════════════════════════════════════════════════════════
#  AppDaemon HA Adapter
# ═══════════════════════════════════════════════════════════


class PulseLoadDetector(_HASS_BASE):
    """AppDaemon app: detects periodic pulse loads on the grid sensor.

    Subscribes to the raw grid sensor (and optionally a battery power sensor),
    runs ``SignFlipDetector``, and publishes the detection state as
    ``sensor.zfi_pld_active``.
    """

    cfg: DetectorConfig
    detector: SignFlipDetector
    _last_battery_w: float | None
    _csv: CsvLogger | None

    def initialize(self) -> None:
        """Called by AppDaemon on startup."""
        self.cfg = DetectorConfig.from_args(self.args)
        self.detector = SignFlipDetector(self.cfg)
        self._monotonic_offset = time.monotonic()
        self._last_battery_w: float | None = None

        # CSV logging
        self._csv: CsvLogger | None = None
        if self.cfg.log_dir:
            self._csv = CsvLogger(
                self.cfg.log_dir, "zfi_pld", DETECTOR_CSV_COLUMNS
            )
            self.log("CSV logging to %s/zfi_pld_*.csv", self.cfg.log_dir)

        self.listen_state(
            self._on_grid_change,
            self.cfg.grid_sensor,
        )

        # Optional battery power subscription
        if self.cfg.battery_power_sensor:
            self.listen_state(
                self._on_battery_change,
                self.cfg.battery_power_sensor,
            )

        self.log(
            "PulseLoadDetector initialized | battery=%s energy_window=%.0fs "
            "energy_ratio_thresh=%.2f",
            self.cfg.battery_power_sensor or "disabled",
            self.cfg.energy_window_s,
            self.cfg.energy_ratio_thresh,
        )

    def _on_battery_change(
        self,
        entity: str,
        attribute: str,
        old: str,
        new: str,
        kwargs: dict[str, Any],
    ) -> None:
        """Store latest battery power reading for use in grid callback."""
        if new in UNAVAILABLE_STATES:
            self._last_battery_w = None
            return
        try:
            self._last_battery_w = float(new)
        except (ValueError, TypeError):
            self._last_battery_w = None

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
        was_active = self.detector.active

        self.detector.update(grid_w, now, self._last_battery_w)

        # Publish sensors
        self._set_sensor("active", int(self.detector.active))
        if self.cfg.debug:
            self._set_sensor("flip_count", self.detector.flip_count)
            if self.detector._has_energy:
                self._set_sensor(
                    "energy_ratio", round(self.detector.energy_ratio, 3)
                )

        # CSV logging
        if self._csv is not None:
            self._csv.log_row({
                "grid_w": round(grid_w),
                "battery_w": (
                    round(self._last_battery_w)
                    if self._last_battery_w is not None
                    else ""
                ),
                "grid_export_ws": round(self.detector.grid_export_ws, 1),
                "battery_discharge_ws": round(
                    self.detector.battery_discharge_ws, 1
                ),
                "energy_ratio": round(self.detector.energy_ratio, 3),
                "flip_count": self.detector.flip_count,
                "active": int(self.detector.active),
            })

        if self.detector.active and not was_active:
            self.log("Pulse load detected — activated")
        elif not self.detector.active and was_active:
            self.log("Pulse load ended — deactivated")

    def _set_sensor(
        self,
        name: str,
        value: object,
        unit: str | None = None,
    ) -> None:
        """Create or update a ``{sensor_prefix}_{name}`` entity in HA."""
        entity_id = f"{self.cfg.sensor_prefix}_{name}"
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
