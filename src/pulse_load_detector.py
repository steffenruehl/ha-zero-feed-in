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
  - ``active``     — 1 when pulse load detected, 0 otherwise
  - ``flip_count`` — sign flips in the activation window (debug)
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

    # Output
    sensor_prefix: str = DEFAULT_SENSOR_PREFIX
    """Prefix for published HA entities."""
    debug: bool = False
    """Publish extra debug sensors (flip_count)."""

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
            sensor_prefix=args.get("sensor_prefix", cls.sensor_prefix),
            debug=bool(args.get("debug", cls.debug)),
        )


# ═══════════════════════════════════════════════════════════
#  Sign-Flip Detector (pure logic)
# ═══════════════════════════════════════════════════════════


class SignFlipDetector:
    """Detects futile controller response via grid power sign flips.

    A sign flip is registered when, within ``flip_window_s``, both a
    sample with ``grid > +flip_thresh_w`` and one with
    ``grid < -flip_thresh_w`` are observed.

    Activation: ``>= activate_count`` flips within ``activate_window_s``.
    Deactivation: no ``|grid| > flip_thresh_w`` for ``deactivate_quiet_s``.
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
            ``True`` if a pulse load is detected.
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
                self._last_flip_check_t = -cfg.flip_window_s - 1.0

        return self.active


# ═══════════════════════════════════════════════════════════
#  AppDaemon HA Adapter
# ═══════════════════════════════════════════════════════════


class PulseLoadDetector(_HASS_BASE):
    """AppDaemon app: detects periodic pulse loads on the grid sensor.

    Subscribes to the raw grid sensor, runs ``SignFlipDetector``, and
    publishes the detection state as ``sensor.zfi_pld_active``.
    """

    cfg: DetectorConfig
    detector: SignFlipDetector

    def initialize(self) -> None:
        """Called by AppDaemon on startup."""
        self.cfg = DetectorConfig.from_args(self.args)
        self.detector = SignFlipDetector(self.cfg)
        self._monotonic_offset = time.monotonic()

        self.listen_state(
            self._on_grid_change,
            self.cfg.grid_sensor,
        )
        self.log("PulseLoadDetector initialized")

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

        self.detector.update(grid_w, now)

        self._set_sensor("active", int(self.detector.active))
        if self.cfg.debug:
            self._set_sensor("flip_count", self.detector.flip_count)

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
