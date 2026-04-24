"""Tests for pulse_load_detector — SignFlipDetector and DetectorConfig.

All tests exercise pure computation classes (no HA/AppDaemon mocking needed).
"""

from __future__ import annotations

from src.pulse_load_detector import (
    DetectorConfig,
    SignFlipDetector,
)


# ═══════════════════════════════════════════════════════════
#  Helpers
# ═══════════════════════════════════════════════════════════


def make_config(**overrides: object) -> DetectorConfig:
    """Create a DetectorConfig with sensible test defaults."""
    defaults: dict[str, object] = dict(
        grid_sensor="sensor.grid",
    )
    defaults.update(overrides)
    return DetectorConfig(**defaults)  # type: ignore[arg-type]


# ═══════════════════════════════════════════════════════════
#  DetectorConfig
# ═══════════════════════════════════════════════════════════


class TestDetectorConfig:
    """Test DetectorConfig defaults and from_args mapping."""

    def test_defaults(self) -> None:
        """Default values match the design document."""
        cfg = make_config()
        assert cfg.flip_thresh_w == 500.0
        assert cfg.flip_window_s == 20.0
        assert cfg.activate_count == 2
        assert cfg.activate_window_s == 120.0
        assert cfg.deactivate_quiet_s == 120.0

    def test_from_args_minimal(self) -> None:
        """Only required key produces valid config with defaults."""
        args = {"grid_power_sensor": "sensor.grid"}
        cfg = DetectorConfig.from_args(args)
        assert cfg.grid_sensor == "sensor.grid"
        assert cfg.flip_thresh_w == 500.0

    def test_from_args_overrides(self) -> None:
        """Custom values in args override defaults."""
        args = {
            "grid_power_sensor": "sensor.grid",
            "flip_thresh_w": 400,
            "deactivate_quiet_s": 90,
        }
        cfg = DetectorConfig.from_args(args)
        assert cfg.flip_thresh_w == 400.0
        assert cfg.deactivate_quiet_s == 90.0


# ═══════════════════════════════════════════════════════════
#  SignFlipDetector
# ═══════════════════════════════════════════════════════════


class TestSignFlipDetector:
    """Test sign-flip detection logic."""

    def test_no_flips_stays_inactive(self) -> None:
        """Small grid values never activate the detector."""
        cfg = make_config()
        det = SignFlipDetector(cfg)
        for i in range(100):
            det.update(50.0, float(i * 5))
        assert not det.active
        assert det.flip_count == 0

    def test_single_flip_not_enough(self) -> None:
        """One sign flip does not activate (need >= 2)."""
        cfg = make_config()
        det = SignFlipDetector(cfg)
        # Create one sign flip: +600 then -600 within 20s
        det.update(600.0, 0.0)
        det.update(-600.0, 10.0)
        assert det.flip_count == 1
        assert not det.active

    def test_two_flips_activate(self) -> None:
        """Two sign flips within the activation window activate the detector."""
        cfg = make_config()
        det = SignFlipDetector(cfg)
        # First flip
        det.update(600.0, 0.0)
        det.update(-600.0, 10.0)
        # Quiet gap (> flip_window_s so a new flip can register)
        det.update(0.0, 35.0)
        # Second flip
        det.update(700.0, 60.0)
        det.update(-700.0, 70.0)
        assert det.flip_count == 2
        assert det.active

    def test_deactivate_after_quiet(self) -> None:
        """Detector deactivates when no large grid values for deactivate_quiet_s."""
        cfg = make_config(deactivate_quiet_s=60.0)
        det = SignFlipDetector(cfg)
        # Activate
        det.update(600.0, 0.0)
        det.update(-600.0, 10.0)
        det.update(0.0, 35.0)
        det.update(600.0, 60.0)
        det.update(-600.0, 70.0)
        assert det.active

        # Stay quiet for deactivate_quiet_s — small values only
        t = 70.0
        while t < 70.0 + 65.0:
            t += 5.0
            det.update(30.0, t)
        assert not det.active

    def test_large_grid_resets_quiet_timer(self) -> None:
        """A large grid reading resets the deactivation quiet timer."""
        cfg = make_config(deactivate_quiet_s=60.0)
        det = SignFlipDetector(cfg)
        # Activate
        det.update(600.0, 0.0)
        det.update(-600.0, 10.0)
        det.update(0.0, 35.0)
        det.update(600.0, 60.0)
        det.update(-600.0, 70.0)
        assert det.active

        # Quiet for 50s, then one large spike
        for i in range(10):
            det.update(30.0, 75.0 + i * 5)
        det.update(800.0, 125.0)  # resets quiet timer
        # Another 55s of quiet — still under 60s since spike
        for i in range(11):
            det.update(30.0, 130.0 + i * 5)
        assert det.active  # should still be active

    def test_step_load_no_flip(self) -> None:
        """A step load (positive only) does not register as a flip."""
        cfg = make_config()
        det = SignFlipDetector(cfg)
        # Step from low to high — never negative
        for i in range(30):
            det.update(50.0, float(i * 5))
        det.update(2000.0, 150.0)
        det.update(1500.0, 155.0)
        det.update(500.0, 160.0)
        assert det.flip_count == 0
        assert not det.active
