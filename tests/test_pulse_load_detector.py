"""Tests for pulse_load_detector — SignFlipDetector and DetectorConfig.

All tests exercise pure computation classes (no HA/AppDaemon mocking needed).
"""

from __future__ import annotations

from src.pulse_load_detector import (
    DETECTOR_CSV_COLUMNS,
    MIN_DISCHARGE_WS,
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


# ═══════════════════════════════════════════════════════════
#  DetectorConfig — energy fields
# ═══════════════════════════════════════════════════════════


class TestDetectorConfigEnergy:
    """Test new energy-ratio and CSV config fields."""

    def test_energy_defaults(self) -> None:
        """Energy fields default to disabled / sensible values."""
        cfg = make_config()
        assert cfg.battery_power_sensor is None
        assert cfg.energy_window_s == 120.0
        assert cfg.energy_ratio_thresh == 0.3
        assert cfg.log_dir == ""

    def test_from_args_energy(self) -> None:
        """Energy fields are read from args dict."""
        args = {
            "grid_power_sensor": "sensor.grid",
            "battery_power_sensor": "sensor.battery",
            "energy_window_s": 90,
            "energy_ratio_thresh": 0.4,
            "log_dir": "/tmp/logs",
        }
        cfg = DetectorConfig.from_args(args)
        assert cfg.battery_power_sensor == "sensor.battery"
        assert cfg.energy_window_s == 90.0
        assert cfg.energy_ratio_thresh == 0.4
        assert cfg.log_dir == "/tmp/logs"

    def test_from_args_no_battery(self) -> None:
        """Missing battery_power_sensor defaults to None."""
        args = {"grid_power_sensor": "sensor.grid"}
        cfg = DetectorConfig.from_args(args)
        assert cfg.battery_power_sensor is None


# ═══════════════════════════════════════════════════════════
#  Energy Integration
# ═══════════════════════════════════════════════════════════


def _energy_config(**overrides: object) -> DetectorConfig:
    """Config with battery sensor enabled for energy tests."""
    defaults: dict[str, object] = dict(
        grid_sensor="sensor.grid",
        battery_power_sensor="sensor.battery",
        energy_window_s=60.0,
        energy_ratio_thresh=0.3,
    )
    defaults.update(overrides)
    return DetectorConfig(**defaults)  # type: ignore[arg-type]


class TestEnergyIntegration:
    """Test trapezoidal energy integration."""

    def test_constant_export_and_discharge(self) -> None:
        """Constant grid export and battery discharge integrate correctly.

        Grid = -1000 W (export), battery = +1000 W (discharge) for 10s
        → export = 10000 W·s, discharge = 10000 W·s, ratio = 1.0.
        """
        cfg = _energy_config()
        det = SignFlipDetector(cfg)
        for t in range(11):
            det.update(-1000.0, float(t), battery_w=1000.0)
        assert abs(det.grid_export_ws - 10000.0) < 1.0
        assert abs(det.battery_discharge_ws - 10000.0) < 1.0
        assert abs(det.energy_ratio - 1.0) < 0.01

    def test_import_only_no_export(self) -> None:
        """Positive grid (import) produces zero export energy."""
        cfg = _energy_config()
        det = SignFlipDetector(cfg)
        for t in range(11):
            det.update(500.0, float(t), battery_w=0.0)
        assert det.grid_export_ws == 0.0
        assert det.energy_ratio == 0.0

    def test_no_discharge_ratio_zero(self) -> None:
        """Zero battery discharge → ratio = 0 (avoids division by zero)."""
        cfg = _energy_config()
        det = SignFlipDetector(cfg)
        for t in range(11):
            det.update(-500.0, float(t), battery_w=-200.0)  # charging
        assert det.grid_export_ws > 0
        assert det.battery_discharge_ws == 0.0
        assert det.energy_ratio == 0.0

    def test_half_wasted(self) -> None:
        """Half the discharge energy is exported → ratio ≈ 0.5.

        Grid alternates: 5s at -500 W (export) then 5s at +500 W (import).
        Battery: constant +500 W discharge for 10s.
        Export = 500 × 5 = 2500 W·s.  Discharge = 500 × 10 = 5000 W·s.
        Ratio = 0.5.
        """
        cfg = _energy_config()
        det = SignFlipDetector(cfg)
        for t in range(6):  # 0–5s: export
            det.update(-500.0, float(t), battery_w=500.0)
        for t in range(6, 11):  # 5–10s: import
            det.update(500.0, float(t), battery_w=500.0)
        # Trapezoidal rule slightly overestimates at the transition point
        # (one trapezoid spans the sign change), so ratio ≈ 0.55 not 0.50.
        assert abs(det.energy_ratio - 0.5) < 0.1

    def test_window_pruning(self) -> None:
        """Samples outside the energy window are dropped."""
        cfg = _energy_config(energy_window_s=10.0)
        det = SignFlipDetector(cfg)
        # 0–10s: big export
        for t in range(11):
            det.update(-1000.0, float(t), battery_w=1000.0)
        assert det.grid_export_ws > 5000.0

        # 10–25s: no export — old samples should be pruned
        for t in range(11, 26):
            det.update(100.0, float(t), battery_w=0.0)
        assert det.grid_export_ws == 0.0
        assert det.battery_discharge_ws == 0.0

    def test_trapezoidal_ramp(self) -> None:
        """Linearly ramping export integrates as a triangle.

        Grid ramps from 0 to -1000 W over 10s.
        Expected export = ½ × 1000 × 10 = 5000 W·s.
        """
        cfg = _energy_config()
        det = SignFlipDetector(cfg)
        for t in range(11):
            grid_w = -100.0 * t  # 0, -100, -200, ..., -1000
            det.update(grid_w, float(t), battery_w=1000.0)
        assert abs(det.grid_export_ws - 5000.0) < 50.0


# ═══════════════════════════════════════════════════════════
#  Energy-Gated Activation
# ═══════════════════════════════════════════════════════════


class TestEnergyGatedActivation:
    """Test that activation requires both flips AND energy ratio."""

    def _two_flips(
        self, det: SignFlipDetector, battery_w: float | None = None
    ) -> None:
        """Helper: inject two sign flips that would activate flip-only mode."""
        det.update(600.0, 0.0, battery_w)
        det.update(-600.0, 10.0, battery_w)
        det.update(0.0, 35.0, battery_w)
        det.update(700.0, 60.0, battery_w)
        det.update(-700.0, 70.0, battery_w)

    def test_flips_only_no_battery(self) -> None:
        """Without battery sensor, flips alone activate (backward compat)."""
        cfg = make_config()  # no battery_power_sensor
        det = SignFlipDetector(cfg)
        self._two_flips(det)
        assert det.active

    def test_flips_without_energy_insufficient(self) -> None:
        """With battery sensor but low energy ratio, flips alone don't activate."""
        cfg = _energy_config(energy_ratio_thresh=0.3)
        det = SignFlipDetector(cfg)
        # Flips with battery charging (no discharge → ratio = 0)
        self._two_flips(det, battery_w=-200.0)
        assert det.flip_count >= 2
        assert det.energy_ratio < 0.3
        assert not det.active

    def test_energy_without_flips_insufficient(self) -> None:
        """High energy ratio but no flips does not activate."""
        cfg = _energy_config(energy_ratio_thresh=0.3)
        det = SignFlipDetector(cfg)
        # Constant export with discharge (high ratio) but no sign flips
        for t in range(60):
            det.update(-800.0, float(t), battery_w=1000.0)
        assert det.energy_ratio > 0.3
        assert det.flip_count == 0
        assert not det.active

    def test_flips_and_energy_activate(self) -> None:
        """Both flips and high energy ratio → activation."""
        cfg = _energy_config(energy_ratio_thresh=0.2)
        det = SignFlipDetector(cfg)
        # Two flips with sustained discharge and alternating grid
        # Export phases: -600 W at t=10, t=70.  Import: +600 W at t=0, t=60.
        det.update(600.0, 0.0, battery_w=800.0)
        det.update(-600.0, 10.0, battery_w=800.0)
        det.update(0.0, 35.0, battery_w=800.0)
        det.update(600.0, 60.0, battery_w=800.0)
        det.update(-600.0, 70.0, battery_w=800.0)
        assert det.flip_count >= 2
        assert det.energy_ratio > 0.2
        assert det.active

    def test_battery_none_falls_back_to_flips_only(self) -> None:
        """When battery_w=None is passed, energy check is skipped."""
        cfg = _energy_config(energy_ratio_thresh=0.3)
        det = SignFlipDetector(cfg)
        # Flips with None battery → energy samples never populated
        self._two_flips(det, battery_w=None)
        # Energy ratio stays 0.0, but _has_energy is True
        # Since no battery_w was ever provided, energy_ratio = 0.0 < 0.3
        assert not det.active  # energy gate blocks activation

    def test_oven_cycle_simulation(self) -> None:
        """Simulate a realistic oven duty cycle: 15s ON / 45s OFF.

        During ON: grid swings to -800 W (export, overreaction).
        During OFF: grid swings to +600 W (import, controller ramps).
        Battery: constant +400 W discharge (controller output).
        """
        cfg = _energy_config(
            flip_thresh_w=400.0,
            activate_count=2,
            activate_window_s=120.0,
            energy_ratio_thresh=0.2,
            energy_window_s=120.0,
        )
        det = SignFlipDetector(cfg)
        t = 0.0
        # Run 3 oven cycles: 15s ON + 45s OFF = 60s each
        for _cycle in range(3):
            # ON phase: oven draws power → grid export (controller overshot)
            for _ in range(15):
                det.update(-800.0, t, battery_w=400.0)
                t += 1.0
            # OFF phase: oven off → grid import (controller chasing)
            for _ in range(45):
                det.update(600.0, t, battery_w=400.0)
                t += 1.0
        assert det.active
        assert det.energy_ratio > 0.2


# ═══════════════════════════════════════════════════════════
#  CSV Columns
# ═══════════════════════════════════════════════════════════


class TestCsvColumns:
    """Test CSV column definition."""

    def test_columns_defined(self) -> None:
        """CSV columns list is non-empty and has expected fields."""
        assert len(DETECTOR_CSV_COLUMNS) == 7
        assert "grid_w" in DETECTOR_CSV_COLUMNS
        assert "energy_ratio" in DETECTOR_CSV_COLUMNS
        assert "active" in DETECTOR_CSV_COLUMNS
