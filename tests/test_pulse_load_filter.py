"""Tests for pulse_load_filter — BaselineEstimator, PulseLoadFilterLogic, FilterConfig.

All tests exercise pure computation classes (no HA/AppDaemon mocking needed).
The filter now receives its active state externally (from the detector).
"""

from __future__ import annotations

import pytest

from src.pulse_load_filter import (
    BaselineEstimator,
    FilterConfig,
    PulseLoadFilterLogic,
)


# ═══════════════════════════════════════════════════════════
#  Helpers
# ═══════════════════════════════════════════════════════════


def make_config(**overrides: object) -> FilterConfig:
    """Create a FilterConfig with sensible test defaults."""
    defaults: dict[str, object] = dict(
        grid_sensor="sensor.grid",
        active_entity="sensor.pld_active",
    )
    defaults.update(overrides)
    return FilterConfig(**defaults)  # type: ignore[arg-type]


# ═══════════════════════════════════════════════════════════
#  FilterConfig
# ═══════════════════════════════════════════════════════════


class TestFilterConfig:
    """Test FilterConfig defaults and from_args mapping."""

    def test_defaults(self) -> None:
        """Default values match the design document."""
        cfg = make_config()
        assert cfg.settle_duration_s == 15.0
        assert cfg.measurement_duration_s == 60.0
        assert cfg.drift_target_w == 30.0
        assert cfg.drift_ki == 0.3
        assert cfg.drift_cycle_s == 60.0

    def test_from_args_minimal(self) -> None:
        """Only required keys produce valid config with defaults."""
        args = {
            "grid_power_sensor": "sensor.grid",
            "active_entity": "sensor.pld_active",
        }
        cfg = FilterConfig.from_args(args)
        assert cfg.grid_sensor == "sensor.grid"
        assert cfg.active_entity == "sensor.pld_active"
        assert cfg.filtered_power_entity == "sensor.zfi_plf_filtered_grid_power"
        assert cfg.dry_run is True

    def test_from_args_overrides(self) -> None:
        """Custom values in args override defaults."""
        args = {
            "grid_power_sensor": "sensor.grid",
            "active_entity": "sensor.pld_active",
            "drift_ki": 0.5,
            "filtered_power_entity": "sensor.custom_output",
            "desired_power_entity": "sensor.custom_desired",
            "dry_run": False,
        }
        cfg = FilterConfig.from_args(args)
        assert cfg.drift_ki == 0.5
        assert cfg.filtered_power_entity == "sensor.custom_output"
        assert cfg.desired_power_entity == "sensor.custom_desired"
        assert cfg.dry_run is False

    def test_desired_power_entity_default(self) -> None:
        """desired_power_entity defaults to sensor.zfi_desired_power."""
        cfg = make_config()
        assert cfg.desired_power_entity == "sensor.zfi_desired_power"


# ═══════════════════════════════════════════════════════════
#  BaselineEstimator
# ═══════════════════════════════════════════════════════════


class TestBaselineEstimator:
    """Test measurement pause and I-controller drift tracking."""

    def test_measurement_captures_minimum(self) -> None:
        """Measurement pause sets baseline to min(grid) over the window."""
        cfg = make_config(settle_duration_s=0.0, measurement_duration_s=30.0)
        est = BaselineEstimator(cfg)
        est.start_measurement(0.0)
        assert est.measuring

        # Simulate oven OFF (30W) and ON (1330W) samples
        samples = [1330, 1330, 30, 30, 30, 30, 30]
        for i, g in enumerate(samples):
            est.update(float(g), float(i * 5))

        # Not done yet (only 30s elapsed at sample 6)
        est.update(30.0, 35.0)
        assert not est.measuring
        assert est.baseline == 30.0

    def test_drift_corrects_upward(self) -> None:
        """I-controller increases baseline when house load increases."""
        cfg = make_config(
            settle_duration_s=0.0,
            measurement_duration_s=5.0,
            drift_target_w=30.0,
            drift_ki=0.3,
            drift_cycle_s=10.0,
        )
        est = BaselineEstimator(cfg)
        est.start_measurement(0.0)
        est.update(30.0, 5.0)  # measurement completes → baseline=30
        assert est.baseline == 30.0

        # Now house load increases to 80W → min(grid) = 80 each cycle
        for cycle in range(5):
            t_base = 10.0 + cycle * 10.0
            for i in range(2):
                est.update(80.0, t_base + i * 5)
            # Trigger cycle end
            est.update(80.0, t_base + 10.0)

        # Baseline should have increased toward 80
        assert est.baseline is not None
        assert est.baseline > 45.0  # significant upward correction

    def test_drift_corrects_downward(self) -> None:
        """I-controller decreases baseline when house load decreases."""
        cfg = make_config(
            settle_duration_s=0.0,
            measurement_duration_s=5.0,
            drift_target_w=30.0,
            drift_ki=0.3,
            drift_cycle_s=10.0,
        )
        est = BaselineEstimator(cfg)
        est.start_measurement(0.0)
        est.update(80.0, 5.0)  # measurement with higher value → baseline=80
        assert est.baseline == 80.0

        # House load drops — min(grid) = 10 each cycle
        for cycle in range(10):
            t_base = 10.0 + cycle * 10.0
            est.update(10.0, t_base + 5.0)
            est.update(10.0, t_base + 10.0)

        # Baseline should have decreased
        assert est.baseline is not None
        assert est.baseline < 60.0

    def test_reset_clears_state(self) -> None:
        """Reset returns estimator to initial state."""
        cfg = make_config(settle_duration_s=0.0, measurement_duration_s=5.0)
        est = BaselineEstimator(cfg)
        est.start_measurement(0.0)
        est.update(30.0, 5.0)
        assert est.baseline is not None

        est.reset()
        assert est.baseline is None
        assert not est.measuring


# ═══════════════════════════════════════════════════════════
#  PulseLoadFilterLogic (integration)
# ═══════════════════════════════════════════════════════════


class TestPulseLoadFilterLogic:
    """Integration tests for the filter pipeline (externally driven)."""

    def test_passthrough_when_inactive(self) -> None:
        """When detector is inactive, output equals raw grid value."""
        cfg = make_config()
        logic = PulseLoadFilterLogic(cfg)
        for val in [50.0, -20.0, 300.0, 0.0]:
            assert logic.update(val, 0.0, active=False) == val
        assert not logic.active

    def test_activation_starts_measurement(self) -> None:
        """External activation triggers measurement pause, returns 0."""
        cfg = make_config(settle_duration_s=0.0)
        logic = PulseLoadFilterLogic(cfg)
        result = logic.update(600.0, 0.0, active=True)
        assert logic.active
        assert logic.measuring
        assert result == 0.0  # battery paused immediately

    def test_measurement_produces_baseline(self) -> None:
        """After measurement, filter outputs the baseline as desired power."""
        cfg = make_config(settle_duration_s=0.0, measurement_duration_s=30.0)
        logic = PulseLoadFilterLogic(cfg)

        # Activate
        result = logic.update(600.0, 0.0, active=True)
        assert logic.measuring
        assert result == 0.0  # battery paused

        # Feed measurement samples (oven cycling, battery paused)
        t = 0.0
        samples = [
            (1330.0, t + 5),
            (1330.0, t + 10),
            (1330.0, t + 15),
            (30.0, t + 20),
            (30.0, t + 25),
            (30.0, t + 30),
            (30.0, t + 35),
        ]
        for grid_w, ts in samples:
            result = logic.update(grid_w, ts, active=True)

        assert not logic.measuring
        assert logic.baseline == 30.0

        # Now filter should output +baseline (desired discharge power)
        result = logic.update(1300.0, t + 40, active=True)
        assert result == pytest.approx(30.0, abs=1.0)

    def test_returns_zero_during_measurement(self) -> None:
        """During measurement, filter returns 0.0 (battery paused)."""
        cfg = make_config(settle_duration_s=0.0, measurement_duration_s=30.0)
        logic = PulseLoadFilterLogic(cfg)

        # Activate
        logic.update(600.0, 0.0, active=True)

        # Samples during measurement should all return 0.0
        for t in [5.0, 10.0, 15.0, 20.0]:
            result = logic.update(500.0, t, active=True)
            assert result == 0.0, f"Expected 0.0 at t={t}, got {result}"

    def test_deactivation_resumes_passthrough(self) -> None:
        """After deactivation, filter passes raw value through."""
        cfg = make_config(settle_duration_s=0.0, measurement_duration_s=10.0)
        logic = PulseLoadFilterLogic(cfg)

        # Activate and complete measurement
        logic.update(600.0, 0.0, active=True)
        logic.update(30.0, 5.0, active=True)
        logic.update(30.0, 15.0, active=True)
        assert not logic.measuring
        assert logic.active

        # Deactivate
        result = logic.update(25.0, 20.0, active=False)
        assert not logic.active
        assert result == 25.0

        # Passthrough continues
        result = logic.update(100.0, 25.0, active=False)
        assert result == 100.0

    def test_reactivation_remeasures(self) -> None:
        """Re-activation after deactivation starts a fresh measurement."""
        cfg = make_config(settle_duration_s=0.0, measurement_duration_s=10.0)
        logic = PulseLoadFilterLogic(cfg)

        # First activation cycle
        logic.update(600.0, 0.0, active=True)
        logic.update(30.0, 5.0, active=True)
        logic.update(30.0, 15.0, active=True)
        assert logic.baseline == 30.0

        # Deactivate
        logic.update(25.0, 20.0, active=False)
        assert logic.baseline is None

        # Re-activate
        result = logic.update(600.0, 30.0, active=True)
        assert logic.measuring
        assert logic.baseline is None
        assert result == 0.0  # battery paused again


# ═══════════════════════════════════════════════════════════
#  Settle Delay
# ═══════════════════════════════════════════════════════════


class TestSettleDelay:
    """Test that samples during the settle phase are discarded."""

    def test_settle_discards_early_samples(self) -> None:
        """Samples during the settle phase are not used for baseline."""
        cfg = make_config(settle_duration_s=15.0, measurement_duration_s=30.0)
        est = BaselineEstimator(cfg)
        est.start_measurement(0.0)

        # During settle (0–15s): battery still running → corrupted readings
        est.update(-1200.0, 5.0)   # grid negative from battery lag
        est.update(-800.0, 10.0)   # still ramping down

        # After settle (15–45s): battery truly off → clean readings
        est.update(150.0, 20.0)    # oven ON
        est.update(30.0, 25.0)     # oven OFF (baseline)
        est.update(150.0, 30.0)
        est.update(30.0, 35.0)
        est.update(150.0, 40.0)
        est.update(30.0, 45.0)     # triggers completion (elapsed >= 15+30)

        assert not est.measuring
        # Baseline should be min of POST-settle samples only (30W)
        assert est.baseline == 30.0

    def test_settle_corrupted_not_in_baseline(self) -> None:
        """Negative readings during settle are excluded from min()."""
        cfg = make_config(settle_duration_s=10.0, measurement_duration_s=20.0)
        est = BaselineEstimator(cfg)
        est.start_measurement(0.0)

        # Settle phase: wildly negative from battery lag
        est.update(-1500.0, 3.0)
        est.update(-600.0, 7.0)

        # Measurement phase: real house load
        est.update(180.0, 12.0)
        est.update(50.0, 18.0)
        est.update(180.0, 24.0)
        est.update(50.0, 30.0)  # triggers completion

        assert est.baseline == 50.0  # not -1500!

    def test_settle_zero_is_backward_compatible(self) -> None:
        """With settle_duration_s=0, all samples are collected (old behavior)."""
        cfg = make_config(settle_duration_s=0.0, measurement_duration_s=10.0)
        est = BaselineEstimator(cfg)
        est.start_measurement(0.0)

        est.update(100.0, 3.0)
        est.update(50.0, 7.0)
        est.update(80.0, 10.0)  # triggers completion

        assert est.baseline == 50.0

    def test_settle_no_samples_uses_target(self) -> None:
        """If settle is longer than total pause, fallback to drift_target_w."""
        cfg = make_config(
            settle_duration_s=20.0,
            measurement_duration_s=5.0,
            drift_target_w=30.0,
        )
        est = BaselineEstimator(cfg)
        est.start_measurement(0.0)

        # All samples are during settle (before 20s)
        est.update(-500.0, 5.0)
        est.update(-300.0, 10.0)
        est.update(-100.0, 15.0)
        # Trigger at 25s (settle=20 + measurement=5)
        est.update(100.0, 25.0)

        assert not est.measuring
        # Only the last sample at t=25 was collected (25 >= 20)
        assert est.baseline == 100.0

    def test_filter_logic_settle_returns_zero(self) -> None:
        """During settle phase, filter logic returns 0.0 (battery paused)."""
        cfg = make_config(settle_duration_s=15.0, measurement_duration_s=30.0)
        logic = PulseLoadFilterLogic(cfg)

        # Activate
        result = logic.update(600.0, 0.0, active=True)
        assert result == 0.0

        # During settle: still returns 0 (no baseline yet)
        result = logic.update(-1200.0, 5.0, active=True)
        assert result == 0.0
        assert logic.measuring
        assert logic.baseline is None

        result = logic.update(-800.0, 10.0, active=True)
        assert result == 0.0

    def test_filter_logic_full_cycle_with_settle(self) -> None:
        """Full cycle: settle → measurement → baseline output → deactivate."""
        cfg = make_config(settle_duration_s=10.0, measurement_duration_s=20.0)
        logic = PulseLoadFilterLogic(cfg)

        # 1. Activate at t=0
        result = logic.update(600.0, 0.0, active=True)
        assert result == 0.0
        assert logic.measuring

        # 2. Settle phase (t=0..10): corrupted samples discarded
        logic.update(-1200.0, 3.0, active=True)
        logic.update(-500.0, 7.0, active=True)
        assert logic.measuring
        assert logic.baseline is None

        # 3. Measurement phase (t=10..30): clean samples collected
        logic.update(150.0, 12.0, active=True)
        logic.update(40.0, 18.0, active=True)
        logic.update(150.0, 24.0, active=True)
        result = logic.update(40.0, 30.0, active=True)  # completes

        assert not logic.measuring
        assert logic.baseline == 40.0
        assert result == pytest.approx(40.0)

        # 4. Filter outputs baseline while active
        result = logic.update(1300.0, 35.0, active=True)
        assert result == pytest.approx(40.0, abs=5.0)  # drift may shift slightly

        # 5. Deactivate → passthrough
        result = logic.update(100.0, 40.0, active=False)
        assert result == 100.0

    def test_from_args_settle_duration(self) -> None:
        """settle_duration_s is correctly parsed from args."""
        args = {
            "grid_power_sensor": "sensor.grid",
            "active_entity": "sensor.pld_active",
            "settle_duration_s": 20,
        }
        cfg = FilterConfig.from_args(args)
        assert cfg.settle_duration_s == 20.0

    def test_from_args_settle_default(self) -> None:
        """settle_duration_s defaults to 15.0 when not in args."""
        args = {
            "grid_power_sensor": "sensor.grid",
            "active_entity": "sensor.pld_active",
        }
        cfg = FilterConfig.from_args(args)
        assert cfg.settle_duration_s == 15.0
