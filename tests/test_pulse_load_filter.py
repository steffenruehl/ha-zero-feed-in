"""Tests for pulse_load_filter — SignFlipDetector, BaselineEstimator, PulseLoadFilterLogic.

All tests exercise pure computation classes (no HA/AppDaemon mocking needed).
"""

from __future__ import annotations

import pytest

from src.pulse_load_filter import (
    BaselineEstimator,
    FilterConfig,
    PulseLoadFilterLogic,
    SignFlipDetector,
)


# ═══════════════════════════════════════════════════════════
#  Helpers
# ═══════════════════════════════════════════════════════════


def make_config(**overrides: object) -> FilterConfig:
    """Create a FilterConfig with sensible test defaults."""
    defaults: dict[str, object] = dict(
        grid_sensor="sensor.grid",
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
        assert cfg.flip_thresh_w == 500.0
        assert cfg.flip_window_s == 20.0
        assert cfg.activate_count == 2
        assert cfg.activate_window_s == 120.0
        assert cfg.deactivate_quiet_s == 120.0
        assert cfg.measurement_duration_s == 60.0
        assert cfg.drift_target_w == 30.0
        assert cfg.drift_ki == 0.3
        assert cfg.drift_cycle_s == 60.0

    def test_from_args_minimal(self) -> None:
        """Only required key produces valid config with defaults."""
        args = {"grid_power_sensor": "sensor.grid"}
        cfg = FilterConfig.from_args(args)
        assert cfg.grid_sensor == "sensor.grid"
        assert cfg.flip_thresh_w == 500.0
        assert cfg.filtered_power_entity == "sensor.zfi_plf_filtered_grid_power"
        assert cfg.dry_run is True

    def test_from_args_overrides(self) -> None:
        """Custom values in args override defaults."""
        args = {
            "grid_power_sensor": "sensor.grid",
            "flip_thresh_w": 400,
            "drift_ki": 0.5,
            "filtered_power_entity": "sensor.custom_output",
            "dry_run": False,
        }
        cfg = FilterConfig.from_args(args)
        assert cfg.flip_thresh_w == 400.0
        assert cfg.drift_ki == 0.5
        assert cfg.filtered_power_entity == "sensor.custom_output"
        assert cfg.dry_run is False


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
#  BaselineEstimator
# ═══════════════════════════════════════════════════════════


class TestBaselineEstimator:
    """Test measurement pause and I-controller drift tracking."""

    def test_measurement_captures_minimum(self) -> None:
        """Measurement pause sets baseline to min(grid) over the window."""
        cfg = make_config(measurement_duration_s=30.0)
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
            measurement_duration_s=5.0,
            drift_target_w=30.0,
            drift_ki=0.3,
            drift_cycle_s=10.0,
        )
        est = BaselineEstimator(cfg)
        est.start_measurement(0.0)
        est.update(80.0, 5.0)  # measurement with higher value → baseline=80
        assert est.baseline == 80.0

        # House load drops to 30W (still above 0)
        # With battery active: battery_cmd = baseline - target = 50W
        # grid_off = 30 - 50 = -20 → min(grid) = -20
        # But we simulate the actual grid observation: min(grid) = 10
        for cycle in range(10):
            t_base = 10.0 + cycle * 10.0
            est.update(10.0, t_base + 5.0)
            est.update(10.0, t_base + 10.0)

        # Baseline should have decreased
        assert est.baseline is not None
        assert est.baseline < 60.0

    def test_reset_clears_state(self) -> None:
        """Reset returns estimator to initial state."""
        cfg = make_config(measurement_duration_s=5.0)
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
    """Integration tests for the full filter pipeline."""

    def _activate(self, logic: PulseLoadFilterLogic, t: float) -> float:
        """Drive the logic through activation with two sign flips.

        Returns the timestamp after activation.
        """
        # First flip
        logic.update(600.0, t)
        logic.update(-600.0, t + 10.0)
        # Gap
        logic.update(0.0, t + 35.0)
        # Second flip
        logic.update(700.0, t + 60.0)
        logic.update(-700.0, t + 70.0)
        return t + 70.0

    def test_passthrough_when_inactive(self) -> None:
        """When no pulse load, output equals raw grid value."""
        cfg = make_config()
        logic = PulseLoadFilterLogic(cfg)
        for val in [50.0, -20.0, 300.0, 0.0]:
            assert logic.update(val, 0.0) == val
        assert not logic.active

    def test_activation_starts_measurement(self) -> None:
        """Activation triggers measurement pause."""
        cfg = make_config()
        logic = PulseLoadFilterLogic(cfg)
        self._activate(logic, 0.0)
        assert logic.active
        assert logic.measuring

    def test_measurement_produces_baseline(self) -> None:
        """After measurement, filter outputs the baseline."""
        cfg = make_config(measurement_duration_s=30.0)
        logic = PulseLoadFilterLogic(cfg)
        t = self._activate(logic, 0.0)

        # Feed measurement samples (oven cycling, battery paused)
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
            logic.update(grid_w, ts)

        assert not logic.measuring
        assert logic.baseline == 30.0

        # Now filter should output baseline, not raw
        result = logic.update(1300.0, t + 40)
        assert result == pytest.approx(30.0, abs=1.0)

    def test_deactivation_resumes_passthrough(self) -> None:
        """After deactivation, filter passes raw value through."""
        cfg = make_config(
            measurement_duration_s=10.0,
            deactivate_quiet_s=30.0,
        )
        logic = PulseLoadFilterLogic(cfg)
        t = self._activate(logic, 0.0)

        # Complete measurement
        logic.update(30.0, t + 5)
        logic.update(30.0, t + 15)
        assert not logic.measuring
        assert logic.active

        # Quiet period → deactivate
        t_quiet = t + 15
        for i in range(10):
            t_quiet += 5
            logic.update(25.0, t_quiet)

        assert not logic.active
        # Passthrough resumes
        assert logic.update(123.0, t_quiet + 5) == 123.0
        assert logic.baseline is None  # reset

    def test_full_oven_cycle_simulation(self) -> None:
        """Simulate a realistic oven scenario end-to-end.

        1. Normal operation (low grid)
        2. Oven turns on, controller chases, sign flips appear
        3. Filter activates → battery paused → measurement with true load
        4. Filter outputs stable baseline through cycling
        5. Oven stops, no large swings, filter deactivates
        """
        cfg = make_config(
            measurement_duration_s=60.0,
            deactivate_quiet_s=60.0,
            drift_cycle_s=60.0,
        )
        logic = PulseLoadFilterLogic(cfg)
        dt = 5.0
        t = 0.0
        oven_on_samples = 3   # 15s at 5s intervals
        oven_off_samples = 9  # 45s at 5s intervals

        # Phase 1: Normal operation (30W house load, battery idle)
        for _ in range(20):
            out = logic.update(30.0, t)
            assert out == 30.0
            t += dt
        assert not logic.active

        # Phase 2: Oven cycling — controller chases, sign flips
        # Grid oscillates between +1300 and -1200 as battery chases.
        # We need 2 sign flips to activate.  Keep feeding until active.
        activated = False
        while not activated:
            for _ in range(oven_on_samples):
                logic.update(1300.0, t)
                t += dt
                if logic.active:
                    activated = True
                    break
            if activated:
                break
            for _ in range(oven_off_samples):
                logic.update(-1200.0, t)
                t += dt
                if logic.active:
                    activated = True
                    break
        assert logic.active
        assert logic.measuring

        # Phase 3: Battery is now paused (HA adapter sends pause on activation).
        # Grid shows true house load during oven OFF, house+oven during ON.
        # The measurement window captures these realistic values.
        for cycle in range(2):
            for _ in range(oven_on_samples):
                logic.update(1330.0, t)  # house(30) + oven(1300)
                t += dt
            for _ in range(oven_off_samples):
                logic.update(30.0, t)    # just house load
                t += dt

        # Measurement should be complete, baseline ≈ 30
        assert not logic.measuring
        assert logic.baseline is not None
        assert logic.baseline == pytest.approx(30.0, abs=5.0)

        # Phase 4: Ongoing cycling — filter outputs baseline
        # In production, battery is now set to (baseline - target).
        # Grid still shows cycling from the oven.
        for cycle in range(3):
            for _ in range(oven_on_samples):
                out = logic.update(1300.0, t)
                # Output should be near baseline, not 1300
                assert out < 100.0
                t += dt
            for _ in range(oven_off_samples):
                out = logic.update(-1200.0, t)
                # Output should be near baseline, not -1200
                assert out < 100.0
                t += dt

        # Phase 5: Oven stops, small grid values
        for _ in range(20):
            logic.update(30.0, t)
            t += dt

        assert not logic.active
        # Back to passthrough
        assert logic.update(42.0, t) == 42.0
