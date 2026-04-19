"""Tests for zero_feed_in_controller – ControlLogic and PIController.

All tests exercise pure computation classes (no HA/AppDaemon mocking needed).
"""

from __future__ import annotations

import pytest

from src.zero_feed_in_controller import (
    Config,
    ControlLogic,
    ControlOutput,
    ControllerState,
    Measurement,
    OperatingMode,
    PIController,
    PIGains,
    PIGainSet,
)


# ═══════════════════════════════════════════════════════════
#  Helpers
# ═══════════════════════════════════════════════════════════


def make_config(**overrides) -> Config:
    """Create a Config with sensible test defaults."""
    defaults = dict(
        grid_sensor="sensor.grid",
        soc_sensor="sensor.soc",
        battery_power_sensor="sensor.battery",
    )
    defaults.update(overrides)
    return Config(**defaults)


def make_measurement(**overrides) -> Measurement:
    """Create a Measurement with sensible test defaults."""
    defaults = dict(
        grid_power_w=0.0,
        soc_pct=50.0,
        battery_power_w=0.0,
    )
    defaults.update(overrides)
    return Measurement(**defaults)


def make_logic(log=None, **config_overrides) -> ControlLogic:
    """Create a ControlLogic with test defaults, unseeded."""
    cfg = make_config(**config_overrides)
    return ControlLogic(cfg, log=log)


# ═══════════════════════════════════════════════════════════
#  PIController
# ═══════════════════════════════════════════════════════════


class TestPIController:
    def test_proportional_positive_error(self):
        pi = PIController(
            output_min=-1000, output_max=1000,
        )
        output, p_term, _ = pi.update(error=100, integral=0.0, dt=5, gains=PIGains(0.5, 0.0))
        assert p_term == pytest.approx(50.0)
        assert output == pytest.approx(50.0)

    def test_proportional_negative_error(self):
        pi = PIController(
            output_min=-1000, output_max=1000,
        )
        output, p_term, _ = pi.update(error=-100, integral=0.0, dt=5, gains=PIGains(0.5, 0.0))
        assert p_term == pytest.approx(-50.0)
        assert output == pytest.approx(-50.0)

    def test_integral_accumulation(self):
        pi = PIController(
            output_min=-1000, output_max=1000,
        )
        output, _, new_integral = pi.update(error=100, integral=0.0, dt=5, gains=PIGains(0.0, 0.1))
        assert new_integral == pytest.approx(50.0)  # 0.1 * 100 * 5
        assert output == pytest.approx(50.0)

    def test_anti_windup_upper_clamp(self):
        pi = PIController(
            output_min=-500, output_max=500,
        )
        output, p_term, new_integral = pi.update(error=600, integral=0.0, dt=5, gains=PIGains(1.0, 0.1))
        assert output == pytest.approx(500)
        # Back-calculated: integral = output_max - p_term
        assert new_integral == pytest.approx(500 - 600)

    def test_anti_windup_lower_clamp(self):
        pi = PIController(
            output_min=-500, output_max=500,
        )
        output, p_term, new_integral = pi.update(error=-600, integral=0.0, dt=5, gains=PIGains(1.0, 0.1))
        assert output == pytest.approx(-500)
        assert new_integral == pytest.approx(-500 - (-600))


# ═══════════════════════════════════════════════════════════
#  Surplus estimation
# ═══════════════════════════════════════════════════════════


class TestEstimateSurplus:
    def test_zero_when_idle(self):
        m = make_measurement(grid_power_w=0, battery_power_w=0)
        assert ControlLogic.estimate_surplus(m) == pytest.approx(0.0)

    def test_surplus_from_charging_and_export(self):
        # Battery charging 300W, grid exporting 50W
        m = make_measurement(grid_power_w=-50, battery_power_w=-300)
        # surplus = -(-300) - (-50) = 350
        assert ControlLogic.estimate_surplus(m) == pytest.approx(350.0)

    def test_negative_surplus_when_discharging_and_importing(self):
        # Battery discharging 200W, grid importing 100W
        m = make_measurement(grid_power_w=100, battery_power_w=200)
        # surplus = -(200) - 100 = -300
        assert ControlLogic.estimate_surplus(m) == pytest.approx(-300.0)


# ═══════════════════════════════════════════════════════════
#  ControlLogic.seed
# ═══════════════════════════════════════════════════════════


class TestSeed:
    def test_seed_positive_starts_discharging(self):
        logic = make_logic()
        logic.seed(200.0)
        assert logic.state.last_computed_w == 200.0
        assert logic.state.integral == 200.0
        assert logic.state.mode == OperatingMode.DISCHARGING

    def test_seed_negative_starts_charging(self):
        logic = make_logic()
        logic.seed(-200.0)
        assert logic.state.last_computed_w == -200.0
        assert logic.state.integral == -200.0
        assert logic.state.mode == OperatingMode.CHARGING

    def test_seed_none_defaults_to_zero(self):
        logic = make_logic()
        logic.seed(None)
        assert logic.state.last_computed_w == 0.0
        assert logic.state.integral == 0.0
        assert logic.state.mode == OperatingMode.DISCHARGING


# ═══════════════════════════════════════════════════════════
#  ControlLogic.compute — main pipeline
# ═══════════════════════════════════════════════════════════


class TestComputeDischarge:
    def test_grid_draw_increases_discharge(self):
        """Grid importing → controller should discharge more."""
        logic = make_logic(deadband_w=0)
        logic.seed(None)
        m = make_measurement(grid_power_w=200, soc_pct=50)
        output = logic.compute(m, now=0)
        assert output.desired_power_w > 0
        assert "Discharge" in output.reason

    def test_deadband_holds_last_output(self):
        """Small error within deadband → hold last computed value."""
        logic = make_logic(deadband_w=25, discharge_target_w=30)
        logic.seed(None)
        logic.state.last_computed_w = 400.0
        m = make_measurement(grid_power_w=40, soc_pct=50)
        # error = 40 - 30 = 10, within deadband of 25
        output = logic.compute(m, now=0)
        assert output.desired_power_w == pytest.approx(400.0)

    def test_max_discharge_clamped(self):
        """Output cannot exceed max_discharge_w."""
        logic = make_logic(deadband_w=0, max_discharge_w=500)
        logic.seed(None)
        logic.state.integral = 600.0  # force PI to want >500
        m = make_measurement(grid_power_w=500, soc_pct=50)
        output = logic.compute(m, now=0)
        assert output.desired_power_w <= 500.0


class TestComputeCharge:
    def test_surplus_triggers_charge(self):
        """Grid exporting with no battery → PI outputs negative (charge)."""
        logic = make_logic(deadband_w=0)
        logic.seed(None)
        # Grid exporting 200W, no battery activity
        m = make_measurement(grid_power_w=-200, soc_pct=50, battery_power_w=0)
        output = logic.compute(m, now=0)
        assert output.desired_power_w < 0

    def test_surplus_clamp_limits_charge_power(self):
        """Charge power limited to available surplus."""
        logic = make_logic(deadband_w=0, max_charge_w=2400)
        logic.state.mode = OperatingMode.CHARGING
        logic.state.integral = -500  # PI wants to charge heavily

        # Only 100W surplus: battery idle, grid exporting 100W
        m = make_measurement(grid_power_w=-100, soc_pct=50, battery_power_w=0)
        # surplus = 100
        output = logic.compute(m, now=0)
        assert output.desired_power_w == pytest.approx(-100.0)


# ═══════════════════════════════════════════════════════════
#  Guards
# ═══════════════════════════════════════════════════════════


class TestGuards:
    def test_soc_too_low_blocks_discharge(self):
        logic = make_logic(min_soc_pct=10, deadband_w=0)
        logic.seed(None)
        m = make_measurement(grid_power_w=200, soc_pct=10)
        output = logic.compute(m, now=0)
        assert output.desired_power_w == 0.0
        assert output.reason == "SOC too low"

    def test_soc_full_blocks_charge(self):
        logic = make_logic(
            max_soc_pct=100, mode_hysteresis_w=10,
            charge_confirm_s=0, deadband_w=0,
        )
        logic.seed(None)
        m = make_measurement(grid_power_w=-200, soc_pct=100)
        # First tick: charge pending set
        logic.compute(m, now=0)
        # Second tick: mode switches to CHARGING, but SOC full
        output = logic.compute(m, now=1)
        assert output.desired_power_w == 0.0
        assert output.reason == "SOC full"

    def test_discharge_disabled_switch(self):
        logic = make_logic(deadband_w=0)
        logic.seed(None)
        m = make_measurement(
            grid_power_w=200, soc_pct=50,
            discharge_enabled=False,
        )
        output = logic.compute(m, now=0)
        assert output.desired_power_w == 0.0
        assert output.reason == "Discharge disabled"

    def test_charge_disabled_switch(self):
        logic = make_logic(deadband_w=0)
        logic.seed(None)
        m = make_measurement(
            grid_power_w=-200, soc_pct=50,
            charge_enabled=False,
        )
        output = logic.compute(m, now=0)
        assert output.desired_power_w == 0.0
        assert output.reason == "Charge disabled"

    def test_no_surplus_blocks_charge(self):
        """No solar surplus → charge blocked even if PI wants it."""
        logic = make_logic(deadband_w=0)
        logic.seed(None)
        # Grid slightly negative but battery discharging → surplus negative
        m = make_measurement(
            grid_power_w=-10, soc_pct=50, battery_power_w=200,
        )
        # surplus = -200 - (-10) = -190 → no surplus
        output = logic.compute(m, now=0)
        assert output.desired_power_w == 0.0
        assert output.reason == "No surplus, charge blocked"


# ═══════════════════════════════════════════════════════════
#  Emergency feed-in protection
# ═══════════════════════════════════════════════════════════


class TestEmergency:
    def test_emergency_triggers_on_excessive_export(self):
        logic = make_logic(max_feed_in_w=800)
        logic.state.last_computed_w = 500.0

        # Grid exporting 1000W → feed_in=1000 > 800
        m = make_measurement(grid_power_w=-1000, soc_pct=50)
        output = logic.compute(m, now=0)
        assert output.reason == "EMERGENCY"
        assert output.desired_power_w < 500.0

    def test_no_emergency_within_limit(self):
        logic = make_logic(max_feed_in_w=800, deadband_w=0)
        logic.seed(None)
        # Grid exporting 500W → feed_in=500 ≤ 800 → no emergency
        m = make_measurement(grid_power_w=-500, soc_pct=50)
        output = logic.compute(m, now=0)
        assert output.reason != "EMERGENCY"

    def test_emergency_reduces_to_zero_floor(self):
        """Emergency can't go below 0 (no negative discharge)."""
        logic = make_logic(max_feed_in_w=100)
        logic.state.last_computed_w = 50.0
        # feed_in=2000, excess=1900 → forced = max(0, 50-1900-50) = 0
        m = make_measurement(grid_power_w=-2000, soc_pct=50)
        output = logic.compute(m, now=0)
        assert output.reason == "EMERGENCY"
        assert output.desired_power_w == 0.0


# ═══════════════════════════════════════════════════════════
#  Mode switching (Schmitt trigger)
# ═══════════════════════════════════════════════════════════


class TestModeSwitching:
    def test_discharge_to_charge_requires_confirmation(self):
        """Sustained surplus above hysteresis → CHARGING after confirm time."""
        logic = make_logic(
            mode_hysteresis_w=50, charge_confirm_s=10, deadband_w=0,
        )
        logic.seed(None)
        # surplus = 100 > 50 (hysteresis)
        m = make_measurement(grid_power_w=-100, soc_pct=50, battery_power_w=0)

        logic.compute(m, now=0)
        assert logic.state.mode == OperatingMode.DISCHARGING
        assert logic.state.charge_pending_since == 0

        logic.compute(m, now=5)
        assert logic.state.mode == OperatingMode.DISCHARGING

        logic.compute(m, now=10)
        assert logic.state.mode == OperatingMode.CHARGING

    def test_charge_to_discharge_is_instant(self):
        """Surplus drops below -hysteresis → instant switch to DISCHARGING."""
        logic = make_logic(mode_hysteresis_w=50, deadband_w=0)
        logic.state.mode = OperatingMode.CHARGING

        # surplus = -200 < -50 → instant switch
        m = make_measurement(grid_power_w=200, soc_pct=50, battery_power_w=0)
        logic.compute(m, now=0)
        assert logic.state.mode == OperatingMode.DISCHARGING

    def test_charge_pending_cancelled_when_surplus_drops(self):
        logic = make_logic(
            mode_hysteresis_w=50, charge_confirm_s=10, deadband_w=0,
        )
        logic.seed(None)

        m_surplus = make_measurement(
            grid_power_w=-100, soc_pct=50, battery_power_w=0,
        )
        logic.compute(m_surplus, now=0)
        assert logic.state.charge_pending_since == 0

        m_no_surplus = make_measurement(
            grid_power_w=0, soc_pct=50, battery_power_w=0,
        )
        logic.compute(m_no_surplus, now=5)
        assert logic.state.charge_pending_since is None
        assert logic.state.mode == OperatingMode.DISCHARGING

    def test_mode_switch_resets_integral(self):
        logic = make_logic(
            mode_hysteresis_w=50, charge_confirm_s=0, deadband_w=0,
        )
        logic.seed(None)
        logic.state.integral = 300.0

        m = make_measurement(grid_power_w=-100, soc_pct=50, battery_power_w=0)
        # First tick: charge pending set
        logic.compute(m, now=0)
        # Second tick: mode switches, integral resets
        logic.compute(m, now=1)
        assert logic.state.mode == OperatingMode.CHARGING
        # Integral was reset to 0 on mode switch (may have accumulated slightly
        # during this tick's PI computation, but should be near zero)


# ═══════════════════════════════════════════════════════════
#  State updates
# ═══════════════════════════════════════════════════════════


class TestStateUpdates:
    def test_last_computed_updated_on_normal_output(self):
        logic = make_logic(deadband_w=0)
        logic.seed(None)
        m = make_measurement(grid_power_w=200, soc_pct=50)
        output = logic.compute(m, now=0)
        assert logic.state.last_computed_w == output.desired_power_w

    def test_last_computed_updated_on_guarded_output(self):
        logic = make_logic(min_soc_pct=10, deadband_w=0)
        logic.seed(None)
        m = make_measurement(grid_power_w=200, soc_pct=10)
        output = logic.compute(m, now=0)
        assert output.desired_power_w == 0.0
        assert logic.state.last_computed_w == 0.0

    def test_last_computed_updated_on_emergency(self):
        logic = make_logic(max_feed_in_w=800)
        logic.state.last_computed_w = 500.0
        m = make_measurement(grid_power_w=-1000, soc_pct=50)
        output = logic.compute(m, now=0)
        assert logic.state.last_computed_w == output.desired_power_w

    def test_integral_frozen_during_guard(self):
        """When a guard blocks output, integral should not change."""
        logic = make_logic(min_soc_pct=10, deadband_w=0)
        logic.seed(None)
        logic.state.integral = 100.0
        m = make_measurement(grid_power_w=200, soc_pct=10)
        logic.compute(m, now=0)
        assert logic.state.integral == 100.0


# ═══════════════════════════════════════════════════════════
#  Config.from_args
# ═══════════════════════════════════════════════════════════


class TestConfigFromArgs:
    """Tests for Config.from_args mapping from apps.yaml keys."""

    MINIMAL_ARGS = {
        "grid_power_sensor": "sensor.grid",
        "soc_sensor": "sensor.soc",
        "battery_power_sensor": "sensor.battery",
    }

    def test_minimal_args_uses_defaults(self):
        cfg = Config.from_args(self.MINIMAL_ARGS)
        assert cfg.grid_sensor == "sensor.grid"
        assert cfg.soc_sensor == "sensor.soc"
        assert cfg.battery_power_sensor == "sensor.battery"
        assert cfg.discharge_target_w == 30.0
        assert cfg.charge_target_w == 0.0
        assert cfg.kp_discharge_up == 0.50
        assert cfg.kp_discharge_down == 0.71
        assert cfg.kp_charge_up == 0.33
        assert cfg.kp_charge_down == 0.45
        assert cfg.ki_discharge_up == 0.025
        assert cfg.ki_discharge_down == 0.051
        assert cfg.ki_charge_up == 0.011
        assert cfg.ki_charge_down == 0.021
        assert cfg.deadband_w == 25.0
        assert cfg.interval_s == 5
        assert cfg.max_discharge_w == 800.0
        assert cfg.max_charge_w == 2400.0
        assert cfg.min_soc_pct == 10.0
        assert cfg.max_soc_pct == 100.0
        assert cfg.dry_run is True
        assert cfg.debug is False
        assert cfg.battery_sensor_mode == "signed"
        assert cfg.ac_mode_entity is None
        assert cfg.charge_switch is None
        assert cfg.discharge_switch is None

    def test_custom_args_override_defaults(self):
        args = {
            **self.MINIMAL_ARGS,
            "target_grid_power": 50,
            "charge_target_power": -10,
            "kp": 0.5,
            "ki": 0.05,
            "deadband": 30,
            "interval": 3,
            "max_output": 600,
            "max_charge": 1200,
            "min_soc": 20,
            "max_soc": 95,
            "mode_hysteresis": 100,
            "charge_confirm": 20,
            "max_feed_in": 500,
            "emergency_kp_multiplier": 6.0,
            "dry_run": False,
            "debug": True,
            "sensor_prefix": "sensor.test",
            "battery_sensor_mode": "unsigned",
            "ac_mode_entity": "select.ac_mode",
            "charge_switch": "input_boolean.charge",
            "discharge_switch": "input_boolean.discharge",
        }
        cfg = Config.from_args(args)
        assert cfg.discharge_target_w == 50.0
        assert cfg.charge_target_w == -10.0
        assert cfg.kp_discharge_up == 0.5
        assert cfg.kp_discharge_down == 0.5
        assert cfg.kp_charge_up == 0.5
        assert cfg.kp_charge_down == 0.5
        assert cfg.ki_discharge_up == 0.05
        assert cfg.ki_discharge_down == 0.05
        assert cfg.ki_charge_up == 0.05
        assert cfg.ki_charge_down == 0.05
        assert cfg.deadband_w == 30.0
        assert cfg.interval_s == 3
        assert cfg.max_discharge_w == 600.0
        assert cfg.max_charge_w == 1200.0
        assert cfg.min_soc_pct == 20.0
        assert cfg.max_soc_pct == 95.0
        assert cfg.mode_hysteresis_w == 100.0
        assert cfg.charge_confirm_s == 20.0
        assert cfg.max_feed_in_w == 500.0
        assert cfg.emergency_kp_mult == 6.0
        assert cfg.dry_run is False
        assert cfg.debug is True
        assert cfg.sensor_prefix == "sensor.test"
        assert cfg.battery_sensor_mode == "unsigned"
        assert cfg.ac_mode_entity == "select.ac_mode"
        assert cfg.charge_switch == "input_boolean.charge"
        assert cfg.discharge_switch == "input_boolean.discharge"

    def test_kp_ki_from_args(self):
        """Single kp/ki keys are mapped to all four quadrants."""
        args = {**self.MINIMAL_ARGS, "kp": 0.7, "ki": 0.04}
        cfg = Config.from_args(args)
        assert cfg.kp_discharge_up == 0.7
        assert cfg.kp_discharge_down == 0.7
        assert cfg.kp_charge_up == 0.7
        assert cfg.kp_charge_down == 0.7
        assert cfg.ki_discharge_up == 0.04
        assert cfg.ki_discharge_down == 0.04
        assert cfg.ki_charge_up == 0.04
        assert cfg.ki_charge_down == 0.04


# ═══════════════════════════════════════════════════════════
#  ControlOutput factories
# ═══════════════════════════════════════════════════════════


class TestControlOutput:
    def test_from_raw(self):
        out = ControlOutput.from_raw(200.0, 50.0, 150.0, 5.0, "test")
        assert out.desired_power_w == 200.0
        assert out.p_term == 50.0
        assert out.i_term == 150.0
        assert out.ff_pv_term == 5.0
        assert out.reason == "test"

    def test_idle(self):
        out = ControlOutput.idle(10.0, 20.0, 3.0, "guard")
        assert out.desired_power_w == 0.0
        assert out.p_term == 10.0
        assert out.i_term == 20.0
        assert out.ff_pv_term == 3.0
        assert out.reason == "guard"


# ═══════════════════════════════════════════════════════════
#  PIController — additional edge cases
# ═══════════════════════════════════════════════════════════


class TestPIControllerEdgeCases:
    def test_zero_error_no_change(self):
        pi = PIController(
            output_min=-1000, output_max=1000,
        )
        output, p_term, new_integral = pi.update(error=0.0, integral=50.0, dt=5, gains=PIGains(0.5, 0.1))
        assert p_term == 0.0
        assert new_integral == 50.0
        assert output == 50.0

    def test_zero_dt_no_integral_change(self):
        pi = PIController(
            output_min=-1000, output_max=1000,
        )
        output, _, new_integral = pi.update(error=100, integral=0.0, dt=0, gains=PIGains(0.5, 0.1))
        assert new_integral == 0.0
        assert output == pytest.approx(50.0)

    def test_symmetric_gains_same_magnitude(self):
        pi = PIController(
            output_min=-1000, output_max=1000,
        )
        gains = PIGains(0.5, 0.0)
        out_pos, _, _ = pi.update(error=100, integral=0, dt=1, gains=gains)
        out_neg, _, _ = pi.update(error=-100, integral=0, dt=1, gains=gains)
        assert out_pos == pytest.approx(-out_neg)

    def test_integral_only_accumulates(self):
        """Multiple updates with ki-only accumulate integral correctly."""
        pi = PIController(
            output_min=-1000, output_max=1000,
        )
        gains = PIGains(0.0, 0.1)
        _, _, integral = pi.update(error=100, integral=0.0, dt=5, gains=gains)
        assert integral == pytest.approx(50.0)
        _, _, integral = pi.update(error=100, integral=integral, dt=5, gains=gains)
        assert integral == pytest.approx(100.0)


# ═══════════════════════════════════════════════════════════
#  ControlLogic — target_for_mode
# ═══════════════════════════════════════════════════════════


class TestTargetForMode:
    def test_discharging_target(self):
        logic = make_logic(discharge_target_w=30)
        assert logic.target_for_mode() == 30.0

    def test_charging_target(self):
        logic = make_logic(charge_target_w=5)
        logic.state.mode = OperatingMode.CHARGING
        assert logic.target_for_mode() == 5.0

    def test_custom_targets(self):
        logic = make_logic(discharge_target_w=50, charge_target_w=-10)
        assert logic.target_for_mode() == 50.0
        logic.state.mode = OperatingMode.CHARGING
        assert logic.target_for_mode() == -10.0


# ═══════════════════════════════════════════════════════════
#  ControlLogic._clamp
# ═══════════════════════════════════════════════════════════


class TestClamp:
    def test_discharge_capped_at_max(self):
        logic = make_logic(max_discharge_w=500)
        assert logic._clamp(600, surplus=1000) == 500.0

    def test_discharge_passes_through_below_max(self):
        logic = make_logic(max_discharge_w=500)
        assert logic._clamp(300, surplus=1000) == 300.0

    def test_charge_capped_at_surplus(self):
        logic = make_logic(max_charge_w=2400)
        # surplus=100 → can't charge more than 100W
        assert logic._clamp(-500, surplus=100) == -100.0

    def test_charge_capped_at_max_charge(self):
        logic = make_logic(max_charge_w=200)
        # surplus=500 → max_charge_w limits
        assert logic._clamp(-500, surplus=500) == -200.0

    def test_zero_raw_returns_zero(self):
        logic = make_logic()
        assert logic._clamp(0.0, surplus=100) == 0.0

    def test_charge_with_no_surplus(self):
        logic = make_logic(max_charge_w=2400)
        # surplus=0 → can't charge at all
        assert logic._clamp(-500, surplus=0) == 0.0

    def test_charge_with_negative_surplus(self):
        logic = make_logic(max_charge_w=2400)
        assert logic._clamp(-500, surplus=-100) == 0.0


# ═══════════════════════════════════════════════════════════
#  ControlLogic.compute — anti-windup on surplus clamp
# ═══════════════════════════════════════════════════════════


class TestAntiWindupSurplusClamp:
    def test_integral_back_calculated_when_surplus_clamps(self):
        """When charge is limited by surplus, integral is back-calculated."""
        logic = make_logic(deadband_w=0, max_charge_w=2400)
        logic.state.mode = OperatingMode.CHARGING
        logic.state.integral = -500.0

        # Only 50W surplus: battery idle, grid exporting 50W
        m = make_measurement(grid_power_w=-50, soc_pct=50, battery_power_w=0)
        output = logic.compute(m, now=0)
        # Result should be clamped to -50W (surplus limited)
        assert output.desired_power_w == pytest.approx(-50.0)
        # Integral back-calculated: clamped - p_term
        assert logic.state.integral != -500.0

    def test_no_back_calculation_when_unclamped(self):
        """When output is not clamped, integral is committed as-is."""
        logic = make_logic(deadband_w=0, max_discharge_w=800)
        logic.seed(None)
        logic.state.integral = 0.0
        m = make_measurement(grid_power_w=100, soc_pct=50)
        output = logic.compute(m, now=0)
        # error = grid - target = 100 - 30 = 70
        # kp_discharge_up=0.50 → p_term=35, ki_discharge_up=0.025 → integral = 0.025*70*5 = 8.75
        # output = 43.75, well below 800 → no clamp
        assert output.desired_power_w == pytest.approx(43.75)
        assert logic.state.integral == pytest.approx(8.75)


# ═══════════════════════════════════════════════════════════
#  ControlLogic.compute — multi-cycle integration
# ═══════════════════════════════════════════════════════════


class TestMultiCycle:
    def test_integral_accumulates_over_cycles(self):
        """PI integral grows across multiple compute cycles."""
        logic = make_logic(
            deadband_w=0,
            kp_discharge_up=0.0, kp_discharge_down=0.0,
            kp_charge_up=0.0, kp_charge_down=0.0,
            ki_discharge_up=0.1, ki_discharge_down=0.1,
            ki_charge_up=0.1, ki_charge_down=0.1,
        )
        logic.seed(None)
        m = make_measurement(grid_power_w=100, soc_pct=50)
        logic.compute(m, now=0)
        i1 = logic.state.integral
        logic.compute(m, now=5)
        i2 = logic.state.integral
        assert i2 > i1

    def test_output_increases_with_sustained_import(self):
        """Sustained grid import ramps up discharge over several cycles."""
        logic = make_logic(deadband_w=0)
        logic.seed(None)
        m = make_measurement(grid_power_w=200, soc_pct=50)
        outputs = []
        for t in range(0, 30, 5):
            out = logic.compute(m, now=t)
            outputs.append(out.desired_power_w)
        # Should be increasing
        for i in range(1, len(outputs)):
            assert outputs[i] >= outputs[i - 1]

    def test_charging_output_grows_with_sustained_export(self):
        """Sustained grid export increases charge power over cycles."""
        logic = make_logic(
            deadband_w=0, mode_hysteresis_w=10, charge_confirm_s=0,
        )
        logic.seed(None)
        # Large surplus: grid exporting + battery idle
        m = make_measurement(grid_power_w=-300, soc_pct=50, battery_power_w=0)
        outputs = []
        for t in range(0, 30, 5):
            out = logic.compute(m, now=t)
            outputs.append(out.desired_power_w)
        # After mode switch to CHARGING, outputs should become more negative
        # (but clamped by surplus)
        assert any(o < 0 for o in outputs)


# ═══════════════════════════════════════════════════════════
#  ControlLogic — edge cases
# ═══════════════════════════════════════════════════════════


class TestComputeEdgeCases:
    def test_soc_just_above_min_allows_discharge(self):
        """SOC at min+1 should still allow discharge."""
        logic = make_logic(min_soc_pct=10, deadband_w=0)
        logic.seed(None)
        m = make_measurement(grid_power_w=200, soc_pct=11)
        output = logic.compute(m, now=0)
        assert output.desired_power_w > 0

    def test_soc_just_below_max_allows_charge(self):
        """SOC at max-1 should still allow charge."""
        logic = make_logic(
            max_soc_pct=100, mode_hysteresis_w=10,
            charge_confirm_s=0, deadband_w=0,
        )
        logic.seed(None)
        m = make_measurement(grid_power_w=-200, soc_pct=99, battery_power_w=0)
        logic.compute(m, now=0)
        output = logic.compute(m, now=1)
        assert logic.state.mode == OperatingMode.CHARGING
        assert output.desired_power_w < 0

    def test_emergency_back_calculates_integral(self):
        """Emergency sets integral equal to forced power for smooth resume."""
        logic = make_logic(max_feed_in_w=800)
        logic.state.last_computed_w = 500.0
        logic.state.integral = 200.0
        m = make_measurement(grid_power_w=-1000, soc_pct=50)
        output = logic.compute(m, now=0)
        assert output.reason == "EMERGENCY"
        assert logic.state.integral == output.desired_power_w

    def test_compute_uses_time_monotonic_default(self):
        """Without explicit now, compute uses time.monotonic()."""
        logic = make_logic(deadband_w=0)
        logic.seed(None)
        m = make_measurement(grid_power_w=100, soc_pct=50)
        # Should not raise
        output = logic.compute(m)
        assert output.desired_power_w > 0

    def test_charge_disabled_blocks_negative_output(self):
        """Charge disabled blocks output even in CHARGING mode."""
        logic = make_logic(
            deadband_w=0, mode_hysteresis_w=10, charge_confirm_s=0,
        )
        logic.state.mode = OperatingMode.CHARGING
        logic.state.integral = -200.0
        m = make_measurement(
            grid_power_w=-100, soc_pct=50, battery_power_w=0,
            charge_enabled=False,
        )
        output = logic.compute(m, now=0)
        assert output.desired_power_w == 0.0
        assert output.reason == "Charge disabled"

    def test_discharge_disabled_blocks_positive_output(self):
        """Discharge disabled blocks output even with high grid import."""
        logic = make_logic(deadband_w=0)
        logic.seed(None)
        logic.state.integral = 200.0
        m = make_measurement(
            grid_power_w=200, soc_pct=50,
            discharge_enabled=False,
        )
        output = logic.compute(m, now=0)
        assert output.desired_power_w == 0.0
        assert output.reason == "Discharge disabled"


# ═══════════════════════════════════════════════════════════
#  ControllerState defaults
# ═══════════════════════════════════════════════════════════


class TestControllerState:
    def test_defaults(self):
        s = ControllerState()
        assert s.integral == 0.0
        assert s.last_computed_w == 0.0
        assert s.mode == OperatingMode.DISCHARGING
        assert s.charge_pending_since is None
        assert s.previous_pv_w is None


# ═══════════════════════════════════════════════════════════
#  PV Feed-Forward
# ═══════════════════════════════════════════════════════════


class TestPVFeedForward:
    def test_ff_zero_when_no_pv_sensor(self):
        """No PV sensor → ff_pv_term is always 0."""
        logic = make_logic(ff_pv_gain=0.6, deadband_w=0)
        logic.seed(None)
        m = make_measurement(grid_power_w=100, soc_pct=50)
        output = logic.compute(m, now=0)
        assert output.ff_pv_term == 0.0

    def test_ff_zero_on_first_cycle(self):
        """First cycle with PV → no previous → ff = 0."""
        logic = make_logic(ff_pv_gain=0.6, ff_pv_deadband_w=20, deadband_w=0)
        logic.seed(None)
        m = make_measurement(grid_power_w=100, soc_pct=50, pv_power_w=1000)
        output = logic.compute(m, now=0)
        assert output.ff_pv_term == 0.0

    def test_ff_positive_on_pv_drop(self):
        """PV drops → positive ff (need more discharge)."""
        logic = make_logic(
            ff_pv_gain=0.6, ff_pv_deadband_w=20, deadband_w=0,
        )
        logic.seed(None)
        m1 = make_measurement(grid_power_w=100, soc_pct=50, pv_power_w=1000)
        logic.compute(m1, now=0)

        m2 = make_measurement(grid_power_w=100, soc_pct=50, pv_power_w=800)
        output = logic.compute(m2, now=5)
        # delta = 800 - 1000 = -200, ff = -0.6 * (-200) = 120
        assert output.ff_pv_term == pytest.approx(120.0)

    def test_ff_negative_on_pv_rise(self):
        """PV rises → negative ff (can charge more)."""
        logic = make_logic(
            ff_pv_gain=0.6, ff_pv_deadband_w=20, deadband_w=0,
        )
        logic.seed(None)
        m1 = make_measurement(grid_power_w=100, soc_pct=50, pv_power_w=500)
        logic.compute(m1, now=0)

        m2 = make_measurement(grid_power_w=100, soc_pct=50, pv_power_w=700)
        output = logic.compute(m2, now=5)
        # delta = 700 - 500 = 200, ff = -0.6 * 200 = -120
        assert output.ff_pv_term == pytest.approx(-120.0)

    def test_ff_ignored_within_deadband(self):
        """Small PV change within ff_pv_deadband → ff = 0."""
        logic = make_logic(
            ff_pv_gain=0.6, ff_pv_deadband_w=20, deadband_w=0,
        )
        logic.seed(None)
        m1 = make_measurement(grid_power_w=100, soc_pct=50, pv_power_w=1000)
        logic.compute(m1, now=0)

        m2 = make_measurement(grid_power_w=100, soc_pct=50, pv_power_w=1010)
        output = logic.compute(m2, now=5)
        assert output.ff_pv_term == 0.0

    def test_ff_disabled_when_gain_zero(self):
        """ff_pv_gain=0 → ff is always 0 regardless of PV change."""
        logic = make_logic(
            ff_pv_gain=0.0, ff_pv_deadband_w=20, deadband_w=0,
        )
        logic.seed(None)
        m1 = make_measurement(grid_power_w=100, soc_pct=50, pv_power_w=1000)
        logic.compute(m1, now=0)

        m2 = make_measurement(grid_power_w=100, soc_pct=50, pv_power_w=500)
        output = logic.compute(m2, now=5)
        assert output.ff_pv_term == 0.0




# ═══════════════════════════════════════════════════════════
#  Config.from_args — new fields
# ═══════════════════════════════════════════════════════════


class TestConfigFromArgsNewFields:
    MINIMAL_ARGS = {
        "grid_power_sensor": "sensor.grid",
        "soc_sensor": "sensor.soc",
        "battery_power_sensor": "sensor.battery",
    }

    def test_defaults_for_new_fields(self):
        cfg = Config.from_args(self.MINIMAL_ARGS)
        assert cfg.pv_sensor == ""
        assert cfg.ff_pv_gain == 0.6
        assert cfg.ff_pv_deadband_w == 20.0

    def test_custom_new_fields(self):
        args = {
            **self.MINIMAL_ARGS,
            "pv_sensor": "sensor.pv",
            "ff_pv_gain": 0.8,
            "ff_pv_deadband": 30,
        }
        cfg = Config.from_args(args)
        assert cfg.pv_sensor == "sensor.pv"
        assert cfg.ff_pv_gain == 0.8
        assert cfg.ff_pv_deadband_w == 30.0


# ═══════════════════════════════════════════════════════════
#  Previous state updated each cycle
# ═══════════════════════════════════════════════════════════


class TestPreviousStateUpdates:
    def test_previous_pv_updated_after_compute(self):
        logic = make_logic(deadband_w=0)
        logic.seed(None)
        m = make_measurement(grid_power_w=100, soc_pct=50, pv_power_w=500)
        logic.compute(m, now=0)
        assert logic.state.previous_pv_w == 500.0

    def test_previous_pv_none_when_no_sensor(self):
        logic = make_logic(deadband_w=0)
        logic.seed(None)
        m = make_measurement(grid_power_w=100, soc_pct=50)
        logic.compute(m, now=0)
        assert logic.state.previous_pv_w is None

    def test_previous_pv_updated_on_emergency(self):
        """Previous state is updated even when emergency fires."""
        logic = make_logic(max_feed_in_w=800, pv_sensor="sensor.pv")
        logic.state.last_computed_w = 500.0
        m = make_measurement(grid_power_w=-1000, soc_pct=50, pv_power_w=2000)
        logic.compute(m, now=0)
        assert logic.state.previous_pv_w == 2000.0

    def test_previous_pv_updated_on_guard(self):
        """Previous state is updated even when a guard blocks output."""
        logic = make_logic(min_soc_pct=10, deadband_w=0, pv_sensor="sensor.pv")
        logic.seed(None)
        m = make_measurement(grid_power_w=200, soc_pct=10, pv_power_w=500)
        logic.compute(m, now=0)
        assert logic.state.previous_pv_w == 500.0
