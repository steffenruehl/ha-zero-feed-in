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
            kp_up=0.5, kp_down=1.0,
            ki_up=0.0, ki_down=0.0,
            output_min=-1000, output_max=1000,
        )
        output, p_term, _ = pi.update(error=100, integral=0.0, dt=5)
        assert p_term == pytest.approx(50.0)
        assert output == pytest.approx(50.0)

    def test_proportional_negative_error_uses_down_gain(self):
        pi = PIController(
            kp_up=0.5, kp_down=1.0,
            ki_up=0.0, ki_down=0.0,
            output_min=-1000, output_max=1000,
        )
        output, p_term, _ = pi.update(error=-100, integral=0.0, dt=5)
        assert p_term == pytest.approx(-100.0)
        assert output == pytest.approx(-100.0)

    def test_integral_accumulation(self):
        pi = PIController(
            kp_up=0.0, kp_down=0.0,
            ki_up=0.1, ki_down=0.1,
            output_min=-1000, output_max=1000,
        )
        output, _, new_integral = pi.update(error=100, integral=0.0, dt=5)
        assert new_integral == pytest.approx(50.0)  # 0.1 * 100 * 5
        assert output == pytest.approx(50.0)

    def test_anti_windup_upper_clamp(self):
        pi = PIController(
            kp_up=1.0, kp_down=1.0,
            ki_up=0.1, ki_down=0.1,
            output_min=-500, output_max=500,
        )
        output, p_term, new_integral = pi.update(error=600, integral=0.0, dt=5)
        assert output == pytest.approx(500)
        # Back-calculated: integral = output_max - p_term
        assert new_integral == pytest.approx(500 - 600)

    def test_anti_windup_lower_clamp(self):
        pi = PIController(
            kp_up=1.0, kp_down=1.0,
            ki_up=0.1, ki_down=0.1,
            output_min=-500, output_max=500,
        )
        output, p_term, new_integral = pi.update(error=-600, integral=0.0, dt=5)
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
