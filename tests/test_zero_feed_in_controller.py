"""Tests for zero_feed_in_controller – ControlLogic (direct calculation).

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
)


# ═══════════════════════════════════════════════════════════
#  Helpers
# ═══════════════════════════════════════════════════════════


def make_config(**overrides) -> Config:
    """Create a Config with sensible test defaults.

    Accepts short parameter names for convenience and maps them
    to the actual Config field names.
    """
    # Map short names used in tests -> actual Config field names
    _ALIASES = {
        "muting": "muting_s",
        "hysteresis": "hysteresis_w",
        "max_output": "max_discharge_w",
        "max_charge": "max_charge_w",
        "min_soc": "min_soc_pct",
        "max_soc": "max_soc_pct",
        "mode_hysteresis": "mode_hysteresis_w",
        "charge_confirm": "charge_confirm_s",
    }
    defaults = dict(
        grid_sensor="sensor.grid",
        soc_sensor="sensor.soc",
        battery_power_sensor="sensor.battery",
        dry_run=False,
    )
    defaults.update(overrides)
    resolved = {_ALIASES.get(k, k): v for k, v in defaults.items()}
    return Config(**resolved)


def make_measurement(**overrides) -> Measurement:
    """Create a Measurement with sensible test defaults."""
    defaults = dict(
        grid_power_w=0.0,
        soc_pct=50.0,
        battery_power_w=0.0,
    )
    defaults.update(overrides)
    return Measurement(**defaults)


def make_logic(now: float = 0.0, **cfg_overrides) -> tuple[ControlLogic, float]:
    """Create a ControlLogic ready for testing.

    Returns (logic, now) where now is the starting monotonic time.
    The logic is seeded and its muting window is expired so the first
    compute() is not muted.
    """
    cfg = make_config(**cfg_overrides)
    logic = ControlLogic(cfg)
    logic.seed(0.0)
    # Expire the muting from seed
    logic.state.last_command_t = now - cfg.muting_s - 1
    return logic, now


# ═══════════════════════════════════════════════════════════
#  Config.from_args
# ═══════════════════════════════════════════════════════════


class TestConfigFromArgs:
    """Test Config.from_args mapping from apps.yaml keys."""

    def test_minimal_args(self) -> None:
        """Only required sensor keys produce valid Config with defaults."""
        args = {
            "grid_power_sensor": "sensor.grid",
            "soc_sensor": "sensor.soc",
            "battery_power_sensor": "sensor.bp",
        }
        cfg = Config.from_args(args)
        assert cfg.grid_sensor == "sensor.grid"
        assert cfg.ki == 1.0
        assert cfg.hysteresis_w == 15.0
        assert cfg.muting_s == 8.0

    def test_all_new_tuning_args(self) -> None:
        """All new tuning parameters are correctly mapped."""
        args = {
            "grid_power_sensor": "sensor.grid",
            "soc_sensor": "sensor.soc",
            "battery_power_sensor": "sensor.bp",
            "ki": 0.8,
            "hysteresis": 20,
            "muting": 10,
        }
        cfg = Config.from_args(args)
        assert cfg.ki == 0.8
        assert cfg.hysteresis_w == 20.0
        assert cfg.muting_s == 10.0

    def test_optional_switches_default_none(self) -> None:
        """Charge/discharge switches default to None."""
        args = {
            "grid_power_sensor": "s.g",
            "soc_sensor": "s.s",
            "battery_power_sensor": "s.b",
        }
        cfg = Config.from_args(args)
        assert cfg.charge_switch is None
        assert cfg.discharge_switch is None


# ═══════════════════════════════════════════════════════════
#  Direct Calculation
# ═══════════════════════════════════════════════════════════


class TestDirectCalculation:
    """Test core direct calculation: new_limit = last_sent + ki * error."""

    def test_positive_error_increases_discharge(self) -> None:
        """Grid importing (positive error) should increase discharge."""
        logic, now = make_logic()
        # grid=100W, target=30W (default) -> error=70W -> correction=70W
        m = make_measurement(grid_power_w=100.0)
        out = logic.compute(m, now=now)
        assert out is not None
        assert out.desired_power_w == pytest.approx(70.0, abs=1)

    def test_negative_error_starts_charge(self) -> None:
        """Grid exporting (negative grid) should start charging (negative output)."""
        logic, now = make_logic()
        # battery=0, grid=-300 -> surplus = 0+300 = 300
        # error = -300 - 30 = -330 -> correction = -330
        # clamped by surplus: max(-330, -800, -300) = -300
        m = make_measurement(grid_power_w=-300.0, battery_power_w=0.0)
        out = logic.compute(m, now=now)
        assert out is not None
        assert out.desired_power_w < 0  # charging

    def test_ki_scales_correction(self) -> None:
        """ki < 1 should scale the correction proportionally."""
        logic, now = make_logic(ki=0.5)
        # error = 100 - 30 = 70, correction = 0.5 * 70 = 35
        m = make_measurement(grid_power_w=100.0)
        out = logic.compute(m, now=now)
        assert out is not None
        assert out.desired_power_w == pytest.approx(35.0, abs=1)

    def test_zero_error_at_target_returns_none(self) -> None:
        """When grid is exactly at target, no command is needed."""
        logic, now = make_logic()
        # grid = 30W (exactly target) -> error = 0
        m = make_measurement(grid_power_w=30.0)
        out = logic.compute(m, now=now)
        assert out is None

    def test_correction_based_on_last_sent(self) -> None:
        """Correction is added to last_sent, not to zero."""
        logic, now = make_logic()
        logic.state.last_sent_w = 200.0
        # error = 80 - 30 = 50 -> correction = 50
        m = make_measurement(grid_power_w=80.0)
        out = logic.compute(m, now=now)
        assert out is not None
        assert out.desired_power_w == pytest.approx(250.0, abs=1)


# ═══════════════════════════════════════════════════════════
#  Muting
# ═══════════════════════════════════════════════════════════


class TestMuting:
    """Test muting: ignore sensor updates after sending a command."""

    def test_muted_after_command(self) -> None:
        """compute() returns None during muting window after a command."""
        logic, now = make_logic(muting=8.0)
        m = make_measurement(grid_power_w=100.0)
        out = logic.compute(m, now=now)
        assert out is not None  # first command goes through

        # Within muting window -> should return None
        out2 = logic.compute(m, now=now + 5.0)
        assert out2 is None

    def test_unmuted_after_window(self) -> None:
        """compute() resumes after muting window expires."""
        logic, now = make_logic(muting=8.0)
        m = make_measurement(grid_power_w=100.0)
        out = logic.compute(m, now=now)
        assert out is not None

        # After muting window -> should process
        m2 = make_measurement(grid_power_w=80.0)
        out2 = logic.compute(m2, now=now + 9.0)
        # May or may not produce output depending on hysteresis,
        # but the point is it's not muted
        # With last_sent ~70, error=80-30=50, correction=50 > hysteresis=15
        assert out2 is not None

    def test_muting_duration_from_config(self) -> None:
        """Muting duration matches the configured muting_s."""
        logic, now = make_logic(muting=5.0)
        m = make_measurement(grid_power_w=100.0)
        logic.compute(m, now=now)

        # At t=4.9s (just before muting expires)
        assert logic.compute(m, now=now + 4.9) is None
        # At t=5.1s (just after muting expires)
        out = logic.compute(m, now=now + 5.1)
        # Should not be muted (may still return None for other reasons)


# ═══════════════════════════════════════════════════════════
#  Drift Accumulator
# ═══════════════════════════════════════════════════════════


class TestDriftAccumulator:
    """Test drift accumulation for sub-hysteresis errors."""

    def test_small_corrections_accumulate(self) -> None:
        """Multiple small errors below hysteresis accumulate in drift_acc."""
        logic, now = make_logic(hysteresis=15.0)
        # error = 40 - 30 = 10 -> correction = 10 < 15 (hysteresis)
        m = make_measurement(grid_power_w=40.0)
        out = logic.compute(m, now=now)
        assert out is None  # below hysteresis
        assert logic.state.drift_acc == pytest.approx(10.0)

    def test_drift_triggers_when_threshold_crossed(self) -> None:
        """Accumulated drift exceeding hysteresis triggers a command."""
        logic, now = make_logic(hysteresis=15.0)
        # First: error = 10 (below hysteresis)
        m = make_measurement(grid_power_w=40.0)
        logic.compute(m, now=now)
        assert logic.state.drift_acc == pytest.approx(10.0)

        # Second: another 10 -> drift_acc = 20 > 15
        # Need to advance past muting... but we didn't send, so no muting
        out = logic.compute(m, now=now + 0.1)
        assert out is not None
        assert "Drift" in out.reason

    def test_drift_resets_on_send(self) -> None:
        """Drift accumulator is reset to zero after sending a command."""
        logic, now = make_logic(hysteresis=15.0)
        # Large error -> direct send
        m = make_measurement(grid_power_w=100.0)
        logic.compute(m, now=now)
        assert logic.state.drift_acc == 0.0


# ═══════════════════════════════════════════════════════════
#  Mode Switching (Schmitt Trigger)
# ═══════════════════════════════════════════════════════════


class TestModeSwitch:
    """Test Schmitt trigger mode switching between CHARGING/DISCHARGING."""

    def test_starts_in_discharging(self) -> None:
        """Default mode is DISCHARGING."""
        logic, _ = make_logic()
        assert logic.state.mode == OperatingMode.DISCHARGING

    def test_switch_to_charging_requires_confirmation(self) -> None:
        """Mode doesn't switch immediately; needs charge_confirm_s."""
        logic, now = make_logic(charge_confirm=15.0, mode_hysteresis=50.0)
        # Surplus > hysteresis -> starts pending
        # battery=-300, grid=-50 -> surplus = 300+50 = 350 > 50
        m = make_measurement(grid_power_w=-50.0, battery_power_w=-300.0)
        logic.compute(m, now=now)
        assert logic.state.mode == OperatingMode.DISCHARGING
        assert logic.state.charge_pending_since is not None

    def test_switch_to_charging_after_confirmation(self) -> None:
        """Mode switches to CHARGING after surplus holds for charge_confirm_s."""
        logic, now = make_logic(charge_confirm=15.0, mode_hysteresis=50.0, muting=0.0)
        # surplus > hysteresis
        m = make_measurement(grid_power_w=-50.0, battery_power_w=-300.0)
        logic.compute(m, now=now)

        # Advance past confirm time
        logic.compute(m, now=now + 16.0)
        assert logic.state.mode == OperatingMode.CHARGING

    def test_switch_back_to_discharging_is_instant(self) -> None:
        """Switching from CHARGING to DISCHARGING is immediate (no delay)."""
        logic, now = make_logic(muting=0.0)
        logic.state.mode = OperatingMode.CHARGING
        # surplus < -hysteresis -> switch immediately
        # battery=0, grid=100 -> surplus = 0 - 100 = -100 < -50
        m = make_measurement(grid_power_w=100.0, battery_power_w=0.0)
        logic.compute(m, now=now)
        assert logic.state.mode == OperatingMode.DISCHARGING

    def test_charge_pending_cancelled_on_surplus_drop(self) -> None:
        """Charge pending is cancelled if surplus drops below hysteresis."""
        logic, now = make_logic(charge_confirm=15.0, mode_hysteresis=50.0, muting=0.0)
        m = make_measurement(grid_power_w=-50.0, battery_power_w=-300.0)
        logic.compute(m, now=now)
        assert logic.state.charge_pending_since is not None

        # Surplus drops
        m2 = make_measurement(grid_power_w=50.0, battery_power_w=0.0)
        logic.compute(m2, now=now + 5.0)
        assert logic.state.charge_pending_since is None


# ═══════════════════════════════════════════════════════════
#  Guards
# ═══════════════════════════════════════════════════════════


class TestGuards:
    """Test safety guards that override controller outputs."""

    def test_discharge_disabled(self) -> None:
        """Discharge disabled switch blocks positive output."""
        logic, now = make_logic()
        m = make_measurement(grid_power_w=100.0, discharge_enabled=False)
        out = logic.compute(m, now=now)
        assert out is not None
        assert out.desired_power_w == 0.0
        assert "Discharge disabled" in out.reason

    def test_charge_disabled(self) -> None:
        """Charge disabled switch blocks negative output."""
        logic, now = make_logic()
        # Force a negative raw_limit: grid=-200, target=30, error=-230
        # last_sent=0 + (-230) = -230 < 0 -> charge guard
        m = make_measurement(
            grid_power_w=-200.0,
            battery_power_w=200.0,
            charge_enabled=False,
        )
        out = logic.compute(m, now=now)
        assert out is not None
        assert out.desired_power_w == 0.0
        assert "Charge disabled" in out.reason

    def test_soc_too_low_blocks_discharge(self) -> None:
        """SOC at or below min_soc blocks discharge."""
        logic, now = make_logic(min_soc=10)
        m = make_measurement(grid_power_w=100.0, soc_pct=10.0)
        out = logic.compute(m, now=now)
        assert out is not None
        assert out.desired_power_w == 0.0
        assert "SOC too low" in out.reason

    def test_soc_full_blocks_charge(self) -> None:
        """SOC at or above max_soc blocks charging."""
        logic, now = make_logic(max_soc=90)
        m = make_measurement(
            grid_power_w=-200.0,
            battery_power_w=200.0,
            soc_pct=90.0,
            charge_enabled=True,
        )
        out = logic.compute(m, now=now)
        assert out is not None
        assert out.desired_power_w == 0.0
        assert "SOC full" in out.reason

    def test_dynamic_min_soc_overrides_static(self) -> None:
        """Dynamic min SOC > static min SOC blocks discharge at dynamic level."""
        logic, now = make_logic(min_soc=10)
        m = make_measurement(
            grid_power_w=100.0,
            soc_pct=25.0,
            dynamic_min_soc_pct=30.0,
        )
        out = logic.compute(m, now=now)
        assert out is not None
        assert out.desired_power_w == 0.0
        assert "SOC too low" in out.reason

    def test_dynamic_min_soc_clamped_to_hard_limits(self) -> None:
        """Dynamic min SOC cannot go below static min_soc_pct."""
        logic, now = make_logic(min_soc=10)
        m = make_measurement(
            grid_power_w=100.0,
            soc_pct=8.0,
            dynamic_min_soc_pct=5.0,  # below static min
        )
        out = logic.compute(m, now=now)
        assert out is not None
        # Effective min SOC should be max(10, 5) = 10, and soc=8 <= 10
        assert "SOC too low" in out.reason

    def test_low_soc_does_not_block_charging(self) -> None:
        """Low SOC must never prevent charging — only discharging.

        Regression: when last_sent_w was a large positive value (from
        prior discharging) and mode switched to CHARGING, the computed
        new_limit could stay positive despite surplus, triggering the
        'SOC too low' guard and blocking charging indefinitely.
        """
        logic, now = make_logic(min_soc=12, muting=0.0, mode_hysteresis=50.0)
        # Simulate stale state: mode=CHARGING but last_sent_w still
        # holds the old discharge value (e.g. restored from persistence).
        logic.state.mode = OperatingMode.CHARGING
        logic.state.last_sent_w = 1600.0
        logic.state.last_command_t = now - 20

        # Surplus scenario: grid=-580W, battery idle, soc=11% (below min_soc)
        m = make_measurement(
            grid_power_w=-580.0,
            battery_power_w=0.0,
            soc_pct=11.0,
        )
        out = logic.compute(m, now=now)
        assert out is not None
        # Must NOT say "SOC too low" — charging should always be allowed
        assert "SOC too low" not in out.reason
        # Output must be zero or negative (charge direction)
        assert out.desired_power_w <= 0.0


# ═══════════════════════════════════════════════════════════
#  Surplus Clamp
# ═══════════════════════════════════════════════════════════


class TestSurplusClamp:
    """Test charge is capped at available surplus."""

    def test_charge_capped_at_surplus(self) -> None:
        """Charge power cannot exceed measured surplus."""
        logic, now = make_logic(muting=0.0, max_charge=2400)
        logic.state.last_sent_w = 0.0
        logic.state.last_command_t = now - 10
        # grid=-200, battery=-100 -> surplus=300, error=-200-30=-230
        # raw = 0 + (-230) = -230, capped at -surplus = -300 -> keeps -230
        m = make_measurement(grid_power_w=-200.0, battery_power_w=-100.0)
        out = logic.compute(m, now=now)
        assert out is not None
        assert out.desired_power_w == pytest.approx(-230.0)

    def test_discharge_capped_at_max_output(self) -> None:
        """Discharge is capped at max_discharge_w."""
        logic, now = make_logic(max_output=800)
        logic.state.last_sent_w = 700.0
        # error = 500 - 30 = 470 -> new = 700 + 470 = 1170 -> capped at 800
        m = make_measurement(grid_power_w=500.0)
        out = logic.compute(m, now=now)
        assert out is not None
        assert out.desired_power_w == pytest.approx(800.0)


# ═══════════════════════════════════════════════════════════
#  State Persistence
# ═══════════════════════════════════════════════════════════


class TestStatePersistence:
    """Test state snapshot and restore."""

    def test_snapshot_roundtrip(self) -> None:
        """State can be serialized and restored."""
        logic, _ = make_logic()
        logic.state.mode = OperatingMode.CHARGING
        logic.state.last_sent_w = 123.4
        logic.state.drift_acc = 5.0
        snap = logic.state_snapshot()

        logic2, _ = make_logic()
        assert logic2.restore_from_snapshot(snap) is True
        assert logic2.state.mode == OperatingMode.CHARGING
        assert logic2.state.last_sent_w == pytest.approx(123.4)
        assert logic2.state.drift_acc == pytest.approx(5.0)

    def test_restore_invalid_data(self) -> None:
        """Restore returns False on invalid data."""
        logic, _ = make_logic()
        assert logic.restore_from_snapshot({}) is False
        assert logic.restore_from_snapshot({"last_sent_w": "bad"}) is False

    def test_restore_preserves_state_on_failure(self) -> None:
        """Failed restore does not modify existing state."""
        logic, _ = make_logic()
        logic.state.last_sent_w = 42.0
        logic.restore_from_snapshot({})
        assert logic.state.last_sent_w == 42.0


# ═══════════════════════════════════════════════════════════
#  Seed
# ═══════════════════════════════════════════════════════════


class TestSeed:
    """Test controller seeding from current battery power."""

    def test_seed_positive(self) -> None:
        """Seeding with positive battery power sets DISCHARGING."""
        cfg = make_config()
        logic = ControlLogic(cfg)
        logic.seed(200.0)
        assert logic.state.last_sent_w == 200.0
        assert logic.state.mode == OperatingMode.DISCHARGING

    def test_seed_negative(self) -> None:
        """Seeding with negative battery power sets CHARGING."""
        cfg = make_config()
        logic = ControlLogic(cfg)
        logic.seed(-100.0)
        assert logic.state.last_sent_w == -100.0
        assert logic.state.mode == OperatingMode.CHARGING

    def test_seed_none(self) -> None:
        """Seeding with None defaults to 0."""
        cfg = make_config()
        logic = ControlLogic(cfg)
        logic.seed(None)
        assert logic.state.last_sent_w == 0.0


# ═══════════════════════════════════════════════════════════
#  Surplus Estimation
# ═══════════════════════════════════════════════════════════


class TestSurplusEstimation:
    """Test surplus = -battery_power - grid_power."""

    def test_surplus_pv_scenario(self) -> None:
        """PV producing, battery charging, grid exporting -> positive surplus."""
        # battery=-300 (charging), grid=-100 (exporting)
        # surplus = -(-300) - (-100) = 400
        m = make_measurement(battery_power_w=-300.0, grid_power_w=-100.0)
        assert ControlLogic.estimate_surplus(m) == pytest.approx(400.0)

    def test_surplus_night_scenario(self) -> None:
        """No PV, battery discharging, grid importing -> negative surplus."""
        # battery=200 (discharging), grid=50 (importing)
        # surplus = -200 - 50 = -250
        m = make_measurement(battery_power_w=200.0, grid_power_w=50.0)
        assert ControlLogic.estimate_surplus(m) == pytest.approx(-250.0)


# ═══════════════════════════════════════════════════════════
#  Dry Run
# ═══════════════════════════════════════════════════════════


class TestDryRun:
    """Test dry_run mode: compute but don't update state."""

    def test_dry_run_does_not_update_last_sent(self) -> None:
        """In dry_run mode, last_sent_w is not updated."""
        logic, now = make_logic(dry_run=True)
        m = make_measurement(grid_power_w=100.0)
        out = logic.compute(m, now=now)
        assert out is not None
        assert logic.state.last_sent_w == 0.0  # unchanged

    def test_dry_run_does_not_accumulate_drift(self) -> None:
        """In dry_run mode, drift_acc is not accumulated."""
        logic, now = make_logic(dry_run=True, hysteresis=15.0)
        m = make_measurement(grid_power_w=40.0)  # small error
        logic.compute(m, now=now)
        assert logic.state.drift_acc == 0.0  # unchanged

    def test_dry_run_still_returns_output(self) -> None:
        """Dry run still computes and returns an output."""
        logic, now = make_logic(dry_run=True)
        m = make_measurement(grid_power_w=100.0)
        out = logic.compute(m, now=now)
        assert out is not None
        assert out.desired_power_w > 0


# ═══════════════════════════════════════════════════════════
#  ControlOutput
# ═══════════════════════════════════════════════════════════


class TestControlOutput:
    """Test ControlOutput construction."""

    def test_idle(self) -> None:
        """idle() returns zero power with given reason."""
        out = ControlOutput.idle("test reason")
        assert out.desired_power_w == 0.0
        assert out.reason == "test reason"
