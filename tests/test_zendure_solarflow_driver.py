"""Tests for zendure_solarflow_driver – AdaptiveLockout and RelayStateMachine.

All tests exercise pure computation classes (no HA/AppDaemon mocking needed).

Note: timestamps start at t=1.0 (not 0.0) because the production code uses
``time.monotonic()`` which is always > 0, and uses 0.0 as a sentinel for
"no previous tick".
"""

from __future__ import annotations

import pytest

from src.zendure_solarflow_driver import (
    DIRECTION_THRESHOLD_W,
    MIN_ACTIVE_POWER_W,
    RELAY_SAFETY_TIMEOUT_S,
    ROUNDING_STEP_W,
    AdaptiveLockout,
    RelayDirection,
    RelayState,
    RelayStateMachine,
    ZendureSolarFlowDriver,
)


# ═══════════════════════════════════════════════════════════
#  Helpers
# ═══════════════════════════════════════════════════════════

T0 = 1.0
"""Base timestamp for tests — matches production where time.monotonic() > 0."""


def make_lockout(full_power_w: float = 200.0) -> AdaptiveLockout:
    """Create an AdaptiveLockout with default reference power."""
    return AdaptiveLockout(full_power_w=full_power_w)


def make_sm(
    base_lockout_s: float = 30.0,
    idle_lockout_s: float = 5.0,
    ref_w: float = 200.0,
    log_fn=None,
) -> RelayStateMachine:
    """Create a RelayStateMachine with test defaults."""
    return RelayStateMachine(
        idle_lockout_s=idle_lockout_s,
        charge_lockout=AdaptiveLockout(full_power_w=ref_w),
        discharge_lockout=AdaptiveLockout(full_power_w=ref_w),
        base_lockout_s=base_lockout_s,
        log_fn=log_fn,
    )


# ═══════════════════════════════════════════════════════════
#  AdaptiveLockout
# ═══════════════════════════════════════════════════════════


class TestAdaptiveLockout:
    """Tests for the energy-integrating lockout timer."""

    def test_fresh_lockout_not_settled(self):
        al = make_lockout()
        assert not al.is_settled(base_s=30.0)

    def test_reset_clears_accumulator(self):
        al = make_lockout()
        al.tick(200, now=T0)
        al.tick(200, now=T0 + 10)
        al.reset()
        assert al.progress == 0.0
        assert not al.is_settled(base_s=30.0)

    def test_single_tick_no_accumulation(self):
        """First tick only sets the timestamp, no dt available yet."""
        al = make_lockout()
        al.tick(200, now=T0)
        assert al.progress == 0.0

    def test_threshold_calculation(self):
        al = make_lockout(full_power_w=200.0)
        assert al.threshold(base_s=30.0) == 6000.0

    # -- Doc examples: full_power_w=200, base=30s, threshold=6000 W·s --

    def test_200w_sustained_settles_in_30s(self):
        """200 W sustained -> 30 s (from docs table)."""
        al = make_lockout(full_power_w=200.0)
        al.tick(200, now=T0)
        al.tick(200, now=T0 + 30)
        assert al.is_settled(base_s=30.0)

    def test_200w_just_under_30s_not_settled(self):
        al = make_lockout(full_power_w=200.0)
        al.tick(200, now=T0)
        al.tick(200, now=T0 + 29.9)
        assert not al.is_settled(base_s=30.0)

    def test_100w_sustained_settles_in_60s(self):
        """100 W sustained -> 60 s (from docs table)."""
        al = make_lockout(full_power_w=200.0)
        al.tick(100, now=T0)
        al.tick(100, now=T0 + 60)
        assert al.is_settled(base_s=30.0)

    def test_100w_just_under_60s_not_settled(self):
        al = make_lockout(full_power_w=200.0)
        al.tick(100, now=T0)
        al.tick(100, now=T0 + 59)
        assert not al.is_settled(base_s=30.0)

    def test_50w_sustained_settles_in_120s(self):
        """50 W sustained -> 120 s (from docs table)."""
        al = make_lockout(full_power_w=200.0)
        al.tick(50, now=T0)
        al.tick(50, now=T0 + 120)
        assert al.is_settled(base_s=30.0)

    def test_20w_sustained_settles_in_300s(self):
        """<20 W -> 300 s (from docs table)."""
        al = make_lockout(full_power_w=200.0)
        al.tick(20, now=T0)
        al.tick(20, now=T0 + 300)
        assert al.is_settled(base_s=30.0)

    def test_variable_power_proportional(self):
        """Variable power: each slice contributes proportionally."""
        al = make_lockout(full_power_w=200.0)
        # 200 W for 15 s -> 3000 W-s
        al.tick(200, now=T0)
        al.tick(200, now=T0 + 15)
        assert al.progress == pytest.approx(3000.0)
        assert not al.is_settled(base_s=30.0)
        # Then 100 W for 30 s -> 3000 W-s more -> total 6000 -> settled
        al.tick(100, now=T0 + 45)
        assert al.progress == pytest.approx(6000.0)
        assert al.is_settled(base_s=30.0)

    def test_negative_power_uses_abs(self):
        """Charging (negative) power contributes via absolute value."""
        al = make_lockout(full_power_w=200.0)
        al.tick(-200, now=T0)
        al.tick(-200, now=T0 + 30)
        assert al.is_settled(base_s=30.0)

    def test_zero_power_no_progress(self):
        """Zero power contributes nothing to the accumulator."""
        al = make_lockout(full_power_w=200.0)
        al.tick(0, now=T0)
        al.tick(0, now=T0 + 1000)
        assert al.progress == 0.0
        assert not al.is_settled(base_s=30.0)

    def test_reset_then_reaccumulate(self):
        """After reset, accumulation restarts from zero."""
        al = make_lockout(full_power_w=200.0)
        al.tick(200, now=T0)
        al.tick(200, now=T0 + 15)
        assert al.progress == pytest.approx(3000.0)
        al.reset()
        al.tick(200, now=T0 + 20)
        al.tick(200, now=T0 + 50)
        assert al.progress == pytest.approx(6000.0)
        assert al.is_settled(base_s=30.0)


# ═══════════════════════════════════════════════════════════
#  RelayStateMachine — classify helper
# ═══════════════════════════════════════════════════════════


class TestClassify:
    """Tests for RelayStateMachine._classify (static)."""

    def test_positive_above_threshold_is_discharging(self):
        assert RelayStateMachine._classify(100.0) == RelayState.DISCHARGING

    def test_negative_below_threshold_is_charging(self):
        assert RelayStateMachine._classify(-100.0) == RelayState.CHARGING

    def test_within_threshold_is_idle(self):
        assert RelayStateMachine._classify(0.0) == RelayState.IDLE
        assert RelayStateMachine._classify(5.0) == RelayState.IDLE
        assert RelayStateMachine._classify(-5.0) == RelayState.IDLE

    def test_exact_threshold_is_idle(self):
        assert RelayStateMachine._classify(DIRECTION_THRESHOLD_W) == RelayState.IDLE
        assert RelayStateMachine._classify(-DIRECTION_THRESHOLD_W) == RelayState.IDLE

    def test_just_above_threshold_is_discharging(self):
        assert RelayStateMachine._classify(DIRECTION_THRESHOLD_W + 0.1) == RelayState.DISCHARGING

    def test_just_below_neg_threshold_is_charging(self):
        assert RelayStateMachine._classify(-DIRECTION_THRESHOLD_W - 0.1) == RelayState.CHARGING


# ═══════════════════════════════════════════════════════════
#  RelayStateMachine — clamp helper
# ═══════════════════════════════════════════════════════════


class TestClamp:
    """Tests for RelayStateMachine._clamp_for_state (static)."""

    def test_idle_always_zero(self):
        assert RelayStateMachine._clamp_for_state(RelayState.IDLE, 500.0) == 0.0
        assert RelayStateMachine._clamp_for_state(RelayState.IDLE, -500.0) == 0.0

    def test_discharging_floor(self):
        """In DISCHARGING, small positive values are floored to MIN_ACTIVE_POWER_W."""
        assert RelayStateMachine._clamp_for_state(RelayState.DISCHARGING, 5.0) == MIN_ACTIVE_POWER_W

    def test_discharging_passthrough(self):
        """In DISCHARGING, values above floor pass through."""
        assert RelayStateMachine._clamp_for_state(RelayState.DISCHARGING, 200.0) == 200.0

    def test_charging_floor(self):
        """In CHARGING, small negative values are floored to -MIN_ACTIVE_POWER_W."""
        assert RelayStateMachine._clamp_for_state(RelayState.CHARGING, -5.0) == -MIN_ACTIVE_POWER_W

    def test_charging_passthrough(self):
        """In CHARGING, values below floor pass through."""
        assert RelayStateMachine._clamp_for_state(RelayState.CHARGING, -200.0) == -200.0


# ═══════════════════════════════════════════════════════════
#  RelayStateMachine — seed
# ═══════════════════════════════════════════════════════════


class TestSeed:
    """Tests for initial state seeding from device."""

    def test_seed_discharge(self):
        sm = make_sm()
        sm.seed(RelayDirection.DISCHARGE)
        assert sm.state == RelayState.DISCHARGING

    def test_seed_charge(self):
        sm = make_sm()
        sm.seed(RelayDirection.CHARGE)
        assert sm.state == RelayState.CHARGING

    def test_seed_idle(self):
        sm = make_sm()
        sm.seed(RelayDirection.IDLE)
        assert sm.state == RelayState.IDLE


# ═══════════════════════════════════════════════════════════
#  RelayStateMachine — transitions
# ═══════════════════════════════════════════════════════════


class TestTransitions:
    """Tests for relay state machine transition behavior."""

    def test_idle_to_discharge_requires_lockout(self):
        """Transition from IDLE to DISCHARGING requires adaptive lockout to settle."""
        sm = make_sm(base_lockout_s=30.0, ref_w=200.0)
        result = sm.update(200.0, now=T0)
        assert sm.state == RelayState.IDLE
        assert result == 0.0

    def test_idle_to_discharge_after_lockout(self):
        """After sustained power meets threshold, transition fires."""
        sm = make_sm(base_lockout_s=30.0, ref_w=200.0)
        sm.update(200.0, now=T0)
        sm.update(200.0, now=T0 + 15)
        assert sm.state == RelayState.IDLE
        sm.update(200.0, now=T0 + 30)
        assert sm.state == RelayState.DISCHARGING

    def test_idle_to_charge_after_lockout(self):
        """Transition to CHARGING after adaptive lockout settles."""
        sm = make_sm(base_lockout_s=30.0, ref_w=200.0)
        sm.update(-200.0, now=T0)
        sm.update(-200.0, now=T0 + 15)
        assert sm.state == RelayState.IDLE
        sm.update(-200.0, now=T0 + 30)
        assert sm.state == RelayState.CHARGING

    def test_discharge_to_idle_requires_idle_lockout(self):
        """Transition from DISCHARGING to IDLE requires idle_lockout_s to elapse."""
        sm = make_sm(base_lockout_s=30.0, idle_lockout_s=5.0)
        sm.seed(RelayDirection.DISCHARGE)
        sm.update(0.0, now=T0)
        assert sm.state == RelayState.DISCHARGING
        sm.update(0.0, now=T0 + 4.9)
        assert sm.state == RelayState.DISCHARGING
        sm.update(0.0, now=T0 + 5)
        assert sm.state == RelayState.IDLE

    def test_charge_to_idle_requires_lockout(self):
        sm = make_sm(base_lockout_s=30.0, idle_lockout_s=5.0)
        sm.seed(RelayDirection.CHARGE)
        sm.update(0.0, now=T0)
        assert sm.state == RelayState.CHARGING
        sm.update(0.0, now=T0 + 5)
        assert sm.state == RelayState.IDLE

    def test_discharge_to_charge_goes_through_lockout(self):
        """Switching from DISCHARGING to CHARGING must accumulate enough energy."""
        sm = make_sm(base_lockout_s=30.0, ref_w=200.0)
        sm.seed(RelayDirection.DISCHARGE)
        sm.update(-200.0, now=T0)
        assert sm.state == RelayState.DISCHARGING
        sm.update(-200.0, now=T0 + 15)
        assert sm.state == RelayState.DISCHARGING
        sm.update(-200.0, now=T0 + 30)
        assert sm.state == RelayState.CHARGING

    def test_charge_to_discharge_goes_through_lockout(self):
        sm = make_sm(base_lockout_s=30.0, ref_w=200.0)
        sm.seed(RelayDirection.CHARGE)
        sm.update(200.0, now=T0)
        assert sm.state == RelayState.CHARGING
        sm.update(200.0, now=T0 + 30)
        assert sm.state == RelayState.DISCHARGING

    def test_same_state_no_lockout(self):
        """Staying in the same state requires no lockout."""
        sm = make_sm(base_lockout_s=30.0)
        sm.seed(RelayDirection.DISCHARGE)
        result = sm.update(200.0, now=T0)
        assert sm.state == RelayState.DISCHARGING
        assert result == 200.0

    def test_clamped_during_lockout_discharge(self):
        """During lockout in DISCHARGING, charging request is clamped to +MIN_ACTIVE_POWER_W."""
        sm = make_sm(base_lockout_s=30.0, ref_w=200.0)
        sm.seed(RelayDirection.DISCHARGE)
        result = sm.update(-100.0, now=T0)
        assert result == MIN_ACTIVE_POWER_W
        assert sm.state == RelayState.DISCHARGING

    def test_clamped_during_lockout_charge(self):
        """During lockout in CHARGING, discharge request is clamped to -MIN_ACTIVE_POWER_W."""
        sm = make_sm(base_lockout_s=30.0, ref_w=200.0)
        sm.seed(RelayDirection.CHARGE)
        result = sm.update(100.0, now=T0)
        assert result == -MIN_ACTIVE_POWER_W
        assert sm.state == RelayState.CHARGING

    def test_direction_flip_resets_accumulator(self):
        """If desired direction flips mid-lockout, the accumulator resets."""
        sm = make_sm(base_lockout_s=30.0, ref_w=200.0)
        sm.update(200.0, now=T0)
        sm.update(200.0, now=T0 + 15)
        # Halfway there -- now flip to charge
        sm.update(-200.0, now=T0 + 16)
        sm.update(-200.0, now=T0 + 31)
        assert sm.state == RelayState.IDLE
        # Continue charging for the full 30s from flip
        sm.update(-200.0, now=T0 + 46)
        assert sm.state == RelayState.CHARGING

    def test_low_power_longer_lockout(self):
        """50 W takes 120 s to transition (from docs: threshold 6000 / 50 = 120s)."""
        sm = make_sm(base_lockout_s=30.0, ref_w=200.0)
        sm.update(50.0, now=T0)
        sm.update(50.0, now=T0 + 60)
        assert sm.state == RelayState.IDLE  # Only halfway
        sm.update(50.0, now=T0 + 120)
        assert sm.state == RelayState.DISCHARGING


# ═══════════════════════════════════════════════════════════
#  RelayStateMachine — safety timeout
# ═══════════════════════════════════════════════════════════


class TestSafetyTimeout:
    """Tests for the safety timeout that caps maximum lockout."""

    def test_safety_timeout_forces_transition(self):
        """Even at low power, safety timeout fires after RELAY_SAFETY_TIMEOUT_S."""
        sm = make_sm(base_lockout_s=30.0, ref_w=200.0)
        # 11 W is above DIRECTION_THRESHOLD_W (10) but needs 6000/11 ≈ 545s
        # to settle naturally — well above the 300s safety cap.
        sm.update(11.0, now=T0)
        sm.update(11.0, now=T0 + RELAY_SAFETY_TIMEOUT_S - 1)
        assert sm.state == RelayState.IDLE
        sm.update(11.0, now=T0 + RELAY_SAFETY_TIMEOUT_S)
        assert sm.state == RelayState.DISCHARGING

    def test_safety_timeout_on_charge(self):
        sm = make_sm(base_lockout_s=30.0, ref_w=200.0)
        sm.update(-11.0, now=T0)
        sm.update(-11.0, now=T0 + RELAY_SAFETY_TIMEOUT_S)
        assert sm.state == RelayState.CHARGING


# ═══════════════════════════════════════════════════════════
#  RelayStateMachine — publish callback
# ═══════════════════════════════════════════════════════════


class TestPublish:
    """Tests for state machine sensor publication."""

    def test_publish_calls_callback(self):
        published = {}

        def capture(name, value, unit=None, icon=None):
            published[name] = value

        sm = RelayStateMachine(
            idle_lockout_s=5.0,
            charge_lockout=AdaptiveLockout(full_power_w=200.0),
            discharge_lockout=AdaptiveLockout(full_power_w=200.0),
            base_lockout_s=30.0,
            publish_fn=capture,
        )
        sm.publish()
        assert published["relay_sm_state"] == "idle"
        assert published["relay_sm_pending"] == "none"
        assert published["relay_sm_charge_pct"] == "0"
        assert published["relay_sm_discharge_pct"] == "0"
        assert published["relay_sm_idle_pct"] == "0"
        # Unified lockout sensors — no pending transition
        assert published["relay_sm_lockout_pct"] == "0"
        assert published["relay_sm_accumulated_ws"] == "0"
        assert published["relay_sm_threshold_ws"] == "0"

    def test_publish_shows_pending_progress(self):
        published = {}

        def capture(name, value, unit=None, icon=None):
            published[name] = value

        sm = RelayStateMachine(
            idle_lockout_s=5.0,
            charge_lockout=AdaptiveLockout(full_power_w=200.0),
            discharge_lockout=AdaptiveLockout(full_power_w=200.0),
            base_lockout_s=30.0,
            publish_fn=capture,
        )
        sm.update(200.0, now=T0)
        sm.update(200.0, now=T0 + 15)
        sm.publish()
        assert published["relay_sm_pending"] == "discharging"
        pct = int(published["relay_sm_discharge_pct"])
        assert 49 <= pct <= 51  # ~50%
        # Unified lockout mirrors the active transition
        unified_pct = int(published["relay_sm_lockout_pct"])
        assert 49 <= unified_pct <= 51
        assert published["relay_sm_accumulated_ws"] == "3000"
        assert published["relay_sm_threshold_ws"] == "6000"

    def test_publish_charge_progress(self):
        """Charging transition publishes accumulated energy and threshold."""
        published = {}

        def capture(name, value, unit=None, icon=None):
            published[name] = value

        sm = RelayStateMachine(
            idle_lockout_s=5.0,
            charge_lockout=AdaptiveLockout(full_power_w=200.0),
            discharge_lockout=AdaptiveLockout(full_power_w=200.0),
            base_lockout_s=30.0,
            publish_fn=capture,
        )
        sm.update(-200.0, now=T0)
        sm.update(-200.0, now=T0 + 10)
        sm.publish()
        assert published["relay_sm_pending"] == "charging"
        assert published["relay_sm_accumulated_ws"] == "2000"
        assert published["relay_sm_threshold_ws"] == "6000"
        pct = int(published["relay_sm_charge_pct"])
        assert 32 <= pct <= 34  # ~33%

    def test_no_publish_without_callback(self):
        """publish() with no callback doesn't crash."""
        sm = make_sm()
        sm.publish()  # Should not raise


# ═══════════════════════════════════════════════════════════
#  Rounding helper
# ═══════════════════════════════════════════════════════════


class TestRounding:
    """Tests for the static rounding helper."""

    def test_exact_multiple(self):
        assert ZendureSolarFlowDriver._round_to_step(100.0) == 100

    def test_rounds_up(self):
        assert ZendureSolarFlowDriver._round_to_step(106.0) == 110

    def test_rounds_down(self):
        assert ZendureSolarFlowDriver._round_to_step(104.0) == 100

    def test_negative(self):
        assert ZendureSolarFlowDriver._round_to_step(-106.0) == -110

    def test_zero(self):
        assert ZendureSolarFlowDriver._round_to_step(0.0) == 0

    def test_small_value(self):
        assert ZendureSolarFlowDriver._round_to_step(3.0) == 0

    def test_midpoint_bankers_rounding(self):
        """Python uses banker's rounding: 0.5 rounds to nearest even."""
        assert ZendureSolarFlowDriver._round_to_step(5.0) == 0   # round(0.5) = 0
        assert ZendureSolarFlowDriver._round_to_step(15.0) == 20  # round(1.5) = 2
        assert ZendureSolarFlowDriver._round_to_step(25.0) == 20  # round(2.5) = 2
        assert ZendureSolarFlowDriver._round_to_step(105.0) == 100  # round(10.5) = 10


# ═══════════════════════════════════════════════════════════
#  Integration: full relay cycle
# ═══════════════════════════════════════════════════════════


class TestFullRelayCycle:
    """End-to-end tests simulating typical relay transitions."""

    def test_idle_discharge_idle_cycle(self):
        """IDLE -> DISCHARGING -> IDLE full cycle."""
        sm = make_sm(base_lockout_s=30.0, idle_lockout_s=5.0, ref_w=200.0)
        assert sm.state == RelayState.IDLE

        # Ramp up discharge for 30s
        sm.update(200.0, now=T0)
        sm.update(200.0, now=T0 + 30)
        assert sm.state == RelayState.DISCHARGING

        # Run at discharge for a while
        result = sm.update(200.0, now=T0 + 60)
        assert result == 200.0

        # Request idle
        sm.update(0.0, now=T0 + 61)
        assert sm.state == RelayState.DISCHARGING  # Idle lockout pending
        sm.update(0.0, now=T0 + 66)
        assert sm.state == RelayState.IDLE

    def test_idle_charge_discharge_cycle(self):
        """IDLE -> CHARGING -> DISCHARGING with full lockouts."""
        sm = make_sm(base_lockout_s=30.0, idle_lockout_s=5.0, ref_w=200.0)

        # Enter charging
        sm.update(-200.0, now=T0)
        sm.update(-200.0, now=T0 + 30)
        assert sm.state == RelayState.CHARGING

        # Now switch to discharging
        sm.update(200.0, now=T0 + 31)
        assert sm.state == RelayState.CHARGING  # Lockout pending
        sm.update(200.0, now=T0 + 46)
        assert sm.state == RelayState.CHARGING  # Still not enough
        sm.update(200.0, now=T0 + 61)
        assert sm.state == RelayState.DISCHARGING

    def test_marginal_power_stays_idle(self):
        """Power within the direction threshold stays IDLE."""
        sm = make_sm()
        sm.update(5.0, now=T0)
        sm.update(5.0, now=T0 + 100)
        assert sm.state == RelayState.IDLE

    def test_oscillating_input_prevents_transition(self):
        """Rapid direction changes keep resetting the accumulator."""
        sm = make_sm(base_lockout_s=30.0, ref_w=200.0)
        t = T0
        for _ in range(10):
            sm.update(200.0, t)
            t += 10.0
            sm.update(-200.0, t)
            t += 10.0
        # Never commits to one direction long enough
        assert sm.state == RelayState.IDLE
