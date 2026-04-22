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
    RELAY_SWITCH_DELAY_S,
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


def make_lockout(cutoff_w: float = 1.0) -> AdaptiveLockout:
    """Create an AdaptiveLockout with default cutoff."""
    return AdaptiveLockout(cutoff_w=cutoff_w)


def make_sm(
    threshold_ws: float = 6000.0,
    idle_lockout_s: float = 5.0,
    cutoff_w: float = 1.0,
    min_active_power_w: float = MIN_ACTIVE_POWER_W,
    log_fn=None,
) -> RelayStateMachine:
    """Create a RelayStateMachine with test defaults."""
    return RelayStateMachine(
        idle_lockout_s=idle_lockout_s,
        charge_lockout=AdaptiveLockout(cutoff_w=cutoff_w),
        discharge_lockout=AdaptiveLockout(cutoff_w=cutoff_w),
        threshold_ws=threshold_ws,
        min_active_power_w=min_active_power_w,
        log_fn=log_fn,
    )


# ═══════════════════════════════════════════════════════════
#  AdaptiveLockout
# ═══════════════════════════════════════════════════════════


class TestAdaptiveLockout:
    """Tests for the energy-integrating lockout timer."""

    def test_fresh_lockout_not_settled(self):
        al = make_lockout()
        assert not al.is_settled(6000.0)

    def test_reset_clears_accumulator(self):
        al = make_lockout()
        al.tick(200, now=T0)
        al.tick(200, now=T0 + 10)
        al.reset()
        assert al.progress == 0.0
        assert not al.is_settled(6000.0)

    def test_single_tick_no_accumulation(self):
        """First tick only sets the timestamp, no dt available yet."""
        al = make_lockout()
        al.tick(200, now=T0)
        assert al.progress == 0.0

    def test_threshold_calculation(self):
        al = make_lockout()
        assert al.threshold(6000.0) == 6000.0

    # -- Doc examples: full_power_w=200, base=30s, threshold=6000 W·s --

    def test_200w_sustained_settles_in_30s(self):
        """200 W sustained -> 30 s (from docs table)."""
        al = make_lockout()
        al.tick(200, now=T0)
        al.tick(200, now=T0 + 30)
        assert al.is_settled(6000.0)

    def test_200w_just_under_30s_not_settled(self):
        al = make_lockout()
        al.tick(200, now=T0)
        al.tick(200, now=T0 + 29.9)
        assert not al.is_settled(6000.0)

    def test_100w_sustained_settles_in_60s(self):
        """100 W sustained -> 60 s (from docs table)."""
        al = make_lockout()
        al.tick(100, now=T0)
        al.tick(100, now=T0 + 60)
        assert al.is_settled(6000.0)

    def test_100w_just_under_60s_not_settled(self):
        al = make_lockout()
        al.tick(100, now=T0)
        al.tick(100, now=T0 + 59)
        assert not al.is_settled(6000.0)

    def test_50w_sustained_settles_in_120s(self):
        """50 W sustained -> 120 s (from docs table)."""
        al = make_lockout()
        al.tick(50, now=T0)
        al.tick(50, now=T0 + 120)
        assert al.is_settled(6000.0)

    def test_20w_sustained_settles_in_300s(self):
        """<20 W -> 300 s (from docs table)."""
        al = make_lockout()
        al.tick(20, now=T0)
        al.tick(20, now=T0 + 300)
        assert al.is_settled(6000.0)

    def test_variable_power_proportional(self):
        """Variable power: each slice contributes proportionally."""
        al = make_lockout()
        # 200 W for 15 s -> 3000 W-s
        al.tick(200, now=T0)
        al.tick(200, now=T0 + 15)
        assert al.progress == pytest.approx(3000.0)
        assert not al.is_settled(6000.0)
        # Then 100 W for 30 s -> 3000 W-s more -> total 6000 -> settled
        al.tick(100, now=T0 + 45)
        assert al.progress == pytest.approx(6000.0)
        assert al.is_settled(6000.0)

    def test_negative_power_uses_abs(self):
        """Charging (negative) power contributes via absolute value."""
        al = make_lockout()
        al.tick(-200, now=T0)
        al.tick(-200, now=T0 + 30)
        assert al.is_settled(6000.0)

    def test_zero_power_uses_cutoff(self):
        """Zero power still accumulates at the cutoff rate."""
        al = make_lockout(cutoff_w=1.0)
        al.tick(0, now=T0)
        al.tick(0, now=T0 + 100)
        assert al.progress == pytest.approx(100.0)

    def test_cutoff_prevents_infinite_lockout(self):
        """Cutoff floor ensures progress even at very low power."""
        al = make_lockout(cutoff_w=25.0)
        al.tick(5, now=T0)        # 5 < 25 → effective = 25
        al.tick(5, now=T0 + 240)  # 25 × 240 = 6000
        assert al.is_settled(6000.0)

    def test_reset_then_reaccumulate(self):
        """After reset, accumulation restarts from zero."""
        al = make_lockout()
        al.tick(200, now=T0)
        al.tick(200, now=T0 + 15)
        assert al.progress == pytest.approx(3000.0)
        al.reset()
        al.tick(200, now=T0 + 20)
        al.tick(200, now=T0 + 50)
        assert al.progress == pytest.approx(6000.0)
        assert al.is_settled(6000.0)


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
        sm = make_sm()
        result = sm.update(200.0, now=T0)
        assert sm.state == RelayState.IDLE
        assert result == 0.0

    def test_idle_to_discharge_after_lockout(self):
        """After sustained power meets threshold, transition fires."""
        sm = make_sm()
        sm.update(200.0, now=T0)
        sm.update(200.0, now=T0 + 15)
        assert sm.state == RelayState.IDLE
        sm.update(200.0, now=T0 + 30)
        assert sm.state == RelayState.DISCHARGING

    def test_idle_to_charge_after_lockout(self):
        """Transition to CHARGING after adaptive lockout settles."""
        sm = make_sm()
        sm.update(-200.0, now=T0)
        sm.update(-200.0, now=T0 + 15)
        assert sm.state == RelayState.IDLE
        sm.update(-200.0, now=T0 + 30)
        assert sm.state == RelayState.CHARGING

    def test_discharge_to_idle_requires_idle_lockout(self):
        """Transition from DISCHARGING to IDLE requires idle_lockout_s to elapse."""
        sm = make_sm(idle_lockout_s=5.0)
        sm.seed(RelayDirection.DISCHARGE)
        sm.update(0.0, now=T0)
        assert sm.state == RelayState.DISCHARGING
        sm.update(0.0, now=T0 + 4.9)
        assert sm.state == RelayState.DISCHARGING
        sm.update(0.0, now=T0 + 5)
        assert sm.state == RelayState.IDLE

    def test_charge_to_idle_requires_lockout(self):
        sm = make_sm(idle_lockout_s=5.0)
        sm.seed(RelayDirection.CHARGE)
        sm.update(0.0, now=T0)
        assert sm.state == RelayState.CHARGING
        sm.update(0.0, now=T0 + 5)
        assert sm.state == RelayState.IDLE

    def test_discharge_to_charge_goes_through_lockout(self):
        """Switching from DISCHARGING to CHARGING must accumulate enough energy."""
        sm = make_sm()
        sm.seed(RelayDirection.DISCHARGE)
        sm.update(-200.0, now=T0)
        assert sm.state == RelayState.DISCHARGING
        sm.update(-200.0, now=T0 + 15)
        assert sm.state == RelayState.DISCHARGING
        sm.update(-200.0, now=T0 + 30)
        assert sm.state == RelayState.CHARGING

    def test_charge_to_discharge_goes_through_lockout(self):
        sm = make_sm()
        sm.seed(RelayDirection.CHARGE)
        sm.update(200.0, now=T0)
        assert sm.state == RelayState.CHARGING
        sm.update(200.0, now=T0 + 30)
        assert sm.state == RelayState.DISCHARGING

    def test_same_state_no_lockout(self):
        """Staying in the same state requires no lockout."""
        sm = make_sm()
        sm.seed(RelayDirection.DISCHARGE)
        result = sm.update(200.0, now=T0)
        assert sm.state == RelayState.DISCHARGING
        assert result == 200.0

    def test_clamped_during_lockout_discharge(self):
        """During lockout in DISCHARGING, charging request is clamped to +MIN_ACTIVE_POWER_W."""
        sm = make_sm()
        sm.seed(RelayDirection.DISCHARGE)
        result = sm.update(-100.0, now=T0)
        assert result == MIN_ACTIVE_POWER_W
        assert sm.state == RelayState.DISCHARGING

    def test_clamped_during_lockout_charge(self):
        """During lockout in CHARGING, discharge request is clamped to -MIN_ACTIVE_POWER_W."""
        sm = make_sm()
        sm.seed(RelayDirection.CHARGE)
        result = sm.update(100.0, now=T0)
        assert result == -MIN_ACTIVE_POWER_W
        assert sm.state == RelayState.CHARGING

    def test_direction_flip_new_target_starts_fresh(self):
        """Flipping from DISCHARGE to CHARGE target while in IDLE:
        the new charge accumulator starts from zero (no prior ticks),
        but the old discharge accumulator preserves its progress."""
        sm = make_sm()
        sm.update(200.0, now=T0)
        sm.update(200.0, now=T0 + 15)
        # Halfway there -- now flip to charge
        sm.update(-200.0, now=T0 + 16)
        sm.update(-200.0, now=T0 + 31)
        assert sm.state == RelayState.IDLE
        # Continue charging for the full 30s from flip
        sm.update(-200.0, now=T0 + 46)
        assert sm.state == RelayState.CHARGING

    def test_direction_flip_preserves_other_accumulator(self):
        """Flipping target back after a brief interruption resumes
        from the preserved accumulator progress."""
        sm = make_sm()
        sm.update(200.0, now=T0)
        sm.update(200.0, now=T0 + 15)  # discharge acc = 3000 W·s
        # Briefly flip to charge for one tick
        sm.update(-200.0, now=T0 + 16)
        # Flip back to discharge — acc preserved at 3000 W·s
        sm.update(200.0, now=T0 + 17)  # first tick after pause (sets ts)
        sm.update(200.0, now=T0 + 32)  # +3000 → total 6000 → settled
        assert sm.state == RelayState.DISCHARGING

    def test_oscillation_idle_discharge_while_charging(self):
        """In CHARGING, oscillating between IDLE and DISCHARGE targets
        must still allow the IDLE timer to accumulate and eventually fire."""
        sm = make_sm(idle_lockout_s=5.0)
        sm.seed(RelayDirection.CHARGE)

        # Alternating: 2 s IDLE, 1 s DISCHARGE — IDLE accumulates ~2 s per cycle
        sm.update(0.0, now=T0)          # IDLE first tick (sets ts)
        sm.update(0.0, now=T0 + 2)     # IDLE acc = 2 s
        sm.update(200.0, now=T0 + 3)   # DISCHARGE (pauses idle tick clock)
        sm.update(0.0, now=T0 + 4)     # IDLE first tick after pause (sets ts)
        sm.update(0.0, now=T0 + 6)     # IDLE acc = 2 + 2 = 4 s
        assert sm.state == RelayState.CHARGING  # not yet at 5 s

        sm.update(200.0, now=T0 + 7)   # DISCHARGE (pauses idle)
        sm.update(0.0, now=T0 + 8)     # IDLE first tick (sets ts)
        sm.update(0.0, now=T0 + 10)    # IDLE acc = 4 + 2 = 6 s → settled
        assert sm.state == RelayState.IDLE

    def test_oscillation_idle_charge_while_discharging(self):
        """In DISCHARGING, oscillating between IDLE and CHARGE targets
        must still allow the IDLE timer to accumulate and eventually fire."""
        sm = make_sm(idle_lockout_s=5.0)
        sm.seed(RelayDirection.DISCHARGE)

        sm.update(0.0, now=T0)
        sm.update(0.0, now=T0 + 2)
        sm.update(-200.0, now=T0 + 3)
        sm.update(0.0, now=T0 + 4)
        sm.update(0.0, now=T0 + 6)
        assert sm.state == RelayState.DISCHARGING  # 4 s < 5 s

        sm.update(-200.0, now=T0 + 7)
        sm.update(0.0, now=T0 + 8)
        sm.update(0.0, now=T0 + 10)
        assert sm.state == RelayState.IDLE  # 6 s ≥ 5 s

    def test_low_power_longer_lockout(self):
        """50 W takes 120 s to transition (from docs: threshold 6000 / 50 = 120s)."""
        sm = make_sm()
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
        sm = make_sm(threshold_ws=12000.0)
        # 11 W is above DIRECTION_THRESHOLD_W (10) but needs 12000/11 ≈ 1091s
        # to settle naturally — well above the safety cap.
        sm.update(11.0, now=T0)
        sm.update(11.0, now=T0 + RELAY_SAFETY_TIMEOUT_S - 1)
        assert sm.state == RelayState.IDLE
        sm.update(11.0, now=T0 + RELAY_SAFETY_TIMEOUT_S)
        assert sm.state == RelayState.DISCHARGING

    def test_safety_timeout_on_charge(self):
        sm = make_sm(threshold_ws=12000.0)
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
            charge_lockout=AdaptiveLockout(cutoff_w=1.0),
            discharge_lockout=AdaptiveLockout(cutoff_w=1.0),
            threshold_ws=6000.0,
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
            charge_lockout=AdaptiveLockout(cutoff_w=1.0),
            discharge_lockout=AdaptiveLockout(cutoff_w=1.0),
            threshold_ws=6000.0,
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
            charge_lockout=AdaptiveLockout(cutoff_w=1.0),
            discharge_lockout=AdaptiveLockout(cutoff_w=1.0),
            threshold_ws=6000.0,
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
        assert ZendureSolarFlowDriver._round_to_step(103.0) == 105

    def test_rounds_down(self):
        assert ZendureSolarFlowDriver._round_to_step(101.0) == 100

    def test_negative(self):
        assert ZendureSolarFlowDriver._round_to_step(-103.0) == -105

    def test_zero(self):
        assert ZendureSolarFlowDriver._round_to_step(0.0) == 0

    def test_small_value(self):
        assert ZendureSolarFlowDriver._round_to_step(2.0) == 0

    def test_midpoint_bankers_rounding(self):
        """Python uses banker's rounding: 0.5 rounds to nearest even."""
        assert ZendureSolarFlowDriver._round_to_step(2.5) == 0    # round(0.5) = 0
        assert ZendureSolarFlowDriver._round_to_step(7.5) == 10   # round(1.5) = 2
        assert ZendureSolarFlowDriver._round_to_step(12.5) == 10  # round(2.5) = 2
        assert ZendureSolarFlowDriver._round_to_step(52.5) == 50  # round(10.5) = 10


# ═══════════════════════════════════════════════════════════
#  Integration: full relay cycle
# ═══════════════════════════════════════════════════════════


class TestFullRelayCycle:
    """End-to-end tests simulating typical relay transitions."""

    def test_idle_discharge_idle_cycle(self):
        """IDLE -> DISCHARGING -> IDLE full cycle."""
        sm = make_sm(idle_lockout_s=5.0)
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
        sm = make_sm(idle_lockout_s=5.0)

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
        sm = make_sm()
        t = T0
        for _ in range(10):
            sm.update(200.0, t)
            t += 10.0
            sm.update(-200.0, t)
            t += 10.0
        # Never commits to one direction long enough
        assert sm.state == RelayState.IDLE


# ═══════════════════════════════════════════════════════════
#  Driver Config.from_args
# ═══════════════════════════════════════════════════════════


class TestDriverConfig:
    """Tests for driver Config.from_args mapping."""

    MINIMAL_ARGS = {
        "desired_power_sensor": "sensor.zfi_desired_power",
        "output_limit_entity": "number.output_limit",
        "input_limit_entity": "number.input_limit",
        "ac_mode_entity": "select.ac_mode",
    }

    def test_minimal_args_uses_defaults(self):
        from src.zendure_solarflow_driver import Config as DriverConfig

        cfg = DriverConfig.from_args(self.MINIMAL_ARGS)
        assert cfg.desired_power_sensor == "sensor.zfi_desired_power"
        assert cfg.output_entity == "number.output_limit"
        assert cfg.input_entity == "number.input_limit"
        assert cfg.ac_mode_entity == "select.ac_mode"
        assert cfg.watchdog_s == 15
        assert cfg.relay_lockout_idle_s == 5.0
        assert cfg.relay_sm_enabled is True
        assert cfg.relay_lockout_cutoff_w == 1.0
        assert cfg.dry_run is True
        assert cfg.debug is False
        assert cfg.sensor_prefix == "sensor.zfi"

    def test_custom_args_override_defaults(self):
        from src.zendure_solarflow_driver import Config as DriverConfig

        args = {
            **self.MINIMAL_ARGS,
            "watchdog_s": 5,
            "relay_lockout_idle_s": 10.0,
            "relay_sm_enabled": False,
            "relay_lockout_cutoff_w": 50.0,
            "dry_run": False,
            "debug": True,
            "sensor_prefix": "sensor.test",
        }
        cfg = DriverConfig.from_args(args)
        assert cfg.watchdog_s == 5
        assert cfg.relay_lockout_idle_s == 10.0
        assert cfg.relay_sm_enabled is False
        assert cfg.relay_lockout_cutoff_w == 50.0
        assert cfg.dry_run is False
        assert cfg.debug is True
        assert cfg.sensor_prefix == "sensor.test"


# ═══════════════════════════════════════════════════════════
#  DriverState defaults
# ═══════════════════════════════════════════════════════════


class TestDriverState:
    """Tests for DriverState default values."""

    def test_defaults(self):
        from src.zendure_solarflow_driver import DriverState

        ds = DriverState()
        assert ds.last_sent_discharge_w == -1
        assert ds.last_sent_charge_w == -1
        assert ds.last_relay_change_t == 0.0
        assert ds.last_set_relay == RelayDirection.IDLE


# ═══════════════════════════════════════════════════════════
#  RelayStateMachine — logging callback
# ═══════════════════════════════════════════════════════════


class TestStateMachineLogging:
    """Tests that state machine transitions invoke the log callback."""

    def test_log_on_transition(self):
        logged = []
        sm = make_sm(log_fn=logged.append)
        sm.update(200.0, now=T0)
        sm.update(200.0, now=T0 + 30)
        assert sm.state == RelayState.DISCHARGING
        assert any("IDLE" in msg and "DISCHARGING" in msg for msg in logged)

    def test_no_log_when_stable(self):
        logged = []
        sm = make_sm(log_fn=logged.append)
        sm.seed(RelayDirection.DISCHARGE)
        sm.update(200.0, now=T0)
        sm.update(200.0, now=T0 + 10)
        assert len(logged) == 0

    def test_no_log_without_callback(self):
        """No crash when log_fn is None."""
        sm = make_sm(log_fn=None)
        sm.update(200.0, now=T0)
        sm.update(200.0, now=T0 + 30)
        assert sm.state == RelayState.DISCHARGING


# ═══════════════════════════════════════════════════════════
#  RelayStateMachine — lockout_progress for IDLE target
# ═══════════════════════════════════════════════════════════


class TestLockoutProgress:
    """Tests for _lockout_progress for different targets."""

    def test_no_pending_returns_zeros(self):
        sm = make_sm()
        pct, acc, thresh = sm._lockout_progress()
        assert pct == 0.0
        assert acc == 0.0
        assert thresh == 0.0

    def test_discharge_pending_progress(self):
        sm = make_sm()
        sm.update(200.0, now=T0)
        sm.update(200.0, now=T0 + 15)
        pct, acc, thresh = sm._lockout_progress()
        assert thresh == pytest.approx(6000.0)
        assert acc == pytest.approx(3000.0)
        assert 49 <= pct <= 51

    def test_charge_pending_progress(self):
        sm = make_sm()
        sm.update(-200.0, now=T0)
        sm.update(-200.0, now=T0 + 15)
        pct, acc, thresh = sm._lockout_progress()
        assert thresh == pytest.approx(6000.0)
        assert acc == pytest.approx(3000.0)
        assert 49 <= pct <= 51

    def test_idle_pending_progress(self):
        sm = make_sm(idle_lockout_s=10.0)
        sm.seed(RelayDirection.DISCHARGE)
        sm.update(0.0, now=T0)
        sm.update(0.0, now=T0 + 5)
        pct, acc, thresh = sm._lockout_progress()
        assert thresh == pytest.approx(10.0)
        assert acc == pytest.approx(5.0)
        assert 49 <= pct <= 51


# ═══════════════════════════════════════════════════════════
#  RelayStateMachine — publish with idle pending
# ═══════════════════════════════════════════════════════════


class TestPublishIdlePending:
    """Tests for publish when idle transition is pending."""

    def test_publish_idle_progress(self):
        published = {}

        def capture(name, value, unit=None, icon=None):
            published[name] = value

        sm = RelayStateMachine(
            idle_lockout_s=10.0,
            charge_lockout=AdaptiveLockout(cutoff_w=1.0),
            discharge_lockout=AdaptiveLockout(cutoff_w=1.0),
            threshold_ws=6000.0,
            publish_fn=capture,
        )
        sm.seed(RelayDirection.DISCHARGE)
        sm.update(0.0, now=T0)
        sm.update(0.0, now=T0 + 5)
        sm.publish()
        assert published["relay_sm_state"] == "discharging"
        assert published["relay_sm_pending"] == "idle"
        pct = int(published["relay_sm_idle_pct"])
        assert 49 <= pct <= 51

    def test_publish_after_transition_resets(self):
        """After a completed transition, publish shows no pending."""
        published = {}

        def capture(name, value, unit=None, icon=None):
            published[name] = value

        sm = RelayStateMachine(
            idle_lockout_s=5.0,
            charge_lockout=AdaptiveLockout(cutoff_w=1.0),
            discharge_lockout=AdaptiveLockout(cutoff_w=1.0),
            threshold_ws=6000.0,
            publish_fn=capture,
        )
        sm.update(200.0, now=T0)
        sm.update(200.0, now=T0 + 30)  # transition fires
        sm.publish()
        assert published["relay_sm_state"] == "discharging"
        assert published["relay_sm_pending"] == "none"
        assert published["relay_sm_lockout_pct"] == "0"


# ═══════════════════════════════════════════════════════════
#  RelayStateMachine — departure_since reset
# ═══════════════════════════════════════════════════════════


class TestDepartureSince:
    """Tests for departure_since tracking."""

    def test_departure_since_set_on_first_departure(self):
        sm = make_sm()
        assert sm._departure_since == 0.0
        sm.update(200.0, now=T0)
        assert sm._departure_since == T0

    def test_departure_since_not_reset_by_subsequent_ticks(self):
        sm = make_sm()
        sm.update(200.0, now=T0)
        sm.update(200.0, now=T0 + 10)
        assert sm._departure_since == T0  # Still first departure time

    def test_departure_since_reset_on_stability(self):
        sm = make_sm()
        sm.update(200.0, now=T0)
        assert sm._departure_since == T0
        # Back to stable IDLE
        sm.update(0.0, now=T0 + 1)  # first idle tick
        sm.update(0.0, now=T0 + 2)  # idle but still pending
        # Force transition via safety timeout
        sm = make_sm()
        sm.update(200.0, now=T0)
        sm.update(200.0, now=T0 + 30)
        assert sm.state == RelayState.DISCHARGING
        assert sm._departure_since == 0.0  # Reset after transition

    def test_departure_since_reset_on_target_matches_state(self):
        sm = make_sm()
        sm.seed(RelayDirection.DISCHARGE)
        sm.update(-200.0, now=T0)  # wants CHARGING
        assert sm._departure_since == T0
        sm.update(200.0, now=T0 + 5)  # back to DISCHARGING (matches state)
        assert sm._departure_since == 0.0


# ═══════════════════════════════════════════════════════════
#  AdaptiveLockout — additional edge cases
# ═══════════════════════════════════════════════════════════


class TestAdaptiveLockoutEdgeCases:
    def test_tick_with_zero_last_tick_t(self):
        """First tick after reset sets timestamp without accumulating."""
        al = make_lockout()
        al.reset()
        al.tick(200, now=T0)
        assert al.progress == 0.0

    def test_multiple_resets(self):
        """Multiple resets don't break accumulation."""
        al = make_lockout()
        al.tick(200, now=T0)
        al.tick(200, now=T0 + 10)
        al.reset()
        al.reset()
        al.tick(200, now=T0 + 20)
        al.tick(200, now=T0 + 30)
        assert al.progress == pytest.approx(2000.0)

    def test_very_small_power(self):
        """Very small power requires proportionally longer time."""
        al = make_lockout()
        al.tick(1, now=T0)
        al.tick(1, now=T0 + 6000)  # 1 W × 6000 s = 6000 W·s
        assert al.is_settled(6000.0)

    def test_negative_dt_ignored(self):
        """If timestamps go backwards, dt is not accumulated (dt <= 0)."""
        al = make_lockout()
        al.tick(200, now=T0 + 10)
        al.tick(200, now=T0 + 5)  # backwards
        assert al.progress == 0.0


# ═══════════════════════════════════════════════════════════
#  RelayStateMachine — update return values
# ═══════════════════════════════════════════════════════════


class TestUpdateReturnValues:
    """Tests verifying the returned (clamped) power from update()."""

    def test_idle_returns_zero(self):
        sm = make_sm()
        result = sm.update(5.0, now=T0)
        assert result == 0.0

    def test_discharging_returns_desired(self):
        sm = make_sm(threshold_ws=2.0)
        sm.seed(RelayDirection.DISCHARGE)
        result = sm.update(200.0, now=T0)
        assert result == 200.0

    def test_charging_returns_desired(self):
        sm = make_sm(threshold_ws=2.0)
        sm.seed(RelayDirection.CHARGE)
        result = sm.update(-200.0, now=T0)
        assert result == -200.0

    def test_idle_clamps_charge_request_to_zero(self):
        sm = make_sm()
        result = sm.update(-5.0, now=T0)
        assert result == 0.0

    def test_transition_returns_clamped_first_tick(self):
        """First tick after transition returns clamped value for new state."""
        sm = make_sm()
        sm.update(200.0, now=T0)
        result = sm.update(200.0, now=T0 + 30)
        assert sm.state == RelayState.DISCHARGING
        assert result == 200.0


# ═══════════════════════════════════════════════════════════
#  Relay switch delay holdoff
# ═══════════════════════════════════════════════════════════


def _sim_relay_locked(
    sm: RelayStateMachine,
    desired: float,
    now: float,
    last_transition_t: float,
) -> tuple[bool, float]:
    """Simulate the driver's relay_locked logic.

    Returns (relay_locked, updated last_transition_t).
    """
    prev_state = sm.state
    allowed = sm.update(desired, now)
    if sm.state != prev_state:
        last_transition_t = now
    sm_clamping = allowed != desired
    relay_switching = (now - last_transition_t) < RELAY_SWITCH_DELAY_S
    relay_locked = sm_clamping or relay_switching
    return relay_locked, last_transition_t


class TestRelaySwitchDelay:
    """Tests for the 8-second holdoff on relay_locked after SM transitions."""

    def test_relay_locked_true_immediately_after_transition(self):
        """relay_locked stays true right after SM transitions."""
        sm = make_sm()
        last_t = -RELAY_SWITCH_DELAY_S  # no holdoff at start
        # Request discharge until SM transitions
        sm.update(200.0, now=T0)
        locked, last_t = _sim_relay_locked(sm, 200.0, T0 + 30, last_t)
        assert sm.state == RelayState.DISCHARGING
        # SM just transitioned — allowed==desired but holdoff active
        assert locked is True

    def test_relay_locked_true_during_holdoff(self):
        """relay_locked stays true within RELAY_SWITCH_DELAY_S of transition."""
        sm = make_sm()
        last_t = -RELAY_SWITCH_DELAY_S
        sm.update(200.0, now=T0)
        _, last_t = _sim_relay_locked(sm, 200.0, T0 + 30, last_t)
        # 7 seconds after transition — still within holdoff
        locked, last_t = _sim_relay_locked(sm, 200.0, T0 + 30 + 7, last_t)
        assert locked is True

    def test_relay_locked_false_after_holdoff(self):
        """relay_locked goes false once holdoff expires and SM is stable."""
        sm = make_sm()
        last_t = -RELAY_SWITCH_DELAY_S
        sm.update(200.0, now=T0)
        _, last_t = _sim_relay_locked(sm, 200.0, T0 + 30, last_t)
        # 8+ seconds after transition — holdoff expired
        locked, last_t = _sim_relay_locked(sm, 200.0, T0 + 30 + RELAY_SWITCH_DELAY_S, last_t)
        assert locked is False

    def test_sm_clamping_overrides_holdoff(self):
        """relay_locked is true when SM clamps, regardless of holdoff timer."""
        sm = make_sm()
        last_t = -RELAY_SWITCH_DELAY_S
        # SM is IDLE, request discharge — SM clamps to 0
        locked, last_t = _sim_relay_locked(sm, 200.0, T0, last_t)
        assert sm.state == RelayState.IDLE
        assert locked is True

    def test_no_holdoff_at_startup(self):
        """No false holdoff when driver starts (initialized to -RELAY_SWITCH_DELAY_S)."""
        sm = make_sm()
        sm.seed(RelayDirection.DISCHARGE)
        last_t = -RELAY_SWITCH_DELAY_S
        # Stable in DISCHARGING — no clamping, no holdoff
        locked, last_t = _sim_relay_locked(sm, 200.0, T0, last_t)
        assert locked is False

    def test_holdoff_resets_on_each_transition(self):
        """Each SM transition resets the holdoff timer."""
        sm = make_sm(threshold_ws=2.0, idle_lockout_s=0.01)
        last_t = -RELAY_SWITCH_DELAY_S
        # Transition to DISCHARGING (near-zero lockout)
        sm.update(200.0, now=T0)
        _, last_t = _sim_relay_locked(sm, 200.0, T0 + 0.02, last_t)
        assert sm.state == RelayState.DISCHARGING
        t_first = last_t
        # Wait out holdoff
        locked, last_t = _sim_relay_locked(sm, 200.0, t_first + RELAY_SWITCH_DELAY_S, last_t)
        assert locked is False
        # Now transition to IDLE (need idle lockout to pass too)
        t_idle_start = t_first + RELAY_SWITCH_DELAY_S + 1
        sm.update(0.0, now=t_idle_start)
        _, last_t = _sim_relay_locked(sm, 0.0, t_idle_start + 0.02, last_t)
        assert sm.state == RelayState.IDLE
        # Holdoff active again
        locked, last_t = _sim_relay_locked(sm, 0.0, last_t + 1, last_t)
        assert locked is True


# ═══════════════════════════════════════════════════════════
#  State Persistence
# ═══════════════════════════════════════════════════════════


class TestRelayStateMachinePersistence:
    """Tests for RelayStateMachine.state_snapshot / restore_from_snapshot."""

    def test_snapshot_round_trip_idle(self):
        """Snapshot captures IDLE state and restores it."""
        sm = make_sm()
        assert sm.state == RelayState.IDLE
        snap = sm.state_snapshot()
        sm2 = make_sm()
        sm2.state = RelayState.DISCHARGING  # force a different state
        assert sm2.restore_from_snapshot(snap) is True
        assert sm2.state == RelayState.IDLE

    def test_snapshot_round_trip_discharging(self):
        """Snapshot captures DISCHARGING state after transition."""
        sm = make_sm()
        sm.update(200.0, now=T0)
        sm.update(200.0, now=T0 + 30)
        assert sm.state == RelayState.DISCHARGING

        snap = sm.state_snapshot()

        sm2 = make_sm()
        assert sm2.restore_from_snapshot(snap) is True
        assert sm2.state == RelayState.DISCHARGING

    def test_snapshot_round_trip_charging(self):
        """Snapshot captures CHARGING state after transition."""
        sm = make_sm()
        sm.update(-200.0, now=T0)
        sm.update(-200.0, now=T0 + 30)
        assert sm.state == RelayState.CHARGING

        snap = sm.state_snapshot()

        sm2 = make_sm()
        assert sm2.restore_from_snapshot(snap) is True
        assert sm2.state == RelayState.CHARGING

    def test_snapshot_is_json_serializable(self):
        """Snapshot dict survives JSON round-trip."""
        import json
        sm = make_sm()
        sm.update(200.0, now=T0)
        sm.update(200.0, now=T0 + 30)

        snap = sm.state_snapshot()
        restored = json.loads(json.dumps(snap))

        sm2 = make_sm()
        assert sm2.restore_from_snapshot(restored) is True
        assert sm2.state == RelayState.DISCHARGING

    def test_restore_invalid_state_returns_false(self):
        """Restore fails on invalid state name."""
        sm = make_sm()
        assert sm.restore_from_snapshot({"state": "INVALID"}) is False

    def test_restore_missing_key_returns_false(self):
        """Restore fails when state key is missing."""
        sm = make_sm()
        assert sm.restore_from_snapshot({}) is False

    def test_restore_overrides_seed(self):
        """Restore after seed takes precedence."""
        sm = make_sm()
        sm.seed(RelayDirection.DISCHARGE)
        assert sm.state == RelayState.DISCHARGING

        sm.restore_from_snapshot({"state": "IDLE"})
        assert sm.state == RelayState.IDLE
