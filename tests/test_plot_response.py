"""Tests for tools/plot_response.py — scenario generation and plotting."""
from __future__ import annotations

import numpy as np
import pytest

from tools.plot_response import (
    ALL_SCENARIOS,
    CURRENT_GAINS,
    DEFAULT_SYS_CFG,
    Scenario,
    make_trace_arrays,
    scenario_charge_transition,
    scenario_cloud,
    scenario_low_soc,
    scenario_oscillating,
    scenario_real,
    scenario_step_down,
    scenario_step_up,
)
from tools.optimize_gains import SimResult, TraceArrays, simulate_fast

# Plant model for testing.
PLANT_ABC = (0.9094, 0.0804, -1.96)


class TestScenarios:
    """Verify each scenario generator produces valid data."""

    @pytest.mark.parametrize("name,factory", list(ALL_SCENARIOS.items()))
    def test_scenario_shape_and_types(self, name: str, factory) -> None:
        """Each scenario has matching array lengths and correct types."""
        sc = factory()
        assert isinstance(sc, Scenario)
        assert sc.name == name
        assert len(sc.disturbance) == len(sc.soc) == len(sc.dt)
        assert sc.disturbance.dtype == np.float64
        assert sc.soc.dtype == np.float64
        assert sc.dt.dtype == np.float64

    def test_step_up_values(self) -> None:
        """Step-up scenario has correct disturbance levels."""
        sc = scenario_step_up(n=100)
        assert sc.disturbance[0] == 100.0
        assert sc.disturbance[50] == 500.0
        assert sc.disturbance[99] == 500.0

    def test_step_down_values(self) -> None:
        """Step-down scenario has correct disturbance levels."""
        sc = scenario_step_down(n=100)
        assert sc.disturbance[0] == 500.0
        assert sc.disturbance[50] == 100.0

    def test_cloud_has_deficit_and_recovery(self) -> None:
        """Cloud scenario has a deficit phase and a recovery ramp."""
        sc = scenario_cloud(n=400)
        assert sc.disturbance[0] == -200.0  # initial surplus
        assert sc.disturbance[100] == 300.0  # deficit phase
        assert sc.disturbance[399] == -200.0  # recovered

    def test_charge_transition_is_monotonic(self) -> None:
        """Charge transition ramps from positive to negative."""
        sc = scenario_charge_transition(n=200)
        assert sc.disturbance[0] > 0
        assert sc.disturbance[-1] < 0
        # Should be strictly decreasing
        assert np.all(np.diff(sc.disturbance) < 0)

    def test_oscillating_has_periodicity(self) -> None:
        """Oscillating scenario has sinusoidal behavior."""
        sc = scenario_oscillating(n=600)
        assert sc.disturbance.min() < 100
        assert sc.disturbance.max() > 300

    def test_low_soc_ramps_below_min(self) -> None:
        """Low-SOC scenario ramps SOC below the guard threshold."""
        sc = scenario_low_soc(n=300)
        assert sc.soc[0] == 20.0
        assert sc.soc[-1] == pytest.approx(14.0, abs=0.1)


class TestMakeTraceArrays:
    """Verify Scenario → TraceArrays conversion."""

    def test_basic_conversion(self) -> None:
        """TraceArrays has correct shape and dtype."""
        sc = scenario_step_up(n=50)
        ta = make_trace_arrays(sc)
        assert isinstance(ta, TraceArrays)
        assert ta.n == 50
        assert ta.disturbance.dtype == np.float64
        assert ta.soc.dtype == np.float64
        assert ta.dt.dtype == np.float64
        assert len(ta.grid_actual) == 50

    def test_values_preserved(self) -> None:
        """Conversion preserves disturbance / soc / dt values."""
        sc = scenario_step_up(n=100)
        ta = make_trace_arrays(sc)
        np.testing.assert_array_equal(ta.disturbance, sc.disturbance)
        np.testing.assert_array_equal(ta.soc, sc.soc)
        np.testing.assert_array_equal(ta.dt, sc.dt)


class TestScenarioReal:
    """Verify real-data scenario extraction."""

    def test_slicing(self) -> None:
        """Real scenario correctly slices the trace."""
        traces = {
            "disturbance_w": list(range(100)),
            "soc_pct": [50.0] * 100,
            "dt_s": [5.0] * 100,
        }
        sc = scenario_real(traces, start=10, end=30)
        assert len(sc.disturbance) == 20
        assert sc.disturbance[0] == 10.0
        assert sc.disturbance[-1] == 29.0
        assert "10–30" in sc.title

    def test_default_end(self) -> None:
        """Without end, uses full trace length."""
        traces = {
            "disturbance_w": list(range(50)),
            "soc_pct": [50.0] * 50,
            "dt_s": [5.0] * 50,
        }
        sc = scenario_real(traces, start=0)
        assert len(sc.disturbance) == 50


class TestSimulationOnScenarios:
    """Run simulate_fast on each scenario — sanity checks only."""

    @pytest.mark.parametrize("name,factory", list(ALL_SCENARIOS.items()))
    def test_simulation_runs(self, name: str, factory) -> None:
        """simulate_fast completes without error on each scenario."""
        sc = factory()
        ta = make_trace_arrays(sc)
        r = simulate_fast(CURRENT_GAINS, PLANT_ABC, ta, DEFAULT_SYS_CFG)
        assert isinstance(r, SimResult)
        assert len(r.grid_w) == ta.n
        assert len(r.desired_w) == ta.n
        assert len(r.battery_w) == ta.n
        assert np.isfinite(r.feed_in_energy_wh)
        assert np.isfinite(r.iae)
        assert r.feed_in_energy_wh >= 0

    def test_step_up_has_positive_discharge(self) -> None:
        """Step-up (high load) should cause positive desired output."""
        sc = scenario_step_up(n=200)
        ta = make_trace_arrays(sc)
        r = simulate_fast(CURRENT_GAINS, PLANT_ABC, ta, DEFAULT_SYS_CFG)
        # After the step, desired should be mostly positive (discharging)
        assert r.desired_w[100:].mean() > 0

    def test_low_soc_reduces_discharge(self) -> None:
        """Low-SOC guard should reduce or stop discharge output."""
        sc = scenario_low_soc(n=300)
        ta = make_trace_arrays(sc)
        r = simulate_fast(CURRENT_GAINS, PLANT_ABC, ta, DEFAULT_SYS_CFG)
        # At the end (SOC=14%, below min_soc=15%), desired should be 0
        assert r.desired_w[-1] == pytest.approx(0.0, abs=1.0)
