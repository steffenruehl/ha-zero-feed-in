"""Tests for tools/optimize_gains.py — fast vs full simulation parity.

These tests verify that ``simulate_fast`` produces identical results to
``_simulate_full`` (which uses the real ``ControlLogic`` class).  Any
divergence means the inlined fast path has a logic bug.

Also tests I/O functions (save/load result/checkpoint), the cost
function, and parameter-to-config conversion.
"""
from __future__ import annotations

import json
import numpy as np
import pytest
from pathlib import Path

from tools.optimize_gains import (
    BOUNDS,
    IDENT_DT_S,
    PARAM_NAMES,
    PlantModel,
    SimResult,
    SystemConfig,
    TraceArrays,
    _simulate_full,
    cost,
    load_checkpoint,
    load_result,
    params_to_config,
    save_checkpoint,
    save_result,
    simulate,
    simulate_fast,
)


# ═══════════════════════════════════════════════════════════
#  Helpers
# ═══════════════════════════════════════════════════════════

# Typical plant params (from real identification)
PLANT_ABC = (0.9094, 0.0804, -1.96)

DEFAULT_PARAMS = np.array([
    0.25, 0.35, 0.17, 0.23,     # kp
    0.05, 0.05, 0.07, 0.07,     # ki
    35.0,                         # deadband
    30.0, 30.0,                   # ff_tau, ff_deadband
    0.8, 0.6,                     # ff_gain_pv, ff_gain_load
])

DEFAULT_SYS_CFG = SystemConfig(
    max_discharge_w=1200, max_charge_w=800, max_feed_in_w=800,
    min_soc_pct=15, max_soc_pct=85,
    discharge_target_w=30, charge_target_w=0,
    mode_hysteresis_w=50, charge_confirm_s=20,
    deadband_leak_ws=250, interval_s=5,
)


def make_traces(
    disturbance: list[float],
    soc: list[float],
    dt: list[float] | None = None,
) -> dict:
    """Build a raw traces dict for ``_simulate_full``."""
    n = len(disturbance)
    if dt is None:
        dt = [5.0] * n
    return {
        "disturbance_w": disturbance,
        "soc_pct": soc,
        "dt_s": dt,
        "grid_actual_w": [0.0] * n,
    }


def make_trace_arrays(traces: dict) -> TraceArrays:
    """Convert a raw traces dict into pre-converted TraceArrays."""
    return TraceArrays(
        disturbance=np.array(traces["disturbance_w"], dtype=np.float64),
        soc=np.array(traces["soc_pct"], dtype=np.float64),
        dt=np.array(traces["dt_s"], dtype=np.float64),
        grid_actual=np.array(traces["grid_actual_w"], dtype=np.float64),
        n=len(traces["disturbance_w"]),
    )


def assert_results_match(
    full: SimResult,
    fast: SimResult,
    *,
    rtol: float = 1e-6,
    atol_w: float = 0.1,
    label: str = "",
) -> None:
    """Assert that two SimResults are equivalent within tolerance."""
    prefix = f"[{label}] " if label else ""
    np.testing.assert_allclose(
        fast.grid_w, full.grid_w,
        rtol=rtol, atol=atol_w,
        err_msg=f"{prefix}grid_w mismatch",
    )
    np.testing.assert_allclose(
        fast.desired_w, full.desired_w,
        rtol=rtol, atol=atol_w,
        err_msg=f"{prefix}desired_w mismatch",
    )
    np.testing.assert_allclose(
        fast.battery_w, full.battery_w,
        rtol=rtol, atol=atol_w,
        err_msg=f"{prefix}battery_w mismatch",
    )
    assert fast.feed_in_energy_wh == pytest.approx(
        full.feed_in_energy_wh, abs=0.1,
    ), f"{prefix}feed_in_energy_wh"
    assert fast.iae == pytest.approx(
        full.iae, rel=1e-4,
    ), f"{prefix}iae"
    assert fast.control_effort == pytest.approx(
        full.control_effort, rel=1e-4,
    ), f"{prefix}control_effort"


# ═══════════════════════════════════════════════════════════
#  Tests
# ═══════════════════════════════════════════════════════════


class TestFastVsFullParity:
    """Verify simulate_fast matches _simulate_full step-by-step."""

    def test_steady_discharge(self) -> None:
        """Constant disturbance above target — pure discharge, SOC stays high."""
        n = 200
        traces = make_traces(
            disturbance=[200.0] * n,
            soc=[50.0] * n,
        )
        ta = make_trace_arrays(traces)
        full = _simulate_full(DEFAULT_PARAMS, PLANT_ABC, traces, DEFAULT_SYS_CFG)
        fast = simulate_fast(DEFAULT_PARAMS, PLANT_ABC, ta, DEFAULT_SYS_CFG)
        assert_results_match(full, fast, label="steady_discharge")

    def test_steady_charge(self) -> None:
        """Constant negative disturbance — surplus, should transition to charging."""
        n = 200
        traces = make_traces(
            disturbance=[-300.0] * n,
            soc=[50.0] * n,
        )
        ta = make_trace_arrays(traces)
        full = _simulate_full(DEFAULT_PARAMS, PLANT_ABC, traces, DEFAULT_SYS_CFG)
        fast = simulate_fast(DEFAULT_PARAMS, PLANT_ABC, ta, DEFAULT_SYS_CFG)
        assert_results_match(full, fast, label="steady_charge")

    def test_soc_low_guard(self) -> None:
        """SOC at or below min_soc — discharge should be blocked."""
        n = 100
        traces = make_traces(
            disturbance=[200.0] * n,
            soc=[15.0] * n,  # exactly at min_soc
        )
        ta = make_trace_arrays(traces)
        full = _simulate_full(DEFAULT_PARAMS, PLANT_ABC, traces, DEFAULT_SYS_CFG)
        fast = simulate_fast(DEFAULT_PARAMS, PLANT_ABC, ta, DEFAULT_SYS_CFG)
        assert_results_match(full, fast, label="soc_low_guard")

    def test_soc_high_guard(self) -> None:
        """SOC at max_soc — charge should be blocked."""
        n = 100
        traces = make_traces(
            disturbance=[-300.0] * n,
            soc=[85.0] * n,  # at max_soc
        )
        ta = make_trace_arrays(traces)
        full = _simulate_full(DEFAULT_PARAMS, PLANT_ABC, traces, DEFAULT_SYS_CFG)
        fast = simulate_fast(DEFAULT_PARAMS, PLANT_ABC, ta, DEFAULT_SYS_CFG)
        assert_results_match(full, fast, label="soc_high_guard")

    def test_deadband_hold_with_guard(self) -> None:
        """Error inside deadband while charging at low surplus → guard should fire.

        This is the specific bug case: the PI is in deadband (returns
        last_computed < 0), but a guard should still force idle because
        surplus has dropped.
        """
        n = 80
        # Start with surplus to enter charging mode, then drop it.
        # Phase 1 (0-39): strong surplus → charging
        # Phase 2 (40-79): disturbance flips positive → no surplus,
        #   but error may still be in deadband from previous setpoint.
        disturbance = [-200.0] * 40 + [25.0] * 40
        soc = [50.0] * n
        traces = make_traces(disturbance=disturbance, soc=soc)
        ta = make_trace_arrays(traces)
        full = _simulate_full(DEFAULT_PARAMS, PLANT_ABC, traces, DEFAULT_SYS_CFG)
        fast = simulate_fast(DEFAULT_PARAMS, PLANT_ABC, ta, DEFAULT_SYS_CFG)
        assert_results_match(full, fast, label="deadband_hold_with_guard")

    def test_emergency_protection(self) -> None:
        """Large feed-in spike should trigger emergency and slash output."""
        n = 60
        # Start discharging, then a big negative spike (feed-in)
        disturbance = [200.0] * 20 + [-1200.0] * 10 + [200.0] * 30
        soc = [50.0] * n
        traces = make_traces(disturbance=disturbance, soc=soc)
        ta = make_trace_arrays(traces)
        full = _simulate_full(DEFAULT_PARAMS, PLANT_ABC, traces, DEFAULT_SYS_CFG)
        fast = simulate_fast(DEFAULT_PARAMS, PLANT_ABC, ta, DEFAULT_SYS_CFG)
        assert_results_match(full, fast, label="emergency_protection")

    def test_mode_transition_discharge_to_charge(self) -> None:
        """Gradual surplus increase triggers DISCHARGING → CHARGING."""
        n = 100
        # Ramp from positive to negative disturbance
        disturbance = [200.0 - 6.0 * k for k in range(n)]
        soc = [50.0] * n
        traces = make_traces(disturbance=disturbance, soc=soc)
        ta = make_trace_arrays(traces)
        full = _simulate_full(DEFAULT_PARAMS, PLANT_ABC, traces, DEFAULT_SYS_CFG)
        fast = simulate_fast(DEFAULT_PARAMS, PLANT_ABC, ta, DEFAULT_SYS_CFG)
        assert_results_match(full, fast, label="mode_transition_d2c")

    def test_mode_transition_charge_to_discharge(self) -> None:
        """Surplus drop triggers CHARGING → DISCHARGING."""
        n = 100
        # Start negative (surplus), ramp to positive
        disturbance = [-200.0 + 6.0 * k for k in range(n)]
        soc = [50.0] * n
        traces = make_traces(disturbance=disturbance, soc=soc)
        ta = make_trace_arrays(traces)
        full = _simulate_full(DEFAULT_PARAMS, PLANT_ABC, traces, DEFAULT_SYS_CFG)
        fast = simulate_fast(DEFAULT_PARAMS, PLANT_ABC, ta, DEFAULT_SYS_CFG)
        assert_results_match(full, fast, label="mode_transition_c2d")

    def test_deadband_leak(self) -> None:
        """Persistent small error inside deadband accumulates until leak fires."""
        n = 200
        # Small positive disturbance that puts error inside deadband
        # error = grid - target = (disturbance - battery) - 30
        # With battery near 0 initially: error ≈ 50 - 30 = 20 < 35 (deadband)
        traces = make_traces(
            disturbance=[50.0] * n,
            soc=[50.0] * n,
        )
        ta = make_trace_arrays(traces)
        full = _simulate_full(DEFAULT_PARAMS, PLANT_ABC, traces, DEFAULT_SYS_CFG)
        fast = simulate_fast(DEFAULT_PARAMS, PLANT_ABC, ta, DEFAULT_SYS_CFG)
        assert_results_match(full, fast, label="deadband_leak")

    def test_variable_dt(self) -> None:
        """Non-uniform timesteps (as in real data with gaps)."""
        n = 100
        dt = [5.0] * 30 + [15.0] * 10 + [5.0] * 59 + [5.0]
        traces = make_traces(
            disturbance=[150.0 + 50 * np.sin(k / 10) for k in range(n)],
            soc=[50.0] * n,
            dt=dt,
        )
        ta = make_trace_arrays(traces)
        full = _simulate_full(DEFAULT_PARAMS, PLANT_ABC, traces, DEFAULT_SYS_CFG)
        fast = simulate_fast(DEFAULT_PARAMS, PLANT_ABC, ta, DEFAULT_SYS_CFG)
        assert_results_match(full, fast, label="variable_dt")

    def test_surplus_clamp(self) -> None:
        """Charge rate should be clamped to available surplus."""
        n = 100
        # Small surplus — charge should be limited
        disturbance = [-50.0] * n
        soc = [50.0] * n
        traces = make_traces(disturbance=disturbance, soc=soc)
        ta = make_trace_arrays(traces)
        full = _simulate_full(DEFAULT_PARAMS, PLANT_ABC, traces, DEFAULT_SYS_CFG)
        fast = simulate_fast(DEFAULT_PARAMS, PLANT_ABC, ta, DEFAULT_SYS_CFG)
        assert_results_match(full, fast, label="surplus_clamp")

    @pytest.mark.parametrize("seed", [0, 7, 42, 99, 123])
    def test_random_gains_parity(self, seed: int) -> None:
        """Random gain vectors should produce identical results."""
        rng = np.random.RandomState(seed)
        params = np.array([
            rng.uniform(lo, hi) for lo, hi in BOUNDS
        ])
        n = 150
        # Mix of disturbance patterns
        rng2 = np.random.RandomState(seed + 1000)
        disturbance = list(rng2.uniform(-300, 400, n))
        soc = list(np.clip(rng2.uniform(10, 90, n), 10, 90))
        traces = make_traces(disturbance=disturbance, soc=soc)
        ta = make_trace_arrays(traces)
        full = _simulate_full(params, PLANT_ABC, traces, DEFAULT_SYS_CFG)
        fast = simulate_fast(params, PLANT_ABC, ta, DEFAULT_SYS_CFG)
        assert_results_match(full, fast, label=f"random_gains_seed{seed}")


class TestFastSimulationOnRealData:
    """Test with real plant model data if available."""

    @pytest.fixture
    def real_data(self):
        """Load real plant model — skip if not available."""
        import json
        from pathlib import Path

        model_path = Path("data/plant_model.json")
        if not model_path.exists():
            pytest.skip("data/plant_model.json not available")
        with open(model_path) as f:
            data = json.load(f)
        return data

    def test_real_data_parity(self, real_data: dict) -> None:
        """Fast and full simulation produce identical results on real traces."""
        p = real_data["plant"]
        traces = real_data["traces"]
        plant_abc = (p["a"], p["b"], p["c"])

        ta = make_trace_arrays(traces)
        full = _simulate_full(DEFAULT_PARAMS, plant_abc, traces, DEFAULT_SYS_CFG)
        fast = simulate_fast(DEFAULT_PARAMS, plant_abc, ta, DEFAULT_SYS_CFG)
        assert_results_match(full, fast, label="real_data")


# ═══════════════════════════════════════════════════════════
#  I/O: save_result / load_result round-trip
# ═══════════════════════════════════════════════════════════


def _make_sim_result(n: int = 10, **overrides) -> SimResult:
    """Helper to create a SimResult with sensible defaults."""
    defaults = dict(
        grid_w=np.linspace(20, 40, n),
        desired_w=np.linspace(100, 200, n),
        battery_w=np.linspace(80, 160, n),
        feed_in_energy_wh=42.5,
        iae=12345.0,
        max_feed_in_w=67.0,
        control_effort=890.0,
    )
    defaults.update(overrides)
    return SimResult(**defaults)


class TestSaveLoadResult:
    """Verify optimization result JSON round-trips correctly."""

    @pytest.fixture
    def result_path(self, tmp_path: Path) -> Path:
        """Temporary path for a result JSON file."""
        return tmp_path / "result.json"

    def test_round_trip(self, result_path: Path) -> None:
        """save_result → load_result preserves all key fields."""
        sr = _make_sim_result()
        save_result(
            result_path,
            optimized_params=DEFAULT_PARAMS * 1.1,
            current_params=DEFAULT_PARAMS,
            baseline=sr, optimized=sr,
            baseline_cost=1000.0, optimized_cost=800.0,
            plant_abc=PLANT_ABC, sys_cfg=DEFAULT_SYS_CFG,
            generations=10, converged=True, elapsed_s=42.5,
        )
        data = load_result(result_path)
        assert data["version"] == 1
        assert len(data["param_vector"]) == len(PARAM_NAMES)
        assert len(data["current_vector"]) == len(PARAM_NAMES)
        assert data["baseline_cost"] == 1000.0
        assert data["optimized_cost"] == 800.0
        assert data["optimization"]["generations"] == 10
        assert data["optimization"]["converged"] is True

    def test_param_names_match(self, result_path: Path) -> None:
        """Optimized params dict has correct parameter names."""
        sr = _make_sim_result()
        save_result(
            result_path,
            optimized_params=DEFAULT_PARAMS, current_params=DEFAULT_PARAMS,
            baseline=sr, optimized=sr,
            baseline_cost=100.0, optimized_cost=90.0,
            plant_abc=PLANT_ABC, sys_cfg=DEFAULT_SYS_CFG,
            generations=5, converged=False, elapsed_s=10.0,
        )
        data = load_result(result_path)
        assert list(data["optimized_params"].keys()) == PARAM_NAMES
        assert list(data["current_params"].keys()) == PARAM_NAMES

    def test_metrics_stored(self, result_path: Path) -> None:
        """Baseline and optimized metrics are present."""
        sr = _make_sim_result()
        save_result(
            result_path,
            optimized_params=DEFAULT_PARAMS, current_params=DEFAULT_PARAMS,
            baseline=sr, optimized=sr,
            baseline_cost=100.0, optimized_cost=100.0,
            plant_abc=PLANT_ABC, sys_cfg=DEFAULT_SYS_CFG,
            generations=1, converged=False, elapsed_s=1.0,
        )
        data = load_result(result_path)
        for key in ("feed_in_energy_wh", "max_feed_in_w", "iae", "control_effort"):
            assert key in data["baseline_metrics"]
            assert key in data["optimized_metrics"]

    def test_improvement_pct(self, result_path: Path) -> None:
        """Improvement percentages are computed correctly."""
        baseline = _make_sim_result(feed_in_energy_wh=100.0)
        optimized = _make_sim_result(feed_in_energy_wh=80.0)
        save_result(
            result_path,
            optimized_params=DEFAULT_PARAMS, current_params=DEFAULT_PARAMS,
            baseline=baseline, optimized=optimized,
            baseline_cost=1000.0, optimized_cost=800.0,
            plant_abc=PLANT_ABC, sys_cfg=DEFAULT_SYS_CFG,
            generations=5, converged=True, elapsed_s=10.0,
        )
        data = load_result(result_path)
        assert data["improvement_pct"]["feed_in"] == pytest.approx(-20.0)
        assert data["improvement_pct"]["cost"] == pytest.approx(-20.0)

    def test_atomic_write(self, result_path: Path) -> None:
        """No .tmp file left after save."""
        sr = _make_sim_result()
        save_result(
            result_path,
            optimized_params=DEFAULT_PARAMS, current_params=DEFAULT_PARAMS,
            baseline=sr, optimized=sr,
            baseline_cost=100.0, optimized_cost=100.0,
            plant_abc=PLANT_ABC, sys_cfg=DEFAULT_SYS_CFG,
            generations=1, converged=False, elapsed_s=1.0,
        )
        assert result_path.exists()
        assert not result_path.with_suffix(".tmp").exists()

    def test_valid_json(self, result_path: Path) -> None:
        """Output is valid JSON parseable by the stdlib."""
        sr = _make_sim_result()
        save_result(
            result_path,
            optimized_params=DEFAULT_PARAMS, current_params=DEFAULT_PARAMS,
            baseline=sr, optimized=sr,
            baseline_cost=100.0, optimized_cost=100.0,
            plant_abc=PLANT_ABC, sys_cfg=DEFAULT_SYS_CFG,
            generations=1, converged=False, elapsed_s=1.0,
        )
        raw = result_path.read_text()
        data = json.loads(raw)
        assert isinstance(data, dict)

    def test_plant_and_system_config(self, result_path: Path) -> None:
        """Plant model and system config are stored in the result."""
        sr = _make_sim_result()
        save_result(
            result_path,
            optimized_params=DEFAULT_PARAMS, current_params=DEFAULT_PARAMS,
            baseline=sr, optimized=sr,
            baseline_cost=100.0, optimized_cost=100.0,
            plant_abc=PLANT_ABC, sys_cfg=DEFAULT_SYS_CFG,
            generations=1, converged=False, elapsed_s=1.0,
        )
        data = load_result(result_path)
        assert data["plant"]["a"] == pytest.approx(PLANT_ABC[0])
        assert data["plant"]["b"] == pytest.approx(PLANT_ABC[1])
        assert data["plant"]["c"] == pytest.approx(PLANT_ABC[2])
        assert data["system_config"]["max_discharge_w"] == DEFAULT_SYS_CFG.max_discharge_w
        assert data["system_config"]["min_soc_pct"] == DEFAULT_SYS_CFG.min_soc_pct

    def test_bounds_stored(self, result_path: Path) -> None:
        """Optimization bounds are stored in the result."""
        sr = _make_sim_result()
        save_result(
            result_path,
            optimized_params=DEFAULT_PARAMS, current_params=DEFAULT_PARAMS,
            baseline=sr, optimized=sr,
            baseline_cost=100.0, optimized_cost=100.0,
            plant_abc=PLANT_ABC, sys_cfg=DEFAULT_SYS_CFG,
            generations=1, converged=False, elapsed_s=1.0,
        )
        data = load_result(result_path)
        assert "bounds" in data
        assert len(data["bounds"]) == len(PARAM_NAMES)


# ═══════════════════════════════════════════════════════════
#  I/O: save_checkpoint / load_checkpoint round-trip
# ═══════════════════════════════════════════════════════════


class TestSaveLoadCheckpoint:
    """Verify checkpoint JSON round-trips correctly."""

    def test_round_trip(self, tmp_path: Path) -> None:
        """save_checkpoint → load_checkpoint preserves all fields."""
        cp_path = tmp_path / "checkpoint.json"
        save_checkpoint(
            cp_path,
            best_x=DEFAULT_PARAMS, best_cost=42.0, gen=7,
            baseline_cost=100.0, baseline_feedin=50.0,
            current_gains=DEFAULT_PARAMS,
        )
        data = load_checkpoint(cp_path)
        assert data["generation"] == 7
        assert data["best_cost"] == 42.0
        assert data["baseline_cost"] == 100.0
        assert data["baseline_feedin"] == 50.0
        np.testing.assert_array_almost_equal(
            data["best_x"], DEFAULT_PARAMS.tolist(),
        )

    def test_atomic_write(self, tmp_path: Path) -> None:
        """No .tmp file left after save."""
        cp_path = tmp_path / "cp.json"
        save_checkpoint(
            cp_path,
            best_x=DEFAULT_PARAMS, best_cost=1.0, gen=1,
            baseline_cost=2.0, baseline_feedin=3.0,
            current_gains=DEFAULT_PARAMS,
        )
        assert cp_path.exists()
        assert not cp_path.with_suffix(".tmp").exists()

    def test_overwrite(self, tmp_path: Path) -> None:
        """Second save overwrites the first."""
        cp_path = tmp_path / "cp.json"
        save_checkpoint(
            cp_path,
            best_x=DEFAULT_PARAMS, best_cost=100.0, gen=1,
            baseline_cost=200.0, baseline_feedin=50.0,
            current_gains=DEFAULT_PARAMS,
        )
        save_checkpoint(
            cp_path,
            best_x=DEFAULT_PARAMS * 2, best_cost=50.0, gen=5,
            baseline_cost=200.0, baseline_feedin=50.0,
            current_gains=DEFAULT_PARAMS,
        )
        data = load_checkpoint(cp_path)
        assert data["generation"] == 5
        assert data["best_cost"] == 50.0


# ═══════════════════════════════════════════════════════════
#  params_to_config
# ═══════════════════════════════════════════════════════════


class TestParamsToConfig:
    """Verify parameter vector → Config conversion."""

    def test_kp_mapping(self) -> None:
        """kp values end up in the correct Config fields."""
        params = np.array([
            0.1, 0.2, 0.3, 0.4,
            0.01, 0.02, 0.03, 0.04,
            20.0, 30.0, 10.0, 0.8, 0.6,
        ])
        cfg = params_to_config(params, DEFAULT_SYS_CFG)
        assert cfg.kp_discharge_up == pytest.approx(0.1)
        assert cfg.kp_discharge_down == pytest.approx(0.2)
        assert cfg.kp_charge_up == pytest.approx(0.3)
        assert cfg.kp_charge_down == pytest.approx(0.4)

    def test_ki_mapping(self) -> None:
        """ki values end up in the correct Config fields."""
        params = np.array([
            0.1, 0.2, 0.3, 0.4,
            0.01, 0.02, 0.03, 0.04,
            20.0, 30.0, 10.0, 0.8, 0.6,
        ])
        cfg = params_to_config(params, DEFAULT_SYS_CFG)
        assert cfg.ki_discharge_up == pytest.approx(0.01)
        assert cfg.ki_discharge_down == pytest.approx(0.02)
        assert cfg.ki_charge_up == pytest.approx(0.03)
        assert cfg.ki_charge_down == pytest.approx(0.04)

    def test_soc_limits_propagated(self) -> None:
        """min_soc_pct and max_soc_pct come from SystemConfig, not defaults."""
        sys_cfg = SystemConfig(min_soc_pct=20.0, max_soc_pct=90.0)
        cfg = params_to_config(DEFAULT_PARAMS, sys_cfg)
        assert cfg.min_soc_pct == 20.0
        assert cfg.max_soc_pct == 90.0

    def test_ff_sources_created(self) -> None:
        """Feed-forward sources are created with correct gains and signs."""
        cfg = params_to_config(DEFAULT_PARAMS, DEFAULT_SYS_CFG)
        assert cfg.ff_enabled is True
        assert len(cfg.ff_sources) == 2  # pv + load
        pv_src = cfg.ff_sources[0]
        load_src = cfg.ff_sources[1]
        assert pv_src.sign == -1.0
        assert pv_src.gain == pytest.approx(0.8)
        assert load_src.sign == 1.0
        assert load_src.gain == pytest.approx(0.6)

    def test_deadband_and_ff_params(self) -> None:
        """Deadband, FF tau, and FF deadband passed correctly."""
        params = np.array([
            0.1, 0.2, 0.3, 0.4,
            0.01, 0.02, 0.03, 0.04,
            42.0, 55.0, 15.0, 0.9, 0.7,
        ])
        cfg = params_to_config(params, DEFAULT_SYS_CFG)
        assert cfg.deadband_w == pytest.approx(42.0)
        assert cfg.ff_filter_tau_s == pytest.approx(55.0)
        assert cfg.ff_deadband_w == pytest.approx(15.0)


# ═══════════════════════════════════════════════════════════
#  Cost function
# ═══════════════════════════════════════════════════════════


class TestCostFunction:
    """Verify the cost function composition."""

    @pytest.fixture(autouse=True)
    def _set_globals(self) -> None:
        """Set module-level globals that cost() depends on."""
        import tools.optimize_gains as og
        og._plant_abc = PLANT_ABC
        og._traces = make_trace_arrays(make_traces(
            disturbance=[200.0] * 100,
            soc=[50.0] * 100,
        ))
        og._sys_cfg = DEFAULT_SYS_CFG

    def test_cost_returns_scalar(self) -> None:
        """cost() returns a finite positive float."""
        c = cost(DEFAULT_PARAMS)
        assert isinstance(c, float)
        assert np.isfinite(c)
        assert c > 0

    def test_cost_weight_dominance(self) -> None:
        """Feed-in energy dominates the cost (weight 100)."""
        import tools.optimize_gains as og
        r = simulate(DEFAULT_PARAMS, PLANT_ABC, og._traces, DEFAULT_SYS_CFG)
        c = cost(DEFAULT_PARAMS)
        feedin_term = 100.0 * r.feed_in_energy_wh
        assert feedin_term <= c

    def test_cost_consistent_with_simulate(self) -> None:
        """cost() equals the manual cost formula applied to simulate()."""
        import tools.optimize_gains as og
        r = simulate(DEFAULT_PARAMS, PLANT_ABC, og._traces, DEFAULT_SYS_CFG)
        expected = (
            100.0 * r.feed_in_energy_wh
            + 0.001 * r.iae
            + 0.0001 * r.control_effort
        )
        assert cost(DEFAULT_PARAMS) == pytest.approx(expected)


# ═══════════════════════════════════════════════════════════
#  simulate() dispatch
# ═══════════════════════════════════════════════════════════


class TestSimulateDispatch:
    """Verify simulate() routes to fast or full path correctly."""

    def test_dict_uses_full(self) -> None:
        """Raw dict traces → _simulate_full path."""
        traces = make_traces([200.0] * 50, [50.0] * 50)
        r = simulate(DEFAULT_PARAMS, PLANT_ABC, traces, DEFAULT_SYS_CFG)
        assert isinstance(r, SimResult)

    def test_trace_arrays_uses_fast(self) -> None:
        """TraceArrays → simulate_fast path."""
        traces = make_traces([200.0] * 50, [50.0] * 50)
        ta = make_trace_arrays(traces)
        r = simulate(DEFAULT_PARAMS, PLANT_ABC, ta, DEFAULT_SYS_CFG)
        assert isinstance(r, SimResult)

    def test_both_paths_agree(self) -> None:
        """Both dispatch paths produce the same result."""
        traces = make_traces([200.0] * 100, [50.0] * 100)
        ta = make_trace_arrays(traces)
        r_dict = simulate(DEFAULT_PARAMS, PLANT_ABC, traces, DEFAULT_SYS_CFG)
        r_ta = simulate(DEFAULT_PARAMS, PLANT_ABC, ta, DEFAULT_SYS_CFG)
        assert_results_match(r_dict, r_ta, label="dispatch_parity")


# ═══════════════════════════════════════════════════════════
#  BOUNDS consistency
# ═══════════════════════════════════════════════════════════


class TestBounds:
    """Verify BOUNDS and PARAM_NAMES are consistent."""

    def test_same_length(self) -> None:
        """BOUNDS and PARAM_NAMES have the same length."""
        assert len(BOUNDS) == len(PARAM_NAMES)

    def test_all_lower_less_than_upper(self) -> None:
        """Every bound has lo < hi."""
        for i, (lo, hi) in enumerate(BOUNDS):
            assert lo < hi, f"BOUNDS[{i}] ({PARAM_NAMES[i]}): {lo} >= {hi}"

    def test_default_params_within_bounds(self) -> None:
        """DEFAULT_PARAMS are within the optimization bounds."""
        for i, (lo, hi) in enumerate(BOUNDS):
            assert lo <= DEFAULT_PARAMS[i] <= hi, (
                f"{PARAM_NAMES[i]}: {DEFAULT_PARAMS[i]} not in [{lo}, {hi}]"
            )


# ═══════════════════════════════════════════════════════════
#  PlantModel dt-aware discretisation
# ═══════════════════════════════════════════════════════════


class TestPlantModel:
    """Verify the dt-aware PlantModel."""

    def test_nominal_dt_matches_original(self) -> None:
        """At dt=IDENT_DT_S (5s), step() reproduces a*x + b*u + c exactly."""
        a, b, c = PLANT_ABC
        pm = PlantModel(a, b, c)
        pm.reset(100.0)
        out = pm.step(200.0, dt=IDENT_DT_S)
        expected = a * 100.0 + b * 200.0 + c
        assert out == pytest.approx(expected, rel=1e-10)

    def test_default_dt_is_nominal(self) -> None:
        """step() without explicit dt uses IDENT_DT_S."""
        pm1 = PlantModel(*PLANT_ABC)
        pm2 = PlantModel(*PLANT_ABC)
        pm1.reset(50.0)
        pm2.reset(50.0)
        out1 = pm1.step(300.0)  # default dt
        out2 = pm2.step(300.0, dt=IDENT_DT_S)
        assert out1 == pytest.approx(out2, rel=1e-12)

    def test_double_dt_differs(self) -> None:
        """dt=10s should give a different result than dt=5s."""
        pm5 = PlantModel(*PLANT_ABC)
        pm10 = PlantModel(*PLANT_ABC)
        pm5.reset(100.0)
        pm10.reset(100.0)
        out5 = pm5.step(200.0, dt=5.0)
        out10 = pm10.step(200.0, dt=10.0)
        assert out5 != pytest.approx(out10, abs=0.1)

    def test_two_5s_steps_match_one_10s_step(self) -> None:
        """Two consecutive 5s steps with constant input ≈ one 10s step."""
        pm_2x5 = PlantModel(*PLANT_ABC)
        pm_1x10 = PlantModel(*PLANT_ABC)
        pm_2x5.reset(100.0)
        pm_1x10.reset(100.0)
        pm_2x5.step(200.0, dt=5.0)
        out_2x5 = pm_2x5.step(200.0, dt=5.0)
        out_1x10 = pm_1x10.step(200.0, dt=10.0)
        assert out_2x5 == pytest.approx(out_1x10, rel=1e-6)

    def test_continuous_params_derived_correctly(self) -> None:
        """tau, dc_gain, c_cont match analytical formulas."""
        a, b, c = PLANT_ABC
        pm = PlantModel(a, b, c)
        assert pm.tau == pytest.approx(-IDENT_DT_S / np.log(a), rel=1e-10)
        assert pm.dc_gain == pytest.approx(b / (1 - a), rel=1e-10)
        assert pm.c_cont == pytest.approx(c / (1 - a), rel=1e-10)

    def test_longer_dt_approaches_steady_state_faster(self) -> None:
        """With larger dt, the plant should reach steady state in fewer steps."""
        desired = 200.0
        pm_fast = PlantModel(*PLANT_ABC)
        pm_slow = PlantModel(*PLANT_ABC)
        # 10 steps at 10s vs 10 steps at 5s, same input
        for _ in range(10):
            pm_fast.step(desired, dt=10.0)
            pm_slow.step(desired, dt=5.0)
        # Steady state = K * desired + c_cont
        ss = pm_fast.dc_gain * desired + pm_fast.c_cont
        # pm_fast should be closer to steady state
        err_fast = abs(pm_fast.battery_w - ss)
        err_slow = abs(pm_slow.battery_w - ss)
        assert err_fast < err_slow

    def test_zero_dt_holds_state(self) -> None:
        """dt=0 means no evolution — state stays the same."""
        pm = PlantModel(*PLANT_ABC)
        pm.reset(150.0)
        out = pm.step(999.0, dt=0.0)
        assert out == pytest.approx(150.0, abs=1e-10)


# ═══════════════════════════════════════════════════════════
#  Jitter parity: variable dt in fast vs full sim
# ═══════════════════════════════════════════════════════════


class TestJitterParity:
    """Verify fast and full sims stay in parity with timing jitter."""

    def test_realistic_jitter(self) -> None:
        """Gaussian jitter (mean=5s, std=0.5s) preserves parity."""
        rng = np.random.RandomState(42)
        n = 200
        dt = np.clip(rng.normal(5.0, 0.5, n), 3.0, 8.0).tolist()
        traces = make_traces(
            disturbance=[200.0] * n,
            soc=[50.0] * n,
            dt=dt,
        )
        ta = make_trace_arrays(traces)
        full = _simulate_full(DEFAULT_PARAMS, PLANT_ABC, traces, DEFAULT_SYS_CFG)
        fast = simulate_fast(DEFAULT_PARAMS, PLANT_ABC, ta, DEFAULT_SYS_CFG)
        assert_results_match(full, fast, label="realistic_jitter")

    def test_large_gap_parity(self) -> None:
        """A 15s gap (e.g. missed ticks) preserves parity."""
        n = 100
        dt = [5.0] * 40 + [15.0] * 5 + [5.0] * 55
        traces = make_traces(
            disturbance=[150.0] * n,
            soc=[50.0] * n,
            dt=dt,
        )
        ta = make_trace_arrays(traces)
        full = _simulate_full(DEFAULT_PARAMS, PLANT_ABC, traces, DEFAULT_SYS_CFG)
        fast = simulate_fast(DEFAULT_PARAMS, PLANT_ABC, ta, DEFAULT_SYS_CFG)
        assert_results_match(full, fast, label="large_gap")

    def test_jitter_changes_result_vs_constant_dt(self) -> None:
        """Variable dt should produce different metrics than constant dt."""
        n = 200
        disturbance = [200.0] * n
        soc = [50.0] * n

        traces_const = make_traces(disturbance, soc, dt=[5.0] * n)
        ta_const = make_trace_arrays(traces_const)
        r_const = simulate_fast(DEFAULT_PARAMS, PLANT_ABC, ta_const, DEFAULT_SYS_CFG)

        rng = np.random.RandomState(99)
        dt_jitter = np.clip(rng.normal(5.0, 1.5, n), 2.0, 12.0).tolist()
        traces_jitter = make_traces(disturbance, soc, dt=dt_jitter)
        ta_jitter = make_trace_arrays(traces_jitter)
        r_jitter = simulate_fast(DEFAULT_PARAMS, PLANT_ABC, ta_jitter, DEFAULT_SYS_CFG)

        # Results should differ because the plant model responds to dt
        assert r_const.feed_in_energy_wh != pytest.approx(
            r_jitter.feed_in_energy_wh, abs=0.01,
        )


# ═══════════════════════════════════════════════════════════
#  Transport delay
# ═══════════════════════════════════════════════════════════


DELAY_SYS_CFG = SystemConfig(
    max_discharge_w=1200, max_charge_w=800, max_feed_in_w=800,
    min_soc_pct=15, max_soc_pct=85,
    discharge_target_w=30, charge_target_w=0,
    mode_hysteresis_w=50, charge_confirm_s=20,
    deadband_leak_ws=250, interval_s=5,
    delay_steps=2,
)


class TestPlantModelDelay:
    """Verify PlantModel transport delay buffer."""

    def test_delay_0_is_immediate(self) -> None:
        """delay_steps=0: plant sees desired immediately."""
        pm = PlantModel(*PLANT_ABC, delay_steps=0)
        pm.reset(0.0)
        out = pm.step(200.0)
        # Should change from 0 immediately
        assert out != pytest.approx(0.0, abs=0.1)

    def test_delay_2_holds_for_2_steps(self) -> None:
        """delay_steps=2: first two steps ignore the new desired."""
        pm = PlantModel(*PLANT_ABC, delay_steps=2)
        pm.reset(0.0)
        # Step 1: buffer has [0, 0]; desired=200 queued; plant uses 0
        out1 = pm.step(200.0)
        # Step 2: buffer has [0, 200]; plant uses 0
        out2 = pm.step(200.0)
        # Step 3: buffer has [200, 200]; plant uses 200
        out3 = pm.step(200.0)

        # First two steps: plant only sees effective=0 (bias c only)
        assert abs(out1) < 5.0  # close to c_cont*(1-alpha) term only
        assert abs(out2) < 5.0
        # Third step: plant finally sees 200
        assert out3 > 10.0

    def test_delay_buffer_resets(self) -> None:
        """reset() clears the delay buffer."""
        pm = PlantModel(*PLANT_ABC, delay_steps=2)
        pm.step(500.0)
        pm.step(500.0)
        pm.reset(0.0)
        # After reset, buffer should be [0, 0] again
        out = pm.step(200.0)
        assert abs(out) < 5.0  # still buffered


class TestTransportDelayParity:
    """Verify fast and full sims match with transport delay."""

    @pytest.mark.parametrize("delay", [1, 2, 3])
    def test_delay_parity(self, delay: int) -> None:
        """Fast and full paths produce identical results with delay."""
        n = 200
        cfg = SystemConfig(
            max_discharge_w=1200, max_charge_w=800, max_feed_in_w=800,
            min_soc_pct=15, max_soc_pct=85,
            discharge_target_w=30, charge_target_w=0,
            mode_hysteresis_w=50, charge_confirm_s=20,
            deadband_leak_ws=250, interval_s=5,
            delay_steps=delay,
        )
        traces = make_traces(
            disturbance=[200.0] * n,
            soc=[50.0] * n,
        )
        ta = make_trace_arrays(traces)
        full = _simulate_full(DEFAULT_PARAMS, PLANT_ABC, traces, cfg)
        fast = simulate_fast(DEFAULT_PARAMS, PLANT_ABC, ta, cfg)
        assert_results_match(full, fast, label=f"delay_{delay}")

    def test_delay_increases_feed_in(self) -> None:
        """Transport delay should increase feed-in compared to no delay.

        With delay, the battery responds later to controller commands,
        causing more overshoot and thus more feed-in.
        """
        n = 200
        traces = make_traces(
            disturbance=[200.0] * 50 + [-200.0] * 50 + [200.0] * 100,
            soc=[50.0] * n,
        )
        ta = make_trace_arrays(traces)

        cfg_no_delay = SystemConfig(
            max_discharge_w=1200, max_charge_w=800, max_feed_in_w=800,
            min_soc_pct=15, max_soc_pct=85,
            discharge_target_w=30, charge_target_w=0,
            mode_hysteresis_w=50, charge_confirm_s=20,
            deadband_leak_ws=250, interval_s=5,
            delay_steps=0,
        )
        r_no_delay = simulate_fast(DEFAULT_PARAMS, PLANT_ABC, ta, cfg_no_delay)

        cfg_delay = SystemConfig(
            max_discharge_w=1200, max_charge_w=800, max_feed_in_w=800,
            min_soc_pct=15, max_soc_pct=85,
            discharge_target_w=30, charge_target_w=0,
            mode_hysteresis_w=50, charge_confirm_s=20,
            deadband_leak_ws=250, interval_s=5,
            delay_steps=2,
        )
        r_delay = simulate_fast(DEFAULT_PARAMS, PLANT_ABC, ta, cfg_delay)

        # Delay should cause worse control performance
        assert r_delay.iae > r_no_delay.iae

    def test_delay_changes_optimal_kp(self) -> None:
        """Optimal Kp should differ when transport delay is modeled.

        Without delay the optimizer may pick gains that don't account
        for the command-to-effect latency.  Adding delay changes
        the cost landscape, yielding different gains.
        """
        n = 300
        traces = make_traces(
            disturbance=[500.0] * 50 + [100.0] * 100 + [500.0] * 150,
            soc=[50.0] * n,
        )
        ta = make_trace_arrays(traces)

        kp_values = [0.05, 0.10, 0.20, 0.40, 0.60, 0.80, 1.00]

        def best_kp(delay_steps: int) -> float:
            """Return the Kp that minimizes IAE for the given delay."""
            cfg = SystemConfig(
                max_discharge_w=1200, max_charge_w=800, max_feed_in_w=800,
                min_soc_pct=15, max_soc_pct=85,
                discharge_target_w=30, charge_target_w=0,
                mode_hysteresis_w=50, charge_confirm_s=20,
                deadband_leak_ws=250, interval_s=5,
                delay_steps=delay_steps,
            )
            best, best_iae = kp_values[0], float("inf")
            for kp in kp_values:
                params = DEFAULT_PARAMS.copy()
                params[0] = kp
                params[1] = kp
                r = simulate_fast(params, PLANT_ABC, ta, cfg)
                if r.iae < best_iae:
                    best_iae = r.iae
                    best = kp
            return best

        kp_no_delay = best_kp(0)
        kp_delay = best_kp(3)

        # Delay should change the optimal gains
        assert kp_delay != kp_no_delay
