#!/usr/bin/env python3
"""Plot closed-loop system response for different gain sets.

Runs simulation with one or more gain configurations on sample
scenarios (step, ramp, real data) and plots the resulting grid
power, desired output, and battery response side by side.

Reads optimized gains from the optimization result JSON produced
by ``optimize_gains.py``.

Usage:
    # Compare current vs optimized gains on all scenarios:
    python3 tools/plot_response.py

    # Compare with a specific result file:
    python3 tools/plot_response.py --result data/optimization_result.json

    # Only specific scenarios:
    python3 tools/plot_response.py --scenario step_up --scenario cloud

    # Use real trace data:
    python3 tools/plot_response.py --scenario real --real-range 0:2000
"""
from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from tools.optimize_gains import (
    DEFAULT_RESULT,
    PARAM_NAMES,
    SimResult,
    SystemConfig,
    TraceArrays,
    load_result,
    simulate_fast,
)


# ── Gain sets ────────────────────────────────────────────────


CURRENT_GAINS = np.array([
    0.25, 0.35, 0.17, 0.23,     # kp
    0.05, 0.05, 0.07, 0.07,     # ki
    35.0,                         # deadband
    30.0, 30.0,                   # ff_tau, ff_deadband
    0.8, 0.6,                     # ff_gain_pv, ff_gain_load
    10000, 25, 90, 25,           # relay: lockout_ws, cutoff_w, idle_s, min_active_w
])

DEFAULT_SYS_CFG = SystemConfig(
    max_discharge_w=1200, max_charge_w=800, max_feed_in_w=800,
    min_soc_pct=15, max_soc_pct=85,
    discharge_target_w=30, charge_target_w=0,
    mode_hysteresis_w=50, charge_confirm_s=20,
    deadband_leak_ws=250, interval_s=5,
)

# Plant model — loaded at runtime, defaults for quick testing.
DEFAULT_PLANT_ABC = (0.9094, 0.0804, -1.96)


# ── Scenarios ────────────────────────────────────────────────


@dataclass
class Scenario:
    """A synthetic or real-data test scenario."""

    name: str
    title: str
    disturbance: np.ndarray
    soc: np.ndarray
    dt: np.ndarray


def make_trace_arrays(sc: Scenario) -> TraceArrays:
    """Convert a Scenario into TraceArrays for simulate_fast."""
    return TraceArrays(
        disturbance=sc.disturbance.astype(np.float64),
        soc=sc.soc.astype(np.float64),
        dt=sc.dt.astype(np.float64),
        grid_actual=np.zeros(len(sc.disturbance), dtype=np.float64),
        n=len(sc.disturbance),
    )


def scenario_step_up(n: int = 300) -> Scenario:
    """Load step: house load jumps from 100W to 500W at t=50."""
    dist = np.full(n, 100.0)
    dist[50:] = 500.0
    return Scenario(
        name="step_up",
        title="Step Up: 100W → 500W load",
        disturbance=dist,
        soc=np.full(n, 50.0),
        dt=np.full(n, 5.0),
    )


def scenario_step_down(n: int = 300) -> Scenario:
    """Load step: house load drops from 500W to 100W at t=50."""
    dist = np.full(n, 500.0)
    dist[50:] = 100.0
    return Scenario(
        name="step_down",
        title="Step Down: 500W → 100W load",
        disturbance=dist,
        soc=np.full(n, 50.0),
        dt=np.full(n, 5.0),
    )


def scenario_cloud(n: int = 400) -> Scenario:
    """Cloud transient: PV drops sharply, recovers.

    Simulated as a disturbance swing from -200W (surplus) to +300W
    (deficit) and back.
    """
    dist = np.full(n, -200.0)
    # Cloud: surplus gone at t=80 for 60 steps
    dist[80:140] = 300.0
    # Recovery
    dist[140:170] = np.linspace(300, -200, 30)
    return Scenario(
        name="cloud",
        title="Cloud Transient: surplus → deficit → recovery",
        disturbance=dist,
        soc=np.full(n, 50.0),
        dt=np.full(n, 5.0),
    )


def scenario_charge_transition(n: int = 400) -> Scenario:
    """Transition from discharge to charge mode (growing PV surplus)."""
    # Ramp from +300W (discharge) to -400W (charge)
    dist = np.linspace(300, -400, n)
    return Scenario(
        name="charge_transition",
        title="Discharge → Charge transition",
        disturbance=dist,
        soc=np.full(n, 50.0),
        dt=np.full(n, 5.0),
    )


def scenario_oscillating(n: int = 600) -> Scenario:
    """Oscillating load (e.g. heat pump cycling)."""
    t = np.arange(n) * 5.0
    dist = 200.0 + 150.0 * np.sin(2 * np.pi * t / 300)
    return Scenario(
        name="oscillating",
        title="Oscillating load (5-min cycle)",
        disturbance=dist,
        soc=np.full(n, 50.0),
        dt=np.full(n, 5.0),
    )


def scenario_low_soc(n: int = 300) -> Scenario:
    """Discharge demand with SOC near minimum — tests guard behavior."""
    dist = np.full(n, 300.0)
    # SOC ramps from 20% down to 14% (below min_soc=15%)
    soc = np.linspace(20.0, 14.0, n)
    return Scenario(
        name="low_soc",
        title="Low SOC: discharge demand near min_soc guard",
        disturbance=dist,
        soc=soc,
        dt=np.full(n, 5.0),
    )


ALL_SCENARIOS = {
    "step_up": scenario_step_up,
    "step_down": scenario_step_down,
    "cloud": scenario_cloud,
    "charge_transition": scenario_charge_transition,
    "oscillating": scenario_oscillating,
    "low_soc": scenario_low_soc,
}


def scenario_real(traces: dict, start: int = 0, end: int | None = None) -> Scenario:
    """Extract a slice of the real disturbance trace."""
    d = np.array(traces["disturbance_w"])
    s = np.array(traces["soc_pct"])
    dt = np.array(traces["dt_s"])
    if end is None:
        end = len(d)
    return Scenario(
        name="real",
        title=f"Real data (steps {start}–{end})",
        disturbance=d[start:end],
        soc=s[start:end],
        dt=dt[start:end],
    )


# ── Plotting ─────────────────────────────────────────────────


def plot_scenario(
    sc: Scenario,
    gain_sets: dict[str, np.ndarray],
    plant_abc: tuple[float, float, float],
    sys_cfg: SystemConfig,
) -> plt.Figure:
    """Plot grid, desired, and battery power for each gain set on a scenario."""
    ta = make_trace_arrays(sc)
    n = ta.n
    t = np.cumsum(ta.dt) - ta.dt[0]  # time axis in seconds
    t_min = t / 60  # convert to minutes

    results: dict[str, SimResult] = {}
    for label, params in gain_sets.items():
        results[label] = simulate_fast(params, plant_abc, ta, sys_cfg)

    fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
    fig.suptitle(sc.title, fontsize=14, fontweight="bold")
    colors = plt.cm.tab10.colors

    # Panel 1: Grid power
    ax = axes[0]
    ax.set_ylabel("Grid power (W)")
    ax.axhline(sys_cfg.discharge_target_w, color="gray", ls="--", lw=0.8,
               label=f"target ({sys_cfg.discharge_target_w}W)")
    ax.axhline(0, color="black", ls="-", lw=0.5)
    ax.fill_between(t_min, 0, np.minimum(0, results[list(results)[0]].grid_w),
                     alpha=0.08, color="red", label="feed-in zone")
    for i, (label, r) in enumerate(results.items()):
        ax.plot(t_min, r.grid_w, color=colors[i % len(colors)], lw=1.2,
                label=label, alpha=0.85)
    ax.legend(fontsize=8, loc="upper right")
    ax.grid(True, alpha=0.3)

    # Panel 2: Desired output (setpoint)
    ax = axes[1]
    ax.set_ylabel("Desired output (W)")
    ax.axhline(0, color="black", ls="-", lw=0.5)
    for i, (label, r) in enumerate(results.items()):
        ax.plot(t_min, r.desired_w, color=colors[i % len(colors)], lw=1.2,
                label=label, alpha=0.85)
    ax.legend(fontsize=8, loc="upper right")
    ax.grid(True, alpha=0.3)

    # Panel 3: P and I terms
    ax = axes[2]
    ax.set_ylabel("P / I term (W)")
    ax.axhline(0, color="black", ls="-", lw=0.5)
    for i, (label, r) in enumerate(results.items()):
        c = colors[i % len(colors)]
        ax.plot(t_min, r.p_term_w, color=c, lw=1.0, ls="-",
                label=f"{label} P", alpha=0.85)
        ax.plot(t_min, r.i_term_w, color=c, lw=1.0, ls="--",
                label=f"{label} I", alpha=0.85)
    ax.legend(fontsize=8, loc="upper right")
    ax.grid(True, alpha=0.3)

    # Panel 4: Disturbance (input)
    ax = axes[3]
    ax.set_ylabel("Disturbance (W)")
    ax.set_xlabel("Time (min)")
    ax.plot(t_min, sc.disturbance[:n], color="gray", lw=1.0, alpha=0.7, label="disturbance")
    ax.axhline(0, color="black", ls="-", lw=0.5)
    ax.legend(fontsize=8, loc="upper right")
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    return fig


def print_scenario_metrics(
    sc: Scenario,
    gain_sets: dict[str, np.ndarray],
    plant_abc: tuple[float, float, float],
    sys_cfg: SystemConfig,
) -> None:
    """Print key metrics for each gain set on a scenario."""
    ta = make_trace_arrays(sc)
    print(f"\n  {sc.title}")
    print(f"  {'':22s}  {'Feed-in':>10s}  {'Max FI':>8s}  {'IAE':>12s}  {'Effort':>10s}")
    print("  " + "-" * 68)
    for label, params in gain_sets.items():
        r = simulate_fast(params, plant_abc, ta, sys_cfg)
        print(f"  {label:22s}  {r.feed_in_energy_wh:8.1f} Wh  {r.max_feed_in_w:6.0f} W"
              f"  {r.iae:10.0f} W·s  {r.control_effort:8.0f} W")


# ── Main ─────────────────────────────────────────────────────


def main():
    parser = argparse.ArgumentParser(
        description="Plot system response for different gain sets")
    parser.add_argument("-m", "--model", type=Path,
                        default=Path("data/plant_model.json"),
                        help="Plant model JSON")
    parser.add_argument("--result", type=Path, default=None,
                        help="Load optimized gains from result JSON (default: data/optimization_result.json)")
    parser.add_argument("--scenario", action="append", default=None,
                        choices=list(ALL_SCENARIOS) + ["real", "all"],
                        help="Scenarios to plot (default: all synthetic)")
    parser.add_argument("--real-range", default="0:2000",
                        help="Slice of real data trace as start:end (default: 0:2000)")
    parser.add_argument("--save", type=Path, default=None,
                        help="Save figures to directory instead of showing")
    args = parser.parse_args()

    # Load plant model
    plant_abc = DEFAULT_PLANT_ABC
    traces = None
    if args.model.exists():
        with open(args.model) as f:
            model_data = json.load(f)
        p = model_data["plant"]
        plant_abc = (p["a"], p["b"], p["c"])
        traces = model_data["traces"]
        print(f"  Plant model: tau={p['tau_s']:.1f}s  DC gain={p['dc_gain']:.3f}")

    # Build gain sets to compare
    gain_sets: dict[str, np.ndarray] = {"current": CURRENT_GAINS}

    result_path = args.result if args.result is not None else DEFAULT_RESULT

    if result_path.exists():
        result_data = load_result(result_path)
        gain_sets["optimized"] = np.array(result_data["param_vector"])
        gen = result_data["optimization"]["generations"]
        pct = result_data["improvement_pct"]["feed_in"]
        print(f"  Loaded optimized gains from {result_path} "
              f"(gen {gen}, feed-in {pct:+.1f}%)")
    else:
        print(f"  No result file found at {result_path} — showing current gains only")

    # Select scenarios
    scenario_names = args.scenario or ["all"]
    if "all" in scenario_names:
        scenario_names = list(ALL_SCENARIOS)
        if traces is not None:
            scenario_names.append("real")

    scenarios: list[Scenario] = []
    for name in scenario_names:
        if name == "real":
            if traces is None:
                print(f"  WARNING: --scenario real requires plant model with traces")
                continue
            parts = args.real_range.split(":")
            start = int(parts[0])
            end = int(parts[1]) if len(parts) > 1 and parts[1] else None
            scenarios.append(scenario_real(traces, start, end))
        else:
            scenarios.append(ALL_SCENARIOS[name]())

    print(f"\n  Gain sets: {', '.join(gain_sets.keys())}")
    print(f"  Scenarios: {', '.join(s.name for s in scenarios)}")

    # Print metrics table
    for sc in scenarios:
        print_scenario_metrics(sc, gain_sets, plant_abc, DEFAULT_SYS_CFG)

    # Plot
    figs: list[tuple[str, plt.Figure]] = []
    for sc in scenarios:
        fig = plot_scenario(sc, gain_sets, plant_abc, DEFAULT_SYS_CFG)
        figs.append((sc.name, fig))

    if args.save:
        args.save.mkdir(parents=True, exist_ok=True)
        for name, fig in figs:
            path = args.save / f"response_{name}.png"
            fig.savefig(path, dpi=150, bbox_inches="tight")
            print(f"  Saved: {path}")
        plt.close("all")
    else:
        plt.show()


if __name__ == "__main__":
    main()
