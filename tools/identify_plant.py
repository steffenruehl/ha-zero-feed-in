#!/usr/bin/env python3
"""Identify a plant model from zero-feed-in controller CSV logs.

Reads controller log CSVs, fits a first-order discrete plant model
(battery_power as a function of desired_power), and writes the model
parameters plus extracted disturbance trace to a JSON file that
optimize_gains.py can consume.

Usage:
    python3 tools/identify_plant.py data/controller_2026-04-21.csv data/controller_2026-04-22.csv

Output:
    data/plant_model.json
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd


def load_logs(paths: list[Path]) -> pd.DataFrame:
    """Load and concatenate controller CSVs, coercing numeric columns."""
    dfs = []
    for p in paths:
        df = pd.read_csv(p, parse_dates=["timestamp"])
        dfs.append(df)
    df = pd.concat(dfs, ignore_index=True).sort_values("timestamp").reset_index(drop=True)

    num_cols = [
        "grid_w", "soc_pct", "battery_power_w", "desired_power_w",
        "surplus_w", "error_w", "target_w", "p_term", "i_term",
        "ff_term", "integral", "kp_used", "ki_used",
    ]
    for col in num_cols:
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors="coerce")
    df = df.dropna(subset=["grid_w", "battery_power_w", "desired_power_w"])
    return df


def identify_plant(df: pd.DataFrame) -> dict:
    """Fit batt[k] = a * batt[k-1] + b * desired[k-d] + c via least squares.

    Tries delay d = 0, 1, 2, 3 steps and picks the delay with the
    highest R².  Only uses rows where the controller is active
    (desired != 0) and the tick interval is close to nominal (4-6 s)
    to avoid transient artefacts.

    The fitted delay accounts for the device's command-to-effect
    transport delay (~10-15 s for the Zendure SolarFlow).
    """
    df = df.copy()
    df["dt"] = df["timestamp"].diff().dt.total_seconds()
    df["batt_prev"] = df["battery_power_w"].shift(1)

    mask = (
        df["dt"].between(4, 6)
        & df["desired_power_w"].abs().gt(10)
        & df["batt_prev"].notna()
    )

    best: dict | None = None
    best_r2 = -1.0

    for delay in range(4):  # d = 0, 1, 2, 3
        df[f"desired_d{delay}"] = df["desired_power_w"].shift(delay)
        delay_mask = mask & df[f"desired_d{delay}"].notna()
        active = df.loc[delay_mask]
        if len(active) < 100:
            continue

        X = np.column_stack([
            active["batt_prev"].values,
            active[f"desired_d{delay}"].values,
            np.ones(len(active)),
        ])
        y = active["battery_power_w"].values
        coeffs, _, _, _ = np.linalg.lstsq(X, y, rcond=None)
        a, b, c = coeffs

        y_pred = X @ coeffs
        ss_res = np.sum((y - y_pred) ** 2)
        ss_tot = np.sum((y - y.mean()) ** 2)
        r_squared = 1 - ss_res / ss_tot if ss_tot > 0 else 0.0

        if r_squared > best_r2:
            best_r2 = r_squared
            best = {"a": a, "b": b, "c": c, "delay": delay,
                    "r_squared": r_squared, "n_samples": len(active)}

    if best is None:
        print("WARNING: insufficient active rows for any delay — model unreliable",
              file=sys.stderr)
        best = {"a": 0.0, "b": 0.0, "c": 0.0, "delay": 0,
                "r_squared": 0.0, "n_samples": 0}

    a, b, c = best["a"], best["b"], best["c"]
    tau_s = -5.0 / np.log(a) if 0 < a < 1 else float("inf")
    dc_gain = b / (1 - a) if abs(1 - a) > 1e-9 else float("inf")

    # Print all delay candidates for comparison.
    print("  Delay scan (d=steps, each 5s):")
    for delay in range(4):
        col = f"desired_d{delay}"
        if col not in df.columns:
            continue
        delay_mask = mask & df[col].notna()
        active = df.loc[delay_mask]
        if len(active) < 100:
            print(f"    d={delay}: insufficient data")
            continue
        X = np.column_stack([
            active["batt_prev"].values,
            active[col].values,
            np.ones(len(active)),
        ])
        y = active["battery_power_w"].values
        coeffs_d, _, _, _ = np.linalg.lstsq(X, y, rcond=None)
        y_pred = X @ coeffs_d
        ss_res = np.sum((y - y_pred) ** 2)
        ss_tot = np.sum((y - y.mean()) ** 2)
        r2 = 1 - ss_res / ss_tot if ss_tot > 0 else 0.0
        marker = " ◄" if delay == best["delay"] else ""
        print(f"    d={delay}: R²={r2:.4f}  a={coeffs_d[0]:.4f}  b={coeffs_d[1]:.4f}  c={coeffs_d[2]:.2f}{marker}")

    return {
        "a": round(float(a), 6),
        "b": round(float(b), 6),
        "c": round(float(c), 4),
        "delay_steps": best["delay"],
        "tau_s": round(tau_s, 1),
        "dc_gain": round(dc_gain, 4),
        "r_squared": round(float(best_r2), 4),
        "n_samples": best["n_samples"],
    }


def extract_traces(df: pd.DataFrame) -> dict:
    """Extract disturbance trace and supporting data for simulation.

    disturbance[k] = grid_w[k] + battery_power_w[k]
    This is the net load minus PV — the part the controller can't influence.
    """
    dt = df["timestamp"].diff().dt.total_seconds().fillna(5.0).values
    disturbance = (df["grid_w"] + df["battery_power_w"]).values
    soc = df["soc_pct"].values
    # Actual values for validation
    grid_actual = df["grid_w"].values
    desired_actual = df["desired_power_w"].values
    battery_actual = df["battery_power_w"].values
    timestamps = df["timestamp"].dt.strftime("%Y-%m-%dT%H:%M:%S").tolist()

    # Gains that were active during this data (for validation)
    kp_values = sorted(df["kp_used"].dropna().unique().tolist()) if "kp_used" in df.columns else []
    ki_values = sorted(df["ki_used"].dropna().unique().tolist()) if "ki_used" in df.columns else []

    return {
        "timestamps": timestamps,
        "dt_s": [round(float(x), 2) for x in dt],
        "disturbance_w": [round(float(x), 1) for x in disturbance],
        "soc_pct": [round(float(x), 1) for x in soc],
        "grid_actual_w": [round(float(x), 1) for x in grid_actual],
        "desired_actual_w": [round(float(x), 1) for x in desired_actual],
        "battery_actual_w": [round(float(x), 1) for x in battery_actual],
        "kp_values_seen": kp_values,
        "ki_values_seen": ki_values,
    }


def main():
    parser = argparse.ArgumentParser(description="Identify plant model from controller logs")
    parser.add_argument("csvs", nargs="+", type=Path, help="Controller CSV log files")
    parser.add_argument("-o", "--output", type=Path, default=Path("data/plant_model.json"),
                        help="Output JSON file (default: data/plant_model.json)")
    args = parser.parse_args()

    print(f"Loading {len(args.csvs)} log file(s)...")
    df = load_logs(args.csvs)
    print(f"  {len(df)} rows, {df.timestamp.min()} to {df.timestamp.max()}")

    print("\nIdentifying plant model...")
    plant = identify_plant(df)
    print(f"  batt[k] = {plant['a']:.4f} · batt[k-1] + {plant['b']:.4f} · desired[k-{plant['delay_steps']}] + {plant['c']:.2f}")
    print(f"  τ = {plant['tau_s']:.1f}s, DC gain = {plant['dc_gain']:.3f}, R² = {plant['r_squared']:.4f}")
    print(f"  Transport delay: {plant['delay_steps']} steps ({plant['delay_steps'] * 5.0:.0f}s at 5s sampling)")
    print(f"  Fitted on {plant['n_samples']} active ticks")

    print("\nExtracting disturbance traces...")
    traces = extract_traces(df)
    print(f"  {len(traces['disturbance_w'])} timesteps")
    if traces["kp_values_seen"]:
        print(f"  Kp values seen: {traces['kp_values_seen']}")
        print(f"  Ki values seen: {traces['ki_values_seen']}")

    output = {"plant": plant, "traces": traces}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with open(args.output, "w") as f:
        json.dump(output, f, indent=2)
    print(f"\nWritten to {args.output} ({args.output.stat().st_size / 1024:.0f} KB)")


if __name__ == "__main__":
    main()
