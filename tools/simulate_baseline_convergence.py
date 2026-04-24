#!/usr/bin/env python3
"""Simulate I-controller baseline convergence with a simple plant model.

Since recorded data has the controller chasing oscillations, we can't
close the loop from it. Instead, this script models:
- The filter outputs a baseline value to the controller
- The controller drives battery to (baseline - target)
- Actual grid = house_load + oven_power - battery + inverter_loss
- We observe min(grid) during oven OFF phases and correct baseline

This tests whether the I-controller converges from the Option 3 seed.

Usage:
    python3 tools/simulate_baseline_convergence.py
"""
from __future__ import annotations


# ── Plant model parameters ───────────────────────────────
TRUE_BASELINE_W = 30.0       # actual house load (what we want to find)
TARGET_W = 30.0              # controller target (desired grid import)
OVEN_POWER_W = 1300.0        # oven element power
OVEN_ON_S = 15.0             # seconds ON per cycle
OVEN_OFF_S = 45.0            # seconds OFF per cycle
INVERTER_LOSS_W = 100.0      # battery sensor overreads by this much
BATTERY_DELAY_S = 12.0       # controller response latency

# ── I-controller parameters ──────────────────────────────
BASELINE_KI = 0.3            # correction gain per cycle
CYCLE_WINDOW_S = 60.0        # observation window


def simulate(initial_baseline: float, n_cycles: int = 20) -> None:
    """Run the convergence simulation."""
    baseline = initial_baseline
    dt = 5.0  # sample interval

    print(f"Initial baseline: {baseline:.0f} W  (true: {TRUE_BASELINE_W:.0f} W, "
          f"error: {baseline - TRUE_BASELINE_W:+.0f} W)")
    print(f"Target: {TARGET_W:.0f} W, Inverter loss: {INVERTER_LOSS_W:.0f} W")
    print(f"Oven: {OVEN_POWER_W:.0f} W, {OVEN_ON_S:.0f}s ON / {OVEN_OFF_S:.0f}s OFF")
    print()
    print(f"{'Cycle':>5}  {'Baseline':>8}  {'BatOut':>6}  "
          f"{'GridOFF':>7}  {'GridON':>6}  {'MinGrid':>7}  "
          f"{'Error':>7}  {'Corr':>6}  {'BLerr':>6}")
    print("─" * 80)

    for cycle in range(n_cycles):
        # Controller drives battery based on filter output (baseline)
        # Controller sees baseline, wants grid=target, so:
        # battery_cmd = baseline - target (what to discharge)
        battery_cmd = baseline - TARGET_W
        if battery_cmd < 0:
            battery_cmd = 0  # can't charge in discharge mode

        # Actual battery output (what the sensor reports)
        battery_actual = battery_cmd  # assume controller achieves target

        # During oven OFF (steady state, battery at battery_cmd):
        # grid = house_load - battery_actual
        grid_off = TRUE_BASELINE_W - battery_actual

        # During oven ON:
        # grid = house_load + oven_power - battery_actual
        grid_on = TRUE_BASELINE_W + OVEN_POWER_W - battery_actual

        # The sensor reports battery_actual + inverter_loss
        # So min(grid + battery_sensor) = grid_off + (battery_actual + inverter_loss)
        #                               = TRUE_BASELINE + INVERTER_LOSS
        # This is the Option 3 value — constant regardless of baseline setting

        # What the I-controller observes:
        min_grid = grid_off  # minimum grid is during oven OFF

        # I-controller correction
        error = min_grid - TARGET_W
        correction = BASELINE_KI * error
        baseline_err = baseline - TRUE_BASELINE_W

        print(f"{cycle:>5}  {baseline:>8.1f}  {battery_cmd:>6.0f}  "
              f"{grid_off:>7.1f}  {grid_on:>6.0f}  {min_grid:>7.1f}  "
              f"{error:>7.1f}  {correction:>6.1f}  {baseline_err:>+6.0f}")

        baseline += correction

    print()
    print(f"Final baseline: {baseline:.1f} W  (true: {TRUE_BASELINE_W:.0f} W, "
          f"error: {baseline - TRUE_BASELINE_W:+.1f} W)")


def main() -> None:
    print("=" * 80)
    print("Scenario 1: April 24 — seed from min(grid+battery) ≈ 107 W")
    print("=" * 80)
    simulate(initial_baseline=107.0)

    print()
    print("=" * 80)
    print("Scenario 2: April 21 — seed from min(grid+battery) ≈ 200 W")
    print("=" * 80)
    simulate(initial_baseline=200.0)

    print()
    print("=" * 80)
    print("Scenario 3: Worst case — seed = 500 W (very stale estimate)")
    print("=" * 80)
    simulate(initial_baseline=500.0)


if __name__ == "__main__":
    main()
