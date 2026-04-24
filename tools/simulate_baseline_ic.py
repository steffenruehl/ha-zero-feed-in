#!/usr/bin/env python3
"""Simulate sign-flip detector + I-controller baseline (Approach B+C).

Detection: sign-flip detector from Approach B.
Baseline: initial seed from min(grid + battery), then I-controller
corrects based on observed min(grid) vs target.

Usage:
    python3 tools/simulate_baseline_ic.py data/zfi_controller_2026-04-24.csv [--compact]
"""
from __future__ import annotations

import sys
from collections import deque
from datetime import datetime


# ── Detector parameters (same as Approach B) ─────────────
FLIP_THRESH_W = 500.0
FLIP_WINDOW_S = 20.0
ACTIVATE_COUNT = 2
ACTIVATE_WINDOW_S = 120.0
DEACTIVATE_S = 120.0

# ── Baseline I-controller parameters ─────────────────────
TARGET_W = 30.0              # desired grid power (controller target)
BASELINE_KI = 0.3            # correction gain per cycle
CYCLE_WINDOW_S = 60.0        # window for observing min(grid)
SEED_SAMPLES = 3             # export samples needed to seed baseline


class SignFlipDetector:
    """Detects futile controller responses via grid sign flips."""

    def __init__(self) -> None:
        self.recent: deque[tuple[float, float]] = deque()
        self.flip_times: deque[float] = deque()
        self.active: bool = False
        self.last_flip_t: float = 0.0
        self._last_flip_check_t: float = 0.0

    def _prune_recent(self, now: float) -> None:
        cutoff = now - FLIP_WINDOW_S
        while self.recent and self.recent[0][0] < cutoff:
            self.recent.popleft()

    def _prune_flips(self, now: float) -> None:
        cutoff = now - ACTIVATE_WINDOW_S
        while self.flip_times and self.flip_times[0] < cutoff:
            self.flip_times.popleft()

    def _check_flip(self, now: float) -> bool:
        has_pos = any(g > FLIP_THRESH_W for _, g in self.recent)
        has_neg = any(g < -FLIP_THRESH_W for _, g in self.recent)
        return has_pos and has_neg

    def update(self, grid_w: float, t: float) -> bool:
        self.recent.append((t, grid_w))
        self._prune_recent(t)

        is_flip = self._check_flip(t)
        if is_flip and (t - self._last_flip_check_t) > FLIP_WINDOW_S:
            self.flip_times.append(t)
            self.last_flip_t = t
            self._last_flip_check_t = t

        self._prune_flips(t)

        if not self.active:
            if len(self.flip_times) >= ACTIVATE_COUNT:
                self.active = True
        else:
            if (t - self.last_flip_t) > DEACTIVATE_S:
                self.active = False

        return self.active


class BaselineController:
    """I-controller that converges baseline from min(grid+battery) seed.

    Seed: min(grid + battery) from first few export samples (overestimate).
    Correction: every CYCLE_WINDOW_S, observe min(grid) and adjust:
        baseline += ki * (min_grid - target)
    If min_grid < target: we're exporting too much → decrease baseline.
    If min_grid > target: we're importing → increase baseline.
    """

    def __init__(self) -> None:
        self.baseline: float | None = None
        self.seeded: bool = False
        self._export_sums: list[float] = []  # grid+battery at export moments
        self._cycle_start_t: float = 0.0
        self._min_grid_in_cycle: float = float("inf")
        self._corrections: list[tuple[float, float, float]] = []  # (t, min_grid, new_baseline)

    def update(self, grid_w: float, bat_w: float, t: float, active: bool) -> float | None:
        """Process one sample. Returns baseline if active and seeded, else None."""
        if not active:
            # Reset on deactivation
            self.baseline = None
            self.seeded = False
            self._export_sums.clear()
            self._min_grid_in_cycle = float("inf")
            return None

        # Phase 1: Seed from min(grid + battery) at export moments
        if not self.seeded:
            if grid_w < -FLIP_THRESH_W:
                self._export_sums.append(grid_w + bat_w)
            if len(self._export_sums) >= SEED_SAMPLES:
                self.baseline = min(self._export_sums)
                self.seeded = True
                self._cycle_start_t = t
                self._min_grid_in_cycle = grid_w
            return self.baseline

        # Phase 2: I-controller correction
        # Track min grid in current cycle window
        if grid_w < self._min_grid_in_cycle:
            self._min_grid_in_cycle = grid_w

        # Apply correction at end of each cycle window
        if t - self._cycle_start_t >= CYCLE_WINDOW_S:
            error = self._min_grid_in_cycle - TARGET_W
            correction = BASELINE_KI * error
            self.baseline += correction
            self._corrections.append((t, self._min_grid_in_cycle, self.baseline))
            # Reset for next cycle
            self._cycle_start_t = t
            self._min_grid_in_cycle = grid_w

        return self.baseline

    @property
    def corrections(self) -> list[tuple[float, float, float]]:
        return self._corrections


def parse_timestamp(ts: str) -> float:
    dt = datetime.fromisoformat(ts)
    return dt.timestamp()


def format_time(t: float) -> str:
    return datetime.fromtimestamp(t).strftime("%H:%M:%S")


def main() -> None:
    if len(sys.argv) < 2:
        print("Usage: simulate_baseline_ic.py <controller_csv> [--compact]")
        sys.exit(1)

    csv_path = sys.argv[1]
    compact = "--compact" in sys.argv

    detector = SignFlipDetector()
    baseline_ctrl = BaselineController()

    # Parse CSV
    rows: list[tuple[float, float, float]] = []
    with open(csv_path) as f:
        header = f.readline().strip().split(",")
        grid_idx = header.index("grid_w")
        bat_idx = header.index("battery_power_w")
        for line in f:
            parts = line.strip().split(",")
            if len(parts) <= max(grid_idx, bat_idx):
                continue
            t = parse_timestamp(parts[0])
            grid = float(parts[grid_idx])
            bat = float(parts[bat_idx])
            rows.append((t, grid, bat))

    # Print header
    print(f"{'Time':>10}  {'Grid':>6}  {'Batt':>6}  "
          f"{'Active':>6}  {'Baseln':>6}  Note")
    print("─" * 72)

    prev_active = False
    prev_baseline = None

    for t, grid, bat in rows:
        active = detector.update(grid, t)
        baseline = baseline_ctrl.update(grid, bat, t, active)

        note = ""
        if active and not prev_active:
            note = ">>> ACTIVATED"
        elif not active and prev_active:
            note = "<<< DEACTIVATED"
        elif baseline is not None and prev_baseline is None:
            note = f"SEEDED baseline={baseline:.0f}"
        elif (baseline is not None and prev_baseline is not None
              and abs(baseline - prev_baseline) > 0.5):
            note = f"correction → {baseline:.0f}"

        baseline_str = f"{baseline:6.0f}" if baseline is not None else "   ---"

        if compact:
            if note or (active != prev_active):
                print(f"{format_time(t):>10}  {grid:>6.0f}  {bat:>6.0f}  "
                      f"{'YES' if active else 'no':>6}  {baseline_str}  {note}")
        else:
            if abs(grid) > 200 or active or note:
                print(f"{format_time(t):>10}  {grid:>6.0f}  {bat:>6.0f}  "
                      f"{'YES' if active else 'no':>6}  {baseline_str}  {note}")

        prev_active = active
        prev_baseline = baseline

    # Print correction summary
    if baseline_ctrl.corrections:
        print("\n── Baseline Corrections ────────────────────────────────")
        print(f"{'Time':>10}  {'MinGrid':>7}  {'Baseline':>8}")
        for ct, mg, bl in baseline_ctrl.corrections:
            print(f"{format_time(ct):>10}  {mg:>7.0f}  {bl:>8.1f}")


if __name__ == "__main__":
    main()
