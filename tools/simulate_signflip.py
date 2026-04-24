#!/usr/bin/env python3
"""Simulate the sign-flip detector (Approach B) against controller CSV data.

Reads the controller CSV (timestamp, grid_w, battery_power_w columns)
and detects futile-response sign flips in the grid signal.

A sign flip is: within a FLIP_WINDOW_S window, there exists a sample
with grid > +FLIP_THRESH_W AND a sample with grid < -FLIP_THRESH_W.

Activation: >= ACTIVATE_COUNT flips within ACTIVATE_WINDOW_S
Deactivation: no flips for DEACTIVATE_S

Usage:
    python3 tools/simulate_signflip.py data/zfi_controller_2026-04-24.csv [--compact]
"""
from __future__ import annotations

import sys
from collections import deque
from datetime import datetime


# ── Detector parameters ──────────────────────────────────
FLIP_THRESH_W = 500.0       # grid magnitude for a flip leg
FLIP_WINDOW_S = 20.0        # max time between positive and negative legs
ACTIVATE_COUNT = 2           # sign flips needed to activate
ACTIVATE_WINDOW_S = 120.0   # window for counting flips
DEACTIVATE_S = 120.0        # no-flip duration to deactivate


class SignFlipDetector:
    """Detects futile controller responses via grid sign flips."""

    def __init__(self) -> None:
        # Recent grid samples: (t, grid_w)
        self.recent: deque[tuple[float, float]] = deque()
        # Timestamps of detected sign flips
        self.flip_times: deque[float] = deque()
        self.active: bool = False
        self.last_flip_t: float = 0.0
        self._last_flip_check_t: float = 0.0

    def _prune_recent(self, now: float) -> None:
        """Keep only samples within FLIP_WINDOW_S."""
        cutoff = now - FLIP_WINDOW_S
        while self.recent and self.recent[0][0] < cutoff:
            self.recent.popleft()

    def _prune_flips(self, now: float) -> None:
        """Keep only flips within ACTIVATE_WINDOW_S."""
        cutoff = now - ACTIVATE_WINDOW_S
        while self.flip_times and self.flip_times[0] < cutoff:
            self.flip_times.popleft()

    def _check_flip(self, now: float) -> bool:
        """Check if current recent window contains a sign flip."""
        has_pos = any(g > FLIP_THRESH_W for _, g in self.recent)
        has_neg = any(g < -FLIP_THRESH_W for _, g in self.recent)
        return has_pos and has_neg

    def update(self, grid_w: float, t: float) -> bool:
        """Process one sample, return whether detector is active."""
        self.recent.append((t, grid_w))
        self._prune_recent(t)

        # Check for new sign flip (only register once per flip event)
        is_flip = self._check_flip(t)
        if is_flip and (t - self._last_flip_check_t) > FLIP_WINDOW_S:
            self.flip_times.append(t)
            self.last_flip_t = t
            self._last_flip_check_t = t

        self._prune_flips(t)

        # State transitions
        if not self.active:
            if len(self.flip_times) >= ACTIVATE_COUNT:
                self.active = True
        else:
            if (t - self.last_flip_t) > DEACTIVATE_S:
                self.active = False

        return self.active


def parse_timestamp(ts: str) -> float:
    """Parse ISO timestamp to epoch seconds."""
    dt = datetime.fromisoformat(ts)
    return dt.timestamp()


def format_time(t: float) -> str:
    """Format epoch seconds to HH:MM:SS."""
    return datetime.fromtimestamp(t).strftime("%H:%M:%S")


def main() -> None:
    if len(sys.argv) < 2:
        print("Usage: simulate_signflip.py <controller_csv> [--compact]")
        sys.exit(1)

    csv_path = sys.argv[1]
    compact = "--compact" in sys.argv

    detector = SignFlipDetector()

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
          f"{'Flips':>5}  {'Active':>6}  Note")
    print("─" * 70)

    prev_active = False
    prev_flips = 0

    for t, grid, bat in rows:
        active = detector.update(grid, t)
        n_flips = len(detector.flip_times)

        # Determine note
        note = ""
        if active and not prev_active:
            note = ">>> ACTIVATED"
        elif not active and prev_active:
            note = "<<< DEACTIVATED"
        elif n_flips > prev_flips:
            note = f"FLIP #{n_flips}"

        if compact:
            # Only print state changes and flips
            if note or (active != prev_active):
                print(f"{format_time(t):>10}  {grid:>6.0f}  {bat:>6.0f}  "
                      f"{n_flips:>5}  {'YES' if active else 'no':>6}  {note}")
        else:
            # Only print when something interesting (grid > 200 or active or note)
            if abs(grid) > 200 or active or note:
                print(f"{format_time(t):>10}  {grid:>6.0f}  {bat:>6.0f}  "
                      f"{n_flips:>5}  {'YES' if active else 'no':>6}  {note}")

        prev_active = active
        prev_flips = n_flips


if __name__ == "__main__":
    main()
