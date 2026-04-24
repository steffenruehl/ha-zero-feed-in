#!/usr/bin/env python3
"""Simulate the pulse-load filter against real oven data.

Reads timestamp,grid_w pairs and runs the HP+energy detector
and running median baseline filter, printing a table of results.
"""
from __future__ import annotations

import math
import statistics
from collections import deque
from datetime import datetime


# ── Filter parameters (matching the sketch) ──────────────
HP_TAU_S = 60.0
ENERGY_ALPHA = 0.05
ENERGY_THRESHOLD_W = 300.0
ENERGY_HYSTERESIS = 0.5
HISTORY_S = 120.0         # keep 2 min of raw history for baseline lookback
LOOKBACK_S = 30.0         # step this far back in time at activation
BASELINE_ALPHA = 0.05     # slow EMA for calm baseline tracking
CALM_WINDOW_S = 60.0      # check last 60s of raw samples for calm
CALM_BAND_W = 200.0       # max-min within calm window to declare "calm"
CALM_HP_MAX_W = 400.0     # |HP| must be below this to allow calm deactivation


# ── Filter state ─────────────────────────────────────────
class FilterSim:
    def __init__(self) -> None:
        self.prev_raw: float | None = None
        self.prev_t: float | None = None
        self.hp: float = 0.0
        self.energy_sq: float = 0.0
        self.active: bool = False
        self.calm_baseline: float | None = None
        self.frozen_baseline: float = 0.0
        self.method: str = ""
        # Store (t, raw, calm_ema) so lookback can retrieve the EMA value
        self.history: deque[tuple[float, float, float]] = deque()

    @property
    def energy(self) -> float:
        return math.sqrt(self.energy_sq)

    def _prune_history(self, now: float) -> None:
        """Drop history samples older than HISTORY_S."""
        cutoff = now - HISTORY_S
        while self.history and self.history[0][0] < cutoff:
            self.history.popleft()

    def _calm_ema_at(self, before_t: float) -> float:
        """Retrieve the calm EMA snapshot from LOOKBACK_S before the given time."""
        target_t = before_t - LOOKBACK_S
        best_ema = 0.0
        for t, _raw, ema in self.history:
            if t > target_t:
                break
            best_ema = ema
        return best_ema

    def _is_calm(self, now: float) -> bool:
        """Check if all recent raw samples are within a tight band."""
        cutoff = now - CALM_WINDOW_S
        recent = [v for t, v, _ in self.history if t >= cutoff]
        if len(recent) < 3:
            return False
        return (max(recent) - min(recent)) <= CALM_BAND_W

    def update(self, raw_w: float, t: float) -> float:
        """Process one sample, return filtered value."""
        # Update calm baseline EMA (always, even when active — but only from raw)
        if self.calm_baseline is None:
            self.calm_baseline = raw_w
        elif not self.active:
            self.calm_baseline = (
                BASELINE_ALPHA * raw_w
                + (1 - BASELINE_ALPHA) * self.calm_baseline
            )

        self.history.append((t, raw_w, self.calm_baseline))
        self._prune_history(t)

        # HP IIR
        if self.prev_raw is not None and self.prev_t is not None:
            dt = t - self.prev_t
            if dt > 0:
                alpha = HP_TAU_S / (HP_TAU_S + dt)
                self.hp = alpha * (self.hp + raw_w - self.prev_raw)
        self.prev_raw = raw_w
        self.prev_t = t

        # Energy envelope
        self.energy_sq = (
            ENERGY_ALPHA * self.hp * self.hp
            + (1 - ENERGY_ALPHA) * self.energy_sq
        )

        # State transitions
        energy = self.energy
        if not self.active:
            if energy > ENERGY_THRESHOLD_W:
                self.active = True
                # Retrieve clean EMA from before contamination
                self.frozen_baseline = self._calm_ema_at(t)
                self.method = f"FREEZE({self.frozen_baseline:.0f})"
        else:
            # Deactivate via calm detector OR energy decay
            if self._is_calm(t) and abs(self.hp) < CALM_HP_MAX_W:
                self.active = False
                self.calm_baseline = raw_w
                # Reset energy so it must re-accumulate to re-activate
                self.hp = 0.0
                self.energy_sq = 0.0
                self.method = "raw (calm deactivation)"
            elif energy < ENERGY_THRESHOLD_W * ENERGY_HYSTERESIS:
                self.active = False
                self.calm_baseline = raw_w
                self.method = "raw (energy deactivation)"

        # Output
        if self.active:
            self.method = f"hold({self.frozen_baseline:.0f})"
            return self.frozen_baseline
        self.method = f"raw (calm={self.calm_baseline:.0f})" if self.calm_baseline else "raw"
        return raw_w


# ── Load data ────────────────────────────────────────────
DATA_RAW = """
2026-04-24T15:48:02,50
2026-04-24T15:48:18,3
2026-04-24T15:48:38,59
2026-04-24T15:48:54,2
2026-04-24T15:49:29,1340
2026-04-24T15:49:44,-1258
2026-04-24T15:50:06,5
2026-04-24T15:50:31,1318
2026-04-24T15:50:46,-1235
2026-04-24T15:51:02,14
2026-04-24T15:51:29,116
2026-04-24T15:51:44,-67
2026-04-24T15:52:19,34
2026-04-24T15:52:35,1284
2026-04-24T15:52:50,-1220
2026-04-24T15:53:10,12
2026-04-24T15:53:36,1301
2026-04-24T15:53:52,-1223
2026-04-24T15:54:08,13
2026-04-24T15:54:34,1353
2026-04-24T15:54:50,-1272
2026-04-24T15:55:06,11
2026-04-24T15:55:37,1316
2026-04-24T15:55:52,-1235
2026-04-24T15:56:07,11
2026-04-24T15:56:39,1337
2026-04-24T15:56:54,-1257
2026-04-24T15:57:10,2
2026-04-24T15:57:36,37
2026-04-24T15:57:51,13
2026-04-24T15:58:43,1317
2026-04-24T15:59:00,-1235
2026-04-24T15:59:20,12
2026-04-24T15:59:46,1320
2026-04-24T16:00:01,-1246
2026-04-24T16:00:22,12
2026-04-24T16:00:49,1317
2026-04-24T16:01:01,-1231
2026-04-24T16:01:16,8
2026-04-24T16:01:47,1384
2026-04-24T16:02:02,-1305
2026-04-24T16:02:17,9
2026-04-24T16:02:32,2216
2026-04-24T16:02:48,2108
2026-04-24T16:03:03,742
2026-04-24T16:03:19,749
2026-04-24T16:03:35,757
2026-04-24T16:03:50,2094
2026-04-24T16:04:06,733
2026-04-24T16:04:21,731
2026-04-24T16:04:36,-1407
2026-04-24T16:04:51,2139
2026-04-24T16:05:06,756
2026-04-24T16:05:22,734
2026-04-24T16:05:37,-1414
2026-04-24T16:05:52,12
2026-04-24T16:06:27,24
2026-04-24T16:06:48,42
2026-04-24T16:07:03,1282
2026-04-24T16:07:20,-1217
2026-04-24T16:07:40,2202
2026-04-24T16:07:56,-1413
2026-04-24T16:08:11,8
2026-04-24T16:08:47,2201
2026-04-24T16:09:03,-114
2026-04-24T16:09:18,915
2026-04-24T16:09:33,-1413
2026-04-24T16:09:49,2221
2026-04-24T16:10:04,-87
2026-04-24T16:10:20,-1300
2026-04-24T16:10:37,12
2026-04-24T16:10:57,2178
2026-04-24T16:11:12,-1408
2026-04-24T16:11:27,2158
2026-04-24T16:11:43,-1406
2026-04-24T16:11:58,2
2026-04-24T16:12:34,32
2026-04-24T16:12:49,10
2026-04-24T16:13:15,32
2026-04-24T16:13:36,17
2026-04-24T16:13:56,44
2026-04-24T16:14:28,11
2026-04-24T16:15:34,33
2026-04-24T16:15:49,14
2026-04-24T16:16:31,26
2026-04-24T16:16:46,51
2026-04-24T16:17:07,22
2026-04-24T16:17:29,43
2026-04-24T16:17:49,15
2026-04-24T16:18:04,47
2026-04-24T16:18:46,24
2026-04-24T16:19:01,-19
2026-04-24T16:19:28,21
2026-04-24T16:19:43,48
""".strip()


def parse_data(filepath: str | None = None) -> list[tuple[float, float]]:
    """Parse CSV into (epoch_seconds, grid_w) pairs.
    
    If filepath given, read from file. Otherwise use inline DATA_RAW.
    """
    result = []
    if filepath:
        with open(filepath) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("timestamp"):
                    continue
                ts_str, val_str = line.split(",", 1)
                t = datetime.fromisoformat(ts_str).timestamp()
                result.append((t, float(val_str)))
    else:
        for line in DATA_RAW.splitlines():
            ts_str, val_str = line.split(",")
            t = datetime.fromisoformat(ts_str).timestamp()
            result.append((t, float(val_str)))
    return result


def main() -> None:
    import sys
    filepath = sys.argv[1] if len(sys.argv) > 1 else None
    compact = "--compact" in sys.argv
    data = parse_data(filepath)
    filt = FilterSim()

    print(f"{'Time':>8s}  {'Raw':>7s}  {'Filtered':>8s}  {'HP':>7s}  {'Energy':>7s}  {'Active':>6s}  {'Method'}")
    print("─" * 80)

    prev_active = False
    for t, raw_w in data:
        filtered = filt.update(raw_w, t)
        ts_short = datetime.fromtimestamp(t).strftime("%H:%M:%S")
        
        # In compact mode: only show transitions, active samples, and a few surrounding
        show = True
        if compact:
            is_interesting = (
                filt.active  # currently filtering
                or filt.active != prev_active  # transition
                or abs(raw_w) > 500  # large raw value
                or filt.energy > 100  # energy building up
            )
            show = is_interesting
        
        if show:
            print(
                f"{ts_short:>8s}  {raw_w:>7.0f}  {filtered:>8.1f}  "
                f"{filt.hp:>7.0f}  {filt.energy:>7.1f}  "
                f"{'YES' if filt.active else 'no':>6s}  "
                f"{filt.method}"
            )
        prev_active = filt.active


if __name__ == "__main__":
    main()
