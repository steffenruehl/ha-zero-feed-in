# Pulse-Load Filter — Approach Comparison

## Problem

The Zendure SolarFlow 2400 AC+ has a device response latency of 10–15 s.
Appliances with bang-bang temperature control (e.g. oven: 15 s ON at ~1300 W /
45 s OFF) create periodic power pulses that the battery cannot track. The
controller oscillates in phase with the pulses, exporting energy during OFF
gaps that is permanently lost.

**Goal:** A standalone filter that sits between the grid sensor and the
controller. During periodic pulsing, it outputs a stable *baseline* (the grid
power level without the pulsing appliance). When no pulsing is detected, it
passes the raw value through transparently.

**Key constraint:** We need the *baseline*, not an average. An average of ON/OFF
cycles still causes the battery to continuously export the duty-cycle-weighted
power during OFF phases.

---

## Approach A — HP IIR + Frozen Baseline

### Algorithm

Two-stage design: **detect** oscillation, then **hold** a clean baseline.

#### Stage 1 — Detection (HP IIR + RMS Energy Envelope)

1. **High-pass IIR filter** removes DC, keeping only rapid changes:
   ```
   hp[n] = α × (hp[n-1] + x[n] - x[n-1])
   α = τ / (τ + dt)        # adapts to irregular sample spacing
   τ = 60 s
   ```

2. **RMS energy envelope** smooths the squared HP output:
   ```
   energy_sq[n] = β × hp² + (1 − β) × energy_sq[n-1]
   energy = √energy_sq
   β = 0.05
   ```

3. **Activation:** `energy > 300 W`
4. **Deactivation** (two independent paths):
   - *Calm detector:* last 60 s of raw samples have range ≤ 200 W **AND** |HP| < 400 W
   - *Energy decay:* `energy < 150 W` (300 × 0.5 hysteresis)

   On deactivation, HP and energy are reset to zero to prevent immediate
   re-activation from residual energy.

#### Stage 2 — Baseline (Frozen Pre-Disturbance EMA)

- A **calm EMA** (α = 0.05) continuously tracks the raw power while the filter
  is *inactive*. It is frozen (not updated) while active.
- A 120 s **ring buffer** stores `(timestamp, raw, calm_ema)` tuples.
- At *activation*, the baseline is retrieved from **30 s before** the current
  time via the ring buffer — before the initial spike contaminated the EMA.
- This frozen value is held as the filter output for the entire active period.

#### Parameters

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `HP_TAU_S` | 60 s | HP filter time constant |
| `ENERGY_ALPHA` | 0.05 | Energy envelope smoothing |
| `ENERGY_THRESHOLD_W` | 300 W | Activation threshold |
| `ENERGY_HYSTERESIS` | 0.5 | Deactivation at 50% of threshold |
| `HISTORY_S` | 120 s | Ring buffer length |
| `LOOKBACK_S` | 30 s | How far back to retrieve clean EMA |
| `BASELINE_ALPHA` | 0.05 | Calm EMA smoothing factor |
| `CALM_WINDOW_S` | 60 s | Window for calm range check |
| `CALM_BAND_W` | 200 W | Max range within calm window |
| `CALM_HP_MAX_W` | 400 W | HP magnitude guard for calm deactivation |

### Implementation Sketch

```
class PulseLoadFilter (AppDaemon hass.Hass app):
    ├── Config dataclass from apps.yaml
    ├── listen_state(raw_grid_sensor) → _on_grid_change()
    │
    ├── FilterState:
    │   ├── prev_raw, prev_t          # for HP difference
    │   ├── hp, energy_sq             # detection state
    │   ├── active: bool              # filter engaged
    │   ├── calm_baseline: float      # running EMA (frozen when active)
    │   ├── frozen_baseline: float    # held output value
    │   └── history: deque[(t, raw, calm_ema)]
    │
    ├── _update_hp_iir(raw, t)        # HP + energy envelope
    ├── _is_calm(t) → bool            # range check on raw history
    ├── _calm_ema_at(t) → float       # lookback into ring buffer
    │
    └── Output: set_state("sensor.zfi_filtered_grid_power", value)
         + debug sensors: active, baseline, energy, hp
```

Standalone app — controller reads `sensor.zfi_filtered_grid_power` instead of
the raw grid sensor. In dry-run mode, both exist on the dashboard for
comparison.

### Simulation Results (2026-04-21 full day, 1668 samples)

Data source: `sensor.smart_meter_sum_active_instantaneous_power` from
07:00–16:20 UTC, covering oven, washing machine, dryer, and coffee machine.

#### Event-by-Event Summary

| Time (UTC) | Appliance | Activations | Hold Value | True Baseline | Error | Duration |
|------------|-----------|-------------|------------|---------------|-------|----------|
| 07:13–07:17 | Unknown transient | 1 | −4 W | ~0 W | 4 W | 4 min |
| 07:21–07:24 | Unknown transient | 1 | −13 W | ~0 W | 13 W | 3 min |
| 08:21–08:22 | Washing machine ON spike | 1 | 2 W | ~500 W (step) | — | 75 s (false +) |
| 08:27–08:29 | Washing machine → battery overshoot | 1 | 490 W | ~490 W | 0 W | 2 min |
| 09:23–09:34 | Dryer (intermittent bursts) | 4 | 3→373→359→595 | varies | 0–240 W | ~100 s each |
| 10:51–10:58 | Chaotic load (dryer heating) | 1 | 75 W | ~75 W | 0 W | 7 min |
| 13:38–13:46 | Coffee machine | 2 | 72→39 W | ~40 W | 0–33 W | 8 min |
| 15:06–15:10 | Quick pulse burst | 1 | 104 W | ~50 W | 54 W | 4 min |
| **15:29–16:12** | **Oven session** | **5** | **34→17→46→42→31** | **~30 W** | **0–16 W** | **43 min** |

#### Key Findings

1. **Oven (primary target) — excellent.** Hold values within 0–16 W of true
   baseline through 43 min of continuous duty-cycling, including a phase where a
   second element activates (peaks at ±2200 W instead of ±1300 W). Multiple
   deactivation/reactivation cycles at inter-pulse gaps are clean.

2. **Initial oven ON-phase fix.** Without the HP magnitude guard, the calm
   detector falsely deactivated during the oven's initial steady-ON phase
   (~2600 W for 60 s) because the range was small. Adding `|HP| < 400 W` as a
   guard condition blocks this — the HP is ~600 at that point. No regression on
   other events (dryer at 09:24 deactivates normally with HP ≈ 301 < 400).

3. **Washing machine (step load) — benign false positive.** The initial ON
   spike (2157 W) triggers activation with hold(2), but the sustained ~500 W
   load is calm, so it deactivates in 75 s. Controller sees hold(2) for 75 s
   instead of 500 W — a temporary under-estimate, but self-corrects quickly.

4. **Dryer (intermittent bursts) — acceptable.** Four short activate/deactivate
   cycles due to burst-gap-burst pattern. Hold values lag by one cycle because
   the frozen baseline looks 30 s back. Self-correcting as calm EMA re-tracks
   between bursts.

5. **False activations are mostly harmless.** When the filter incorrectly
   activates on a non-periodic event, the hold value is close to the
   pre-disturbance baseline, which is close to the true level. Worst case is a
   ~50 W error for ~4 min (15:06 event).

6. **Deactivation is reliable.** The dual-path deactivation (calm detector +
   energy decay) with HP guard and energy reset provides clean transitions. No
   sustained false-hold or stuck-active scenarios in 9+ hours of data.

#### Known Limitations

- **Step loads cause brief false activation** (75 s). Could be mitigated with a
  minimum-active-duration or a check for oscillation period, but adds
  complexity.
- **Dryer cycling baseline lags by one burst** due to 30 s lookback. Acceptable
  because each burst is short (~100 s) and the error is bounded.
- **Single-sensor design** — only sees grid power, not individual appliance
  circuits. Cannot distinguish oven pulsing from other oscillation sources.

#### Simulation Script

`tools/simulate_filter.py` — reads CSV `(timestamp, grid_w)`, runs filter,
outputs per-sample table with: time, raw, filtered, HP, energy, active flag,
method. Supports `--compact` mode (only state-change samples).
