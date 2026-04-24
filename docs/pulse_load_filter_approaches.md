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

---

## Approach B — Sign-Flip Detector (detection only)

### Philosophy

Approach A detects the **cause** (oscillating load signal). Approach B detects
the **symptom** (the controller's response is futile — energy discharged by the
battery goes to the grid instead of the house).

When the controller chases a pulse load:
1. Oven ON → grid imports +1300 W → controller ramps battery
2. Oven OFF → house load drops, battery still outputting → grid exports −1300 W
3. The battery energy went straight to the grid — a futile round-trip

The tell-tale signature visible in the grid signal: rapid flips between large
positive (import) and large negative (export) values. If grid power never goes
negative, there is no loss — the house consumed everything.

### Algorithm

#### Sign-Flip Definition

A **sign flip** is detected when, within a `FLIP_WINDOW_S` window (20 s),
there exists:
- A sample with `grid > +FLIP_THRESH_W` (+500 W)
- AND a sample with `grid < -FLIP_THRESH_W` (−500 W)

This typically corresponds to two consecutive samples (one positive leg, one
negative leg) occurring 12–16 s apart during oven duty-cycling.

#### Activation

≥ `ACTIVATE_COUNT` (2) sign flips within a `ACTIVATE_WINDOW_S` (120 s) sliding
window → **activate**.

In practice: the first oven duty cycle produces flip #1, the second produces
flip #2. Activation after ~2 duty cycles ≈ 120 s of cycling.

#### Deactivation

No sign flips for `DEACTIVATE_S` (120 s) → **deactivate**.

#### Parameters

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `FLIP_THRESH_W` | 500 W | Minimum grid magnitude for each flip leg |
| `FLIP_WINDOW_S` | 20 s | Max time between positive and negative legs |
| `ACTIVATE_COUNT` | 2 | Sign flips needed to activate |
| `ACTIVATE_WINDOW_S` | 120 s | Sliding window for counting flips |
| `DEACTIVATE_S` | 120 s | No-flip duration to deactivate |

### What This Approach Does NOT Do

**No baseline calculation.** This is detection only — output is a boolean
`active` flag. Baseline determination is a separate concern to be combined
with this detector.

### Simulation Results

Tested against controller CSV data which includes both `grid_w` and
`battery_power_w` at 5 s resolution, with the controller actively running.

#### April 24, 2026 (oven, washing machine, dryer, coffee machine)

| Time (UTC) | Event | Flips | Active Duration | Notes |
|------------|-------|-------|-----------------|-------|
| 06:37 | Unknown pre-dawn | 2 | 2 min | Brief, harmless |
| 07:14–07:24 | Morning transient | 2 | 2 min | Brief |
| 09:30–09:33 | Dryer bursts | 1 each | — | **Not activated** — flips too spread |
| 10:51–10:58 | Chaotic load | 2 | 2 min | Brief |
| 13:43–13:47 | Coffee machine | 4 | 3 min | Correct |
| **15:35–15:48** | **Oven cycling** | 3 | **13 min** | Correct |
| **15:49–16:04** | **Oven cycling** | 3 | **15 min** | Re-activated after gap |
| **16:04–16:13** | **Oven cycling** | 2 | **9 min** | Re-activated after gap |

#### April 21, 2026 (oven session at 16:38–17:14)

| Time (UTC) | Event | Flips | Active Duration | Notes |
|------------|-------|-------|-----------------|-------|
| 08:27–08:31 | Washing machine overshoot | 4 | 3 min | Correct |
| 10:34–10:38 | Load transient | 5 | 4 min | Correct |
| 13:37–13:40 | Coffee machine | 3 | 2 min | Correct |
| **16:44–17:14** | **Oven session** | 5+/window | **~30 min** | Brief 4 s deactivation gap at 16:53 |

### Key Findings

1. **Oven detection — correct.** Both days correctly detect the oven duty-
   cycling phase. The initial oven ON-phase (steady ~2600 W ramp) is correctly
   **ignored** — no sign flips during the step response, so the controller
   handles it normally. Activation only triggers once cycling begins.

2. **No false activation on oven initial ON.** This was Approach A's main
   weakness — the frozen baseline was contaminated when the calm detector
   falsely deactivated during the steady ON phase. Approach B avoids this
   entirely because a step load without cycling produces no sign flips.

3. **Dryer bursts correctly ignored.** The 09:23–09:34 intermittent dryer
   bursts on April 24 never accumulate 2 flips within 120 s (bursts are ~3 min
   apart). Approach A falsely activated 4 times on this event.

4. **Deactivation gaps during oven cycling.** On April 24, the detector
   deactivates briefly at 15:48 and 16:04 when the oven has a slightly longer
   pause between duty cycles. The cycling signal naturally spans multiple
   activate/deactivate windows rather than one continuous active period. This is
   acceptable — during the gap, the controller briefly resumes normal operation,
   and re-activation occurs within 1–2 duty cycles.

5. **Only needs the grid sensor.** Although the battery output data was useful
   for understanding the problem, the detector itself only reads grid power.
   No additional sensor subscription required.

### Comparison with Approach A

| Aspect | Approach A (HP IIR) | Approach B (Sign-Flip) |
|--------|--------------------|-----------------------|
| Detects | Load oscillation (cause) | Futile response (symptom) |
| Sensors | Grid only | Grid only |
| Initial oven ON | False deactivation → 140 W error for 10 min (fixed with HP guard) | Correctly ignored — no sign flips |
| Dryer bursts | 4 false activations | Correctly ignored |
| Oven cycling | Continuous active | Brief deactivation gaps |
| Baseline | Built-in (frozen EMA) | **Not included** — separate concern |
| Complexity | Higher (HP IIR + energy + calm + HP guard + lookback) | Lower (window scan + counter) |
| Activation delay | ~1 sample (immediate on energy threshold) | ~2 duty cycles (~120 s) |
| Parameters | 10 | 5 |

### Known Limitations

- **Activation delay of ~2 duty cycles** (~120 s). The first two cycles of
  futile response are not caught. Approach A activates within one sample.
- **Deactivation gaps** during long oven sessions when inter-pulse spacing
  varies. Controller briefly resumes chasing before re-activation.
- **Detection only** — no baseline output. Must be combined with a separate
  baseline mechanism.

#### Simulation Script

`tools/simulate_signflip.py` — reads controller CSV
`(timestamp, grid_w, battery_power_w)`, runs sign-flip detector, outputs
per-sample table with: time, grid, battery, flip count, active flag, notes.
Supports `--compact` mode (only state-change and flip events).

---

## Baseline Estimation — Investigation Notes

### The Problem

The sign-flip detector (Approach B) produces an `active` flag but no baseline
value. If we want to output a stable power value for the controller, we need to
estimate the house's true baseline load (i.e. grid power *without* the pulsing
appliance).

### Approaches Evaluated

#### (max + min) / 2 of grid over a window

**Idea:** During symmetric oscillation, the midpoint equals the baseline.

**Result:** Works for single-element oven cycling on April 24 (mid ≈ 30–54 W,
true ≈ 30 W). Completely fails on April 21 when two oven elements create
asymmetric oscillation (mid ranges 253–2407 W). **Rejected.**

#### min(grid + battery) at export moments

**Idea:** At the export moment (oven OFF, battery high), `grid + battery ≈
house_load + inverter_loss`. The minimum such sum approximates the baseline.

**Result:**
- April 24: sum ≈ 107–130 W (true ≈ 30 W) → offset ~100 W
- April 21: sum ≈ 197–213 W (true ≈ 10 W) → offset ~200 W

Consistent overestimate due to inverter losses in the battery sensor.
Offset is not constant across days (100 vs 200 W). **Usable with calibration.**

#### Pre-disturbance EMA with step-back (selected approach)

**Idea:** Keep a slow EMA of grid power while the detector is inactive. Store a
long ring buffer (~10 min). At activation, walk backward through the buffer to
find the grid value *before* the initial load step (oven turn-on), which may
have occurred minutes before cycling started.

**Data evidence:**
- April 24: Grid was ~30 W at 15:29:24, oven ON at 15:29:59, steady ~2600 W
  for ~90 s, then cycling from 15:35. Sign-flip activation at 15:36:28.
  EMA from 15:29 = ~30 W ✓.
- April 21: Grid was ~10 W at 16:38:44, oven ON at 16:38:49, steady ~1540 W
  for 2 min, second element ON at 16:40:39 for another 2 min, cycling from
  16:42:44. Sign-flip activation at 16:45:14. EMA at activation is ~3000 W
  (contaminated). But ring buffer walk-back to 16:38:44 gives ~10 W ✓.
  Requires ~7 min of history.

**Step detection for walk-back:** The initial load turn-on produces a large
positive step in grid power (Δ > 1000 W in one sample). Walk backward through
the ring buffer until we find this step edge, then use the EMA from just before
it.

### Combined Baseline Strategy (to be implemented)

1. **Primary:** Pre-disturbance EMA via ring buffer walk-back past the initial
   ON step. Ring buffer length ~10 min (600 s).

2. **Refinement:** Once cycling produces export samples, compute `min(grid +
   battery)` over last 60 s. Compare with pre-disturbance EMA to estimate the
   inverter loss offset. This offset can be used to sanity-check the EMA or to
   refine the baseline during long active periods.

3. **Fallback:** If the ring buffer is too short to reach pre-disturbance (e.g.
   filter module just started and oven was already cycling), use `min(grid +
   battery) - estimated_offset`. Default offset TBD from data (~100–200 W).

### Status

Detection (sign-flip): **validated, ready for implementation.**
Baseline estimation: **design complete, not yet simulated.** Next step is to
add baseline computation to the sign-flip simulation and validate against
April 21 and April 24 data.
