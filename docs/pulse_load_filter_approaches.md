# Pulse-Load Filter — Design

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

## Chosen Approach — Sign-Flip Detection + Measurement Pause + I-Controller

Three stages: **detect** futile cycling, **measure** baseline with battery
paused, **track** drift with an I-controller.

### Stage 1 — Detection (Sign-Flip Detector)

Detects the **symptom** of the problem: grid power rapidly flipping between
large import and large export. This means the controller is chasing oscillations
and the battery output is going to the grid instead of the house.

#### Sign-Flip Definition

A **sign flip** occurs when, within a 20 s window, the grid sensor reports both
`grid > +500 W` and `grid < −500 W`.

#### Activation

≥ 2 sign flips within a 120 s sliding window → **activate filter**.

#### Deactivation

No grid sample exceeding ±500 W for 120 s → **deactivate filter**.

Note: small sign flips around zero are expected during filtered operation (the
controller hunts around the setpoint). The deactivation condition ignores those
— it only checks that no *large* grid swings occur. This prevents false
deactivation when the filter is working correctly.

#### Parameters

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `FLIP_THRESH_W` | 500 W | Min magnitude for each flip leg |
| `FLIP_WINDOW_S` | 20 s | Max time between positive and negative legs |
| `ACTIVATE_COUNT` | 2 | Sign flips needed to activate |
| `ACTIVATE_WINDOW_S` | 120 s | Sliding window for counting flips |
| `DEACTIVATE_QUIET_S` | 120 s | No |grid| > threshold → deactivate |

#### Why Sign Flips?

- **No false activation on step loads.** A washing machine turning on (step to
  500 W) or the oven's initial steady-ON phase (~2600 W for 60+ s) produce no
  sign flips. The controller handles step loads fine — only cycling causes
  futile round-trips.
- **Dryer bursts correctly ignored.** Intermittent bursts 3 min apart don't
  accumulate 2 flips within 120 s.
- **Only needs the grid sensor.** No additional subscriptions required.

### Stage 2 — Baseline Measurement (Measurement Pause)

At activation, pause the battery for one oven cycle (~60 s) and measure the
true baseline directly.

#### Algorithm

1. Sign-flip detector fires → set `desired_power = 0` (pause battery)
2. Observe grid for 60 s (one full 15 s ON + 45 s OFF cycle)
3. `baseline = min(grid)` during the pause — this is the oven-OFF house load

#### Why This Works

With the battery at zero, the grid sensor reads exactly `house_load` during
oven-OFF and `house_load + oven_power` during oven-ON. The minimum over one
cycle is the true baseline — no inverter loss offset, no estimation, no
calibration.

#### Cost

~60 s of unfiltered operation at activation. But the sign-flip detector needs
~120 s (2 duty cycles) to confirm activation anyway, so the measurement pause
overlaps with the detection window. Net additional delay: 0 s.

#### Simulation Validation

Tested on recorded data where battery was naturally at 0 during oven start:

| Day | min(grid) with bat=0 | Pre-oven baseline | Status |
|-----|---------------------|-------------------|--------|
| Apr 24 | 158–165 W | ~148 W (grid alone ~15 W + bat ~24 W ≈ 39 W) | Measurement captures house load directly |
| Apr 21 | 363–365 W | ~10 W grid alone | During 2-element oven, higher baseline includes both elements' steady draw |

Note: April 24 min(grid) ≈ 165 W during measurement window reflects the actual
grid import including the house load that was running at that time (not just the
~30 W oven-less baseline). The measurement captures whatever the house is
consuming at that moment — which is exactly what the controller needs.

### Stage 3 — Drift Tracking (I-Controller)

After the measurement pause establishes the initial baseline, an I-controller
corrects for slow drift (appliances turning on/off during the oven session).

#### Algorithm

Every 60 s (one observation cycle):
```
min_grid = min(grid samples in last 60 s)
error    = min_grid − target
baseline += ki × error
```

#### Parameters

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `TARGET_W` | 30 W | Desired grid import during oven-OFF |
| `BASELINE_KI` | 0.3 | Correction gain per cycle |
| `CYCLE_WINDOW_S` | 60 s | Observation window |

#### Convergence Properties

Validated via synthetic plant model (`tools/simulate_baseline_convergence.py`):

| Seed Error | < 10 W | < 5 W | Final (20 cycles) |
|-----------|--------|-------|-------------------|
| +77 W | 7 min | 8 min | 30.1 W |
| +170 W | 9 min | 10 min | 30.1 W |
| +470 W | 10 min | 13 min | 30.4 W |

- **Monotonic from above** — no undershoot or oscillation
- **Exponential decay** — error × 0.7 per cycle
- **Bidirectional once running** — from a known-good starting point, the
  I-controller corrects both upward drift (new appliance) and downward drift
  (appliance off / solar ramp). The one-direction limitation only applies when
  `baseline < target` with `true_baseline ≤ target`, which can't happen when
  seeded from the measurement pause.

### Combined Timeline

```
t=0      Oven turns on (steady ~2600 W)
         Controller handles this fine — no sign flips yet
t≈90s    Oven starts cycling (15s ON / 45s OFF)
         Sign flips begin appearing
t≈210s   2nd sign flip detected → ACTIVATE
         Set battery = 0, start measurement pause
t≈270s   Measurement pause complete (60 s)
         baseline = min(grid) during pause
         Resume battery: desired_power = baseline − target
t≈330s+  I-controller tracks drift every 60 s
...
t=end    No |grid| > 500 W for 120 s → DEACTIVATE
         Resume passthrough of raw grid value
```

### Implementation Sketch

```
class PulseLoadFilter (AppDaemon hass.Hass app):
    ├── Config dataclass from apps.yaml
    ├── listen_state(raw_grid_sensor) → _on_grid_change()
    │
    ├── SignFlipDetector:
    │   ├── flip_times: deque       # recent flip timestamps
    │   ├── last_large_grid_t       # last |grid| > threshold
    │   └── active: bool
    │
    ├── BaselineMeasurement:
    │   ├── measuring: bool         # in measurement pause
    │   ├── pause_start: float      # when pause began
    │   ├── pause_samples: list     # grid samples during pause
    │   └── baseline: float         # measured value
    │
    ├── DriftTracker:
    │   ├── cycle_start: float      # current 60s window start
    │   ├── cycle_min_grid: float   # min grid in current window
    │   └── baseline: float         # I-controller output
    │
    └── Output: set_state("sensor.zfi_filtered_grid_power", value)
         + debug sensors: active, measuring, baseline, flip_count
```

Standalone app — controller reads `sensor.zfi_filtered_grid_power` instead of
the raw grid sensor. In dry-run mode, both exist on the dashboard for
comparison.

### Simulation Scripts

- `tools/simulate_signflip.py` — sign-flip detection on recorded data
- `tools/simulate_baseline_convergence.py` — I-controller convergence with
  synthetic plant model

---

## Discarded Alternatives

### A. HP IIR + Frozen Baseline

Two-stage detection using a high-pass IIR filter (τ=60 s) with RMS energy
envelope for activation, and a frozen pre-disturbance EMA for baseline.

**Why discarded:**
- 10 parameters vs 5 for sign-flip detection
- False activation on step loads (washing machine ON spike → 75 s false hold)
- False deactivation during oven initial ON-phase required HP magnitude guard
  (`|HP| < 400 W`) — added complexity
- 4 false activations on dryer bursts (sign-flip: 0)
- Frozen baseline requires 10-min ring buffer with step-edge walk-back for
  worst case (April 21: 7 min from oven ON to cycling start)

Simulation: `tools/simulate_filter.py`. Full results in git history.

### B. Baseline via min(grid + battery)

Estimate baseline from `min(grid + battery_sensor)` during export moments.

**Why discarded:**
- Consistent 100–200 W overestimate due to inverter losses in battery sensor
- Offset not constant across days (100 W on Apr 24, 200 W on Apr 21)
- Requires calibration or I-controller convergence from an inflated seed
- Measurement pause gives the exact baseline with no offset

### C. Baseline via (max + min) / 2

Midpoint of grid oscillation as baseline estimate.

**Why discarded:** Only works for symmetric oscillation. April 21 two-element
oven creates asymmetric peaks — midpoint ranges 253–2407 W (true: ~10 W).
Completely unusable.

### D. Pre-disturbance EMA with Ring Buffer Walk-Back

Keep slow EMA while inactive; at activation, walk backward through a 10-min
ring buffer past the initial ON step to find the pre-disturbance value.

**Why discarded:**
- Needs 7+ min of history for April 21 worst case (oven ON for 4 min before
  cycling starts)
- Step detection heuristic (Δ > 1000 W) is fragile
- If module starts while oven is already cycling, no pre-disturbance value
  exists — needs fallback
- Measurement pause is simpler and more reliable

### E. I-Controller from Option 3 Seed (without measurement pause)

Seed baseline from `min(grid + battery)` (overestimates by inverter loss), then
let I-controller converge downward.

**Why discarded:**
- I-controller can only correct from above — if seed is below true baseline
  (e.g. after subtracting a constant offset guess), it gets stuck permanently
- Convergence takes 8–13 min from typical seeds
- During convergence, excess export wastes energy
- Measurement pause gives exact value immediately, making the seed irrelevant
