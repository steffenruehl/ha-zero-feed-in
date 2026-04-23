---
name: Gain optimization project
description: System ID + simulation + gain optimization pipeline for ZFI controller — current status and open tasks
type: project
---

## Goal
Use logged data to identify plant model, simulate closed-loop, optimize PI/deadband/FF gains.
Primary optimization target: minimize feed-in (grid < 0).

## What's done

### 1. Interval-independent controller (committed ec68113)
- `compute()` measures real dt between ticks, capped at 3× nominal
- Fixed bug: `_run_pi()` was still passing `cfg.interval_s` instead of measured `dt`
- FeedForward computes per-tick EMA alpha as `dt/(tau+dt)`
- 4 new tests added (107 total pass)

### 2. Plant identification script: `tools/identify_plant.py`
- Fits first-order discrete model: `batt[k] = a·batt[k-1] + b·desired[k] + c`
- Extracts disturbance trace: `disturbance = grid_w + battery_power_w`
- Outputs `data/plant_model.json` (plant params + traces for simulation)
- Identified model (Apr 21-22 data): a=0.9094, b=0.0804, c=-1.96, τ=52.6s, DC gain=0.888, R²=0.976

### 3. Optimization script: `tools/optimize_gains.py`
- 5-step pipeline: validate → baseline → optimize → results → sensitivity analysis
- Uses `scipy.optimize.differential_evolution` with multiprocessing (`--workers`)
- 13 parameters: 4×Kp, 4×Ki, deadband, ff_tau, ff_deadband, ff_gain_pv, ff_gain_load
- Cost function: 100×feed_in_Wh + 0.001×IAE + 0.0001×control_effort
- Progress tracking shows cost + feed-in improvement per generation
- Sensitivity analysis perturbs each param ±5%/±20%, ranks by cost impact

### 4. Validation results
- Simulation correlation r=0.948 vs real data (Apr 21-22)
- Feed-in estimate: sim=1110 Wh vs actual=1195 Wh (7% error)
- Baseline (current prod gains): feed-in=1111 Wh, cost=138,169

### 5. Partial optimization run (killed at gen 9 of 15)
- Already showed -40% feed-in (1111→664 Wh) and -32% cost
- Was still improving — needs a full run

## Data files (not committed — in `data/`)
- `data/controller_2026-04-21.csv`, `data/controller_2026-04-22.csv` — raw logs (consistent params)
- `data/plant_model.json` — identified model + traces (3.3 MB)
- `data/controller_all.csv`, `data/driver_all.csv` — full concatenated logs (Apr 19-23)

## Parameter history in logs
- Apr 19: old schema (no kp_used/ki_used columns) — unusable
- Apr 20: mixed params (ki changed mid-day) — unusable
- **Apr 21-22: consistent** — kp=[0.17,0.23,0.25,0.35], ki=[0.025,0.036]
- Apr 23: params changed again around 05:55 to current production values

## Current production gains (on server)
```
kp_discharge_up: 0.25    ki_discharge_up: 0.05
kp_discharge_down: 0.35  ki_discharge_down: 0.05
kp_charge_up: 0.17       ki_charge_up: 0.07
kp_charge_down: 0.23     ki_charge_down: 0.07
deadband: 35             deadband_leak_ws: 250
ff_filter_tau_s: 30      ff_deadband: 30
max_output: 1200         max_charge: 800
```

## Open tasks

1. **Run full optimization** — `python3 tools/optimize_gains.py --maxiter=50 --popsize=20 --workers=4`
   The partial run was very promising (-40% feed-in). Needs to finish + produce sensitivity report.

2. **Add `data/` and `tools/` to `.gitignore`** — data files shouldn't be committed.

3. **FF limitation**: The simulation doesn't have individual FF sensor traces (PV, loads), so FF
   gain optimization is based on the PI-only loop. In production, FF is active and interacts with
   the PI. Consider extracting per-sensor traces from HA history for more accurate FF optimization.

4. **Commit tools** — `tools/identify_plant.py` and `tools/optimize_gains.py` are not committed yet.

5. **Apply optimized gains** — Once optimization finishes, update `apps.yaml` on server and monitor.

## How to use (for another system)

```bash
# 1. Get controller logs from the other system (needs consistent params)
scp root@other-ha:/addon_configs/.../logs/zfi_controller_YYYY-MM-DD.csv data/

# 2. Identify that system's plant model
python3 tools/identify_plant.py data/controller_*.csv -o data/plant_model_other.json

# 3. Edit optimize_gains.py SystemConfig to match the other system's fixed params
#    (max_discharge_w, max_charge_w, min_soc, etc.)

# 4. Update logged_gains and current_gains arrays to match that system

# 5. Optimize
python3 tools/optimize_gains.py -m data/plant_model_other.json --workers=4
```
