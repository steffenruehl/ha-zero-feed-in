# Offline Tools

Three Python scripts form a pipeline for identifying the plant model,
optimizing controller gains, and visualising the results.  They run
**offline** on a workstation — never inside AppDaemon.

```
CSV logs  ──►  identify_plant.py  ──►  plant_model.json
                                            │
                                            ▼
                                      optimize_gains.py  ──►  optimization_result.json
                                            │                        │
                                            ▼                        ▼
                                      checkpoint.json          plot_response.py  ──►  PNG / display
```

All three tools live in `tools/` and import the production
`ControlLogic` from `src/zero_feed_in_controller.py` so the simulation
matches real behaviour exactly.

---

## identify_plant.py — Plant Model Identification

Reads controller CSV logs exported by `csv_logger.py`, fits a
first-order discrete plant model, and writes the result to JSON for
consumption by the optimizer.

### Model

```
batt[k] = a · batt[k-1] + b · desired[k-d] + c
```

The tool scans transport delays d = 0, 1, 2, 3 steps and picks the
delay that maximises R².  Only rows with a nominal 5 s tick
(`4 ≤ dt ≤ 6 s`) and active control (`|desired| > 10 W`) are used.
Output includes the time constant τ = −dt / ln(a), DC gain = b / (1 − a),
R², and the best-fit delay d in steps.

### CLI

| Flag | Type | Default | Description |
|------|------|---------|-------------|
| `csvs` (positional) | `Path` (one or more) | *required* | Controller CSV log files |
| `-o, --output` | `Path` | `data/plant_model.json` | Output JSON path |

### Example

```bash
python3 tools/identify_plant.py data/controller_2026-04-21.csv data/controller_2026-04-22.csv
```

### Output — `plant_model.json`

```json
{
  "plant": {
    "a": 0.8879, "b": 0.1015, "c": -2.38,
    "delay_steps": 1,
    "tau_s": 42.1, "dc_gain": 0.906, "r_squared": 0.977,
    "n_samples": 15607
  },
  "traces": {
    "timestamps": ["2026-04-21T10:30:45", "..."],
    "dt_s":            [5.0, 5.1, "..."],
    "disturbance_w":   [100.5, 102.3, "..."],
    "soc_pct":         [50.0, 50.1, "..."],
    "grid_actual_w":   [100.5, 102.3, "..."],
    "desired_actual_w": [150.0, 145.0, "..."],
    "battery_actual_w": [250.0, 245.0, "..."]
  }
}
```

---

## optimize_gains.py — Gain Optimization

Replays the recorded disturbance trace through `ControlLogic` in
closed loop to find gains that minimise feed-in energy.  Uses
`scipy.optimize.differential_evolution` with optional multiprocessing.

### Parameter Vector (13 elements)

| Index | Name | Bounds | Description |
|-------|------|--------|-------------|
| 0–3 | `kp_discharge_up/down`, `kp_charge_up/down` | 0.05 – 0.8 | Proportional gains (four quadrants) |
| 4–7 | `ki_discharge_up/down`, `ki_charge_up/down` | 0.005 – 0.3 | Integral gains |
| 8 | `deadband_w` | 5 – 60 W | Error deadband width |
| 9 | `ff_filter_tau_s` | 3 – 120 s | Feed-forward low-pass time constant |
| 10 | `ff_deadband_w` | 0 – 60 W | Feed-forward deadband |
| 11 | `ff_gain_pv` | 0.1 – 1.5 | PV feed-forward gain |
| 12 | `ff_gain_load` | 0.1 – 1.5 | Load feed-forward gain |

### Cost Function

```
cost = 100 · feed_in_energy_wh  +  0.001 · IAE  +  0.0001 · control_effort
```

Primary objective is minimising feed-in energy (grid < 0).  IAE and
control effort are secondary tie-breakers.

### CLI

| Flag | Type | Default | Description |
|------|------|---------|-------------|
| `-m, --model` | `Path` | `data/plant_model.json` | Plant model JSON |
| `--validate-only` | flag | `False` | Validate simulation vs logged data, then exit |
| `--maxiter` | `int` | `50` | Maximum generations |
| `--popsize` | `int` | `20` | Population size multiplier (total = popsize × 13) |
| `--workers` | `int` | `0` (auto) | Parallel workers; 0 = min(CPU, 8); 1 = serial |
| `--seed` | `int` | `42` | Random seed |
| `--resume` | `Path` | auto-detect | Resume from checkpoint (auto-detects default path) |
| `--checkpoint` | `Path` | `data/optimizer_checkpoint.json` | Checkpoint save path |
| `--result` | `Path` | `data/optimization_result.json` | Final result JSON path |

### Examples

```bash
# Full optimization from scratch
python3 tools/optimize_gains.py

# Validate only (no optimization)
python3 tools/optimize_gains.py --validate-only

# Resume a previous run with more generations
python3 tools/optimize_gains.py --resume --maxiter 100

# Custom population size and workers
python3 tools/optimize_gains.py --popsize 30 --workers 4
```

### Execution Flow

1. **Load model** — parse `plant_model.json`
2. **Validate** — simulate with logged gains, compare to real data
   (correlation must exceed 0.6)
3. **Baseline** — simulate with current production gains → baseline cost
4. **Optimize** — differential evolution with per-generation checkpoint
5. **Sensitivity analysis** — perturb each param ±5 % / ±20 %, rank by
   impact
6. **Save** — write checkpoint + result JSON; print `apps.yaml` snippet

### Checkpoint — `optimizer_checkpoint.json`

Saved every generation for resume:

```json
{
  "generation": 15,
  "best_cost": 8234.5,
  "baseline_cost": 9100.2,
  "baseline_feedin": 342.1,
  "best_x": [0.25, 0.35, "..."],
  "current_gains": [0.25, 0.35, "..."],
  "param_names": ["kp_discharge_up", "..."],
  "bounds": [[0.05, 0.8], "..."]
}
```

### Result — `optimization_result.json`

Canonical output consumed by `plot_response.py`:

```json
{
  "version": 1,
  "optimized_params": { "kp_discharge_up": 0.268, "..." : "..." },
  "current_params":   { "kp_discharge_up": 0.250, "..." : "..." },
  "param_vector":     [0.268, "..."],
  "current_vector":   [0.250, "..."],
  "bounds":           { "kp_discharge_up": [0.05, 0.8], "..." : "..." },
  "baseline_metrics": {
    "feed_in_energy_wh": 342.1,
    "max_feed_in_w": 850.0,
    "iae": 50000.0,
    "control_effort": 25000.0
  },
  "optimized_metrics": { "..." : "..." },
  "baseline_cost":    9100.2,
  "optimized_cost":   7430.1,
  "improvement_pct":  { "feed_in": -25.1, "cost": -18.3 },
  "plant":            { "a": 0.8879, "b": 0.1015, "c": -2.38,
                        "delay_steps": 1, "ident_dt_s": 5.0, "tau_s": 42.1,
                        "dc_gain": 0.906, "c_cont": -21.3 },
  "system_config":    { "max_discharge_w": 1200, "..." : "..." },
  "optimization":     { "generations": 47, "converged": true, "elapsed_s": 1245.3 }
}
```

### Fast vs Full Simulation

Two simulation paths exist:

| Path | Function | Speed | Used by |
|------|----------|-------|---------|
| **Fast** | `simulate_fast()` | ~5× faster | Optimizer (via `cost()`) |
| **Full** | `_simulate_full()` | 1× (baseline) | Validation against real data |

Both produce identical results — verified by 20 parity tests
(including variable-dt, jitter, and transport-delay scenarios) in
`tests/test_optimize_gains.py`.  The fast path inlines the PI loop
in pure numpy; the full path instantiates the real `ControlLogic` class.

### Jitter-Aware Plant Model

The discrete plant model `batt[k] = a·batt[k-1] + b·desired[k] + c`
was identified at a fixed 5 s sampling interval.  To handle timing
jitter (real intervals vary between 3–15 s), the simulator derives
continuous-time parameters from the discrete coefficients:

$$\tau = -\frac{dt_{\text{ident}}}{\ln a}, \quad K = \frac{b}{1-a}, \quad c_{\text{cont}} = \frac{c}{1-a}$$

Then for each step with actual elapsed time $dt$:

$$\alpha = e^{-dt/\tau}$$
$$a(dt) = \alpha, \quad b(dt) = K(1-\alpha), \quad c(dt) = c_{\text{cont}}(1-\alpha)$$

At the nominal dt=5 s, this reproduces the original coefficients
exactly.  With real jitter, the plant model correctly accounts for
how much the battery state evolves over longer or shorter intervals.

### Transport Delay

`identify_plant.py` scans transport delays d = 0 … 3 steps (each 5 s)
and picks the delay that maximises R².  The selected `delay_steps` is
stored in `plant_model.json` and flows through to the optimizer via
`SystemConfig.delay_steps`.

In simulation, a FIFO buffer defers the controller's desired output
by d steps before the plant sees it:

```
effective[k] = desired[k - d]
```

This models the real 10–15 s command-to-effect latency of the Zendure
device.  With d = 1 (5 s), the re-identified model improves from
R² = 0.976 to 0.977 with a faster time constant (τ = 42.1 s vs 52.6 s)
and higher DC gain (0.906 vs 0.888).

---

## plot_response.py — Response Visualisation

Simulates the closed-loop response on synthetic scenarios and real
data, comparing current vs optimised gains.  Reads the result JSON
produced by the optimizer.

### Scenarios

| Name | Description |
|------|-------------|
| `step_up` | Load step 100 W → 500 W |
| `step_down` | Load step 500 W → 100 W |
| `cloud` | PV surplus → deficit → recovery (cloud transient) |
| `charge_transition` | Ramp from discharge to charge mode |
| `oscillating` | Sinusoidal load with 5-min period (heat-pump cycling) |
| `low_soc` | Discharge demand with SOC near min guard (14 %) |
| `real` | Slice of real logged trace data |

### CLI

| Flag | Type | Default | Description |
|------|------|---------|-------------|
| `-m, --model` | `Path` | `data/plant_model.json` | Plant model JSON |
| `--result` | `Path` | `data/optimization_result.json` | Optimised gains result JSON |
| `--scenario` | `str` (repeatable) | `all` | Scenario(s) to plot |
| `--real-range` | `str` | `0:2000` | Index range for the `real` scenario |
| `--save` | `Path` | *None* (show) | Save PNGs to this directory |

### Examples

```bash
# Plot all scenarios (current vs optimised)
python3 tools/plot_response.py

# Plot only step responses
python3 tools/plot_response.py --scenario step_up --scenario step_down

# Real data, custom slice, save to disk
python3 tools/plot_response.py --scenario real --real-range 500:3500 --save figs/
```

### Output

Each scenario produces a three-panel figure:

1. **Grid power** — overlaid traces per gain set; red shading marks
   feed-in zone (grid < 0)
2. **Desired output** — controller setpoint
3. **Disturbance** — imposed load / PV input (grey)

If `--save` is given, figures are written as
`response_<scenario>.png`; otherwise `plt.show()` opens an
interactive window.

---

## Test Coverage

| Test file | Covers |
|-----------|--------|
| `tests/test_optimize_gains.py` | Parity (17), plant model (7), jitter parity (3), plant delay (3), delay parity (5), I/O round-trip (11), params_to_config (5), cost function (3), simulate dispatch (3), bounds (3) |
| `tests/test_plot_response.py` | Scenario shapes (12), trace conversion (2), real-data slicing (2), simulation on scenarios (8) |
