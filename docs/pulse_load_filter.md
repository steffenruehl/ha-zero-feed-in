# Pulse-Load Filter — `pulse_load_filter.py`

Baseline mitigation for periodic pulse loads. Subscribes to the detector's `active` entity and the raw grid sensor. When active, outputs a stable baseline instead of the oscillating grid signal.

---

## How It Works

### Stage 1: Settle Delay

On activation (rising edge of `active` entity), the filter issues a battery-pause command (`desired_power = 0`). However, the inverter takes 10–15 seconds to actually ramp down. During this **settle phase** (`settle_duration_s`, default 15 s), all grid samples are discarded — they are corrupted by residual battery output and would produce a wildly incorrect baseline (e.g. −1200 W instead of +150 W).

### Stage 2: Measurement

After the settle delay, the filter collects grid samples for `measurement_duration_s` (default 60 s). With the battery truly silent, `min(grid)` gives the exact house-load baseline — no inverter-loss offset, no estimation. Total pause = settle + measurement = 75 s by default.

### Stage 3: Asymmetric Drift Tracking

After the initial measurement, the baseline is corrected asymmetrically:

**Downward (grid < 0 — export):** Immediate full correction. Negative grid means the battery is definitely overshooting — oven pulses only cause positive spikes (import), never export. The correction is `baseline += (grid_w - drift_target_w)` applied on the same sample.

**Upward (grid ≥ 0 — import):** Cautious cycle-based correction. Positive grid could be either a genuine increase in house load or an oven-ON spike. Uses `min(grid)` over `drift_cycle_s` (default 60 s) to filter out pulse spikes:

```
error = min(grid_in_cycle) - drift_target_w
baseline += drift_ki × error
```

This asymmetry ensures the filter responds within one sample (~15 s) to overshoot while remaining robust against oven-ON spikes that would falsely increase the baseline.

### Pass-Through

When the detector is inactive, the filter passes the raw grid value through to `filtered_power_entity` (for debug/dashboard). It does **not** write to `desired_power_entity` — the controller is in charge.

When deactivated (falling edge), the estimator resets — a fresh measurement will run on the next activation.

---

## Integration with Controller

The filter writes directly to `sensor.zfi_desired_power` when active, taking over from the controller. The controller must be configured with `pulse_load_active_entity` so it yields control:

```yaml
# Controller yields when pulse-load is active
zero_feed_in_controller:
  pulse_load_active_entity: sensor.zfi_pld_active

# Filter takes over desired_power when active
pulse_load_filter:
  grid_power_sensor: sensor.smart_meter_sum_active_instantaneous_power
  active_entity: sensor.zfi_pld_active
  desired_power_entity: sensor.zfi_desired_power
  dry_run: false
```

**Handover sequence:**
1. Detector publishes `active = 1`
2. Controller sees `active = 1` → skips computation
3. Filter starts settle + measurement pause → writes `desired_power = 0` (battery pauses)
4. After 15 s settle + 60 s measurement, baseline is measured → writes `desired_power = +baseline` (discharge)
5. Detector publishes `active = 0` → filter stops writing, controller resumes with reset state

---

## Published HA Sensors

| Entity | Type | Unit | Description |
| --- | --- | --- | --- |
| `filtered_power_entity` | number | W | Debug output — raw grid when inactive, desired power when active |
| `desired_power_entity` | number | W | Driver input — written when active and not dry_run |
| `sensor.zfi_plf_measuring` | number | — | `1` during settle + measurement pause |
| `sensor.zfi_plf_baseline` | number | W | Current baseline estimate |

---

## Configuration Reference

```yaml
pulse_load_filter:
  module: pulse_load_filter
  class: PulseLoadFilter

  grid_power_sensor: sensor.YOUR_GRID_POWER_SENSOR    # MANDATORY
  active_entity: sensor.zfi_pld_active                 # MANDATORY

  settle_duration_s: 15        # wait for inverter ramp-down before collecting (s)
  measurement_duration_s: 60   # baseline sample collection window after settle (s)
  drift_target_w: 30           # should match controller's discharge target (W)
  drift_ki: 0.3                # correction gain per cycle
  drift_cycle_s: 60            # observation window (s)

  filtered_power_entity: sensor.zfi_plf_filtered_grid_power   # debug output
  desired_power_entity: sensor.zfi_desired_power              # driver input
  sensor_prefix: sensor.zfi_plf
  dry_run: true                # set false to enable writing desired_power
  debug: true
```

See `apps.yaml.example` section 7 for the full template.

---

## Code Organization

```
FilterConfig:           — typed config from apps.yaml
BaselineEstimator:      — settle delay + measurement pause + I-controller drift (pure logic)
PulseLoadFilterLogic:   — stateful filter driven by external active signal
PulseLoadFilter:        — AppDaemon adapter (listen_state on grid + active entity)
```

---

## Design Notes

For the full exploration of alternative approaches (HP IIR filter, min(grid+bat), midpoint estimation, EMA walk-back, I-controller without pause), see [pulse_load_filter_approaches.md](pulse_load_filter_approaches.md).
