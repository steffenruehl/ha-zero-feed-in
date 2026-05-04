# Pulse-Load Filter — `pulse_load_filter.py`

Baseline mitigation for periodic pulse loads. Subscribes to the detector's `active` entity and the raw grid sensor. When active, outputs a stable baseline instead of the oscillating grid signal.

---

## How It Works

### Stage 1: Measurement Pause

On activation (rising edge of `active` entity), the filter pauses the battery for `measurement_duration_s` (default 60 s). During this time, with battery output at 0, `min(grid)` gives the exact house-load baseline — no inverter-loss offset, no estimation.

### Stage 2: I-Controller Drift Tracking

After the initial measurement, the baseline is corrected every `drift_cycle_s` (default 60 s):

```
error = min(grid_in_cycle) - drift_target_w
baseline += drift_ki × error
```

This tracks slow changes in house load (appliance on/off, solar ramp) without needing another measurement pause. Starting from a known-good baseline (from measurement), the I-controller can correct bidirectionally.

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
3. Filter starts measurement pause → writes `desired_power = 0` (battery pauses)
4. After 60 s, baseline is measured → writes `desired_power = +baseline` (discharge)
5. Detector publishes `active = 0` → filter stops writing, controller resumes with reset state

---

## Published HA Sensors

| Entity | Type | Unit | Description |
| --- | --- | --- | --- |
| `filtered_power_entity` | number | W | Debug output — raw grid when inactive, desired power when active |
| `desired_power_entity` | number | W | Driver input — written when active and not dry_run |
| `sensor.zfi_plf_measuring` | number | — | `1` during measurement pause |
| `sensor.zfi_plf_baseline` | number | W | Current baseline estimate |

---

## Configuration Reference

```yaml
pulse_load_filter:
  module: pulse_load_filter
  class: PulseLoadFilter

  grid_power_sensor: sensor.YOUR_GRID_POWER_SENSOR    # MANDATORY
  active_entity: sensor.zfi_pld_active                 # MANDATORY

  measurement_duration_s: 60   # battery pause duration (s)
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
BaselineEstimator:      — measurement pause + I-controller drift (pure logic)
PulseLoadFilterLogic:   — stateful filter driven by external active signal
PulseLoadFilter:        — AppDaemon adapter (listen_state on grid + active entity)
```

---

## Design Notes

For the full exploration of alternative approaches (HP IIR filter, min(grid+bat), midpoint estimation, EMA walk-back, I-controller without pause), see [pulse_load_filter_approaches.md](pulse_load_filter_approaches.md).
