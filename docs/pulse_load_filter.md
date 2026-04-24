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

When the detector is inactive, the filter passes the raw grid value through transparently. When deactivated (falling edge), the estimator resets — a fresh measurement will run on the next activation.

---

## Integration with Controller

To close the loop, point the controller's `grid_power_sensor` at the filter's output:

```yaml
# Controller reads filtered output instead of raw sensor
zero_feed_in_controller:
  grid_power_sensor: sensor.zfi_plf_filtered_grid_power

# Filter reads raw sensor + detector active
pulse_load_filter:
  grid_power_sensor: sensor.smart_meter_sum_active_instantaneous_power
  active_entity: sensor.zfi_pld_active
  filtered_power_entity: sensor.zfi_plf_filtered_grid_power
```

---

## Published HA Sensors

| Entity | Type | Unit | Description |
| --- | --- | --- | --- |
| `filtered_power_entity` | number | W | Main output (configurable entity ID) |
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

  filtered_power_entity: sensor.zfi_plf_filtered_grid_power
  sensor_prefix: sensor.zfi_plf
  dry_run: true
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
