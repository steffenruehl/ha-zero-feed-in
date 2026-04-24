# Pulse-Load Detector — `pulse_load_detector.py`

Detects periodic pulse loads (e.g. oven duty-cycling 15 s ON / 45 s OFF) that cause the controller to futilely chase oscillations. Detection is based on **sign-flip analysis**: the grid rapidly alternates between large import and large export.

This module only *detects* — it does not mitigate. A separate mitigation module subscribes to the active entity and implements the chosen reaction strategy.

---

## How It Works

### Sign-Flip Detection

A **sign flip** occurs when, within `flip_window_s` (default 20 s), both a sample with `grid > +flip_thresh_w` and one with `grid < -flip_thresh_w` are observed. This means the grid swung from large export to large import (or vice versa) — the hallmark of a battery chasing a duty-cycling load.

### Activation

When `≥ activate_count` (default 2) sign flips are detected within `activate_window_s` (default 120 s), the detector activates and publishes `sensor.zfi_pld_active = 1`.

### Deactivation

When no `|grid| > flip_thresh_w` for `deactivate_quiet_s` (default 120 s), the detector deactivates. This uses absolute grid magnitude, not flip counting — small oscillations around the setpoint during filtered operation are normal and should not trigger deactivation.

---

## Published HA Sensors

| Entity | Type | Description |
| --- | --- | --- |
| `sensor.zfi_pld_active` | number | `1` = pulse load detected, `0` = normal |
| `sensor.zfi_pld_flip_count` | number | Flips in the activation window (debug) |

---

## Configuration Reference

```yaml
pulse_load_detector:
  module: pulse_load_detector
  class: PulseLoadDetector

  grid_power_sensor: sensor.YOUR_GRID_POWER_SENSOR   # MANDATORY

  flip_thresh_w: 500        # min |grid| for a flip leg (W)
  flip_window_s: 20         # max time between flip legs (s)
  activate_count: 2         # flips needed to activate
  activate_window_s: 120    # sliding window for counting flips (s)
  deactivate_quiet_s: 120   # quiet duration before deactivating (s)

  sensor_prefix: sensor.zfi_pld
  debug: true
```

See `apps.yaml.example` section 6 for the full template.

---

## Code Organization

```
DetectorConfig:     — typed config from apps.yaml
SignFlipDetector:    — pure logic, no HA dependency
PulseLoadDetector:  — AppDaemon adapter (listen_state, set_state)
```
