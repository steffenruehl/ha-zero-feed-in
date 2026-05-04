# Pulse-Load Detector — `pulse_load_detector.py`

Detects periodic pulse loads (e.g. oven duty-cycling 15 s ON / 45 s OFF) that cause the controller to futilely chase oscillations. Detection is based on **sign-flip analysis** combined with an **energy-ratio criterion**: the grid rapidly alternates between large import and large export, and a significant fraction of battery discharge energy is wasted as grid export.

This module only *detects* — it does not mitigate. A separate mitigation module subscribes to the active entity and implements the chosen reaction strategy.

---

## How It Works

### Sign-Flip Detection

A **sign flip** occurs when, within `flip_window_s` (default 20 s), both a sample with `grid > +flip_thresh_w` and one with `grid < -flip_thresh_w` are observed. This means the grid swung from large export to large import (or vice versa) — the hallmark of a battery chasing a duty-cycling load.

### Energy-Ratio Detection

When a `battery_power_sensor` is configured, the detector also tracks the **energy balance** over a sliding window (`energy_window_s`). Two integrals are computed via the trapezoidal rule:

- **Grid export energy** (W·s) = ∫ max(0, −grid_w) dt — energy fed into the grid
- **Battery discharge energy** (W·s) = ∫ max(0, battery_w) dt — energy drawn from the battery

The **energy ratio** = grid_export / battery_discharge. A high ratio (≥ `energy_ratio_thresh`, default 0.3) means ≥30% of battery discharge is being wasted as grid export — the hallmark of futile controller chasing during pulse loads.

### Activation

Activation requires **both conditions** (AND logic):

1. `≥ activate_count` (default 2) sign flips within `activate_window_s` (default 120 s)
2. `energy_ratio ≥ energy_ratio_thresh` (default 0.3) within `energy_window_s` (default 120 s)

If no `battery_power_sensor` is configured, condition 2 is skipped — flip-only mode (backward compatible).

### Deactivation

When no `|grid| > flip_thresh_w` for `deactivate_quiet_s` (default 120 s), the detector deactivates. This uses absolute grid magnitude, not flip counting — small oscillations around the setpoint during filtered operation are normal and should not trigger deactivation.

---

## Published HA Sensors

| Entity | Type | Description |
| --- | --- | --- |
| `sensor.zfi_pld_active` | number | `1` = pulse load detected, `0` = normal |
| `sensor.zfi_pld_flip_count` | number | Flips in the activation window (debug) |
| `sensor.zfi_pld_energy_ratio` | number | Grid export / battery discharge ratio (debug) |

---

## CSV Logging

When `log_dir` is set, the detector writes a CSV file per day (`zfi_pld_YYYY-MM-DD.csv`) with columns:

| Column | Unit | Description |
| --- | --- | --- |
| `timestamp` | ISO 8601 | Auto-prepended by CsvLogger |
| `grid_w` | W | Raw grid power (positive = import) |
| `battery_w` | W | Battery power (+discharge / -charge) |
| `grid_export_ws` | W·s | Cumulative grid export in the energy window |
| `battery_discharge_ws` | W·s | Cumulative battery discharge in the energy window |
| `energy_ratio` | — | grid_export / battery_discharge (0–1+) |
| `flip_count` | — | Sign flips in the activation window |
| `active` | 0/1 | Detector state |

---

## Configuration Reference

```yaml
pulse_load_detector:
  module: pulse_load_detector
  class: PulseLoadDetector

  grid_power_sensor: sensor.YOUR_GRID_POWER_SENSOR   # MANDATORY

  # Sign-flip detection
  flip_thresh_w: 500        # min |grid| for a flip leg (W)
  flip_window_s: 20         # max time between flip legs (s)
  activate_count: 2         # flips needed to activate
  activate_window_s: 120    # sliding window for counting flips (s)
  deactivate_quiet_s: 120   # quiet duration before deactivating (s)

  # Energy-ratio detection (optional, recommended)
  battery_power_sensor: sensor.YOUR_BATTERY_SENSOR  # +discharge/-charge
  energy_window_s: 120      # sliding window for energy integration (s)
  energy_ratio_thresh: 0.3  # grid_export / battery_discharge threshold

  # Output
  sensor_prefix: sensor.zfi_pld
  debug: true
  log_dir: /config/logs     # CSV logging directory (empty = disabled)
```

See `apps.yaml.example` section 6 for the full template.

---

## Code Organization

```
DetectorConfig:     — typed config from apps.yaml
SignFlipDetector:    — pure logic, no HA dependency (sign flips + energy ratio)
PulseLoadDetector:  — AppDaemon adapter (listen_state, set_state, CSV logging)
```
