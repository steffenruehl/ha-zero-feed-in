# Zero Feed-In Controller — Development Context

## Project Summary

Two-app AppDaemon system for a Zendure SolarFlow 2400 AC+ that keeps
the grid meter at ~0 W. Device-agnostic controller publishes a signed
desired-power value; a separate device driver translates it into
hardware commands.

## Architecture

```
┌──────────────────────────────────────────────────────────┐
│  zero_feed_in_controller.py                              │
│                                                          │
│  Sensors (read):          ControlLogic (pure Python):    │
│    grid_power_sensor        PI controller                │
│    soc_sensor                 asymmetric Kp/Ki           │
│    battery_power_sensor       back-calculation windup    │
│                             Schmitt trigger (mode)       │
│                             Guards (SOC, surplus, switch)│
│                             Surplus clamp                │
│                                                          │
│  Output: sensor.zfi_desired_power (+discharge/−charge)   │
│          sensor.zfi_mode, sensor.zfi_* (debug)           │
└────────────────────────┬─────────────────────────────────┘
                         │  HA sensor entity (coupling point)
                         ▼
┌──────────────────────────────────────────────────────────┐
│  zendure_solarflow_driver.py                             │
│                                                          │
│  Input: sensor.zfi_desired_power                         │
│                                                          │
│  RelayStateMachine:         Device commands:             │
│    3 states (IDLE/CHG/DIS)    outputLimit (number)       │
│    adaptive energy lockout    inputLimit (number)        │
│    safety timeout             acMode (select)            │
│    per-direction accumulators                            │
│                                                          │
│  Output: actual device commands + sensor.zfi_relay etc.  │
└──────────────────────────────────────────────────────────┘
```

### Why Two Apps

- **Controller** is device-agnostic: knows nothing about Zendure, MQTT
  topics, relay modes, or command entities. Could drive any storage.
- **Driver** is device-specific: handles AC mode switching, relay dead
  time, command deduplication, power rounding. Swappable for other
  hardware without touching the controller.
- **Coupling** via a single HA sensor entity: `sensor.zfi_desired_power`.
  Controller publishes, driver polls. No direct dependency.

### Key Data Flow

```
Sensors → Measurement → ControlLogic.compute() → ControlOutput
  │                                                    │
  │                                          desired_power_w
  │                                                    │
  │                                          published to HA
  │                                                    │
  │                              Driver reads sensor.zfi_desired_power
  │                                                    │
  │                              RelayStateMachine.update(desired)
  │                                                    │
  │                              → allowed_power (may differ from desired)
  │                                                    │
  │                              → outputLimit / inputLimit / acMode
```

## Controller Internals (zero_feed_in_controller.py)

### Class Separation

- `Config` — typed dataclass from apps.yaml
- `Measurement` — immutable sensor snapshot (grid, SOC, battery_power, switches)
- `ControlOutput` — desired_power + PI terms + reason
- `ControllerState` — integral, last_computed_w, mode, charge_pending_since
- `PIController` — pure math, asymmetric gains, back-calculation anti-windup
- `ControlLogic` — all decisions, no HA imports, testable with plain Measurement objects
- `ZeroFeedInController(hass.Hass)` — thin HA adapter (read sensors, publish, schedule)

### PI Controller

Velocity form with asymmetric gains:
```
error >= 0 (need more power):  Kp_up, Ki_up   (slower ramp up)
error <  0 (need less power):  Kp_down, Ki_down (faster ramp down)
```

Anti-windup via back-calculation:
```
output = p_term + integral
if output > max:
    integral = max - p_term    ← back-calculate
    output = max
```

### Surplus Estimation

Uses actual battery power sensor (not reconstructed from last_sent):
```
surplus = -battery_power_w - grid_power_w
```
Sign: battery_power is +discharge/-charge.

### Operating Mode (Schmitt Trigger)

Asymmetric transitions:
- DISCHARGING → CHARGING: surplus > hysteresis for `charge_confirm_s` consecutive seconds
- CHARGING → DISCHARGING: instant when surplus < -hysteresis

Integral resets to 0 on every mode transition.

### Guards

Evaluated before integral commit (PI freezes while guard active):
1. Discharge disabled (switch off)
2. Charge disabled (switch off)
3. SOC too low (discharge blocked)
4. No surplus (charge blocked)
5. SOC full (charge blocked)

### Surplus Clamp

After guards, charge power capped at available surplus:
```python
if raw < 0:
    max_safe = max(0, surplus)
    clamped = max(raw, -max_charge, -max_safe)
```

## Driver Internals (zendure_solarflow_driver.py)

### Relay State Machine

Three states: IDLE, CHARGING, DISCHARGING.

Transition uses **adaptive energy integrator** (not fixed time):
```
accumulated += |desired_power| × dt
threshold = ref_power × base_lockout_s

transition when: accumulated >= threshold
```

Effect: high power → fast transition, low power → slow transition.
Prevents chattering at low power where relay switching cost is
disproportionate.

Per-direction accumulators are independent — oscillating desired power
between charge and discharge makes progress toward both transitions
simultaneously. Only reset when stable in current state or on actual
transition.

Safety timeout: never block longer than 300s regardless of accumulator.

### AC Mode Management

Driver tracks its own intent (`last_set_relay`) separately from the
HA entity state, because MQTT overwrites the entity faster than the
physical relay switches (10-15s). Retry after 30s if device hasn't
confirmed.

### Command Deduplication

outputLimit and inputLimit only sent when value differs from last-sent.

## Current apps.yaml Parameters

### Controller
```
kp_up: 0.4    kp_down: 0.45
ki_up: 0.04   ki_down: 0.05
deadband: 35W
discharge target: 30W, charge target: 0W
mode_hysteresis: 50W, charge_confirm: 20s
max_output: 1200W, max_charge: 800W
min_soc: 10%, max_soc: 85%
```

### Driver
```
interval: 2s (faster than controller's 5s)
direction_lockout: 30s (base for adaptive integrator)
adaptive_lockout_ref_w: 200W
relay_sm_enabled: true
```

## Published HA Sensors

### Always (controller)
| Entity | Description |
|---|---|
| `zfi_desired_power` | Signed setpoint (W), main coupling to driver |
| `zfi_mode` | Operating regime: charging/discharging |

### Debug (controller, when debug=true)
| Entity | Description |
|---|---|
| `zfi_surplus` | Estimated solar surplus (W) |
| `zfi_battery_power` | Signed battery power (W) |
| `zfi_p_term` | Proportional component (W) |
| `zfi_i_term` | Integral component (W) |
| `zfi_integral` | Integral accumulator (W) |
| `zfi_target` | Active PI target (W) |
| `zfi_error` | Regulation error (W) |
| `zfi_reason` | Decision reason (text) |

### Always (driver)
| Entity | Description |
|---|---|
| `zfi_device_output` | What was sent to device (signed W) |
| `zfi_discharge_limit` | outputLimit sent (W) |
| `zfi_charge_limit` | inputLimit sent (W) |
| `zfi_relay` | Device AC mode reading |

### Debug (driver, when debug=true)
| Entity | Description |
|---|---|
| `zfi_relay_sm_state` | SM state: idle/charging/discharging |
| `zfi_relay_sm_pending` | Pending transition target |
| `zfi_relay_sm_lockout_pct` | Transition progress (%) |
| `zfi_relay_sm_accumulated_ws` | Energy accumulated (W·s) |
| `zfi_relay_sm_threshold_ws` | Energy threshold (W·s) |
| `zfi_relay_sm_charge_pct` | Charge transition progress (%) |
| `zfi_relay_sm_discharge_pct` | Discharge transition progress (%) |
| `zfi_relay_sm_idle_pct` | Idle transition progress (%) |

## Known Issues / Not Yet Implemented

### Must verify
- Flash wear from frequent outputLimit/inputLimit writes
- Whether setDeviceAutomationInOutLimit is available as alternative
- Actual relay dead time and ramp time constants for Kp/Ki tuning

### Missing features
- **D-term (PID)**: react to grid power rate of change for faster load step response
- **PV feed-forward**: react to PV power changes before they appear at grid
- **Slew rate limiter**: match commands to device ramp capability, prevent integral windup during large steps
- **Persistence**: integral, mode, last_computed reset on restart
- **Unit tests**: ControlLogic is testable but no tests written yet
- **Multi-device**: no support for multiple SolarFlow units
- **Dynamic tariffs**: no grid charging during negative prices

### Code quality
- `_last_p` / `_last_i` as side-effect instance vars — could be in a cycle-scoped context
- No Config validation (invalid entity names silently accepted)
- Driver and controller share `sensor_prefix` — could collide if misconfigured

## Related Projects
- **solarflow-control** (GitHub): Python, targets DC-coupled hubs
- **ioBroker zendure-solarflow**: full adapter with PI script for 2400 AC
- **alkly.de**: commercial HA blueprint for SolarFlow zero feed-in
