# Zero Feed-In — Development Context

## Project Summary

Two AppDaemon apps for a Zendure SolarFlow 2400 AC+ that implement
bidirectional zero feed-in control:

1. **Controller** (`zero_feed_in_controller.py`) — device-agnostic PI
   controller. Reads grid power, SOC, and battery power sensors.
   Publishes signed desired power to `sensor.zfi_desired_power`.
2. **Driver** (`zendure_solarflow_driver.py`) — Zendure-specific driver.
   Reads desired power and writes `outputLimit`, `inputLimit`, `acMode`.

The SolarFlow is connected to Home Assistant via local MQTT. The grid meter
is an EDL21 IR reader.

The owner (Steffen) has a Q.HOME+ ESS HYB-G2 as primary solar inverter
and the Zendure SolarFlow 2400 AC+ as secondary AC-coupled storage.
Home Assistant runs locally, AppDaemon is installed as HA addon.

## Hardware Setup

```
Existing PV → Q.HOME+ inverter → House grid ← Utility grid
                                      ↕ AC
                              SolarFlow 2400 AC+
                              (MQTT ↔ Home Assistant)
                                      ↕
                              EDL21 IR reader (grid meter)
```

## Architecture Decisions Made

### Why two apps (controller + driver)

- Controller handles PI control, mode switching, surplus estimation — reusable for any battery
- Driver handles Zendure-specific protocol: AC mode relay, power rounding, lockout timing
- Controller publishes `sensor.zfi_desired_power` (signed W, +discharge/-charge)
- Driver polls this sensor every 2 s and translates to device commands
- Separation allows swapping the driver for other batteries without touching control logic

### Why AppDaemon (not HA Blueprint)

- PI controller with state, mode machine, guards, and lockouts doesn't fit YAML templates
- AppDaemon gives real Python: dataclasses, enums, testable methods, proper logging

### Why not OpenDTU-OnBattery

- OpenDTU controls Hoymiles inverter limits — the SolarFlow 2400 AC+ has its own inverter
- No lever to pull: OpenDTU can't throttle the AC+ output

### Why two actuators (not one)

- `setOutputLimit`: discharge power (W), feeds into house
- `setInputLimit`: AC charge power (W), charges battery
- `acMode`: select entity, must be "Input mode" before charging, "Output mode" before discharging
- All MQTT entities, actuators are write-only from the driver's perspective
- `setDeviceAutomationInOutLimit` exists (single bidirectional entity) but availability depends on firmware

### Why sensors are read-only, actuators write-only

- Number entities show *commanded* value, not actual output
- PI uses its own state (`last_computed_w`), surplus uses `battery_power_sensor`
- Only external sensors needed by controller: `grid_power`, `soc`, `battery_power`

### AC Mode Management in Driver

- Zendure MQTT integration overwrites the AC mode entity with device reports faster than the physical relay switches (10-15 s)
- Cannot use entity state to suppress redundant sends — it always shows old mode
- Driver tracks its own intent (`last_set_relay`) and only re-sends after 30 s timeout
- Relay lockout uses driver's intent tracking, not entity state
- Power limits (outputLimit, inputLimit) are always sent when values change — no gating on device mode confirmation

## Key Concepts

### Solar Surplus Estimation

```
Energy balance: PV + battery_power + grid_power = house_load
Therefore: surplus = PV - house = -battery_power - grid_power
```

- `battery_power_sensor`: reports actual battery power with sign (+discharge/-charge)
- Controller reads sensor directly — no negation needed
- `surplus = -battery_power_w - grid_power_w`
- More accurate than old `last_sent_w` approach — reflects actual device state during 10-15 s lag

### Operating Mode (Schmitt Trigger with Charge Confirmation)

```
                    surplus > +hysteresis (50W), sustained for charge_confirm_s
  DISCHARGING ───────────────────────────────────▸ CHARGING
              ◂───────────────────────────────────
                    surplus < -hysteresis (50W), instant
```

- Entering CHARGING requires surplus to stay above threshold for charge_confirm_s (default 15-20 s)
- Entering DISCHARGING is instant — demand should be covered quickly
- On mode transition, integral and last_computed_w are reset to 0

### Asymmetric Targets

- DISCHARGING → target = +30 W (allow small grid draw as buffer)
- CHARGING → target = 0 W (absorb all surplus, never pull from grid)

### Surplus Clamp

```python
if raw < 0:  # wants to charge
    max_safe = max(0, surplus)
    clamped = max(raw, -max_charge, -max_safe)
```

### Grid-Charge Protection (layered)

1. **Mode gate**: surplus ≤ 0 → charging blocked (guard)
2. **Surplus clamp**: charge capped at available surplus (clamp)
3. **Asymmetric target**: target = 0 prevents PI from requesting grid power (PI)

## File Structure

```
config/appdaemon/apps/
├── zero_feed_in_controller.py    # device-agnostic PI controller
├── zendure_solarflow_driver.py   # Zendure SolarFlow driver
└── apps.yaml                     # configuration for both apps
```

### Controller Code Organization (zero_feed_in_controller.py)

```
Constants:  UNAVAILABLE_STATES, DEFAULT_SENSOR_PREFIX, EMERGENCY_SAFETY_MARGIN_W

Enums:      OperatingMode (CHARGING, DISCHARGING)

Dataclasses:
  Config          — typed config from apps.yaml
  SensorReading   — grid_power_w, soc_pct, battery_power_w
  ControlOutput   — desired_power_w, p/i terms, reason
  ControllerState — integral, last_computed_w, mode, charge_pending_since

PIController:     — asymmetric gains, anti-windup back-calculation

ZeroFeedInController(hass.Hass):
  initialize()                  — config, PI, seed, schedule
  _seed_state()                 — seed from battery_power_sensor
  _on_tick()                    — read → compute → update → log → publish
  _read_sensors()               — grid + soc + battery_power
  _estimate_solar_surplus()     — -battery_power_w - grid_power_w
  _update_operating_mode()      — Schmitt trigger with charge confirmation
  _compute()                    — emergency → mode → PI → guards → clamp
  _apply_guards()               — switches + SOC + grid-charge protection
  _clamp()                      — limit enforcement + surplus cap
  _publish_ha_sensors()         — publishes sensor.zfi_* entities
```

### Driver Code Organization (zendure_solarflow_driver.py)

```
Constants:  DIRECTION_THRESHOLD_W, ROUNDING_STEP_W, AC_MODE_*,
            AC_MODE_RETRY_S

Enums:      RelayDirection (CHARGE, IDLE, DISCHARGE)

Dataclasses:
  Config       — desired_power_sensor, device entities, lockout settings
  DriverState  — last_sent limits, relay tracking, lockout timer

ZendureSolarFlowDriver(hass.Hass):
  initialize()              — config, seed, schedule (2 s interval)
  _seed_from_device()       — read current limits + AC mode
  _on_tick()                — read desired → lockout check → round → send
  _is_relay_locked()        — lockout using own intent tracking
  _send_limits()            — AC mode + power limits
  _set_ac_mode()            — send-once with 30 s retry timeout
  _set_sensor()             — publish sensor.zfi_* driver states
```

## Published HA Sensors (sensor.zfi_*)

### Controller sensors

| Entity | Type | Description |
| --- | --- | --- |
| `zfi_desired_power` | W | Signed desired power (+discharge, -charge) |
| `zfi_mode` | text | Operating regime: charging/discharging |
| `zfi_surplus` | W | Estimated solar surplus |
| `zfi_battery_power` | W | Actual battery power (+discharge, -charge) |
| `zfi_target` | W | Active PI target (0 or 30) |
| `zfi_error` | W | Regulation error |
| `zfi_p_term` | W | Proportional component |
| `zfi_i_term` | W | Integral component |
| `zfi_integral` | W | Integral accumulator |
| `zfi_reason` | text | Decision reason |

### Driver sensors

| Entity | Type | Description |
| --- | --- | --- |
| `zfi_device_output` | W | Signed power sent to device |
| `zfi_discharge_limit` | W | outputLimit sent (≥ 0) |
| `zfi_charge_limit` | W | inputLimit sent (≥ 0) |
| `zfi_relay` | text | Physical relay state from AC mode entity |

## Current Status

- Two-app architecture: controller + Zendure driver
- Controller publishes `sensor.zfi_desired_power`, driver reads it
- PI controller uses position form with asymmetric gains and anti-windup back-calculation
- Battery sensor convention: +discharge/-charge (read directly, no negation)
- Surplus estimated from `battery_power_sensor`: `-battery_power_w - grid_power_w`
- Driver sends AC mode once on intent change, retries after 30 s
- Power limits always sent when values change (no mode gating)
- Relay lockout uses driver's own tracking (not HA entity, which MQTT overwrites)
- Redundant sends suppressed (only send when values change)
- Charge confirmation: surplus must hold for 20 s before CHARGING
- `set_state` uses `str(value)` and `replace=True` with try/except
- Device response latency: **10-15 seconds** (not 2-4 s as originally assumed)

### Entity names (current setup)

- Grid: `sensor.smart_meter_sum_active_instantaneous_power`
- SOC: `sensor.hec4nencn492140_electriclevel`
- Battery power: `sensor.energy_battery_hec4nencn492140_outputhomepower_hec4nencn492140_gridinputpower_net_power`
- Output limit: `number.hec4nencn492140_outputlimit`
- Input limit: `number.hec4nencn492140_inputlimit`
- AC mode: `select.hec4nencn492140_acmode`

## Known Issues / Not Yet Implemented

### Operational observations

- Device response time is 10-15 s — much slower than docs suggest
- Flash writes: whether `setOutputLimit`/`setInputLimit` write to flash is unknown
- `setDeviceAutomationInOutLimit` as flash-safe alternative — not yet tested

### Missing features

- **No persistence across restarts**: integral, mode reset. Startup seeding mitigates.
- **No unit tests**: PIController, compute pipeline, guards are testable pure functions
- **No time-of-use / dynamic tariff support**: could charge from grid during negative prices
- **Emergency only curtails discharge**: doesn't start charging to absorb existing PV surplus
- **Single-instance only**: no multi-device HEMS support

### Code quality improvements
- `_last_p` and `_last_i` are instance variables set as side effects of `_run_pi()` and read in `_apply_guards()` / `_build_output()`. Could be returned explicitly or stored in a cycle-scoped dataclass.
- `Config.from_args()` has no validation — invalid entity names or out-of-range values silently accepted.
- Log format could include a cycle counter for easier trace analysis.
- `_publish_ha_sensors()` calls `set_state` 12 times per cycle (every 5s) — could batch or reduce frequency.

## Tuning Guide

| Parameter | Default | Purpose |
|---|---|---|
| `kp_up` | 0.3 | Proportional gain for ramp-up (cautious). |
| `kp_down` | 0.8 | Proportional gain for ramp-down (aggressive). |
| `ki_up` | 0.03 | Integral gain for ramp-up (slow). |
| `ki_down` | 0.08 | Integral gain for ramp-down (fast). |
| `deadband` | 25W | No action within this error range. |
| `target_grid_power` | 30W | Discharge target. Small grid draw as safety buffer. |
| `charge_target_power` | 0W | Charge target. Absorb all surplus exactly. |
| `mode_hysteresis` | 50W | Surplus band for mode switching. Increase if mode flaps. |
| `charge_confirm` | 15s | Seconds surplus must hold before entering CHARGING. |
| `direction_lockout` | 5s | Min time between relay state changes. |
| `max_output` | 800W | Legal BKW limit (2400W with electrician sign-off). |
| `max_charge` | 2400W | AC charge limit. |
| `max_feed_in` | 800W | Emergency threshold. |
| `emergency_kp_multiplier` | 4.0 | How aggressively to curtail during emergency. |

### Asymmetric gain rationale
The gains are intentionally asymmetric:
- **Ramp up (kp_up=0.3, ki_up=0.03)**: Cautious. The device has 10-15s response latency.
  Ramping up too fast causes overshoot into feed-in, wasting battery energy.
- **Ramp down (kp_down=0.8, ki_down=0.08)**: Aggressive. When we're feeding in,
  cut power immediately. Battery energy is too precious to waste on the grid.

### Device latency
Measured at 10-15 seconds.  This is the time from sending a new
setOutputLimit/setInputLimit to seeing the effect on the grid meter.
Kp values above 1.0 risk oscillation with this latency.

## Testing Approach

1. **Dry run** (`dry_run: true`): controller computes, publishes sensors, logs — sends nothing. SOC can be faked via `input_number`. Grid sensor should be real.
2. **Manual output**: set SolarFlow output manually while dry run is active. Controller sees real physics and shows what it would do.
3. **Ramp up**: `dry_run: false`, start with `max_output: 200`, increase over days.
4. **Monitor**: `sensor.zfi_*` entities in HA history graphs.

## Related Projects / Prior Art

- **alkly.de Blueprint**: commercial HA blueprint for SolarFlow 2400 Pro zero feed-in. Inspiration for this project. Uses Jinja2 templates, not PI control.
- **solarflow-control** (GitHub: reinhard-brandstaedter): Python tool for DC-coupled SolarFlow hubs + OpenDTU. More mature, but targets Hub 2000/AIO with Hoymiles WR, not AC-coupled storage.
- **ioBroker zendure-solarflow adapter**: full-featured adapter with `setDeviceAutomationInOutLimit`, cloud relay, and PI controller script. The PI script for 2400 AC by forum user "schimi" was a reference for the PI approach.
- **z-master42/solarflow**: HA MQTT YAML config for SolarFlow entities. Useful reference for MQTT topic structure.
