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
- On mode transition, integral is reset to 0

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
src/
├── zero_feed_in_controller.py    # device-agnostic PI controller + ControlLogic
├── zendure_solarflow_driver.py   # Zendure SolarFlow driver
└── csv_logger.py                 # shared daily-rotating CSV file logger
config/
└── apps.yaml                     # AppDaemon configuration for both apps
tests/
├── test_zero_feed_in_controller.py  # unit tests for ControlLogic & PIController
├── test_zendure_solarflow_driver.py  # unit tests for AdaptiveLockout & RelayStateMachine
└── test_csv_logger.py               # unit tests for CsvLogger
docs/
├── development_context.md        # this file
└── zero_feed_in_docs.md          # full technical documentation
```

### Controller Code Organization (src/zero_feed_in_controller.py)

```
Constants:  UNAVAILABLE_STATES, DEFAULT_SENSOR_PREFIX, EMERGENCY_SAFETY_MARGIN_W,
            CONTROLLER_CSV_COLUMNS

Enums:      OperatingMode (CHARGING, DISCHARGING)

Dataclasses:
  Config          — typed config from apps.yaml
  FeedForwardSource — entity, gain, sign, previous_w
  Measurement     — grid_power_w, soc_pct, battery_power_w, ff_readings, switch states, relay_locked
  ControlOutput   — desired_power_w, p/i/ff terms, reason
  ControllerState — integral, last_computed_w, mode, charge_pending_since

PIController:     — anti-windup back-calculation, gains supplied per call (PIGains)
  PIGains:          — Kp/Ki pair for one quadrant
  PIGainSet:        — four quadrant gain sets with select(mode, error)

FeedForward:      — multi-source feed-forward compensation (deadband on sum)
  entities          — list of entity IDs the HA adapter must read
  compute()         — sum(sign × gain × delta), deadband on total
  update_previous() — store current readings for next cycle

ControlLogic:     — pure-computation control logic (no HA dependency)
  seed()                        — initialise from battery_power_sensor
  estimate_surplus()            — -battery_power_w - grid_power_w
  compute()                     — emergency → mode → PI+FF → guards → clamp → relay-lock freeze
  target_for_mode()             — active grid-power target
  _update_operating_mode()      — Schmitt trigger with charge confirmation
  _apply_guards()               — switches + SOC + grid-charge protection (resets integral)
  _clamp()                      — limit enforcement + surplus cap
  _check_emergency()            — feed-in protection
  _run_pi()                     — PI computation (without committing state)

ZeroFeedInController(hass.Hass): — thin HA adapter
  initialize()                  — config, seed, CsvLogger, schedule
  _on_tick()                    — read → compute → log → publish → csv
  _read_measurement()           — assemble Measurement from HA sensors
  _publish_ha_sensors()         — publishes sensor.zfi_* entities
  _log_csv()                    — append row to CSV file (if log_dir set)
```

### Driver Code Organization (src/zendure_solarflow_driver.py)

```
Constants:  DIRECTION_THRESHOLD_W, ROUNDING_STEP_W, AC_MODE_*,
            AC_MODE_RETRY_S, RELAY_SAFETY_TIMEOUT_S, MIN_ACTIVE_POWER_W

Enums:      RelayDirection (CHARGE, IDLE, DISCHARGE)
            RelayState     (IDLE, CHARGING, DISCHARGING)

Dataclasses:
  AdaptiveLockout — energy integrator for relay lockout (pure computation)
  Config          — desired_power_sensor, device entities, lockout settings
  DriverState     — last_sent limits, relay tracking

RelayStateMachine:  — guards relay transitions (pure computation)
  seed()            — set initial state from device
  update()          — process desired power, return allowed power
  publish()         — push state and transition progress to HA sensors
  _classify()       — map desired_w to target RelayState
  _clamp_for_state()— constrain power to current state's direction

ZendureSolarFlowDriver(_HASS_BASE):
  initialize()              — config, seed, schedule (2 s interval)
  _seed_from_device()       — read current limits + AC mode
  _on_tick()                — read desired → SM update → round → send
  _send_limits()            — AC mode + power limits
  _set_ac_mode()            — send-once with 30 s retry timeout
  _set_sensor()             — publish sensor.zfi_* driver states
```

## Published HA Sensors (sensor.zfi_*)

Sensors marked *(debug)* are only published when `debug: true` in the respective app config.

### Controller sensors

| Entity | Type | Description |
| --- | --- | --- |
| `zfi_desired_power` | W | Signed desired power (+discharge, -charge) |
| `zfi_mode` | text | Operating regime: charging/discharging |
| `zfi_surplus` | W | Estimated solar surplus *(debug)* |
| `zfi_battery_power` | W | Actual battery power (+discharge, -charge) *(debug)* |
| `zfi_target` | W | Active PI target (0 or 30) *(debug)* |
| `zfi_error` | W | Regulation error *(debug)* |
| `zfi_p_term` | W | Proportional component *(debug)* |
| `zfi_i_term` | W | Integral component *(debug)* |
| `zfi_ff` | W | Feed-forward component *(debug)* |
| `zfi_integral` | W | Integral accumulator *(debug)* |
| `zfi_reason` | text | Decision reason *(debug)* |

### Driver sensors

| Entity | Type | Description |
| --- | --- | --- |
| `zfi_device_output` | W | Signed power sent to device |
| `zfi_discharge_limit` | W | outputLimit sent (≥ 0) |
| `zfi_charge_limit` | W | inputLimit sent (≥ 0) |
| `zfi_relay` | text | Physical relay state from AC mode entity |
| `zfi_relay_locked` | text | `true` when SM is clamping output or relay is physically switching (8 s holdoff after SM transition) |
| `zfi_relay_sm_state` | text | Current SM state (idle/charging/discharging) *(debug)* |
| `zfi_relay_sm_pending` | text | Pending transition target (or "none") *(debug)* |
| `zfi_relay_sm_lockout_pct` | % | Unified lockout progress for active transition *(debug)* |
| `zfi_relay_sm_accumulated_ws` | W·s | Accumulated energy toward transition threshold *(debug)* |
| `zfi_relay_sm_threshold_ws` | W·s | Energy threshold required for transition *(debug)* |
| `zfi_relay_sm_charge_pct` | % | Charge transition progress *(debug)* |
| `zfi_relay_sm_discharge_pct` | % | Discharge transition progress *(debug)* |
| `zfi_relay_sm_idle_pct` | % | Idle transition progress *(debug)* |

## Current Status

- Two-app architecture: controller + Zendure driver
- Controller publishes `sensor.zfi_desired_power`, driver reads it
- PI controller uses position form with anti-windup back-calculation
- Multi-source feed-forward: compensates PV and load changes before they appear at the grid meter
- Battery sensor convention: +discharge/-charge (read directly, no negation)
- Surplus estimated from `battery_power_sensor`: `-battery_power_w - grid_power_w`
- Driver sends AC mode once on intent change, retries after 30 s
- Power limits always sent when values change (no mode gating)
- Relay lockout uses driver's own tracking (not HA entity, which MQTT overwrites)
- Driver publishes `sensor.zfi_relay_locked` (always, not debug-only) so the controller can freeze the integral during relay lockout
- `relay_locked` stays `true` for 8 seconds after each SM transition (`RELAY_SWITCH_DELAY_S`) to account for physical relay switching time
- Redundant sends suppressed (only send when values change)
- Charge confirmation: surplus must hold for `charge_confirm_s` (default 15 s, apps.yaml 20 s) before CHARGING
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
  → **Resolved**: 88 tests (32 controller + 56 driver) covering ControlLogic, PIController, AdaptiveLockout, RelayStateMachine
- **No time-of-use / dynamic tariff support**: could charge from grid during negative prices
- **Emergency only curtails discharge**: doesn't start charging to absorb existing PV surplus
- **Single-instance only**: no multi-device HEMS support

### Code quality improvements
- `_last_p` and `_last_i` are instance variables set as side effects of `compute()` and read in `_apply_guards()`. Could be returned explicitly or stored in a cycle-scoped dataclass.
- `Config.from_args()` has no validation — invalid entity names or out-of-range values silently accepted.
- Log format could include a cycle counter for easier trace analysis.
- `_publish_ha_sensors()` calls `set_state` 2 times per cycle (10 times when `debug: true`) — could batch or reduce frequency.

## Tuning Guide

| Parameter | Default | Purpose |
|---|---|---|
| `kp_discharge_up` | 0.50 | Proportional gain: increase discharge. |
| `kp_discharge_down` | 0.71 | Proportional gain: decrease discharge. |
| `kp_charge_up` | 0.33 | Proportional gain: increase charge. |
| `kp_charge_down` | 0.45 | Proportional gain: decrease charge. |
| `ki_discharge_up` | 0.025 | Integral gain: increase discharge. |
| `ki_discharge_down` | 0.051 | Integral gain: decrease discharge. |
| `ki_charge_up` | 0.011 | Integral gain: increase charge. |
| `ki_charge_down` | 0.021 | Integral gain: decrease charge. |
| `deadband` | 25W | No action within this error range. |
| `feed_forward_sources` | (list) | Feed-forward sources: entity, gain, sign. |
| `ff_enabled` | true | Master switch to enable/disable feed-forward. |
| `ff_deadband` | 20W | Ignore total FF correction below this threshold. |
| `target_grid_power` | 30W | Discharge target. Small grid draw as safety buffer. |
| `charge_target_power` | 0W | Charge target. Absorb all surplus exactly. |
| `mode_hysteresis` | 50W | Surplus band for mode switching. Increase if mode flaps. |
| `charge_confirm` | 15s | Seconds surplus must hold before entering CHARGING. |
| `relay_locked_sensor` | (none) | HA entity for relay lockout feedback from driver. |
| `direction_lockout` | 5s | Min time between relay state changes. |
| `max_output` | 800W | Legal BKW limit (2400W with electrician sign-off). |
| `max_charge` | 2400W | AC charge limit. |
| `max_feed_in` | 800W | Emergency threshold. |
| `emergency_kp_multiplier` | 4.0 | Loaded but currently unused in emergency logic. |

### Gain tuning
Gains are derived from step response measurements per quadrant using SIMC rules.
See the gain table above and the selection matrix in the main docs.
Kp values above 1.0 risk oscillation with the device's 10-15s latency.

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
