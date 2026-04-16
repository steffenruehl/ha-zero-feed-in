# Zero Feed-In Controller — Development Context

## Project Summary

AppDaemon app (`zero_feed_in.py`) for a Zendure SolarFlow 2400 AC+ that
implements bidirectional zero feed-in control via a PI controller. The
SolarFlow is connected to Home Assistant via local MQTT. The grid meter
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

### Why AppDaemon (not HA Blueprint)
- Started as a HA Blueprint (YAML + Jinja2) but complexity grew unmanageable
- PI controller with state, mode machine, guards, and lockouts doesn't fit YAML templates
- AppDaemon gives real Python: dataclasses, enums, testable methods, proper logging

### Why not OpenDTU-OnBattery
- OpenDTU-OnBattery controls Hoymiles inverter limits — the SolarFlow 2400 AC+ has its own integrated inverter
- No lever to pull: OpenDTU can't throttle the AC+ output
- Only useful for DC-coupled SolarFlow hubs (Hub 2000, AIO) with separate Hoymiles WR

### Why two actuators (not one)
- `setOutputLimit`: discharge power (W), feeds into house
- `setInputLimit`: AC charge power (W), charges battery
- `acMode`: select entity, must be "Input mode" before charging, "Output mode" before discharging
- All are MQTT entities, actuators are write-only from the controller's perspective
- AC mode is read from HA entity state to determine current relay direction
- There's also `setDeviceAutomationInOutLimit` (single bidirectional entity, negative = charge) but entity availability depends on firmware version — not yet tested

### Why sensors are read-only, actuators write-only
- Initially the code read `output_limit_entity` both for current state and to write commands
- Problem: the number entity shows the *commanded* value, not actual output
- Fix: PI uses `state.last_computed_w` (its own history), surplus uses `state.last_sent_w`
- Only two external sensors needed: `grid_power` and `soc`

## Key Concepts

### Solar Surplus Estimation
```
surplus = -last_sent_w - grid_power_w
```
- `last_sent_w`: what was actually sent to device (0 in dry run)
- `grid_power_w`: meter reading (+ draw, - feed-in)
- Example: charging 500W, grid +100W → surplus = 500 - 100 = 400W
- Example: idle, grid +200W → surplus = 0 - 200 = -200W (no solar)

This reconstructs PV production minus house load regardless of current SolarFlow activity. `grid_power > 0` does NOT mean "discharge needed" — it can mean "charging too much."

### Operating Mode (Schmitt Trigger with Charge Confirmation)
```
                    surplus > +hysteresis (50W), sustained for charge_confirm_s
  DISCHARGING ───────────────────────────────────▸ CHARGING
              ◂───────────────────────────────────
                    surplus < -hysteresis (50W), instant
```
- Entering CHARGING requires surplus to stay above threshold for charge_confirm_s (default 15s)
- Entering DISCHARGING is instant — demand should be covered quickly
- Prevents transient surplus spikes from causing expensive relay switches
- On mode transition, integral and last_computed_w are reset to 0
- Separate from relay direction (physical relay state with 5s time lockout)

### Asymmetric Targets
- DISCHARGING → target = +30W (allow small grid draw as buffer)
- CHARGING → target = 0W (absorb all surplus, never pull from grid)

Why: with target=30 during charging, PI would try to maintain +30W grid draw = pulling 30W from grid to charge. Target=0 prevents this.

### Surplus Clamp
```python
if raw < 0:  # wants to charge
    max_safe = max(0, surplus)
    clamped = max(raw, -max_charge, -max_safe)
```
Caps charge power at estimated surplus. Safety net against PI overshoot — the device never receives more than what solar provides.

### Dual State Tracking
- `last_computed_w`: what the PI calculated (used as PI base for velocity form)
- `last_sent_w`: what was actually sent to device (used for surplus estimation)
- In dry run: `last_computed_w` grows normally, `last_sent_w` stays 0
- This prevents phantom surplus values in dry run testing

### Grid-Charge Protection (layered)
1. **Mode gate**: surplus ≤ 0 → charging blocked (guard)
2. **Surplus clamp**: charge capped at available surplus (clamp)
3. **Asymmetric target**: target=0 prevents PI from requesting grid power (PI)

## File Structure

```
config/appdaemon/apps/
├── zero_feed_in.py    # main controller
└── apps.yaml          # configuration
```

### Code Organization (zero_feed_in.py)

```
Constants:  DIRECTION_THRESHOLD_W, ROUNDING_STEP_W, EMERGENCY_SAFETY_MARGIN_W
            AC_MODE_INPUT, AC_MODE_OUTPUT

Enums:      OperatingMode (CHARGING, DISCHARGING)
            RelayDirection (CHARGE, IDLE, DISCHARGE)

Dataclasses:
  Config          — typed config from apps.yaml, with defaults and from_args()
  SensorReading   — grid_power_w, soc_pct (read-only snapshot)
  ControlOutput   — discharge_limit_w, charge_limit_w, raw_limit_w, p/i terms, reason
  ControllerState — integral, last_computed_w, last_sent_w, mode, relay timestamp,
                    last_sent_discharge_w, last_sent_charge_w, charge_pending_since

PIController:     — asymmetric gains (kp_up/kp_down, ki_up/ki_down),
                    anti-windup back-calculation, no side effects

ZeroFeedIn(hass.Hass):
  initialize()                  — config, PI, seed from device, schedule
  _seed_state_from_device()     — read current output/input/ac_mode at startup
  _on_tick()                    — main loop: read → compute → update → log → send
  _read_sensors()               — reads grid + soc, logs which sensor is missing
  _estimate_solar_surplus()     — surplus from last_sent_w + grid
  _update_operating_mode()      — Schmitt trigger on surplus w/ charge confirmation
  _compute()                    — orchestrates: emergency → mode → PI → guards → lockout → clamp
  _check_emergency()            — curtail if feed-in > 800W
  _run_pi()                     — PI step (position form) with deadband freeze
  _apply_guards()               — SOC limits + grid-charge protection
  _check_relay_lockout()        — 5s direction change lockout (reads AC mode entity)
  _clamp_and_round()            — limit enforcement + surplus cap + 10W rounding
  _update_state()               — updates last_computed_w
  _send_limits()                — sends to device only if values changed
  _set_ac_mode()                — set AC mode + update relay timestamp
  _read_current_relay()         — derive relay direction from AC mode entity
  _publish_ha_sensors()         — publishes ~12 sensor.zfi_* entities
  _log_output()                 — structured log line
  _check_entities()             — startup validation of all entity names
```

## Published HA Sensors (sensor.zfi_*)

| Entity | Type | Description |
|---|---|---|
| `zfi_mode` | text | Operating regime: charging/discharging |
| `zfi_relay` | text | Physical relay: charge/idle/discharge |
| `zfi_surplus` | W | Estimated solar surplus |
| `zfi_device_output` | W | What was last sent to device (0 in dry run) |
| `zfi_target` | W | Active PI target (0 or 30) |
| `zfi_error` | W | Regulation error |
| `zfi_p_term` | W | Proportional component |
| `zfi_i_term` | W | Integral component |
| `zfi_integral` | W | Integral accumulator |
| `zfi_discharge_limit` | W | Discharge command |
| `zfi_charge_limit` | W | Charge command |
| `zfi_reason` | text | Decision reason |

## Current Status

- App loads and runs in AppDaemon (live operation, `dry_run: false`)
- PI controller uses **position form** with asymmetric gains and anti-windup back-calculation
- Asymmetric gains: cautious ramp-up (kp=0.3, ki=0.03), aggressive ramp-down (kp=0.8, ki=0.08)
- AC mode entity (`select.*_acmode`) integration: reads current state, only switches when needed
- Relay direction derived from HA entity state (no internal tracking that could drift)
- State seeded from device on startup to avoid unnecessary commands/relay clicks
- Redundant sends suppressed: output/input limits only sent when values change
- Charge confirmation: surplus must hold for 15s before switching to CHARGING mode
- Integral reset on mode transitions prevents cross-mode windup
- Integral frozen during relay lockout prevents windup during AC mode transition
- `set_state` uses `str(value)` and `replace=True` with try/except (fixes 400 errors)
- Grid sensor (`sensor.smart_meter_sum_active_instantaneous_power`) works
- SolarFlow entity names: `hec4nencn492140_*` (outputlimit, inputlimit, acmode, electriclevel)
- Device response latency measured at **10-15 seconds** (not 2-4s as originally assumed)

## Known Issues / Not Yet Implemented

### Operational observations
- Device response time is 10-15s — much slower than originally assumed (2-4s)
- Whether `setOutputLimit`/`setInputLimit` write to device flash (flash wear concern)
- `setDeviceAutomationInOutLimit` availability as flash-safe alternative

### Missing features
- **No PV production sensor**: surplus is estimated from grid + last_sent. A direct PV power sensor (from Q.HOME+ or SolarFlow MPPT) would be more accurate and faster.
- **No time-of-use / dynamic tariff support**: could charge from grid during negative electricity prices (Tibber, aWATTar). Currently grid charging is always blocked.
- **No charge-through / battery health**: solarflow-control has a `full_charge_interval` (force full charge every N hours for battery health). Not implemented here.
- **No persistence across AppDaemon restarts**: integral, mode, and last_sent_w reset to defaults on restart. Startup seeding from device largely mitigates this, but integral history is lost.
- **No unit tests**: `_compute()`, `_apply_guards()`, `_clamp_and_round()`, `PIController.update()` are pure-ish functions that could be tested with pytest and mock SensorReadings.
- **Emergency only curtails discharge**: if the *existing PV system* alone feeds >800W into the grid (nothing to do with SolarFlow), the emergency logic tries to reduce SolarFlow output — but SolarFlow might already be at 0. Could add logic to start *charging* to absorb the PV surplus.
- **Single-instance only**: no support for multiple SolarFlow units (HEMS mode with up to 6 devices).

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
