# Zero Feed-In Controller — Improvement Roadmap

## Repository

https://github.com/steffenruehl/ha-zero-feed-in

## Architecture

```
Forecast.Solar ──▸ HA Automation ──▸ input_number.zfi_min_soc
                                              │
Grid Sensor ───────▸                          │
SOC Sensor ─────────▸ Controller (PI) ────────▸ sensor.zfi_desired_power
Battery Power ──────▸   (ControlLogic)                │
Charge/Discharge SW ▸                                 ▼
relay_locked ◂──────────────────────────── Driver (Relay SM)
                                                      │
                                                      ▼
                                              SolarFlow 2400 AC+
                                                      │
                                              MQTT Watchdog ──▸ reconnect via HTTP
```

Three AppDaemon apps:
- **Controller** (`zero_feed_in_controller.py`): device-agnostic PI,
  publishes `sensor.zfi_desired_power` (+discharge, −charge).
  `ControlLogic` class is pure Python, no HA imports, fully testable.
- **Driver** (`zendure_solarflow_driver.py`): translates desired power
  into outputLimit/inputLimit/acMode via relay state machine.
  Publishes `sensor.zfi_relay_locked` back to controller.
- **Watchdog** (`solarflow_mqtt_watchdog.py`): triggers MQTT reconnect
  via local HTTP API on HA startup or when device goes unavailable.

## Current State (working, April 2026)

- Bidirectional PI with four-quadrant Kp/Ki (per mode × error-sign)
- Back-calculation anti-windup
- Deadband with leak correction (persistent bias fix)
- Surplus estimation from battery_power sensor
- Schmitt trigger with charge-confirmation delay
- Relay state machine with adaptive energy-integrator lockout
- `relay_locked` feedback: driver signals controller to freeze integral
- AC mode management with retry logic
- Charge/discharge switches (input_boolean)
- Signed + unsigned battery sensor support
- 12+ debug sensors published to HA
- Startup seeding from device state (controller + driver)
- Forecast-based dynamic min_soc (reads `input_number.zfi_min_soc`)
- Feed-forward: EMA-filtered multi-source (PV + arbitrary loads), configurable per source
- MQTT reconnect watchdog (HA startup + entity-stale triggers)
- State persistence: controller + driver save/restore state across restarts (JSON)
- Relay switch counter (persists to JSON, publishes `sensor.zfi_relay_switches_today`)
- smartMode flash wear protection (RAM-only device writes, auto-reenabled after reboot)
- Driver stale-check: sends safe state (0 W) when controller hasn't updated in 30 s
- MQTT heartbeat publishing: controller + driver publish ISO-8601 timestamps for ESP monitoring
- Watchdog heartbeat monitoring: checks entity last_updated, HA persistent notifications on stale
- 263 unit tests (103 controller, 127 driver, 17 PV forecast, 8 CSV logger, 8 watchdog) — all passing
- CSV file logging (controller + driver)
- Lovelace dashboard

## Attempted and Abandoned

### D-Term (PID on grid signal)
**Problem**: Grid sensor noise. D amplifies measurement noise into
output jitter → oscillations. The EDL21 IR reader has ±10-20W jitter;
Kd×20W fires every cycle. Deadband helps but defeats the purpose for
small load steps.

### PV Feed-Forward
**Status**: Re-implemented with EMA filter (`ff_filter_tau_s=30s`) to
attenuate MPPT ripple. Infrastructure is in code (`FeedForward` class,
`feed_forward_sources` config). Whether it actually helps with the
SolarFlow's ±30-50W PV noise at 30s tau is unverified in production.
If it causes problems, set `ff_enabled: false` in apps.yaml.

### Slew Rate Limiter
**Not needed**: Step response measurements show the SolarFlow has a
fixed processing delay (~3-7s), not a slew rate limit. A 200W step
and a 1400W step take similar time. No benefit from rate limiting.

## Measured Device Timing (April 2026)

Relay closed, step response settle times:

| Quadrant | Physical action | T1 (s) |
|---|---|---|
| discharge_up | increase discharge | 5.0 |
| discharge_down | decrease discharge | 3.5 |
| charge_up | increase charge | 7.5 |
| charge_down | decrease charge | 5.5 |

Relay switching:
- Charge relay on: 8s, off: 1s
- Discharge relay on: 3-6s, off: 4s

Key finding: no slew rate behavior. Settle time does not scale with
step size. The device has a processing delay, not a ramp limit.

## Improvements — Prioritized

### 1. Reduce Relay Switching — TUNED

**Problem**: Initially 50 switches/day observed. Electrical relay life:
~100k cycles at full load. Target: under 20/day.

**Analysis (April 2026, 4-day CSV logs):** Baseline was ~30 sw/day
(not 50 as originally estimated). Switches cluster at dawn/dusk
transitions. Simulated approaches (higher thresholds, longer idle,
trajectory prediction) could not reliably hit ≤20/day without
significant energy loss. Root cause of "useless" switches: most were
AppDaemon restart artifacts (seed misreads stale ac_mode → spurious
IDLE↔DISCHARGING bounce). Persistence fix (item 5) eliminates these.

**Production config** (tuned, April 2026):
- `relay_lockout_ws`: 10000 W·s (400W → 25s, 200W → 50s, 100W → 100s, 25W → 400s)
- `relay_lockout_cutoff_w`: 25 W (floor prevents infinite lockout)
- `relay_lockout_idle_s`: 90 s (long idle holdoff reduces dawn/dusk cycling)
- `mode_hysteresis_w`: 50 W (controller)

**Status**: ~30 sw/day with current config. Further reduction requires
fundamental changes (e.g. time-of-day idle windows) that aren't worth
the complexity. The persistence fix (item 5) eliminates restart-caused
useless switches, which were the main actionable category.

### 2. Deployment / Installation

**Problem**: Installing requires manual steps — copy files, configure
apps.yaml by hand, ensure module paths match AppDaemon's layout.
No clear instructions, error-prone for reinstall after HA migration.

**Goal**: Clone repo, copy one config file, done.

**Proposed structure**:
```
apps/
  zero_feed_in/          ← AppDaemon module directory
    src
      zero_feed_in_controller.py
      zendure_solarflow_driver.py
      solarflow_mqtt_watchdog.py
      csv_logger.py
   config/
       some_include.yaml
   tests/
      pointless to deploy, but no harm.
apps.yaml                ← references module: zero_feed_in.zero_feed_in_controller
apps.yaml.example        ← fully documented template with all keys + comments
```

AppDaemon resolves modules relative to the `apps/` directory, so
`module: zero_feed_in.zero_feed_in_controller` works when the repo
is cloned into `config/apps/zero_feed_in/` on the HA host.

**Steps**:
- Reorganize source layout so the package name matches a single clone target
- Write `apps.yaml.example` with every supported key, types, defaults,
  and a short inline comment explaining each
- Add a `README` section: "Installation in 3 steps"

### 3. Driver Stale-Check — DONE

**Problem**: If the controller dies but AppDaemon lives, the driver
keeps acting on a stale `sensor.zfi_desired_power`.

**Solution**: In driver `_run()`, check `last_updated` timestamp of
`sensor.zfi_desired_power` using `datetime.fromisoformat()` and wall
clock comparison.  If age > `controller_stale_s` (default 30 s):
- Both `outputLimit` and `inputLimit` sent to **0 W** (safe state).
- `sensor.zfi_controller_stale` published as `true`.
- WARNING logged on stale→fresh and fresh→stale transitions.

Config: `controller_stale_s: 30` (set to `0` to disable).

### 4. Watchdog (ESP32) — OPEN (MQTT heartbeats ready)

**Problem**: If HA/AppDaemon dies entirely, SolarFlow continues
executing the last command indefinitely. The MQTT watchdog
(`solarflow_mqtt_watchdog.py`) only handles MQTT reconnection;
it runs inside AppDaemon and dies with it.

**Defense layers currently in place:**
- Layer 1: SolarFlow BMS (hardware, always active)
- Layer 2: Driver stale-check (see #3 above)
- Layer 3: MQTT heartbeat publishing (ready for ESP32 consumer)
- Layer 4: Watchdog heartbeat monitoring (HA notifications)
- Layer 5: ESP32 watchdog (not yet implemented)

**MQTT heartbeats implemented**: Both controller and driver can publish
ISO-8601 UTC timestamps to configurable MQTT topics on every tick:
- `heartbeat_mqtt_topic: zfi/heartbeat/controller`
- `heartbeat_mqtt_topic: zfi/heartbeat/driver`

**Watchdog heartbeat monitoring implemented**: The MQTT watchdog
optionally checks `last_updated` of configured HA entities (e.g.
`sensor.zfi_desired_power`, `sensor.zfi_device_output`) every
`heartbeat_check_s` seconds.  Creates HA persistent notifications
when stale, dismisses on recovery.

**Remaining**: ESP32 (ESPHome) running independently of HA.  Subscribes
to MQTT heartbeat topics.  If stale for >2 minutes, sends HTTP POST
to SolarFlow local API to set output/input to 0W.

### 5. Persistence — DONE

**Problem**: AppDaemon restart loses integral, mode, last_computed_w.
Seed from battery_power helps but integral starts at 0.

**Solution**: Both apps now persist state to JSON files
(`zfi_controller_state.json`, `zfi_driver_state.json`) next to the
source files. Atomic writes via tmp + `os.replace()`.

- **Controller**: saves integral, last_computed_w, mode, deadband
  accumulator every ~60 s and on `terminate()`. On startup, restores
  from file; falls back to battery_power seed if file is missing or
  corrupt.
- **Driver**: saves relay SM state on each transition and on
  `terminate()`. On startup, restores SM state from file instead of
  inferring it from the device ac_mode entity (which was the cause of
  spurious restart switches).
- Optional `state_file` config key overrides the default path.

### 6. Flash Wear Investigation — DONE

**Problem**: Unknown if frequent `outputLimit`/`inputLimit` MQTT writes
cause flash wear on the SolarFlow.

**Investigation (April 2026):**
- No community reports of actual device degradation from MQTT writes.
- However, the ioBroker Zendure community confirmed with Zendure that
  `setOutputLimit`/`setInputLimit` **do write to device flash** on each call.
- Flash has ~100K write cycle endurance — a concern over years of use.
- `setDeviceAutomationInOutLimit` exists in some firmware versions as a
  combined bidirectional command, but is **not exposed as an HA entity**
  by the Zendure MQTT integration and would require raw `mqtt/publish`.
- The Zendure MQTT integration exposes a `smartMode` **switch entity**
  (`switch.*_smartmode`).  When ON, all property writes go to **RAM
  instead of flash**.  Values revert on device reboot.

**Solution**: The driver now enables `smartMode` at startup via
`call_service("switch/turn_on")`.  Since the switch resets to OFF on
device reboot, the driver re-checks it on every watchdog tick and
re-enables if needed.

- Config: `smart_mode_entity: switch.hec4nencn492140_smartmode`
- No changes to the existing command path (`outputLimit`, `inputLimit`,
  `acMode` via `number/set_value` and `select/select_option`)
- Existing deduplication still reduces unnecessary writes
- Backward compatible: omit `smart_mode_entity` to disable

## Key Design Principles

1. **Controller knows nothing about hardware.** All device specifics
   live in the driver. Controller outputs a signed watt value.

2. **ControlLogic has no HA imports.** Testable with plain dataclasses.
   The HA adapter is a thin wrapper.

3. **Surplus estimation uses measured battery power**, not reconstructed
   from last command. Eliminates dry-run drift and device lag issues.

4. **"Never charge from grid" is a guard, not a mode.** Surplus ≤ 0
   → charge blocked.

5. **Relay protection lives in the driver.** The controller can freely
   request any power level. The driver's state machine decides when
   and how to execute it.

6. **Forecast is conservative, not optimizing.** One threshold
   (1.5 kWh) switches between two min_soc values (10% / 30%).
   No complex optimization, no intraday headroom calculation.

7. **D-term and high-frequency FF don't work with current sensors.**
   Grid and PV noise levels are too high for derivative-based control.
   PV FF re-implemented with 30s EMA filter — effectiveness unverified.
   Load-specific sensors (wallbox, smart plug) are a cleaner FF path.
