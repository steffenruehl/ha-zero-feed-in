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
- 208 unit tests (101 controller, 99 driver, 8 CSV logger) — all passing
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

### 3. Driver Stale-Check — SMALL

**Problem**: If the controller dies but AppDaemon lives, the driver
keeps acting on a stale `sensor.zfi_desired_power`.

**Solution**: In driver `_run`, check `last_updated` age:
```python
last_update = self.get_state(self.cfg.desired_power_sensor,
                             attribute="last_updated")
age = (now_wall - parse(last_update)).total_seconds()
if age > 30:  # controller hasn't updated in 30s
    self._send_safe_state()  # 0W output, 0W input
    return
```

Note: use wall time for this check (not `time.monotonic()`).

### 4. Watchdog (ESP32) — OPEN

**Problem**: If HA/AppDaemon dies entirely, SolarFlow continues
executing the last command indefinitely. The MQTT watchdog
(`solarflow_mqtt_watchdog.py`) only handles MQTT reconnection;
it runs inside AppDaemon and dies with it.

**Defense layers currently in place:**
- Layer 1: SolarFlow BMS (hardware, always active)
- Layer 2: Driver stale-check (see #3 above)
- Layer 3: ESP32 watchdog (not yet implemented)

**Solution**: ESP32 (ESPHome) running independently of HA. Polls
HA API every 30s to check if `sensor.zfi_desired_power` is fresh.
If stale for >2 minutes, sends HTTP POST to SolarFlow local API
to set output/input to 0W.

```
ESP32 checks:
  1. HTTP GET http://HA_IP:8123/api/states/sensor.zfi_desired_power
  2. Parse last_updated, check age
  3. If age > 120s for 4 consecutive checks:
     HTTP POST http://SOLARFLOW_IP/rpc → safe state (0W)
```

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

### 6. Flash Wear Investigation — RESEARCH

**Problem**: Unknown if frequent `outputLimit`/`inputLimit` MQTT writes
cause flash wear on the SolarFlow. Driver sends a command every
`watchdog_s` seconds; redundant-send suppression deduplicates identical
values, so actual writes are fewer — but the exact write rate is unknown.

**Action**: Check Zendure community/forums for flash wear reports.
Test if `setDeviceAutomationInOutLimit` exists as a single bidirectional
entity that might reduce write frequency.

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
