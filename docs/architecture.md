# Zero Feed-In — Architecture & Installation

## Overview

AppDaemon apps for the Zendure SolarFlow 2400 AC+ that keep the grid meter at ~0 W:

1. **Controller** (`zero_feed_in_controller.py`) — Device-agnostic event-based controller. Reads grid power, SOC, and battery power sensors. Publishes a signed desired-power value.
2. **Driver** (`zendure_solarflow_driver.py`) — Zendure-specific driver. Reads the desired power and translates it into `outputLimit`, `inputLimit`, and `acMode` commands.
3. **PV Forecast Manager** (`pv_forecast_manager.py`) — Adjusts min SOC based on tomorrow's PV forecast.
4. **Pulse-Load Detector** (`pulse_load_detector.py`) — Detects periodic pulse loads (oven duty-cycling) via sign-flip analysis.
5. **Pulse-Load Filter** (`pulse_load_filter.py`) — Baseline mitigation: outputs a stable baseline during detected pulse loads.
6. **MQTT Watchdog** (`solarflow_mqtt_watchdog.py`) — Reconnects SolarFlow when MQTT goes stale.

Supporting modules:
- `csv_logger.py` — Shared daily-rotating CSV file logger
- `relay_switch_counter.py` — Relay switch event counter

---

## System Architecture

```
┌──────────────┐                     ┌───────────────┐
│  Existing PV │───── AC ──────────▸ │   House Grid  │
│  System      │  (solar power)      │               │
└──────────────┘                     │  ┌─────────┐  │
                                     │  │  Loads  │  │
┌──────────────┐    MQTT (W)         │  └─────────┘  │
│  EDL21 IR    │───────────────────▸ │               │
│  Reader      │                     │               │◂──── Utility Grid
└──────────────┘                     └───────┬───────┘
       │                                     │ AC
       ▼                                     │
┌──────────────────────────────────┐ ┌───────┴────────┐
│  Home Assistant                  │ │  SolarFlow     │
│  ┌────────────────────────────┐  │ │  2400 AC+      │
│  │  AppDaemon                 │  │ │                │
│  │  ┌──────────────────────┐  │  │ │  outputLimit   │
│  │  │  PL Detector         │  │  │ │  inputLimit    │
│  │  │  (sign-flip detect)  │  │  │ │  acMode        │
│  │  └──────────┬───────────┘  │  │ └────────────────┘
│  │    zfi_pld_active          │  │        ▲
│  │  ┌──────────▼───────────┐  │  │        │
│  │  │  PL Filter           │  │  │        │
│  │  │  (baseline mitigat)  │  │  │        │
│  │  └──────────┬───────────┘  │  │        │
│  │    filtered_grid_power     │  │        │
│  │  ┌──────────▼───────────┐  │  │        │
│  │  │  Controller          │──┤──┤────────┘
│  │  │  (direct calc)       │  │  │
│  │  └──────────┬───────────┘  │  │
│  │    zfi_desired_power       │  │
│  │  ┌──────────▼───────────┐  │  │
│  │  │  Driver (Zendure)    │──┘  │
│  │  └──────────────────────┘     │
│  └────────────────────────────┘  │
└──────────────────────────────────┘
```

### Data Flow

```
PL Detector (event-driven):
  grid_power_sensor ──▸ sign-flip analysis ──▸ sensor.zfi_pld_active

PL Filter (event-driven):
  grid_power_sensor ──────┐
  sensor.zfi_pld_active ──┤──▸ baseline estimation ──▸ filtered_power_entity
                          │

Controller (event-driven):
  grid_power_sensor ─────────┐
  soc_sensor ────────────────┤──▸ direct calc ──▸ sensor.zfi_desired_power
  battery_power_sensor───────┤
  dynamic_min_soc_entity ────┘

Driver (every 2s):
  sensor.zfi_desired_power ──▸ AC mode + outputLimit + inputLimit
```

### Why Two Apps (Controller + Driver)?

| Concern | Controller | Driver |
| --- | --- | --- |
| What it knows | Grid power, SOC, surplus | Device protocol, relay timing |
| What it doesn't know | outputLimit, inputLimit, acMode | ki, targets, modes |
| Reusable for | Any battery | Only Zendure SolarFlow |
| Update rate | Event-driven (grid sensor) | 2 s (react to new desired power) |

---

## Design Decisions

### Why AppDaemon (not HA Blueprint)?

- Controller with state, mode machine, guards, and lockouts doesn't fit YAML templates
- AppDaemon gives real Python: dataclasses, enums, testable methods, proper logging

### Why Two Actuators (not one)?

- `setOutputLimit`: discharge power (W), feeds into house
- `setInputLimit`: AC charge power (W), charges battery
- `acMode`: select entity, must be "Input mode" before charging, "Output mode" before discharging
- All MQTT entities, actuators are write-only from the driver's perspective

### Why Sensors are Read-Only, Actuators Write-Only

- Number entities show *commanded* value, not actual output
- Controller uses its own state (`last_sent_w`), surplus uses `battery_power_sensor`
- Only external sensors needed by controller: `grid_power`, `soc`, `battery_power`

### Key Design Principles

1. **Controller knows nothing about hardware.** All device specifics live in the driver.
2. **ControlLogic has no HA imports.** Testable with plain dataclasses.
3. **Surplus estimation uses measured battery power**, not reconstructed from last command.
4. **"Never charge from grid" is a guard, not a mode.** Surplus ≤ 0 → charge blocked.
5. **Relay protection lives in the driver.** The controller can freely request any power level.
6. **Forecast is conservative, not optimizing.** One threshold, two min_soc values.
7. **Muting prevents chasing stale readings.** Simpler and more robust than complex compensators.
8. **Detection and mitigation are separate.** Detector publishes an active signal. Mitigation modules can be swapped independently.

---

## File Structure

```
src/
├── zero_feed_in_controller.py    # device-agnostic direct calculation controller
├── zendure_solarflow_driver.py   # Zendure SolarFlow driver
├── pv_forecast_manager.py        # PV forecast → dynamic min SOC
├── pulse_load_detector.py        # sign-flip detection (publishes active signal)
├── pulse_load_filter.py          # baseline mitigation (reads active, outputs filtered)
├── relay_switch_counter.py       # relay switch event counter
├── csv_logger.py                 # shared daily-rotating CSV file logger
└── solarflow_mqtt_watchdog.py    # MQTT reconnect watchdog
config/
├── apps.yaml.example             # documented configuration template
├── apps.yaml                     # your configuration (git-ignored)
├── secrets.yaml.example          # template for device credentials
└── lovelace_*.yaml               # Lovelace dashboard YAML files
tests/
└── test_*.py                     # unit tests covering all modules
docs/
├── architecture.md               # this file — global architecture & installation
├── controller.md                 # controller documentation
├── driver.md                     # driver documentation
├── pv_forecast_manager.md        # PV forecast manager documentation
├── pulse_load_detector.md        # pulse-load detector documentation
├── pulse_load_filter.md          # pulse-load filter documentation
├── DASHBOARDS.md                 # Lovelace dashboard setup guide
└── pulse_load_filter_approaches.md  # design notes (discarded alternatives)
```

---

## Measured Device Timing

Relay closed, step response settle times:

| Quadrant | Physical action | T1 (s) |
|---|---|---|
| discharge_up | increase discharge | 5.0 |
| discharge_down | decrease discharge | 3.5 |
| charge_up | increase charge | 7.5 |
| charge_down | decrease charge | 5.5 |

Relay switching:
- Charge relay on: 8 s, off: 1 s
- Discharge relay on: 3–6 s, off: 4 s

**Key finding**: no slew rate behavior. Settle time does not scale with step size. The device has a processing delay, not a ramp limit.

---

## Installation

### 1. Install AppDaemon

Settings → Add-ons → Add-on Store → AppDaemon → Install → Start

### 2. Clone the Repository

```bash
cd /root/addon_configs/a0d7b954_appdaemon/apps
git clone https://github.com/steffenruehl/ha-zero-feed-in.git zero_feed_in
```

### 3. Configure

```bash
cp zero_feed_in/config/apps.yaml.example zero_feed_in/config/apps.yaml
```

Edit `zero_feed_in/config/apps.yaml` — fill in the **MANDATORY** sections for each app:

| apps.yaml key | App | Typical HA entity |
| --- | --- | --- |
| `grid_power_sensor` | Controller | `sensor.smart_meter_*` |
| `soc_sensor` | Controller | `sensor.*_electriclevel` |
| `battery_power_sensor` | Controller | `sensor.*_net_power` (template or helper) |
| `desired_power_sensor` | Driver | `sensor.zfi_desired_power` (from controller) |
| `output_limit_entity` | Driver | `number.*_outputlimit` |
| `input_limit_entity` | Driver | `number.*_inputlimit` |
| `ac_mode_entity` | Driver | `select.*_acmode` |
| `forecast_entities` | Forecast | `sensor.energy_production_tomorrow*` |
| `watch_entity` | Watchdog | `sensor.*_electriclevel` |
| `grid_power_sensor` | Detector | Same as controller |
| `active_entity` | Filter | `sensor.zfi_pld_active` (from detector) |

Optional switches (create as HA helpers → Toggle):

| apps.yaml key | HA entity |
| --- | --- |
| `charge_switch` | `input_boolean.zfi_charge_enabled` |
| `discharge_switch` | `input_boolean.zfi_discharge_enabled` |

Add secrets to your AppDaemon `secrets.yaml` (see `config/secrets.yaml.example`).

### 4. Start in Dry Run

Set `dry_run: true` (the YAML anchor at the top of `apps.yaml` applies to all apps). Set `debug: true` to publish internal sensors for the debug dashboards. Monitor via AppDaemon log and `sensor.zfi_*` entities.

### 5. Go Live

Set `dry_run: false`. Start with `max_output: 200`. Once stable, set `debug: false` to reduce HA sensor churn.

---

## Known Limitations

- **Device response time**: 10–15 s (not 2–4 s as Zendure docs suggest)
- **Flash writes**: `setOutputLimit`/`setInputLimit` may write to device flash (mitigated by `smartMode` RAM lock)
- **Single phase**: SolarFlow feeds one phase; three-phase balancing at meter works
- **Single instance only**: no multi-device HEMS support

---

## Example Scenarios

### 1. Sunny day — charging from surplus

```
PV=1200W, house=500W, battery charging 400W
battery_sensor = -400 (charging → negative per convention)
grid = house - PV + charge = 500 - 1200 + 400 = -300W (feeding in)

surplus = -battery_power_w - grid = -(-400) - (-300) = 700W
mode: CHARGING, target = 0W
→ desired_power ≈ -650W
→ driver: inputLimit=650W, outputLimit=0W
```

### 2. Cloud passes — reduce charging

```
PV drops to 600W, house=500W, battery charging 650W
grid = 500 - 600 + 650 = +550W (drawing from grid!)

surplus = 650 - 550 = 100W
mode: stays CHARGING (hysteresis), target = 0W
→ desired_power ≈ -100W
→ driver: inputLimit=100W
```

### 3. Evening — discharge to cover load

```
PV=0W, house=400W, battery idle → grid = +400W

surplus = -400W → DISCHARGING, target = 30W
error = 400 - 30 = 370
→ desired_power ≈ +370W
→ driver: outputLimit=370W, inputLimit=0W
```

### 4. Battery full — surplus goes to grid

```
SOC=85%, surplus=500W → guard: SOC ≥ max → idle
→ desired_power = 0W → surplus flows to grid
```

---

## Troubleshooting

| Problem | Action |
| --- | --- |
| Output oscillates | Reduce ki, increase hysteresis or muting |
| Persistent offset | Increase ki (default 1.0 should eliminate offsets) |
| Sluggish on load changes | Increase ki, reduce muting |
| Relay clicks frequently | Increase hysteresis, increase muting |
| Mode flaps | Increase mode_hysteresis (80–100 W), increase charge_confirm (25 s) |
| Charges from grid briefly | Check surplus in logs, increase hysteresis |

---

## Related Projects & References

- **alkly.de Blueprint**: commercial HA blueprint for SolarFlow 2400 Pro zero feed-in
- **solarflow-control** (GitHub: reinhard-brandstaedter): Python tool for DC-coupled SolarFlow hubs + OpenDTU
- **ioBroker zendure-solarflow adapter**: full-featured adapter with controller scripts
- **z-master42/solarflow**: HA MQTT YAML config for SolarFlow entities
