# ha-zero-feed-in

Zero feed-in controller for the **Zendure SolarFlow 2400 AC+**, running as [AppDaemon](https://appdaemon.readthedocs.io/) apps on [Home Assistant](https://www.home-assistant.io/).

Keeps the grid meter at ~0 W by charging and discharging the battery based on solar surplus.

- **Solar surplus** → charge battery (absorb excess PV)
- **Solar deficit** → discharge battery (cover house demand)
- **No surplus** → never charge from grid

## Architecture

Five apps with clear separation of concerns:

```
┌──────────────────────────────┐        ┌──────────────────┐
│  Controller (device-agnostic)│        │  SolarFlow       │
│  ┌────────────────────────┐  │  MQTT  │  2400 AC+        │
│  │ Direct calculation    │  │◂─────▸│                  │
│  │ Mode switching         │  │       │  outputLimit     │
│  │ Surplus estimation     │──┤       │  inputLimit      │
│  └──────────┬─────────────┘  │       │  acMode          │
│             │                │       └──────────────────┘
│    sensor.zfi_desired_power  │
│             │                │
│  ┌──────────▼─────────────┐  │
│  │ Driver (Zendure)       │──┘
│  │ AC mode management     │
│  │ Power rounding/gating  │
│  └────────────────────────┘
└──────────────────────────────┘
```

| App | File | Responsibility |
| --- | --- | --- |
| Controller | `src/zero_feed_in_controller.py` | Direct calculation control, mode switching, surplus estimation, safety guards |
| Driver | `src/zendure_solarflow_driver.py` | AC mode, relay lockout, 5 W rounding, redundant-send suppression |
| Forecast | `src/pv_forecast_manager.py` | Adjusts dynamic min SOC based on PV forecast |
| Counter | `src/relay_switch_counter.py` | Tracks daily relay switch count |
| Watchdog | `src/solarflow_mqtt_watchdog.py` | Reconnects SolarFlow to MQTT broker after broker restarts |

### Key Features

- **Event-driven direct calculation** with muting — reacts to grid sensor changes, waits for device response
- **Drift accumulator** — sub-hysteresis errors accumulate for eventual correction
- **Schmitt trigger mode switching** with charge confirmation delay
- **AC mode send-once** — command sent on intent change, retried after 30 s timeout
- **Relay lockout** — driver tracks own intent, not MQTT entity (10-15 s device lag)
- **Device state seeding** — no unnecessary commands on startup
- **Direction switches** — enable/disable charge or discharge from HA UI
- **Layered safety**: SOC limits, grid-charge protection, surplus clamp, emergency curtailment

## Installation

### Quick Start (3 steps)

**1. Install AppDaemon**

Home Assistant → Settings → Add-ons → Add-on Store → **AppDaemon** → Install → Start

**2. Clone and Configure**

```bash
cd /root/addon_configs/a0d7b954_appdaemon/apps
git clone https://github.com/steffenruehl/ha-zero-feed-in.git zero_feed_in
cd zero_feed_in
cp config/apps.yaml.example config/apps.yaml
```

Edit `config/apps.yaml`:
- Fill in **grid_power_sensor**, **soc_sensor**, **battery_power_sensor** (controller)
- Fill in **output_limit_entity**, **input_limit_entity**, **ac_mode_entity** (driver)
- Fill in **forecast_entities** if using PV forecast
- Add device secrets to AppDaemon's `secrets.yaml`

**3. Test and Deploy**

```bash
# Dry run: no device commands, just monitoring
Set dry_run: true in config/apps.yaml
Restart AppDaemon, check sensor.zfi_* entities in Home Assistant

# Go live
Set dry_run: false
Start with max_output: 200 and increase over days
```

For detailed configuration, tuning, and troubleshooting, see the docs.

## Configuration

## Lovelace Dashboards

Pre-built debug dashboards included in `config/`:

| Dashboard | Purpose |
| --- | --- |
| [`lovelace_zfi_operations.yaml`](config/lovelace_zfi_operations.yaml) | Main control loop: grid power, surplus, battery, desired power, device state |
| [`lovelace_zfi_pi_debug.yaml`](config/lovelace_zfi_pi_debug.yaml) | Controller internals: error, muting, drift, mode transitions (requires `debug: true`) |
| [`lovelace_zfi_relay_sm_debug.yaml`](config/lovelace_zfi_relay_sm_debug.yaml) | Relay state machine: transitions, lockout progress, energy accumulation (requires `debug: true`) |

**Setup**: In Home Assistant, go to **Dashboards** → **Create new** → **Edit in YAML** → paste the entire contents of a dashboard file.

See [docs/zero_feed_in_docs.md#lovelace-dashboards](docs/zero_feed_in_docs.md#lovelace-dashboards) for detailed descriptions and setup.

## Documentation

- **[docs/zero_feed_in_docs.md](docs/zero_feed_in_docs.md)** — Complete technical documentation:
  - System architecture and design decisions
  - Controller and driver concepts
  - Direct calculation controller tuning (ki, hysteresis, muting)
  - Protection mechanisms (SOC, grid-charge, relay switching)
  - Configuration reference and published sensors
  - Flowcharts, example scenarios, and troubleshooting guide

- **[docs/DASHBOARDS.md](docs/DASHBOARDS.md)** — Lovelace dashboard setup:
  - Pre-built dashboards for monitoring and debugging
  - Dashboard installation and entity ID customization

## Hardware

Developed for:

- **Zendure SolarFlow 2400 AC+** (AC-coupled battery storage)
- **Home Assistant** with AppDaemon addon
- Grid meter via IR reader (EDL21 or similar)

The SolarFlow must be connected to Home Assistant via local MQTT.

## License

[MIT](LICENSE)
