# ha-zero-feed-in

Zero feed-in controller for the **Zendure SolarFlow 2400 AC+**, running as [AppDaemon](https://appdaemon.readthedocs.io/) apps on [Home Assistant](https://www.home-assistant.io/).

Keeps the grid meter at ~0 W by charging and discharging the battery based on solar surplus.

- **Solar surplus** → charge battery (absorb excess PV)
- **Solar deficit** → discharge battery (cover house demand)
- **No surplus** → never charge from grid

## Architecture

Two apps with clear separation of concerns:

```
┌──────────────────────────────┐        ┌──────────────────┐
│  Controller (device-agnostic)│        │  SolarFlow       │
│  ┌────────────────────────┐  │  MQTT  │  2400 AC+        │
│  │ PI controller          │  │◂─────▸│                  │
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
| Controller | `zero_feed_in_controller.py` | PI control, mode switching, surplus estimation, safety guards |
| Driver | `zendure_solarflow_driver.py` | AC mode, relay lockout, 5 W rounding, redundant-send suppression |
| Watchdog | `solarflow_mqtt_watchdog.py` | Reconnects SolarFlow to MQTT broker after broker restarts |

### Key Features

- **Position-form PI** with asymmetric gains — cautious ramp-up, aggressive ramp-down
- **Anti-windup back-calculation** — integral stays sane at output limits
- **Schmitt trigger mode switching** with charge confirmation delay
- **AC mode send-once** — command sent on intent change, retried after 30 s timeout
- **Relay lockout** — driver tracks own intent, not MQTT entity (10-15 s device lag)
- **Device state seeding** — no unnecessary commands on startup
- **Direction switches** — enable/disable charge or discharge from HA UI
- **Layered safety**: SOC limits, grid-charge protection, surplus clamp, emergency curtailment

## Installation

### 1. Install AppDaemon

Home Assistant → Settings → Add-ons → Add-on Store → **AppDaemon** → Install → Start

### 2. Deploy Files

Copy to your AppDaemon apps directory:

```
config/appdaemon/apps/
├── zero_feed_in_controller.py
├── zendure_solarflow_driver.py
├── solarflow_mqtt_watchdog.py
└── apps.yaml
```

Create `config/appdaemon/secrets.yaml` with your device credentials (see `config/secrets.yaml.example`):

```yaml
solarflow_ip:     "192.168.x.x"
solarflow_serial: "XXXXXXXXXXX"
mqtt_broker_ip:   "192.168.x.x"
mqtt_username:    "mqtt"
mqtt_password:    "mqtt"
```

### 3. Configure Entity Names

Find your SolarFlow's actual entity names (via MQTT Explorer or HA Developer Tools) and update `apps.yaml`:

```yaml
# Controller
zero_feed_in_controller:
  module: zero_feed_in_controller
  class: ZeroFeedInController
  grid_power_sensor: sensor.your_grid_meter
  soc_sensor: sensor.your_solarflow_electriclevel
  battery_power_sensor: sensor.your_battery_net_power  # +discharge/-charge

# Driver
zendure_solarflow_driver:
  module: zendure_solarflow_driver
  class: ZendureSolarFlowDriver
  desired_power_sensor: sensor.zfi_desired_power
  output_limit_entity: number.your_solarflow_outputlimit
  input_limit_entity: number.your_solarflow_inputlimit
  ac_mode_entity: select.your_solarflow_acmode
```

### 4. Start in Dry Run

Both apps default to `dry_run: true` — they compute and publish HA sensors but send no commands.

### 5. Go Live

```yaml
  dry_run: false
```

Start with `max_output: 200` and increase over several days.

## Configuration

### Controller

| Parameter | Default | Description |
| --- | --- | --- |
| `target_grid_power` | 30 W | Discharge target (small grid draw as buffer) |
| `charge_target_power` | 0 W | Charge target (absorb all surplus) |
| `kp_discharge_up` / `kp_discharge_down` | 0.25 / 0.35 | Proportional gain: increase / decrease discharge |
| `kp_charge_up` / `kp_charge_down` | 0.17 / 0.23 | Proportional gain: increase / decrease charge |
| `ki_discharge_up` / `ki_discharge_down` | 0.025 / 0.025 | Integral gain: increase / decrease discharge |
| `ki_charge_up` / `ki_charge_down` | 0.036 / 0.036 | Integral gain: increase / decrease charge |
| `deadband` | 25 W | Error range where PI freezes |
| `interval` | 5 s | Control cycle interval |
| `max_output` | 800 W | Maximum discharge power |
| `max_charge` | 2400 W | Maximum charge power |
| `min_soc` / `max_soc` | 10% / 100% | SOC limits |
| `mode_hysteresis` | 50 W | Surplus band for mode switching |
| `charge_confirm` | 15 s | Seconds surplus must hold before charging |
| `max_feed_in` | 800 W | Emergency curtailment threshold |
| `ff_enabled` | false | Enable multi-source feed-forward |
| `ff_deadband` | 30 W | Ignore total FF term below this magnitude |
| `ff_filter_tau_s` | 30 s | EMA time constant for FF derivative filter |

### Driver

| Parameter | Default | Description |
| --- | --- | --- |
| `interval` | 2 s | Poll interval (faster than controller) |
| `direction_lockout` | 30 s | Base lockout time between relay direction changes |
| `relay_sm_enabled` | true | Enable/disable relay state machine |
| `adaptive_lockout_ref_w` | 200 W | Reference power for full-speed lockout (low power → longer lockout) |
| `min_active_power_w` | 25 W | Minimum power floor when relay is in an active state |

### Watchdog

| Parameter | Default | Description |
| --- | --- | --- |
| `watch_entity` | — | HA entity to monitor (goes unavailable when MQTT drops) |
| `unavailable_duration_s` | 120 s | Trigger reconnect after this many seconds unavailable |
| `startup_delay_s` | 30 s | Wait after HA start before sending reconnect (lets Mosquitto start first) |

See [apps.yaml](config/apps.yaml) for the full annotated configuration.

## Published HA Sensors

Both apps publish `sensor.zfi_*` entities every cycle:

Always published:

| Sensor | Source | Description |
| --- | --- | --- |
| `zfi_desired_power` | Controller | Signed desired power (W): +discharge, -charge |
| `zfi_mode` | Controller | Operating regime (charging/discharging) |
| `zfi_device_output` | Driver | Signed power sent to device (W) |
| `zfi_discharge_limit` | Driver | outputLimit sent (W) |
| `zfi_charge_limit` | Driver | inputLimit sent (W) |
| `zfi_relay` | Driver | Physical relay state |
| `zfi_relay_locked` | Driver | `true` while SM is clamping or relay is physically switching |

Published when `debug: true`:

| Sensor | Source | Description |
| --- | --- | --- |
| `zfi_surplus` | Controller | Estimated solar surplus (W) |
| `zfi_battery_power` | Controller | Actual battery power (W) |
| `zfi_error` | Controller | Regulation error (W) |
| `zfi_p_term` / `zfi_i_term` / `zfi_ff` | Controller | PI and feed-forward components (W) |
| `zfi_ff_pv_raw` / `zfi_ff_pv_ema` / `zfi_ff_pv_contrib` | Controller | PV FF: live reading, EMA state, contribution pre-deadband (W) |
| `zfi_ff_others_contrib` | Controller | Non-PV load sources FF contribution pre-deadband (W) |
| `zfi_integral` | Controller | PI integral accumulator (W) |
| `zfi_reason` | Controller | Human-readable decision reason |
| `zfi_relay_sm_state` / `zfi_relay_sm_pending` | Driver | Relay state machine state and pending target |
| `zfi_relay_sm_lockout_pct` | Driver | Transition progress (%) |

## Documentation

- [zero_feed_in_docs.md](zero_feed_in_docs.md) — Full technical documentation with flowcharts, dashboards, and tuning guide
- [development_context.md](development_context.md) — Architecture decisions, known issues, and development history

## Hardware

Developed for:

- **Zendure SolarFlow 2400 AC+** (AC-coupled battery storage)
- **Home Assistant** with AppDaemon addon
- Grid meter via IR reader (EDL21 or similar)

The SolarFlow must be connected to Home Assistant via local MQTT.

## License

[MIT](LICENSE)
