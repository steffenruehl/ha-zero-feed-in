# ha-zero-feed-in

Zero feed-in controller for the **Zendure SolarFlow 2400 AC+**, running as an [AppDaemon](https://appdaemon.readthedocs.io/) app on [Home Assistant](https://www.home-assistant.io/).

Keeps the grid meter at ~0 W by charging and discharging the battery based on solar surplus.

- **Solar surplus** → charge battery (absorb excess PV)
- **Solar deficit** → discharge battery (cover house demand)
- **No surplus** → never charge from grid

## How It Works

A PI controller reads the grid power meter every 5 seconds and adjusts the SolarFlow's charge/discharge limits to keep net grid power near zero.

```
┌──────────────┐                     ┌───────────────┐
│  Existing PV │───── AC ──────────▸ │   House Grid  │
│  System      │  (solar power)      │               │
└──────────────┘                     │  ┌─────────┐  │
                                     │  │  Loads  │  │
┌──────────────┐    MQTT (W)         │  └─────────┘  │
│  Grid Meter  │───────────────────▸ │               │
│  (IR reader) │                     │               │◂──── Utility Grid
└──────────────┘                     └───────┬───────┘
                                             │ AC
┌──────────────────────────┐        ┌────────┴────────┐
│  Home Assistant          │  MQTT  │  SolarFlow      │
│  ┌────────────────────┐  │◂─────▸│  2400 AC+       │
│  │  AppDaemon         │  │       │                  │
│  │  ZeroFeedIn        │  │       │  charge/discharge│
│  └────────────────────┘  │       └──────────────────┘
└──────────────────────────┘
```

### Key Features

- **Position-form PI** with asymmetric gains — cautious ramp-up, aggressive ramp-down
- **Anti-windup back-calculation** — integral stays sane at output limits
- **Schmitt trigger mode switching** with charge confirmation delay
- **Relay wear minimization** — AC mode read from entity, redundant sends suppressed
- **Device state seeding** — no unnecessary commands on startup
- **Direction switches** — enable/disable charge or discharge from HA UI
- **Layered safety**: SOC limits, grid-charge protection, surplus clamp, feed-in emergency curtailment

## Installation

### 1. Install AppDaemon

Home Assistant → Settings → Add-ons → Add-on Store → **AppDaemon** → Install → Start

### 2. Deploy Files

Copy to your AppDaemon apps directory:

```
config/appdaemon/apps/
├── zero_feed_in.py
└── apps.yaml
```

### 3. Configure Entity Names

Find your SolarFlow's actual entity names (via MQTT Explorer or HA Developer Tools) and update `apps.yaml`:

```yaml
zero_feed_in:
  module: zero_feed_in
  class: ZeroFeedIn

  # Sensors
  grid_power_sensor: sensor.your_grid_meter
  solarflow_soc_sensor: sensor.your_solarflow_electriclevel

  # Actuators
  output_limit_entity: number.your_solarflow_outputlimit
  input_limit_entity: number.your_solarflow_inputlimit
  ac_mode_entity: select.your_solarflow_acmode

  # Optional: direction switches (create as HA helpers)
  # charge_switch: input_boolean.zfi_charge_enabled
  # discharge_switch: input_boolean.zfi_discharge_enabled
```

### 4. Start in Dry Run

The default `dry_run: true` computes everything and publishes HA sensors but sends no commands. Monitor via the AppDaemon log and `sensor.zfi_*` entities.

### 5. Go Live

```yaml
  dry_run: false
```

Start with `max_output: 200` and increase over several days.

## Configuration

| Parameter | Default | Description |
|---|---|---|
| `target_grid_power` | 30 W | Discharge target (small grid draw as buffer) |
| `charge_target_power` | 0 W | Charge target (absorb all surplus) |
| `kp_up` / `kp_down` | 0.3 / 0.8 | Proportional gains (up = cautious, down = aggressive) |
| `ki_up` / `ki_down` | 0.03 / 0.08 | Integral gains |
| `deadband` | 25 W | Error range where PI freezes |
| `interval` | 5 s | Control cycle interval |
| `max_output` | 800 W | Maximum discharge power |
| `max_charge` | 2400 W | Maximum charge power |
| `min_soc` / `max_soc` | 10% / 100% | SOC limits |
| `mode_hysteresis` | 50 W | Surplus band for mode switching |
| `charge_confirm` | 15 s | Seconds surplus must hold before charging |
| `direction_lockout` | 5 s | Min time between relay direction changes |
| `max_feed_in` | 800 W | Emergency curtailment threshold |

See [apps.yaml](apps.yaml) for the full annotated configuration.

## Published HA Sensors

The controller publishes `sensor.zfi_*` entities every cycle:

| Sensor | Description |
|---|---|
| `zfi_mode` | Operating regime (charging/discharging) |
| `zfi_relay` | Physical relay state (charge/idle/discharge) |
| `zfi_surplus` | Estimated solar surplus (W) |
| `zfi_error` | Regulation error (W) |
| `zfi_p_term` / `zfi_i_term` | PI components (W) |
| `zfi_discharge_limit` / `zfi_charge_limit` | Commands sent to device (W) |
| `zfi_reason` | Human-readable decision reason |

## Documentation

- [zero_feed_in_docs.md](zero_feed_in_docs.md) — Full technical documentation with flowcharts, examples, and tuning guide
- [development_context.md](development_context.md) — Architecture decisions, known issues, and development history

## Hardware

Developed for:
- **Zendure SolarFlow 2400 AC+** (AC-coupled battery storage)
- **Home Assistant** with AppDaemon addon
- Grid meter via IR reader (EDL21 or similar)

The SolarFlow must be connected to Home Assistant via local MQTT.

## License

[MIT](LICENSE)
