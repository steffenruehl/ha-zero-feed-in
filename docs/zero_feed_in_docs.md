# Zero Feed-In — Documentation

## Overview

Two AppDaemon apps for the Zendure SolarFlow 2400 AC+ that keep the grid meter at ~0 W:

1. **Controller** (`zero_feed_in_controller.py`) — Device-agnostic PI controller. Reads grid power, SOC, and battery power sensors. Publishes a signed desired-power value.
2. **Driver** (`zendure_solarflow_driver.py`) — Zendure-specific driver. Reads the desired power and translates it into `outputLimit`, `inputLimit`, and `acMode` commands.

- **Solar surplus** → charge battery (absorb excess PV)
- **Solar deficit** → discharge battery (cover house demand)
- **No surplus** → **never** charge from grid

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
┌──────────────────────────┐        ┌────────┴────────┐
│  Home Assistant          │  MQTT  │  SolarFlow      │
│  ┌────────────────────┐  │◂─────▸│  2400 AC+       │
│  │  AppDaemon         │  │       │                  │
│  │  ┌──────────────┐  │  │       │  setOutputLimit  │
│  │  │  Controller   │──┤──┤──────▸│  setInputLimit   │
│  │  │  (PI)        │  │  │       │  acMode          │
│  │  └──────┬───────┘  │  │       └──────────────────┘
│  │         │desired_W  │  │
│  │  ┌──────▼───────┐  │  │
│  │  │  Driver      │──┘  │
│  │  │  (Zendure)   │     │
│  │  └──────────────┘     │
│  │                       │
│  │  sensor.zfi_*         │
│  │  (published states)   │
│  └───────────────────────┘
└──────────────────────────┘
```

### Data flow

```
Controller (every 5s):
  grid_power_sensor ─────────┐
  soc_sensor ────────────────┤
  battery_power_sensor───────┤──▸ PI + FF ──▸ sensor.zfi_desired_power
  ff_sources (optional)──────┤               sensor.zfi_mode, surplus, etc.
  relay_locked_sensor────────┤               (integral frozen when locked)
  dynamic_min_soc_entity ────┘               (forecast-adjusted min SOC)

Driver (event-driven + watchdog every 5 s):
  sensor.zfi_desired_power ──▸ AC mode + outputLimit + inputLimit
                                sensor.zfi_device_output, relay, etc.
```

### Why two apps?

| Concern | Controller | Driver |
| --- | --- | --- |
| What it knows | Grid power, SOC, surplus | Device protocol, relay timing |
| What it doesn't know | outputLimit, inputLimit, acMode | PI gains, targets, modes |
| Reusable for | Any battery | Only Zendure SolarFlow |
| Update rate | 5 s (PI cycle) | Event-driven + watchdog (typically 5 s) |

---

## Controller: Key Concepts

### 1. Solar Surplus Estimation

The controller uses the **actual battery power sensor** (updated every ~4 s by the device) instead of the last commanded value:

```
battery_power_sensor: +positive = discharge, -negative = charge
controller convention: +positive = discharge, -negative = charge (same)

battery_power_w = battery_power_sensor  (no negation)

surplus = -battery_power_w - grid_power_w
        = -(actual battery) - grid
```

This is more accurate than the old `last_sent_w` approach because it reflects what the device is actually doing, not what was commanded. Particularly important during the 10-15 s device response lag.

| Scenario | battery_power_w | grid | surplus |
| --- | --- | --- | --- |
| Charging 500 W, grid +100 W | -500 | +100 | 400 W |
| Discharging 300 W, grid +100 W | +300 | +100 | -400 W |
| Charging 500 W, grid -200 W | -500 | -200 | 700 W |
| Idle, grid -50 W | 0 | -50 | 50 W |

### 2. Operating Mode (Schmitt Trigger with Charge Confirmation)

```
                    surplus > +hysteresis (sustained for charge_confirm_s)
    DISCHARGING ──────────────────────────────────▸ CHARGING
                ◂──────────────────────────────────
                    surplus < -hysteresis (instant)
```

- **DISCHARGING → CHARGING**: Surplus must stay above threshold for `charge_confirm_s` (default 15–20 s). Prevents transient spikes from triggering expensive relay switches.
- **CHARGING → DISCHARGING**: Instant. Cover demand quickly.
- On mode transition: integral and last_computed_w reset to 0.

### 3. Asymmetric Targets

| Mode | Target | Rationale |
| --- | --- | --- |
| DISCHARGING | +30 W | Small grid draw OK as safety buffer |
| CHARGING | 0 W | Absorb all surplus, never pull from grid |

### 4. Surplus Clamp

```python
if raw < 0:  # wants to charge
    max_safe = max(0, surplus)
    clamped = max(raw, -max_charge, -max_safe)
```

### 5. Direction Switches

Optional `input_boolean` entities for HA UI control:

| Switch | When off |
| --- | --- |
| `charge_switch` | Controller idles instead of charging |
| `discharge_switch` | Controller idles instead of discharging |

When any guard forces the controller to idle (direction switches, SOC limits,
no-surplus protection), the **integral is reset to zero**.  The integral only
has meaning in a closed control loop; while the output is blocked (loop open),
there is no feedback, so the integral becomes stale.  Resetting ensures the PI
starts fresh when the guard clears.

---

## Controller: PI Controller + Feed-Forward

### Position Form

```
error = grid_power - target

gains = gain_set.select(mode, error)    # four-quadrant lookup
P = gains.kp × error
I = I_prev + gains.ki × error × dt

pi_output = P + I
```

NOT velocity/incremental form. Critical with the SolarFlow's 10-15 s response latency.

### Multi-Source Feed-Forward

The ``FeedForward`` class processes an arbitrary list of sensor sources.
Each source has a sign (+1 for loads, -1 for generation) and a gain.

Instead of a raw delta, the derivative is taken on an **EMA-filtered** value
(ISA PID D-filter). This attenuates high-frequency sensor noise while preserving
slow ramps that merit a feed-forward response:

```
α = interval_s / (filter_tau_s + interval_s)   # e.g. 5/(30+5) ≈ 0.14

for each source:
    EMA_t = α × current + (1−α) × EMA_{t-1}    # update filter state
    delta = α × (current − EMA_{t-1})           # filtered derivative
    contrib += sign × gain × delta

if |contrib| < ff_deadband:
    contrib = 0
combined = pi_output + contrib
```

With τ=30 s a 200 W spike produces a 28 W FF impulse — below the 30 W deadband,
so short cloud transients are ignored. Sustained slow ramps exceed the deadband
and do trigger a feed-forward response.

PV drops → positive ff (increase discharge). Load increases → positive ff.
New sources are a YAML entry, no code changes required.
Set ``ff_enabled: false`` to disable feed-forward without removing sources.

Each source can carry an optional ``name`` that is used for HA debug sensor
names (e.g. ``name: pv`` → ``sensor.zfi_ff_pv_raw``).

Example configuration:
```yaml
ff_enabled: true
ff_deadband: 30
ff_filter_tau_s: 30     # EMA time constant (s); α = interval/(tau+interval)
feed_forward_sources:
  - entity: sensor.pv_power
    gain: 0.6
    sign: -1.0          # generation
    name: pv            # → sensor.zfi_ff_pv_raw / _ema / _contrib
  - entity: sensor.wallbox_power
    gain: 0.8
    sign: 1.0           # load
```

Backward compatible: legacy ``pv_sensor`` / ``ff_pv_gain`` keys
still work and create a single FF source automatically.

### Four-Quadrant Gains

Gains are selected per cycle based on operating mode × error sign.
Derived from step response measurements (SIMC: Kp = interval / (2×T1), Ki = Kp / (4×T1)).

| Quadrant | Physical action | T1 (s) | Kp | Ki |
| --- | --- | --- | --- | --- |
| `discharge_up` | increase discharge | 5.0 | 0.50 | 0.025 |
| `discharge_down` | decrease discharge | 3.5 | 0.71 | 0.051 |
| `charge_up` | increase charge | 7.5 | 0.33 | 0.011 |
| `charge_down` | decrease charge | 5.5 | 0.45 | 0.021 |

Selection matrix:

|  | error ≥ 0 | error < 0 |
| --- | --- | --- |
| DISCHARGING | discharge_up | discharge_down |
| CHARGING | charge_down | charge_up |

### Anti-Windup (Back-Calculation)

When output hits limits, `integral = limit - P_term`. Prevents windup during saturation.

### Relay Lockout Anti-Windup

When the driver’s relay state machine is clamping output (e.g. during a relay
transition lockout), the driver publishes `sensor.zfi_relay_locked = "true"`.Additionally, `relay_locked` stays `true` for 8 seconds after each SM
transition (`RELAY_SWITCH_DELAY_S`) to account for the physical relay
switching time. This prevents the controller from winding up the integral
while the relay is physically switching and the device cannot yet actuate
the new setpoint.The controller reads this via the optional `relay_locked_sensor` config and
**freezes the integral**: no new integral is committed, and no back-calculation
is performed. The normal output pipeline still runs (P + I + FF → clamp), so
the driver sees the controller’s intent, but the integral does not wind up
while the device cannot actuate.

The reason string includes a " (relay locked)" suffix when active.

### Deadband

When |error| ≤ deadband_w: P = 0, integral frozen, output unchanged.

---

## Driver: Key Concepts

### 1. AC Mode Management

The Zendure SolarFlow requires the correct AC mode before accepting power limits:
- "Input mode" before sending `inputLimit` (charge)
- "Output mode" before sending `outputLimit` (discharge)

The driver sends the AC mode command **once** on intent change, then waits for the device. It does NOT spam the command every tick — the MQTT integration overwrites the entity with device reports faster than the relay physically switches (10-15 s).

Re-send after 30 s if the intent persists (`AC_MODE_RETRY_S`).

### 2. Power Limits

Power limits (outputLimit, inputLimit) are sent whenever their values change. No gating on device mode confirmation — the device is responsible for applying limits in the correct mode.

### 3. Relay Lockout (adaptive, energy-integrator)

The `RelayStateMachine` gates relay transitions behind an energy integrator (`AdaptiveLockout`).  Each tick accumulates `|power| × dt`; the transition fires when the accumulated energy reaches the configured `relay_lockout_ws` threshold (in W·s).

```
threshold = relay_lockout_ws   (e.g. 10 000 W·s)
```

For sustained constant power the effective lockout is `threshold / |power|`:

| |desired_power| | Lockout (relay_lockout_ws=10000) | Rationale |
| --- | --- | --- |
| 500 W+ | 20 s | High surplus — worth switching |
| 200 W | 50 s | Moderate — wait longer |
| 100 W | 100 s | Marginal — probably not worth the relay wear |
| <cutoff_w | Capped at 600 s (safety timeout) | Negligible — idle instead |

IDLE transitions use an accumulated-time lockout (`idle_lockout_s`): time is only counted during ticks where IDLE is the target, so oscillations between non-current states accumulate IDLE time across interruptions.

**Independent accumulators:** Each non-current state tracks its own transition progress independently. Switching between two non-current targets (e.g. oscillating between IDLE and DISCHARGE while in CHARGING) does **not** reset the other's accumulator. This ensures that even with oscillating desired power, the state machine eventually transitions — whichever accumulator reaches its threshold first wins. All accumulators are reset only when the desired state matches the current state (stable) or when an actual transition fires.

A 600 s safety timeout (`RELAY_SAFETY_TIMEOUT_S`) forces the transition if the integrator hasn't reached threshold (e.g. very low power).

During lockout, power is **clamped** to the current direction's minimum active power (`min_active_power_w`, default 25 W), keeping the device responsive while preventing relay chatter.

### 4. Rounding and Suppression

- Power rounded to 5 W steps (`ROUNDING_STEP_W`)
- Redundant sends suppressed (only send when values change)

---

## Protection Mechanisms

### 1. Emergency (Feed-in > 800 W)

Direct curtailment: output reduced by (excess + 50 W margin). Integral back-calculated to the forced value so the PI resumes smoothly.

### 2. Direction Lockout (adaptive, direction-aware)

... gates transitions behind an energy integrator (`AdaptiveLockout`): each tick accumulates `|power| × dt` until the configured `relay_lockout_ws` threshold (in W·s) is reached. High power → short lockout; low power → long lockout. Each non-current state tracks its own accumulator independently — switching between two non-current targets does not reset the other's progress. A 600 s safety timeout prevents infinite lockout. During lockout: power clamped to the current direction's minimum active power (`min_active_power_w`), keeping the device responsive.

### 3. SOC Protection

| Condition | Effect |
| --- | --- |
| SOC ≤ effective min_soc | Discharge blocked |
| SOC ≥ max_soc | Charge blocked |

**Dynamic min SOC (forecast-based):** When `dynamic_min_soc_entity` is configured (default: `sensor.zfi_dynamic_min_soc`, published by the PV Forecast Manager app), the controller reads it every cycle. The effective min SOC is clamped to `[Config.min_soc_pct, Config.max_soc_pct]` so it can never violate the hard limits from `apps.yaml`. When the entity is unavailable, the static `min_soc_pct` applies.

The PV Forecast Manager (`pv_forecast_manager.py`) evaluates at 06:00, 15:00, and 20:00. If the PV forecast for tomorrow (summed across all configured Forecast.Solar entities) is below 1.5 kWh:
- **06:00 (morning)**: sets `sensor.zfi_dynamic_min_soc` to **50%** — prevents daytime discharge when PV won't recharge.
- **15:00 / 20:00 (evening)**: sets it to **30%** — preserves a night buffer.

Otherwise resets to 10%. This prevents pointless deep discharge on days when PV won't deliver enough to recharge.

### 4. Grid-Charge Protection

Three layers:
1. **Mode gate**: surplus ≤ 0 → charging blocked
2. **Surplus clamp**: charge capped at available surplus
3. **Asymmetric target**: target = 0 prevents PI from requesting grid power

### 5. Controller Stale-Check (driver)

If the controller app dies but AppDaemon continues running, the driver
would perpetually act on a stale `sensor.zfi_desired_power`. The driver
checks the entity's `last_updated` timestamp on every tick:

- **Fresh** (age ≤ `controller_stale_s`, default 30 s): normal operation.
- **Stale** (age > threshold): driver publishes its own output sensors as
  **0 W** (`device_output`, `discharge_limit`, `charge_limit`) and sets
  `sensor.zfi_controller_stale = true`.  The driver does **not** overwrite
  `sensor.zfi_desired_power` (that is the controller's entity).
  The watchdog handles sending 0 W to the physical device.
- Logs a WARNING on the stale → fresh transition and vice versa.
- Set `controller_stale_s: 0` to disable the check.

### 6. MQTT Heartbeat Publishing

Both the controller and driver can publish an ISO-8601 UTC timestamp to
a configurable MQTT topic on every tick.  This enables external
monitoring by an independent device (e.g. an ESP32 fallback controller)
that watches the heartbeat topic and takes action when it goes stale.

- Controller: `heartbeat_mqtt_topic: zfi/heartbeat/controller`
- Driver: `heartbeat_mqtt_topic: zfi/heartbeat/driver`
- Omit or leave empty to disable.

### 7. Watchdog Heartbeat Monitoring + Safe State

The MQTT Watchdog app (`solarflow_mqtt_watchdog.py`) can optionally
monitor a list of HA entities for staleness.  When any entity's
`last_updated` exceeds `heartbeat_stale_s` (default 60 s):

- A HA **persistent notification** is created (one per entity).
- If `output_limit_entity` and `input_limit_entity` are configured,
  both are set to **0 W** (device safe state).  This catches the case
  where the driver itself has died — the driver's own stale-check
  only covers a dead controller, not a dead driver.
- Safe state is sent once per stale transition.  When **all** entities
  recover, safe state is released and normal operation resumes.
- When the entity recovers, the notification is dismissed.
- Entities to monitor are configured via `heartbeat_entities` (e.g.
  `sensor.zfi_desired_power`, `sensor.zfi_device_output`).

A **startup grace period** (`heartbeat_grace_s`, default 120 s)
suppresses all stale checks after the watchdog starts, giving the
controller, driver, and device time to boot without triggering
spurious notifications or safe-state enforcement.

The future ESP32 watchdog runs independently of HA — it subscribes to
the MQTT heartbeat topics and only needs to **notify** the user.
Safe-state enforcement lives in the HA watchdog because it has direct
access to device entities via HA services.

### 8. ESP Watchdog (`zfi_watchdog.yaml`)

An ESPHome package (`src/zfi_watchdog.yaml`) that runs on an existing
ESP8266 alongside other sensors (e.g. `heizung.yaml`).  Included via
ESPHome `packages:` directive for clean separation.

**How it works:**

1. Connects to the MQTT broker and subscribes to
   `zfi/heartbeat/controller` and `zfi/heartbeat/driver`.
2. Tracks the last-received timestamp for each heartbeat.
3. Every 30 s, checks if either heartbeat is older than 2 minutes.
4. If stale: sends HTTP POST to the SolarFlow's **local HTTP API**
   (`http://{ip}/properties/write`) with
   `{"sn":"...","properties":{"outputLimit":0,"inputLimit":0}}`.
5. When heartbeats resume: stops sending safe-state.

**Independence:** The safe-state path is HA → MQTT broker → ESP → HTTP →
SolarFlow.  If HA/AppDaemon die, only the broker and the SolarFlow
need to be reachable on the LAN.  The HTTP API
(`/properties/write`, `/properties/report`) is a direct device
interface discovered in the Zendure HA integration source
(`ZendureZenSdk.httpPost`).

**Grace period:** 3 minutes after boot, all stale checks are
suppressed to allow the controller and driver to start up.

**HA entities published (via native API):**
- `binary_sensor.heizung_zfi_watchdog_safe_state` — `problem` when
  safe-state is active
- `sensor.heizung_zfi_controller_heartbeat_age` — seconds since last
  controller heartbeat
- `sensor.heizung_zfi_driver_heartbeat_age` — seconds since last
  driver heartbeat

**Configuration:** Add to ESPHome secrets:
```yaml
mqtt_broker: "192.168.x.x"
mqtt_user: "your_mqtt_user"
mqtt_pass: "your_mqtt_password"
solarflow_ip: "192.168.x.x"
solarflow_serial: "HECxxxxxxxxxx"
```

---

## Flowcharts

### Main Control Loop (Controller)

```mermaid
flowchart TD
    A([Tick: every 5s]) --> B{Sensors OK?}
    B -- No --> Z([Skip])
    B -- Yes --> C[Read grid, SOC, battery_power]

    C --> D["Estimate surplus<br>= -battery_power_w - grid"]

    D --> E{Feed-in > 800W?}
    E -- Yes --> F["EMERGENCY<br>Curtail, reset integral"]
    F --> PUB

    E -- No --> G[Update operating mode<br>Schmitt trigger on surplus]
    G --> H[Select target<br>CHARGING→0W / DISCHARGING→30W]
    H --> I["error = grid - target"]

    I --> J{"|error| ≤ deadband?"}
    J -- Yes --> K3["Freeze PI, keep output"]
    J -- No --> L["PI step: P + I"]

    K3 --> FF
    L --> FF["Feed-forward<br>sum(sign × gain × delta)<br>deadband on total"]

    FF --> N{"|raw| > 0?<br>(discharge)"}
    N -- Yes --> O{SOC ≤ min?}
    O -- Yes --> P["Idle: SOC too low"]
    O -- No --> CL

    N -- No --> R{surplus ≤ 0?}
    R -- Yes --> S["Idle: No surplus"]
    R -- No --> T{SOC ≥ max?}
    T -- Yes --> U["Idle: SOC full"]
    T -- No --> CL

    CL["Clamp to limits<br>Cap charge at surplus"]

    CL --> PUB
    P --> PUB
    S --> PUB
    U --> PUB

    PUB["Publish sensor.zfi_desired_power<br>+ all sensor.zfi_* states"]
```

### Driver Loop

```mermaid
flowchart TD
    A([Tick: event-driven + watchdog]) --> B{desired_power<br>available?}
    B -- No --> Z([Skip])
    B -- Yes --> STALE{Controller stale?<br>last_updated > 30s}
    STALE -- Yes --> SAFE["Safe sensors: 0W out, limits<br>Publish controller_stale=true"]
    SAFE --> Z2([Done])
    STALE -- No --> SM{SM enabled?}
    SM -- Yes --> SMU["RelayStateMachine.update()<br>clamps power to current state"]
    SM -- No --> PASS["allowed = desired"]
    SMU --> RND
    PASS --> RND

    RND["Round to 5W steps<br>discharge_w, charge_w"]

    RND --> DRY{dry_run?}
    DRY -- Yes --> PUB
    DRY -- No --> SEND

    SEND["_send_limits()"]
    SEND --> ACM{"SM state?"}
    ACM -- DISCHARGING --> AO["Set AC mode: Output<br>(send-once, retry 30s)"]
    ACM -- CHARGING --> AI["Set AC mode: Input<br>(send-once, retry 30s)"]
    ACM -- IDLE --> SKIP_AC["No AC mode change"]

    AO --> LIM
    AI --> LIM
    SKIP_AC --> LIM

    LIM["Send outputLimit / inputLimit<br>(only when value changed)"]

    LIM --> PUB
    PUB["Publish sensor.zfi_device_output<br>discharge_limit, charge_limit, relay"]
```

### Mode Selection

```mermaid
flowchart LR
    A[Estimate surplus] --> B{Current mode?}

    B -- DISCHARGING --> C{"surplus > +50W?"}
    C -- Yes --> CP{"Pending for<br>charge_confirm_s?"}
    CP -- "Not yet" --> PND["Start/continue<br>charge pending"]
    CP -- "Yes, confirmed" --> D["Switch to CHARGING"]
    C -- No --> CANCEL["Cancel pending<br>if any"]
    CANCEL --> E["Stay DISCHARGING"]

    B -- CHARGING --> F{"surplus < -50W?"}
    F -- Yes --> G["Switch to DISCHARGING<br>(instant)"]
    F -- No --> H["Stay CHARGING"]

    D --> T["target = 0W"]
    E --> T2["target = 30W"]
    PND --> T2
    G --> T2
    H --> T

    style D fill:#66cc66,color:#000
    style G fill:#6699ff,color:#000
    style PND fill:#ffcc66,color:#000
```

---

## Published HA Sensors

### Controller sensors (always published)

| Entity | Type | Unit | Description |
| --- | --- | --- | --- |
| `zfi_desired_power` | number | W | **Main output**: signed desired power (+discharge, -charge) |
| `zfi_mode` | text | — | Operating regime: `charging` or `discharging` |

### Controller sensors (debug only)

Published only when `debug: true` in the controller config.

| Entity | Type | Unit | Description |
| --- | --- | --- | --- |
| `zfi_surplus` | number | W | Estimated PV surplus |
| `zfi_battery_power` | number | W | Actual battery power (+discharge, -charge) |
| `zfi_target` | number | W | Active PI target (0 or 30) |
| `zfi_error` | number | W | Regulation error |
| `zfi_p_term` | number | W | Proportional component |
| `zfi_i_term` | number | W | Integral component |
| `zfi_ff` | number | W | Feed-forward component (post-deadband) |
| `zfi_ff_pv_raw` | number | W | Live PV sensor reading |
| `zfi_ff_pv_ema` | number | W | PV EMA filter state (τ = 30 s) |
| `zfi_ff_pv_contrib` | number | W | PV contribution before deadband |
| `zfi_ff_others_contrib` | number | W | All non-PV load sources summed, before deadband |
| `zfi_integral` | number | W | Integral accumulator |
| `zfi_reason` | text | — | Decision reason |

### Driver sensors (always published)

| Entity | Type | Unit | Description |
| --- | --- | --- | --- |
| `zfi_device_output` | number | W | Signed power sent to device |
| `zfi_discharge_limit` | number | W | outputLimit sent (≥ 0) |
| `zfi_charge_limit` | number | W | inputLimit sent (≥ 0) |
| `zfi_relay` | text | — | Physical relay state from AC mode entity |
| `zfi_relay_locked` | text | — | `true` when SM is clamping output or relay is physically switching (8 s holdoff) |
| `zfi_controller_stale` | text | — | `true` when the controller's desired-power sensor hasn't updated within `controller_stale_s` — driver publishes safe sensors (0 W) |

### Driver sensors (debug only)

Published only when `debug: true` in the driver config.

| Entity | Type | Unit | Description |
| --- | --- | --- | --- |
| `zfi_relay_sm_state` | text | — | Current SM state (idle/charging/discharging) |
| `zfi_relay_sm_pending` | text | — | Pending transition target (or "none") |
| `zfi_relay_sm_lockout_pct` | number | % | Unified lockout progress for active transition |
| `zfi_relay_sm_accumulated_ws` | number | W·s | Accumulated energy toward transition threshold |
| `zfi_relay_sm_threshold_ws` | number | W·s | Energy threshold required for transition |
| `zfi_relay_sm_charge_pct` | number | % | Charge transition progress |
| `zfi_relay_sm_discharge_pct` | number | % | Discharge transition progress |
| `zfi_relay_sm_idle_pct` | number | % | Idle transition progress |

---

## Lovelace Dashboards

This repository includes pre-built Lovelace dashboard YAML files for different purposes.

**For detailed setup, signal descriptions, and troubleshooting, see [DASHBOARDS.md](DASHBOARDS.md).**

### Quick Reference

| Dashboard | File | Purpose | Requirements |
|-----------|------|---------|--------------|
| **Operations** | `config/lovelace_zfi_operations.yaml` | Main control loop view: grid power, surplus, battery, desired power, device commands | Always available |
| **PI Debug** | `config/lovelace_zfi_pi_debug.yaml` | PI controller internals: P/I terms, integral, target, error, deadband, mode changes | `debug: true` |
| **Relay State Machine** | `config/lovelace_zfi_relay_sm_debug.yaml` | Relay transitions: current state, pending target, lockout progress, energy accumulation | `debug: true` |
| **Feed-Forward Debug** | `config/lovelace_ff_debug.yaml` | Feed-forward filtering: PV EMA, contribution sources, forecast min SOC | `debug: true` (if FF enabled) |

### Setup Instructions

1. **In Home Assistant**, navigate to **Dashboards** → Create a new dashboard (or edit an existing one)
2. In **raw YAML edit mode**, paste the entire contents of one of the dashboard files above
3. Replace sensor entity IDs (e.g., `sensor.zfi_*`) if you customized your sensor prefix
4. Save and view

For detailed signal descriptions, tuning guidance, and troubleshooting, see [DASHBOARDS.md](DASHBOARDS.md).

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
| `smart_mode_entity` | Driver | `switch.*_smartmode` (optional — enables RAM-only writes) |
| `forecast_entities` | Forecast | `sensor.energy_production_tomorrow*` |
| `watch_entity` | Watchdog | `sensor.*_electriclevel` |

Optional switches (create as HA helpers → Toggle):

| apps.yaml key | HA entity |
| --- | --- |
| `charge_switch` | `input_boolean.zfi_charge_enabled` |
| `discharge_switch` | `input_boolean.zfi_discharge_enabled` |

Add secrets to your AppDaemon `secrets.yaml` (see `config/secrets.yaml.example`).

### 4. Runtime Files

State files (PI integral, relay SM state, relay switch count) are stored in `zero_feed_in/run/`.  This directory is auto-created on first start and git-ignored, so `git pull` never overwrites runtime state.

### 5. Start in Dry Run

Set `dry_run: true` (the YAML anchor at the top of `apps.yaml` applies to all apps). Set `debug: true` to publish internal sensors for the debug dashboards. Monitor via AppDaemon log and `sensor.zfi_*` entities.

### 6. Go Live

Set `dry_run: false`. Start with `max_output: 200`. Once stable, set `debug: false` to reduce HA sensor churn.

### 7. Updating

```bash
cd /root/addon_configs/a0d7b954_appdaemon/apps/zero_feed_in
git pull origin main
```

AppDaemon detects file changes and reloads automatically.  State in `run/` is preserved across updates.

---

## Example Scenarios

### 1. Sunny day — charging from surplus

```
PV=1200W, house=500W, battery charging 400W
battery_sensor = -400 (charging → negative per convention)
battery_power_w = -400 (signed mode, no transformation)
grid = house - PV + charge = 500 - 1200 + 400 = -300W (feeding in)

surplus = -battery_power_w - grid = -(-400) - (-300) = 400 + 300 = 700W
mode: 700 > 50 → CHARGING, target = 0W
error = -300 - 0 = -300W
PI increases charge
clamp: cap at surplus=700 → charge up to 700W
→ desired_power ≈ -650W
→ driver: inputLimit=650W, outputLimit=0W
```

### 2. Cloud passes — reduce charging

```
PV drops to 600W, house=500W, battery charging 650W
battery_sensor = -650, battery_power_w = -650
grid = 500 - 600 + 650 = +550W (drawing from grid!)

surplus = -(-650) - 550 = 650 - 550 = 100W
mode: 100 > -50 → stays CHARGING (hysteresis!), target = 0W
error = 550 - 0 = 550
PI reduces charge significantly
clamp: cap at surplus=100W
→ desired_power ≈ -100W
→ driver: inputLimit=100W
```

### 3. Evening — discharge to cover load

```
PV=0W, house=400W, battery idle
battery_sensor = 0, battery_power_w = 0
grid = +400W

surplus = 0 - 400 = -400W
mode: -400 < -50 → DISCHARGING, target = 30W
error = 400 - 30 = 370
PI increases discharge
→ desired_power ≈ +370W
→ driver: outputLimit=370W, inputLimit=0W
```

### 4. Battery discharging, load drops

```
PV=0W, house=100W, battery discharging 300W
battery_sensor = +300, battery_power_w = +300
grid = 100 - 0 - 300 = -200W (feeding in!)

surplus = -(+300) - (-200) = -300 + 200 = -100W
mode: stays DISCHARGING, PI reduces output
→ desired_power drops toward 100W
```

### 5. Emergency — feed-in exceeds 800W

```
grid = -1100W (1100W flowing to grid)
feed_in = 1100 > 800 → EMERGENCY
excess = 1100 - 800 = 300
forced = current_output - 300 - 50 (safety margin)
integral → forced (back-calculated)
```

### 6. Battery full — surplus goes to grid

```
SOC=85%, surplus=500W, mode=CHARGING
PI wants to charge → guard: SOC ≥ max → idle
→ desired_power = 0W
→ Surplus flows to grid (unavoidable)
```

| Problem | Action |
| --- | --- |
| Output oscillates | Reduce Kp for the relevant quadrant, increase deadband |
| Persistent offset | Increase Ki for the relevant quadrant |
| Sluggish on load changes | Increase Kp for the relevant quadrant, reduce interval |
| Relay clicks frequently | Increase direction_lockout (30+ s), lower adaptive_lockout_ref_w (100 W), increase deadband |
| Mode flaps | Increase mode_hysteresis (80–100 W), increase charge_confirm (25 s) |
| Relay switches on marginal surplus | Lower adaptive_lockout_ref_w, increase adaptive_lockout_max_mult |
| Charges from grid briefly | Check surplus in logs, increase hysteresis |

---

## Known Limitations

- **Device response time**: 10–15 s (not 2–4 s as Zendure docs suggest)
- **Flash writes**: `setOutputLimit`/`setInputLimit` write to device flash by
  default.  Mitigated by enabling `smartMode` via `smart_mode_entity` config —
  routes all writes to RAM.  The driver re-enables smartMode automatically
  after device reboot.
- **Single phase**: SolarFlow feeds one phase; three-phase balancing at meter works
- **Single instance only**: no multi-device HEMS support
