# Zero Feed-In — System Architecture

## Status

Draft architecture proposal based on `architecture_requirements.md`.
Each decision is documented as an ADR (Architecture Decision Record)
with context, alternatives, rationale, and consequences. 16 ADRs total.

---

## Goals

The architecture must:
- Run zero feed-in control reliably as the foundation
- Support incremental development — each phase usable on its own (US-8)
- Allow heterogeneous hardware (US-2) and dynamic system composition (US-3)
- Hide internal complexity from end users while exposing it to developers (US-1)
- Separate concerns by time scale (real-time vs. planning, US-4)
- Enable post-mortem analysis from logs alone (US-7)
- Cleanly separate open-source core from optional premium services (US-6)
- Enable "what-if" simulations against historic data to evaluate hardware changes (US-9)

## Non-Goals

The architecture explicitly does NOT prescribe:
- Which control algorithm to use (direct calculation, PI, MPC...)
- How the optimizer scores options (rules, MILP, ML...)
- What the visualization looks like
- Specific aggregation strategies (proportional, SOC-balanced...)

These are implementation choices within architectural slots.

---

## Architecture Overview

The system is a **federation of single-purpose components** communicating
via a **typed message bus**, deployable across **three tiers** with
clear isolation boundaries.

```
┌────────────────────────────────────────────────────────────────────┐
│  TIER 3 — BROWSER                                                  │
│  Live energy flow visualization, decision transparency,            │
│  historical playback, configuration UI                             │
└──────────────────────────────┬─────────────────────────────────────┘
                               │ HTTPS / WebSocket
┌──────────────────────────────┴─────────────────────────────────────┐
│  TIER 2 — SERVER (optional)                                        │
│                                                                    │
│  Optimizer  Auto-Tuner  Forecasts  Recorder  Simulator  Web Server  │
│      └──────────┴───────────┴──────────┴──────────┘                │
│                          MQTT Bus (Tier 2)                         │
└──────────────────────────────┬─────────────────────────────────────┘
                               │ Bridge (selected topics, mTLS)
┌──────────────────────────────┴─────────────────────────────────────┐
│  TIER 1 — LOCAL (HA host)                                          │
│                                                                    │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │  zfi Add-on (Docker container)                              │   │
│  │                                                             │   │
│  │  ┌────────────────────────────────────────────────────────┐ │   │
│  │  │  Embedded Mosquitto :1884 (zfi-internal bus)           │ │   │
│  │  └─┬──┬──┬───┬───┬───┬───┬───┬─────┬─────┬─────────────┘ │   │
│  │    │  │  │   │   │   │   │   │     │     │                │   │
│  │  ┌─┴┐┌┴─┐┌┴┐ ┌┴──┐┌┴──┐┌┴─┐┌┴──┐┌┴───┐┌┴──┐  ...        │   │
│  │  │GD││BD││PV│ │Det││Ctl││Sup││MUX││Brdg││Log│             │   │
│  │  └──┘└──┘└──┘ └───┘└───┘└──┘└───┘└────┘└───┘              │   │
│  │                                                             │   │
│  │  Drivers · Detectors · Controllers · Supervisor · MUX       │   │
│  │  · Tier-2 Bridge · Logger                                   │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                              ↑                                     │
│           reads HA sensors (WebSocket API),                        │
│           writes HA actuators (REST API),                          │
│           Custom Integration subscribes to :1884                   │
│                              ↓                                     │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │  HA Process                                                 │   │
│  │  Thin Custom Integration: zfi_* entities only               │   │
│  │  (sensor.zfi_mode, sensor.zfi_grid_power, ...)              │   │
│  └─────────────────────────────────────────────────────────────┘   │
└────────────────────────────────────────────────────────────────────┘
```

The fundamental shape: **components publish to topics, components
subscribe to topics**. Topics are the only coupling. A component
can be replaced, removed, or added without touching others, as long
as the topic contracts remain stable.

---

# Architecture Decisions

## ADR-1: Components Communicate via Typed MQTT Bus

### Context

Requirements REQ-1 (hidden internal communication), REQ-2 (dynamic
loading), REQ-3 (developer observability), REQ-30 (graceful
degradation), REQ-31 (component independence) all point toward
a publish/subscribe message bus.

D-1 already decided: separate processes preferred, MQTT as IPC
candidate.

### Decision

All inter-component communication uses MQTT topics with **typed,
schema-validated JSON messages**. The Tier 1 bus runs on a Mosquitto
broker bundled with the zfi Add-on (port 1884 by default — separate
from any user-installed Mosquitto on port 1883). This avoids depending
on the user having HA's MQTT integration installed, and keeps zfi
internal traffic out of the user's existing broker namespace. See
ADR-4 for the deployment context.

Each message contains:
- `timestamp`: ISO 8601
- `source`: component instance ID
- `payload`: typed data (validated against Pydantic schema)
- `causal_context`: optional reference to the message that caused
  this one (for REQ-3a causal tracing)

### Alternatives Considered

**In-process pub/sub** (Python callbacks): Faster, simpler, but
forces all components into the same OS process. A bug in any
component crashes everything. No introspection without bespoke
tooling.

**ROS 2**: Battle-tested, has typed messages, lifecycle nodes,
introspection tools. But: heavyweight DDS middleware, separate
build system (colcon), not Python-native, drastically out of
proportion for this domain.

**HA event bus** (`hass.bus.async_fire`): Built into HA, no extra
infrastructure. But: tied to HA, partially visible to users,
harder to expose across processes/tiers.

**gRPC**: Strong typing via protobuf, efficient binary protocol.
But: each component needs a server, point-to-point not pub/sub
by default, protobuf adds build complexity.

**Plain WebSocket between components**: Web-native, simple. But:
implementing fan-out, persistence, and broker semantics
ourselves duplicates what MQTT provides.

### Rationale

**MQTT was chosen because:**

1. **Lightweight infrastructure**: Mosquitto is a small Docker layer.
   Bundling it with the Add-on means no external dependency, no
   conflict with user setups, and explicit version control.
2. **Pub/sub native**: Matches the architecture pattern exactly.
   No wrapper code needed.
3. **Process-agnostic**: Components in the same Python process,
   different processes, or different machines all use the same
   API. The "where" is a deployment decision.
4. **Built-in introspection**: Any MQTT client (mqtt-explorer, CLI)
   subscribes and sees live traffic. Free debugging tool.
5. **Resilience**: Components that disconnect (crash, restart) and
   reconnect don't need special handling. The broker handles
   buffering for retained messages.

**Pydantic + JSON for messages because:**

1. **Type safety** without protobuf's build complexity
2. **Human-readable** for debugging and replay
3. **Native Python** — no codegen, no IDL files
4. **Schema evolution** — Pydantic supports versioned schemas

### Consequences

**Enables:**
- Components run in any process configuration
- Dynamic add/remove without restarts (subscriber appears, gets messages)
- Free observability via standard MQTT tools
- Cross-tier communication uses the same primitive (extends to Tier 2)
- Test harness can replace bus with in-memory fake
- Independence from user's MQTT setup: zfi works whether or not the
  user has HA MQTT integration installed

**Constrains:**
- All messages must be JSON-serializable (no NumPy arrays without conversion)
- Schema discipline required — ad-hoc dict messages forbidden
- MQTT broker becomes critical infrastructure (if it dies, system stops)
- Some latency overhead vs. in-process calls (<1 ms locally, acceptable
  for our timescales)
- Embedded broker adds a small operational surface (configuration,
  ACL files, port management) — but it is fully controlled by the
  Add-on, not visible to the user

---

## ADR-2: Six Component Types with Defined Contracts

### Context

REQ-4 (layered architecture), REQ-5 (driver abstraction), REQ-6
(pluggable controllers), REQ-7 (situation detectors), REQ-9
(automatic aggregation), REQ-13 (planning vs. real-time separation).

IDEA-3 proposed Driver/Detector/Controller/Supervisor. IDEA-5 added
Aggregator. A Multiplexer is added to handle controller switching
cleanly with sleep/wake lifecycle management.

### Decision

The system has **exactly seven component types**, each with a single
defined responsibility:

| Type | Reads | Writes | Lifetime | Examples |
|---|---|---|---|---|
| **Driver** | Hardware via HA WebSocket/REST API, native MQTT, or HTTP — depending on device | Bus topics; for actuators: HA service calls or device-native protocol | Per device function | GridMeterDriver, BatteryDriver, PVDriver |
| **Aggregator** | N drivers of same type | Bus (looks like one driver) | When N>1 of same type | BatteryFusion |
| **Detector** | Bus, HA entities | Bus (situation topics) | Optional, configurable | StoveDetector, EVDetector |
| **Controller** | Bus (sensors, situations, plans) | Bus (mux input channel) | Multiple registered, one active | NormalController, StoveController |
| **Supervisor** | Bus (situations) | Bus (controller selection) | One per system | RuleBasedSupervisor |
| **Multiplexer** | All controller channels + active_controller | desired_power + lifecycle signals | One per device set | ControllerMux |
| **Service** | Anything | Bus, external systems | Tier 2 mostly | Optimizer, AutoTuner, Recorder, Bridge |

A **Service** is a catch-all for components that don't fit the
real-time control loop: optimizers, recorders, web servers, tuners,
bridges. They follow the same bus rules but have no constraints
on lifetime or activation.

**Note on hardware abstraction**: A driver's "hardware" may be a
physical device with native MQTT (e.g., Zendure direct), an HTTP API
(e.g., REST endpoint), or a HA entity (e.g., grid meter exposed by
HA's EDL21 integration). All three are uniformly Drivers — the
distinction is implementation detail, not architecture. There is no
separate "Bridge" component type.

### Multiplexer — Detail

The Multiplexer is the execution arm of the Supervisor. The
Supervisor decides **which** controller is active; the Multiplexer
**enacts** that decision. This separation keeps the Supervisor
focused on logic and the Multiplexer focused on routing.

```
NormalController  ──→ zfi/1/control/mux/normal/desired_power  ──→ ┐
StoveController   ──→ zfi/1/control/mux/stove/desired_power   ──→ MUX ──→ zfi/1/control/desired_power
EVController      ──→ zfi/1/control/mux/ev/desired_power      ──→ ┘           (Driver reads here)
                                          ↑
                         zfi/1/control/active_controller
                               (Supervisor writes here)
```

**Controllers always publish to their own channel**, regardless
of whether they are active. They never know which channel the MUX
is routing. This follows the Open/Closed principle: adding a new
controller requires no changes to existing controllers or the
Supervisor.

**Sleep/Wake protocol:**

When the active channel changes, the MUX sends lifecycle signals
to all registered controllers:

```python
@dataclass
class LifecycleSignal:
    target: str          # controller name
    state: Literal["ACTIVE", "SLEEP"]
    wake_seed: WakeSeed | None  # only when state = ACTIVE

@dataclass
class WakeSeed:
    last_desired_power: float  # last value routed by MUX
    timestamp: datetime
```

A **sleeping controller**:
- Freezes internal state (no integral accumulation, no muting timer)
- Unsubscribes from sensor topics (no MQTT traffic, no CPU)
- Continues to publish to its own MUX channel at low rate (heartbeat,
  optional) or stops publishing entirely
- Does not process grid sensor updates

A **waking controller**:
- Receives `WakeSeed` with the last routed desired_power
- Initializes `last_sent = wake_seed.last_desired_power`
- Re-subscribes to sensor topics
- Begins normal operation from a known baseline (not from 0)

This ensures the hard switch (D-3) has minimal overshoot: the
waking controller starts from the last commanded value, not
from zero.

**Controller lifecycle states:**

```
UNCONFIGURED ──→ INACTIVE ──→ SLEEP ↔ ACTIVE ──→ INACTIVE ──→ FINALIZED
                                  ↑         ↑
                           MUX sends SLEEP/ACTIVE + optional WakeSeed
```

SLEEP is between INACTIVE (not part of the system) and ACTIVE
(currently commanding). A SLEEPING controller is loaded and ready
but consuming no resources.

**MUX degraded behavior:**

- `active_controller` topic absent or stale: hold last known
  active channel, log warning
- Active channel produces nothing for >timeout: publish idle (0W),
  alert Supervisor
- All channels absent: publish idle (0W) immediately

### Alternatives Considered

**Fewer types, more flexibility**: Have a single "Component" base
class. Less restrictive but less self-documenting. Drivers,
detectors, and controllers genuinely have different contracts —
forcing them into one shape obscures intent.

**More types**: Separate "InputDriver" (read-only) and "OutputDriver"
(read-write). Adds ceremony without benefit — the Driver base
class can express both via optional capabilities.

**Hierarchical types**: Make Aggregator a subclass of Driver
(it does present as a driver upstream). Decided against because
the *role* differs — an Aggregator orchestrates other drivers,
which is a meaningfully different responsibility.

### Rationale

Seven types covers all requirements with minimal redundancy:

- **Driver** handles hardware abstraction (REQ-5).
- **Aggregator** is the auto-inserted module for multi-instance
  setups (REQ-9). Separating it from Driver lets controllers
  remain unaware of how many physical devices exist.
- **Detector** isolates situation-recognition logic (REQ-7), keeping
  controllers focused on control.
- **Controller** publishes to its own MUX channel always (REQ-6),
  regardless of whether it is active. No meta-decision about
  its own activation.
- **Supervisor** centralizes the activation logic (REQ-6), deciding
  which controller should be active without routing traffic.
- **Multiplexer** separates the routing concern from the decision
  concern. It routes the active channel and manages controller
  sleep/wake, keeping both Supervisor and Controllers simpler.
- **Service** prevents pollution of the real-time tier with
  long-running infrastructure components (recorder, optimizer)
  that don't fit the controller cycle.

The clear contracts also make testing straightforward: each type
has a typed interface and well-defined topic patterns.

### Consequences

**Enables:**
- New hardware = implement Driver + register
- New situation = implement Detector + register
- New strategy = implement Controller + Supervisor rule
- Heterogeneous setups via Aggregator without controller awareness
- Tier 2 features (optimizer, recorder) follow same patterns as
  Tier 1 components — no architectural special-casing

**Constrains:**
- Some logic doesn't fit cleanly (e.g., a detector that needs to
  command something is forbidden — must be split into Detector +
  Controller). This is a feature, not a bug.
- Adding a seventh type requires careful justification — no
  type proliferation.

---

## ADR-3: Topic Namespace and Message Schemas

### Context

ADR-1 establishes MQTT as the bus. The topic structure determines
how components discover each other and how the system scales.

### Decision

Topic structure follows a strict hierarchy:

```
zfi/<tier>/<category>/<entity>/<aspect>

zfi/1/sensors/grid/power                  # GridMeterDriver publishes
zfi/1/sensors/battery/<id>/state          # BatteryDriver publishes
zfi/1/sensors/battery/aggregate/state     # Aggregator publishes
zfi/1/sensors/pv/power                    # PVDriver publishes
zfi/1/situations/stove                    # StoveDetector publishes
zfi/1/situations/ev_charging              # EVDetector publishes
zfi/1/control/mux/<name>/desired_power    # Each Controller publishes to own channel
zfi/1/control/desired_power               # MUX routes active channel here
zfi/1/control/active_controller           # Supervisor publishes
zfi/1/lifecycle/<component>               # Each component publishes status
zfi/1/lifecycle/<name>                    # MUX publishes SLEEP/ACTIVE signals
zfi/1/log/<component>                     # Each component publishes log records

zfi/2/plans/current                       # Optimizer publishes
zfi/2/parameters/proposed                 # AutoTuner publishes
```

Tier number in the path. Each component publishes to its own
"namespace" — never another component's. Subscribers use wildcards
(`zfi/1/sensors/+/power`) for fan-in patterns.

Message schemas live in a shared Python package (`zfi.messages`)
as Pydantic models. Each topic has exactly one schema. Schema
versioning via a `schema_version` field; consumers reject unknown
versions.

### Alternatives Considered

**Flat topic names** (`zfi.grid_power`): Simpler, but no fan-in
without listing every topic. No way to query "all sensors".

**Per-component topic prefix** (`zfi/grid_meter_1/power`): Couples
topic names to instance IDs, breaks if instances are replaced.

**Topic per message type only** (`zfi/measurement`, `zfi/command`):
Forces consumers to filter by content, defeats MQTT's filtering.

### Rationale

Hierarchical topics with category as the second level:

1. **Discovery**: A new optimizer can subscribe to all `zfi/1/sensors/+`
   without knowing what sensors exist
2. **Tier separation**: `zfi/1/*` vs `zfi/2/*` makes the bridge's
   job (ADR-7) trivial
3. **Stable contracts**: Topic = contract. Components rely on the
   topic, not on which other component publishes there. An
   Aggregator can replace multiple BatteryDrivers transparently.

Pydantic schemas with versioning:

1. **Detect mismatches early**: Producer and consumer using different
   schemas is caught at message validation, not at runtime confusion
2. **Self-documenting**: The schema IS the contract. New developers
   read the message classes
3. **Safe evolution**: Versioning forces consumers to opt into
   compatibility

### Consequences

**Enables:**
- Wildcards for fan-in (`zfi/1/log/+` collects all logs)
- Symmetric structure between tiers
- Tooling can introspect by walking the namespace
- Auto-tuner queries "all sensors" via `zfi/1/sensors/+/+`

**Constrains:**
- Topic names cannot change without coordinated upgrades
- Adding a new aspect requires documenting it
- Pydantic adds a small CPU cost per message (negligible at our rates)

---

## ADR-4: HA Coupling via Add-on + Thin Custom Integration

### Context

REQ-22 (HA UI is insufficient for the full system), REQ-25
(client/server separation), REQ-32 (incremental tier deployment).

HA offers several extension mechanisms:

- **Custom Integration** (`custom_components/`): Python inside the
  HA process. Full API access, Config Flow, Entity Registry. Coupled
  to HA's process lifecycle.
- **Add-on**: Docker container managed by HA Supervisor. Separate
  process, communicates with HA via REST/WebSocket API. Standard
  model for Mosquitto, Zigbee2MQTT, ESPHome.
- **MQTT Discovery**: External program publishes entity config to
  `homeassistant/<type>/<id>/config`. HA creates entities
  automatically. Requires HA's MQTT integration to be installed —
  this cannot be assumed as a given.

MQTT in HA is optional and requires the Mosquitto Add-on plus the
HA MQTT integration to be explicitly set up. It cannot be a
required dependency.

The engine's internal bus (ADR-1) already requires MQTT — but that
is the engine's own Mosquitto instance, separate from any HA MQTT
setup.

### Decision

**Two-part HA coupling: engine as Add-on, entities via a thin
Custom Integration. No MQTT dependency on the HA side.**

**Part 1: Engine as HA Add-on**

The zfi engine runs as a Docker container in HA Supervisor:

- Ships with an **embedded Mosquitto** instance dedicated to the
  zfi internal bus. This is zfi's infrastructure — entirely separate
  from any HA MQTT setup.
- Add-on configuration (`config.yaml` schema) is editable in the
  HA Supervisor UI — this replaces a Config Flow without requiring
  any Python in HA:

```yaml
# Add-on config schema (shown in Supervisor UI)
options:
  grid_power_sensor: sensor.edl21_power
  battery_soc_sensor: sensor.zendure_soc
  battery_output_entity: number.zendure_outputlimit
  target_grid_power: 30
  min_soc: 10
  max_soc: 85
```

- **Reads HA sensors** via the HA WebSocket API (no MQTT needed):

```python
# ha_bridge: reads HA state changes
async def listen(self, entity_id: str, callback):
    await ws.subscribe_entities([entity_id])
    # on state change → publish to zfi/1/sensors/grid/power
```

- **Writes HA actuators** via HA REST API:

```python
await rest.call_service("number", "set_value", {
    "entity_id": "number.zendure_outputlimit",
    "value": 800,
})
```

The engine never touches HA's MQTT integration.

**Part 2: Thin Custom Integration (entities only)**

A minimal Custom Integration in `custom_components/zero_feed_in/`
has one job: create HA entities from bus values so the user sees a
Device in the HA UI. It contains **no control logic, no algorithm,
no Config Flow**.

It connects to the embedded Mosquitto (zfi-internal bus) and
registers entity platforms:

```python
# sensor.py — the entire "logic" is just reading from bus
class ZFIGridPowerSensor(SensorEntity):
    """Displays zfi/1/sensors/grid/power in HA UI."""

    async def async_added_to_hass(self):
        self._unsub = await mqtt_subscribe(
            self.hass,
            "zfi/1/sensors/grid/power",
            self._on_message,
            qos=0,
        )

    def _on_message(self, msg):
        data = json.loads(msg.payload)
        self._attr_native_value = data["power_w"]
        self.async_write_ha_state()
```

The entities exposed to the user (selectively — internal bus
topics are not exposed):

| Entity | Type | Source topic |
|---|---|---|
| ZFI Mode | sensor | `zfi/1/control/active_controller` |
| ZFI Grid Power | sensor | `zfi/1/sensors/grid/power` |
| ZFI Battery SOC | sensor | `zfi/1/sensors/battery/aggregate/state` |
| ZFI Desired Power | sensor | `zfi/1/control/desired_power` |
| ZFI Min SOC | number | `zfi/1/config/min_soc` |
| ZFI Discharge Enabled | switch | `zfi/1/config/discharge_enabled` |

The Custom Integration subscribes to the zfi-internal Mosquitto
on a well-known local port (e.g., 1884 — separate from any
user-installed Mosquitto on 1883). No conflict, no dependency on
HA MQTT integration.

**Evolution path:**

When Tier 2 and Tier 3 exist:
- The Custom Integration shrinks further or disappears (Tier 3 is
  the real UI)
- The Add-on bridge role narrows (just sensor reads + actuator writes)
- No architectural change needed — just scope reduction

### Alternatives Considered

**MQTT Discovery only (no Custom Integration)**: Requires HA MQTT
integration to be installed. Cannot be assumed. Breaks for users
who haven't set up MQTT in HA.

**Custom Integration contains everything** (current AppDaemon
model): Engine runs inside HA. Prevents Tier 2 separation, couples
all code changes to HA restart, limits component isolation.

**No Custom Integration, Add-on only (no HA entities)**: Engine
runs standalone, no HA UI whatsoever. Valid for advanced users
but unacceptable UX for the initial release — user has no way to
see what the system is doing from within HA.

**Full Config Flow in Custom Integration**: More polished setup UX
than Add-on config. But: requires maintaining Python UI code in
addition to engine code. Add-on config schema achieves equivalent
result with significantly less code.

**Shared Mosquitto (user's HA MQTT setup)**: Use the user's
existing Mosquitto broker for the internal bus. Simpler, but:
requires HA MQTT to be installed, adds zfi topics to the user's
MQTT broker (topic pollution), makes the internal bus dependent on
user infrastructure. The zfi-embedded Mosquitto is cleaner.

### Rationale

The two-part coupling maps directly to the two different goals:

1. **Engine isolation** (Add-on): The engine runs in its own
   container with its own MQTT broker. It is independent of HA's
   process, HA's MQTT setup, and HA restart cycles.

2. **HA UI presence** (Custom Integration): The user expects to see
   their system in HA's device list. A thin integration that simply
   mirrors bus values into HA entities achieves this with minimal code.

The Custom Integration has so little code (no logic, just subscriptions)
that it rarely needs to change. An engine code change does not require
touching the Custom Integration. HA reload is needed only when the
entity list changes — a rare event.

The embedded Mosquitto runs on a non-default port (1884), so it
does not conflict with a user-installed Mosquitto on 1883. Users
who already have Mosquitto still get a clean separation between
their own MQTT and zfi's internal bus.

### Consequences

**Enables:**
- Engine runs without any HA MQTT setup (no dependency)
- Engine restarts don't require HA restart
- Internal bus topics stay hidden from HA entity list by default
- Engine testable completely without HA (mock WebSocket API)
- Embedded Mosquitto is fully under zfi's control (version, config, ACLs)

**Constrains:**
- Two components to install: Add-on + Custom Integration
  (mitigated: documented as one-step setup)
- Custom Integration must subscribe to embedded Mosquitto on
  non-standard port — documented in integration manifest
- Two separate MQTT instances on the same host
  (negligible resource impact for a lightweight broker)

**Not constrained:**
- HA MQTT integration — not required, not affected
- User's existing Mosquitto on port 1883 — no conflict

---

## ADR-5: Configuration as User-Level + System-Level

### Context

REQ-8 (config-driven assembly), REQ-10 (per-instance parameters),
D-5 (Config Flow for users, YAML for developers).

ADR-4 establishes that the engine runs as an HA Add-on, with a thin
Custom Integration only for entity registration. The Add-on framework
provides its own configuration mechanism via the Add-on Config Schema,
which the HA Supervisor renders as an editable form. This replaces
Config Flow in the original D-5 plan.

### Decision

Configuration has two layers with clear ownership:

**User-level** (Add-on Config Schema → editable in HA Supervisor UI):

The zfi Add-on declares its configuration in `config.yaml` using the
standard HA Add-on schema format. The HA Supervisor renders this as
an editable form with validation, descriptions, and defaults — equivalent
functionality to a Config Flow, but achieved through the Add-on
framework with no Python UI code in HA.

```yaml
# Add-on config.yaml — defines schema and defaults
options:
  hardware:
    grid_meter:
      type: ha_entity
      entity: sensor.smart_meter_power
    batteries:
      - name: "Living Room"
        type: zendure_2400ac
        output_entity: number.zendure_1_outputlimit
        input_entity: number.zendure_1_inputlimit
        soc_entity: sensor.zendure_1_soc
        max_charge: 800
  modes:
    detectors:
      - type: stove
        entity: sensor.kitchen_power
        threshold: 1500
    controllers:
      - normal
      - stove

schema:
  hardware:
    grid_meter:
      type: list(ha_entity|native_mqtt)
      entity: str
    batteries:
      - name: str
        type: list(zendure_2400ac|growatt|ha_battery)
        output_entity: str?
        input_entity: str?
        soc_entity: str
        max_charge: int(0,5000)
  modes:
    detectors:
      - type: list(stove|ev|low_battery)
        entity: str
        threshold: int(0,10000)
    controllers:
      - list(normal|stove|ev_priority)
```

The Supervisor presents this as a structured form. Validation happens
on save. Changes require an Add-on restart (acceptable — config changes
are rare).

**System-level** (YAML in Add-on container, edited by developer):

```yaml
# /app/config/system.yaml — shipped with the Add-on, version-controlled
component_types:
  drivers:
    zendure_2400ac: zfi.drivers.zendure.ZendureSolarFlow2400AC
    growatt: zfi.drivers.growatt.Growatt
    ha_battery: zfi.drivers.ha.HABatteryDriver
    ha_entity: zfi.drivers.ha.HAEntityDriver
  controllers:
    normal: zfi.controllers.normal.NormalController
    stove: zfi.controllers.stove.StoveController
  detectors:
    stove: zfi.detectors.stove.StoveDetector

bus:
  broker: localhost
  port: 1884
  topic_prefix: zfi

defaults:
  controller:
    hysteresis_w: 15
    target_grid_power: 30
```

The Add-on Config Schema only offers types that exist in the system
config. It cannot reference unknown components.

### Alternatives Considered

**Single YAML file** (current AppDaemon style): Works for power
users, hostile to non-developers. Doesn't fit HA's standard
configuration patterns.

**Single Add-on schema, no system YAML**: All component types must
be hardcoded in Python. Adding a new driver requires a code change
visible to users — bad separation.

**Custom Integration with Config Flow**: The thin Custom Integration
from ADR-4 could implement a Config Flow. But this duplicates what
the Add-on Config Schema already provides, and Config Flow code is
harder to maintain than a YAML schema. Reserved for cases where
Config Flow's dynamic capabilities are genuinely needed (none in
Phase 1-6).

**Database (SQLite)**: Configuration is small and rarely changes
— database is overkill. Plain YAML/JSON is more inspectable.

### Rationale

The two-layer split mirrors the user/developer distinction in US-1:

- **System-level YAML** is part of the codebase, version-controlled,
  reviewed in PRs. It defines what the system can do.
- **User-level Add-on Config** is per-installation, lives in the
  HA Supervisor, edited via UI. It defines what the system does.

This means:
- Adding a new driver type = code change + system YAML update in
  the Add-on. Users see it as a new option in the Add-on config form
  after updating the Add-on.
- Reconfiguring battery max_charge = HA Supervisor UI, no code touched.

The Add-on Config Schema is more constrained than a Config Flow but
sufficient for the configuration needs in Phase 1-6. If dynamic forms
become necessary later (e.g., conditional fields based on selections),
the Custom Integration could be extended to provide a Config Flow as
an additional configuration path.

### Consequences

**Enables:**
- Non-developer users configure via the standard HA Add-on UI
- Power users add new component types without breaking UI
- Clear audit trail (system config in git, user config in HA backup)
- No Python UI code to maintain

**Constrains:**
- Two places to look for configuration (mitigated by clear docs and
  the Add-on UI showing only available types)
- System YAML changes require Add-on rebuild and update
- Add-on schema syntax is less expressive than Python Config Flow
  (no dynamic step generation) — acceptable for the foreseeable
  configuration patterns

---

## ADR-6: Module Graph Builder Assembles the System at Startup

### Context

REQ-12 (config-driven startup), REQ-9 (auto-insert aggregators),
REQ-11 (graph visibility).

### Decision

A single component, the **Graph Builder**, runs at startup and
constructs the module graph from configuration:

1. Read user config + system config
2. Resolve component types (string → class)
3. Detect multi-instance cases → insert Aggregators automatically
4. Instantiate components with their per-instance parameters
5. Components register their bus subscriptions on creation
6. Graph Builder publishes `zfi/1/lifecycle/graph` with the
   resulting topology

```python
# Pseudocode
def build(user_cfg, system_cfg, bus):
    components = []
    
    # Drivers
    for hw in user_cfg["hardware"]["batteries"]:
        cls = resolve_type(system_cfg, "drivers", hw["type"])
        driver = cls(name=hw["name"], config=hw["config"], bus=bus)
        components.append(driver)
    
    # Auto-aggregator if N>1
    if len([c for c in components if c.kind == "battery_driver"]) > 1:
        agg = BatteryAggregator(
            instances=[c for c in components if c.kind == "battery_driver"],
            bus=bus,
        )
        components.append(agg)
    
    # ... detectors, controllers, supervisor
    
    return Graph(components)
```

The Graph object is the runtime representation, queryable by
the visualization (REQ-11).

### Alternatives Considered

**Hardcoded wiring**: Each setup is its own Python file. Maximum
flexibility, zero dynamism. Defeats REQ-8 (config-driven).

**Dependency injection framework** (e.g., `dependency-injector`):
Mature, feature-rich. But: adds a learning curve, can be opaque
when graph is complex, may not handle MQTT-based wiring naturally.

**Each component self-registers**: Components find each other via
bus discovery messages. Eliminates central builder. But: makes
startup order non-deterministic, complicates failure handling.

### Rationale

A central builder gives:

1. **Determinism**: Startup order is explicit
2. **Visibility**: The graph is a real object, inspectable and
   loggable
3. **Validation**: Misconfigurations (unknown type, missing entity)
   detected before runtime
4. **Auto-aggregation** (REQ-9) is natural — the builder counts
   instances of each type

It's not a generic DI framework because we have a narrow problem:
a static topology defined by configuration. Generic DI would be
overkill.

### Consequences

**Enables:**
- Single source of truth for the running topology
- Simple failure modes: builder fails → no system, clear error
- Aggregators auto-inserted without user awareness (REQ-9)

**Constrains:**
- Adding components at runtime is harder (must go through builder)
- Graph reconstruction requires reload (acceptable per US-3)

---

## ADR-7: Tiered Deployment with Topic-Level Bridging

### Context

REQ-25 (client/server separation), REQ-26 (open source / commercial
boundary), REQ-32 (incremental tier deployment).

### Decision

Three tiers with explicit communication paths:

```
Tier 3 (Browser)
  ↑ HTTPS / WebSocket
  │
Tier 2 (Server)
  │   Internal: MQTT bus (Tier 2 broker)
  │   External: Bridge component
  ↑ MQTT (selected topics, encrypted)
  │
Tier 1 (Local)
  │   Internal: MQTT bus (Tier 1 broker)
  │   External: Bridge component
  Hardware
```

**Each tier has its own broker.** They are connected by **bridge
components** that forward selected topics in selected directions.

Tier 1 → Tier 2:
- All sensor topics (`zfi/1/sensors/+/+`)
- All log topics (`zfi/1/log/+`)
- Active mode and controller (`zfi/1/control/active_*`)

Tier 2 → Tier 1:
- Plans (`zfi/2/plans/current`)
- Parameter proposals (`zfi/2/parameters/proposed`)

Topics that are NOT bridged stay private to their tier.

**Each tier's components run independently of other tiers.** Tier 1
falls back to defaults (configured via system YAML) if Tier 2 is
unreachable.

### Alternatives Considered

**Single shared broker**: All tiers connect to one broker. Simpler,
but: Tier 2 sees all of Tier 1's internal topics, no isolation.
A misbehaving Tier 2 component could subscribe to anything. Hard
to enforce the open-source/commercial boundary (REQ-26).

**REST API between tiers**: Tier 2 polls Tier 1 endpoints. Loses
the push model that fits real-time data. Adds HTTP server in
Tier 1.

**Direct WebSocket**: Tier 1 connects to Tier 2 via WebSocket,
sends events. Custom protocol design. Reinvents what MQTT bridges
provide.

### Rationale

Topic-level bridging gives **the strongest isolation with the least
new infrastructure**:

1. Mosquitto supports broker-to-broker bridging natively (no custom
   code initially)
2. The bridge configuration explicitly lists which topics cross —
   privacy by default
3. Same protocol both sides — no impedance mismatch
4. Can start with both brokers on the same machine (Phase 1-2)
   and split when Tier 2 moves remote (Phase 4+)

Each tier independent supports REQ-30 (graceful degradation):
Tier 2 offline ≠ Tier 1 broken.

### Consequences

**Enables:**
- Open-source/commercial boundary is enforceable: Tier 2 source
  can be closed, Tier 1 stays open. Topic contracts are the API.
- Tier 2 can be added later without changes to Tier 1 (just
  enable the bridge)
- Tier 2 outages don't affect Tier 1 — bridge silently buffers
  or drops
- Multiple Tier 1 instances could connect to one Tier 2 (multi-home)

**Constrains:**
- Two brokers to operate (mitigated by both being lightweight)
- Bridge config must be explicit — accidentally exposing internal
  topics requires opting them in
- Bridge becomes a critical path for Tier 2 features (handled by
  graceful degradation)

---

## ADR-8: Logging via Bus + Recorder

### Context

REQ-27 (structured post-mortem logs), REQ-28 (queryable), REQ-29
(replay capability). D-2 (logs + replay via simulation).

### Decision

Logging follows the bus pattern:

1. Every component publishes structured log records to
   `zfi/1/log/<component>`. Same JSON+Pydantic schema as
   other messages.
2. A dedicated **Recorder** service (Tier 2 by default, Tier 1
   in Phase 1) subscribes to all log topics and persists to
   a time-series store.
3. Replay = read records from store, publish them back onto a
   test bus. The simulation harness consumes them as if live.

```python
@dataclass
class LogRecord:
    timestamp: datetime
    schema_version: str
    component: str
    level: str  # DEBUG | INFO | WARN | ERROR
    event_type: str  # "control_cycle" | "guard_blocked" | "mode_transition" | ...
    data: dict  # Type-specific payload
    causal_context: dict | None  # Triggering message / event
```

Storage choice (D-6): InfluxDB or SQLite, decided per deployment.
The Recorder abstracts the backend.

### Alternatives Considered

**Standard Python logging to files**: Easy. But: text format,
hard to query, no structure for replay.

**Direct database writes from each component**: Each component
knows about the database. Couples everything. Hard to test.

**HA Recorder**: Already exists, but limited query capability and
not designed for high-frequency structured events. Schema is
locked.

### Rationale

Logs as bus messages mean:
1. **Logging is just another component** — uniform pattern
2. **Recording is opt-in** — disable the Recorder, system still
   works (graceful degradation)
3. **Replay is symmetric to recording** — the same simulator
   that runs synthetic data runs recorded data
4. **The Visualization (REQ-24)** can subscribe to live logs OR
   query historical from the Recorder — same interface

The causal_context field threads through messages so a downstream
log can reference what caused it. This builds the causal chain
required by REQ-3a without special tooling.

### Consequences

**Enables:**
- Post-mortem from logs alone (REQ-27)
- Auto-tuner trains on the same data the visualization shows
  (REQ-19)
- Test harness replays production scenarios deterministically
- Multiple Recorder backends possible (file, InfluxDB, SQLite)
  without component changes

**Constrains:**
- Logging volume grows fast — retention policies matter
- Storage decision deferred (D-6) — Recorder must abstract it
  cleanly to allow swap

---

## ADR-9: Process Model Starts Simple, Scales by Splitting

### Context

D-1 (separate processes preferred, not hard requirement),
REQ-31 (component independence), Raspberry Pi resource constraints.
ADR-4 establishes the Add-on container as the primary deployment
unit on Tier 1.

### Decision

The architecture has **two unavoidable processes from day one** —
HA itself, and the zfi Add-on container. Beyond that, scaling
splits into more processes only when isolation needs justify it.

| Phase | Tier 1 process structure | Notes |
|---|---|---|
| 1 | HA + Add-on (engine + Mosquitto) | 2 containers minimum, engine monolithic inside Add-on |
| 2-4 | Same as Phase 1 | Engine grows internally but stays in one container |
| 5+ | Engine may split into subprocesses | Drivers, Optimizer, etc. as separate workers within the Add-on |
| Tier 2 | Separate server container | Same host as Tier 1 or remote, doesn't affect Tier 1 |

**Within the Add-on container** (Phase 1):
- Mosquitto runs as one process (managed by s6-overlay or similar
  lightweight process supervisor — standard HA Add-on pattern)
- The engine runs as one Python process containing all components
  (controllers, drivers, supervisor, multiplexer)
- Components communicate via the embedded Mosquitto on localhost:1884

**On the HA side** (Phase 1):
- HA itself is one process
- The thin Custom Integration runs inside HA as a few entity
  registrations — no significant CPU footprint

Because all components communicate via MQTT (ADR-1), the process
boundary within the engine is a deployment decision, not an
architectural one. Components don't know whether their bus peers
are in the same Python process or another.

### Alternatives Considered

**Engine in HA's Custom Integration** (no Add-on): One container
fewer. But: locks engine code to HA's process lifecycle, every
engine change triggers HA reload, conflicts with separate-process
preference (D-1) and Tier 2 separation (REQ-25).

**Always N processes (one per component)**: Maximum isolation.
But: Raspberry Pi has 4 cores. 20+ processes is wasteful.
Operational complexity outweighs benefits at this scale.

**Threads instead of subprocess workers**: Lighter than processes.
But: Python's GIL limits CPU parallelism, and one component blocking
the event loop blocks others. Doesn't provide the crash isolation
D-1 wants.

**Engine and Mosquitto as two separate Add-ons**: Cleaner separation.
But: doubles the Add-on installation steps, requires the user to
configure inter-Add-on networking, and Mosquitto in the engine
Add-on can be tuned specifically for zfi's traffic patterns.

### Rationale

Starting at "two containers, monolithic engine" and scaling by
splitting:

1. **Two containers is the natural Add-on shape**. Once we commit
   to the Add-on model (ADR-4), we have HA-process plus Add-on-
   container by default. There is no "one process" baseline.

2. **Engine monolithic is fastest to ship**. Phase 1's primary
   goal is to migrate working logic into the new architecture.
   In-engine boundaries (component-to-component) are bus-abstracted
   already — they can be split later without code changes.

3. **The bus abstracts the boundary**. Whether two components live
   in the same engine process or in two subprocess workers is
   invisible to the components themselves. Splitting later is
   configuration plus a process supervisor change, not a refactor.

4. **Failure isolation matters most where bugs live**. The Zendure
   driver (MQTT, retry, state machine) is high-risk. Optimizer and
   AutoTuner (Phase 5+) run untrusted-ish per-tenant logic and
   benefit from process isolation. Detectors and Controllers are
   usually trivial. Split where it matters.

This is the "stage your isolation" principle: pay the operational
cost where it earns its keep.

### Consequences

**Enables:**
- Phase 1 ships as standard HA Add-on plus thin Custom Integration
- Engine restart doesn't require HA restart, and vice versa
- Add-on updates managed by HA Supervisor (standard ecosystem path)
- Splitting components into subprocesses later requires no component
  code changes — only Add-on internal structure changes
- Tier 2 server is a separate concern, not blocked by Tier 1 process model

**Constrains:**
- Two containers to operate (mitigated: HA Supervisor handles both)
- Components must NEVER share state outside the bus (no shared
  globals, no module-level mutable state) — discipline required
  for future split flexibility
- Test infrastructure must work for both single-process and split
  modes (mitigated by bus abstraction in tests)

---

## ADR-10: Graceful Degradation as a Component Contract

### Context

REQ-30 (graceful degradation), REQ-31 (component independence).
US-8 (incremental development — phases must be working systems).

### Decision

Every component must define its **degraded behavior** when its
inputs are absent or stale:

```python
class Component(ABC):
    @abstractmethod
    def degraded_behavior(self) -> DegradedMode:
        """What does this component do when inputs are missing?
        
        Returns:
            DegradedMode.IDLE: Stop publishing, become inactive
            DegradedMode.DEFAULT: Use configured defaults
            DegradedMode.LAST_KNOWN: Hold last published value
            DegradedMode.SAFE: Publish a known-safe value
        """
```

Specific contracts:

- **Drivers** that lose hardware connection: publish `unavailable`
  status, stop sensor messages. Controllers see `unavailable` and
  pause.
- **Controllers** with no input data: publish `idle` desired_power
  (0W). Driver's safe state is also 0W.
- **Detectors** with no sensor data: publish `unknown` situation.
  Supervisor falls back to default controller.
- **Supervisor** with no detector data: activate default controller.
- **Optimizer** absent: planning topics are silent. Real-time
  controllers use configured defaults (e.g., fixed min_soc).
- **AutoTuner** absent: no parameter proposals. Parameters stay
  at configured values.
- **Recorder** absent: bus messages flow normally, just nothing
  is persisted.
- **Bridge** down: Tier 2 silent. Tier 1 falls back to defaults.

A **Watchdog** service (separate from REQ system, an ESP32 from
the existing improvements list) provides the ultimate safety net
when even the engine fails.

### Alternatives Considered

**Fail-fast everywhere**: Components crash on missing inputs.
Simpler code, but defeats incremental deployment (US-8) and
makes the system fragile.

**Optional everywhere with no contract**: Each component invents
its own degraded behavior. Inconsistent, hard to reason about.

**Centralized fault handler**: A "Fault Manager" component handles
all degradation. Adds central coordination point that can itself
fail.

### Rationale

Graceful degradation is the only way to satisfy US-8 (incremental
deployment): every component must be optional in the sense that
its absence degrades the system, doesn't break it.

Making degraded_behavior part of the component contract ensures
the developer thinks about it. Forgetting to define it is a
design omission visible at code review.

The contract types (IDLE, DEFAULT, LAST_KNOWN, SAFE) cover the
common cases without becoming an essay.

### Consequences

**Enables:**
- System runs with any subset of components
- Phase 1 (no detectors, no optimizer) works
- Each component testable for both normal and degraded paths
- New components added at any time

**Constrains:**
- Every component must explicitly handle missing inputs (no
  KeyError, no None propagation)
- Test suite must cover degraded paths (additional test work,
  but worth it)

---

## ADR-11: Multi-Tenancy via Tenant-Scoped Topic Namespace

### Context

REQ-26 (open source / commercial separation) and the strategic goal
of offering paid server services require the architecture to support
multiple independent users sharing Tier 2 infrastructure.

Each user has their own Tier 1 (local HA host). The question is
whether Tier 2 can be shared across users without data leakage or
cross-tenant interference.

The current ADR-3 topic structure (`zfi/<tier>/...`) has no tenant
dimension — two users publishing to the same broker would collide.

### Decision

The topic namespace is extended with a **tenant dimension**:

```
Tier 1 (local, unchanged):
  zfi/<tier>/<category>/<entity>/<aspect>
  e.g.: zfi/1/sensors/grid/power

Tier 2 (shared, tenant-scoped):
  zfi/<tenant>/<tier>/<category>/<entity>/<aspect>
  e.g.: zfi/abc123/1/sensors/grid/power
         zfi/xyz789/1/sensors/grid/power
```

**Tier 1 is unchanged.** Components within a user's local system
continue to use the unscoped topic structure. The tenant dimension
is added exclusively at the bridge — on ingress to Tier 2 and
stripped on egress back to Tier 1:

```python
# Bridge Tier 1 → Tier 2 (adds tenant)
local_topic  = "zfi/1/sensors/grid/power"
remote_topic = f"zfi/{tenant_id}/1/sensors/grid/power"

# Bridge Tier 2 → Tier 1 (strips tenant)
remote_topic = "zfi/abc123/2/plans/current"
local_topic  = "zfi/2/plans/current"
```

**Broker-level ACLs enforce isolation.** Each tenant authenticates
with tenant-specific MQTT credentials. Mosquitto ACL rules constrain
each tenant to their own namespace:

```
# Mosquitto ACL
user tenant_abc123
topic readwrite zfi/abc123/#
topic deny zfi/+/#
```

A tenant can neither read nor write another tenant's topics, even
if they know the topic name.

**The Recorder is tenant-aware.** Data is partitioned by tenant —
either separate databases or a tenant tag in a shared schema.
No query can return data across tenants.

**Optimizer and AutoTuner** are instantiated per tenant. On a shared
server they run as concurrent instances, each scoped to their
tenant's topics and data.

### Deployment Models

Two deployment variants enabled by this decision:

**Self-hosted (Modell A):**
User runs their own Tier 2 instance. Single-tenant, no ACLs needed.
Current architecture + tenant dimension is optional overhead they
can ignore. `tenant_id` is just a constant in their bridge config.

**Managed service (Modell B):**
Operator runs shared Tier 2. Each customer gets a `tenant_id`
(UUID), MQTT credentials, and an isolated namespace. One Mosquitto
broker, one InfluxDB cluster, one web server — N tenants.

The open-source/commercial boundary (REQ-26) maps cleanly:
- Tier 1 code: open source, user owns and runs it
- Tier 2 code: operator's choice (open or closed), user connects to it
- Revenue model: subscription for managed Tier 2 (optimization,
  auto-tuning, visualization) while Tier 1 (zero feed-in control)
  remains permanently free

### Alternatives Considered

**Separate Tier 2 per user (always)**: Zero architectural change.
But: linear cost scaling makes managed service uneconomical. N users
= N servers.

**Tenant as MQTT username only (no topic scoping)**: Use separate
MQTT credentials but shared topic structure, filter by auth context.
Leaks topic existence across tenants. Non-standard Mosquitto
behavior required.

**Separate broker per tenant on shared hardware**: Per-tenant
Mosquitto instance. Full isolation, but port allocation complexity
and 10+ processes for 10 users is operationally heavy.

**API gateway between Tier 1 and Tier 2**: Replace direct MQTT
bridging with an HTTP/WebSocket API. Tenant routing at the gateway.
Adds complexity and loses the simplicity of MQTT for data streaming.

### Rationale

Topic scoping at the bridge is the **minimal viable change**:

1. **Tier 1 is entirely unaffected** — no changes to local
   components, no code shipped to users
2. **Mosquitto supports this natively** — ACL files are built-in,
   no custom plugin
3. **One codebase, two deployment modes** — the bridge config
   determines tenant_id; for self-hosted it's a constant, for
   managed it's a per-user value
4. **Additive change** — ADR-3 gains one segment. Existing code
   that constructs topics adds one prefix variable. Nothing breaks.
5. **Revenue boundary is explicit** — the tenant_id is the contract
   between user (free local tier) and operator (paid cloud tier).
   A user who stops paying simply doesn't get their tenant's topics
   forwarded.

The alternative of keeping Tier 2 per-user forever is viable for
a hobbyist project but forecloses the commercial option. Given that
commercial Tier 2 is an explicit goal (US-6, REQ-26), the small
additional complexity is justified.

### Consequences

**Enables:**
- Managed service: one Tier 2 installation serves N paying users
- Revenue model: Tier 1 (zero feed-in) is free forever; Tier 2
  (optimization, visualization, auto-tuning) is the commercial product
- Privacy by default: ACLs enforce isolation without application code
- Self-hosted users are not affected — single-tenant deployment
  ignores the tenant dimension

**Constrains:**
- Tenant ID must be provisioned before first bridge connection
  (small onboarding step for managed service)
- Bridge configuration gains one required field (`tenant_id`)
- Recorder and Optimizer must never mix tenant data — this must
  be tested explicitly
- Schema evolution must preserve backward compatibility per tenant
  independently (one tenant can upgrade before another)

**Does not constrain:**
- Tier 1 code — zero changes to local components
- Topic structure within a tenant's namespace — same as single-tenant
- Self-hosted deployment model — opt-in complexity

---

## ADR-12: Authentication and Authorization

### Context

The architecture spans three tiers, multiple processes per tier
(ADR-9), shared infrastructure (ADR-11), and distinct trust levels:
local user (high trust), service operator (medium trust), external
attackers (zero trust). Without authentication, any actor with
network access to the embedded Mosquitto broker can publish to the
bus (`zfi/1/control/desired_power`) and command the battery. Without
authorization, a tenant could read another tenant's data.

ADR-4 establishes that the Tier 1 bus runs on a Mosquitto bundled
with the zfi Add-on (port 1884), distinct from any user-installed
Mosquitto. Auth scope below refers to this embedded broker.

### Decision

Authentication and authorization are layered per trust boundary:

**Tier 1 internal bus (zfi-embedded Mosquitto on port 1884):**
- The broker requires username + password for all connections,
  including those from the engine and the thin Custom Integration
- Each component instance has its own credential pair
- Topic-level ACLs limit each component to its declared
  publish/subscribe namespace (e.g., GridMeterDriver can only
  publish to `zfi/1/sensors/grid/+`)
- Credentials are generated at Add-on first start and stored in
  the Add-on's persistent volume (`/data/credentials.json`)
- The Custom Integration receives its credentials via a small
  bootstrap mechanism (e.g., environment variable or shared file
  in `/share/zfi/`) — documented in the Add-on installation steps

**Bridge to Tier 2 (Internet-facing):**
- Mutual TLS (mTLS): bridge presents client certificate, broker
  presents server certificate
- Tenant credentials in addition to mTLS (defense in depth)
- Credentials are tenant-specific and provisioned during
  managed-service onboarding
- For self-hosted Tier 2, mTLS optional; for managed Tier 2, mTLS
  required

**Tier 2 internal:**
- Same as Tier 1: per-component credentials, ACL-enforced topic
  scoping
- Cross-tenant ACLs prevent any tenant principal from reading
  topics outside `zfi/<their_tenant>/+`

**Tier 3 (Browser → Tier 2 web server):**
- OAuth2 with PKCE for browser flows
- JWT tokens with short expiry (1 hour) and refresh tokens
- CORS restricted to known origins
- WebSocket connections authenticate via JWT in the upgrade request

**Component identity:**
Every message on the bus carries a `source` field (already in ADR-1).
The broker's auth context binds the source to the connecting client.
Forging a source field is detectable: the broker's authenticated
client name must match the message's source.

### Alternatives Considered

**No authentication on the embedded broker (localhost-only)**:
Bind Mosquitto to localhost only, skip auth. But: the Add-on
container's network namespace is shared with other Add-ons by
default, and the threat model includes a malicious or compromised
Add-on on the same host. ACLs without auth provide no isolation.

**Single shared credential across components**: Simpler operationally.
But: if any component is compromised (e.g., a buggy detector reads
a credential file), the attacker gets full bus access. Per-component
credentials limit blast radius.

**Pure JWT throughout (no MQTT auth)**: JWT in every message
payload. Avoids managing MQTT credentials. But: validating JWT
on every message adds CPU cost, and Mosquitto's native auth is
faster and battle-tested.

**API gateway in front of Tier 2 broker**: All connections go
through a custom gateway that checks tokens. Adds a SPOF and
custom code. Mosquitto's TLS+ACL is sufficient for the protocol.

### Rationale

Layered authentication matches the trust boundary structure:

- **Within a tier**: lightweight authentication (broker-managed
  credentials), focus on ACL enforcement
- **Across tiers**: heavyweight authentication (mTLS), focus on
  preventing impersonation
- **Browser-facing**: standard web auth (OAuth2/JWT), focus on
  user identity

Per-component credentials at the bus level mean a compromised
component cannot escalate. Topic ACLs make this enforceable —
forgetting to set them is a bug, not a vulnerability surface
that requires runtime checks.

mTLS for the bridge is justified by the threat model: the bridge
crosses the Internet, exposing it to network-level attacks. TLS
alone authenticates the server but not the client; mTLS authenticates
both. For managed Tier 2, this prevents anyone with stolen credentials
from impersonating a tenant — they would also need the client cert.

The Custom Integration is the only consumer outside the Add-on
container. Sharing its credentials via `/share/zfi/` (a HA-managed
shared volume) keeps the trust boundary tight without forcing the
user to copy-paste secrets.

### Consequences

**Enables:**
- Compromise of one component doesn't compromise others
- Tenant impersonation requires both credentials AND client cert
- Audit log can attribute every message to a specific identity
- Security review can be done per-layer independently

**Constrains:**
- Operational complexity: credentials must be generated, stored,
  rotated (see ADR-13)
- Bridge config must include TLS material (cert, key, CA)
- New component installation requires credential provisioning
- Broker performance: ACL evaluation per message (negligible at our
  message rates, but worth measuring)
- Custom Integration needs read access to credentials in
  `/share/zfi/` — documented as part of Add-on installation

**Operational responsibilities:**
- Self-hosted user: credentials are auto-generated by the Add-on
  on first start; user copies displayed Custom Integration credentials
  during setup (or the Add-on writes them to the shared volume
  automatically)
- Managed operator: provision tenant credentials and certs during
  onboarding, distribute securely

---

## ADR-13: Encryption (Transport and At-Rest)

### Context

Data flows over multiple network paths and rests in multiple
storage layers. Encryption protects against passive observers
(network sniffing, disk theft) and is increasingly a regulatory
requirement (GDPR Art. 32) for managed services.

### Decision

**Transport encryption:**

| Path | Encryption |
|---|---|
| Tier 1 bus (localhost) | Optional — recommend TLS for non-loopback |
| Tier 1 ↔ Tier 2 bridge | TLS 1.3 mandatory, mTLS for managed service |
| Tier 2 internal bus | TLS within data center, optional same-host |
| Tier 2 ↔ Tier 3 (browser) | HTTPS mandatory, HSTS enabled |
| Tier 2 ↔ external (forecast APIs) | HTTPS, certificate validation |

**Encryption at rest:**

| Storage | Encryption | Rationale |
|---|---|---|
| Tier 1 config (HA) | HA's own at-rest (filesystem level) | User's responsibility |
| Tier 1 logs | Filesystem encryption recommended | User's responsibility |
| Tier 2 time-series DB | Required for managed service, recommended for self-hosted | Contains sensor history |
| Tier 2 user credentials | Hashed (Argon2id) for passwords, sealed for API tokens | Standard practice |
| Tier 2 tenant data backups | Encrypted with tenant-specific key | Recoverable per-tenant |

**Key management:**

- Tier 1 keys: stored in HA's secrets storage, rotated by user
  (no automatic rotation in Phase 1)
- Bridge mTLS certificates: provisioned at onboarding, valid
  1 year, automatic renewal via short-lived re-issuance API
  (Phase 4+)
- Tier 2 server certificates: standard ACME (Let's Encrypt) or
  enterprise CA, automatic renewal
- Tier 2 database encryption keys: managed by the operator, ideally
  in a KMS (HashiCorp Vault, cloud KMS); for Phase 1, a
  configuration secret is acceptable

**End-to-end encryption (E2EE):**

A future option, not Phase 1. The model: Tier 1 encrypts payloads
with a tenant-controlled key before publishing to the bridge, Tier 2
operator never sees plaintext. Tradeoff: Tier 2 services
(optimization, visualization) cannot operate on encrypted data.

For Phase 1: data is encrypted in transit and at rest, but the
operator has access. This matches the trust model (operator is
medium-trust). E2EE is documented as a future option for users
who don't trust the operator at all (REQ-26 boundary).

### Alternatives Considered

**No encryption (plaintext everywhere)**: Simplest. But: untenable
for managed service (GDPR), unwise for cross-Internet traffic, and
removes a layer of defense.

**E2EE from day one**: Maximum privacy. But: makes Tier 2 services
useless without partial decryption (homomorphic encryption is too
slow for our use case). Defers commercial Tier 2 indefinitely.

**Application-level encryption only (skip TLS)**: Encrypt message
payloads but not the transport. Saves TLS handshake cost. But:
TLS provides server authentication and perfect forward secrecy,
which application-level encryption alone doesn't.

**Use HA's encryption for everything**: HA encrypts entity history.
But: HA Recorder isn't suitable for our needs (REQ-28 query
requirements), and HA's encryption is filesystem-level, not
field-level — doesn't help once the system is unlocked.

### Rationale

The encryption strategy follows the trust boundaries:

- **Localhost** (Tier 1 internal): TLS optional because the
  attacker model on localhost is process-level, not
  network-level. ACLs (ADR-12) provide isolation.
- **Internet-crossing** (bridge, browser): TLS mandatory, no
  exceptions. The cost is negligible, the benefit substantial.
- **Storage**: encrypted in the managed service to satisfy GDPR
  and reduce operator-side risk; recommended for self-hosted
  but not enforced (user's choice).

E2EE is deferred because it conflicts with the Tier 2 commercial
proposition. A user who doesn't want operator access can self-host
Tier 2 (Modell A from ADR-11) — that's the privacy-maximalist path.

mTLS for the bridge (also in ADR-12) goes beyond what most
home-automation systems do. It's justified because the bridge
is the boundary between user-trust and operator-trust, and
because the credential lifecycle is short enough that automation
makes it manageable.

### Consequences

**Enables:**
- GDPR Art. 32 ("appropriate technical measures") satisfied for
  managed service
- Self-hosted users can run unencrypted internally if they want
  performance, encrypted externally
- Defense in depth: even if one layer is broken, others stand
- E2EE addable later without architectural changes (just an
  optional encryption layer in the bridge)

**Constrains:**
- TLS material must be provisioned and renewed
- Performance overhead: TLS adds ~5-10% CPU on commodity hardware
  (negligible for our rates)
- Encrypted storage means key loss = data loss; backup discipline required
- Cannot inspect bridge traffic with standard MQTT tools without
  the right cert (intentional, but harder to debug)

**Operational responsibilities:**
- Self-hosted: TLS setup is an Add-on configuration option;
  documented but optional
- Managed: TLS is required, certificates auto-managed

---

## ADR-14: Multi-Tenant Isolation Beyond ACLs

### Context

ADR-11 establishes tenant-scoped topics with broker ACLs. ADR-12
adds authentication. But ACLs and auth alone don't prevent:

- A tenant publishing 100,000 messages/second, exhausting broker resources
- A tenant's data appearing in another tenant's logs due to a bug
- An optimizer that runs on shared CPU starving other tenants
- Cross-tenant inference via timing or storage patterns

For a managed service, robust isolation is a regulatory and
operational requirement.

### Decision

Multi-tenant isolation has four layers:

**1. Resource quotas per tenant:**

| Resource | Limit | Enforcement |
|---|---|---|
| MQTT messages/second | 100 (configurable) | Mosquitto rate limiter or rate-limiter plugin |
| MQTT message size | 16 KB | Broker config |
| Concurrent connections per tenant | 5 | Broker config |
| Database storage | 1 GB (configurable) | Application-level monitoring with cutoff |
| Optimizer CPU time | 30 sec/run, 5 runs/hour | Service-level scheduling |
| API requests/minute | 60 | Web tier rate limiter |

Limits are documented as the "free tier"; paid tiers have higher
limits.

**2. Code-level tenant context:**

Every Tier 2 service operates on an explicit `TenantContext` object
that scopes all data access:

```python
@dataclass(frozen=True)
class TenantContext:
    tenant_id: str
    db_namespace: str  # e.g., "tenant_abc123" — DB schema or bucket
    topic_prefix: str  # e.g., "zfi/abc123"

class Optimizer:
    async def run(self, ctx: TenantContext):
        # ALL data access uses ctx.db_namespace
        # ALL bus publishes use ctx.topic_prefix
        ...
```

Services cannot bypass `TenantContext`. There is no "global query"
function. A code review can verify tenant scoping by checking that
every database call takes a `TenantContext` parameter.

**3. Process isolation for sensitive workloads:**

Optimizer and AutoTuner run as **per-tenant subprocess workers**,
not as in-process functions. Each worker:
- Is spawned with the tenant context
- Has CPU and memory limits (cgroup or container limits)
- Cannot access other tenant data because it never has the context

A misbehaving optimizer (infinite loop, memory blowup) gets killed
without affecting other tenants.

**4. Audit logging:**

All cross-tenant boundary crossings are logged:
- Tenant authentication events
- Bridge connections established/dropped
- Quota violations
- Admin actions on tenant data
- Data export/deletion (GDPR right of access/erasure)

Audit logs are append-only and stored separately from operational
data. Retention: 1 year minimum.

### Alternatives Considered

**Trust ACLs alone**: Simplest. But: a bug in topic naming, a
misconfigured ACL, or a compiler optimization could leak. Defense
in depth is cheap.

**Container per tenant** (full isolation): Maximum isolation. But:
N tenants = N containers = N×(broker overhead + service overhead).
Doesn't scale economically for the managed service.

**Database per tenant**: Strong isolation, easy backups. But:
N databases harder to operate. A schema migration touches N places.
For Phase 1, a single database with namespacing is acceptable;
Phase 4+ may revisit per the operational reality.

**Application-level checks only**: Trust the application code to
include tenant_id in every query. But: forgetting one query is
a leak. Type-system enforcement (TenantContext) is much safer.

### Rationale

The four layers each address a different failure mode:

- **Quotas** prevent denial-of-service (intentional or accidental)
- **Tenant context** prevents data leakage from code bugs
- **Process isolation** prevents resource exhaustion from misbehaving
  per-tenant code
- **Audit** provides accountability and incident response

Resource quotas are essential for the commercial model: they
define the free vs. paid tier difference and prevent abuse.

The TenantContext discipline is borrowed from multi-tenant SaaS
patterns (e.g., Salesforce, Slack). It makes tenant scoping
visible in code review and uncaptured by accident.

Process isolation for optimizer/auto-tuner reflects that these
are the only Tier 2 services running tenant-specific code. A bug
in optimizer code shouldn't crash all tenants' optimizers.

### Consequences

**Enables:**
- Free vs. paid tier differentiation enforced technically
- Bug-induced data leak between tenants is a code-review-visible defect
- Operational incidents bounded to one tenant
- GDPR right-of-access/erasure straightforward (TenantContext-scoped queries)

**Constrains:**
- Every Tier 2 query must take a TenantContext (more boilerplate)
- Process spawning per optimizer run has overhead (mitigated by
  warm worker pool)
- Quota tuning requires monitoring real usage patterns
- Audit log volume can become significant — retention policy needed

**Operational responsibilities:**
- Operator: monitor quota usage, tune limits, investigate audit
  log alerts
- Self-hosted: quotas usually irrelevant (one tenant), but the
  architecture is the same

---

## ADR-15: GDPR Compliance and Data Lifecycle

### Context

A managed service operating in the EU is bound by GDPR. Even a
self-hosted user might want their data handled according to GDPR
principles (clear lifecycle, ability to delete). The architecture
must support these requirements as design constraints, not as
afterthoughts.

### Decision

The system implements GDPR-relevant capabilities by design:

**1. Lawful basis and purpose limitation:**

Each tenant's onboarding records the purposes of data processing:
- Real-time control (necessary for service)
- Optimization (necessary for service)
- Auto-tuning (necessary for service)
- Analytics for service improvement (legitimate interest, opt-out)
- Marketing (consent, opt-in)

Every component publishes which purpose its processing serves.
Audit logs capture this.

**2. Data minimization:**

The bus carries operational data (sensor values, controller states,
plans). Personal data is limited to:
- Tenant ID (UUID, pseudonymous)
- Email (for service contact, optional)
- Billing data (separate system)

The bus does NOT carry user names, addresses, or identifying info.
The mapping `tenant_id → real identity` lives in a separate, more
restricted system.

**3. Right of access (Art. 15):**

A `data_export` API endpoint returns all data for a given tenant_id:
- Configuration history
- Sensor data history (from Recorder)
- Decision logs (from Recorder)
- Tenant metadata
- Audit log entries for that tenant

Output format: JSON, downloadable, machine-readable.

**4. Right to erasure (Art. 17):**

A `data_delete` API endpoint executes within 30 days:
- Deletes time-series data for the tenant
- Deletes config history
- Anonymizes audit logs (replaces tenant_id with a deletion marker)
- Revokes tenant credentials
- Disconnects the bridge

Backups are reprocessed at next backup cycle to remove the tenant.

**5. Right to rectification (Art. 16):**

User can edit configuration and personal data via Tier 3 UI.
Changes are versioned (audit log).

**6. Right to data portability (Art. 20):**

The `data_export` output uses an open format (JSON Lines for
time-series, YAML for configuration) that another zero-feed-in
implementation could ingest. The schema is published as part of
the open-source Tier 1.

**7. Retention policies:**

| Data type | Retention | Rationale |
|---|---|---|
| Real-time sensor data | 90 days (configurable) | Beyond 90 days, aggregate statistics are kept |
| Aggregated statistics (daily/hourly) | 5 years | Long-term trend analysis |
| Decision logs | 1 year | Post-mortem analysis |
| Audit logs | 1 year minimum | Regulatory |
| Configuration history | Tenant lifetime + 90 days | Allow rollback |
| Backups | 30 days | Standard |

Retention is enforced by the Recorder (Tier 2 service) via scheduled
deletion jobs.

**8. Data Processing Agreement (DPA):**

For managed service, a DPA template is published and signed at
onboarding. The DPA describes:
- What data is processed
- What sub-processors are used (cloud provider, monitoring)
- Data location (EU-only for EU customers)
- Security measures (refers to ADR-12, ADR-13, ADR-14)
- Breach notification process

### Alternatives Considered

**Treat GDPR as operator's problem**: Don't build it into the
architecture, handle it ad hoc. Risk: every GDPR request is a
manual operation, retrofit is expensive when scale grows.

**Maximum data collection**: Keep everything forever, optimize
later. Violates data minimization and purpose limitation. Liability
in a breach is much higher.

**Anonymization at ingest**: Strip all identifiers before storage.
Strongest privacy. But: makes per-tenant features impossible.
Doesn't match the service model.

### Rationale

Building GDPR capabilities into the architecture (rather than
bolting them on) means:

- Right of access / erasure are routine queries, not engineering projects
- Retention policies are configuration, not code changes
- Audit trail is automatic (it's just the log topic)
- Sub-processor changes are tractable (operator notifies tenants
  through the system)

Data minimization in the bus (no PII in operational topics) is
defense in depth: a leak of operational data is less harmful than
a leak that includes user identities. The mapping
`tenant_id → identity` is a small, hardened table.

Retention as policy rather than ad-hoc cleanup means the system's
storage footprint is predictable, and tenants know what they're
agreeing to.

### Consequences

**Enables:**
- Managed service is GDPR-compliant by construction, not by audit
- Self-hosted users get a clean data lifecycle for free
- Audit and incident response are query-able, not archaeology
- Switching providers is feasible (data portability)

**Constrains:**
- TenantContext discipline (ADR-14) extends to user-data services
- Every new data type requires a retention policy decision
- Pseudonymization of audit logs after deletion adds complexity
  (the audit must remain useful for forensics while respecting
  the deletion)
- Cross-jurisdictional service (e.g., US tenants) requires
  additional thought

**Operational responsibilities:**
- Operator: maintain DPA, manage sub-processors, run deletion
  jobs, respond to subject requests within deadline
- Self-hosted: retention policies in user's hands, but the
  capability is built in

---

## ADR-16: Amortization Simulation via Historic Data Replay

### Context

US-9 (what-if analysis): A user wants to evaluate whether adding
a new component (e.g., a second battery, larger inverter, additional
PV string) is economically justified. This requires:
- REQ-33: Historic load/PV/grid data stored in Tier 2
- REQ-34: A simulation engine that replays history with modified
  system parameters and reports energy/cost deltas

The system may already be recording via the Recorder (ADR-8), but
for installations that connect to Tier 2 after running stand-alone
for a while, there must be a way to **backfill** historic data from
Home Assistant's built-in recorder into Tier 2.

### Decision

**1. Historic data ingest (HA → Tier 2)**

A **HistorySync** service (Tier 2) provides a one-shot, user-triggered
import of Home Assistant history into the Recorder's time-series store.

- Triggered explicitly via Tier 3 UI or CLI (`POST /api/history-sync`)
- Never runs automatically — avoids surprise bandwidth/load
- Reads HA long-term statistics API (`/api/history/period/...`)
- Writes to the Recorder's time-series store, tagged as `source: ha_import`
- Idempotent: re-running the same time range overwrites, does not duplicate
- Import scope: user selects entities and date range

```
User → Tier 3 UI → POST /api/history-sync
                        ↓
                  HistorySync service
                        ↓
              HA WebSocket/REST API (read history)
                        ↓
              Recorder time-series store (write)
```

**2. Amortization Simulator (Tier 2)**

A **Simulator** service replays historic data through a parameterized
plant model to answer "what if" questions:

- Input: historic load/PV profile (from Recorder), modified system
  config (e.g., battery capacity ×2, additional PV)
- Engine: closed-loop simulation using the same control logic as
  Tier 1, with a configurable plant model
- Output: energy balance delta (kWh saved from grid, kWh less
  feed-in), cost delta (at given tariff), payback estimate

The Simulator is stateless — it reads historic data from the Recorder,
runs the simulation, and returns results. No persistent state beyond
what's already in the Recorder.

```
User → Tier 3 UI → POST /api/simulate
                        ↓
                  Simulator service
                    ├── reads historic profiles from Recorder
                    ├── runs closed-loop sim with modified params
                    └── returns energy/cost delta
```

**3. Data requirements**

The Recorder must store at minimum:
- Grid power (import/export) at ≤ 5-minute resolution
- PV production
- Battery power and SOC
- Consumer load (derived: load = grid + battery - PV export)

These are already recorded by the Tier 1 Logger (ADR-8). The
HistorySync backfills the same schema from HA history.

### Alternatives Considered

**Automatic HA sync on Tier 2 connection**: Simpler UX, but
violates the user's explicit-trigger requirement. Could cause
unexpected network load or HA overload on large histories.

**Simulation in Tier 3 (browser)**: Avoids server load. But:
large datasets (months of 5s data) exceed browser memory. The
plant model and control logic would need a JS port. Not portable
to headless CLI use.

**Dedicated data warehouse separate from Recorder**: Overkill —
the Recorder already has the time-series store, retention policies,
and tenant isolation. Adding a parallel store doubles complexity.

### Rationale

- **Explicit trigger** respects the user's control over their data
  and network usage
- **Reusing the Recorder** avoids a separate persistence layer
- **Stateless Simulator** is easy to scale, test, and replace
- **Same control logic** in simulation as in production ensures
  fidelity (simulation output matches what the system would actually
  do with the modified hardware)
- Historic backfill is a one-time bootstrapping step; once Tier 2
  is connected, the normal Recorder path takes over

### Consequences

**Enables:**
- Users can make data-driven hardware purchasing decisions
- Supports evaluating any hardware change: battery size, PV
  capacity, inverter limits, tariff changes
- Historic data is available for other Tier 2 services (AutoTuner,
  Optimizer) even if Tier 2 was connected late
- CLI and UI access to simulation (not UI-only)

**Constrains:**
- HistorySync depends on HA's history API availability and retention
  (HA default: 10 days; long-term statistics: indefinite but hourly)
- Simulation fidelity is bounded by plant model accuracy
- Large history imports may take minutes — needs progress reporting
- Recorder retention policies (ADR-15) apply to imported data too

---

# Component Catalog

Reference table of component types and the topics they publish/subscribe.

## Drivers

### GridMeterDriver
- **Reads**: HA sensor entity via WebSocket API (e.g., EDL21 power),
  or native MQTT for MQTT-direct meters
- **Publishes**: `zfi/1/sensors/grid/power` (Power message)
- **Degraded**: Publishes `unavailable` status

### BatteryDriver
- **Reads**: HA entities via WebSocket API (SOC, battery_power, output
  state), or native MQTT (e.g., direct Zendure connection)
- **Publishes**: `zfi/1/sensors/battery/<id>/state` (BatteryState message)
- **Subscribes**: `zfi/1/control/battery/<id>/cmd` (Command message)
- **Writes**: HA entities via REST API service calls (output_limit,
  input_limit, ac_mode), or native MQTT commands
- **Degraded**: Publishes `unavailable`, ignores commands

### PVDriver
- **Reads**: HA sensor entity via WebSocket API (PV power)
- **Publishes**: `zfi/1/sensors/pv/power` (Power message)
- **Degraded**: Publishes `unavailable`

## Aggregators

### BatteryAggregator
- **Subscribes**: `zfi/1/sensors/battery/+/state` (per-instance states)
- **Publishes**: `zfi/1/sensors/battery/aggregate/state` (combined)
- **Subscribes**: `zfi/1/control/battery/aggregate/cmd`
- **Publishes**: `zfi/1/control/battery/<id>/cmd` (distributed)
- **Degraded**: With N-1 instances, recompute aggregate

## Detectors

### StoveDetector
- **Reads**: HA entity (kitchen power meter)
- **Publishes**: `zfi/1/situations/stove` (Situation message: bool)
- **Degraded**: Publishes `unknown`

### EVChargingDetector
- **Reads**: HA entity (wallbox power)
- **Publishes**: `zfi/1/situations/ev_charging`
- **Degraded**: Publishes `unknown`

## Controllers

### NormalController
- **Subscribes**: `zfi/1/sensors/+/state`, `zfi/2/plans/current`,
  `zfi/1/lifecycle/normal` (SLEEP/ACTIVE signals)
- **Publishes**: `zfi/1/control/mux/normal/desired_power` (always, even when sleeping — or stops in SLEEP)
- **Lifecycle**: SLEEP when not active (freezes state, unsubscribes sensors)
- **Activated by**: MUX (via lifecycle signal)

### StoveController
- Similar to NormalController, publishes to `zfi/1/control/mux/stove/desired_power`

## Multiplexer

### ControllerMux
- **Subscribes**: `zfi/1/control/mux/+/desired_power` (all controller channels)
- **Subscribes**: `zfi/1/control/active_controller` (from Supervisor)
- **Publishes**: `zfi/1/control/desired_power` (active channel passthrough)
- **Publishes**: `zfi/1/lifecycle/<name>` (SLEEP/ACTIVE + WakeSeed per controller)
- **Degraded**: Hold last known active channel; publish idle (0W) if channel stale

## Supervisor

### RuleBasedSupervisor
- **Subscribes**: `zfi/1/situations/+`
- **Publishes**: `zfi/1/control/active_controller`
- **Logic**: Rule cascade (detector → controller mapping)

## Services

### Recorder
- **Subscribes**: `zfi/+/log/+`, `zfi/+/sensors/+/+`, others as configured
- **Writes**: Time-series store (InfluxDB / SQLite)
- **Degraded**: Drops messages (logged warning)

### Optimizer (Tier 2)
- **Subscribes**: forecasts, prices, historical data
- **Publishes**: `zfi/2/plans/current`
- **Schedule**: Periodic (15 min) + on forecast update

### AutoTuner (Tier 2)
- **Reads**: Recorder backend (historical data)
- **Publishes**: `zfi/2/parameters/proposed`
- **Schedule**: Daily at 03:00

### Simulator (Tier 2)
- **Reads**: Recorder backend (historic load/PV/battery profiles)
- **Input**: Modified system config (via REST API)
- **Output**: Energy/cost delta report (via REST API)
- **Stateless**: No persistent state, pure computation

### HistorySync (Tier 2)
- **Reads**: HA history API (long-term statistics, entity history)
- **Writes**: Recorder time-series store
- **Triggered**: Explicitly via REST API (`POST /api/history-sync`)
- **Never automatic**: User must initiate

### Bridge (Tier 1 ↔ Tier 2)
- **Subscribes**: Topic patterns from Tier 1
- **Publishes**: To Tier 2 broker
- **Reverse**: Subscribes Tier 2 patterns, publishes to Tier 1

### HA Integration (Tier 1)
- Already covered in ADR-4

---

# Requirements Traceability

| Req | Title | Addressed By |
|---|---|---|
| REQ-1 | Internal message bus | ADR-1, ADR-3 |
| REQ-2 | Dynamic component loading | ADR-6 (Graph Builder) |
| REQ-3 | Developer observability | ADR-1 (MQTT introspection) |
| REQ-3a | Causal tracing | ADR-1 (causal_context), ADR-8 |
| REQ-3b | Module activity visibility | ADR-1 (lifecycle topic), ADR-6 |
| REQ-4 | Layered architecture | ADR-2 (component types) |
| REQ-5 | Driver abstraction | ADR-2, D-4 |
| REQ-6 | Pluggable controllers | ADR-2 (Controller + Supervisor + **Multiplexer**) |
| REQ-7 | Situation detectors | ADR-2 (Detector type) |
| REQ-8 | Config-driven assembly | ADR-5, ADR-6 |
| REQ-9 | Auto aggregators | ADR-6 (Graph Builder) |
| REQ-10 | Per-instance parameters | ADR-5 (user config) |
| REQ-11 | Graph visibility | ADR-6 (Graph object), Tier 3 viz |
| REQ-12 | Config-driven startup | ADR-6 |
| REQ-13 | Planning vs real-time | ADR-7 (tier separation) |
| REQ-14 | Forecast integration | Optimizer service (Tier 2) |
| REQ-15 | Multi-objective optimization | Optimizer slot (impl deferred) |
| REQ-16 | Consumer scheduling | Optimizer + flexible HA switches |
| REQ-17 | Plan adaptation | Optimizer schedule + on forecast update |
| REQ-18 | Operational data recording | ADR-8 (Recorder) |
| REQ-19 | System identification | AutoTuner reads Recorder |
| REQ-20 | Parameter proposals with reasoning | AutoTuner |
| REQ-21 | Safe auto-apply with bounds | AutoTuner + supervisor approval |
| REQ-22 | Custom visualization | Tier 3 (web app) |
| REQ-23 | Decision transparency | ADR-8 (logs), Tier 3 |
| REQ-24 | Historical playback | ADR-8 (Recorder) + Tier 3 |
| REQ-25 | Client/server separation | ADR-7 (tiers + bridge) |
| REQ-26 | Open source / commercial | ADR-7 (tier boundary), **ADR-11 (tenant = commercial boundary)** |
| REQ-27 | Structured post-mortem | ADR-8 |
| REQ-28 | Log queryability | ADR-8 (Recorder backend) |
| REQ-29 | Log-based replay | ADR-8 + simulation harness |
| REQ-30 | Graceful degradation | ADR-10 |
| REQ-31 | Component independence | ADR-1 (bus only coupling), ADR-10 |
| REQ-32 | Incremental tier deployment | ADR-7 (independent tiers) |
| REQ-33 | Historic data available in Tier 2 | ADR-8 (Recorder), ADR-16 (HistorySync backfill) |
| REQ-34 | What-if simulation against historic data | ADR-16 (Simulator service) |

All 34 requirements addressed.

---

# Open Architectural Issues

The following architectural questions are intentionally deferred:

1. **Schema evolution policy** — How to handle breaking changes in
   message schemas across versions. Probably semver + deprecation
   windows, but no formal policy yet.

2. **Authentication/encryption** — MQTT supports TLS+credentials.
   Tier 1 ↔ Tier 2 traffic should be encrypted when crossing the
   network. Local broker in Phase 1 can be unencrypted.

3. **State persistence on restart** — Should controllers save their
   state to recover on restart? Currently planned: no, rely on
   bus state and quick reconvergence. May need to revisit.

4. **Supervisor failure mode** — What if the Supervisor itself
   crashes? Default: the Multiplexer holds the last known active
   channel. A watchdog supervisor that activates a safe default
   controller is the longer-term solution. Detail in component spec.

5. **Controller sleep depth** — Should a sleeping controller
   unsubscribe from ALL topics or keep a lightweight heartbeat
   subscription? Full unsubscribe is more efficient; heartbeat
   allows faster wake. Decided in component spec based on measured
   wake-up latency.

5. **Driver-to-driver dependencies** — E.g., a driver might need
   another driver's data. Architecture says go through bus; need
   to verify this is always efficient enough.

6. **Tenant provisioning and lifecycle** — How is a tenant_id
   created? What happens when a user stops paying (bridge disabled)?
   How are tenant namespaces cleaned up? These are operational
   questions for the managed service, not for Phase 1-3.

These don't block Phase 1 but should be resolved before later phases.
