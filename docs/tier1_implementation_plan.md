# Tier 1 — Implementation Plan

## Purpose

This document defines how to implement Tier 1 (the local control
system) in phases that each deliver value standalone. It maps the
existing AppDaemon code into the new architecture (`architecture_design.md`).

Tier 2 and Tier 3 are out of scope here. A coarse timeline at the
end shows when their development can begin in parallel with Tier 1.

## Guiding Principles

1. **Existing code first.** The current `zero_feed_in_controller.py`
   and `zendure_solarflow_driver.py` represent working logic. They
   are the starting point, not throwaways.

2. **Each phase ships.** No phase requires the next to be useful.
   Stopping after any phase leaves a working system — possibly with
   fewer features than the end state.

3. **Architecture-first migration.** The first phase establishes
   the bus and component structure. Later phases add capability
   *into* that structure, not alongside it.

4. **Bus as scaffolding.** The MQTT bus is built early even when
   only one component uses it. This makes adding components
   later trivial — no big-bang migration.

## Existing Assets

What already exists and must not be rewritten:

| Asset | Location | Disposition |
|---|---|---|
| `ControlLogic` class | `zero_feed_in_controller.py` | Keep verbatim, move to `engine/logic/controller.py` |
| `RelayStateMachine` | `zendure_solarflow_driver.py` | Keep verbatim, move to `engine/logic/relay_sm.py` |
| Schmitt trigger, charge-confirm | inline in ControlLogic | Stays in ControlLogic |
| Surplus estimation | inline | Stays in ControlLogic |
| Step response measurements | `timings` file | Translate to test fixtures |

What does NOT yet exist:

- HA Add-on packaging (Dockerfile, config.yaml, s6-overlay)
- Embedded Mosquitto configuration and credentials management
- HA WebSocket and REST API clients (engine-side)
- Thin Custom Integration (entity registration via MQTT)
- MQTT bus client wrapper (Tier 1 internal bus)
- Component base classes (Driver, Detector, Controller, Supervisor, Multiplexer)
- Graph Builder
- Structured logging infrastructure
- Lifecycle protocol implementation

---

## Phase Overview

```
Phase 1: Bus Foundation + Lift-and-Shift     (Weeks 1-4)
   Today's logic, in an HA Add-on with embedded Mosquitto,
   plus thin Custom Integration for HA UI
   → Value: Working zero-feed-in, testable, observable, no AppDaemon

Phase 2: Component Type Discipline           (Weeks 5-6)
   Refactor lift-and-shift into proper component types
   → Value: Clean architecture, ready to extend

Phase 3: Multiplexer + Second Controller     (Weeks 7-8)
   Stove controller as proof of concept
   → Value: Mode switching, validates extensibility

Phase 4: Detectors + Supervisor              (Weeks 9-10)
   Decouple "is the stove on" from controllers
   → Value: Adding detectors becomes one-liner

Phase 5: Driver Generalization               (Weeks 11-13)
   Driver interface, native-MQTT driver path
   → Value: Code is hardware-agnostic, transport-agnostic

Phase 6: Configuration-Driven Assembly       (Weeks 14-15)
   Graph Builder reads Add-on options, assembles topology
   → Value: User-friendly setup, multi-instance ready

Phase 7: Aggregator (when N>1 needed)        (When user has 2+ batteries)
   Battery fusion as new component type
   → Value: Multi-battery setups
```

Each phase is described below with concrete steps, what it
delivers, and what becomes possible afterward.

---

## Phase 1: Bus Foundation + Lift-and-Shift

**Duration**: 3-4 weeks (Add-on packaging adds initial overhead)
**Prerequisite**: Existing AppDaemon code working as today
**Goal**: Today's working system, running on tomorrow's architecture

### What gets built

Two distinct deliverables, per ADR-4:

```
zfi-addon/                          # The Add-on (Docker container)
├── Dockerfile
├── config.yaml                     # Add-on schema for HA Supervisor
├── repository.yaml                 # for HACS-style installation
├── rootfs/
│   └── etc/services.d/
│       ├── mosquitto/              # process supervisor (s6-overlay)
│       │   └── run
│       └── zfi-engine/
│           └── run
├── mosquitto/
│   ├── mosquitto.conf              # listens on 1884
│   └── acl.conf                    # per-component ACL rules
└── engine/                         # Python package — the engine
    ├── __init__.py
    ├── main.py                     # entry point: start bus, build graph
    ├── bus/                        # bus client + message schemas
    │   ├── client.py
    │   ├── messages.py             # Pydantic schemas v1
    │   └── topics.py
    ├── logic/                      # PURE PYTHON — no HA, no MQTT imports
    │   ├── controller.py           # ControlLogic — verbatim from existing
    │   ├── relay_sm.py             # RelayStateMachine — verbatim
    │   └── types.py                # Measurement, ControlOutput, Config
    ├── runtime/                    # bus-aware wrappers around logic
    │   ├── controller_runtime.py
    │   └── driver_runtime.py
    ├── drivers/                    # Phase 1 drivers (Zendure-specific)
    │   ├── ha_grid_meter.py        # reads HA via WebSocket → bus
    │   ├── ha_battery.py           # reads HA via WebSocket → bus
    │   └── ha_actuator.py          # bus → HA REST service calls
    └── ha_client/                  # WebSocket + REST clients for HA API
        ├── websocket.py
        └── rest.py


custom_components/zero_feed_in/    # The thin Custom Integration in HA
├── __init__.py                     # async_setup_entry / unload
├── manifest.json
├── const.py                        # DOMAIN, MQTT broker port (1884)
├── mqtt.py                         # subscribes to embedded Mosquitto
├── sensor.py                       # zfi_* sensors mirror bus topics
├── number.py                       # zfi_* numbers (e.g., min_soc)
├── switch.py                       # zfi_* switches
└── translations/en.json
```

The **engine** is what does the work. The **Custom Integration** is a
thin reflection of bus state into HA entities.

### Concrete Steps

**Step 1.1 — Add-on packaging skeleton**

Create the Add-on structure following HA Add-on conventions:
- `config.yaml` declares the Add-on schema (slug, version, options)
- Dockerfile based on `homeassistant/base` images
- s6-overlay or simple supervisord to run two processes (Mosquitto + engine)

Verify the empty Add-on installs and starts in HA Supervisor.

**Step 1.2 — Embedded Mosquitto**

Add Mosquitto to the container:
- Listens on `0.0.0.0:1884` (so the Custom Integration on the host
  can reach it via the container port mapping)
- Initial ACL: permissive within zfi namespace, can be tightened
  (Phase 2+) once components stabilize
- Persistent password file in `/data/mosquitto/passwords` (Add-on data
  volume — survives restarts)
- Generates default credentials on first run, writes them to
  `/share/zfi/credentials.json` for the Custom Integration to read

Test: connect from the host with `mosquitto_sub -h localhost -p 1884`.

**Step 1.3 — Engine bus client**

A thin async wrapper around `asyncio-mqtt`:

```python
class BusClient:
    async def connect(self): ...
    async def publish(self, topic: str, msg: BaseModel): ...
    async def subscribe(self, topic: str, handler: Callable): ...
    async def disconnect(self): ...
```

Handles JSON serialization, schema validation on receive, reconnection.
Tested standalone (no HA needed).

**Step 1.4 — Define v1 message schemas**

Pydantic models for the topics needed in Phase 1:

```python
# bus/messages.py
class GridPower(BaseModel):
    schema_version: str = "1.0"
    timestamp: datetime
    source: str
    power_w: float

class BatteryState(BaseModel):
    schema_version: str = "1.0"
    timestamp: datetime
    source: str
    soc_pct: float
    power_w: float  # signed: + discharge, - charge

class DesiredPower(BaseModel):
    schema_version: str = "1.0"
    timestamp: datetime
    source: str
    power_w: float
    reason: str

class LogRecord(BaseModel):
    schema_version: str = "1.0"
    timestamp: datetime
    source: str
    level: str
    event_type: str
    data: dict
```

Only what Phase 1 needs. More schemas added per phase.

**Step 1.5 — Move logic verbatim**

Take `ControlLogic` and `RelayStateMachine` from the current AppDaemon
code and place them in `engine/logic/`. **No changes to the algorithms.**

The `logic/` package has zero HA imports, zero MQTT imports. It works
with `Measurement` in, `ControlOutput` out — same as today.

Verify with pytest tests (write them now if none exist).

**Step 1.6 — HA WebSocket and REST clients**

The engine talks to HA via HA's authenticated APIs:

```python
# engine/ha_client/websocket.py
class HAWebSocketClient:
    """Subscribes to entity state changes via HA's WebSocket API."""

    async def connect(self, base_url: str, token: str): ...
    async def subscribe_entities(self, entity_ids: list[str], handler): ...

# engine/ha_client/rest.py
class HARESTClient:
    """Calls HA services to write to entities."""

    async def call_service(self, domain, service, data): ...
    async def get_state(self, entity_id): ...
```

Authentication uses a long-lived access token, configured in the
Add-on options (or auto-provisioned via HA Supervisor's internal
proxy — `http://supervisor/core/api`).

**Step 1.7 — Drivers (Phase 1: HA-based)**

Drivers that read HA sensors via WebSocket, publish to the bus:

```python
# engine/drivers/ha_grid_meter.py
class HAGridMeterDriver:
    def __init__(self, ha_ws: HAWebSocketClient, entity_id: str, bus: BusClient):
        ...

    async def start(self):
        await self.ha_ws.subscribe_entities(
            [self.entity_id], self._on_state
        )

    async def _on_state(self, entity_id, new_state):
        if new_state in ("unknown", "unavailable"):
            return
        await self.bus.publish(
            "zfi/1/sensors/grid/power",
            GridPower(
                timestamp=datetime.utcnow(),
                source=f"driver.ha_grid_meter.{self.entity_id}",
                power_w=float(new_state),
            ),
        )
```

Same pattern for battery driver (reads SOC + battery_power) and
actuator driver (subscribes to `zfi/1/control/desired_power`,
translates to HA service calls).

**Step 1.8 — Controller runtime**

Wraps `ControlLogic` for the bus:

```python
# engine/runtime/controller_runtime.py
class ControllerRuntime:
    def __init__(self, logic: ControlLogic, bus: BusClient):
        ...

    async def start(self):
        await self.bus.subscribe("zfi/1/sensors/grid/power", self._on_grid)
        await self.bus.subscribe("zfi/1/sensors/battery/main/state", self._on_battery)

    async def _on_grid(self, msg):
        self._latest_grid = msg
        await self._maybe_compute()

    async def _maybe_compute(self):
        if not self._latest_grid or not self._latest_battery:
            return
        m = Measurement(
            grid_power_w=self._latest_grid.power_w,
            soc_pct=self._latest_battery.soc_pct,
            battery_power_w=self._latest_battery.power_w,
        )
        output = self.logic.compute(m)
        if output is not None:
            await self.bus.publish(
                "zfi/1/control/desired_power",
                DesiredPower(...),
            )
```

The runtime is the only place that touches both the bus and the
pure logic.

**Step 1.9 — Engine entry point**

`main.py` reads Add-on config, instantiates everything, starts the
bus loop:

```python
# engine/main.py
async def main():
    config = load_addon_options()  # /data/options.json (HA-managed)
    
    bus = BusClient(broker="localhost", port=1884, credentials=...)
    await bus.connect()
    
    ha_ws = HAWebSocketClient()
    await ha_ws.connect(
        base_url="http://supervisor/core",
        token=os.environ["SUPERVISOR_TOKEN"],
    )
    ha_rest = HARESTClient(...)
    
    # Pure logic
    cfg = Config.from_dict(config)
    control_logic = ControlLogic(cfg)
    
    # Drivers (Phase 1: HA-based)
    grid = HAGridMeterDriver(ha_ws, config["grid_sensor"], bus)
    battery = HABatteryDriver(ha_ws, config["battery_sensors"], bus)
    actuator = HAActuatorDriver(ha_rest, config["actuators"], bus)
    
    # Controller runtime
    controller = ControllerRuntime(control_logic, bus)
    
    # Start everything
    for c in [grid, battery, actuator, controller]:
        await c.start()
    
    await asyncio.Event().wait()  # run forever
```

**Step 1.10 — Thin Custom Integration**

In `custom_components/zero_feed_in/`, register HA entities that
subscribe to bus topics on the embedded Mosquitto:

```python
# custom_components/zero_feed_in/sensor.py
class ZFIGridPowerSensor(SensorEntity):
    _attr_native_unit_of_measurement = "W"
    _attr_device_class = SensorDeviceClass.POWER
    _attr_state_class = SensorStateClass.MEASUREMENT

    async def async_added_to_hass(self):
        self._unsub = await self.hass.data[DOMAIN]["mqtt"].subscribe(
            "zfi/1/sensors/grid/power", self._on_message
        )

    def _on_message(self, msg):
        data = json.loads(msg.payload)
        self._attr_native_value = data["power_w"]
        self.async_write_ha_state()
```

The integration reads broker credentials from `/share/zfi/credentials.json`
(written by the Add-on on first start). No Config Flow.

Entities to expose in Phase 1:
- `sensor.zfi_grid_power`
- `sensor.zfi_battery_soc`
- `sensor.zfi_desired_power`
- `sensor.zfi_mode`
- `number.zfi_min_soc`

**Step 1.11 — Structured logging via bus**

Every component publishes `LogRecord` to `zfi/1/log/<component>`.
A simple subscriber in the engine writes to a rotating file in
`/data/logs/`. (Recorder service comes in Phase 4.)

### What works after Phase 1

- zfi runs as an HA Add-on, installable via HA Supervisor UI
- Configuration via standard HA Add-on options form
- Zero feed-in operates exactly as today
- All sensor values, commands, and decisions flow through the bus
- An MQTT client (mqtt-explorer connecting to localhost:1884) shows
  the live system in real time
- Logic is testable without HA (pure `ControlLogic` tests)
- Engine restarts don't require HA restart
- HA reload doesn't restart the engine

### What does NOT work yet (out of scope)

- Multiple controllers (Phase 3)
- Detectors (Phase 4)
- Driver abstraction is informal — HA-based drivers are concrete
  classes, not yet a unified pattern (Phase 5)
- Config-driven assembly (Phase 6) — `main.py` wires statically
- Aggregator (Phase 7)

### Migration risk

Medium. The logic is unchanged, but Add-on packaging is new
infrastructure work. The Custom Integration registering entities
from MQTT is a known HA pattern (used by Zigbee2MQTT) but new
to this project. Failure modes are addressable with graceful
degradation (ADR-10).

### Architecture questions raised

- **Q-P1-A**: Add-on Supervisor token vs. long-lived token —
  which to use for HA WebSocket access? Resolved: use Supervisor
  token via `http://supervisor/core/api` when running as Add-on
  (auto-provided). Long-lived token is fallback for development
  outside Supervisor.

- **Q-P1-B**: Where does the Custom Integration get broker
  credentials? Resolved: from `/share/zfi/credentials.json`
  written by the Add-on. Documented in installation steps.

- **Q-P1-C**: `main.py` instantiates everything statically. Phase 6
  introduces the Graph Builder which replaces this. Until then,
  "graph builder" is a hardcoded function — accept this.

---

## Phase 2: Component Type Discipline

**Duration**: 1-2 weeks
**Prerequisite**: Phase 1 complete and stable
**Goal**: Refactor the Phase-1 lift-and-shift into proper ADR-2 component types

### What gets built

Component base classes from ADR-2:

```python
# components/base.py
class Component(ABC):
    """Base for all bus components."""
    name: str
    bus: BusClient

    async def start(self): ...
    async def stop(self): ...
    @abstractmethod
    def degraded_behavior(self) -> DegradedMode: ...

class Driver(Component):
    """Hardware abstraction. Reads from / writes to physical
    devices via HA WebSocket/REST API, native MQTT, or HTTP.
    All drivers are uniform — no separate Bridge concept."""
    pass

class Controller(Component):
    """Computes desired_power. Publishes to its own MUX channel.

    Phase 2 stub: only NormalController, no MUX yet.
    Will get sleep/wake protocol in Phase 3.
    """
    @abstractmethod
    def compute(self, m: Measurement) -> ControlOutput | None: ...

class Detector(Component):
    """Pure observer. Reads bus/HA, publishes situation state."""
    pass

class Supervisor(Component):
    """Decides which controller is active. No routing."""
    pass

class Multiplexer(Component):
    """Routes active controller, manages sleep/wake. Phase 3."""
    pass

class Aggregator(Component):
    """Fans out to N drivers. Phase 7."""
    pass

class Service(Component):
    """Catch-all for non-real-time components."""
    pass
```

### Concrete Steps

**Step 2.1 — Define base classes**

Move from informal "runtime" wrappers to typed `Component` subclasses.
Each existing piece of Phase 1 is reclassified:

| Phase 1 module | Phase 2 type |
|---|---|
| `runtime/controller_runtime.py` | `controllers/normal.py` (subclass of `Controller`) |
| `runtime/driver_runtime.py` | Folded into the actuator driver |
| `drivers/ha_grid_meter.py` | `drivers/ha_grid_meter.py` (subclass of `Driver`) |
| `drivers/ha_battery.py` | `drivers/ha_battery.py` (subclass of `Driver`) |
| `drivers/ha_actuator.py` | Merged into `ha_battery.py` (battery driver does both read and write) |

Per ADR-2's note on hardware abstraction: HA-based and native-MQTT
drivers are both Drivers. There is no "Bridge" type. The HA-based
drivers from Phase 1 simply become subclasses of `Driver` with no
fanfare.

**Step 2.2 — Establish lifecycle protocol**

Even before MUX (Phase 3), every component supports:
- `async start()` — subscribe to topics, initialize state
- `async stop()` — cleanup
- `degraded_behavior()` — return mode (REQ-30)

This is the foundation for Sleep/Wake in Phase 3.

**Step 2.3 — Component registry**

A simple registry mapping name → class:

```python
# components/registry.py
COMPONENT_TYPES = {
    "drivers.ha_grid_meter": HAGridMeterDriver,
    "drivers.ha_battery": HABatteryDriver,
    "controllers.normal": NormalController,
    # ...
}
```

The Coordinator looks up types by name. Phase 6 will read names
from config; in Phase 2 the names are hardcoded.

**Step 2.4 — Refactor without behavior change**

The actual behavior of the system MUST NOT change in Phase 2. This
is a pure refactor: reorganize the code into the type discipline,
verify all existing tests still pass, no new features.

### What works after Phase 2

- Code is organized by ADR-2 component types
- Adding a new controller is a clear pattern (subclass `Controller`)
- Lifecycle hooks exist for Phase 3
- Test infrastructure can mock at the component level

### What does NOT work yet

- Multiple controllers (need MUX from Phase 3)
- Component registration is still hardcoded
- Drivers are HA-API-based only (native MQTT and other transports
  not yet supported — Phase 5)

### Architecture questions raised

- **Q-P2-A**: How should component instances be named? Singletons
  (`controllers.normal`) work for Phase 2 but break in Phase 7
  with multiple battery drivers. Tentative answer: `<type>.<instance_id>`,
  with `<instance_id>` defaulting to "main" if absent. Resolved
  in Phase 6 with the Graph Builder.

---

## Phase 3: Multiplexer + Second Controller

**Duration**: 2 weeks
**Prerequisite**: Phase 2 complete
**Goal**: Add a second controller with proper switching

### What gets built

- `Multiplexer` component (ADR-2 detail, MUX section)
- `StoveController` as a second controller
- Sleep/Wake protocol in `Controller` base
- Updated topic structure: `zfi/1/control/mux/<n>/desired_power`

### Concrete Steps

**Step 3.1 — Implement Multiplexer**

```python
class ControllerMux(Multiplexer):
    def __init__(self, bus, controllers: list[str]):
        self.controllers = controllers  # names of controllers to manage
        self.active: str | None = None
        self.last_routed: float = 0.0

    async def start(self):
        for name in self.controllers:
            await self.bus.subscribe(
                f"zfi/1/control/mux/{name}/desired_power",
                self._make_handler(name),
            )
        await self.bus.subscribe(
            "zfi/1/control/active_controller",
            self._on_active_change,
        )

    def _make_handler(self, name: str):
        async def handler(msg: DesiredPower):
            if name != self.active:
                return  # ignore inactive channels
            self.last_routed = msg.power_w
            await self.bus.publish("zfi/1/control/desired_power", msg)
        return handler

    async def _on_active_change(self, msg: ActiveController):
        old = self.active
        self.active = msg.controller
        if old:
            await self._send_lifecycle(old, "SLEEP")
        await self._send_lifecycle(self.active, "ACTIVE",
                                    wake_seed=self.last_routed)

    async def _send_lifecycle(self, name, state, wake_seed=None):
        await self.bus.publish(
            f"zfi/1/lifecycle/{name}",
            LifecycleSignal(target=name, state=state, wake_seed=...),
        )
```

**Step 3.2 — Refactor Controller base for sleep/wake**

```python
class Controller(Component):
    def __init__(self, bus, name: str):
        super().__init__(name, bus)
        self.is_active = False
        self.last_sent_w = 0.0

    async def start(self):
        await self.bus.subscribe(
            f"zfi/1/lifecycle/{self.name}", self._on_lifecycle
        )
        # Subscribe to sensors as before, but only act when active

    async def _on_lifecycle(self, msg: LifecycleSignal):
        if msg.state == "ACTIVE":
            self.is_active = True
            if msg.wake_seed:
                self.last_sent_w = msg.wake_seed.last_desired_power
            await self._on_wake()
        else:  # SLEEP
            self.is_active = False
            await self._on_sleep()

    async def _on_sleep(self):
        # Default: stop processing. Subclasses can extend.
        pass

    async def _on_wake(self):
        # Default: nothing special. Subclasses can extend.
        pass
```

The existing `NormalController` becomes a subclass that publishes
to `zfi/1/control/mux/normal/desired_power` instead of
`zfi/1/control/desired_power`.

**Step 3.3 — StoveController stub**

Initially, `StoveController` is a copy of `NormalController` with
different parameters (e.g., higher hysteresis to avoid flapping
during stove transients). Refining its behavior is an
implementation detail outside this plan.

**Step 3.4 — Manual switch entity**

For Phase 3, the "supervisor" is a HA `select` entity that the
user toggles manually:

```yaml
select.zfi_active_controller: ["normal", "stove"]
```

A small adapter in the engine reads this select entity (via the
HA WebSocket client) and publishes to `zfi/1/control/active_controller`.
Real Supervisor (auto-decision) comes in Phase 4.

### What works after Phase 3

- Two controllers coexist
- User can switch between them via HA select entity
- Sleep/wake protocol verified — inactive controller uses no resources
- Switching is clean (MUX prevents race conditions)

### What does NOT work yet

- Automatic switching based on situation (Phase 4)
- Real stove logic in StoveController (implementation detail)

### Architecture questions raised

- **Q-P3-A**: How does the MUX know which controllers exist?
  Phase 3: configured by name list passed at construction.
  Phase 6: discovered via Graph Builder.

- **Q-P3-B**: Should the MUX validate that the active controller
  is in its known list? Yes — defensive: ignore unknown active
  controller, keep last routing. Add to ADR-2 as an MUX detail
  if not already.

---

## Phase 4: Detectors + Supervisor

**Duration**: 1-2 weeks
**Prerequisite**: Phase 3 complete
**Goal**: Replace manual controller switching with automatic
situation-based decisions

### What gets built

- `StoveDetector` reading kitchen power sensor
- `RuleBasedSupervisor` mapping detectors to controllers
- Detectors registered in coordinator
- HA select entity becomes read-only display (or override)

### Concrete Steps

**Step 4.1 — Detector base class**

```python
class Detector(Component):
    @abstractmethod
    async def evaluate(self, ...) -> Situation: ...
```

A detector subscribes to its input source (HA sensor or bus topic),
computes a situation state, publishes to `zfi/1/situations/<n>`.

**Step 4.2 — StoveDetector**

```python
class StoveDetector(Detector):
    def __init__(self, bus, ha_ws, sensor: str, threshold_w: float):
        ...

    async def _on_kitchen_power(self, power_w: float):
        active = power_w > self.threshold_w
        await self.bus.publish(
            "zfi/1/situations/stove",
            Situation(
                timestamp=...,
                source=self.name,
                kind="stove",
                active=active,
                detail={"power_w": power_w},
            ),
        )
```

**Step 4.3 — Supervisor with rule cascade**

```python
class RuleBasedSupervisor(Supervisor):
    def __init__(self, bus, rules: list[Rule]):
        # rules: ordered list, first match wins
        self.rules = rules
        self.active = "normal"  # default

    async def start(self):
        for rule in self.rules:
            for situation in rule.requires:
                await self.bus.subscribe(
                    f"zfi/1/situations/{situation}",
                    self._reevaluate,
                )

    async def _reevaluate(self, _msg):
        for rule in self.rules:
            if rule.matches(self._situations):
                if rule.activate != self.active:
                    self.active = rule.activate
                    await self.bus.publish(
                        "zfi/1/control/active_controller",
                        ActiveController(controller=rule.activate),
                    )
                return
```

Rules are simple in Phase 4: "if stove situation active → activate
stove controller, else normal".

**Step 4.4 — Optional manual override**

The HA select entity from Phase 3 stays, now as an *override*. If
the user manually selects, the Supervisor respects it (writes to
its internal state) until the user switches back to "auto".

### What works after Phase 4

- System automatically detects stove and switches controller
- Adding a detector: one new class + one rule entry
- Supervisor logic is centralized and inspectable

### What does NOT work yet

- Detectors are still hand-instantiated in coordinator (until Phase 6)
- Driver abstraction (Phase 5)

### Architecture questions raised

- **Q-P4-A**: Rule format. YAML in Phase 6, hardcoded list in
  Phase 4. Should rules be in user config or developer config?
  Rules tie detectors to controllers — both must exist. User
  config is the right place once Graph Builder is live.

---

## Phase 5: Driver Generalization

**Duration**: 2-3 weeks
**Prerequisite**: Phase 4 complete
**Goal**: Promote ad-hoc Driver subclasses to a formal interface;
prepare for adding non-Zendure batteries and native-MQTT drivers

### What gets built

- `BatteryDriver` base class with formal interface (read_state,
  send_command, capabilities)
- `GridMeterDriver` and `PVDriver` base classes
- Existing HA-API drivers refactored against the new interfaces
- First non-HA driver: `ZendureNativeMQTTDriver` connects directly
  to the Zendure device's MQTT (skipping HA's entities)

### Concrete Steps

**Step 5.1 — Define Driver interfaces**

```python
class BatteryDriver(Driver):
    """Reads battery state, accepts power commands."""
    @abstractmethod
    async def read_state(self) -> BatteryState: ...
    @abstractmethod
    async def send_command(self, power_w: float) -> None: ...
    @property
    @abstractmethod
    def capabilities(self) -> BatteryCapabilities:
        """Max charge/discharge, signed/unsigned interface, etc."""

class GridMeterDriver(Driver):
    @abstractmethod
    async def read_power(self) -> float: ...

class PVDriver(Driver):
    @abstractmethod
    async def read_power(self) -> float: ...
```

Capabilities are machine-readable hardware specs. Useful for
aggregators (Phase 7) that need to distribute commands proportional
to capacity.

**Step 5.2 — Refactor existing HA-API drivers**

Make Phase 2's HA-based drivers conform to the new interfaces:
- `HAGridMeterDriver` → implements `GridMeterDriver`
- `HABatteryDriver` → implements `BatteryDriver`

Behavior unchanged, type interface added.

**Step 5.3 — First native-MQTT driver**

Implement `ZendureNativeMQTTDriver` as a proof of concept for
non-HA-mediated drivers:

- Subscribes directly to the Zendure device's MQTT topics on the
  user's broker (or the device's local broker)
- Sends commands via direct MQTT, bypassing HA service calls
- All Zendure-specific protocol code (AC mode management, output/input
  limit dedup, etc.) lives here
- The RelayStateMachine from `logic/relay_sm.py` is used the same way

This driver coexists with `HABatteryDriver`. The user picks one
based on their setup (some users have the battery only exposed via
HA, others have direct MQTT access).

**Step 5.4 — Topic standardization**

All drivers publish to canonical sensor topics:

```
zfi/1/sensors/grid/power
zfi/1/sensors/battery/<id>/state
zfi/1/sensors/pv/power
zfi/1/sensors/battery/<id>/capabilities  (Q-P5-A)
```

Controllers and detectors subscribe to these abstract topics, never
to driver-specific ones. Swapping the driver doesn't change the
controller.

### What works after Phase 5

- Adding a new battery vendor = implement `BatteryDriver` subclass
- Two transports for the same hardware (HA-mediated and native-MQTT)
  coexist
- Controllers don't know whether the battery is Zendure, Growatt, etc.,
  nor whether it's HA-mediated or native-MQTT
- Capabilities make resource-aware aggregation possible (Phase 7)

### What does NOT work yet

- Configuration-driven driver selection (Phase 6)
- Multiple batteries (Phase 7)

### Architecture questions raised

- **Q-P5-A**: Should `BatteryCapabilities` be published as a
  bus topic (`zfi/1/sensors/battery/<id>/capabilities`)? Yes —
  this lets aggregators discover capabilities at runtime, supports
  hot-add of new drivers. Add to ADR-3 topic taxonomy.

---

## Phase 6: Configuration-Driven Assembly

**Duration**: 2 weeks
**Prerequisite**: Phase 5 complete
**Goal**: Replace hardcoded `main.py` wiring with Graph Builder;
let users configure setups via the Add-on options form without
code changes

### What gets built

- `GraphBuilder` (per ADR-6)
- Per-instance configuration support
- Add-on Config Schema with dynamic-shaped lists (per ADR-5)
- System YAML in the Add-on for component type registry (per ADR-5)
- `Graph` object as runtime topology, queryable
- Optional: minor extension of the Custom Integration to publish
  graph topology as a service or attribute (for Tier 3 later)

### Concrete Steps

**Step 6.1 — System config (developer YAML, in Add-on container)**

```yaml
# /app/config/system.yaml — shipped with the Add-on
component_types:
  drivers:
    zendure_2400ac_native: zfi.drivers.zendure_native.ZendureNativeMQTTDriver
    ha_battery: zfi.drivers.ha.HABatteryDriver
    ha_grid_meter: zfi.drivers.ha.HAGridMeterDriver
  controllers:
    normal: zfi.controllers.normal.NormalController
    stove: zfi.controllers.stove.StoveController
  detectors:
    stove: zfi.detectors.stove.StoveDetector
  supervisors:
    rule_based: zfi.supervisors.rule_based.RuleBasedSupervisor
```

**Step 6.2 — User config (Add-on options schema)**

Per ADR-5, the user-facing configuration is the Add-on Config Schema
edited in HA Supervisor:

```yaml
# Add-on config.yaml — Supervisor renders this as a form
options:
  hardware:
    grid_meter:
      type: ha_grid_meter
      entity: sensor.smart_meter_power
    batteries:
      - name: "Living Room"
        type: ha_battery
        soc_entity: sensor.zendure_1_soc
        battery_power_entity: sensor.zendure_1_battery_power
        output_entity: number.zendure_1_outputlimit
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
      type: list(ha_grid_meter|zendure_2400ac_native)
      entity: str?
    batteries:
      - name: str
        type: list(ha_battery|zendure_2400ac_native)
        soc_entity: str?
        battery_power_entity: str?
        output_entity: str?
        max_charge: int(0,5000)
  modes:
    detectors:
      - type: list(stove|ev|low_battery)
        entity: str
        threshold: int(0,10000)
    controllers:
      - list(normal|stove|ev_priority)
```

The Supervisor validates against this schema. Saved options are
exposed to the engine as `/data/options.json`.

**Step 6.3 — Graph Builder**

```python
class GraphBuilder:
    def __init__(self, system_cfg: dict, user_cfg: dict, bus: BusClient):
        self.system_cfg = system_cfg
        self.user_cfg = user_cfg
        self.bus = bus

    def build(self) -> Graph:
        components = []
        # 1. Drivers
        for spec in self.user_cfg["hardware"]["batteries"]:
            cls = self._resolve("drivers", spec["type"])
            driver = cls(name=spec["name"], bus=self.bus, **spec)
            components.append(driver)
        # 2. Auto-aggregator if N>1 (Phase 7)
        # 3. Detectors
        # 4. Controllers (Multiplexer + N Controllers)
        # 5. Supervisor
        return Graph(components)
```

**Step 6.4 — Engine entry point simplification**

```python
# engine/main.py
async def main():
    user_cfg = json.load(open("/data/options.json"))
    system_cfg = yaml.safe_load(open("/app/config/system.yaml"))

    bus = BusClient(...)
    await bus.connect()

    builder = GraphBuilder(system_cfg, user_cfg, bus)
    graph = builder.build()

    for component in graph.topological_order():
        await component.start()

    await asyncio.Event().wait()
```

The entry point no longer knows about specific component types.

**Step 6.5 — Graph as inspection target**

The `Graph` object is queryable for the future visualization
(REQ-11):

```python
graph.list_components() -> list[ComponentInfo]
graph.connections() -> list[Edge]  # who subscribes to whom
```

Phase 6 just exposes this; the visualization itself is Tier 3.
Optionally, the Custom Integration in HA can expose graph
topology as a sensor attribute or service for ad-hoc inspection.

### What works after Phase 6

- User adds devices via standard HA Add-on configuration form
- Adding a new component type = code + system YAML update inside
  the Add-on (rebuild + Add-on update)
- Topology is inspectable at runtime
- Multi-instance setups are configurable (basis for Phase 7)
- A user without coding skills can reconfigure their setup

### What does NOT work yet

- Aggregators not auto-inserted (Phase 7)
- No multi-battery test in production

### Architecture questions raised

- **Q-P6-A**: How does the Add-on schema know which component types
  to offer? Resolved: the schema lists allowed types statically.
  When a new component type is added (code + system.yaml), the
  Add-on schema is updated in the same release.

- **Q-P6-B**: Validation order — does the Add-on schema validate
  per-field on save or all-at-once? Resolved: HA Supervisor validates
  on save (all-at-once). Per-field validation happens client-side
  via the schema's type declarations.

- **Q-P6-C**: How dynamic can the Add-on schema be? It does not
  support conditional fields (e.g., showing fields only for certain
  driver types). For Phase 6 this is acceptable: the user fills in
  irrelevant fields as empty strings, and the engine ignores them.
  If conditional UI becomes necessary, a Config Flow in the Custom
  Integration can be added as a complementary path (see ADR-5
  alternatives).

---

## Phase 7: Aggregator (When Needed)

**Duration**: 2 weeks (when triggered)
**Prerequisite**: User has 2+ batteries OR architectural readiness
demanded
**Goal**: Multi-battery support via auto-inserted Aggregator

### Trigger

Phase 7 is **on-demand**. As long as the user has one battery, the
Aggregator is unnecessary. The first user with multiple batteries
triggers this phase. Could be the original developer or a
community contributor.

### What gets built

- `BatteryAggregator` component
- Graph Builder auto-inserts Aggregator when N>1 of a driver type
- Distribution strategy (proportional, SOC-balanced, etc.) —
  implementation detail

### Concrete Steps

**Step 7.1 — Aggregator base + concrete implementation**

```python
class Aggregator(Component):
    """Fans out commands to N drivers, aggregates their states."""

class BatteryAggregator(Aggregator):
    def __init__(self, bus, drivers: list[BatteryDriver]):
        ...

    async def start(self):
        # Subscribe to each driver's state
        for d in self.drivers:
            await self.bus.subscribe(
                f"zfi/1/sensors/battery/{d.name}/state",
                self._on_state,
            )
        # Subscribe to combined command
        await self.bus.subscribe(
            "zfi/1/control/battery/aggregate/cmd",
            self._on_command,
        )

    async def _on_state(self, msg):
        # Recompute aggregate, publish
        ...

    async def _on_command(self, msg):
        # Distribute to drivers
        ...
```

**Step 7.2 — Graph Builder auto-insertion**

```python
# In GraphBuilder.build():
batteries = [c for c in components if isinstance(c, BatteryDriver)]
if len(batteries) > 1:
    agg = BatteryAggregator(bus=self.bus, drivers=batteries)
    components.append(agg)
    # Re-route controller commands to aggregate topic
```

**Step 7.3 — Controller transparency**

Controllers continue to publish to `zfi/1/control/desired_power`.
The MUX routes to either:
- A single driver (when N=1)
- The Aggregator (when N>1)

This is configured by the Graph Builder; controllers don't see it.

### What works after Phase 7

- Multi-battery setups work
- Distribution strategies pluggable
- All previous phases unaffected

### Architecture questions raised

- **Q-P7-A**: Distribution strategy as part of Aggregator config
  or as a separate strategy object? Strategy object — pluggable.

---

## What Becomes Possible When (Tier 1 Phase Map)

After Phase 1: HA-only Tier 2 skeleton can start (visualization
prototyping using bus data).

After Phase 2: Tier 2 component patterns can be designed (services
follow the same lifecycle as Tier 1 components).

After Phase 4: Auto-tuner prototype possible (logs → analysis).

After Phase 5: Multi-vendor support broadens user base.

After Phase 6: Tier 2 work can begin in earnest — Tier 1 has stable
configuration and component contracts.

## Coarse Timeline (Tier 1 + Brief Tier 2/3 Mention)

```
Months 1-2:    Phases 1-2  (foundation, lift-and-shift)
                 → Working bus-based system
Months 3-4:    Phases 3-4  (multi-controller, supervision)
                 → System reacts to situations
Months 5-6:    Phases 5-6  (driver abstraction, config-driven)
                 → User-friendly setup, hardware-agnostic
Month 7+:      Phase 7 when triggered

Tier 2 work begins around Month 5 (parallel to Phase 5-6):
   recorder service, time-series storage, basic web UI

Tier 3 work begins around Month 7-8:
   visualization frontend, decision transparency views

These are tentative — Tier 1 progress is the gating factor. Tier 2
and Tier 3 stay completely out of scope of this document beyond
this brief mention.
```

---

## Architecture Questions Raised in This Plan

These are questions surfaced during phase planning that affect
the architecture document:

| Q | Phase | Topic | Resolution |
|---|---|---|---|
| Q-P1-A | 1 | HA token for Supervisor vs. long-lived | Use Supervisor token via `http://supervisor/core/api` |
| Q-P1-B | 1 | Custom Integration broker credentials | Read from `/share/zfi/credentials.json` written by Add-on |
| Q-P1-C | 1 | Static `main.py` wiring | Replaced by GraphBuilder in Phase 6 |
| Q-P2-A | 2 | Component instance naming | `<type>.<instance_id>`, resolved by GraphBuilder |
| Q-P3-A | 3 | MUX knows controllers how? | Hardcoded list → GraphBuilder |
| Q-P3-B | 3 | MUX validates active controller | Yes — defensive, add to ADR-2 detail |
| Q-P4-A | 4 | Rule format location | Hardcoded → user config (Phase 6) |
| Q-P5-A | 5 | Capabilities as bus topic | Yes — add to ADR-3 |
| Q-P6-A | 6 | Add-on schema component discovery | Schema lists allowed types statically |
| Q-P6-B | 6 | Add-on schema validation timing | All-at-once on save (Supervisor default) |
| Q-P6-C | 6 | Conditional Add-on schema fields | Not supported; engine ignores irrelevant fields |
| Q-P7-A | 7 | Distribution strategy | Pluggable strategy object |

These should be reviewed against `architecture_design.md` and
folded back as ADR amendments where they add real architectural
content (e.g., Q-P5-A becomes part of ADR-3).
