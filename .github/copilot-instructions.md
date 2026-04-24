# Zero Feed-In — Copilot Instructions

## Project Overview

AppDaemon apps for a Zendure SolarFlow 2400 AC+ implementing bidirectional zero feed-in control on Home Assistant. Read these files at the start of every conversation:

- `docs/architecture.md` — system architecture, design decisions, data flow, installation, file structure
- `docs/controller.md` — controller logic, surplus estimation, direct calculation, muting, flowcharts
- `docs/driver.md` — driver, AC mode, relay lockout, power limits, flowchart
- `docs/pulse_load_detector.md` — pulse-load detection (sign-flip algorithm)
- `docs/pulse_load_filter.md` — pulse-load mitigation (baseline estimation, measurement pause)
- `docs/pv_forecast_manager.md` — PV forecast and time-of-day rules

## File Structure

```
src/                          # Application source code
  zero_feed_in_controller.py  # Device-agnostic direct calculation controller + ControlLogic
  zendure_solarflow_driver.py # Zendure SolarFlow driver with relay state machine
  pulse_load_detector.py      # Pulse-load detection (sign-flip algorithm)
  pulse_load_filter.py        # Pulse-load mitigation (baseline estimation)
  pv_forecast_manager.py      # PV forecast and time-of-day rules
  csv_logger.py               # CSV logging utility
  solarflow_mqtt_watchdog.py  # MQTT reconnect watchdog (HTTP API trigger)
  zfi_watchdog.esphome        # ESPHome package: independent ESP watchdog (.esphome to avoid AppDaemon scan)
config/
  apps.yaml                   # AppDaemon configuration (uses !secret references)
  secrets.yaml                # Device credentials — git-ignored, not committed
  secrets.yaml.example        # Template for secrets.yaml
tests/                        # Unit tests (one per module)
docs/
  architecture.md             # System architecture and design decisions
  controller.md               # Controller documentation
  driver.md                   # Driver documentation
  pulse_load_detector.md      # Pulse-load detector documentation
  pulse_load_filter.md        # Pulse-load filter documentation
  pv_forecast_manager.md      # PV forecast documentation
  DASHBOARDS.md               # Lovelace dashboard guide
```

## Code Conventions

- Python 3.10+ (dataclasses, enums, `X | None` union syntax)
- AppDaemon `hass.Hass` base class for HA integration
- Signed power convention: **+discharge / -charge** everywhere
- Constants as module-level `UPPER_SNAKE_CASE`
- Configuration via `@dataclass` with `from_args(cls, args)` classmethod
- State published to HA via `set_state()` as `sensor.zfi_*` entities
- No external dependencies beyond AppDaemon

## Docstrings & Type Annotations (mandatory)

**Every public class, method, and function must have a docstring.** Private methods (`_foo`) must also have at least a one-line docstring describing their purpose.

**All function signatures must have full type annotations** — parameters and return types. Use `X | None` union syntax (not `Optional[X]`).

Dataclass fields should have per-field docstrings (triple-quoted string after the field) explaining units, sign conventions, and edge cases.

When modifying existing code, add missing docstrings and type annotations to any function or class you touch in the same commit.

## Documentation Rule

**Keep docs up to date.** Whenever a code change affects behavior, architecture, configuration, or published sensors, update the relevant doc in `docs/` in the same commit. Do not defer documentation to a follow-up step.

## Device Constraints

- Device response latency: **10–15 seconds** (not 2–4 s)
- AC mode relay physically switches — minimize switching to avoid wear
- `outputLimit` / `inputLimit` are write-only from driver perspective
- MQTT integration overwrites HA entity state faster than relay switches — driver tracks its own intent
- Power rounded to 5 W steps

## Git

- Single `main` branch, remote `origin` → `github.com:steffenruehl/ha-zero-feed-in.git`
- Commit messages: imperative mood, summary line + blank line + body for non-trivial changes
