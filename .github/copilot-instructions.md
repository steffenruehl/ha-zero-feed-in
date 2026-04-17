# Zero Feed-In — Copilot Instructions

## Project Overview

Two AppDaemon apps for a Zendure SolarFlow 2400 AC+ implementing bidirectional zero feed-in control on Home Assistant. Read these files at the start of every conversation:

- `docs/development_context.md` — architecture decisions, hardware setup, key concepts, code organization
- `docs/zero_feed_in_docs.md` — full documentation: PI controller, driver, protection mechanisms, tuning guide, flowcharts

## File Structure

```
src/                          # Application source code
  zero_feed_in_controller.py  # Device-agnostic PI controller + ControlLogic
  zendure_solarflow_driver.py # Zendure SolarFlow driver with relay state machine
config/
  apps.yaml                   # AppDaemon configuration for both apps
tests/
  test_zero_feed_in_controller.py  # Unit tests for ControlLogic & PIController
  test_zendure_solarflow_driver.py  # Unit tests for AdaptiveLockout & RelayStateMachine
docs/
  development_context.md      # Architecture decisions and design rationale
  zero_feed_in_docs.md        # Full technical documentation
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

**Keep docs up to date.** Whenever a code change affects behavior, architecture, configuration, or published sensors, update `docs/development_context.md` and/or `docs/zero_feed_in_docs.md` in the same commit. Do not defer documentation to a follow-up step.

## Device Constraints

- Device response latency: **10–15 seconds** (not 2–4 s)
- AC mode relay physically switches — minimize switching to avoid wear
- `outputLimit` / `inputLimit` are write-only from driver perspective
- MQTT integration overwrites HA entity state faster than relay switches — driver tracks its own intent
- Power rounded to 10 W steps

## Git

- Single `main` branch, remote `origin` → `github.com:steffenruehl/ha-zero-feed-in.git`
- Commit messages: imperative mood, summary line + blank line + body for non-trivial changes

## Terminal

- Use `bash -c "..."` to wrap terminal commands (avoids zsh autocorrect issues)
- The system has `python3` (not `python`)
- Run tests: `bash -c "cd /home/ruehl/src/zero-feed-in && python3 -m pytest tests/ -v"`
