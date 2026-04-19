# Four-Quadrant PI Gains — Implementation Spec

## Problem

The current PI controller selects gains by error sign (`kp_up`/`kp_down`).
Error sign maps to "need more power" vs "need less power." But the
device response time depends on **mode × direction**:

```
error > 0 ("need more power"):
  DISCHARGE mode → increase discharge   T1 ≈ 5.0s
  CHARGE mode    → decrease charge      T1 ≈ 5.5s

error < 0 ("need less power"):
  DISCHARGE mode → decrease discharge   T1 ≈ 3.5s
  CHARGE mode    → increase charge      T1 ≈ 7.5s  ← 2× slower!
```

A single `kp_down` serves both T1=3.5s (wants Kp=0.71) and T1=7.5s
(wants Kp=0.33). Any value is wrong for one case by a factor of 2.

## Measured T1 per Quadrant

Source: manual step response tests, relay already closed.

| Quadrant | Physical action | T1 (s) |
|---|---|---|
| discharge_up | increase discharge | 5.0 |
| discharge_down | decrease discharge | 3.5 |
| charge_up | increase charge | 7.5 |
| charge_down | decrease charge | 5.5 |

## Calculated Gains

SIMC: `Kp = interval / (2 × T1)`, `Ki = Kp / (4 × T1)`, interval = 5s.

| Quadrant | Kp | Ki |
|---|---|---|
| discharge_up | 0.50 | 0.025 |
| discharge_down | 0.71 | 0.051 |
| charge_up | 0.33 | 0.011 |
| charge_down | 0.45 | 0.021 |

## Selection Logic

```
                  error >= 0           error < 0

DISCHARGING       discharge_up         discharge_down
CHARGING          charge_down          charge_up
```

Note CHARGING is flipped: error >= 0 means "charging too much, reduce"
→ charge_down. Error < 0 means "surplus available, charge more"
→ charge_up.

## Changes

### New dataclasses

```python
@dataclass
class PIGains:
    """Kp/Ki pair for one physical quadrant."""
    kp: float
    ki: float

@dataclass
class PIGainSet:
    """Four gain sets: mode × direction."""
    discharge_up: PIGains
    discharge_down: PIGains
    charge_up: PIGains
    charge_down: PIGains

    def select(self, mode: OperatingMode, error: float) -> PIGains:
        if mode == OperatingMode.DISCHARGING:
            return self.discharge_up if error >= 0 else self.discharge_down
        else:
            return self.charge_up if error < 0 else self.charge_down
```

### Config

Remove `kp_up`, `kp_down`, `ki_up`, `ki_down`.

Add:
```python
kp_discharge_up: float = 0.50
kp_discharge_down: float = 0.71
kp_charge_up: float = 0.33
kp_charge_down: float = 0.45
ki_discharge_up: float = 0.025
ki_discharge_down: float = 0.051
ki_charge_up: float = 0.011
ki_charge_down: float = 0.021
```

Update `from_args` accordingly. For backward compatibility, if old-style
`kp_up`/`kp_down` keys are present, map them to all four quadrants:
```python
if "kp_discharge_up" in args:
    # new style
    ...
elif "kp_up" in args or "kp" in args:
    kp_up = float(args.get("kp_up", args.get("kp", 0.5)))
    kp_down = float(args.get("kp_down", args.get("kp", 0.5)))
    ki_up = float(args.get("ki_up", args.get("ki", 0.05)))
    ki_down = float(args.get("ki_down", args.get("ki", 0.05)))
    # old mapping: up/down = error sign
    kp_discharge_up = kp_up
    kp_discharge_down = kp_down
    kp_charge_up = kp_down      # error < 0 in charge → old kp_down
    kp_charge_down = kp_up      # error >= 0 in charge → old kp_up
    # same pattern for ki
```

### PIController

Remove stored gains from `__init__`. Accept `PIGains` per call:

```python
class PIController:
    def __init__(self, output_min: float, output_max: float) -> None:
        self.output_min = output_min
        self.output_max = output_max

    def update(self, error, integral, dt, gains: PIGains) -> tuple[float, float, float]:
        p_term = gains.kp * error
        new_integral = integral + gains.ki * error * dt
        output = p_term + new_integral
        # back-calculation anti-windup (unchanged)
        if output > self.output_max:
            new_integral = self.output_max - p_term
            output = self.output_max
        elif output < self.output_min:
            new_integral = self.output_min - p_term
            output = self.output_min
        return output, p_term, new_integral
```

### ControlLogic

In `__init__`, build `PIGainSet` from config.

In `_run_pi`, select gains before calling PI:
```python
def _run_pi(self, error):
    if abs(error) <= self.cfg.deadband_w:
        return self.state.last_computed_w, 0.0, self.state.integral

    gains = self.gains.select(self.state.mode, error)
    return self.pi.update(error, self.state.integral, self.cfg.interval_s, gains)
```

### Logging

Startup log should show all four quadrants:
```python
f"Gains | dis_up=({g.discharge_up.kp:.2f},{g.discharge_up.ki:.3f}) "
f"dis_dn=({g.discharge_down.kp:.2f},{g.discharge_down.ki:.3f}) "
f"chg_up=({g.charge_up.kp:.2f},{g.charge_up.ki:.3f}) "
f"chg_dn=({g.charge_down.kp:.2f},{g.charge_down.ki:.3f})"
```

Optional debug sensor: `sensor.zfi_pi_quadrant` → e.g. "discharge_up".

### Driver

No changes.
