# PID + Feed-Forward Enhancement — Implementation Spec

## Context

The zero feed-in system consists of two AppDaemon apps:

1. **Controller** (`zero_feed_in_controller.py`) — device-agnostic PI
   controller. Reads grid, SOC, battery power sensors. Publishes
   `sensor.zfi_desired_power` (signed W: +discharge, −charge).

2. **Driver** (`zendure_solarflow_driver.py`) — device-specific.
   Reads desired power, manages relay state machine, sends
   outputLimit/inputLimit/acMode to the Zendure SolarFlow 2400 AC+.

This spec adds three enhancements. Each must go in the correct app:

| Enhancement | Where | Why |
|---|---|---|
| D-term (PID) | Controller (`ControlLogic`) | Reacts to grid signal, part of regulation |
| PV feed-forward | Controller (`ControlLogic`) | Reacts to PV disturbance, pre-error compensation |
| Slew rate limiter | Controller (`ControlLogic`) | Prevents PI windup from commanding beyond device capability |

All three modify `ControlLogic` only — the driver is unchanged.
The driver's relay SM already handles physical device constraints.

## Available Sensors

| Sensor | Example entity | Read by | Notes |
|---|---|---|---|
| Grid power | `sensor.smart_meter_...` | Controller | + draw, − feed-in |
| SOC | `sensor..._electriclevel` | Controller | 0-100% |
| Battery power | `sensor..._net_power` | Controller | +discharge, −charge (signed) |
| PV power | `sensor.q_home_pv_power` (TBD) | Controller | Always ≥ 0 (NEW) |

## Current Pipeline (ControlLogic.compute)

```python
def compute(self, m: Measurement) -> ControlOutput:
    surplus = estimate_surplus(m)          # -battery_power - grid
    emergency check                        # feed-in > 800W?
    update_operating_mode(surplus)          # Schmitt trigger
    target = target_for_mode()             # 30W or 0W
    error = grid - target
    raw, p, new_i = _run_pi(error)         # PI with deadband
    guarded = _apply_guards(raw, m, surplus)
    if guarded: return guarded             # SOC, surplus, switch checks
    commit integral
    clamped = _clamp(raw, surplus)          # surplus cap + power limits
    back-calculate integral if clamped != raw
    return ControlOutput(clamped, ...)
```

## Enhanced Pipeline

```python
def compute(self, m: Measurement) -> ControlOutput:
    surplus = estimate_surplus(m)
    emergency check

    update_operating_mode(surplus)
    target = target_for_mode()
    error = grid - target

    pid_out, p, d, new_i = _run_pid(error, m.grid_power_w)    # PI + D
    ff_pv = _compute_pv_feed_forward(m.pv_power_w)             # PV FF
    combined = pid_out + ff_pv
    slew_limited = _apply_slew_limit(combined)                  # slew

    guarded = _apply_guards(slew_limited, m, surplus)
    if guarded: return guarded
    commit integral
    clamped = _clamp(slew_limited, surplus)
    back-calculate integral if clamped != slew_limited
    return ControlOutput(clamped, ...)
```

## Signal Flow

```
PV sensor ──────────────▸ FF (PV) ──▸ ff_pv_term ──╮
                                                     ╲
Grid sensor ──▸ error ──▸ PID (P+I+D) ──▸ pid_out ──┤──▸ combined
                                                     │
                                                     ▼
                                               slew limiter
                                                     │
                                                     ▼
                                               guards + clamp
                                                     │
                                                     ▼
                                          sensor.zfi_desired_power
                                                     │
                                                     ▼
                                          Driver (relay SM, device commands)
```

## Enhancement 1: D-Term (PID)

### What

Add a derivative term to the existing PI, making it PID. The D-term
reacts to the rate of change of grid power — load steps produce a
large instantaneous delta that D catches immediately.

### Where in Code

`ControlLogic` — modify `_run_pi` → `_run_pid`.

### State Changes

Add to `Measurement`:
```python
# No change needed — grid_power_w already present
```

Add to `ControllerState`:
```python
previous_grid_w: float | None = None
```

### New Config Fields

```python
kd: float = 0.3
"""Derivative gain on grid power delta."""

kd_deadband_w: float = 30.0
"""Ignore grid changes smaller than this (noise filter)."""
```

Asymmetric Kd (kd_up / kd_down) is possible but likely unnecessary
for v1 — D acts on delta magnitude, not on direction.

### Implementation

```python
def _compute_d_term(self, grid_power: float) -> float:
    prev = self.state.previous_grid_w
    if prev is None:
        return 0.0

    delta = grid_power - prev
    if abs(delta) < self.cfg.kd_deadband_w:
        return 0.0

    return self.cfg.kd * delta
```

D-term is added to the PI output BEFORE guards and clamping.
It is NOT committed to the integral — it's a transient correction.

### Modified _run_pid

```python
def _run_pid(self, error: float, grid_power: float
) -> tuple[float, float, float, float]:
    """Returns (output, p_term, d_term, new_integral)."""

    d_term = self._compute_d_term(grid_power)

    if abs(error) <= self.cfg.deadband_w:
        # Even in deadband, D can fire (large load step crosses deadband)
        if d_term == 0:
            return self.state.last_computed_w, 0.0, 0.0, self.state.integral
        # D-only correction while PI is frozen
        return self.state.last_computed_w + d_term, 0.0, d_term, self.state.integral

    pi_output, p_term, new_integral = self.pi.update(
        error=error,
        integral=self.state.integral,
        dt=self.cfg.interval_s,
    )
    return pi_output + d_term, p_term, d_term, new_integral
```

Note: D fires even when PI is in deadband. A large load step that
pushes error from 0 to +500 creates delta_grid=+500 in one cycle.
The deadband check on `error` would catch it too (500 > 35), but if
error was already at +20 (inside deadband) and jumps to +520, the PI
was frozen — D catches the step immediately.

### State Update

At end of `_on_tick` (in the HA adapter):
```python
self.logic.state.previous_grid_w = m.grid_power_w
```

## Enhancement 2: PV Feed-Forward

### What

True feed-forward from a PV production sensor. Compensates for solar
changes (clouds, sunset) before they appear at the grid meter.

### Why It's Different From D

| | D-term | PV feed-forward |
|---|---|---|
| Input | Grid delta | PV delta |
| Reacts to | Effect (grid changed) | Cause (PV changed) |
| Timing | Same cycle as error | 1-2s before grid change |
| Catches | Load steps (kettle) | Solar changes (clouds) |
| Misses | Solar changes | Load steps |

### Where in Code

`ControlLogic` — new method `_compute_pv_feed_forward`.

### State Changes

Add to `Measurement`:
```python
pv_power_w: float | None = None
"""PV production (W). Always >= 0. None if sensor not configured."""
```

Add to `ControllerState`:
```python
previous_pv_w: float | None = None
```

### New Config Fields

```python
pv_sensor: str = ""
"""HA entity for PV power (W). Empty = disabled."""

ff_pv_gain: float = 0.6
"""Fraction of PV change applied as correction. 0 = disabled."""

ff_pv_deadband_w: float = 20.0
"""Ignore PV changes below this (MPPT noise filter)."""
```

### Implementation

```python
def _compute_pv_feed_forward(self, pv_power: float | None) -> float:
    """True feed-forward from PV production changes.

    PV drops → positive term (need more discharge / less charge)
    PV rises → negative term (can charge more / discharge less)
    """
    if pv_power is None:
        return 0.0

    prev = self.state.previous_pv_w
    if prev is None:
        return 0.0

    delta = pv_power - prev
    if abs(delta) < self.cfg.ff_pv_deadband_w:
        return 0.0

    return -self.cfg.ff_pv_gain * delta
```

Note the negative sign: PV drops (delta < 0) → ff > 0 → increase
discharge / decrease charge. Opposite of PV change direction.

### HA Adapter Changes

In `_read_measurement`, add PV reading:
```python
pv = self._read_float(self.cfg.pv_sensor) if self.cfg.pv_sensor else None
# ... add to Measurement:
m = Measurement(..., pv_power_w=pv)
```

State update at end of tick:
```python
self.logic.state.previous_pv_w = m.pv_power_w
```

### Surplus Estimation

PV sensor does NOT change surplus estimation. Proof:
```
surplus = pv - house_load
        = pv - (grid + pv + battery_power)
        = -grid - battery_power
```
PV cancels out. The existing formula using battery_power is already correct.
The PV sensor's value is purely in the feed-forward timing advantage.

## Enhancement 3: Slew Rate Limiter

### What

Limits how fast `desired_power` can change per cycle, matching the
device's physical ramp rate. Prevents PI integral windup when the
controller commands a large step that the device can only deliver
gradually.

### Where in Code

`ControlLogic` — new method `_apply_slew_limit`. Applied after PID+FF
combination, before guards.

### Why in Controller, Not Driver

The driver's relay SM handles direction *changes* (IDLE↔CHARGE↔DISCHARGE).
The slew limiter handles power *ramp rate* within a direction. These
are independent constraints:

- Relay SM: "can we switch from charging to discharging?" (10-30s energy accumulator)
- Slew limit: "can we go from 200W to 800W discharge?" (device ramp rate)

If slew limiting were in the driver, the controller's PI would still
see the full error and wind up the integral, because from the
controller's perspective it commanded 800W but the grid doesn't respond
(device is still ramping). The controller must limit its own commands.

### How to Measure Slew Rate

Two step tests with relay already closed:
```
Test A: 200W → 400W (Δ=200W), record grid settle time t_a
Test B: 200W → 800W (Δ=600W), record grid settle time t_b

If t_b ≈ 3 × t_a:  slew rate = 200 / t_a  (W/s)
If t_b ≈ t_a:       PT1, slew_rate = 0 (disabled)
```

### New Config Fields

```python
slew_rate_w_per_s: float = 0.0
"""Maximum output change rate (W/s). 0 = disabled. Measure from step response."""
```

### Implementation

```python
def _apply_slew_limit(self, raw: float) -> float:
    if self.cfg.slew_rate_w_per_s <= 0:
        return raw

    max_delta = self.cfg.slew_rate_w_per_s * self.cfg.interval_s
    delta = raw - self.state.last_computed_w

    if abs(delta) <= max_delta:
        return raw

    sign = 1.0 if delta > 0 else -1.0
    return self.state.last_computed_w + sign * max_delta
```

### Back-Calculation Interaction

When slew limiting activates, the clamped output differs from PI output.
The existing back-calculation in `compute()` handles this:
```python
if clamped != raw_limit:
    self.state.integral = clamped - p_term
```
This already applies after `_clamp()`. The slew limiter output feeds
into the same path — no additional anti-windup logic needed.

## Updated ControlOutput

Add D-term and FF-term for logging and sensor publishing:

```python
@dataclass
class ControlOutput:
    desired_power_w: float
    p_term: float
    d_term: float          # NEW
    i_term: float
    ff_pv_term: float      # NEW
    reason: str
```

Update `from_raw` and `idle` accordingly.

## Updated Log Format

```
LIVE | grid=350W surplus=-200W soc=65% mode=DISCHARGING
discharge=370W | P=160 I=180 D=30 FF=0 | Discharge (DISCHARGING)
```

## New Published Sensors (debug=true)

```python
self._set_sensor("d_term", round(output.d_term), "W", "mdi:delta")
self._set_sensor("ff_pv", round(output.ff_pv_term), "W", "mdi:solar-power-variant")
self._set_sensor("pv_power", round(m.pv_power_w or 0), "W", "mdi:solar-panel")
self._set_sensor("slew_limited", 1 if slew_was_active else 0, icon="mdi:speedometer-slow")
```

## apps.yaml Additions

```yaml
zero_feed_in_controller:
  ...
  # PV sensor (optional, enables feed-forward)
  pv_sensor: sensor.q_home_pv_power

  # D-term
  kd: 0.3
  kd_deadband: 30

  # PV feed-forward
  ff_pv_gain: 0.6
  ff_pv_deadband: 20

  # Slew rate (0 = disabled, measure first)
  slew_rate: 0
```

No changes to the driver's apps.yaml section.

## Interaction Matrix

| Mechanism | D-term | PV-FF | Slew limiter |
|---|---|---|---|
| PI deadband | D can fire even in deadband | Independent deadband | Post-combine |
| Surplus clamp | After slew | After slew | Before guards |
| Guards | Output goes through guards | Through guards | Before guards |
| Emergency | Bypassed | Bypassed | Bypassed |
| Mode switch | Mode-agnostic | Mode-agnostic | Mode-agnostic |
| Back-calculation | Through existing path | Through existing path | Through existing path |
| Driver relay SM | Unaffected | Unaffected | Unaffected |

## Kp/Ki Tuning with Slew Rate Known

If device is slew-rate limited:
```
slew_rate = measured (W/s)
T1_effective = typical_step / slew_rate     (use ~200W)
Kp_up = interval / (2 × T1_effective)
Ki_up = Kp_up / (4 × T1_effective)
Kp_down = Kp_up × 1.5   (faster ramp down)
Ki_down = Ki_up × 1.5
```

With slew limiting enabled, Kp can be more aggressive because the
limiter prevents the PI from commanding faster than the device can
follow.

## Implementation Priority

1. **D-term** — smallest change to ControlLogic, biggest impact for load steps
2. **Slew limiter** — enables higher Kp safely, prevents windup
3. **PV feed-forward** — requires PV sensor entity verification first

Each can be implemented and tested independently. The driver needs
no changes for any of them.

## Test Plan

### 0. Measure Device Dynamics

With relay closed:
```
Test A: 200→400W, record settle time and shape
Test B: 200→800W, record settle time and shape
→ slew_rate (W/s) and whether PT1 or slew-limited
```

### 1. D-term Only (dry_run: true)

```yaml
kd: 0.3
ff_pv_gain: 0
slew_rate: 0
```
Turn kettle on/off. Check `sensor.zfi_d_term` shows spikes.

### 2. PV-FF Only (dry_run: true)

```yaml
kd: 0
ff_pv_gain: 0.6
slew_rate: 0
```
Wait for clouds. Check `sensor.zfi_ff_pv` shows corrections.

### 3. Combined (dry_run: false, max_output: 200)

Enable all, ramp up gradually over days.
Compare settling times with PI-only baseline.
