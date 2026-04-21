# Forecast-Based min_soc — Implementation Spec

## Goal

Prevent pointless deep discharge on days when PV won't deliver
enough to recharge. Single rule: if tomorrow looks bad (< 1.5 kWh),
stop discharging at 30% instead of 10%.

No complex optimization. No intraday headroom calculation.
The existing PI controller handles all real-time decisions.

## Integration Points

```
Forecast.Solar sensors (per plane, summed)
            │
            ▼
HA Automation (runs 21:00 + 06:00)
            │
            ▼
input_number.zfi_min_soc (10 or 30)
            │
            ▼
Controller reads min_soc from HA entity
(yaml min_soc / max_soc still apply as hard clamps)
```

## Controller Change

`ControlLogic` currently reads `min_soc_pct` from `Config` (fixed at
startup). Change to: read `input_number.zfi_min_soc` every cycle.

In `ZeroFeedInController._read_measurement()`, add:

```python
dynamic_min_soc = self._read_float("input_number.zfi_min_soc")
if dynamic_min_soc is not None:
    # Clamp to configured hard limits
    self.logic.cfg.min_soc_pct = max(
        HARD_MIN_SOC,
        min(dynamic_min_soc, self.logic.cfg.max_soc_pct)
    )
```

Where `HARD_MIN_SOC` is the yaml-configured floor (e.g. 5%).

Alternative (cleaner): pass `min_soc` as part of `Measurement`
and let `ControlLogic._apply_guards` use it from there. Then Config
keeps the hard clamp, Measurement carries the dynamic value.

```python
@dataclass
class Measurement:
    ...
    dynamic_min_soc_pct: float | None = None
```

In `_apply_guards`:
```python
effective_min = m.dynamic_min_soc_pct or self.cfg.min_soc_pct
if raw_limit > 0 and m.soc_pct <= effective_min:
    return ControlOutput.idle(...)
```

## PV Forecast Sensors

### Forecast.Solar

Creates per-plane sensors. With the free plan, each plane is a
separate integration entry. Entity names follow the pattern:

```
sensor.energy_production_tomorrow              # plane 1
sensor.energy_production_tomorrow_2            # plane 2
sensor.energy_production_tomorrow_3            # plane 3
```

Same for today's remaining:
```
sensor.energy_current_hour                     # plane 1
sensor.energy_current_hour_2                   # plane 2
```

The `_tomorrow` sensor gives total kWh for the full next day.

### Solcast (alternative)

Solcast sums all sites automatically:
```
sensor.solcast_pv_forecast_forecast_tomorrow   # already summed
```

### Summing Multiple Planes

Create a template sensor in `configuration.yaml`:

```yaml
template:
  - sensor:
      - name: "PV Forecast Tomorrow Total"
        unit_of_measurement: "kWh"
        state: >
          {{
            (states('sensor.energy_production_tomorrow') | float(0))
            + (states('sensor.energy_production_tomorrow_2') | float(0))
            + (states('sensor.energy_production_tomorrow_3') | float(0))
          }}
        availability: >
          {{
            states('sensor.energy_production_tomorrow') not in ['unknown', 'unavailable']
          }}
```

Adapt entity names and number of planes to your setup.

## Automation

```yaml
automation:
  - alias: "ZFI: Adjust min SOC from forecast"
    id: zfi_forecast_min_soc
    trigger:
      - platform: time
        at: "21:00"
      - platform: time
        at: "06:00"
    condition:
      - condition: state
        entity_id: sensor.pv_forecast_tomorrow_total
        state:
          - "unknown"
          - "unavailable"
        match: none
    action:
      - variables:
          forecast_kwh: >
            {{ states('sensor.pv_forecast_tomorrow_total') | float(3) }}
          min_soc: >
            {% if forecast_kwh < 1.5 %}
              30
            {% else %}
              10
            {% endif %}
      - service: input_number.set_value
        target:
          entity_id: input_number.zfi_min_soc
        data:
          value: "{{ min_soc }}"
      - service: logbook.log
        data:
          name: "ZFI Forecast"
          message: >
            forecast={{ forecast_kwh }}kWh → min_soc={{ min_soc }}%
```

### Why 06:00, 15:00, and 20:00

- **06:00**: Morning check. If forecast is bad, set min_soc to **50%** —
  prevents discharging during the day when PV won't recharge.
- **15:00**: Afternoon check. Forecast.Solar updates during the day.
  If forecast is bad, set min_soc to **30%** (night buffer).
  If it improved, reset to 10%.
- **20:00**: Evening check. Final adjustment before night.
  Bad forecast → 30%, good → 10%.

### Why 1.5 kWh threshold

```
Battery: 2.88 kWh usable (10%–85% = 75% = 2.16 kWh)

1.5 kWh forecast:
  After house consumption (60-70% of PV on a bad day):
  Battery gets maybe 0.5-0.7 kWh = 17-24% SOC gain
  Starting from 10% → end of day at 27-34%
  → Barely above overnight starting point. Whole cycle was pointless.

Starting from 30% → end of day at 47-54%
  → Battery stayed in mid-range, less stress, same net benefit.
```

The threshold should be adjusted after a few weeks of observing
actual PV yield vs forecast accuracy at your location.

## What This Does NOT Do

- No intraday adjustment (controller handles real-time via surplus)
- No max_soc optimization (stays fixed at 85%)
- No dynamic tariff integration
- No load prediction
- No complex headroom calculation

It's one `if/else` that runs twice a day.

## input_number Entity Setup

Already created. For reference, the expected configuration:

```
Name: ZFI Min SOC
Entity ID: input_number.zfi_min_soc
Min: 5
Max: 50
Step: 5
Initial: 10
Unit: %
```

## File Summary

| What | Where |
|---|---|
| Template sensor (sum planes) | `configuration.yaml` |
| Automation (set min_soc) | `automations.yaml` or UI |
| Controller reads min_soc | `zero_feed_in_controller.py` (small change) |
| Driver | No changes |
