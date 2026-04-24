# PV Forecast Manager — `pv_forecast_manager.py`

Adjusts the controller's minimum SOC based on tomorrow's PV forecast. Prevents pointless deep discharge on days when PV won't deliver enough to recharge.

---

## How It Works

Evaluates at configurable times (default: 06:00, 15:00, 20:00). Sums tomorrow's forecast across all configured Forecast.Solar entities. If the total is below the threshold (default 1.5 kWh), applies time-of-day min SOC rules:

| Time | Low forecast min SOC | Rationale |
| --- | --- | --- |
| Before 12:00 | 50% | Prevent daytime discharge when PV won't recharge |
| Before 23:59 | 30% | Preserve a night buffer |

When the forecast is adequate, resets to the base min SOC (matches the controller's `min_soc`).

---

## Published HA Sensors

| Entity | Type | Unit | Description |
| --- | --- | --- | --- |
| `sensor.zfi_dynamic_min_soc` | number | % | Forecast-adjusted min SOC |
| `sensor.zfi_forecast_kwh` | number | kWh | Tomorrow's forecast total (debug) |

---

## Configuration Reference

See `apps.yaml.example` section 2 (PV Forecast Manager) for all parameters.
