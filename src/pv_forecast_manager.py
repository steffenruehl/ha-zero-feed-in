"""pv_forecast_manager.py – Adjusts dynamic min SOC based on PV forecast.

Replaces the HA automation + template sensor that were previously deployed
as separate YAML files (``ha_automation_forecast_min_soc.yaml`` and
``ha_template_sensors.yaml``).  Everything runs inside AppDaemon.

Published sensors
-----------------
``sensor.<prefix>_pv_forecast_total``
    Sum of all configured forecast entities (kWh).
``sensor.<prefix>_dynamic_min_soc``
    Computed minimum SOC percentage.  The zero-feed-in controller reads this
    via its ``dynamic_min_soc_entity`` configuration key.

Schedule
--------
Evaluation runs at each time in ``evaluation_times`` (default 06:00, 15:00,
20:00) **and** once at AppDaemon startup.

Logic
-----
1. Sum all ``forecast_entities`` → total forecast (kWh).
2. If total < ``forecast_threshold_kwh`` → apply time-of-day rules
   (``min_soc_rules``).
3. If total ≥ threshold → use ``min_soc_high_forecast_pct``.

Config (apps.yaml)::

    pv_forecast_manager:
      module: zero_feed_in.src.pv_forecast_manager
      class: PvForecastManager
      forecast_entities:
        - sensor.energy_production_tomorrow
        - sensor.energy_production_tomorrow_2
      forecast_threshold_kwh: 1.5
      min_soc_high_forecast_pct: 10
      min_soc_rules:
        - before: "12:00"
          soc: 50
        - before: "23:59"
          soc: 30
      evaluation_times:
        - "06:00"
        - "15:00"
        - "20:00"
      sensor_prefix: zfi
      debug: false
"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime

try:
    import appdaemon.plugins.hass.hassapi as hass

    _HASS_BASE = hass.Hass
except ImportError:
    _HASS_BASE = object  # type: ignore[assignment,misc]


# ── configuration ────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class MinSocRule:
    """A time-of-day rule for low-forecast min SOC.

    All times before ``before`` (HH:MM) get ``soc_pct`` as min SOC.
    Rules are evaluated in order; the first match wins.
    """

    before: str
    """Time threshold in HH:MM format (24h).  Inclusive upper bound."""

    soc_pct: float
    """Min SOC percentage to apply when this rule matches."""


@dataclass(frozen=True)
class PvForecastConfig:
    """Configuration for the PV Forecast Manager app."""

    forecast_entities: list[str]
    """List of Forecast.Solar (or equivalent) entity IDs to sum."""

    forecast_threshold_kwh: float = 1.5
    """Below this total forecast, the low-forecast rules apply."""

    min_soc_high_forecast_pct: float = 10.0
    """Min SOC when forecast >= threshold."""

    min_soc_rules: list[MinSocRule] = field(
        default_factory=lambda: [
            MinSocRule(before="12:00", soc_pct=50.0),
            MinSocRule(before="23:59", soc_pct=30.0),
        ]
    )
    """Time-of-day rules for low-forecast days.  Evaluated in order."""

    evaluation_times: list[str] = field(
        default_factory=lambda: ["06:00", "15:00", "20:00"]
    )
    """Times (HH:MM) at which the forecast is evaluated."""

    sensor_prefix: str = "zfi"
    """Prefix for published HA sensor entity IDs."""

    debug: bool = False
    """Enable debug logging."""

    # Defaults for fields using default_factory (not accessible as class attrs)
    _DEFAULT_RULES = [
        MinSocRule(before="12:00", soc_pct=50.0),
        MinSocRule(before="23:59", soc_pct=30.0),
    ]
    _DEFAULT_EVAL_TIMES = ["06:00", "15:00", "20:00"]

    @classmethod
    def from_args(cls, args: dict) -> PvForecastConfig:
        """Build config from AppDaemon ``args`` dict."""
        raw_rules = args.get("min_soc_rules", None)
        if raw_rules is not None:
            rules = [
                MinSocRule(before=str(r["before"]), soc_pct=float(r["soc"]))
                for r in raw_rules
            ]
        else:
            rules = list(cls._DEFAULT_RULES)

        raw_entities = args.get("forecast_entities", [])
        if not raw_entities:
            raise ValueError("forecast_entities must be a non-empty list")

        raw_times = args.get("evaluation_times", None)

        return cls(
            forecast_entities=list(raw_entities),
            forecast_threshold_kwh=float(
                args.get("forecast_threshold_kwh", cls.forecast_threshold_kwh)
            ),
            min_soc_high_forecast_pct=float(
                args.get("min_soc_high_forecast_pct", cls.min_soc_high_forecast_pct)
            ),
            min_soc_rules=rules,
            evaluation_times=list(raw_times) if raw_times else list(cls._DEFAULT_EVAL_TIMES),
            sensor_prefix=str(args.get("sensor_prefix", cls.sensor_prefix)),
            debug=bool(args.get("debug", cls.debug)),
        )


# ── pure logic (no HA imports, fully testable) ───────────────────────────────


def compute_min_soc(
    forecast_kwh: float,
    threshold_kwh: float,
    high_forecast_soc: float,
    rules: list[MinSocRule],
    now: datetime,
) -> float:
    """Return the min SOC percentage for the given forecast and time.

    Parameters
    ----------
    forecast_kwh:
        Total PV forecast in kWh.
    threshold_kwh:
        If ``forecast_kwh >= threshold_kwh``, return ``high_forecast_soc``.
    high_forecast_soc:
        Min SOC for good-forecast days.
    rules:
        Time-of-day rules for low-forecast days.  First match wins.
    now:
        Current time for rule evaluation.
    """
    if forecast_kwh >= threshold_kwh:
        return high_forecast_soc

    now_str = now.strftime("%H:%M")
    for rule in rules:
        if now_str < rule.before:
            return rule.soc_pct

    # No rule matched — fall back to last rule's SOC (or high_forecast default)
    return rules[-1].soc_pct if rules else high_forecast_soc


# ── AppDaemon app ────────────────────────────────────────────────────────────


class PvForecastManager(_HASS_BASE):  # type: ignore[misc]
    """Adjusts dynamic min SOC based on PV forecast.

    Publishes ``sensor.<prefix>_pv_forecast_total`` and
    ``sensor.<prefix>_dynamic_min_soc`` at scheduled times and on startup.
    """

    def initialize(self) -> None:
        """Set up scheduled evaluations and run an initial evaluation."""
        self.cfg = PvForecastConfig.from_args(self.args)

        # Schedule daily evaluations
        for time_str in self.cfg.evaluation_times:
            h, m = (int(x) for x in time_str.split(":"))
            self.run_daily(self._on_schedule, start=f"{h:02d}:{m:02d}:00")

        # Initial evaluation at startup
        self.run_in(self._on_schedule, 5)

        self.log(
            f"PvForecastManager ready | "
            f"entities={self.cfg.forecast_entities} "
            f"threshold={self.cfg.forecast_threshold_kwh} kWh "
            f"times={self.cfg.evaluation_times}"
        )

    def _on_schedule(self, kwargs: dict) -> None:
        """Evaluate forecast and publish sensors."""
        forecast_kwh = self._read_forecast_total()
        if forecast_kwh is None:
            self.log("Forecast entities unavailable — skipping evaluation", level="WARNING")
            return

        now = datetime.now()
        min_soc = compute_min_soc(
            forecast_kwh=forecast_kwh,
            threshold_kwh=self.cfg.forecast_threshold_kwh,
            high_forecast_soc=self.cfg.min_soc_high_forecast_pct,
            rules=self.cfg.min_soc_rules,
            now=now,
        )

        self._publish_sensors(forecast_kwh, min_soc)
        self.log(
            f"Forecast={forecast_kwh:.2f} kWh → min_soc={min_soc:.0f}%"
        )

    def _read_forecast_total(self) -> float | None:
        """Sum all forecast entities.  Returns None if any is unavailable."""
        total = 0.0
        for entity in self.cfg.forecast_entities:
            state = self.get_state(entity)
            if state in (None, "unknown", "unavailable"):
                if self.cfg.debug:
                    self.log(f"Entity {entity} is {state}")
                return None
            try:
                total += float(state)
            except (TypeError, ValueError):
                self.log(f"Cannot parse {entity} state '{state}' as float", level="WARNING")
                return None
        return total

    def _publish_sensors(self, forecast_kwh: float, min_soc: float) -> None:
        """Publish forecast total and dynamic min SOC to HA."""
        prefix = f"sensor.{self.cfg.sensor_prefix}"

        self.set_state(
            f"{prefix}_pv_forecast_total",
            state=round(forecast_kwh, 2),
            attributes={
                "friendly_name": "ZFI PV Forecast Total",
                "unit_of_measurement": "kWh",
                "icon": "mdi:solar-power",
            },
        )

        self.set_state(
            f"{prefix}_dynamic_min_soc",
            state=round(min_soc),
            attributes={
                "friendly_name": "ZFI Dynamic Min SOC",
                "unit_of_measurement": "%",
                "icon": "mdi:battery-arrow-down",
            },
        )
