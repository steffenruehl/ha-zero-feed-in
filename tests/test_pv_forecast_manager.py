"""Tests for PvForecastManager — config parsing and min-SOC logic."""

from __future__ import annotations

from datetime import datetime

import pytest

from src.pv_forecast_manager import MinSocRule, PvForecastConfig, compute_min_soc


# ── PvForecastConfig.from_args ───────────────────────────────────────────────


class TestPvForecastConfigFromArgs:
    """Test configuration parsing from AppDaemon args dict."""

    def test_minimal_config(self) -> None:
        """Only forecast_entities is required; everything else has defaults."""
        cfg = PvForecastConfig.from_args(
            {"forecast_entities": ["sensor.pv_tomorrow"]}
        )
        assert cfg.forecast_entities == ["sensor.pv_tomorrow"]
        assert cfg.forecast_threshold_kwh == 1.5
        assert cfg.min_soc_high_forecast_pct == 10.0
        assert len(cfg.min_soc_rules) == 2
        assert cfg.evaluation_times == ["06:00", "15:00", "20:00"]
        assert cfg.sensor_prefix == "zfi"
        assert cfg.debug is False

    def test_all_params(self) -> None:
        """All parameters can be overridden."""
        cfg = PvForecastConfig.from_args({
            "forecast_entities": ["sensor.a", "sensor.b"],
            "forecast_threshold_kwh": 2.0,
            "min_soc_high_forecast_pct": 15,
            "min_soc_rules": [
                {"before": "10:00", "soc": 60},
                {"before": "18:00", "soc": 40},
            ],
            "evaluation_times": ["07:00", "19:00"],
            "sensor_prefix": "test",
            "debug": True,
        })
        assert cfg.forecast_entities == ["sensor.a", "sensor.b"]
        assert cfg.forecast_threshold_kwh == 2.0
        assert cfg.min_soc_high_forecast_pct == 15.0
        assert len(cfg.min_soc_rules) == 2
        assert cfg.min_soc_rules[0] == MinSocRule(before="10:00", soc_pct=60.0)
        assert cfg.min_soc_rules[1] == MinSocRule(before="18:00", soc_pct=40.0)
        assert cfg.evaluation_times == ["07:00", "19:00"]
        assert cfg.sensor_prefix == "test"
        assert cfg.debug is True

    def test_empty_entities_raises(self) -> None:
        """Must provide at least one forecast entity."""
        with pytest.raises(ValueError, match="forecast_entities"):
            PvForecastConfig.from_args({"forecast_entities": []})

    def test_missing_entities_raises(self) -> None:
        """Missing forecast_entities key raises."""
        with pytest.raises(ValueError, match="forecast_entities"):
            PvForecastConfig.from_args({})

    def test_single_entity(self) -> None:
        """Works with a single forecast entity."""
        cfg = PvForecastConfig.from_args(
            {"forecast_entities": ["sensor.pv"]}
        )
        assert cfg.forecast_entities == ["sensor.pv"]

    def test_type_coercion(self) -> None:
        """Numeric strings from YAML are coerced to float/bool."""
        cfg = PvForecastConfig.from_args({
            "forecast_entities": ["sensor.pv"],
            "forecast_threshold_kwh": "2.5",
            "min_soc_high_forecast_pct": "20",
        })
        assert cfg.forecast_threshold_kwh == 2.5
        assert cfg.min_soc_high_forecast_pct == 20.0


# ── compute_min_soc ──────────────────────────────────────────────────────────


DEFAULT_RULES = [
    MinSocRule(before="12:00", soc_pct=50.0),
    MinSocRule(before="23:59", soc_pct=30.0),
]


class TestComputeMinSoc:
    """Test the pure min-SOC computation logic."""

    def _at(self, hour: int, minute: int = 0) -> datetime:
        """Helper: create a datetime at a specific hour:minute."""
        return datetime(2026, 4, 22, hour, minute)

    # ── high forecast ────────────────────────────────────────────────────

    def test_high_forecast_returns_high_soc(self) -> None:
        """Forecast >= threshold → high_forecast_soc regardless of time."""
        assert compute_min_soc(2.0, 1.5, 10.0, DEFAULT_RULES, self._at(6)) == 10.0
        assert compute_min_soc(2.0, 1.5, 10.0, DEFAULT_RULES, self._at(15)) == 10.0
        assert compute_min_soc(2.0, 1.5, 10.0, DEFAULT_RULES, self._at(22)) == 10.0

    def test_exactly_at_threshold(self) -> None:
        """Forecast exactly at threshold counts as high."""
        assert compute_min_soc(1.5, 1.5, 10.0, DEFAULT_RULES, self._at(8)) == 10.0

    # ── low forecast + time-of-day rules ─────────────────────────────────

    def test_low_forecast_morning(self) -> None:
        """Low forecast + morning → 50% (first rule: before 12:00)."""
        assert compute_min_soc(1.0, 1.5, 10.0, DEFAULT_RULES, self._at(6)) == 50.0
        assert compute_min_soc(1.0, 1.5, 10.0, DEFAULT_RULES, self._at(11, 59)) == 50.0

    def test_low_forecast_afternoon(self) -> None:
        """Low forecast + afternoon → 30% (second rule: before 23:59)."""
        assert compute_min_soc(1.0, 1.5, 10.0, DEFAULT_RULES, self._at(12)) == 30.0
        assert compute_min_soc(1.0, 1.5, 10.0, DEFAULT_RULES, self._at(15)) == 30.0
        assert compute_min_soc(1.0, 1.5, 10.0, DEFAULT_RULES, self._at(20)) == 30.0

    def test_low_forecast_evening(self) -> None:
        """Low forecast + late evening → 30% (still matches second rule)."""
        assert compute_min_soc(1.0, 1.5, 10.0, DEFAULT_RULES, self._at(23, 30)) == 30.0

    def test_zero_forecast(self) -> None:
        """Zero forecast → low-forecast rules apply."""
        assert compute_min_soc(0.0, 1.5, 10.0, DEFAULT_RULES, self._at(8)) == 50.0

    # ── edge cases ───────────────────────────────────────────────────────

    def test_empty_rules_returns_high_forecast_soc(self) -> None:
        """No rules configured → fall back to high_forecast_soc."""
        assert compute_min_soc(0.5, 1.5, 10.0, [], self._at(8)) == 10.0

    def test_single_rule(self) -> None:
        """Single rule covering all day."""
        rules = [MinSocRule(before="23:59", soc_pct=40.0)]
        assert compute_min_soc(0.5, 1.5, 10.0, rules, self._at(6)) == 40.0
        assert compute_min_soc(0.5, 1.5, 10.0, rules, self._at(22)) == 40.0

    def test_custom_threshold(self) -> None:
        """Custom threshold value is respected."""
        assert compute_min_soc(2.0, 3.0, 10.0, DEFAULT_RULES, self._at(8)) == 50.0
        assert compute_min_soc(3.0, 3.0, 10.0, DEFAULT_RULES, self._at(8)) == 10.0

    def test_three_rules(self) -> None:
        """Three time-of-day rules with different boundaries."""
        rules = [
            MinSocRule(before="08:00", soc_pct=60.0),
            MinSocRule(before="16:00", soc_pct=40.0),
            MinSocRule(before="23:59", soc_pct=20.0),
        ]
        assert compute_min_soc(0.5, 1.5, 10.0, rules, self._at(6)) == 60.0
        assert compute_min_soc(0.5, 1.5, 10.0, rules, self._at(10)) == 40.0
        assert compute_min_soc(0.5, 1.5, 10.0, rules, self._at(20)) == 20.0

    def test_negative_forecast_treated_as_low(self) -> None:
        """Negative forecast (sensor error) → low-forecast rules apply."""
        assert compute_min_soc(-1.0, 1.5, 10.0, DEFAULT_RULES, self._at(8)) == 50.0
