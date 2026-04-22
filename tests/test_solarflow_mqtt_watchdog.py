"""Tests for solarflow_mqtt_watchdog – heartbeat monitoring logic.

Tests exercise the heartbeat staleness check without HA/AppDaemon mocking.
"""

from __future__ import annotations

from datetime import datetime, timedelta, timezone

import pytest

from src.solarflow_mqtt_watchdog import SolarFlowMqttWatchdog


# ═══════════════════════════════════════════════════════════
#  _is_entity_stale
# ═══════════════════════════════════════════════════════════


class TestIsEntityStale:
    """Tests for the _is_entity_stale helper."""

    @staticmethod
    def _make_watchdog(stale_s: int = 60) -> object:
        """Create a minimal watchdog-like object with _is_entity_stale bound."""

        class FakeWatchdog:
            """Minimal stand-in for SolarFlowMqttWatchdog."""

            def __init__(self, stale_s: int) -> None:
                self._heartbeat_stale_s = stale_s
                self._attrs: dict[str, dict[str, str | None]] = {}

            def get_state(
                self, entity_id: str, attribute: str | None = None,
            ) -> str | None:
                if attribute is not None:
                    return self._attrs.get(entity_id, {}).get(attribute)
                return self._attrs.get(entity_id, {}).get("state")

        wd = FakeWatchdog(stale_s)
        wd._is_entity_stale = SolarFlowMqttWatchdog._is_entity_stale.__get__(wd)
        return wd

    def test_unavailable_entity_is_stale(self):
        """Entity with state 'unavailable' is considered stale."""
        wd = self._make_watchdog()
        wd._attrs["sensor.test"] = {"state": "unavailable"}
        assert wd._is_entity_stale("sensor.test") is True

    def test_unknown_entity_is_stale(self):
        """Entity with state 'unknown' is considered stale."""
        wd = self._make_watchdog()
        wd._attrs["sensor.test"] = {"state": "unknown"}
        assert wd._is_entity_stale("sensor.test") is True

    def test_missing_entity_is_stale(self):
        """Entity not found at all is considered stale."""
        wd = self._make_watchdog()
        assert wd._is_entity_stale("sensor.nonexistent") is True

    def test_fresh_entity_not_stale(self):
        """Entity updated 10s ago is not stale (threshold=60s)."""
        wd = self._make_watchdog(60)
        ts = (datetime.now(timezone.utc) - timedelta(seconds=10)).isoformat()
        wd._attrs["sensor.test"] = {"state": "200", "last_updated": ts}
        assert wd._is_entity_stale("sensor.test") is False

    def test_old_entity_is_stale(self):
        """Entity updated 120s ago is stale (threshold=60s)."""
        wd = self._make_watchdog(60)
        ts = (datetime.now(timezone.utc) - timedelta(seconds=120)).isoformat()
        wd._attrs["sensor.test"] = {"state": "200", "last_updated": ts}
        assert wd._is_entity_stale("sensor.test") is True

    def test_missing_last_updated_is_stale(self):
        """Entity with no last_updated attribute is stale."""
        wd = self._make_watchdog()
        wd._attrs["sensor.test"] = {"state": "200"}
        assert wd._is_entity_stale("sensor.test") is True

    def test_invalid_timestamp_is_stale(self):
        """Unparseable timestamp is treated as stale."""
        wd = self._make_watchdog()
        wd._attrs["sensor.test"] = {"state": "200", "last_updated": "garbage"}
        assert wd._is_entity_stale("sensor.test") is True


# ═══════════════════════════════════════════════════════════
#  _check_heartbeats
# ═══════════════════════════════════════════════════════════


class TestCheckHeartbeats:
    """Tests for the periodic heartbeat check with notifications."""

    @staticmethod
    def _make_watchdog(
        entities: list[str],
        stale_s: int = 60,
        dry_run: bool = False,
    ) -> object:
        """Create a minimal watchdog with _check_heartbeats bound."""

        class FakeWatchdog:
            def __init__(self) -> None:
                self._heartbeat_entities = entities
                self._heartbeat_stale_s = stale_s
                self._dry_run = dry_run
                self._stale_notified: set[str] = set()
                self._attrs: dict[str, dict[str, str | None]] = {}
                self.service_calls: list[tuple[str, dict]] = []
                self.log_messages: list[str] = []

            def get_state(
                self, entity_id: str, attribute: str | None = None,
            ) -> str | None:
                if attribute is not None:
                    return self._attrs.get(entity_id, {}).get(attribute)
                return self._attrs.get(entity_id, {}).get("state")

            def call_service(self, service: str, **kwargs: object) -> None:
                self.service_calls.append((service, kwargs))

            def log(self, msg: str, **_kwargs: object) -> None:
                self.log_messages.append(msg)

        wd = FakeWatchdog()
        wd._is_entity_stale = SolarFlowMqttWatchdog._is_entity_stale.__get__(wd)
        wd._check_heartbeats = SolarFlowMqttWatchdog._check_heartbeats.__get__(wd)
        return wd

    def test_fresh_entities_no_notification(self):
        """No notification when all entities are fresh."""
        wd = self._make_watchdog(["sensor.a", "sensor.b"])
        ts = (datetime.now(timezone.utc) - timedelta(seconds=10)).isoformat()
        wd._attrs["sensor.a"] = {"state": "200", "last_updated": ts}
        wd._attrs["sensor.b"] = {"state": "100", "last_updated": ts}
        wd._check_heartbeats({})
        assert wd.service_calls == []
        assert wd._stale_notified == set()

    def test_stale_entity_creates_notification(self):
        """Stale entity triggers persistent_notification/create."""
        wd = self._make_watchdog(["sensor.a"])
        ts = (datetime.now(timezone.utc) - timedelta(seconds=120)).isoformat()
        wd._attrs["sensor.a"] = {"state": "200", "last_updated": ts}
        wd._check_heartbeats({})
        assert len(wd.service_calls) == 1
        assert wd.service_calls[0][0] == "persistent_notification/create"
        assert "sensor.a" in wd._stale_notified

    def test_stale_notification_only_once(self):
        """Repeated checks of same stale entity don't duplicate notifications."""
        wd = self._make_watchdog(["sensor.a"])
        ts = (datetime.now(timezone.utc) - timedelta(seconds=120)).isoformat()
        wd._attrs["sensor.a"] = {"state": "200", "last_updated": ts}
        wd._check_heartbeats({})
        wd._check_heartbeats({})
        assert len(wd.service_calls) == 1

    def test_recovery_dismisses_notification(self):
        """Entity recovery dismisses the persistent notification."""
        wd = self._make_watchdog(["sensor.a"])
        # First: stale
        old_ts = (datetime.now(timezone.utc) - timedelta(seconds=120)).isoformat()
        wd._attrs["sensor.a"] = {"state": "200", "last_updated": old_ts}
        wd._check_heartbeats({})
        assert "sensor.a" in wd._stale_notified
        # Then: recover
        fresh_ts = (datetime.now(timezone.utc) - timedelta(seconds=5)).isoformat()
        wd._attrs["sensor.a"]["last_updated"] = fresh_ts
        wd._check_heartbeats({})
        assert "sensor.a" not in wd._stale_notified
        assert wd.service_calls[-1][0] == "persistent_notification/dismiss"

    def test_dry_run_no_service_calls(self):
        """In dry_run mode, stale entities are logged but no service calls made."""
        wd = self._make_watchdog(["sensor.a"], dry_run=True)
        ts = (datetime.now(timezone.utc) - timedelta(seconds=120)).isoformat()
        wd._attrs["sensor.a"] = {"state": "200", "last_updated": ts}
        wd._check_heartbeats({})
        assert wd.service_calls == []
        assert "sensor.a" in wd._stale_notified
        assert any("stale" in m.lower() for m in wd.log_messages)

    def test_notification_id_format(self):
        """Notification IDs are stable and entity-specific."""
        wd = self._make_watchdog(["sensor.zfi_desired_power"])
        ts = (datetime.now(timezone.utc) - timedelta(seconds=120)).isoformat()
        wd._attrs["sensor.zfi_desired_power"] = {"state": "200", "last_updated": ts}
        wd._check_heartbeats({})
        notif_id = wd.service_calls[0][1]["notification_id"]
        assert notif_id == "zfi_heartbeat_sensor_zfi_desired_power"
