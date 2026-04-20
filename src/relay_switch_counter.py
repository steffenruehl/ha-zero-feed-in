"""relay_switch_counter.py – Counts relay activation events.

A relay switch is counted each time the desired-power sensor transitions
from below ``min_active_w`` to at or above it (in either direction).
The count is persisted to a JSON file so it survives AppDaemon restarts.
"""

import json
import os
from datetime import datetime, timezone

import appdaemon.plugins.hass.hassapi as hass


class RelaySwitchCounter(hass.Hass):
    def initialize(self) -> None:
        self._desired_entity: str = self.args.get(
            "desired_power_sensor", "sensor.zfi_desired_power"
        )
        self._min_active_w: float = float(self.args.get("min_active_w", 25))
        self._counter_file: str = self.args.get(
            "counter_file",
            "/addon_configs/a0d7b954_appdaemon/apps/relay_switch_count.json",
        )
        self._sensor_prefix: str = self.args.get("sensor_prefix", "sensor.zfi")

        self._count, self._last_switch_ts = self._load()
        self._last_active = self._is_active(self.get_state(self._desired_entity))

        self.listen_state(self._on_desired_power, self._desired_entity)
        self._publish()

        self.log(
            f"RelaySwitchCounter ready | count={self._count} "
            f"entity={self._desired_entity} "
            f"min_active_w={self._min_active_w} "
            f"file={self._counter_file}"
        )

    # ── state listener ───────────────────────────────────────────────────────

    def _on_desired_power(
        self, entity: str, attribute: str, old: str, new: str, kwargs: dict
    ) -> None:
        active = self._is_active(new)
        if active and not self._last_active:
            self._count += 1
            self._last_switch_ts = datetime.now(timezone.utc).isoformat()
            self._save()
            self._publish()
            self.log(f"Relay switch #{self._count} at {self._last_switch_ts}")
        self._last_active = active

    # ── helpers ──────────────────────────────────────────────────────────────

    def _is_active(self, state: str | None) -> bool:
        try:
            return abs(float(state)) >= self._min_active_w
        except (TypeError, ValueError):
            return False

    def _publish(self) -> None:
        self.set_state(
            f"{self._sensor_prefix}_relay_switches",
            state=self._count,
            attributes={
                "friendly_name": "ZFI Relay Switches",
                "icon": "mdi:counter",
                "last_switch": self._last_switch_ts,
            },
        )

    # ── persistence ──────────────────────────────────────────────────────────

    def _load(self) -> tuple[int, str]:
        try:
            with open(self._counter_file) as f:
                data = json.load(f)
            return int(data.get("count", 0)), data.get("last_switch", "")
        except (FileNotFoundError, json.JSONDecodeError, KeyError):
            return 0, ""

    def _save(self) -> None:
        tmp = self._counter_file + ".tmp"
        with open(tmp, "w") as f:
            json.dump({"count": self._count, "last_switch": self._last_switch_ts}, f)
        os.replace(tmp, self._counter_file)
