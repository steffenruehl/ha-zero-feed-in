"""relay_switch_counter.py – Counts relay state machine transitions.

Watches ``sensor.zfi_relay_sm_state`` (published by the driver) and
increments a counter each time the state changes (e.g. charge → idle,
idle → discharge).  The count is persisted to a JSON file so it
survives AppDaemon restarts.
"""

import json
import os
from datetime import datetime, timezone

import appdaemon.plugins.hass.hassapi as hass

UNAVAILABLE_STATES = {None, "unknown", "unavailable", ""}
"""HA entity states that should be ignored."""


class RelaySwitchCounter(hass.Hass):
    """Count relay state machine transitions."""

    def initialize(self) -> None:
        """Set up the counter and start listening for SM state changes."""
        self._relay_sm_entity: str = self.args.get(
            "relay_sm_state_sensor", "sensor.zfi_relay_sm_state"
        )
        _run_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "run"
        )
        os.makedirs(_run_dir, exist_ok=True)
        self._counter_file: str = self.args.get(
            "counter_file",
            os.path.join(_run_dir, "relay_switch_count.json"),
        )
        self._sensor_prefix: str = self.args.get("sensor_prefix", "sensor.zfi")

        self._count, self._last_switch_ts = self._load()
        init_state = self.get_state(self._relay_sm_entity)
        self._last_state: str | None = (
            init_state if init_state not in UNAVAILABLE_STATES else None
        )

        self.listen_state(self._on_relay_sm_change, self._relay_sm_entity)
        self._publish()

        self.log(
            f"RelaySwitchCounter ready | count={self._count} "
            f"entity={self._relay_sm_entity} "
            f"current={self._last_state} "
            f"file={self._counter_file}"
        )

    # ── state listener ───────────────────────────────────────────────────────

    def _on_relay_sm_change(
        self, entity: str, attribute: str, old: str, new: str, kwargs: dict
    ) -> None:
        """Increment counter on every relay SM state transition."""
        if new in UNAVAILABLE_STATES:
            return
        if new == self._last_state:
            return

        self._last_state = new
        self._count += 1
        self._last_switch_ts = datetime.now(timezone.utc).isoformat()
        self._save()
        self._publish()
        self.log(f"Relay switch #{self._count} ({old} → {new})")

    # ── HA publishing ────────────────────────────────────────────────────────

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
