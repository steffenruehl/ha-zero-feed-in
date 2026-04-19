"""
SolarFlow MQTT Watchdog – Triggers a reconnect via the local HTTP API.

The SolarFlow firmware retries MQTT reconnection ~30 times (1 s apart) after
losing the broker.  A typical Home Assistant restart takes 60-90 s, which
exceeds the retry window.  This app calls the device's local HTTP API to push
the MQTT configuration again, which causes an immediate reconnect attempt.

Two triggers:
  1. HA startup   – waits ``startup_delay_s`` for Mosquitto to come up, then
                    sends the config.
  2. Entity stale – when ``watch_entity`` stays ``unavailable`` for longer than
                    ``unavailable_duration_s``, the config is resent.

Config (apps.yaml):

  solarflow_mqtt_watchdog:
    module: solarflow_mqtt_watchdog
    class: SolarFlowMqttWatchdog
    solarflow_ip:    "192.168.178.145"   # device LAN IP
    serial_number:   "DEINE_SERIENNUMMER"
    mqtt_broker_ip:  "192.168.178.X"     # Mosquitto LAN IP
    mqtt_port:       1883                # optional, default 1883
    mqtt_username:   "DEIN_USER"
    mqtt_password:   "DEIN_PASSWORT"
    watch_entity:    sensor.hec4nencn492140_electriclevel
    unavailable_duration_s: 120          # optional, default 120 s
    startup_delay_s: 30                  # optional, default 30 s
    dry_run: false                       # optional, default false
"""

from __future__ import annotations

import json

try:
    import requests as _requests
except ImportError:  # pragma: no cover
    _requests = None  # type: ignore[assignment]

try:
    import appdaemon.plugins.hass.hassapi as hass
    _HASS_BASE = hass.Hass
except ImportError:
    _HASS_BASE = object  # type: ignore[assignment,misc]


class SolarFlowMqttWatchdog(_HASS_BASE):
    """Watches the SolarFlow MQTT connection and triggers reconnects via HTTP."""

    def initialize(self) -> None:
        self._solarflow_ip: str = self.args["solarflow_ip"]
        self._serial: str = self.args["serial_number"]
        self._broker_ip: str = self.args["mqtt_broker_ip"]
        self._broker_port: int = int(self.args.get("mqtt_port", 1883))
        self._mqtt_user: str = self.args["mqtt_username"]
        self._mqtt_pass: str = self.args["mqtt_password"]
        self._watch_entity: str = self.args["watch_entity"]
        self._unavailable_s: int = int(self.args.get("unavailable_duration_s", 120))
        self._startup_delay_s: int = int(self.args.get("startup_delay_s", 30))
        self._dry_run: bool = bool(self.args.get("dry_run", False))

        # Trigger 1: HA startup
        self.listen_event(self._on_ha_start, "homeassistant_started")

        # Trigger 2: entity unavailable for too long
        self.listen_state(
            self._on_entity_stale,
            self._watch_entity,
            new="unavailable",
            duration=self._unavailable_s,
        )

        self.log(
            f"Started | watch={self._watch_entity} "
            f"stale_after={self._unavailable_s}s "
            f"startup_delay={self._startup_delay_s}s "
            f"target=http://{self._solarflow_ip}/rpc "
            f"dry_run={self._dry_run}"
        )

    # ── Event handlers ────────────────────────────────────────────────

    def _on_ha_start(self, event_name: str, data: dict, kwargs: dict) -> None:
        self.log(f"HA started — will reconnect SolarFlow in {self._startup_delay_s}s")
        self.run_in(self._trigger_reconnect, self._startup_delay_s)

    def _on_entity_stale(
        self, entity: str, attribute: str, old: str, new: str, kwargs: dict
    ) -> None:
        self.log(
            f"{entity} has been unavailable for {self._unavailable_s}s — triggering reconnect",
            level="WARNING",
        )
        self._trigger_reconnect({})

    # ── Reconnect ─────────────────────────────────────────────────────

    def _trigger_reconnect(self, kwargs: dict) -> None:
        """POST the MQTT config to the SolarFlow HTTP API."""
        url = f"http://{self._solarflow_ip}/rpc"
        payload = {
            "sn": self._serial,
            "method": "HA.Mqtt.SetConfig",
            "params": {
                "config": {
                    "enable": True,
                    "server": self._broker_ip,
                    "port": self._broker_port,
                    "protocol": "mqtt",
                    "username": self._mqtt_user,
                    "password": self._mqtt_pass,
                }
            },
        }

        if self._dry_run:
            self.log(f"DRY RUN — would POST to {url}: {json.dumps(payload)}")
            return

        if _requests is None:
            self.log(
                "requests library not available — cannot trigger reconnect",
                level="ERROR",
            )
            return

        try:
            resp = _requests.post(url, json=payload, timeout=10)
            self.log(
                f"Reconnect sent → HTTP {resp.status_code} "
                f"body={resp.text[:200]!r}"
            )
        except Exception as exc:  # noqa: BLE001
            self.log(f"Reconnect failed: {exc}", level="ERROR")

