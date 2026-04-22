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

Optional heartbeat monitoring with safe-state enforcement:
  When ``heartbeat_entities`` is configured, the watchdog periodically checks
  the ``last_updated`` timestamp of each entity.  If any entity is older than
  ``heartbeat_stale_s`` seconds:
    1. A HA persistent notification is created.
    2. If ``output_limit_entity`` and ``input_limit_entity`` are configured,
       both are set to 0 W (safe state) — stopping the device.
  When all entities recover, the notification is dismissed and normal
  operation resumes (the driver will re-send its desired limits).

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
    # Heartbeat monitoring (optional)
    heartbeat_entities:                  # empty = heartbeat monitoring disabled
      - sensor.zfi_desired_power         # controller heartbeat
      - sensor.zfi_device_output         # driver heartbeat
    heartbeat_stale_s: 60                # max age before notification (s)
    heartbeat_check_s: 30                # check interval (s)
    # Safe-state: device entities to zero when any heartbeat is stale
    output_limit_entity: number.YOUR_SOLARFLOW_OUTPUTLIMIT
    input_limit_entity: number.YOUR_SOLARFLOW_INPUTLIMIT
"""

from __future__ import annotations

import json
from datetime import datetime, timezone

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

        # Heartbeat monitoring
        self._heartbeat_entities: list[str] = list(
            self.args.get("heartbeat_entities", [])
        )
        self._heartbeat_stale_s: int = int(
            self.args.get("heartbeat_stale_s", 60)
        )
        self._heartbeat_check_s: int = int(
            self.args.get("heartbeat_check_s", 30)
        )
        self._stale_notified: set[str] = set()

        # Safe-state device entities (optional)
        self._output_limit_entity: str = self.args.get(
            "output_limit_entity", ""
        )
        self._input_limit_entity: str = self.args.get(
            "input_limit_entity", ""
        )
        self._safe_state_active: bool = False

        # Trigger 1: HA startup
        self.listen_event(self._on_ha_start, "homeassistant_started")

        # Trigger 2: entity unavailable for too long
        self.listen_state(
            self._on_entity_stale,
            self._watch_entity,
            new="unavailable",
            duration=self._unavailable_s,
        )

        # Trigger 3: entity already unavailable at AppDaemon start
        current = self.get_state(self._watch_entity)
        if current in (None, "unavailable", "unknown"):
            self.log(
                f"{self._watch_entity} is already {current!r} at startup — "
                f"scheduling reconnect in {self._startup_delay_s}s"
            )
            self.run_in(self._trigger_reconnect, self._startup_delay_s)

        # Heartbeat check timer
        if self._heartbeat_entities:
            self.run_every(
                self._check_heartbeats, "now+60", self._heartbeat_check_s,
            )

        safe_state_cfg = (
            f"out={self._output_limit_entity} in={self._input_limit_entity}"
            if self._output_limit_entity
            else "disabled"
        )
        self.log(
            f"Started | watch={self._watch_entity} "
            f"stale_after={self._unavailable_s}s "
            f"startup_delay={self._startup_delay_s}s "
            f"target=http://{self._solarflow_ip}/rpc "
            f"heartbeat_entities={len(self._heartbeat_entities)} "
            f"heartbeat_stale_s={self._heartbeat_stale_s} "
            f"safe_state={safe_state_cfg} "
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

    # ── Heartbeat monitoring ──────────────────────────────────────────

    def _check_heartbeats(self, _kwargs: dict) -> None:
        """Periodically check ``last_updated`` of heartbeat entities.

        Creates a HA persistent notification when an entity is stale
        and dismisses it when the entity recovers.  When any entity is
        stale and device entities are configured, sends safe state (0 W)
        to both outputLimit and inputLimit.
        """
        any_stale = False
        for entity in self._heartbeat_entities:
            stale = self._is_entity_stale(entity)
            if stale:
                any_stale = True
            notif_id = f"zfi_heartbeat_{entity.replace('.', '_')}"

            if stale and entity not in self._stale_notified:
                self._stale_notified.add(entity)
                label = entity.split(".")[-1].replace("_", " ").title()
                self.log(
                    f"Heartbeat stale: {entity} (>{self._heartbeat_stale_s}s)",
                    level="WARNING",
                )
                if not self._dry_run:
                    self.call_service(
                        "persistent_notification/create",
                        title="ZFI Heartbeat Lost",
                        message=(
                            f"**{label}** (`{entity}`) has not updated for "
                            f"more than {self._heartbeat_stale_s}s."
                        ),
                        notification_id=notif_id,
                    )
            elif not stale and entity in self._stale_notified:
                self._stale_notified.discard(entity)
                self.log(f"Heartbeat recovered: {entity}")
                if not self._dry_run:
                    self.call_service(
                        "persistent_notification/dismiss",
                        notification_id=notif_id,
                    )

        # Safe-state enforcement: send 0 W when any entity is stale
        if any_stale and not self._safe_state_active:
            self._safe_state_active = True
            self._send_safe_state()
        elif not any_stale and self._safe_state_active:
            self._safe_state_active = False
            self.log("All heartbeats recovered — safe state released")

    def _is_entity_stale(self, entity: str) -> bool:
        """Return True if *entity*'s ``last_updated`` exceeds the threshold."""
        state = self.get_state(entity)
        if state in (None, "unavailable", "unknown"):
            return True
        last_updated_raw = self.get_state(entity, attribute="last_updated")
        if not last_updated_raw:
            return True
        try:
            last_updated = datetime.fromisoformat(str(last_updated_raw))
            age_s = (datetime.now(timezone.utc) - last_updated).total_seconds()
            return age_s > self._heartbeat_stale_s
        except (ValueError, TypeError):
            return True

    def _send_safe_state(self) -> None:
        """Send 0 W to both outputLimit and inputLimit (device safe state).

        Called when any heartbeat entity goes stale.  Requires
        ``output_limit_entity`` and ``input_limit_entity`` to be
        configured.  Skipped in dry_run mode.
        """
        if not self._output_limit_entity or not self._input_limit_entity:
            return
        if self._dry_run:
            self.log(
                "DRY RUN — would send safe state "
                f"(0W to {self._output_limit_entity}, {self._input_limit_entity})"
            )
            return
        self.log(
            "Sending safe state (0W) — heartbeat stale",
            level="WARNING",
        )
        try:
            self.call_service(
                "number/set_value",
                entity_id=self._output_limit_entity,
                value=0,
            )
            self.call_service(
                "number/set_value",
                entity_id=self._input_limit_entity,
                value=0,
            )
        except Exception as exc:  # noqa: BLE001
            self.log(f"Safe state send failed: {exc}", level="ERROR")

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

