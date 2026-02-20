# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import time
from typing import Optional

import paho.mqtt.client as mqtt

from .config import MqttConfig


class MqttAdapter:
    def __init__(self, config: MqttConfig):
        self._config = config
        self._client = mqtt.Client(client_id=config.client_id or "", clean_session=True)
        if config.username or config.password:
            self._client.username_pw_set(config.username, config.password)
        self._client.on_connect = self._on_connect
        self._client.on_disconnect = self._on_disconnect
        self._connected = False

    @property
    def connected(self) -> bool:
        return self._connected

    def _on_connect(self, client, userdata, flags, rc):
        self._connected = rc == 0

    def _on_disconnect(self, client, userdata, rc):
        self._connected = False

    def connect_with_retry(self, max_attempts: int = 0) -> None:
        attempt = 0
        backoff = 1.0
        while True:
            attempt += 1
            try:
                self._client.connect(self._config.host, self._config.port, self._config.keepalive_sec)
                self._client.loop_start()
                deadline = time.time() + 5.0
                while time.time() < deadline:
                    if self._connected:
                        return
                    time.sleep(0.05)
                raise RuntimeError("MQTT connect timeout")
            except Exception:
                if max_attempts > 0 and attempt >= max_attempts:
                    raise
                time.sleep(backoff)
                backoff = min(backoff * 2.0, 10.0)

    def publish(self, topic: str, payload: str) -> None:
        if not self._connected:
            raise RuntimeError("MQTT client not connected")
        info = self._client.publish(topic, payload=payload, qos=self._config.qos, retain=self._config.retain)
        if info.rc != mqtt.MQTT_ERR_SUCCESS:
            raise RuntimeError(f"MQTT publish failed rc={info.rc}")

    def disconnect(self) -> None:
        try:
            self._client.loop_stop()
            self._client.disconnect()
        finally:
            self._connected = False
