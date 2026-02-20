# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

from pathlib import Path
from typing import Dict

import rclpy
from rclpy.node import Node

from .config import BridgeConfig, ConfigError, WriteAction, load_bridge_config
from .fanuc_adapter import FanucAdapter
from .mqtt_adapter import MqttAdapter
from .serializers import build_error_data, build_payload, topic_for


class FanucMqttBridgeNode(Node):
    def __init__(self):
        super().__init__("fanuc_mqtt_bridge")

        self.declare_parameter("config_file", "")
        config_file = self.get_parameter("config_file").get_parameter_value().string_value
        if not config_file:
            raise RuntimeError("required ROS parameter `config_file` is empty")

        self._cfg: BridgeConfig = load_bridge_config(config_file)
        self._base_topic = self._cfg.publish.base_topic or f"fanuc/{self._cfg.publish.robot_id}"

        self._mqtt = MqttAdapter(self._cfg.mqtt)
        self._fanuc = FanucAdapter(self._cfg.robot)

        self._sequence = 0
        self._cycle_index = 0
        self._robot_connected = False
        self._startup_actions_done = False

        self._timer = self.create_timer(1.0 / float(self._cfg.publish.rate_hz), self._tick)

    def _next_sequence(self) -> int:
        self._sequence += 1
        return self._sequence

    def _publish_category(self, category: str, data: Dict):
        topic = topic_for(self._base_topic, category)
        payload = build_payload(
            robot_id=self._cfg.publish.robot_id,
            sequence=self._next_sequence(),
            data=data,
            include_timestamp=self._cfg.publish.include_timestamp,
        )
        self._mqtt.publish(topic, payload)

    def _publish_error(self, error_type: str, message: str, operation: str, recoverable: bool):
        data = build_error_data(error_type, message, operation, recoverable)
        self._publish_category("errors", data)

    def _publish_connection(self, connected: bool):
        self._publish_category("connection", {"connected": connected})

    def _ensure_mqtt_connected(self) -> bool:
        if self._mqtt.connected:
            return True
        try:
            self._mqtt.connect_with_retry(max_attempts=1)
            return True
        except Exception as e:
            self.get_logger().error(f"MQTT connect failed: {e}")
            return False

    def _ensure_robot_connected(self) -> bool:
        if self._robot_connected:
            return True
        try:
            self._fanuc.connect_with_retry(max_attempts=1)
            self._robot_connected = True
            self._startup_actions_done = False
            self.get_logger().info("Connected to FANUC RMI")
            return True
        except Exception as e:
            self._robot_connected = False
            self.get_logger().error(f"FANUC connect failed: {e}")
            return False

    def _safe_read_publish(self, category: str, operation: str, fn):
        try:
            data = fn()
            self._publish_category(category, data)
        except Exception as e:
            self._publish_error("read_error", str(e), operation, True)

    def _run_actions(self, actions: list[WriteAction]):
        for action in actions:
            interval = int(action.params.get("interval_cycles", 1))
            if self._cycle_index % interval != 0:
                continue
            try:
                result = self._fanuc.execute_action(action)
                self._publish_category(
                    "errors",
                    {
                        "event": "write_action_executed",
                        "action": action.action,
                        "result": result,
                    },
                )
            except Exception as e:
                self._publish_error("write_error", str(e), action.action, True)

    def _tick(self):
        self._cycle_index += 1

        if not self._ensure_mqtt_connected():
            return

        if not self._ensure_robot_connected():
            self._publish_connection(False)
            return

        self._publish_connection(True)

        if self._cfg.writes.enabled and not self._startup_actions_done:
            self._run_actions(self._cfg.writes.startup_actions)
            self._startup_actions_done = True

        t = self._cfg.telemetry
        if t.joints:
            self._safe_read_publish("joints", "read_joints", self._fanuc.read_joints)
        if t.status:
            self._safe_read_publish("status", "read_status", self._fanuc.read_status)
        if t.status_ext:
            self._safe_read_publish("status_ext", "read_status_ext", self._fanuc.read_status_ext)
        if t.digital_inputs or t.digital_outputs or t.flags:
            self._safe_read_publish("io_digital", "read_digital", lambda: self._fanuc.read_digital(t))
        if t.analog_inputs or t.analog_outputs:
            self._safe_read_publish("io_analog", "read_analog", lambda: self._fanuc.read_analog(t))
        if t.group_inputs or t.group_outputs:
            self._safe_read_publish("io_group", "read_group", lambda: self._fanuc.read_group(t))
        if t.num_registers:
            self._safe_read_publish(
                "registers_num", "read_num_registers", lambda: self._fanuc.read_num_registers(t)
            )
        if t.pos_registers:
            self._safe_read_publish(
                "registers_pos", "read_pos_registers", lambda: self._fanuc.read_pos_registers(t)
            )
        if t.variables:
            self._safe_read_publish("variables", "read_variables", lambda: self._fanuc.read_variables(t))

        if self._cfg.writes.enabled:
            self._run_actions(self._cfg.writes.cyclic_actions)

    def shutdown(self):
        try:
            self._fanuc.disconnect()
        except Exception:
            pass
        try:
            self._mqtt.disconnect()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FanucMqttBridgeNode()
        rclpy.spin(node)
    except ConfigError as e:
        raise SystemExit(f"Invalid bridge config: {e}") from e
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
