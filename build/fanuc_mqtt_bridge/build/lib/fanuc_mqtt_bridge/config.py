# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List

import yaml


class ConfigError(ValueError):
    pass


@dataclass(frozen=True)
class RobotConfig:
    ip: str
    rmi_port: int = 16001
    timeout_sec: float = 1.0
    backend: str = "auto"


@dataclass(frozen=True)
class MqttConfig:
    host: str
    port: int = 1883
    username: str = ""
    password: str = ""
    client_id: str = ""
    keepalive_sec: int = 60
    qos: int = 1
    retain: bool = False


@dataclass(frozen=True)
class PublishConfig:
    robot_id: str
    base_topic: str = ""
    rate_hz: int = 10
    include_timestamp: bool = True


@dataclass(frozen=True)
class TelemetryConfig:
    joints: bool = True
    status: bool = True
    status_ext: bool = True
    digital_inputs: List[int] = field(default_factory=list)
    digital_outputs: List[int] = field(default_factory=list)
    flags: List[int] = field(default_factory=list)
    analog_inputs: List[int] = field(default_factory=list)
    analog_outputs: List[int] = field(default_factory=list)
    group_inputs: List[int] = field(default_factory=list)
    group_outputs: List[int] = field(default_factory=list)
    num_registers: List[int] = field(default_factory=list)
    pos_registers: List[int] = field(default_factory=list)
    variables: List[str] = field(default_factory=list)


@dataclass(frozen=True)
class WriteAction:
    action: str
    params: Dict[str, Any]


@dataclass(frozen=True)
class WritesConfig:
    enabled: bool = False
    startup_actions: List[WriteAction] = field(default_factory=list)
    cyclic_actions: List[WriteAction] = field(default_factory=list)


@dataclass(frozen=True)
class BridgeConfig:
    robot: RobotConfig
    mqtt: MqttConfig
    publish: PublishConfig
    telemetry: TelemetryConfig
    writes: WritesConfig


_ALLOWED_ACTIONS = {
    "set_gen_override": {"value"},
    "set_num_register": {"index", "value"},
    "set_io_port": {"io_type", "index", "value"},
    "set_payload_schedule": {"payload_schedule_id"},
    "set_payload_value": {
        "payload_schedule_id",
        "mass",
        "cg_x",
        "cg_y",
        "cg_z",
        "use_in",
        "in_x",
        "in_y",
        "in_z",
    },
    "set_payload_comp": {
        "payload_schedule_id",
        "mass",
        "cg_x",
        "cg_y",
        "cg_z",
        "in_x",
        "in_y",
        "in_z",
    },
}


def _ensure_type(name: str, value: Any, expected: type) -> Any:
    if not isinstance(value, expected):
        raise ConfigError(f"`{name}` must be {expected.__name__}")
    return value


def _get_mapping(root: Dict[str, Any], key: str, required: bool = True) -> Dict[str, Any]:
    value = root.get(key)
    if value is None:
        if required:
            raise ConfigError(f"missing required config section `{key}`")
        return {}
    if not isinstance(value, dict):
        raise ConfigError(f"section `{key}` must be a mapping")
    return value


def _get_list_of_ints(mapping: Dict[str, Any], key: str) -> List[int]:
    raw = mapping.get(key, [])
    if not isinstance(raw, list):
        raise ConfigError(f"`telemetry.{key}` must be a list")
    out: List[int] = []
    for item in raw:
        if not isinstance(item, int):
            raise ConfigError(f"`telemetry.{key}` entries must be int")
        out.append(item)
    return out


def _get_list_of_strings(mapping: Dict[str, Any], key: str) -> List[str]:
    raw = mapping.get(key, [])
    if not isinstance(raw, list):
        raise ConfigError(f"`telemetry.{key}` must be a list")
    out: List[str] = []
    for item in raw:
        if not isinstance(item, str):
            raise ConfigError(f"`telemetry.{key}` entries must be string")
        out.append(item)
    return out


def _parse_actions(raw_actions: Any, field_name: str) -> List[WriteAction]:
    if raw_actions is None:
        return []
    if not isinstance(raw_actions, list):
        raise ConfigError(f"`writes.{field_name}` must be a list")

    actions: List[WriteAction] = []
    for idx, action_obj in enumerate(raw_actions):
        if not isinstance(action_obj, dict):
            raise ConfigError(f"`writes.{field_name}[{idx}]` must be a mapping")
        action_name = action_obj.get("action")
        if action_name not in _ALLOWED_ACTIONS:
            raise ConfigError(f"unknown write action `{action_name}` in `writes.{field_name}[{idx}]`")

        params = {k: v for k, v in action_obj.items() if k != "action"}
        required = _ALLOWED_ACTIONS[action_name]
        missing = sorted(required - set(params.keys()))
        if missing:
            raise ConfigError(
                f"`writes.{field_name}[{idx}]` action `{action_name}` missing keys: {', '.join(missing)}"
            )

        if "interval_cycles" in params:
            interval = params["interval_cycles"]
            if not isinstance(interval, int) or interval <= 0:
                raise ConfigError(
                    f"`writes.{field_name}[{idx}].interval_cycles` must be positive int"
                )

        actions.append(WriteAction(action=action_name, params=params))
    return actions


def load_bridge_config(path: str) -> BridgeConfig:
    cfg_path = Path(path)
    if not cfg_path.exists():
        raise ConfigError(f"config file does not exist: {path}")

    with cfg_path.open("r", encoding="utf-8") as f:
        try:
            doc = yaml.safe_load(f)
        except yaml.YAMLError as e:
            raise ConfigError(f"failed to parse yaml: {e}") from e

    if not isinstance(doc, dict):
        raise ConfigError("top-level YAML must be a mapping")

    robot_raw = _get_mapping(doc, "robot")
    mqtt_raw = _get_mapping(doc, "mqtt")
    publish_raw = _get_mapping(doc, "publish")
    telemetry_raw = _get_mapping(doc, "telemetry", required=False)
    writes_raw = _get_mapping(doc, "writes", required=False)

    robot = RobotConfig(
        ip=_ensure_type("robot.ip", robot_raw.get("ip"), str),
        rmi_port=int(robot_raw.get("rmi_port", 16001)),
        timeout_sec=float(robot_raw.get("timeout_sec", 1.0)),
        backend=_ensure_type("robot.backend", robot_raw.get("backend", "auto"), str),
    )

    if robot.backend not in {"auto", "fanuc_rmi", "sim"}:
        raise ConfigError("`robot.backend` must be one of: auto, fanuc_rmi, sim")

    mqtt = MqttConfig(
        host=_ensure_type("mqtt.host", mqtt_raw.get("host"), str),
        port=int(mqtt_raw.get("port", 1883)),
        username=_ensure_type("mqtt.username", mqtt_raw.get("username", ""), str),
        password=_ensure_type("mqtt.password", mqtt_raw.get("password", ""), str),
        client_id=_ensure_type("mqtt.client_id", mqtt_raw.get("client_id", ""), str),
        keepalive_sec=int(mqtt_raw.get("keepalive_sec", 60)),
        qos=int(mqtt_raw.get("qos", 1)),
        retain=bool(mqtt_raw.get("retain", False)),
    )

    publish = PublishConfig(
        robot_id=_ensure_type("publish.robot_id", publish_raw.get("robot_id"), str),
        base_topic=_ensure_type("publish.base_topic", publish_raw.get("base_topic", ""), str),
        rate_hz=int(publish_raw.get("rate_hz", 10)),
        include_timestamp=bool(publish_raw.get("include_timestamp", True)),
    )

    if publish.rate_hz <= 0:
        raise ConfigError("`publish.rate_hz` must be > 0")

    telemetry = TelemetryConfig(
        joints=bool(telemetry_raw.get("joints", True)),
        status=bool(telemetry_raw.get("status", True)),
        status_ext=bool(telemetry_raw.get("status_ext", True)),
        digital_inputs=_get_list_of_ints(telemetry_raw, "digital_inputs"),
        digital_outputs=_get_list_of_ints(telemetry_raw, "digital_outputs"),
        flags=_get_list_of_ints(telemetry_raw, "flags"),
        analog_inputs=_get_list_of_ints(telemetry_raw, "analog_inputs"),
        analog_outputs=_get_list_of_ints(telemetry_raw, "analog_outputs"),
        group_inputs=_get_list_of_ints(telemetry_raw, "group_inputs"),
        group_outputs=_get_list_of_ints(telemetry_raw, "group_outputs"),
        num_registers=_get_list_of_ints(telemetry_raw, "num_registers"),
        pos_registers=_get_list_of_ints(telemetry_raw, "pos_registers"),
        variables=_get_list_of_strings(telemetry_raw, "variables"),
    )

    writes = WritesConfig(
        enabled=bool(writes_raw.get("enabled", False)),
        startup_actions=_parse_actions(writes_raw.get("startup_actions", []), "startup_actions"),
        cyclic_actions=_parse_actions(writes_raw.get("cyclic_actions", []), "cyclic_actions"),
    )

    return BridgeConfig(robot=robot, mqtt=mqtt, publish=publish, telemetry=telemetry, writes=writes)
