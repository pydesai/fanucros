#!/usr/bin/env python3

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any, Callable

import yaml


def parse_bool(name: str, raw: str) -> bool:
    value = raw.strip().lower()
    if value in {"1", "true", "t", "yes", "y", "on"}:
        return True
    if value in {"0", "false", "f", "no", "n", "off"}:
        return False
    raise ValueError(f"{name} must be a boolean-like value, got `{raw}`")


def parse_int(name: str, raw: str) -> int:
    try:
        return int(raw.strip())
    except ValueError as exc:
        raise ValueError(f"{name} must be an integer, got `{raw}`") from exc


def parse_float(name: str, raw: str) -> float:
    try:
        return float(raw.strip())
    except ValueError as exc:
        raise ValueError(f"{name} must be a number, got `{raw}`") from exc


def parse_int_list(name: str, raw: str) -> list[int]:
    value = raw.strip()
    if not value:
        return []
    out: list[int] = []
    for item in value.split(","):
        item = item.strip()
        if not item:
            continue
        out.append(parse_int(name, item))
    return out


def parse_string_list(_: str, raw: str) -> list[str]:
    value = raw.strip()
    if not value:
        return []
    return [item.strip() for item in value.split(",") if item.strip()]


def parse_json_list(name: str, raw: str) -> list[dict[str, Any]]:
    try:
        loaded = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise ValueError(f"{name} must be valid JSON") from exc
    if not isinstance(loaded, list):
        raise ValueError(f"{name} must decode to a JSON list")
    for idx, item in enumerate(loaded):
        if not isinstance(item, dict):
            raise ValueError(f"{name}[{idx}] must be a JSON object")
    return loaded


def set_nested(root: dict[str, Any], path: tuple[str, ...], value: Any) -> None:
    node = root
    for part in path[:-1]:
        child = node.get(part)
        if not isinstance(child, dict):
            child = {}
            node[part] = child
        node = child
    node[path[-1]] = value


def apply_override(
    root: dict[str, Any],
    path: tuple[str, ...],
    env_name: str,
    parser: Callable[[str, str], Any],
) -> None:
    if env_name not in os.environ:
        return
    raw = os.environ[env_name]
    set_nested(root, path, parser(env_name, raw))


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        loaded = yaml.safe_load(f)
    if not isinstance(loaded, dict):
        raise ValueError(f"template config at {path} must be a mapping")
    return loaded


def write_yaml(path: Path, content: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(content, f, sort_keys=False)


def main() -> None:
    config_file = os.getenv("BRIDGE_CONFIG_FILE", "").strip()
    if config_file:
        path = Path(config_file)
        if not path.exists():
            raise SystemExit(f"BRIDGE_CONFIG_FILE does not exist: {path}")
        print(f"Using provided config file: {path}")
    else:
        template_path = Path(
            os.getenv("BRIDGE_CONFIG_TEMPLATE", "/opt/fanuc/config/bridge_config.yaml")
        )
        generated_path = Path(
            os.getenv("BRIDGE_CONFIG_OUT", "/tmp/bridge_config.generated.yaml")
        )

        config = load_yaml(template_path)

        # For containers targeting real controllers, auto tries fanuc_rmi first and
        # falls back to the simulator when fanuc_rmi is unavailable.
        if "ROBOT_BACKEND" not in os.environ:
            set_nested(config, ("robot", "backend"), "auto")

        overrides: list[tuple[tuple[str, ...], str, Callable[[str, str], Any]]] = [
            (("robot", "ip"), "ROBOT_IP", lambda _, v: v),
            (("robot", "rmi_port"), "ROBOT_RMI_PORT", parse_int),
            (("robot", "timeout_sec"), "ROBOT_TIMEOUT_SEC", parse_float),
            (("robot", "backend"), "ROBOT_BACKEND", lambda _, v: v),
            (("mqtt", "host"), "MQTT_HOST", lambda _, v: v),
            (("mqtt", "port"), "MQTT_PORT", parse_int),
            (("mqtt", "username"), "MQTT_USERNAME", lambda _, v: v),
            (("mqtt", "password"), "MQTT_PASSWORD", lambda _, v: v),
            (("mqtt", "client_id"), "MQTT_CLIENT_ID", lambda _, v: v),
            (("mqtt", "keepalive_sec"), "MQTT_KEEPALIVE_SEC", parse_int),
            (("mqtt", "qos"), "MQTT_QOS", parse_int),
            (("mqtt", "retain"), "MQTT_RETAIN", parse_bool),
            (("publish", "robot_id"), "PUBLISH_ROBOT_ID", lambda _, v: v),
            (("publish", "base_topic"), "PUBLISH_BASE_TOPIC", lambda _, v: v),
            (("publish", "rate_hz"), "PUBLISH_RATE_HZ", parse_int),
            (
                ("publish", "include_timestamp"),
                "PUBLISH_INCLUDE_TIMESTAMP",
                parse_bool,
            ),
            (("telemetry", "joints"), "TELEMETRY_JOINTS", parse_bool),
            (("telemetry", "status"), "TELEMETRY_STATUS", parse_bool),
            (("telemetry", "status_ext"), "TELEMETRY_STATUS_EXT", parse_bool),
            (
                ("telemetry", "digital_inputs"),
                "TELEMETRY_DIGITAL_INPUTS",
                parse_int_list,
            ),
            (
                ("telemetry", "digital_outputs"),
                "TELEMETRY_DIGITAL_OUTPUTS",
                parse_int_list,
            ),
            (("telemetry", "flags"), "TELEMETRY_FLAGS", parse_int_list),
            (
                ("telemetry", "analog_inputs"),
                "TELEMETRY_ANALOG_INPUTS",
                parse_int_list,
            ),
            (
                ("telemetry", "analog_outputs"),
                "TELEMETRY_ANALOG_OUTPUTS",
                parse_int_list,
            ),
            (
                ("telemetry", "group_inputs"),
                "TELEMETRY_GROUP_INPUTS",
                parse_int_list,
            ),
            (
                ("telemetry", "group_outputs"),
                "TELEMETRY_GROUP_OUTPUTS",
                parse_int_list,
            ),
            (
                ("telemetry", "num_registers"),
                "TELEMETRY_NUM_REGISTERS",
                parse_int_list,
            ),
            (
                ("telemetry", "pos_registers"),
                "TELEMETRY_POS_REGISTERS",
                parse_int_list,
            ),
            (
                ("telemetry", "variables"),
                "TELEMETRY_VARIABLES",
                parse_string_list,
            ),
            (("writes", "enabled"), "WRITES_ENABLED", parse_bool),
            (
                ("writes", "startup_actions"),
                "WRITES_STARTUP_ACTIONS_JSON",
                parse_json_list,
            ),
            (
                ("writes", "cyclic_actions"),
                "WRITES_CYCLIC_ACTIONS_JSON",
                parse_json_list,
            ),
        ]

        for path_tuple, env_name, parser in overrides:
            apply_override(config, path_tuple, env_name, parser)

        # If robot ID changes and base topic was not set explicitly, keep a sane default.
        if "PUBLISH_ROBOT_ID" in os.environ and "PUBLISH_BASE_TOPIC" not in os.environ:
            robot_id = str(config.get("publish", {}).get("robot_id", "")).strip()
            if robot_id:
                set_nested(config, ("publish", "base_topic"), f"fanuc/{robot_id}")

        write_yaml(generated_path, config)
        path = generated_path
        print(f"Generated config file: {path}")

    os.execvp(
        "ros2",
        [
            "ros2",
            "run",
            "fanuc_mqtt_bridge",
            "fanuc_mqtt_bridge",
            "--ros-args",
            "-p",
            f"config_file:={path}",
        ],
    )


if __name__ == "__main__":
    main()
