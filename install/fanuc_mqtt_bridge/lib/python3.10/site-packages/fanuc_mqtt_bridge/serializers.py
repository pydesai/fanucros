# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import json
import time
from typing import Any, Dict

SOURCE_NAME = "fanuc_mqtt_bridge"

TOPIC_SUFFIX = {
    "connection": "connection",
    "joints": "joints",
    "status": "status",
    "status_ext": "status_ext",
    "io_digital": "io/digital",
    "io_analog": "io/analog",
    "io_group": "io/group",
    "registers_num": "registers/num",
    "registers_pos": "registers/pos",
    "variables": "variables",
    "errors": "errors",
}


def topic_for(base_topic: str, category: str) -> str:
    suffix = TOPIC_SUFFIX.get(category)
    if suffix is None:
        raise KeyError(f"unsupported topic category `{category}`")
    return f"{base_topic.rstrip('/')}/{suffix}"


def build_payload(robot_id: str, sequence: int, data: Dict[str, Any], include_timestamp: bool = True) -> str:
    envelope: Dict[str, Any] = {
        "robot_id": robot_id,
        "source": SOURCE_NAME,
        "sequence": sequence,
        "data": data,
    }
    if include_timestamp:
        envelope["ts_unix_ms"] = int(time.time() * 1000)
    return json.dumps(envelope, separators=(",", ":"), sort_keys=True)


def build_error_data(error_type: str, message: str, operation: str, recoverable: bool) -> Dict[str, Any]:
    return {
        "error_type": error_type,
        "message": message,
        "operation": operation,
        "recoverable": recoverable,
    }
