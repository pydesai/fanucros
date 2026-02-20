# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import importlib
import time
from typing import Any, Callable, Dict, List, Optional

from .config import RobotConfig, TelemetryConfig, WriteAction
from .simulated_rmi import SimulatedRMIConnection


class FanucAdapter:
    def __init__(
        self,
        robot_config: RobotConfig,
        connection_factory: Optional[Callable[[str, int], Any]] = None,
    ):
        self._cfg = robot_config
        self._connection_factory = connection_factory or self._resolve_connection_factory(robot_config.backend)
        self._conn = None

    def _resolve_connection_factory(self, backend: str) -> Callable[[str, int], Any]:
        if backend == "sim":
            return lambda robot_ip, rmi_port: SimulatedRMIConnection(robot_ip, rmi_port)
        if backend == "fanuc_rmi":
            return self._default_connection_factory
        if backend == "auto":
            try:
                importlib.import_module("fanuc_rmi")
                return self._default_connection_factory
            except ImportError:
                return lambda robot_ip, rmi_port: SimulatedRMIConnection(robot_ip, rmi_port)
        raise ValueError(f"unsupported backend `{backend}`")

    @staticmethod
    def _default_connection_factory(robot_ip: str, rmi_port: int) -> Any:
        try:
            module = importlib.import_module("fanuc_rmi")
        except ImportError as e:
            raise RuntimeError(
                "Python module `fanuc_rmi` is required for direct RMI polling backend. "
                "Install a compatible FANUC RMI Python binding or inject a custom connection factory."
            ) from e

        if not hasattr(module, "RMIConnection"):
            raise RuntimeError("`fanuc_rmi` module does not expose `RMIConnection`")
        return module.RMIConnection(robot_ip, rmi_port)

    @property
    def connected(self) -> bool:
        return self._conn is not None

    def _invoke(self, names: List[str], *args, **kwargs) -> Any:
        if self._conn is None:
            raise RuntimeError("FANUC RMI adapter is not connected")
        for name in names:
            fn = getattr(self._conn, name, None)
            if callable(fn):
                return fn(*args, **kwargs)
        raise AttributeError(f"No matching method for names {names}")

    @staticmethod
    def _to_dict(resp: Any) -> Dict[str, Any]:
        if isinstance(resp, dict):
            return resp
        if hasattr(resp, "__dict__"):
            return {
                k: v
                for k, v in vars(resp).items()
                if not k.startswith("_") and not callable(v)
            }
        return {"value": resp}

    def connect_with_retry(self, max_attempts: int = 0) -> None:
        attempt = 0
        backoff = 1.0
        while True:
            attempt += 1
            try:
                self._conn = self._connection_factory(self._cfg.ip, self._cfg.rmi_port)
                self._invoke(["connect", "ConnectROS2"], self._cfg.timeout_sec)
                try:
                    self._invoke(["initializeRemoteMotion", "initialize_remote_motion"], self._cfg.timeout_sec)
                except Exception:
                    # Some controllers require reset/abort before initialization.
                    self._invoke(["reset", "ResetRobot"], self._cfg.timeout_sec)
                    self._invoke(["abort", "Abort"], self._cfg.timeout_sec)
                    self._invoke(["initializeRemoteMotion", "initialize_remote_motion"], self._cfg.timeout_sec)
                return
            except Exception:
                self._conn = None
                if max_attempts > 0 and attempt >= max_attempts:
                    raise
                time.sleep(backoff)
                backoff = min(backoff * 2.0, 10.0)

    def disconnect(self) -> None:
        if self._conn is None:
            return
        try:
            self._invoke(["disconnect", "Disconnect"], self._cfg.timeout_sec)
        finally:
            self._conn = None

    def read_joints(self) -> Dict[str, Any]:
        resp = self._invoke(["readJointAngles", "read_joint_angles"], None, self._cfg.timeout_sec)
        return self._to_dict(resp)

    def read_status(self) -> Dict[str, Any]:
        resp = self._invoke(["getStatus", "get_status"], self._cfg.timeout_sec)
        return self._to_dict(resp)

    def read_status_ext(self) -> Dict[str, Any]:
        resp = self._invoke(["getExtendedStatus", "get_extended_status"], self._cfg.timeout_sec)
        return self._to_dict(resp)

    def read_digital(self, telemetry: TelemetryConfig) -> Dict[str, Any]:
        data: Dict[str, Any] = {"inputs": {}, "outputs": {}, "flags": {}}
        for idx in telemetry.digital_inputs:
            resp = self._invoke(["readDigitalInputPort", "read_digital_input_port"], idx, self._cfg.timeout_sec)
            data["inputs"][str(idx)] = self._to_dict(resp)
        for idx in telemetry.digital_outputs:
            resp = self._invoke(["readIOPort", "read_io_port"], "DO", idx, self._cfg.timeout_sec)
            data["outputs"][str(idx)] = self._to_dict(resp)
        for idx in telemetry.flags:
            resp = self._invoke(["readIOPort", "read_io_port"], "FLAG", idx, self._cfg.timeout_sec)
            data["flags"][str(idx)] = self._to_dict(resp)
        return data

    def read_analog(self, telemetry: TelemetryConfig) -> Dict[str, Any]:
        data: Dict[str, Any] = {"inputs": {}, "outputs": {}}
        for idx in telemetry.analog_inputs:
            resp = self._invoke(["readIOPort", "read_io_port"], "AI", idx, self._cfg.timeout_sec)
            data["inputs"][str(idx)] = self._to_dict(resp)
        for idx in telemetry.analog_outputs:
            resp = self._invoke(["readIOPort", "read_io_port"], "AO", idx, self._cfg.timeout_sec)
            data["outputs"][str(idx)] = self._to_dict(resp)
        return data

    def read_group(self, telemetry: TelemetryConfig) -> Dict[str, Any]:
        data: Dict[str, Any] = {"inputs": {}, "outputs": {}}
        for idx in telemetry.group_inputs:
            resp = self._invoke(["readIOPort", "read_io_port"], "GI", idx, self._cfg.timeout_sec)
            data["inputs"][str(idx)] = self._to_dict(resp)
        for idx in telemetry.group_outputs:
            resp = self._invoke(["readIOPort", "read_io_port"], "GO", idx, self._cfg.timeout_sec)
            data["outputs"][str(idx)] = self._to_dict(resp)
        return data

    def read_num_registers(self, telemetry: TelemetryConfig) -> Dict[str, Any]:
        data: Dict[str, Any] = {}
        for idx in telemetry.num_registers:
            resp = self._invoke(["readNumericRegister", "read_numeric_register"], idx, self._cfg.timeout_sec)
            data[str(idx)] = self._to_dict(resp)
        return data

    def read_pos_registers(self, telemetry: TelemetryConfig) -> Dict[str, Any]:
        data: Dict[str, Any] = {}
        for idx in telemetry.pos_registers:
            resp = self._invoke(["readPositionRegister", "read_position_register"], idx, self._cfg.timeout_sec)
            data[str(idx)] = self._to_dict(resp)
        return data

    def read_variables(self, telemetry: TelemetryConfig) -> Dict[str, Any]:
        data: Dict[str, Any] = {}
        for var_name in telemetry.variables:
            resp = self._invoke(["readVariablePacket", "read_variable_packet"], var_name, self._cfg.timeout_sec)
            data[var_name] = self._to_dict(resp)
        return data

    def execute_action(self, action: WriteAction) -> Dict[str, Any]:
        p = action.params
        if action.action == "set_gen_override":
            resp = self._invoke(["setSpeedOverride", "set_speed_override"], p["value"], self._cfg.timeout_sec)
        elif action.action == "set_num_register":
            resp = self._invoke(
                ["writeNumericRegister", "write_numeric_register"],
                p["index"],
                p["value"],
                self._cfg.timeout_sec,
            )
        elif action.action == "set_io_port":
            io_type = "FLAG" if str(p["io_type"]).upper() == "F" else str(p["io_type"]).upper()
            resp = self._invoke(
                ["writeIOPort", "write_io_port"],
                p["index"],
                io_type,
                p["value"],
                self._cfg.timeout_sec,
            )
        elif action.action == "set_payload_schedule":
            resp = self._invoke(
                ["setPayloadSchedule", "set_payload_schedule"],
                p["payload_schedule_id"],
                self._cfg.timeout_sec,
            )
        elif action.action == "set_payload_value":
            resp = self._invoke(
                ["setPayloadValue", "set_payload_value"],
                p["payload_schedule_id"],
                p["mass"],
                p["cg_x"],
                p["cg_y"],
                p["cg_z"],
                p["use_in"],
                p["in_x"],
                p["in_y"],
                p["in_z"],
                self._cfg.timeout_sec,
            )
        elif action.action == "set_payload_comp":
            resp = self._invoke(
                ["setPayloadComp", "set_payload_comp"],
                p["payload_schedule_id"],
                p["mass"],
                p["cg_x"],
                p["cg_y"],
                p["cg_z"],
                p["in_x"],
                p["in_y"],
                p["in_z"],
                self._cfg.timeout_sec,
            )
        else:
            raise ValueError(f"unsupported action: {action.action}")

        return self._to_dict(resp)
