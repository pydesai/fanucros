# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import math
import time
from typing import Any, Dict


class SimulatedRMIConnection:
    """In-process simulator that mimics a subset of FANUC RMI behavior."""

    def __init__(self, robot_ip: str, rmi_port: int = 16001):
        self.robot_ip = robot_ip
        self.rmi_port = rmi_port
        self.connected = False
        self.initialized = False
        self._start_time = time.monotonic()
        self._gen_override = 100
        self._payload_schedule_id = 1
        self._payload_value: Dict[str, Any] = {}
        self._payload_comp: Dict[str, Any] = {}
        self._num_regs: Dict[int, float] = {i: float(i) for i in range(1, 201)}
        self._pos_regs: Dict[int, Dict[str, Any]] = {}
        self._io: Dict[str, Dict[int, float]] = {
            "DI": {},
            "DO": {},
            "FLAG": {},
            "AI": {},
            "AO": {},
            "GI": {},
            "GO": {},
        }

    def _assert_ready(self) -> None:
        if not self.connected:
            raise RuntimeError("sim RMI not connected")

    def _elapsed(self) -> float:
        return time.monotonic() - self._start_time

    def connect(self, timeout: float):
        self.connected = True
        return {"ErrorID": 0, "connected": True}

    def disconnect(self, timeout: float):
        self.connected = False
        self.initialized = False
        return {"ErrorID": 0, "connected": False}

    def initializeRemoteMotion(self, timeout: float):
        self._assert_ready()
        self.initialized = True
        return {"ErrorID": 0, "initialized": True}

    def reset(self, timeout: float):
        self._assert_ready()
        return {"ErrorID": 0}

    def abort(self, timeout: float):
        self._assert_ready()
        return {"ErrorID": 0}

    def readJointAngles(self, group, timeout: float):
        self._assert_ready()
        t = self._elapsed()
        j = {}
        for i in range(1, 10):
            phase = t * 0.4 + i * 0.2
            j[f"J{i}"] = 20.0 * math.sin(phase)
        return {"ErrorID": 0, **j}

    def getStatus(self, timeout: float):
        self._assert_ready()
        t = self._elapsed()
        in_motion = abs(math.sin(t * 0.5)) > 0.05
        return {
            "ErrorID": 0,
            "in_error": False,
            "tp_enabled": True,
            "e_stopped": False,
            "motion_possible": True,
            "contact_stop_mode": 0,
            "in_motion": in_motion,
        }

    def getExtendedStatus(self, timeout: float):
        self._assert_ready()
        return {
            "ErrorID": 0,
            "DrivesPowered": True,
            "ErrorCode": "",
            "GenOverride": self._gen_override,
            "InMotion": abs(math.sin(self._elapsed() * 0.5)) > 0.05,
            "SpeedClampLimit": 100.0,
            "ControlMode": "AUTO",
        }

    def readDigitalInputPort(self, idx: int, timeout: float):
        self._assert_ready()
        value = int((self._elapsed() * 2.0 + idx) % 2)
        return {"ErrorID": 0, "PortNumber": idx, "PortValue": bool(value)}

    def readIOPort(self, io_type: str, idx: int, timeout: float):
        self._assert_ready()
        t = self._elapsed()
        k = io_type.upper()
        if k == "F":
            k = "FLAG"

        if k in {"DO", "FLAG", "AO", "GO"} and idx in self._io[k]:
            value = self._io[k][idx]
        elif k == "AI":
            value = round((math.sin(t + idx * 0.1) + 1.0) * 5.0, 3)
        elif k in {"GI", "GO"}:
            value = int((t * 3 + idx) % 16)
        else:
            value = int((t + idx) % 2)

        return {"ErrorID": 0, "PortType": k, "PortNumber": idx, "PortValue": value}

    def writeIOPort(self, idx: int, io_type: str, value, timeout: float):
        self._assert_ready()
        k = io_type.upper()
        if k == "F":
            k = "FLAG"
        if k not in self._io:
            self._io[k] = {}
        self._io[k][idx] = value
        return {"ErrorID": 0}

    def readNumericRegister(self, idx: int, timeout: float):
        self._assert_ready()
        value = self._num_regs.get(idx, 0.0)
        return {"ErrorID": 0, "RegisterNumber": idx, "RegisterValue": value}

    def writeNumericRegister(self, idx: int, value, timeout: float):
        self._assert_ready()
        self._num_regs[idx] = float(value)
        return {"ErrorID": 0}

    def readPositionRegister(self, idx: int, timeout: float):
        self._assert_ready()
        if idx not in self._pos_regs:
            t = self._elapsed()
            self._pos_regs[idx] = {
                "Representation": "Cartesian",
                "X": round(100.0 + 10.0 * math.sin(t), 3),
                "Y": round(200.0 + 10.0 * math.cos(t), 3),
                "Z": 300.0,
                "W": 0.0,
                "P": 0.0,
                "R": 0.0,
                "J1": 0.0,
                "J2": 0.0,
                "J3": 0.0,
                "J4": 0.0,
                "J5": 0.0,
                "J6": 0.0,
                "J7": 0.0,
                "J8": 0.0,
                "J9": 0.0,
            }
        return {"ErrorID": 0, **self._pos_regs[idx]}

    def readVariablePacket(self, var_name: str, timeout: float):
        self._assert_ready()
        if var_name == "$STMO.$COM_INT":
            value = 8
        elif var_name == "$MOR_GRP[1].$ROB_MOVE":
            value = int(abs(math.sin(self._elapsed() * 0.5)) > 0.05)
        else:
            value = 0
        return {"ErrorID": 0, "VariableName": var_name, "VariableValue": value}

    def setSpeedOverride(self, value: int, timeout: float):
        self._assert_ready()
        self._gen_override = int(value)
        return {"ErrorID": 0}

    def setPayloadSchedule(self, payload_schedule_id: int, timeout: float):
        self._assert_ready()
        self._payload_schedule_id = int(payload_schedule_id)
        return {"ErrorID": 0}

    def setPayloadValue(
        self,
        payload_schedule_id: int,
        mass: float,
        cg_x: float,
        cg_y: float,
        cg_z: float,
        use_in: bool,
        in_x: float,
        in_y: float,
        in_z: float,
        timeout: float,
    ):
        self._assert_ready()
        self._payload_value = {
            "payload_schedule_id": payload_schedule_id,
            "mass": mass,
            "cg_x": cg_x,
            "cg_y": cg_y,
            "cg_z": cg_z,
            "use_in": use_in,
            "in_x": in_x,
            "in_y": in_y,
            "in_z": in_z,
        }
        return {"ErrorID": 0}

    def setPayloadComp(
        self,
        payload_schedule_id: int,
        mass: float,
        cg_x: float,
        cg_y: float,
        cg_z: float,
        in_x: float,
        in_y: float,
        in_z: float,
        timeout: float,
    ):
        self._assert_ready()
        self._payload_comp = {
            "payload_schedule_id": payload_schedule_id,
            "mass": mass,
            "cg_x": cg_x,
            "cg_y": cg_y,
            "cg_z": cg_z,
            "in_x": in_x,
            "in_y": in_y,
            "in_z": in_z,
        }
        return {"ErrorID": 0}
