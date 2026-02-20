import pytest

from fanuc_mqtt_bridge.config import RobotConfig, TelemetryConfig, WriteAction
from fanuc_mqtt_bridge.fanuc_adapter import FanucAdapter


class FakeConn:
    def __init__(self):
        self.calls = []

    def connect(self, timeout):
        self.calls.append(("connect", timeout))
        return {"ok": True}

    def initializeRemoteMotion(self, timeout):
        self.calls.append(("initializeRemoteMotion", timeout))
        return {"ok": True}

    def disconnect(self, timeout):
        self.calls.append(("disconnect", timeout))
        return {"ok": True}

    def readJointAngles(self, group, timeout):
        self.calls.append(("readJointAngles", group, timeout))
        return {"J1": 0.1}

    def getStatus(self, timeout):
        self.calls.append(("getStatus", timeout))
        return {"in_error": False}

    def getExtendedStatus(self, timeout):
        self.calls.append(("getExtendedStatus", timeout))
        return {"drives_powered": True}

    def readDigitalInputPort(self, idx, timeout):
        return {"index": idx, "value": True}

    def readIOPort(self, io_type, idx, timeout):
        return {"type": io_type, "index": idx, "value": 1}

    def readNumericRegister(self, idx, timeout):
        return {"index": idx, "value": 7.0}

    def readPositionRegister(self, idx, timeout):
        return {"index": idx, "x": 1.0}

    def readVariablePacket(self, var_name, timeout):
        return {"name": var_name, "value": 123}

    def setSpeedOverride(self, value, timeout):
        return {"set": value}


def test_adapter_connect_and_reads():
    conn = FakeConn()
    adapter = FanucAdapter(RobotConfig(ip="192.168.1.100"), connection_factory=lambda ip, p: conn)
    adapter.connect_with_retry(max_attempts=1)

    joints = adapter.read_joints()
    status = adapter.read_status()
    status_ext = adapter.read_status_ext()

    assert joints["J1"] == 0.1
    assert status["in_error"] is False
    assert status_ext["drives_powered"] is True


def test_adapter_reads_groups():
    conn = FakeConn()
    adapter = FanucAdapter(RobotConfig(ip="192.168.1.100"), connection_factory=lambda ip, p: conn)
    adapter.connect_with_retry(max_attempts=1)
    telem = TelemetryConfig(
        digital_inputs=[81],
        digital_outputs=[101],
        flags=[1],
        analog_inputs=[1],
        analog_outputs=[2],
        group_inputs=[1],
        group_outputs=[2],
        num_registers=[1],
        pos_registers=[1],
        variables=["$A"],
    )

    assert "81" in adapter.read_digital(telem)["inputs"]
    assert "2" in adapter.read_analog(telem)["outputs"]
    assert "2" in adapter.read_group(telem)["outputs"]
    assert "1" in adapter.read_num_registers(telem)
    assert "1" in adapter.read_pos_registers(telem)
    assert "$A" in adapter.read_variables(telem)


def test_execute_action_dispatch():
    conn = FakeConn()
    adapter = FanucAdapter(RobotConfig(ip="192.168.1.100"), connection_factory=lambda ip, p: conn)
    adapter.connect_with_retry(max_attempts=1)
    result = adapter.execute_action(WriteAction(action="set_gen_override", params={"value": 50}))
    assert result["set"] == 50


def test_forced_fanuc_backend_missing_module_fails():
    adapter = FanucAdapter(RobotConfig(ip="192.168.1.100", backend="fanuc_rmi"))
    with pytest.raises(RuntimeError):
        adapter.connect_with_retry(max_attempts=1)


def test_sim_backend_works_without_external_module():
    adapter = FanucAdapter(RobotConfig(ip="192.168.1.100", backend="sim"))
    adapter.connect_with_retry(max_attempts=1)
    joints = adapter.read_joints()
    assert "J1" in joints
