from pathlib import Path

import pytest

from fanuc_mqtt_bridge.config import ConfigError, load_bridge_config


def _write(tmp_path: Path, content: str) -> str:
    p = tmp_path / "cfg.yaml"
    p.write_text(content, encoding="utf-8")
    return str(p)


def test_load_valid_config_defaults(tmp_path: Path):
    path = _write(
        tmp_path,
        """
robot:
  ip: 192.168.1.100
mqtt:
  host: localhost
  username: user
  password: pass
publish:
  robot_id: robot_a
""",
    )
    cfg = load_bridge_config(path)
    assert cfg.robot.rmi_port == 16001
    assert cfg.publish.rate_hz == 10
    assert cfg.telemetry.joints is True
    assert cfg.writes.enabled is False


def test_missing_required_field_fails(tmp_path: Path):
    path = _write(
        tmp_path,
        """
robot:
  ip: 192.168.1.100
mqtt:
  host: localhost
publish:
  rate_hz: 10
""",
    )
    with pytest.raises(ConfigError):
        load_bridge_config(path)


def test_unknown_action_fails(tmp_path: Path):
    path = _write(
        tmp_path,
        """
robot:
  ip: 192.168.1.100
mqtt:
  host: localhost
publish:
  robot_id: robot_b
writes:
  enabled: true
  startup_actions:
    - action: not_real
      value: 1
""",
    )
    with pytest.raises(ConfigError):
        load_bridge_config(path)


def test_missing_action_keys_fails(tmp_path: Path):
    path = _write(
        tmp_path,
        """
robot:
  ip: 192.168.1.100
mqtt:
  host: localhost
publish:
  robot_id: robot_c
writes:
  enabled: true
  startup_actions:
    - action: set_num_register
      index: 1
""",
    )
    with pytest.raises(ConfigError):
        load_bridge_config(path)


def test_invalid_backend_fails(tmp_path: Path):
    path = _write(
        tmp_path,
        """
robot:
  ip: 192.168.1.100
  backend: bad_backend
mqtt:
  host: localhost
publish:
  robot_id: robot_d
""",
    )
    with pytest.raises(ConfigError):
        load_bridge_config(path)
