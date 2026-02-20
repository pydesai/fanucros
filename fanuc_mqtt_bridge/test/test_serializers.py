import json

from fanuc_mqtt_bridge.serializers import build_error_data, build_payload, topic_for


def test_topic_for_routes_correctly():
    assert topic_for("fanuc/r1", "joints") == "fanuc/r1/joints"
    assert topic_for("fanuc/r1/", "errors") == "fanuc/r1/errors"


def test_build_payload_has_required_fields():
    payload = build_payload("robot_1", 42, {"x": 1}, include_timestamp=True)
    doc = json.loads(payload)
    assert doc["robot_id"] == "robot_1"
    assert doc["source"] == "fanuc_mqtt_bridge"
    assert doc["sequence"] == 42
    assert "ts_unix_ms" in doc
    assert doc["data"] == {"x": 1}


def test_build_error_data_shape():
    err = build_error_data("read_error", "oops", "read_joints", True)
    assert err["error_type"] == "read_error"
    assert err["message"] == "oops"
    assert err["operation"] == "read_joints"
    assert err["recoverable"] is True
