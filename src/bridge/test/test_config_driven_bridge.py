import tempfile
from pathlib import Path

from sensor_msgs.msg import BatteryState

from bridge.config_driven_bridge import (
    build_feedback_messages,
    get_field_value,
    import_message_type,
    load_spec,
    validate_bridge_entry,
)


def test_import_message_type_from_ros_string():
    msg_type = import_message_type("sensor_msgs/msg/BatteryState")
    assert msg_type.__name__ == "BatteryState"


def test_get_field_value_supports_dotted_paths():
    class Inner:
        value = 12.5

    class Outer:
        inner = Inner()

    assert get_field_value(Outer(), "inner.value") == 12.5


def test_validate_bridge_entry_rejects_missing_fields():
    try:
        validate_bridge_entry({"id": "battery"})
    except ValueError as exc:
        assert "missing keys" in str(exc)
    else:
        raise AssertionError(
            "validate_bridge_entry should reject incomplete bridge entries")


def test_load_spec_reads_multi_bridge_yaml_file():
    with tempfile.TemporaryDirectory() as tmpdir:
        spec_file = Path(tmpdir) / "spec.yaml"
        spec_file.write_text(
            """bridges:
  - id: battery
    message_type: sensor_msgs/msg/BatteryState
    input_topic: /battery_state
    output_topic: /nav2_monitor/algorithm_feedback
    module_name: battery_node
    topic_name: /battery_state
    metrics:
      - name: battery_percentage
        field: percentage
        valid_field: present
"""
        )
        spec = load_spec(spec_file)
        assert spec["bridges"][0]["message_type"] == "sensor_msgs/msg/BatteryState"
        assert spec["bridges"][0]["metrics"][0]["name"] == "battery_percentage"


def test_build_feedback_messages_uses_bridge_spec():
    msg = BatteryState()
    msg.percentage = 0.35
    msg.temperature = 28.0
    msg.present = True

    bridge_spec = {
        "id": "battery",
        "message_type": "sensor_msgs/msg/BatteryState",
        "input_topic": "/battery_state",
        "output_topic": "/nav2_monitor/algorithm_feedback",
        "module_name": "battery_node",
        "topic_name": "/battery_state",
        "metrics": [
            {
                "name": "battery_percentage",
                "field": "percentage",
                "valid_field": "present",
            },
            {
                "name": "battery_temperature",
                "field": "temperature",
                "valid_default": True,
            },
        ],
    }

    feedback_messages = build_feedback_messages(msg, bridge_spec, None)
    assert len(feedback_messages) == 2
    assert feedback_messages[0].module_name == "battery_node"
    assert feedback_messages[0].topic_name == "/battery_state"
    assert feedback_messages[0].metric_name == "battery_percentage"
    assert abs(feedback_messages[0].value - 0.35) < 1e-6
    assert feedback_messages[0].valid is True
    assert feedback_messages[1].metric_name == "battery_temperature"
