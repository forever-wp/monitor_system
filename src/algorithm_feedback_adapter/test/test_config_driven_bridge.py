import tempfile
from pathlib import Path

import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import BatteryState

from algorithm_feedback_adapter.config_driven_bridge import (
    ConfigDrivenBridge,
    build_feedback_messages,
    get_field_value,
    import_message_type,
    load_spec,
    resolve_spec_path,
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
            "validate_bridge_entry should reject incomplete algorithm_feedback_adapter entries")


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


def test_resolve_spec_path_accepts_existing_absolute_path(tmp_path):
    spec_file = tmp_path / "spec.yaml"
    spec_file.write_text("bridges: []")

    assert resolve_spec_path(str(spec_file)) == spec_file


def test_resolve_spec_path_does_not_expand_relative_package_path():
    unresolved = Path("config/examples/algorithm_feedback_adapter_spec.yaml")

    assert resolve_spec_path(str(unresolved)) == unresolved


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


def test_reload_spec_applies_new_bridge_entries(tmp_path):
    if not rclpy.ok():
        rclpy.init()

    spec_file = tmp_path / "spec.yaml"
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

    node = ConfigDrivenBridge(
        parameter_overrides=[
            Parameter("spec_file", value=str(spec_file)),
            Parameter("spec_reload_enabled", value=False),
        ]
    )
    try:
        assert node.bridge_ids() == ["battery"]

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
  - id: battery_aux
    message_type: sensor_msgs/msg/BatteryState
    input_topic: /battery_state_aux
    output_topic: /nav2_monitor/algorithm_feedback
    module_name: battery_aux_node
    topic_name: /battery_state_aux
    metrics:
      - name: battery_voltage
        field: voltage
        valid_field: present
"""
        )

        assert node.reload_spec_if_needed(force=True) is True
        assert node.bridge_ids() == ["battery", "battery_aux"]
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_reload_invalid_spec_keeps_previous_bridges(tmp_path):
    if not rclpy.ok():
        rclpy.init()

    spec_file = tmp_path / "spec.yaml"
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

    node = ConfigDrivenBridge(
        parameter_overrides=[
            Parameter("spec_file", value=str(spec_file)),
            Parameter("spec_reload_enabled", value=False),
        ]
    )
    try:
        assert node.bridge_ids() == ["battery"]

        spec_file.write_text(
            """bridges:
  - id: broken
    message_type: sensor_msgs/msg/BatteryState
    input_topic: /broken
    output_topic: /nav2_monitor/algorithm_feedback
    module_name: broken_node
    topic_name: /broken
"""
        )

        assert node.reload_spec_if_needed(force=True) is False
        assert node.bridge_ids() == ["battery"]
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
