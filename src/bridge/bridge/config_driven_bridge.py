from __future__ import annotations

import importlib
from pathlib import Path
from typing import Any

import rclpy
from builtin_interfaces.msg import Time
from nav2_monitor.msg import AlgorithmFeedback
from rclpy.node import Node
import yaml

REQUIRED_SPEC_KEYS = {"bridges"}
REQUIRED_BRIDGE_KEYS = {
    "id",
    "message_type",
    "input_topic",
    "output_topic",
    "module_name",
    "topic_name",
    "metrics",
}
REQUIRED_METRIC_KEYS = {"name", "field"}


def import_message_type(type_str: str):
    pkg, sep, msg_name = type_str.partition("/msg/")
    if not sep or not pkg or not msg_name:
        raise ValueError(f"Invalid message_type '{type_str}', expected '<pkg>/msg/<Msg>'")
    module = importlib.import_module(f"{pkg}.msg")
    return getattr(module, msg_name)


def get_field_value(message: Any, field_path: str):
    current = message
    for field in field_path.split("."):
        current = getattr(current, field)
    return current


def resolve_spec_path(spec_file: str) -> Path:
    path = Path(spec_file)
    if path.is_absolute() and path.exists():
        return path

    return path


def validate_bridge_entry(entry: dict):
    missing = REQUIRED_BRIDGE_KEYS.difference(entry.keys())
    if missing:
        raise ValueError(f"Bridge entry missing keys: {sorted(missing)}")
    if not isinstance(entry["metrics"], list) or not entry["metrics"]:
        raise ValueError(
            "Bridge entry '%s' must define a non-empty metrics list"
            % entry.get("id", "<unknown>")
        )
    for metric in entry["metrics"]:
        metric_missing = REQUIRED_METRIC_KEYS.difference(metric.keys())
        if metric_missing:
            raise ValueError(
                "Bridge entry '%s' has metric missing keys: %s"
                % (entry["id"], sorted(metric_missing))
            )
    return entry


def load_spec(spec_path: str | Path) -> dict:
    path = Path(spec_path)
    data = yaml.safe_load(path.read_text()) or {}
    missing = REQUIRED_SPEC_KEYS.difference(data.keys())
    if missing:
        raise ValueError(f"Spec file '{path}' missing keys: {sorted(missing)}")
    if not isinstance(data["bridges"], list) or not data["bridges"]:
        raise ValueError(f"Spec file '{path}' must define a non-empty bridges list")
    data["bridges"] = [validate_bridge_entry(entry) for entry in data["bridges"]]
    return data


def build_feedback_messages(msg: Any, bridge_spec: dict, stamp) -> list[AlgorithmFeedback]:
    if stamp is None:
        stamp = Time()

    feedback_messages = []
    for metric in bridge_spec["metrics"]:
        value = float(get_field_value(msg, metric["field"]))
        if "valid_field" in metric:
            valid = bool(get_field_value(msg, metric["valid_field"]))
        else:
            valid = bool(metric.get("valid_default", True))

        fb = AlgorithmFeedback()
        fb.stamp = stamp
        fb.module_name = bridge_spec["module_name"]
        fb.topic_name = bridge_spec["topic_name"]
        fb.metric_name = metric["name"]
        fb.value = value
        fb.valid = valid
        feedback_messages.append(fb)
    return feedback_messages


class ConfigDrivenBridge(Node):
    def __init__(self):
        super().__init__("bridge_py_node")

        spec_file = self.declare_parameter(
            "spec_file", "/opt/ry/config/Monitor/bridge/generic_multi_bridge_spec.yaml").value
        spec_path = resolve_spec_path(spec_file)
        self.spec = load_spec(spec_path)
        self._publishers = {}
        self._subscriptions = []

        for bridge_spec in self.spec["bridges"]:
            msg_type = import_message_type(bridge_spec["message_type"])
            publisher = self.create_publisher(
                AlgorithmFeedback, bridge_spec["output_topic"], 50)
            callback = self._make_callback(bridge_spec, publisher)
            subscription = self.create_subscription(
                msg_type, bridge_spec["input_topic"], callback, 10)
            self._publishers[bridge_spec["id"]] = publisher
            self._subscriptions.append(subscription)
            self.get_logger().info(
                "bridge[%s] started: input=%s type=%s output=%s"
                % (
                    bridge_spec["id"],
                    bridge_spec["input_topic"],
                    bridge_spec["message_type"],
                    bridge_spec["output_topic"],
                )
            )

    def _stamp_or_now(self, msg):
        header = getattr(msg, "header", None)
        if header is not None:
            stamp = getattr(header, "stamp", None)
            if stamp is not None and ((stamp.sec != 0) or (stamp.nanosec != 0)):
                return stamp
        return self.get_clock().now().to_msg()

    def _make_callback(self, bridge_spec: dict, publisher):
        def _callback(msg):
            stamp = self._stamp_or_now(msg)
            for feedback in build_feedback_messages(msg, bridge_spec, stamp):
                publisher.publish(feedback)

        return _callback


def main(args=None):
    rclpy.init(args=args)
    node = ConfigDrivenBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
