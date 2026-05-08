from __future__ import annotations

import importlib
import json
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


def normalize_bridge_entry(entry: dict) -> dict:
    normalized = dict(entry)
    normalized["id"] = str(entry["id"]).strip()
    normalized["message_type"] = str(entry["message_type"]).strip()
    normalized["input_topic"] = str(entry["input_topic"]).strip()
    normalized["output_topic"] = str(entry["output_topic"]).strip()
    normalized["module_name"] = str(entry["module_name"]).strip()
    normalized["topic_name"] = str(entry["topic_name"]).strip()

    metrics = []
    for metric in entry["metrics"]:
        metric_copy = dict(metric)
        metric_copy["name"] = str(metric["name"]).strip()
        metric_copy["field"] = str(metric["field"]).strip()
        if "valid_field" in metric_copy:
            metric_copy["valid_field"] = str(metric_copy["valid_field"]).strip()
        if "valid_default" in metric_copy:
            metric_copy["valid_default"] = bool(metric_copy["valid_default"])
        metrics.append(metric_copy)
    normalized["metrics"] = metrics
    return normalized


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
    return normalize_bridge_entry(entry)


def load_spec(spec_path: str | Path) -> dict:
    path = Path(spec_path)
    data = yaml.safe_load(path.read_text()) or {}
    missing = REQUIRED_SPEC_KEYS.difference(data.keys())
    if missing:
        raise ValueError(f"Spec file '{path}' missing keys: {sorted(missing)}")
    if not isinstance(data["bridges"], list) or not data["bridges"]:
        raise ValueError(f"Spec file '{path}' must define a non-empty bridges list")
    data["bridges"] = [validate_bridge_entry(entry) for entry in data["bridges"]]
    seen_ids = set()
    duplicate_ids = set()
    for entry in data["bridges"]:
        bridge_id = entry["id"]
        if bridge_id in seen_ids:
            duplicate_ids.add(bridge_id)
        seen_ids.add(bridge_id)
    if duplicate_ids:
        raise ValueError(f"Spec file '{path}' has duplicate bridge ids: {sorted(duplicate_ids)}")
    return data


def compute_spec_signature(spec: dict) -> str:
    return json.dumps(spec, sort_keys=True, ensure_ascii=False)


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
    def __init__(self, parameter_overrides=None):
        super().__init__(
            "algorithm_feedback_adapter_node",
            parameter_overrides=parameter_overrides or [],
        )

        default_spec = (
            "/opt/ry/config/Monitor/algorithm_feedback_adapter/"
            "algorithm_feedback_adapter_spec.yaml"
        )
        spec_file = self.declare_parameter("spec_file", default_spec).value
        self._spec_path = resolve_spec_path(spec_file)
        self._bridge_publishers = {}
        self._bridge_subscriptions = {}
        self._bridge_specs = {}
        self._active_spec_signature = None
        self._last_seen_mtime_ns = None

        initial_spec = load_spec(self._spec_path)
        self._apply_spec(initial_spec)
        self._active_spec_signature = compute_spec_signature(initial_spec)
        self._last_seen_mtime_ns = self._get_spec_mtime_ns()

        self._spec_reload_enabled = bool(
            self.declare_parameter("spec_reload_enabled", True).value)
        self._spec_reload_period_s = max(
            0.1, float(self.declare_parameter("spec_reload_period_s", 1.0).value))
        self._reload_timer = None
        if self._spec_reload_enabled:
            self._reload_timer = self.create_timer(
                self._spec_reload_period_s,
                self._on_reload_timer,
            )

    def bridge_ids(self) -> list[str]:
        return sorted(self._bridge_specs.keys())

    def _get_spec_mtime_ns(self):
        try:
            return self._spec_path.stat().st_mtime_ns
        except FileNotFoundError:
            return None

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

    def _destroy_bridge(self, bridge_id: str):
        subscription = self._bridge_subscriptions.pop(bridge_id, None)
        if subscription is not None:
            self.destroy_subscription(subscription)
        publisher = self._bridge_publishers.pop(bridge_id, None)
        if publisher is not None:
            self.destroy_publisher(publisher)
        self._bridge_specs.pop(bridge_id, None)

    def _create_or_replace_bridge(self, bridge_spec: dict):
        bridge_id = bridge_spec["id"]
        msg_type = import_message_type(bridge_spec["message_type"])
        publisher = self.create_publisher(
            AlgorithmFeedback, bridge_spec["output_topic"], 50)
        callback = self._make_callback(bridge_spec, publisher)
        subscription = self.create_subscription(
            msg_type, bridge_spec["input_topic"], callback, 10)

        self._destroy_bridge(bridge_id)
        self._bridge_publishers[bridge_id] = publisher
        self._bridge_subscriptions[bridge_id] = subscription
        self._bridge_specs[bridge_id] = bridge_spec
        self.get_logger().info(
            "algorithm_feedback_adapter[%s] active: input=%s type=%s output=%s"
            % (
                bridge_spec["id"],
                bridge_spec["input_topic"],
                bridge_spec["message_type"],
                bridge_spec["output_topic"],
            )
        )

    def _apply_spec(self, spec: dict):
        next_specs = {entry["id"]: entry for entry in spec["bridges"]}
        current_ids = set(self._bridge_specs.keys())
        next_ids = set(next_specs.keys())

        removed_ids = current_ids - next_ids
        added_ids = next_ids - current_ids
        common_ids = current_ids & next_ids
        changed_ids = {
            bridge_id for bridge_id in common_ids
            if self._bridge_specs[bridge_id] != next_specs[bridge_id]
        }

        for bridge_id in sorted(removed_ids):
            self._destroy_bridge(bridge_id)
            self.get_logger().info(
                "algorithm_feedback_adapter[%s] removed by spec reload" % bridge_id
            )

        for bridge_id in sorted(changed_ids):
            self._create_or_replace_bridge(next_specs[bridge_id])
            self.get_logger().info("algorithm_feedback_adapter[%s] reloaded" % bridge_id)

        for bridge_id in sorted(added_ids):
            self._create_or_replace_bridge(next_specs[bridge_id])
            self.get_logger().info("algorithm_feedback_adapter[%s] added" % bridge_id)

    def reload_spec_if_needed(self, force=False) -> bool:
        current_mtime_ns = self._get_spec_mtime_ns()
        if not force:
            if not self._spec_reload_enabled:
                return False
            if current_mtime_ns == self._last_seen_mtime_ns:
                return False

        try:
            spec = load_spec(self._spec_path)
            signature = compute_spec_signature(spec)
        except Exception as exc:
            self._last_seen_mtime_ns = current_mtime_ns
            self.get_logger().error(
                "Failed to reload algorithm_feedback_adapter spec '%s': %s" %
                (self._spec_path, exc)
            )
            return False

        if not force and signature == self._active_spec_signature:
            self._last_seen_mtime_ns = current_mtime_ns
            return False

        self._apply_spec(spec)
        self._active_spec_signature = signature
        self._last_seen_mtime_ns = current_mtime_ns
        return True

    def _on_reload_timer(self):
        self.reload_spec_if_needed(force=False)


def main(args=None):
    rclpy.init(args=args)
    node = ConfigDrivenBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
