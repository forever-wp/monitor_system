from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[3]
MONITOR_ROOT = REPO_ROOT / "config" / "Monitor"


def test_safety_executor_control_source_defaults_are_consistent():
    runtime_params = yaml.safe_load(
        (
            MONITOR_ROOT
            / "safety_emergency_executor"
            / "safety_emergency_executor_params.yaml"
        ).read_text()
    )
    source_params = yaml.safe_load(
        (
            REPO_ROOT
            / "src"
            / "safety_emergency_executor"
            / "config"
            / "safety_emergency_executor_params.yaml"
        ).read_text()
    )

    runtime_ros_params = runtime_params["safety_emergency_executor"]["ros__parameters"]
    source_ros_params = source_params["safety_emergency_executor"]["ros__parameters"]

    for ros_params in (runtime_ros_params, source_ros_params):
        assert ros_params["cmd_vel_navigation_topic"] == "/cmd_vel"
        assert ros_params["cmd_vel_miniapp_topic"] == "/cmd_vel_miniapp"
        assert ros_params["cmd_vel_remote_topic"] == "/cmd_vel_remote"
        assert ros_params["cmd_vel_other_topic"] == "/cmd_vel_other"
        assert ros_params["control_source_state_topic"] == "/control_source_state"
        assert ros_params["active_control_source"] == "navigation"
        assert ros_params["control_source_auto_preempt_enabled"] is False
        assert ros_params["cmd_vel_navigation_extended_fields_enabled"] is False
        assert ros_params["cmd_vel_miniapp_extended_fields_enabled"] is True
        assert ros_params["cmd_vel_remote_extended_fields_enabled"] is True
        assert ros_params["cmd_vel_other_extended_fields_enabled"] is True

    assert runtime_ros_params["cmd_vel_topic"] == "/cmd_vel"
    assert runtime_ros_params["manual_override_service"] == "/set_manual_override"
    assert runtime_ros_params["manual_override_query_service"] == "/get_manual_override"
    assert runtime_ros_params["manual_override_state_topic"] == "/manual_override_active"

    assert "cmd_vel_topic" not in source_ros_params
    assert "manual_override_service" not in source_ros_params
    assert "manual_override_query_service" not in source_ros_params
    assert "manual_override_state_topic" not in source_ros_params
