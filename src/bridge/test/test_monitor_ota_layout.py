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

    for document in (runtime_params, source_params):
        ros_params = document["safety_emergency_executor"]["ros__parameters"]

        assert ros_params["cmd_vel_navigation_topic"] == "/cmd_vel"
        assert ros_params["cmd_vel_miniapp_topic"] == "/cmd_vel_miniapp"
        assert ros_params["cmd_vel_remote_topic"] == "/cmd_vel_remote"
        assert ros_params["cmd_vel_other_topic"] == "/cmd_vel_other"
        assert ros_params["control_source_state_topic"] == "/control_source_state"
        assert ros_params["active_control_source"] == "navigation"
        assert ros_params["control_source_auto_preempt_enabled"] is False

        assert "cmd_vel_topic" not in ros_params
        assert "manual_override_service" not in ros_params
        assert "manual_override_query_service" not in ros_params
        assert "manual_override_state_topic" not in ros_params
