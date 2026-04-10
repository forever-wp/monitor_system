from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[3]
MONITOR_ROOT = REPO_ROOT / "config" / "Monitor"


def test_runtime_configs_package_removed():
    assert not (REPO_ROOT / "src" / "runtime_configs").exists()


def test_monitor_bundle_moves_to_repo_root_config_directory():
    assert (MONITOR_ROOT / "nav2_monitor" / "nav2_monitor_params.yaml").exists()
    assert (MONITOR_ROOT / "bridge" / "bridge_py_params.yaml").exists()
    assert (
        MONITOR_ROOT
        / "safety_emergency_executor"
        / "safety_emergency_executor_params.yaml"
    ).exists()


def test_bridge_workspace_latest_battery_feedback_files_present():
    assert (
        REPO_ROOT / "src" / "bridge" / "include" / "bridge" / "battery_feedback_bridge.hpp"
    ).exists()
    assert (
        REPO_ROOT / "src" / "bridge" / "src" / "battery_feedback_bridge.cpp"
    ).exists()
    assert (
        REPO_ROOT / "src" / "bridge" / "test" / "test_battery_feedback_bridge.cpp"
    ).exists()


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

    assert runtime_ros_params == source_ros_params


def test_monitor_bundle_uses_ota_absolute_paths():
    bridge_params = yaml.safe_load((MONITOR_ROOT / "bridge" / "bridge_py_params.yaml").read_text())
    nav2_params = yaml.safe_load(
        (MONITOR_ROOT / "nav2_monitor" / "nav2_monitor_params.yaml").read_text()
    )

    assert (
        bridge_params["bridge_py_node"]["ros__parameters"]["spec_file"]
        == "/opt/ry/config/Monitor/bridge/generic_multi_bridge_spec.yaml"
    )

    ros_params = nav2_params["nav2_monitor"]["ros__parameters"]
    assert (
        ros_params["fault_config"]
        == "/opt/ry/config/Monitor/nav2_monitor/fault_detector_config.yaml"
    )


def test_collision_zone_direction_fields_exist_in_monitor_bundle():
    fault_config = yaml.safe_load(
        (MONITOR_ROOT / "nav2_monitor" / "fault_detector_config.yaml").read_text()
    )
    collision_cfg = fault_config["collision_detection"]
    zones = {zone["name"]: zone for zone in collision_cfg["zones"]}

    assert collision_cfg["direction_speed_threshold"] == 0.05
    assert zones["front_slow"]["motion_direction"] == "forward"
    assert zones["front_stop"]["motion_direction"] == "forward"


def test_ttc_visualization_switch_exists_in_monitor_bundle():
    fault_config = yaml.safe_load(
        (MONITOR_ROOT / "nav2_monitor" / "fault_detector_config.yaml").read_text()
    )
    collision_cfg = fault_config["collision_detection"]

    assert collision_cfg["ttc_visualization_enabled"] == 0


def test_dynamic_ttc_defaults_exist_in_monitor_bundle():
    fault_config = yaml.safe_load(
        (MONITOR_ROOT / "nav2_monitor" / "fault_detector_config.yaml").read_text()
    )
    collision_cfg = fault_config["collision_detection"]
    zones = {zone["name"]: zone for zone in collision_cfg["zones"]}

    assert "front_ttc" in zones
    front_ttc = zones["front_ttc"]
    assert front_ttc["model"] == "ttc"
    assert "ttc_horizon_s" in front_ttc
    assert "corridor_margin" in front_ttc
    assert "candidate_downsample_resolution" in front_ttc
    assert "points" not in front_ttc
    assert "polygon_pub_topic" not in front_ttc


def test_drift_status_bridge_is_registered_for_light_lm():
    spec = yaml.safe_load((MONITOR_ROOT / "bridge" / "generic_multi_bridge_spec.yaml").read_text())
    bridges = {entry["id"]: entry for entry in spec["bridges"]}

    assert "drift_status" in bridges
    drift_bridge = bridges["drift_status"]
    assert drift_bridge["message_type"] == "geometry_msgs/msg/PoseStamped"
    assert drift_bridge["input_topic"] == "/drift_status"
    assert drift_bridge["module_name"] == "light-lm"
    assert drift_bridge["topic_name"] == "/drift_status"
    assert [metric["name"] for metric in drift_bridge["metrics"]] == [
        "drift_state",
        "drift_delta_norm",
        "drift_reserved",
    ]


def test_light_lm_has_independent_drift_feedback_rules():
    fault_config = yaml.safe_load(
        (MONITOR_ROOT / "nav2_monitor" / "fault_detector_config.yaml").read_text()
    )
    modules = fault_config["modules"]
    light_lm = next(module for module in modules if module["name"] == "light-lm")
    feedback_rules = light_lm["feedback_rules"]
    rules_by_metric = {rule["metric_name"]: rule for rule in feedback_rules}

    assert "drift_state" in rules_by_metric
    assert "drift_delta_norm" in rules_by_metric
    assert rules_by_metric["drift_state"]["source_topic"] == "/drift_status"
    assert rules_by_metric["drift_delta_norm"]["source_topic"] == "/drift_status"


def test_launch_files_and_nav2_source_use_ota_only():
    nav2_launch = (
        REPO_ROOT / "src" / "nav2_monitor" / "launch" / "nav2_monitor.launch.py"
    ).read_text()
    safety_launch = (
        REPO_ROOT
        / "src"
        / "safety_emergency_executor"
        / "launch"
        / "safety_emergency_executor.launch.py"
    ).read_text()
    nav2_source = (
        REPO_ROOT / "src" / "nav2_monitor" / "src" / "nav2_monitor_node.cpp"
    ).read_text()
    nav2_cmake = (REPO_ROOT / "src" / "nav2_monitor" / "CMakeLists.txt").read_text()
    nav2_package = (REPO_ROOT / "src" / "nav2_monitor" / "package.xml").read_text()

    nav2_launch_compact = "".join(nav2_launch.split())
    safety_launch_compact = "".join(safety_launch.split())

    assert (
        'config="/opt/ry/config/Monitor/nav2_monitor/nav2_monitor_params.yaml"'
        in nav2_launch_compact
    )
    assert (
        'config=("/opt/ry/config/Monitor/safety_emergency_executor/"'
        '"safety_emergency_executor_params.yaml")'
        in safety_launch_compact
    )
    assert "ament_index_cpp" not in nav2_source
    assert "runtime_configs" not in nav2_source
    assert "find_package(ament_index_cpp REQUIRED)" not in nav2_cmake
    assert " ament_index_cpp " not in nav2_cmake
    assert "<depend>ament_index_cpp</depend>" not in nav2_package
    assert "<exec_depend>runtime_configs</exec_depend>" not in nav2_package
    assert "/nav2_monitor/collision_ttc_markers" in nav2_source
