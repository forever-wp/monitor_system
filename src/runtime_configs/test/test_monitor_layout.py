from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[3]
MONITOR_ROOT = REPO_ROOT / "src" / "runtime_configs" / "config" / "Monitor"


def test_monitor_tree_contains_all_three_packages():
    assert (MONITOR_ROOT / "nav2_monitor" / "nav2_monitor_params.yaml").exists()
    assert (MONITOR_ROOT / "bridge" / "bridge_py_params.yaml").exists()
    assert (
        MONITOR_ROOT
        / "safety_emergency_executor"
        / "safety_emergency_executor_params.yaml"
    ).exists()


def test_nav2_monitor_params_use_ota_absolute_fault_configs():
    params = yaml.safe_load(
        (MONITOR_ROOT / "nav2_monitor" / "nav2_monitor_params.yaml").read_text()
    )
    ros_params = params["nav2_monitor"]["ros__parameters"]

    assert (
        ros_params["fault_config"]
        == "/opt/ry/config/Monitor/nav2_monitor/fault_detector_config.yaml"
    )
    assert (
        ros_params["task_fault_configs.default"]
        == "/opt/ry/config/Monitor/nav2_monitor/fault_detector_config.yaml"
    )
    assert (
        ros_params["task_fault_configs.todoor"]
        == "/opt/ry/config/Monitor/nav2_monitor/profiles/fault_detector_todoor.yaml"
    )
    assert (
        ros_params["task_fault_configs.elevator"]
        == "/opt/ry/config/Monitor/nav2_monitor/profiles/fault_detector_elevator.yaml"
    )
    assert (
        ros_params["task_fault_configs.reverse"]
        == "/opt/ry/config/Monitor/nav2_monitor/profiles/fault_detector_reverse.yaml"
    )


def test_bridge_params_use_ota_absolute_spec_path():
    params = yaml.safe_load(
        (MONITOR_ROOT / "bridge" / "bridge_py_params.yaml").read_text()
    )
    ros_params = params["bridge_py_node"]["ros__parameters"]

    assert (
        ros_params["spec_file"]
        == "/opt/ry/config/Monitor/bridge/generic_multi_bridge_spec.yaml"
    )


def test_launch_files_point_to_ota_absolute_params():
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


def test_nav2_monitor_source_no_longer_depends_on_package_share_resolution():
    nav2_source = (
        REPO_ROOT / "src" / "nav2_monitor" / "src" / "nav2_monitor_node.cpp"
    ).read_text()
    nav2_cmake = (REPO_ROOT / "src" / "nav2_monitor" / "CMakeLists.txt").read_text()
    nav2_package = (REPO_ROOT / "src" / "nav2_monitor" / "package.xml").read_text()

    assert "ament_index_cpp" not in nav2_source
    assert "get_package_share_directory" not in nav2_source
    assert "find_package(ament_index_cpp REQUIRED)" not in nav2_cmake
    assert " ament_index_cpp " not in nav2_cmake
    assert "<depend>ament_index_cpp</depend>" not in nav2_package


def test_runtime_configs_installs_monitor_bundle_to_opt_ry_config():
    runtime_cmake = (REPO_ROOT / "src" / "runtime_configs" / "CMakeLists.txt").read_text()

    assert "DESTINATION /opt/ry/config" in runtime_cmake
