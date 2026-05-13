from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = "/opt/ry/config/Monitor/nav2_monitor/vehicle_state_judge_params.yaml"

    return LaunchDescription([
        Node(
            package="nav2_monitor",
            executable="vehicle_state_judge_node",
            name="vehicle_state_judge",
            parameters=[config],
            output="screen"
        )
    ])
