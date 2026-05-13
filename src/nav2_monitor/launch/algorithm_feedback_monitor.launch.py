from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = "/opt/ry/config/Monitor/nav2_monitor/algorithm_feedback_monitor_params.yaml"
    return LaunchDescription([
        Node(
            package="nav2_monitor",
            executable="algorithm_feedback_monitor_node",
            name="algorithm_feedback_monitor",
            parameters=[config],
            output="screen"
        )
    ])
