from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = "/opt/ry/config/Monitor/nav2_monitor/node_tf_monitor_params.yaml"

    return LaunchDescription([
        Node(
            package="nav2_monitor",
            executable="node_tf_monitor_node",
            name="node_tf_monitor",
            parameters=[config],
            output="screen"
        )
    ])
