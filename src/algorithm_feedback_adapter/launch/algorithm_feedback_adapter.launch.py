from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = (
        "/opt/ry/config/Monitor/algorithm_feedback_adapter/"
        "algorithm_feedback_adapter_params.yaml"
    )

    return LaunchDescription([
        Node(
            package="algorithm_feedback_adapter",
            executable="algorithm_feedback_adapter_node",
            name="algorithm_feedback_adapter_node",
            parameters=[config],
            output="screen"
        )
    ])
