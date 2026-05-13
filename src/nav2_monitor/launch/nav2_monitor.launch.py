from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    monitor_config = "/opt/ry/config/Monitor/nav2_monitor/nav2_monitor_params.yaml"
    topic_frequency_config = "/opt/ry/config/Monitor/nav2_monitor/topic_frequency_monitor_params.yaml"
    vehicle_state_config = "/opt/ry/config/Monitor/nav2_monitor/vehicle_state_judge_params.yaml"
    node_tf_config = "/opt/ry/config/Monitor/nav2_monitor/node_tf_monitor_params.yaml"
    battery_config = "/opt/ry/config/Monitor/nav2_monitor/battery_monitor_params.yaml"
    algorithm_feedback_config = "/opt/ry/config/Monitor/nav2_monitor/algorithm_feedback_monitor_params.yaml"
    collision_config = "/opt/ry/config/Monitor/nav2_monitor/collision_monitor_params.yaml"

    return LaunchDescription([
        Node(
            package="nav2_monitor",
            executable="topic_frequency_monitor_node",
            name="topic_frequency_monitor",
            parameters=[topic_frequency_config],
            output="screen"
        ),
        Node(
            package="nav2_monitor",
            executable="vehicle_state_judge_node",
            name="vehicle_state_judge",
            parameters=[vehicle_state_config],
            output="screen"
        ),
        Node(
            package="nav2_monitor",
            executable="node_tf_monitor_node",
            name="node_tf_monitor",
            parameters=[node_tf_config],
            output="screen"
        ),
        Node(
            package="nav2_monitor",
            executable="battery_monitor_node",
            name="battery_monitor",
            parameters=[battery_config],
            output="screen"
        ),
        Node(
            package="nav2_monitor",
            executable="algorithm_feedback_monitor_node",
            name="algorithm_feedback_monitor",
            parameters=[algorithm_feedback_config],
            output="screen"
        ),
        Node(
            package="nav2_monitor",
            executable="collision_monitor_node",
            name="collision_monitor",
            parameters=[collision_config],
            output="screen"
        ),
        Node(
            package="nav2_monitor",
            executable="nav2_monitor_aggregator_node",
            name="nav2_monitor_aggregator",
            parameters=[monitor_config],
            output="screen"
        )
    ])
