from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = (
        "/opt/ry/config/Monitor/collision_voxel_layer/collision_voxel_layer_params.yaml"
    )

    return LaunchDescription([
        Node(
            package="collision_voxel_layer",
            executable="collision_voxel_layer_node",
            name="collision_voxel_layer",
            parameters=[config],
            output="screen"
        )
    ])
