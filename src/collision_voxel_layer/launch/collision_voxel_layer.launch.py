from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = (
        get_package_share_directory("collision_voxel_layer")
        + "/config/collision_voxel_layer_params.yaml"
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
