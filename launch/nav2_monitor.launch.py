from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('nav2_monitor'),
        'config',
        'nav2_monitor_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='nav2_monitor',
            executable='nav2_monitor_node',
            name='nav2_monitor',
            parameters=[config],
            output='screen'
        )
    ])
