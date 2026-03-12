# Copyright 2026 tokou
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('safety_emergency_executor'),
        'config',
        'safety_emergency_executor_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='safety_emergency_executor',
            executable='safety_emergency_executor_node',
            name='safety_emergency_executor',
            parameters=[config],
            output='screen'
        )
    ])
