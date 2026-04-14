# Copyright 2026 tokou
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = (
        "/opt/ry/config/Monitor/safety_emergency_executor/"
        "safety_emergency_executor_params.yaml"
    )

    return LaunchDescription([
        Node(
            package="safety_emergency_executor",
            executable="safety_emergency_executor_node",
            name="safety_emergency_executor",
            parameters=[config],
            output="screen"
        )
    ])
