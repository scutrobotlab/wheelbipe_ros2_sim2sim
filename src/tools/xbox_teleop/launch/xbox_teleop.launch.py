#!/usr/bin/env python3
# Copyright (c) 2025 SCUTRobotLab
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    package_share_directory = get_package_share_directory("xbox_teleop")
    config_file = LaunchConfiguration("config_file").perform(context)
    if not config_file:
        config_file = os.path.join(
            package_share_directory,
            "config",
            "xbox_teleop_params.yaml",
        )

    return [
        Node(
            package="xbox_teleop",
            executable="xbox_teleop_node",
            name="xbox_teleop_node",
            output="screen",
            parameters=[config_file],
            emulate_tty=True,
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value="",
                description=(
                    "Path to xbox_teleop YAML config. Empty uses the package "
                    "default."
                ),
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
