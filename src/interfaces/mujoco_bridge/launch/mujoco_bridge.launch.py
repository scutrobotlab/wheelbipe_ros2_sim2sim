#!/usr/bin/env python3
# Copyright (c) 2025 SCUTRobotLab
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Start the WheelBipe MuJoCo ros2_control hardware adapter."""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def _parse_bool(value, name):
    normalized = str(value).strip().lower()
    if normalized in ("1", "true", "yes", "on"):
        return True
    if normalized in ("0", "false", "no", "off"):
        return False
    raise RuntimeError(
        f"{name} must be one of true/false, yes/no, on/off, or 1/0; got {value!r}"
    )


def _parse_duration(value):
    try:
        duration = float(value)
    except ValueError as error:
        raise RuntimeError(f"run_duration must be a number; got {value!r}") from error
    if not math.isfinite(duration) or duration < 0.0:
        raise RuntimeError(
            f"run_duration must be finite and greater than or equal to 0; got {value!r}"
        )
    return duration


def launch_setup(context, *args, **kwargs):
    del args, kwargs

    prefix = LaunchConfiguration("prefix").perform(context)
    render = _parse_bool(LaunchConfiguration("render").perform(context), "render")
    run_duration = _parse_duration(LaunchConfiguration("run_duration").perform(context))

    descriptions_share = get_package_share_directory("robot_descriptions")
    robot_xacro = os.path.join(
        descriptions_share, "wheelbipe_V14", "xacro", "robot_sim.xacro"
    )
    mujoco_model = os.path.join(
        descriptions_share, "wheelbipeV14_2", "mjcf", "scene.xml"
    )
    controller_config = LaunchConfiguration("controller_config").perform(context)

    for required_path in (robot_xacro, mujoco_model, controller_config):
        if not os.path.isfile(required_path):
            raise RuntimeError(f"Required bringup file is missing: {required_path}")

    robot_description = {
        "robot_description": Command([FindExecutable(name="xacro"), " ", robot_xacro])
    }
    simulation = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        namespace=prefix,
        output="screen",
        parameters=[
            robot_description,
            controller_config,
            {
                "mujoco_model_path": mujoco_model,
                "use_sim_time": True,
                "publish_imu": True,
                "imu_topic": "imu/data",
                "imu_frame_id": "imu",
                "render": render,
                "headless_real_time": True,
                "run_duration": run_duration,
            },
        ],
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=prefix,
        output="both",
        parameters=[
            robot_description,
            {"frame_prefix": f"{prefix}/", "use_sim_time": True},
        ],
    )
    shutdown_on_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=simulation,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason="MuJoCo exited; stopping the remaining launch processes."
                    )
                )
            ],
        )
    )

    return [simulation, robot_state_publisher, shutdown_on_exit]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "prefix",
                default_value="wheelbipe_V14",
                description="ROS namespace used by the robot and controller manager.",
            ),
            DeclareLaunchArgument(
                "controller_config",
                description="Absolute path to the ros2_control controller-manager YAML.",
            ),
            DeclareLaunchArgument(
                "render",
                default_value="true",
                description="Create the interactive MuJoCo viewer.",
            ),
            DeclareLaunchArgument(
                "run_duration",
                default_value="0.0",
                description="Simulation seconds before automatic shutdown; 0 runs indefinitely.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
