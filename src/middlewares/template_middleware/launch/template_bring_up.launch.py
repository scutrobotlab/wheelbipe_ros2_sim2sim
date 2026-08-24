#!/usr/bin/env python3
# Copyright (c) 2025 SCUTRobotLab
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Bring up one controller stack with either the MuJoCo or real serial backend."""

import os
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _parse_bool(value, name):
    normalized = str(value).strip().lower()
    if normalized in ("1", "true", "yes", "on"):
        return True
    if normalized in ("0", "false", "no", "off"):
        return False
    raise RuntimeError(
        f"{name} must be one of true/false, yes/no, on/off, or 1/0; got {value!r}"
    )


def _default_controller_parameter_definition():
    share_directory = get_package_share_directory("template_ros2_controller")
    workspace_directory = os.path.abspath(
        os.path.join(share_directory, "..", "..", "..", "..")
    )
    source_path = os.path.join(
        workspace_directory,
        "src",
        "controllers",
        "template_ros2_controller",
        "config",
        "template_ros2_controller_parameters.yaml",
    )
    if os.path.isfile(source_path):
        return source_path
    return os.path.join(
        share_directory, "config", "template_ros2_controller_parameters.yaml"
    )


def _normalize_default_value(specification):
    value = specification.get("default_value")
    parameter_type = specification.get("type")
    if parameter_type == "double":
        return float(value)
    if parameter_type == "int":
        return int(value)
    if parameter_type == "bool":
        return value if isinstance(value, bool) else _parse_bool(value, "boolean default")
    if parameter_type == "string":
        return str(value)
    if parameter_type == "double_array":
        return [float(item) for item in value]
    if parameter_type == "int_array":
        return [int(item) for item in value]
    if parameter_type == "bool_array":
        return [
            item if isinstance(item, bool) else _parse_bool(item, "boolean array default")
            for item in value
        ]
    if parameter_type == "string_array":
        return [str(item) for item in value]
    return value


def _resolve_policy_path(value):
    environment_path = os.environ.get("WHEELBIPE_RL_MODEL_PATH", "")
    if environment_path:
        return environment_path
    if os.path.isabs(value) and os.path.isfile(value):
        return value

    controller_share = get_package_share_directory("template_ros2_controller")
    workspace_directory = os.path.abspath(
        os.path.join(controller_share, "..", "..", "..", "..")
    )
    source_marker = "src/controllers/template_ros2_controller/"
    relative_path = value.split(source_marker, 1)[-1] if source_marker in value else value
    for candidate in (
        os.path.join(
            workspace_directory,
            "src",
            "controllers",
            "template_ros2_controller",
            relative_path,
        ),
        os.path.join(controller_share, relative_path),
    ):
        if os.path.isfile(candidate):
            return candidate
    raise RuntimeError(f"Unable to resolve baseline policy path: {value}")


def _make_runtime_parameter_file(definition_path, auto_enter_rl, use_dt7):
    if not os.path.isfile(definition_path):
        raise RuntimeError(f"Controller parameter definition is missing: {definition_path}")

    with open(definition_path, "r", encoding="utf-8") as definition_file:
        definitions = yaml.safe_load(definition_file) or {}
    specifications = definitions.get("template_ros2_controller", {})
    runtime_parameters = {}
    for name, specification in specifications.items():
        if not isinstance(specification, dict) or "default_value" not in specification:
            continue
        value = _normalize_default_value(specification)
        if name == "rl_model_path":
            value = _resolve_policy_path(value)
        runtime_parameters[name] = value
    runtime_parameters["auto_enter_rl"] = auto_enter_rl
    runtime_parameters["use_dt7"] = use_dt7

    generated = {
        "/**": {
            "template_ros2_controller": {
                "ros__parameters": runtime_parameters,
            }
        }
    }
    output = tempfile.NamedTemporaryFile(
        mode="w", suffix="_wheelbipe_controller.yaml", delete=False
    )
    with output:
        yaml.safe_dump(generated, output, sort_keys=False)
    return output.name


def launch_setup(context, *args, **kwargs):
    del args, kwargs

    prefix = LaunchConfiguration("prefix").perform(context)
    backend = LaunchConfiguration("backend").perform(context).strip().lower()
    if backend not in ("sim", "real"):
        raise RuntimeError(f"backend must be 'sim' or 'real'; got {backend!r}")
    auto_enter_rl = _parse_bool(
        LaunchConfiguration("auto_enter_rl").perform(context), "auto_enter_rl"
    )
    xbox_enabled = _parse_bool(
        LaunchConfiguration("xbox").perform(context), "xbox"
    )
    use_dt7 = _parse_bool(
        LaunchConfiguration("use_dt7").perform(context), "use_dt7"
    )
    if backend != "real" and use_dt7:
        raise RuntimeError("use_dt7=true requires backend=real")
    definition_path = LaunchConfiguration("controller_params").perform(context)
    if not definition_path:
        definition_path = _default_controller_parameter_definition()
    runtime_parameter_file = _make_runtime_parameter_file(
        definition_path,
        auto_enter_rl,
        use_dt7,
    )

    controller_config = os.path.join(
        get_package_share_directory("template_middleware"),
        "config",
        "wheelbipe_V14.yaml",
    )
    controller_manager = f"/{prefix}/controller_manager"
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            controller_manager,
        ],
    )
    baseline_controller = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "template_ros2_controller",
            "--controller-manager",
            controller_manager,
            "--param-file",
            controller_config,
            "--param-file",
            runtime_parameter_file,
        ],
    )

    launch_items = []
    if backend == "sim":
        launch_items.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("mujoco_ros2_control"),
                        "launch",
                        "mujoco_bridge.launch.py",
                    )
                ),
                launch_arguments={
                    "prefix": prefix,
                    "controller_config": controller_config,
                    "render": LaunchConfiguration("render"),
                    "run_duration": LaunchConfiguration("run_duration"),
                }.items(),
            )
        )
    else:
        robot_xacro = os.path.join(
            get_package_share_directory("robot_descriptions"),
            "wheelbipe_V14",
            "xacro",
            "robot_real.xacro",
        )
        xacro_arguments = [
            "xacro",
            robot_xacro,
            f"serial_port:={LaunchConfiguration('real_serial_port').perform(context)}",
            f"baudrate:={LaunchConfiguration('real_baudrate').perform(context)}",
            "serial_reconnect_interval_ms:="
            + LaunchConfiguration("real_serial_reconnect_interval_ms").perform(context),
            f"state_timeout_ms:={LaunchConfiguration('real_state_timeout_ms').perform(context)}",
        ]
        try:
            robot_description = subprocess.run(
                xacro_arguments,
                check=True,
                capture_output=True,
                text=True,
            ).stdout
        except (OSError, subprocess.CalledProcessError) as error:
            details = getattr(error, "stderr", "") or str(error)
            raise RuntimeError(f"Unable to generate real robot description: {details}") from error

        control_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            namespace=prefix,
            output="screen",
            parameters=[
                {"robot_description": robot_description, "use_sim_time": False},
                controller_config,
                runtime_parameter_file,
            ],
        )
        state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=prefix,
            output="screen",
            parameters=[
                {
                    "robot_description": robot_description,
                    "frame_prefix": f"{prefix}/",
                    "use_sim_time": False,
                }
            ],
        )
        launch_items.extend(
            [
                control_node,
                state_publisher,
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=control_node,
                        on_exit=[
                            EmitEvent(
                                event=Shutdown(reason="real ros2_control node exited")
                            )
                        ],
                    )
                ),
            ]
        )

    launch_items.extend([joint_state_broadcaster, baseline_controller])
    if xbox_enabled:
        xbox_config = LaunchConfiguration("xbox_config").perform(context)
        if not xbox_config:
            xbox_config = os.path.join(
                get_package_share_directory("xbox_teleop"),
                "config",
                "xbox_teleop_params.yaml",
            )
        launch_items.append(
            Node(
                package="xbox_teleop",
                executable="xbox_teleop_node",
                name="xbox_teleop_node",
                output="screen",
                emulate_tty=True,
                parameters=[xbox_config, {"prefix": prefix}],
            )
        )

    def remove_runtime_parameter_file(shutdown_context):
        del shutdown_context
        try:
            os.unlink(runtime_parameter_file)
        except FileNotFoundError:
            pass
        return []

    launch_items.append(
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[OpaqueFunction(function=remove_runtime_parameter_file)]
            )
        )
    )
    return launch_items


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "backend",
                default_value="sim",
                description="Hardware backend: sim (MuJoCo) or real (single serial).",
            ),
            DeclareLaunchArgument(
                "prefix",
                default_value="wheelbipe_V14",
                description="ROS namespace used by all robot nodes and topics.",
            ),
            DeclareLaunchArgument(
                "controller_params",
                default_value="",
                description=(
                    "Controller parameter-definition YAML; empty uses the package default."
                ),
            ),
            DeclareLaunchArgument(
                "auto_enter_rl",
                default_value="false",
                description="Enter the RL state automatically after startup.",
            ),
            DeclareLaunchArgument(
                "xbox",
                default_value="false",
                description="Start the Xbox evdev teleoperation node.",
            ),
            DeclareLaunchArgument(
                "use_dt7",
                default_value="false",
                description="Use the normal four-field DT7 input exposed by real_bridge.",
            ),
            DeclareLaunchArgument(
                "xbox_config",
                default_value="",
                description="Xbox parameter YAML; empty uses the package default.",
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
            DeclareLaunchArgument(
                "real_serial_port",
                default_value="/dev/wheelbipe_h7",
                description="Single H7 serial device used by the real backend.",
            ),
            DeclareLaunchArgument(
                "real_baudrate",
                default_value="2000000",
                description="H7 serial baud rate.",
            ),
            DeclareLaunchArgument(
                "real_serial_reconnect_interval_ms",
                default_value="1000",
                description="Delay between single-port reconnect attempts.",
            ),
            DeclareLaunchArgument(
                "real_state_timeout_ms",
                default_value="100",
                description="Age at which H7 state is stale and command output is inhibited.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
