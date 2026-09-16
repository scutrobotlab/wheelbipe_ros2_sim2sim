#!/usr/bin/env bash

# Load either a native ROS 2 Humble installation or an active RoboStack Humble
# environment, then expose the local MuJoCo and ONNX Runtime libraries.
_wheelbipe_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_wheelbipe_ros_setup=""

if [ -n "${CONDA_PREFIX:-}" ] &&
  [ -x "${CONDA_PREFIX}/bin/ros2" ] &&
  [ -f "${CONDA_PREFIX}/setup.bash" ]; then
  _wheelbipe_ros_setup="${CONDA_PREFIX}/setup.bash"
elif [ -f /opt/ros/humble/setup.bash ]; then
  _wheelbipe_ros_setup=/opt/ros/humble/setup.bash
fi

if [ -n "${_wheelbipe_ros_setup}" ]; then
  # shellcheck disable=SC1090
  source "${_wheelbipe_ros_setup}"
fi

if [ "${ROS_DISTRO:-}" != "humble" ] || ! command -v ros2 >/dev/null 2>&1; then
  echo "ROS 2 Humble is not active." >&2
  echo "Install native Humble or run: conda activate wheelbipe_humble" >&2
  unset _wheelbipe_root _wheelbipe_ros_setup
  return 1
fi

if [ -f "${_wheelbipe_root}/setup_mujoco_env.bash" ]; then
  # shellcheck disable=SC1091
  source "${_wheelbipe_root}/setup_mujoco_env.bash"
fi

unset _wheelbipe_root _wheelbipe_ros_setup
