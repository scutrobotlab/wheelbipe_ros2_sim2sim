#!/usr/bin/env bash

# Source this file from the workspace root before build or launch.
_wheelbipe_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_wheelbipe_ros_domain_id="${ROS_DOMAIN_ID:-0}"
_wheelbipe_ros_localhost_only="${ROS_LOCALHOST_ONLY:-1}"
_wheelbipe_rmw_implementation="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi

if [ -f "${_wheelbipe_ws}/setup_mujoco_env.bash" ]; then
  # shellcheck disable=SC1091
  source "${_wheelbipe_ws}/setup_mujoco_env.bash"
fi

if [ -f "${_wheelbipe_ws}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${_wheelbipe_ws}/install/setup.bash"
fi

export WHEELBIPE_WS="${_wheelbipe_ws}"
export ROS_DOMAIN_ID="${_wheelbipe_ros_domain_id}"
# Keep the default demo on one host. Multi-host deployments must opt in after
# choosing an isolated DDS domain and network trust boundary.
export ROS_LOCALHOST_ONLY="${_wheelbipe_ros_localhost_only}"
export RMW_IMPLEMENTATION="${_wheelbipe_rmw_implementation}"

unset _wheelbipe_ws _wheelbipe_ros_domain_id _wheelbipe_ros_localhost_only
unset _wheelbipe_rmw_implementation
