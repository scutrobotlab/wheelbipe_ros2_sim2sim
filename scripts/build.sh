#!/usr/bin/env bash
set -euo pipefail

repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
clean_cache=false

usage() {
  echo "Usage: ./scripts/build.sh [--clean-cache]"
}

while (($#)); do
  case "$1" in
    --clean-cache)
      clean_cache=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

"${repository_root}/scripts/check_env.sh"

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${repository_root}/setup_mujoco_env.bash"
set -u

cmake_args=(
  --no-warn-unused-cli
  -DCMAKE_BUILD_TYPE=Release
  -DBUILD_TESTING=OFF
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
  -DONNXRUNTIME_ROOT="${ONNXRUNTIME_ROOT}"
)

colcon_args=(build --cmake-args "${cmake_args[@]}")
if [[ "${clean_cache}" == true ]]; then
  colcon_args=(build --cmake-clean-cache --cmake-args "${cmake_args[@]}")
fi

cd "${repository_root}"
colcon "${colcon_args[@]}"
echo "Build completed. Source install/setup.bash before direct ros2 commands."
