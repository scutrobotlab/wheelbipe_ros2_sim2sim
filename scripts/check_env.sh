#!/usr/bin/env bash
set -euo pipefail

repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
failed=0

check_command() {
  if command -v "$1" >/dev/null 2>&1; then
    printf '[OK] %s\n' "$1"
  else
    printf '[MISSING] %s\n' "$1" >&2
    failed=1
  fi
}

if [[ "$(uname -m)" == "x86_64" ]]; then
  echo "[OK] architecture: x86_64"
else
  echo "[UNSUPPORTED] architecture: $(uname -m); expected x86_64" >&2
  failed=1
fi

if [[ -f /opt/ros/humble/setup.bash ]]; then
  echo "[OK] ROS 2 Humble"
else
  echo "[MISSING] /opt/ros/humble/setup.bash" >&2
  failed=1
fi

for command_name in cmake colcon g++ pkg-config python3 rosdep sha256sum timeout; do
  check_command "${command_name}"
done

if command -v pkg-config >/dev/null 2>&1 && pkg-config --exists glfw3; then
  echo "[OK] GLFW development package"
else
  echo "[MISSING] GLFW development package (Ubuntu package: libglfw3-dev)" >&2
  failed=1
fi

for required_path in \
  "${repository_root}/.deps/mujoco-3.5.0/include/mujoco/mujoco.h" \
  "${repository_root}/.deps/mujoco-3.5.0/lib/libmujoco.so" \
  "${repository_root}/.deps/onnxruntime-linux-x64-1.20.0/include/onnxruntime_cxx_api.h" \
  "${repository_root}/.deps/onnxruntime-linux-x64-1.20.0/lib/libonnxruntime.so"; do
  if [[ -e "${required_path}" ]]; then
    printf '[OK] %s\n' "${required_path#"${repository_root}/"}"
  else
    printf '[MISSING] %s\n' "${required_path#"${repository_root}/"}" >&2
    failed=1
  fi
done

if [[ -f "${repository_root}/.deps/mujoco-3.5.0/include/mujoco/mujoco.h" ]] &&
  grep -Eq '^#define mjVERSION_HEADER 3005000$' \
    "${repository_root}/.deps/mujoco-3.5.0/include/mujoco/mujoco.h"; then
  echo "[OK] MuJoCo header version: 3.5.0"
else
  echo "[INVALID] MuJoCo headers do not identify version 3.5.0" >&2
  failed=1
fi

if [[ -f "${repository_root}/.deps/onnxruntime-linux-x64-1.20.0/include/onnxruntime_c_api.h" ]] &&
  grep -Eq '^#define ORT_API_VERSION 20$' \
    "${repository_root}/.deps/onnxruntime-linux-x64-1.20.0/include/onnxruntime_c_api.h"; then
  echo "[OK] ONNX Runtime API version: 20 (release 1.20.x)"
else
  echo "[INVALID] ONNX Runtime headers do not expose API version 20" >&2
  failed=1
fi

if ((failed)); then
  echo "Environment check failed. See DEPLOY.md for dependency installation." >&2
  exit 1
fi

echo "Environment check passed."
