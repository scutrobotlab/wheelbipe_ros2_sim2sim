#!/usr/bin/env bash

# Detect project root relative to this script
_WHEELBIPE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -z "${MUJOCO_DIR:-}" ]; then
  export MUJOCO_DIR="${_WHEELBIPE_ROOT}/.deps/mujoco-3.5.0"
fi

if [ -z "${ONNXRUNTIME_ROOT:-}" ]; then
  export ONNXRUNTIME_ROOT="${_WHEELBIPE_ROOT}/.deps/onnxruntime-linux-x64-1.20.0"
fi

if [ -n "${MUJOCO_DIR:-}" ]; then
  if [ -d "${MUJOCO_DIR}/lib" ]; then
    export CMAKE_PREFIX_PATH="${MUJOCO_DIR}/lib${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
    export LD_LIBRARY_PATH="${MUJOCO_DIR}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
  fi
fi

if [ -n "${ONNXRUNTIME_ROOT:-}" ]; then
  export LD_LIBRARY_PATH="${ONNXRUNTIME_ROOT}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi

unset _WHEELBIPE_ROOT
