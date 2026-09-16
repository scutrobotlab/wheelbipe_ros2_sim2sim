#!/usr/bin/env bash
set -euo pipefail
repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export WHEELBIPE_SIM_PROFILE=source_v14
export WHEELBIPE_TERRAIN=rmuc2026
export WHEELBIPE_RL_MODEL_PATH="${WHEELBIPE_RL_MODEL_PATH:-${repository_root}/src/controllers/template_ros2_controller/policy/parallel/UniLab-V14-35-rough-ros2-motion-stop200.onnx}"
exec "${repository_root}/scripts/demo.sh" "$@"
