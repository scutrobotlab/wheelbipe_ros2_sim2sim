#!/usr/bin/env bash
set -euo pipefail

repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
render=true
duration=0.0
xbox=false
backend=sim
real_serial_port="${WHEELBIPE_REAL_SERIAL_PORT:-/dev/wheelbipe_h7}"
use_dt7=false
disable_dt7=false

usage() {
  echo "Usage: ./scripts/demo.sh [--headless] [--xbox] [--duration SECONDS]"
  echo "       ./scripts/demo.sh --real [--serial-port DEVICE] [--no-dt7 | --xbox]"
}

while (($#)); do
  case "$1" in
    --headless)
      render=false
      shift
      ;;
    --xbox)
      xbox=true
      shift
      ;;
    --real)
      backend=real
      render=false
      shift
      ;;
    --no-dt7)
      disable_dt7=true
      shift
      ;;
    --serial-port)
      if (($# < 2)); then
        echo "--serial-port requires a device path" >&2
        exit 2
      fi
      real_serial_port="$2"
      shift 2
      ;;
    --duration)
      if (($# < 2)); then
        echo "--duration requires seconds" >&2
        exit 2
      fi
      duration="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if ! python3 - "${duration}" <<'PY'
import math
import sys

try:
    value = float(sys.argv[1])
except ValueError:
    raise SystemExit(1)
raise SystemExit(0 if math.isfinite(value) and value >= 0.0 else 1)
PY
then
  echo "--duration must be a finite number greater than or equal to 0" >&2
  exit 2
fi

[[ -f "${repository_root}/install/setup.bash" ]] || {
  echo "Workspace is not built. Run ./scripts/build.sh first." >&2
  exit 1
}

set +u
# shellcheck disable=SC1091
source "${repository_root}/setup_ros_domain.bash"
set -u

export WHEELBIPE_RL_MODEL_PATH="${repository_root}/src/controllers/template_ros2_controller/policy/parallel/V14-35-flat-and-rotation-13k.onnx"

auto_enter_rl=true
if [[ "${backend}" == real && "${disable_dt7}" == false ]]; then
  use_dt7=true
fi
if [[ "${xbox}" == true || "${backend}" == real ]]; then
  auto_enter_rl=false
fi
if [[ "${xbox}" == true ]]; then
  use_dt7=false
fi

if [[ "${backend}" == real ]]; then
  if [[ "${duration}" != 0 && "${duration}" != 0.0 ]]; then
    echo "--duration is only available for the MuJoCo backend" >&2
    exit 2
  fi
  "${repository_root}/scripts/check_real.sh" --serial-port "${real_serial_port}"
  echo "Starting REAL hardware backend in IDLE-safe mode."
  echo "Keep the robot supported, make the emergency stop reachable, then command PREPARE/RL explicitly."
fi

ros2 launch template_middleware template_bring_up.launch.py \
  backend:="${backend}" \
  prefix:=wheelbipe_V14 \
  render:="${render}" \
  run_duration:="${duration}" \
  auto_enter_rl:="${auto_enter_rl}" \
  xbox:="${xbox}" \
  use_dt7:="${use_dt7}" \
  real_serial_port:="${real_serial_port}"
