#!/usr/bin/env bash
set -euo pipefail

repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
deps_dir="${repository_root}/.deps"
cache_dir="${repository_root}/.cache/downloads"
mujoco_source=""
onnxruntime_source=""
offline_cache=""
install_ros_dependencies=true

usage() {
  cat <<'EOF'
Usage: ./scripts/bootstrap.sh [options]

Options:
  --mujoco-source DIR       Import an extracted MuJoCo 3.5.0 directory.
  --onnxruntime-source DIR  Import an extracted ONNX Runtime 1.20.0 directory.
  --offline-cache DIR       Read the two official archives from DIR without downloading.
  --skip-rosdep             Do not install missing ROS/system packages with rosdep.
  --help                    Show this help.

With no source options, missing dependencies are downloaded from their official
GitHub releases. Existing valid .deps directories are reused.
EOF
}

while (($#)); do
  case "$1" in
    --mujoco-source)
      mujoco_source="${2:?--mujoco-source requires a directory}"
      shift 2
      ;;
    --onnxruntime-source)
      onnxruntime_source="${2:?--onnxruntime-source requires a directory}"
      shift 2
      ;;
    --offline-cache)
      offline_cache="${2:?--offline-cache requires a directory}"
      shift 2
      ;;
    --skip-rosdep)
      install_ros_dependencies=false
      shift
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

mkdir -p "${deps_dir}"
if [[ -n "${offline_cache}" ]]; then
  [[ -d "${offline_cache}" ]] || {
    echo "Offline cache directory does not exist: ${offline_cache}" >&2
    exit 1
  }
  cache_dir="$(cd "${offline_cache}" && pwd)"
else
  mkdir -p "${cache_dir}"
fi

copy_dependency() {
  local source_dir="$1"
  local destination_dir="$2"
  local required_file="$3"
  local version_file="$4"
  local version_pattern="$5"
  local temporary_dir

  [[ -f "${source_dir}/${required_file}" ]] || {
    echo "Invalid dependency source; missing ${source_dir}/${required_file}" >&2
    exit 1
  }
  [[ -f "${source_dir}/${version_file}" ]] || {
    echo "Invalid dependency source; missing ${source_dir}/${version_file}" >&2
    exit 1
  }
  grep -Eq "${version_pattern}" "${source_dir}/${version_file}" || {
    echo "Dependency source does not match the required version: ${source_dir}" >&2
    exit 1
  }
  if [[ -e "${destination_dir}" ]]; then
    echo "Dependency destination already exists: ${destination_dir}" >&2
    echo "Remove or rename it before importing a different copy." >&2
    exit 1
  fi
  temporary_dir="$(mktemp -d "${deps_dir}/.import.XXXXXX")"
  cp -a "${source_dir}/." "${temporary_dir}/"
  mv "${temporary_dir}" "${destination_dir}"
}

download_dependency() {
  local name="$1"
  local url="$2"
  local expected_sha256="$3"
  local archive="$4"
  local destination_dir="$5"
  local extracted_name="$6"
  local temporary_dir

  command -v tar >/dev/null 2>&1 || {
    echo "tar is required to extract dependency archives." >&2
    exit 1
  }
  if [[ ! -f "${archive}" ]]; then
    if [[ -n "${offline_cache}" ]]; then
      echo "Offline dependency archive is missing: ${archive}" >&2
      exit 1
    fi
    command -v curl >/dev/null 2>&1 || {
      echo "curl is required for online bootstrap." >&2
      exit 1
    }
    echo "[bootstrap] downloading ${name}"
    curl --fail --location --retry 3 --output "${archive}.part" "${url}"
    mv "${archive}.part" "${archive}"
  fi
  echo "${expected_sha256}  ${archive}" | sha256sum --check --status || {
    echo "SHA-256 mismatch: ${archive}" >&2
    exit 1
  }
  temporary_dir="$(mktemp -d "${deps_dir}/.extract.XXXXXX")"
  tar -xzf "${archive}" -C "${temporary_dir}"
  [[ -d "${temporary_dir}/${extracted_name}" ]] || {
    echo "Unexpected archive layout for ${name}" >&2
    exit 1
  }
  mv "${temporary_dir}/${extracted_name}" "${destination_dir}"
  rmdir "${temporary_dir}"
}

mujoco_dir="${deps_dir}/mujoco-3.5.0"
if [[ ! -f "${mujoco_dir}/lib/libmujoco.so" ]]; then
  if [[ -n "${mujoco_source}" ]]; then
    copy_dependency \
      "${mujoco_source}" \
      "${mujoco_dir}" \
      "lib/libmujoco.so" \
      "include/mujoco/mujoco.h" \
      '^#define mjVERSION_HEADER 3005000$'
  else
    download_dependency \
      "MuJoCo 3.5.0" \
      "https://github.com/google-deepmind/mujoco/releases/download/3.5.0/mujoco-3.5.0-linux-x86_64.tar.gz" \
      "df110ae7b9b0d2e991e0594da4136e9077a625e237f84e7c338410d5199193c7" \
      "${cache_dir}/mujoco-3.5.0-linux-x86_64.tar.gz" \
      "${mujoco_dir}" \
      "mujoco-3.5.0"
  fi
fi

onnxruntime_dir="${deps_dir}/onnxruntime-linux-x64-1.20.0"
if [[ ! -f "${onnxruntime_dir}/lib/libonnxruntime.so" ]]; then
  if [[ -n "${onnxruntime_source}" ]]; then
    copy_dependency \
      "${onnxruntime_source}" \
      "${onnxruntime_dir}" \
      "lib/libonnxruntime.so" \
      "include/onnxruntime_c_api.h" \
      '^#define ORT_API_VERSION 20$'
  else
    download_dependency \
      "ONNX Runtime 1.20.0 CPU" \
      "https://github.com/microsoft/onnxruntime/releases/download/v1.20.0/onnxruntime-linux-x64-1.20.0.tgz" \
      "aa70d48b22e264b82e83f63245b51ddc9a47ae4a3a66903efaff1ba68b7b5930" \
      "${cache_dir}/onnxruntime-linux-x64-1.20.0.tgz" \
      "${onnxruntime_dir}" \
      "onnxruntime-linux-x64-1.20.0"
  fi
fi

if [[ "${install_ros_dependencies}" == true ]]; then
  [[ -f /opt/ros/humble/setup.bash ]] || {
    echo "ROS 2 Humble is required before bootstrap; see DEPLOY.md." >&2
    exit 1
  }
  command -v rosdep >/dev/null 2>&1 || {
    echo "rosdep is required; install python3-rosdep and initialize it first." >&2
    exit 1
  }
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
  if ! rosdep check --from-paths "${repository_root}/src" --ignore-src \
    --rosdistro humble >/dev/null 2>&1; then
    echo "[bootstrap] installing missing ROS and system dependencies with rosdep"
    rosdep install --from-paths "${repository_root}/src" --ignore-src \
      --rosdistro humble --default-yes
  fi
fi

"${repository_root}/scripts/check_env.sh"
echo "Dependencies are ready under ${deps_dir}."
