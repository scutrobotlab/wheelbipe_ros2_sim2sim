#!/usr/bin/env bash
set -euo pipefail

serial_port="${WHEELBIPE_REAL_SERIAL_PORT:-/dev/wheelbipe_h7}"

usage() {
  echo "Usage: ./scripts/check_real.sh [--serial-port DEVICE]"
}

while (($#)); do
  case "$1" in
    --serial-port)
      if (($# < 2)); then
        echo "--serial-port requires a device path" >&2
        exit 2
      fi
      serial_port="$2"
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

if [[ ! -e "${serial_port}" ]]; then
  echo "Real serial device does not exist: ${serial_port}" >&2
  echo "Install config/99-wheelbipe-serial.rules or pass --serial-port /dev/ttyUSBx." >&2
  exit 1
fi
if [[ ! -c "${serial_port}" ]]; then
  echo "Real serial path is not a character device: ${serial_port}" >&2
  exit 1
fi
if [[ ! -r "${serial_port}" || ! -w "${serial_port}" ]]; then
  echo "Real serial device is not readable and writable: ${serial_port}" >&2
  echo "Add the current user to dialout, then sign out and back in." >&2
  exit 1
fi

echo "Real serial device is ready: ${serial_port}"
echo "Protocol: one serial port, 158-byte command, 143-byte state, normal DT7 only."
