#!/usr/bin/env bash
set -euo pipefail

found=0
readable=0
shopt -s nullglob

for event_path in /dev/input/event*; do
  event_name="$(basename "${event_path}")"
  name_path="/sys/class/input/${event_name}/device/name"
  [[ -r "${name_path}" ]] || continue
  device_name="$(<"${name_path}")"
  if [[ "${device_name,,}" =~ xbox|x-box|microsoft.*controller ]]; then
    found=1
    if [[ -r "${event_path}" ]]; then
      status="readable"
      readable=1
    else
      status="permission denied"
    fi
    printf '%s: %s (%s)\n' "${event_path}" "${device_name}" "${status}"
  fi
done

if ((found == 0)); then
  echo "No Xbox-compatible evdev device was detected." >&2
  echo "Connect the controller, then check /dev/input and DEPLOY.md." >&2
  exit 1
fi

if ((readable == 0)); then
  echo "A controller was found, but the current user cannot read it." >&2
  echo "Install config/99-wheelbipe-gamepad.rules as described in DEPLOY.md." >&2
  exit 1
fi

echo "Xbox evdev check passed."
