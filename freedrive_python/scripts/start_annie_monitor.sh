#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROBOT_SN="${ROBOT_SN:-Rizon4s-123456}"
ROBOT_INTERFACE_IP="${ROBOT_INTERFACE_IP:-127.0.0.1}"
RECORDING_OUTPUT="${RECORDING_OUTPUT:-${HOME}/Annie}"
PYTHON_BIN="${PYTHON_BIN:-$PROJECT_ROOT/.venv/bin/python}"

if [[ ! -x "$PYTHON_BIN" ]]; then
  echo "Error: project Python was not found: $PYTHON_BIN" >&2
  echo "Create .venv and install requirements.txt first." >&2
  exit 2
fi

OPEN_BROWSER=(--open-browser)
if [[ "${NO_BROWSER:-0}" == "1" ]]; then
  OPEN_BROWSER=()
fi

cat <<EOF
Annie monitor configuration
  Robot: $ROBOT_SN
  PC LAN1 address: $ROBOT_INTERFACE_IP
  Sampling: end-of-arm 6-axis F/T, 50 Hz, read-only
  Preferred UI: http://127.0.0.1:8765
  Recording output: $RECORDING_OUTPUT

This program does not start, stop, or modify Annie on the tablet.
Start recording in the UI, then run Annie from Flexiv Elements on the LAN2 tablet.
If a compatible monitor is already running, this launcher reuses it.
EOF

exec "$PYTHON_BIN" "$PROJECT_ROOT/scripts/run_annie_monitor.py" \
  --robot-sn "$ROBOT_SN" \
  --network-interface-ip "$ROBOT_INTERFACE_IP" \
  --recording-output "$RECORDING_OUTPUT" \
  "${OPEN_BROWSER[@]}" \
  "$@"
