#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROBOT_SN="${ROBOT_SN:-Rizon4s-123456}"
ROBOT_INTERFACE_IP="${ROBOT_INTERFACE_IP:-127.0.0.1}"
RECORDING_OUTPUT="${RECORDING_OUTPUT:-${HOME}/EmJ}"
PYTHON_BIN="${PYTHON_BIN:-$PROJECT_ROOT/.venv/bin/python}"

if [[ ! -x "$PYTHON_BIN" ]]; then
  echo "Error: project Python was not found: $PYTHON_BIN" >&2
  exit 2
fi

OPEN_BROWSER=(--open-browser)
if [[ "${NO_BROWSER:-0}" == "1" ]]; then
  OPEN_BROWSER=()
fi

cat <<EOF
EmJ force monitor configuration
  Robot: $ROBOT_SN
  PC LAN1 address: $ROBOT_INTERFACE_IP
  Sampling: end-of-arm 6-axis F/T, 50 Hz, read-only
  Preferred UI: http://127.0.0.1:8775
  CSV output: $RECORDING_OUTPUT

This program does not start, stop, or modify EmJ on the tablet.
Start force capture in the UI, then run EmJ from Flexiv Elements.
EOF

exec "$PYTHON_BIN" "$PROJECT_ROOT/scripts/run_annie_monitor.py" \
  --project-label EmJ \
  --monitor-profile force-only \
  --robot-sn "$ROBOT_SN" \
  --network-interface-ip "$ROBOT_INTERFACE_IP" \
  --port 8775 \
  --recording-output "$RECORDING_OUTPUT" \
  "${OPEN_BROWSER[@]}" \
  "$@"
