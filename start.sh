#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

VENV_DIR="$SCRIPT_DIR/.venv"
if [ ! -d "$VENV_DIR" ]; then
    echo "Creating venv at $VENV_DIR..."
    python3 -m venv "$VENV_DIR"
    "$VENV_DIR/bin/pip" install -r requirements.txt
fi
source "$VENV_DIR/bin/activate"

TELEMETRY_HOST="${RASPIKE_TELEMETRY_HOST:-127.0.0.1}"
TELEMETRY_PORT="${RASPIKE_TELEMETRY_PORT:-8765}"
WEB_CONTROL_HOST="${RASPIKE_WEB_CONTROL_HOST:-0.0.0.0}"
WEB_CONTROL_PORT="${RASPIKE_WEB_CONTROL_PORT:-8766}"

cleanup() {
    echo ""
    echo "Stopping..."
    kill "$BRIDGE_PID" 2>/dev/null
    wait "$BRIDGE_PID" 2>/dev/null
}
trap cleanup EXIT INT TERM

python3 raspike_bridge.py \
    --serial /dev/ttyACM0 \
    --pty-link /home/sangi/raspike-tty \
    --pty-priority-ms 0 \
    -v &
BRIDGE_PID=$!

echo "Bridge started (PID $BRIDGE_PID), waiting for socket/status..."
sleep 3

python3 ps5_raspike_control.py \
    --event-device /dev/input/event4 \
    --telemetry-host "$TELEMETRY_HOST" \
    --telemetry-port "$TELEMETRY_PORT" \
    --web-control-host "$WEB_CONTROL_HOST" \
    --web-control-port "$WEB_CONTROL_PORT" \
    -v
