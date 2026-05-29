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

cleanup() {
    echo ""
    echo "Stopping..."
    kill "$BRIDGE_PID" 2>/dev/null
    wait "$BRIDGE_PID" 2>/dev/null
}
trap cleanup EXIT INT TERM

python3 raspike_bridge.py \
    --serial /dev/USB_SPIKE \
    --pty-link "$HOME/raspike-tty" \
    --unix-socket /tmp/raspike.sock \
    --pty-priority-ms 200 \
    -v &
BRIDGE_PID=$!

echo "Bridge started (PID $BRIDGE_PID), waiting for socket..."
sleep 1

python3 ps5_raspike_control.py \
    --socket /tmp/raspike.sock \
    --left-port B \
    --right-port A \
    --max-power 60 \
    --config ./ps5_controller.yaml \
    -v
