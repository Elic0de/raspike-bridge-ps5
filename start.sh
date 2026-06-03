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
LEFT_PORT="${RASPIKE_LEFT_PORT:-B}"
RIGHT_PORT="${RASPIKE_RIGHT_PORT:-A}"
ARM_PORT="${RASPIKE_ARM_PORT:-C}"
FORCE_PORT="${RASPIKE_FORCE_PORT:-D}"
COLOR_PORT="${RASPIKE_COLOR_PORT:-E}"
ULTRASONIC_PORT="${RASPIKE_ULTRASONIC_PORT:-F}"
INIT_DELAY_SEC="${RASPIKE_INIT_DELAY_SEC:-0.2}"
INIT_RETRIES="${RASPIKE_INIT_RETRIES:-2}"
INIT_ORDER="${RASPIKE_INIT_ORDER:-arm-first}"
CONTROL_API_ENABLED="${RASPIKE_CONTROL_API_ENABLED:-true}"
CONTROL_API_RESTART="${RASPIKE_CONTROL_API_RESTART:-true}"
CONTROL_API_DIR="${RASPIKE_CONTROL_API_DIR:-$SCRIPT_DIR/../etrobo2026/raspi}"
CONTROL_API_STARTED=0

PS5_ARGS=(
    --event-device /dev/input/event4
    --left-port "$LEFT_PORT"
    --right-port "$RIGHT_PORT"
    --force-port "$FORCE_PORT"
    --init-delay-sec "$INIT_DELAY_SEC"
    --init-retries "$INIT_RETRIES"
    --init-order "$INIT_ORDER"
    --telemetry-host "$TELEMETRY_HOST"
    --telemetry-port "$TELEMETRY_PORT"
    --web-control-host "$WEB_CONTROL_HOST"
    --web-control-port "$WEB_CONTROL_PORT"
    -v
)

case "${ARM_PORT,,}" in
    ""|"none"|"off"|"disable"|"disabled")
        PS5_ARGS+=(--no-arm)
        ;;
    *)
        PS5_ARGS+=(--arm-port "$ARM_PORT")
        ;;
esac

case "${COLOR_PORT,,}" in
    ""|"none"|"off"|"disable"|"disabled")
        PS5_ARGS+=(--no-color-sensor)
        ;;
    *)
        PS5_ARGS+=(--color-port "$COLOR_PORT")
        ;;
esac

case "${ULTRASONIC_PORT,,}" in
    ""|"none"|"off"|"disable"|"disabled")
        PS5_ARGS+=(--no-ultrasonic-sensor)
        ;;
    *)
        PS5_ARGS+=(--ultrasonic-port "$ULTRASONIC_PORT")
        ;;
esac

cleanup() {
    echo ""
    echo "Stopping..."
    if [ -n "${BRIDGE_PID:-}" ]; then
        kill "$BRIDGE_PID" 2>/dev/null || true
        wait "$BRIDGE_PID" 2>/dev/null || true
    fi
    if [ "$CONTROL_API_STARTED" = "1" ]; then
        make -C "$CONTROL_API_DIR" api-stop 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

camera_stream_host() {
    hostname -I 2>/dev/null | tr ' ' '\n' | grep -E '^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$' | head -n 1
}

start_control_api() {
    case "${CONTROL_API_ENABLED,,}" in
        ""|"0"|"false"|"no"|"off"|"disable"|"disabled")
            return
            ;;
    esac

    if [ ! -f "$CONTROL_API_DIR/Makefile" ]; then
        echo "[warn] Control API dir not found: $CONTROL_API_DIR" >&2
        return
    fi

    if [ "${CONTROL_API_RESTART,,}" = "true" ]; then
        make -C "$CONTROL_API_DIR" api-stop 2>/dev/null || true
        CONTROL_API_STARTED=1
    else
        if ! make -C "$CONTROL_API_DIR" api-status 2>/dev/null | grep -q "raspi api running"; then
            CONTROL_API_STARTED=1
        fi
    fi

    ETROBO_CAMERA_STREAM_ENABLED=true make -C "$CONTROL_API_DIR" api
    stream_host="$(camera_stream_host)"
    stream_host="${stream_host:-raspi.local}"
    echo "Control API camera stream: http://$stream_host:8080/stream.mjpg"
}

start_control_api

python3 raspike_bridge.py \
    --serial /dev/raspike-real \
    --pty-link /dev/USB_SPIKE \
    --pty-priority-ms 0 \
    --restart-spike-on-signal \
    --restart-spike-on-broken-pipe \
    -v &
BRIDGE_PID=$!

echo "Bridge started (PID $BRIDGE_PID), waiting for socket/status..."
sleep 3

python3 ps5_raspike_control.py "${PS5_ARGS[@]}"
