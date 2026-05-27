#!/usr/bin/env python3
"""
Drive RasPike motors from a PS5 controller through raspike_bridge.py.

Controls:
  X       : emergency stop, brake motors and latch emergency state
  Circle  : motor stop / coast
  Triangle: gyro heading reset
  Square  : log mark / save
  L stick : steering
  R2/L2   : accelerator / brake-reverse
  R1/L1   : increase/decrease PWM power limit
  D-pad   : experiment mode selection
  OPTIONS : selected experiment start
  SHARE   : cancel / return idle
"""

from __future__ import annotations

import argparse
import json
import select
import socket
import struct
import sys
import time
from dataclasses import dataclass
from pathlib import Path

try:
    from evdev import InputDevice, categorize, ecodes, list_devices
except ModuleNotFoundError:
    InputDevice = None
    categorize = None
    ecodes = None
    list_devices = None


RP_CMD_START = 0xEA
RP_PORT_NONE = 255

RP_CMD_TYPE_MOTOR = 0x3
RP_CMD_TYPE_HUB = 0x5

RP_CMD_ID_MOT_CFG = (RP_CMD_TYPE_MOTOR << 5) | 0x0
RP_CMD_ID_MOT_POW = (RP_CMD_TYPE_MOTOR << 5) | 0x4
RP_CMD_ID_MOT_STP = (RP_CMD_TYPE_MOTOR << 5) | 0x5
RP_CMD_ID_MOT_STP_BRK = (RP_CMD_TYPE_MOTOR << 5) | 0x6
RP_CMD_ID_HUB_IMU_RST_HDG = (RP_CMD_TYPE_HUB << 5) | 0x15

EXPERIMENT_MODES = ("idle", "manual", "calibration", "trial", "replay")


@dataclass
class AxisState:
    value: int = 0
    minimum: int = -32768
    maximum: int = 32767

    def normalized(self, deadzone: float) -> float:
        span = max(abs(self.minimum), abs(self.maximum), 1)
        value = max(-1.0, min(1.0, self.value / span))
        return 0.0 if abs(value) < deadzone else value

    def trigger(self) -> float:
        span = max(self.maximum - self.minimum, 1)
        return max(0.0, min(1.0, (self.value - self.minimum) / span))


def clamp(value: int, low: int, high: int) -> int:
    return max(low, min(high, value))


def clamp_float(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def curve(value: float, exponent: float) -> float:
    if value == 0:
        return 0.0
    return (1.0 if value > 0 else -1.0) * (abs(value) ** exponent)


def slew(current: float, target: float, max_delta: float) -> float:
    delta = target - current
    if abs(delta) <= max_delta:
        return target
    return current + (max_delta if delta > 0 else -max_delta)


def make_packet(port: int, cmd: int, payload: bytes = b"") -> bytes:
    if len(payload) > 255:
        raise ValueError("payload too large")
    return bytes([RP_CMD_START, cmd, len(payload), port]) + payload


def motor_config_packet(port: int) -> bytes:
    return make_packet(port, RP_CMD_ID_MOT_CFG)


def motor_power_packet(port: int, power: int) -> bytes:
    return make_packet(port, RP_CMD_ID_MOT_POW, struct.pack("<i", power))


def motor_stop_packet(port: int) -> bytes:
    return make_packet(port, RP_CMD_ID_MOT_STP)


def motor_brake_packet(port: int) -> bytes:
    return make_packet(port, RP_CMD_ID_MOT_STP_BRK)


def gyro_reset_packet() -> bytes:
    return make_packet(RP_PORT_NONE, RP_CMD_ID_HUB_IMU_RST_HDG)


def port_id(name: str) -> int:
    if len(name) != 1 or name.upper() < "A" or name.upper() > "F":
        raise argparse.ArgumentTypeError("port must be A-F")
    return ord(name.upper()) - ord("A")


def find_controller(path: str | None) -> InputDevice:
    assert InputDevice is not None and list_devices is not None
    if path:
        return InputDevice(path)

    candidates = []
    for dev_path in list_devices():
        dev = InputDevice(dev_path)
        name = dev.name.lower()
        if any(token in name for token in ("dualsense", "wireless controller", "playstation", "ps5")):
            candidates.append(dev)
        else:
            dev.close()

    if not candidates:
        raise RuntimeError("no PS5/DualSense controller found; pass --event-device /dev/input/eventX")
    return candidates[0]


def axis_ranges(device: InputDevice) -> dict[int, AxisState]:
    assert ecodes is not None
    ranges: dict[int, AxisState] = {}
    for code, info in device.capabilities(absinfo=True).get(ecodes.EV_ABS, []):
        ranges[code] = AxisState(minimum=info.min, maximum=info.max)
    return ranges


def button_name(code: int) -> str:
    assert ecodes is not None
    name = ecodes.KEY.get(code, code)
    if isinstance(name, list):
        return name[0]
    return str(name)


def log_event(path: Path, kind: str, **fields: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    record = {"time": time.time(), "kind": kind, **fields}
    with path.open("a", encoding="utf-8") as fp:
        fp.write(json.dumps(record, separators=(",", ":")) + "\n")


def send_stop(sock: socket.socket, left_port: int, right_port: int) -> None:
    sock.sendall(motor_stop_packet(left_port) + motor_stop_packet(right_port))


def send_brake(sock: socket.socket, left_port: int, right_port: int) -> None:
    sock.sendall(motor_brake_packet(left_port) + motor_brake_packet(right_port))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="evdev PS5 controller client for raspike_bridge.py")
    parser.add_argument("--socket", default="/tmp/raspike.sock", help="bridge Unix socket")
    parser.add_argument("--event-device", help="PS5 event device; auto-detected if omitted")
    parser.add_argument("--left-port", type=port_id, default=port_id("B"), help="left motor port A-F")
    parser.add_argument("--right-port", type=port_id, default=port_id("A"), help="right motor port A-F")
    parser.add_argument("--max-power", type=int, default=60, help="initial absolute motor power limit")
    parser.add_argument("--min-power", type=int, default=10, help="minimum adjustable power limit")
    parser.add_argument("--power-step", type=int, default=5, help="R1/L1 power adjustment step")
    parser.add_argument("--deadzone", type=float, default=0.08, help="L stick deadzone as 0.0-1.0")
    parser.add_argument("--rate-hz", type=float, default=30.0, help="drive command send rate")
    parser.add_argument("--drive-style", choices=("car", "arcade"), default="car", help="car uses R2/L2, arcade uses L stick Y")
    parser.add_argument("--steering-curve", type=float, default=1.6, help="higher means softer steering near center")
    parser.add_argument("--throttle-curve", type=float, default=1.4, help="higher means softer throttle near zero")
    parser.add_argument("--idle-steer-scale", type=float, default=0.25, help="turning authority when throttle is zero")
    parser.add_argument("--steering-gain", type=float, default=0.85, help="maximum steering mix into left/right power")
    parser.add_argument("--slew-rate", type=float, default=180.0, help="maximum motor power change per second")
    parser.add_argument("--log-file", default="/tmp/raspike-ps5-events.jsonl", help="JSONL event log")
    parser.add_argument("--configure", action="store_true", help="send motor config packets on startup")
    parser.add_argument("-v", "--verbose", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if sys.platform != "linux":
        print("ps5_raspike_control.py requires Linux evdev.", file=sys.stderr)
        return 2
    if InputDevice is None or ecodes is None:
        print("missing dependency: pip install evdev", file=sys.stderr)
        return 2

    device = find_controller(args.event_device)
    axes = axis_ranges(device)
    log_path = Path(args.log_file)

    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.connect(args.socket)

    if args.configure:
        sock.sendall(motor_config_packet(args.left_port) + motor_config_packet(args.right_port))

    power_limit = clamp(args.max_power, args.min_power, 100)
    mode_index = 0
    emergency = False
    last_power: tuple[int, int] | None = None
    current_left = 0.0
    current_right = 0.0
    next_send_at = 0.0
    interval = 1.0 / args.rate_hz
    last_tick_at = time.monotonic()

    log_event(log_path, "controller_connected", device=device.path, name=device.name)
    print(f"controller: {device.name} ({device.path})")
    print(f"mode: {EXPERIMENT_MODES[mode_index]}, power_limit={power_limit}")

    try:
        while True:
            now = time.monotonic()
            timeout = max(0.0, next_send_at - now)
            readable, _, _ = select.select([device.fd], [], [], min(timeout, interval))

            if device.fd in readable:
                for event in device.read():
                    if event.type == ecodes.EV_ABS:
                        if event.code in axes:
                            axes[event.code].value = event.value
                        if event.code == ecodes.ABS_HAT0X and event.value:
                            mode_index = clamp(mode_index + event.value, 0, len(EXPERIMENT_MODES) - 1)
                            mode = EXPERIMENT_MODES[mode_index]
                            log_event(log_path, "mode_select", mode=mode)
                            print(f"mode: {mode}")
                        elif event.code == ecodes.ABS_HAT0Y and event.value:
                            mode_index = clamp(mode_index + event.value, 0, len(EXPERIMENT_MODES) - 1)
                            mode = EXPERIMENT_MODES[mode_index]
                            log_event(log_path, "mode_select", mode=mode)
                            print(f"mode: {mode}")

                    elif event.type == ecodes.EV_KEY and event.value == 1:
                        if event.code == ecodes.BTN_SOUTH:
                            emergency = True
                            current_left = 0.0
                            current_right = 0.0
                            send_brake(sock, args.left_port, args.right_port)
                            last_power = (0, 0)
                            log_event(log_path, "emergency_stop")
                            print("emergency stop")
                        elif event.code == ecodes.BTN_EAST:
                            current_left = 0.0
                            current_right = 0.0
                            send_stop(sock, args.left_port, args.right_port)
                            last_power = (0, 0)
                            log_event(log_path, "motor_stop_coast")
                            print("motor stop / coast")
                        elif event.code == ecodes.BTN_NORTH:
                            sock.sendall(gyro_reset_packet())
                            log_event(log_path, "gyro_reset")
                            print("gyro reset")
                        elif event.code == ecodes.BTN_WEST:
                            log_event(log_path, "mark", mode=EXPERIMENT_MODES[mode_index], power_limit=power_limit)
                            print("log mark")
                        elif event.code == ecodes.BTN_TR:
                            power_limit = clamp(power_limit + args.power_step, args.min_power, 100)
                            log_event(log_path, "power_limit", value=power_limit)
                            print(f"power_limit={power_limit}")
                        elif event.code == ecodes.BTN_TL:
                            power_limit = clamp(power_limit - args.power_step, args.min_power, 100)
                            log_event(log_path, "power_limit", value=power_limit)
                            print(f"power_limit={power_limit}")
                        elif event.code == ecodes.BTN_START:
                            mode = EXPERIMENT_MODES[mode_index]
                            emergency = False
                            log_event(log_path, "experiment_start", mode=mode)
                            print(f"experiment start: {mode}")
                        elif event.code == ecodes.BTN_SELECT:
                            emergency = False
                            mode_index = 0
                            current_left = 0.0
                            current_right = 0.0
                            send_stop(sock, args.left_port, args.right_port)
                            last_power = (0, 0)
                            log_event(log_path, "cancel_idle")
                            print("cancel / idle")
                        elif args.verbose:
                            print(f"button: {button_name(event.code)}")

            now = time.monotonic()
            if emergency or now < next_send_at:
                continue

            steering = curve(axes.get(ecodes.ABS_X, AxisState()).normalized(args.deadzone), args.steering_curve)
            if args.drive_style == "car":
                accelerator = axes.get(ecodes.ABS_RZ, AxisState(minimum=0, maximum=255)).trigger()
                brake = axes.get(ecodes.ABS_Z, AxisState(minimum=0, maximum=255)).trigger()
                throttle = curve(accelerator - brake, args.throttle_curve)
            else:
                y = axes.get(ecodes.ABS_Y, AxisState()).normalized(args.deadzone)
                throttle = curve(-y, args.throttle_curve)

            steer_scale = args.steering_gain * max(abs(throttle), args.idle_steer_scale)
            target_left = clamp_float((throttle + steering * steer_scale) * power_limit, -power_limit, power_limit)
            target_right = clamp_float((throttle - steering * steer_scale) * power_limit, -power_limit, power_limit)

            dt = max(0.0, now - last_tick_at)
            last_tick_at = now
            max_delta = args.slew_rate * dt
            current_left = slew(current_left, target_left, max_delta)
            current_right = slew(current_right, target_right, max_delta)

            left = clamp(round(current_left), -power_limit, power_limit)
            right = clamp(round(current_right), -power_limit, power_limit)
            power = (left, right)

            if power != last_power:
                sock.sendall(motor_power_packet(args.left_port, left) + motor_power_packet(args.right_port, right))
                if args.verbose:
                    print(
                        f"left={left:4d} right={right:4d} "
                        f"thr={throttle:+.2f} steer={steering:+.2f} limit={power_limit:3d}"
                    )
                last_power = power
            next_send_at = now + interval

    finally:
        device.close()
        sock.close()


if __name__ == "__main__":
    raise SystemExit(main())
