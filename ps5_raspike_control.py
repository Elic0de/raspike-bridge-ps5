#!/usr/bin/env python3
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
    from evdev import InputDevice, ecodes, list_devices
except ModuleNotFoundError:
    InputDevice = None
    ecodes = None
    list_devices = None


RP_CMD_START = 0xEA
RP_PORT_NONE = 255

RP_CMD_ID_ALL_STATUS = 0x01
RP_CMD_ID_ACK = 0x02

RP_CMD_ID_MOT_CFG = 0x60
RP_CMD_ID_MOT_STU = 0x61
RP_CMD_ID_MOT_POW = 0x64
RP_CMD_ID_MOT_STP = 0x65
RP_CMD_ID_MOT_STP_BRK = 0x66
RP_CMD_ID_HUB_IMU_RST_HDG = 0xB5

EXPERIMENT_MODES = ("idle", "manual", "calibration", "trial", "replay")


@dataclass
class AxisState:
    value: int = 0
    minimum: int = -32768
    maximum: int = 32767

    def normalized(self, deadzone: float) -> float:
        center = (self.maximum + self.minimum) / 2.0
        half_span = max((self.maximum - self.minimum) / 2.0, 1.0)
        value = max(-1.0, min(1.0, (self.value - center) / half_span))
        return 0.0 if abs(value) < deadzone else value

    def normalized_around(self, center: int, deadzone: float) -> float:
        if self.value >= center:
            span = max(self.maximum - center, 1)
        else:
            span = max(center - self.minimum, 1)
        value = max(-1.0, min(1.0, (self.value - center) / span))
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
    return bytes([RP_CMD_START, cmd, len(payload), port]) + payload


def motor_config_packet(port: int) -> bytes:
    return make_packet(port, RP_CMD_ID_MOT_CFG)


def motor_setup_packet(port: int, direction: int = 0, reset_count: bool = True) -> bytes:
    # data[0:4] direction, data[4] reset_count
    payload = struct.pack("<i", direction) + bytes([1 if reset_count else 0])
    return make_packet(port, RP_CMD_ID_MOT_STU, payload)


def motor_power_packet(port: int, power: int) -> bytes:
    return make_packet(port, RP_CMD_ID_MOT_POW, struct.pack("<i", power))


def motor_stop_packet(port: int) -> bytes:
    return make_packet(port, RP_CMD_ID_MOT_STP)


def motor_brake_packet(port: int) -> bytes:
    return make_packet(port, RP_CMD_ID_MOT_STP_BRK)


def gyro_reset_packet() -> bytes:
    return make_packet(RP_PORT_NONE, RP_CMD_ID_HUB_IMU_RST_HDG)


def read_protocol_packet(sock: socket.socket, timeout_sec: float = 1.0, buf: bytearray | None = None):
    deadline = time.monotonic() + timeout_sec
    if buf is None:
        buf = bytearray()

    while time.monotonic() < deadline:
        try:
            chunk = sock.recv(4096)
            if chunk:
                buf.extend(chunk)
        except BlockingIOError:
            pass

        while buf and buf[0] != RP_CMD_START:
            del buf[0]

        if len(buf) >= 4:
            cmd = buf[1]
            size = buf[2]
            port = buf[3]
            total = 4 + size
            if len(buf) >= total:
                payload = bytes(buf[4:total])
                del buf[:total]
                return port, cmd, payload

        time.sleep(0.005)

    return None


def wait_ack(sock: socket.socket, port: int, expected_cmd: int, timeout_sec: float = 1.0) -> int:
    deadline = time.monotonic() + timeout_sec
    buf = bytearray()

    while time.monotonic() < deadline:
        pkt = read_protocol_packet(sock, timeout_sec=0.05, buf=buf)
        if pkt is None:
            continue

        ack_port, cmd, payload = pkt
        if cmd != RP_CMD_ID_ACK:
            continue

        if len(payload) < 8:
            continue

        ack_cmd, ack_data = struct.unpack("<ii", payload[:8])

        if ack_port == port and ack_cmd == expected_cmd:
            return ack_data

    raise TimeoutError(f"ACK timeout: port={port} cmd=0x{expected_cmd:02x}")


def send_and_wait_ack(sock: socket.socket, port: int, cmd: int, packet: bytes, label: str) -> None:
    sock.sendall(packet)
    ack = wait_ack(sock, port, cmd, timeout_sec=1.5)
    if ack != 1:
        raise RuntimeError(f"{label} failed: port={port} ack={ack}")


def configure_motors(sock: socket.socket, left_port: int, right_port: int) -> None:
    send_and_wait_ack(sock, left_port, RP_CMD_ID_MOT_CFG, motor_config_packet(left_port), "MOT_CFG left")
    send_and_wait_ack(sock, right_port, RP_CMD_ID_MOT_CFG, motor_config_packet(right_port), "MOT_CFG right")

    send_and_wait_ack(sock, left_port, RP_CMD_ID_MOT_STU, motor_setup_packet(left_port, direction=1), "MOT_STU left")
    send_and_wait_ack(sock, right_port, RP_CMD_ID_MOT_STU, motor_setup_packet(right_port, direction=0), "MOT_STU right")

def drain_socket(sock: socket.socket) -> None:
    try:
        while True:
            data = sock.recv(4096)
            if not data:
                return
    except BlockingIOError:
        return


def port_id(name: str) -> int:
    if len(name) != 1 or name.upper() < "A" or name.upper() > "F":
        raise argparse.ArgumentTypeError("port must be A-F")
    return ord(name.upper()) - ord("A")


def find_controller(path: str | None) -> InputDevice:
    assert InputDevice is not None and list_devices is not None

    if path:
        return InputDevice(path)

    for dev_path in list_devices():
        dev = InputDevice(dev_path)
        name = dev.name.lower()
        if (
            any(token in name for token in ("dualsense", "wireless controller", "playstation", "ps5"))
            and "touchpad" not in name
            and "motion" not in name
        ):
            return dev
        dev.close()

    raise RuntimeError("no PS5/DualSense controller found")


def axis_ranges(device: InputDevice) -> dict[int, AxisState]:
    assert ecodes is not None
    ranges: dict[int, AxisState] = {}

    for code, info in device.capabilities(absinfo=True).get(ecodes.EV_ABS, []):
        ranges[code] = AxisState(value=info.value, minimum=info.min, maximum=info.max)

    return ranges


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
    parser = argparse.ArgumentParser()
    parser.add_argument("--socket", default="/tmp/raspike.sock")
    parser.add_argument("--event-device")
    parser.add_argument("--left-port", type=port_id, default=port_id("B"))
    parser.add_argument("--right-port", type=port_id, default=port_id("A"))
    parser.add_argument("--max-power", type=int, default=60)
    parser.add_argument("--min-power", type=int, default=10)
    parser.add_argument("--power-step", type=int, default=5)
    parser.add_argument("--deadzone", type=float, default=0.08)
    parser.add_argument("--rate-hz", type=float, default=30.0)
    parser.add_argument("--drive-style", choices=("car", "arcade"), default="car")
    parser.add_argument("--steering-center", type=int, default=127)
    parser.add_argument("--steering-curve", type=float, default=1.6)
    parser.add_argument("--throttle-curve", type=float, default=1.4)
    parser.add_argument("--idle-steer-scale", type=float, default=0.25)
    parser.add_argument("--steering-gain", type=float, default=0.85)
    parser.add_argument("--slew-rate", type=float, default=180.0)
    parser.add_argument("--log-file", default="/tmp/raspike-ps5-events.jsonl")
    parser.add_argument("--configure", action="store_true")
    parser.add_argument("-v", "--verbose", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if sys.platform != "linux":
        print("requires Linux evdev", file=sys.stderr)
        return 2

    if InputDevice is None or ecodes is None:
        print("missing dependency: pip install evdev", file=sys.stderr)
        return 2

    device = find_controller(args.event_device)
    axes = axis_ranges(device)
    log_path = Path(args.log_file)

    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.connect(args.socket)
    sock.setblocking(False)

    if args.configure:
        configure_motors(sock, args.left_port, args.right_port)
        print("motor configured: MOT_CFG + MOT_STU + ACK ok")

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
            drain_socket(sock)

            now = time.monotonic()
            timeout = max(0.0, next_send_at - now)
            readable, _, _ = select.select([device.fd], [], [], min(timeout, interval))

            if device.fd in readable:
                for event in device.read():
                    if event.type == ecodes.EV_ABS:
                        if event.code in axes:
                            axes[event.code].value = event.value

                    elif event.type == ecodes.EV_KEY and event.value == 1:
                        if event.code == ecodes.BTN_SOUTH:
                            emergency = True
                            current_left = 0.0
                            current_right = 0.0
                            send_brake(sock, args.left_port, args.right_port)
                            last_power = (0, 0)
                            print("emergency stop")

                        elif event.code == ecodes.BTN_EAST:
                            current_left = 0.0
                            current_right = 0.0
                            send_stop(sock, args.left_port, args.right_port)
                            last_power = (0, 0)
                            print("motor stop / coast")

                        elif event.code == ecodes.BTN_NORTH:
                            sock.sendall(gyro_reset_packet())
                            print("gyro reset")

                        elif event.code == ecodes.BTN_TR:
                            power_limit = clamp(power_limit + args.power_step, args.min_power, 100)
                            print(f"power_limit={power_limit}")

                        elif event.code == ecodes.BTN_TL:
                            power_limit = clamp(power_limit - args.power_step, args.min_power, 100)
                            print(f"power_limit={power_limit}")

                        elif event.code == ecodes.BTN_START:
                            emergency = False
                            print("manual start")

                        elif event.code == ecodes.BTN_SELECT:
                            emergency = False
                            current_left = 0.0
                            current_right = 0.0
                            send_stop(sock, args.left_port, args.right_port)
                            last_power = (0, 0)
                            print("cancel / idle")

            now = time.monotonic()
            if emergency or now < next_send_at:
                continue

            axis_x = axes.get(ecodes.ABS_X, AxisState())
            steering = curve(axis_x.normalized_around(args.steering_center, args.deadzone), args.steering_curve)

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
                sock.sendall(
                    motor_power_packet(args.left_port, left)
                    + motor_power_packet(args.right_port, right)
                )

                log_event(
                    log_path,
                    "drive",
                    throttle=round(throttle, 3),
                    steering=round(steering, 3),
                    left_pwm=left,
                    right_pwm=right,
                    power_limit=power_limit,
                    raw_lx=axes.get(ecodes.ABS_X, AxisState()).value,
                    raw_ly=axes.get(ecodes.ABS_Y, AxisState()).value,
                    raw_rx=axes.get(ecodes.ABS_RX, AxisState()).value,
                    raw_ry=axes.get(ecodes.ABS_RY, AxisState()).value,
                    raw_r2=axes.get(ecodes.ABS_RZ, AxisState(minimum=0, maximum=255)).value,
                    raw_l2=axes.get(ecodes.ABS_Z, AxisState(minimum=0, maximum=255)).value,
                )

                if args.verbose:
                    print(
                        f"left={left:4d} right={right:4d} "
                        f"thr={throttle:+.2f} steer={steering:+.2f} "
                        f"raw_x={axis_x.value:4d} limit={power_limit:3d}"
                    )

                last_power = power

            next_send_at = now + interval

    finally:
        try:
            send_stop(sock, args.left_port, args.right_port)
        except Exception:
            pass
        device.close()
        sock.close()


if __name__ == "__main__":
    raise SystemExit(main())
