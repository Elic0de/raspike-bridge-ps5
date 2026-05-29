#!/usr/bin/env python3
from __future__ import annotations

import argparse
import socket
import struct
import sys
import time

from ps5_controller.config import load_config
from ps5_controller.drive_mixer import DriveMixer
from ps5_controller.input_provider import GamepadProvider, KeyboardProvider
from ps5_controller.protocol import (
    RP_CMD_ID_ACK,
    RP_CMD_ID_MOT_CFG,
    RP_CMD_ID_MOT_STU,
    gyro_reset_packet,
    motor_config_packet,
    motor_setup_packet,
    send_brake,
    send_stop,
)
from ps5_controller.state_publisher import StatePublisher


def clamp(value: int, low: int, high: int) -> int:
    return max(low, min(high, value))


def port_id(name: str) -> int:
    if len(name) != 1 or name.upper() < "A" or name.upper() > "F":
        raise argparse.ArgumentTypeError("port must be A-F")
    return ord(name.upper()) - ord("A")


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
        while buf and buf[0] != 0xEA:
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
        if cmd != RP_CMD_ID_ACK or len(payload) < 8:
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--socket", default="/tmp/raspike.sock")
    parser.add_argument("--event-device")
    parser.add_argument("--left-port", type=port_id, default=port_id("B"))
    parser.add_argument("--right-port", type=port_id, default=port_id("A"))
    parser.add_argument("--max-power", type=int, default=60)
    parser.add_argument("--min-power", type=int, default=10)
    parser.add_argument("--power-step", type=int, default=5)
    parser.add_argument("--rate-hz", type=float, default=30.0)
    parser.add_argument("--log-file", default="/tmp/raspike-ps5-events.jsonl")
    parser.add_argument("--config", default="ps5_controller.yaml")
    parser.add_argument("--no-configure", action="store_true",
                        help="skip motor MOT_CFG/MOT_STU at startup (motors must already be configured, "
                             "otherwise the SPIKE crashes on the first motor command)")
    parser.add_argument("--keyboard", action="store_true", help="force keyboard input even when stdin is not auto-detected")
    parser.add_argument("--no-keyboard-fallback", action="store_true", help="disable automatic keyboard input")
    parser.add_argument("--wait-controller-sec", type=float, default=1.0)
    parser.add_argument("-v", "--verbose", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if sys.platform != "linux":
        print("requires Linux evdev", file=sys.stderr)
        return 2

    cfg = load_config(args.config)
    publisher = StatePublisher(args.socket, args.left_port, args.right_port, args.log_file)
    if not args.no_configure:
        configure_motors(publisher.sock, args.left_port, args.right_port)

    power_limit = clamp(args.max_power, args.min_power, 100)
    safe_mode = False
    emergency = False
    last_power: tuple[int, int] | None = None
    interval = 1.0 / args.rate_hz

    mixer = DriveMixer(
        steering_curve=cfg.input.steering_curve,
        throttle_curve=cfg.input.throttle_curve,
        steering_gain=cfg.input.steering_gain,
        low_speed_steer_gain=cfg.input.low_speed_steer_gain,
        high_speed_steer_gain=cfg.input.high_speed_steer_gain,
        slew_rate=cfg.power.slew_rate,
    )

    keyboard_enabled = args.keyboard or (not args.no_keyboard_fallback and sys.stdin.isatty())
    if keyboard_enabled:
        print("keyboard input enabled: gamepad and keyboard can be used together")

    try:
        gamepad = GamepadProvider(args.event_device, cfg)
    except Exception as exc:
        gamepad = None
        if args.verbose:
            print(f"gamepad unavailable: {exc}", file=sys.stderr)
    with KeyboardProvider(enabled=keyboard_enabled, cfg=cfg) as keyboard:
        last_tick_at = time.monotonic()
        next_controller_probe_at = 0.0
        try:
            while True:
                now = time.monotonic()

                if gamepad is not None and now >= next_controller_probe_at and not gamepad.ensure_connected():
                    if args.verbose:
                        print("waiting for controller...", file=sys.stderr)
                    next_controller_probe_at = now + args.wait_controller_sec
                actions = set()
                actions |= keyboard.poll_actions(now)
                if gamepad is not None:
                    actions |= gamepad.poll_actions(now)

                for action in actions:
                    if action == "emergency_stop":
                        emergency = True
                        mixer.reset()
                        send_brake(publisher.sock, args.left_port, args.right_port)
                        last_power = (0, 0)
                    elif action == "coast_stop":
                        mixer.reset()
                        send_stop(publisher.sock, args.left_port, args.right_port)
                        last_power = (0, 0)
                    elif action == "gyro_reset":
                        publisher.sock.sendall(gyro_reset_packet())
                    elif action == "start":
                        emergency = False
                    elif action == "toggle_safe_mode":
                        safe_mode = not safe_mode
                        print(f"safe_mode={'on' if safe_mode else 'off'}")
                    elif action == "shutdown":
                        print("shutdown requested")
                        return 0

                if emergency:
                    time.sleep(interval)
                    continue

                kb = keyboard.state(now)
                gp = gamepad.state() if gamepad is not None else None

                throttle = gp.throttle if gp is not None and abs(gp.throttle) > 0.01 else kb.throttle
                steering = gp.steering if gp is not None and abs(gp.steering) > 0.01 else kb.steering

                dt = max(0.0, now - last_tick_at)
                last_tick_at = now
                cap = min(power_limit, 30) if safe_mode else power_limit
                left, right, throttle_c, steering_c = mixer.mix(throttle, steering, cap, dt)
                if (left, right) != last_power:
                    publisher.publish_power(left, right)
                    publisher.log("drive", throttle=round(throttle_c, 3), steering=round(steering_c, 3), left_pwm=left, right_pwm=right, power_limit=cap)
                    last_power = (left, right)

                time.sleep(interval)
        finally:
            try:
                send_stop(publisher.sock, args.left_port, args.right_port)
            except Exception:
                pass
            if gamepad is not None and gamepad.controller is not None:
                gamepad.controller.close()
            publisher.close()


if __name__ == "__main__":
    raise SystemExit(main())
