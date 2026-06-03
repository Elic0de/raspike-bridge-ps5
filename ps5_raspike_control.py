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
    RP_CMD_ID_COL_CFG,
    RP_CMD_ID_COL_REF,
    RP_CMD_ID_FRC_CFG,
    RP_CMD_ID_MOT_CFG,
    RP_CMD_ID_MOT_STU,
    RP_CMD_ID_US_CFG,
    bridge_virtual_button_packet,
    bridge_virtual_force_packet,
    color_sensor_config_packet,
    color_sensor_mode_packet,
    force_sensor_config_packet,
    gyro_reset_packet,
    hub_restart_packet,
    motor_config_packet,
    motor_power_packet,
    motor_setup_packet,
    send_brake,
    send_stop,
    ultrasonic_sensor_config_packet,
)
from ps5_controller.remote_control import RemoteControlServer
from ps5_controller.state_publisher import StatePublisher
from ps5_controller.telemetry import UdpTelemetryPublisher


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


def send_and_wait_ack(
    sock: socket.socket,
    port: int,
    cmd: int,
    packet: bytes,
    label: str,
    *,
    retries: int = 0,
    retry_delay_sec: float = 0.2,
) -> None:
    last_error: Exception | None = None
    for attempt in range(retries + 1):
        if attempt:
            time.sleep(retry_delay_sec)
        sock.sendall(packet)
        try:
            ack = wait_ack(sock, port, cmd, timeout_sec=1.5)
        except Exception as exc:
            last_error = exc
            continue
        if ack == 1:
            return
        last_error = RuntimeError(f"{label} failed: port={port} ack={ack}")
    if last_error is not None:
        raise last_error
    raise RuntimeError(f"{label} failed: port={port}")


def configure_motors(sock: socket.socket, left_port: int, right_port: int, init_delay_sec: float, retries: int) -> None:
    send_and_wait_ack(
        sock, left_port, RP_CMD_ID_MOT_CFG, motor_config_packet(left_port), "MOT_CFG left", retries=retries
    )
    time.sleep(init_delay_sec)
    send_and_wait_ack(
        sock, right_port, RP_CMD_ID_MOT_CFG, motor_config_packet(right_port), "MOT_CFG right", retries=retries
    )
    time.sleep(init_delay_sec)
    send_and_wait_ack(
        sock, left_port, RP_CMD_ID_MOT_STU, motor_setup_packet(left_port, direction=1), "MOT_STU left", retries=retries
    )
    time.sleep(init_delay_sec)
    send_and_wait_ack(
        sock, right_port, RP_CMD_ID_MOT_STU, motor_setup_packet(right_port, direction=0), "MOT_STU right", retries=retries
    )
    time.sleep(init_delay_sec)


def configure_motor(sock: socket.socket, port: int, init_delay_sec: float, retries: int, direction: int = 0) -> None:
    send_and_wait_ack(sock, port, RP_CMD_ID_MOT_CFG, motor_config_packet(port), "MOT_CFG aux", retries=retries)
    time.sleep(init_delay_sec)
    send_and_wait_ack(
        sock, port, RP_CMD_ID_MOT_STU, motor_setup_packet(port, direction=direction), "MOT_STU aux", retries=retries
    )
    time.sleep(init_delay_sec)


def configure_force_sensor(sock: socket.socket, port: int, init_delay_sec: float, retries: int) -> None:
    send_and_wait_ack(sock, port, RP_CMD_ID_FRC_CFG, force_sensor_config_packet(port), "FRC_CFG", retries=retries)
    time.sleep(init_delay_sec)


def configure_color_sensor(sock: socket.socket, port: int, init_delay_sec: float, retries: int) -> None:
    send_and_wait_ack(sock, port, RP_CMD_ID_COL_CFG, color_sensor_config_packet(port), "COL_CFG", retries=retries)
    time.sleep(init_delay_sec)
    sock.sendall(color_sensor_mode_packet(port, RP_CMD_ID_COL_REF))
    time.sleep(init_delay_sec)


def configure_ultrasonic_sensor(sock: socket.socket, port: int, init_delay_sec: float, retries: int) -> None:
    send_and_wait_ack(sock, port, RP_CMD_ID_US_CFG, ultrasonic_sensor_config_packet(port), "US_CFG", retries=retries)
    time.sleep(init_delay_sec)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--socket", default="/tmp/raspike.sock")
    parser.add_argument("--event-device")
    parser.add_argument("--left-port", type=port_id, default=port_id("B"))
    parser.add_argument("--right-port", type=port_id, default=port_id("A"))
    parser.add_argument("--arm-port", type=port_id, default=port_id("C"))
    parser.add_argument("--no-arm", action="store_true", help="disable optional arm motor init and control")
    parser.add_argument("--force-port", type=port_id, default=port_id("D"))
    parser.add_argument("--color-port", type=port_id, default=port_id("E"))
    parser.add_argument("--ultrasonic-port", type=port_id, default=port_id("F"))
    parser.add_argument("--no-color-sensor", action="store_true")
    parser.add_argument("--no-ultrasonic-sensor", action="store_true")
    parser.add_argument("--max-power", type=int, default=100)
    parser.add_argument("--arm-power", type=int, default=30)
    parser.add_argument("--min-power", type=int, default=50)
    parser.add_argument("--power-step", type=int, default=5)
    parser.add_argument("--rate-hz", type=float, default=30.0)
    parser.add_argument("--init-delay-sec", type=float, default=0.2,
                        help="delay between startup MOT_CFG/MOT_STU/FRC_CFG commands")
    parser.add_argument("--init-retries", type=int, default=2,
                        help="retry count for startup device configuration commands")
    parser.add_argument("--init-order", choices=("arm-first", "drive-first"), default="drive-first",
                        help="startup motor configuration order")
    parser.add_argument("--log-file", default="/tmp/raspike-ps5-events.jsonl")
    parser.add_argument("--config", default="ps5_controller.yaml")
    parser.add_argument("--no-configure", action="store_true",
                        help="skip motor MOT_CFG/MOT_STU at startup (motors must already be configured, "
                             "otherwise the SPIKE crashes on the first motor command)")
    parser.add_argument("--keyboard", action="store_true", help="force keyboard input even when stdin is not auto-detected")
    parser.add_argument("--no-keyboard-fallback", action="store_true", help="disable automatic keyboard input")
    parser.add_argument("--wait-controller-sec", type=float, default=1.0)
    parser.add_argument("--telemetry-host", default="127.0.0.1")
    parser.add_argument("--telemetry-port", type=int, default=8765)
    parser.add_argument("--no-telemetry", action="store_true")
    parser.add_argument("--web-control-host", default="127.0.0.1")
    parser.add_argument("--web-control-port", type=int, default=8766)
    parser.add_argument("--no-web-control", action="store_true")
    parser.add_argument("--web-control-timeout-sec", type=float, default=0.35)
    parser.add_argument("-v", "--verbose", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if sys.platform != "linux":
        print("requires Linux evdev", file=sys.stderr)
        return 2

    cfg = load_config(args.config)
    publisher = StatePublisher(args.socket, args.left_port, args.right_port, args.log_file)
    telemetry = UdpTelemetryPublisher(args.telemetry_host, args.telemetry_port, enabled=not args.no_telemetry)
    remote = None
    if not args.no_web_control:
        remote = RemoteControlServer(
            args.web_control_host,
            args.web_control_port,
            args.web_control_timeout_sec,
            verbose=args.verbose,
        )
    if args.verbose:
        telemetry_label = "disabled" if args.no_telemetry else f"{args.telemetry_host}:{args.telemetry_port}"
        control_label = "disabled" if args.no_web_control else f"{args.web_control_host}:{args.web_control_port}"
        print(f"telemetry udp -> {telemetry_label}")
        print(f"web control tcp <- {control_label}")
    arm_enabled = not args.no_arm
    if not args.no_configure:
        init_delay_sec = max(0.0, args.init_delay_sec)
        init_retries = max(0, args.init_retries)
        if arm_enabled and args.init_order == "arm-first":
            try:
                configure_motor(publisher.sock, args.arm_port, init_delay_sec, init_retries, direction=0)
            except Exception as exc:
                arm_enabled = False
                print(f"warning: arm motor init skipped on port {args.arm_port} ({exc})", file=sys.stderr)
        configure_motors(publisher.sock, args.left_port, args.right_port, init_delay_sec, init_retries)
        if arm_enabled and args.init_order == "drive-first":
            try:
                configure_motor(publisher.sock, args.arm_port, init_delay_sec, init_retries, direction=0)
            except Exception as exc:
                arm_enabled = False
                print(f"warning: arm motor init skipped on port {args.arm_port} ({exc})", file=sys.stderr)
    if not args.no_configure:
        init_delay_sec = max(0.0, args.init_delay_sec)
        init_retries = max(0, args.init_retries)
        try:
            configure_force_sensor(
                publisher.sock,
                args.force_port,
                init_delay_sec,
                init_retries,
            )
        except Exception as exc:
            print(f"warning: force sensor init skipped on port {args.force_port} ({exc})", file=sys.stderr)
        else:
            publisher.set_sensor_type(args.force_port, "force")
        if not args.no_color_sensor:
            try:
                configure_color_sensor(publisher.sock, args.color_port, init_delay_sec, init_retries)
            except Exception as exc:
                print(f"warning: color sensor init skipped on port {args.color_port} ({exc})", file=sys.stderr)
            else:
                publisher.set_sensor_type(args.color_port, "color")
        if not args.no_ultrasonic_sensor:
            try:
                configure_ultrasonic_sensor(publisher.sock, args.ultrasonic_port, init_delay_sec, init_retries)
            except Exception as exc:
                print(
                    f"warning: ultrasonic sensor init skipped on port {args.ultrasonic_port} ({exc})",
                    file=sys.stderr,
                )
            else:
                publisher.set_sensor_type(args.ultrasonic_port, "ultrasonic")

    power_limit = clamp(args.max_power, args.min_power, 100)
    safe_mode = False
    emergency = False
    last_power: tuple[int, int] | None = None
    last_arm_power: int | None = None
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
                publisher.poll_status()

                if gamepad is not None and now >= next_controller_probe_at and not gamepad.ensure_connected():
                    if args.verbose:
                        print("waiting for controller...", file=sys.stderr)
                    next_controller_probe_at = now + args.wait_controller_sec
                actions = set()
                actions |= keyboard.poll_actions(now)
                if gamepad is not None:
                    actions |= gamepad.poll_actions(now)
                if remote is not None:
                    actions |= remote.poll_actions(now)

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
                    elif action == "center_button":
                        publisher.sock.sendall(bridge_virtual_button_packet(0x10, duration_ms=120))
                    elif action == "toggle_safe_mode":
                        safe_mode = not safe_mode
                        print(f"safe_mode={'on' if safe_mode else 'off'}")
                    elif action == "button_left":
                        publisher.sock.sendall(bridge_virtual_button_packet(0x08, duration_ms=120))
                    elif action == "button_right":
                        publisher.sock.sendall(bridge_virtual_button_packet(0x04, duration_ms=120))
                    elif action == "virtual_force_touch":
                        publisher.sock.sendall(
                            bridge_virtual_force_packet(args.force_port, touched=True, duration_ms=120)
                        )
                    elif action == "shutdown":
                        print("shutdown requested")
                        return 0
                    elif action == "restart":
                        print("restart requested")
                        publisher.sock.sendall(hub_restart_packet())
                        return 0

                if emergency:
                    telemetry.publish(
                        {
                            **publisher.telemetry_snapshot(),
                            "control": {
                                "safe_mode": safe_mode,
                                "emergency": emergency,
                                "power_limit": 0,
                                "throttle": 0.0,
                                "steering": 0.0,
                                "arm": 0.0,
                            },
                        }
                    )
                    time.sleep(interval)
                    continue

                kb = keyboard.state(now)
                gp = gamepad.state() if gamepad is not None else None
                rm = remote.state(now) if remote is not None else None

                throttle = kb.throttle
                steering = kb.steering
                arm = kb.arm
                if rm is not None and (
                    abs(rm.throttle) > 0.01 or abs(rm.steering) > 0.01 or abs(rm.arm) > 0.01
                ):
                    throttle = rm.throttle
                    steering = rm.steering
                    arm = rm.arm
                if gp is not None and (
                    abs(gp.throttle) > 0.01 or abs(gp.steering) > 0.01 or abs(gp.arm) > 0.01
                ):
                    throttle = gp.throttle
                    steering = gp.steering
                    arm = gp.arm

                dt = max(0.0, now - last_tick_at)
                last_tick_at = now
                cap = min(power_limit, 30) if safe_mode else power_limit
                left, right, throttle_c, steering_c = mixer.mix(throttle, steering, cap, dt)
                if (left, right) != last_power:
                    publisher.publish_power(left, right)
                    publisher.log("drive", throttle=round(throttle_c, 3), steering=round(steering_c, 3), left_pwm=left, right_pwm=right, power_limit=cap)
                    last_power = (left, right)
                if arm_enabled:
                    arm_pwm = int(round(clamp(int(arm * args.arm_power), -100, 100)))
                    if arm_pwm != last_arm_power:
                        publisher.sock.sendall(motor_power_packet(args.arm_port, arm_pwm))
                        last_arm_power = arm_pwm
                telemetry.publish(
                    {
                        **publisher.telemetry_snapshot(),
                        "control": {
                            "safe_mode": safe_mode,
                            "emergency": emergency,
                            "power_limit": cap,
                            "throttle": round(throttle_c, 3),
                            "steering": round(steering_c, 3),
                            "arm": round(arm, 3),
                        },
                    }
                )

                time.sleep(interval)
        finally:
            try:
                send_stop(publisher.sock, args.left_port, args.right_port)
                if arm_enabled:
                    publisher.sock.sendall(motor_power_packet(args.arm_port, 0))
            except Exception:
                pass
            if gamepad is not None and gamepad.controller is not None:
                gamepad.controller.close()
            if remote is not None:
                remote.close()
            telemetry.close()
            publisher.close()


if __name__ == "__main__":
    raise SystemExit(main())
