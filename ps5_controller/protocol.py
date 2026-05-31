from __future__ import annotations

import socket
import struct

RP_CMD_INIT = 0xAE
RP_CMD_INIT_MAGIC = 0xCE
RP_CMD_START = 0xEA
RP_PORT_NONE = 255

RP_CMD_ID_ALL_STATUS = 0x01
RP_CMD_ID_ACK = 0x02

RP_CMD_ID_COL_CFG = 0x20
RP_CMD_ID_COL_RGB = 0x21
RP_CMD_ID_COL_COL = 0x22
RP_CMD_ID_COL_HSV = 0x24
RP_CMD_ID_COL_REF = 0x26
RP_CMD_ID_COL_AMB = 0x27
RP_CMD_ID_FRC_CFG = 0x40
RP_CMD_ID_MOT_CFG = 0x60
RP_CMD_ID_MOT_STU = 0x61
RP_CMD_ID_MOT_POW = 0x64
RP_CMD_ID_MOT_STP = 0x65
RP_CMD_ID_MOT_STP_BRK = 0x66
RP_CMD_ID_US_CFG = 0x80
RP_CMD_ID_HUB_IMU_RST_HDG = 0xB5
RP_CMD_ID_BRIDGE_VBUTTON = 0xF0
RP_CMD_ID_BRIDGE_VFORCE = 0xF1


def make_packet(port: int, cmd: int, payload: bytes = b"") -> bytes:
    return bytes([RP_CMD_START, cmd, len(payload), port]) + payload


def motor_config_packet(port: int) -> bytes:
    return make_packet(port, RP_CMD_ID_MOT_CFG)


def color_sensor_config_packet(port: int) -> bytes:
    return make_packet(port, RP_CMD_ID_COL_CFG)


def color_sensor_mode_packet(port: int, mode_cmd: int = RP_CMD_ID_COL_REF) -> bytes:
    return make_packet(port, mode_cmd)


def force_sensor_config_packet(port: int) -> bytes:
    return make_packet(port, RP_CMD_ID_FRC_CFG)


def ultrasonic_sensor_config_packet(port: int) -> bytes:
    return make_packet(port, RP_CMD_ID_US_CFG)


def motor_setup_packet(port: int, direction: int = 0, reset_count: bool = True) -> bytes:
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


def bridge_virtual_button_packet(button_bits: int, duration_ms: int = 120) -> bytes:
    # Bridge-local command: emulate hub button bits for a short pulse.
    payload = struct.pack("<IH", button_bits & 0xFFFFFFFF, max(0, min(duration_ms, 5000)))
    return make_packet(RP_PORT_NONE, RP_CMD_ID_BRIDGE_VBUTTON, payload)


def bridge_virtual_force_packet(port: int, touched: bool = True, duration_ms: int = 120) -> bytes:
    # Bridge-local command: emulate force sensor touch state for a short pulse.
    payload = struct.pack("<BBH", port & 0xFF, 1 if touched else 0, max(0, min(duration_ms, 5000)))
    return make_packet(RP_PORT_NONE, RP_CMD_ID_BRIDGE_VFORCE, payload)


def send_stop(sock: socket.socket, left_port: int, right_port: int) -> None:
    sock.sendall(motor_stop_packet(left_port) + motor_stop_packet(right_port))


def send_brake(sock: socket.socket, left_port: int, right_port: int) -> None:
    sock.sendall(motor_brake_packet(left_port) + motor_brake_packet(right_port))
