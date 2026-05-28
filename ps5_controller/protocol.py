from __future__ import annotations

import socket
import struct

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


def make_packet(port: int, cmd: int, payload: bytes = b"") -> bytes:
    return bytes([RP_CMD_START, cmd, len(payload), port]) + payload


def motor_config_packet(port: int) -> bytes:
    return make_packet(port, RP_CMD_ID_MOT_CFG)


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


def send_stop(sock: socket.socket, left_port: int, right_port: int) -> None:
    sock.sendall(motor_stop_packet(left_port) + motor_stop_packet(right_port))


def send_brake(sock: socket.socket, left_port: int, right_port: int) -> None:
    sock.sendall(motor_brake_packet(left_port) + motor_brake_packet(right_port))
