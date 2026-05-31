from __future__ import annotations

import json
import socket
import struct
import time
from pathlib import Path

from .protocol import RP_CMD_ID_ALL_STATUS, RP_CMD_INIT, RP_CMD_INIT_MAGIC, motor_power_packet

_HANDSHAKE_TIMEOUT = 3.0
_VERSION_LEN = 3
_CONNECT_TIMEOUT = 15.0
_STATUS_PORTS_OFFSET = 40
_STATUS_PORT_SIZE = 16
_STATUS_PORT_COUNT = 6
_STATUS_BUTTON_OFFSET = 36
_PORT_DATA_OFFSET = 4
_FORCE_TOUCHED_INDEX = 8


class StatePublisher:
    def __init__(self, socket_path: str, left_port: int, right_port: int, log_file: str):
        self.sock = self._connect(socket_path)
        self._handshake()
        self.sock.setblocking(False)
        self.left_port = left_port
        self.right_port = right_port
        self.log_path = Path(log_file)
        self._rx_buf = bytearray()
        self._button_bits = 0
        self._force_touched = [False] * _STATUS_PORT_COUNT

    def _connect(self, socket_path: str) -> socket.socket:
        # The bridge only creates the socket after its (possibly several second)
        # handshake with the real SPIKE, so retry until it appears.
        deadline = time.monotonic() + _CONNECT_TIMEOUT
        while True:
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            try:
                sock.connect(socket_path)
                return sock
            except (FileNotFoundError, ConnectionRefusedError):
                sock.close()
                if time.monotonic() >= deadline:
                    raise
                time.sleep(0.2)

    def _handshake(self) -> None:
        # The bridge multiplexes status frames onto this stream, so the
        # INIT/MAGIC reply may not be the first bytes we see. Scan for the
        # marker exactly like libraspike-art does, then drain the 3 version
        # bytes that follow it.
        self.sock.settimeout(_HANDSHAKE_TIMEOUT)
        self.sock.sendall(bytes([RP_CMD_INIT, RP_CMD_INIT_MAGIC]))
        deadline = time.monotonic() + _HANDSHAKE_TIMEOUT
        saw_init = False
        version_left = -1  # -1 until marker found, then counts down 3 bytes
        try:
            while time.monotonic() < deadline:
                chunk = self.sock.recv(64)
                if not chunk:
                    raise RuntimeError("bridge closed connection during handshake")
                for b in chunk:
                    if version_left >= 0:
                        version_left -= 1
                        if version_left == 0:
                            return
                    elif saw_init and b == RP_CMD_INIT_MAGIC:
                        version_left = _VERSION_LEN
                    else:
                        saw_init = b == RP_CMD_INIT
        except socket.timeout:
            pass
        raise RuntimeError("bridge handshake timed out")

    def publish_power(self, left: int, right: int) -> None:
        self.sock.sendall(
            motor_power_packet(self.left_port, left)
            + motor_power_packet(self.right_port, right)
        )

    def poll_status(self) -> None:
        while True:
            try:
                chunk = self.sock.recv(4096)
            except BlockingIOError:
                break
            if not chunk:
                break
            self._rx_buf.extend(chunk)
        self._parse_frames()

    def button_bits(self) -> int:
        return self._button_bits

    def force_touched(self, port: int) -> bool:
        if 0 <= port < len(self._force_touched):
            return self._force_touched[port]
        return False

    def _parse_frames(self) -> None:
        buf = self._rx_buf
        while buf:
            if buf[0] != 0xEA:
                del buf[0]
                continue
            if len(buf) < 4:
                return
            cmd = buf[1]
            size = buf[2]
            total = 4 + size
            if len(buf) < total:
                return
            payload = bytes(buf[4:total])
            del buf[:total]
            if cmd == RP_CMD_ID_ALL_STATUS:
                self._update_status(payload)

    def _update_status(self, payload: bytes) -> None:
        min_len = _STATUS_PORTS_OFFSET + _STATUS_PORT_COUNT * _STATUS_PORT_SIZE
        if len(payload) < min_len:
            return
        self._button_bits = struct.unpack_from("<I", payload, _STATUS_BUTTON_OFFSET)[0]
        for i in range(_STATUS_PORT_COUNT):
            off = _STATUS_PORTS_OFFSET + i * _STATUS_PORT_SIZE
            port = payload[off]
            if port >= _STATUS_PORT_COUNT:
                continue
            touched = payload[off + _PORT_DATA_OFFSET + _FORCE_TOUCHED_INDEX] != 0
            self._force_touched[port] = touched

    def log(self, kind: str, **fields: object) -> None:
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        record = {"time": time.time(), "kind": kind, **fields}
        with self.log_path.open("a", encoding="utf-8") as fp:
            fp.write(json.dumps(record, separators=(",", ":")) + "\n")

    def close(self) -> None:
        self.sock.close()
