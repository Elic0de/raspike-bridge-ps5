from __future__ import annotations

import json
import socket
import time
from pathlib import Path

from .protocol import RP_CMD_INIT, RP_CMD_INIT_MAGIC, motor_power_packet

_HANDSHAKE_TIMEOUT = 3.0
_HANDSHAKE_RESPONSE_LEN = 5


class StatePublisher:
    def __init__(self, socket_path: str, left_port: int, right_port: int, log_file: str):
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.connect(socket_path)
        self._handshake()
        self.sock.setblocking(False)
        self.left_port = left_port
        self.right_port = right_port
        self.log_path = Path(log_file)

    def _handshake(self) -> None:
        self.sock.settimeout(_HANDSHAKE_TIMEOUT)
        self.sock.sendall(bytes([RP_CMD_INIT, RP_CMD_INIT_MAGIC]))
        buf = bytearray()
        while len(buf) < _HANDSHAKE_RESPONSE_LEN:
            chunk = self.sock.recv(_HANDSHAKE_RESPONSE_LEN - len(buf))
            if not chunk:
                raise RuntimeError("bridge closed connection during handshake")
            buf.extend(chunk)
        if buf[0] != RP_CMD_INIT or buf[1] != RP_CMD_INIT_MAGIC:
            raise RuntimeError(f"bridge handshake failed: {buf.hex()}")

    def publish_power(self, left: int, right: int) -> None:
        self.sock.sendall(
            motor_power_packet(self.left_port, left)
            + motor_power_packet(self.right_port, right)
        )

    def log(self, kind: str, **fields: object) -> None:
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        record = {"time": time.time(), "kind": kind, **fields}
        with self.log_path.open("a", encoding="utf-8") as fp:
            fp.write(json.dumps(record, separators=(",", ":")) + "\n")

    def close(self) -> None:
        self.sock.close()
