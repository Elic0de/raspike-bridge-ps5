from __future__ import annotations

import json
import socket
import time
from pathlib import Path

from .protocol import RP_CMD_INIT, RP_CMD_INIT_MAGIC, motor_power_packet

_HANDSHAKE_TIMEOUT = 3.0
_VERSION_LEN = 3


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

    def log(self, kind: str, **fields: object) -> None:
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        record = {"time": time.time(), "kind": kind, **fields}
        with self.log_path.open("a", encoding="utf-8") as fp:
            fp.write(json.dumps(record, separators=(",", ":")) + "\n")

    def close(self) -> None:
        self.sock.close()
