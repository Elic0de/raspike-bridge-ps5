from __future__ import annotations

import json
import socket


class UdpTelemetryPublisher:
    def __init__(self, host: str, port: int, enabled: bool = True):
        self.enabled = enabled
        self.addr = (host, port)
        self.sock: socket.socket | None = None
        if enabled:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setblocking(False)

    def publish(self, payload: dict[str, object]) -> None:
        if not self.enabled or self.sock is None:
            return
        data = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        try:
            self.sock.sendto(data, self.addr)
        except OSError:
            pass

    def close(self) -> None:
        if self.sock is not None:
            self.sock.close()
