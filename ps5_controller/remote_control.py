from __future__ import annotations

import json
import selectors
import socket
import time
from dataclasses import dataclass

from .input_provider import ProviderState


@dataclass
class _Client:
    sock: socket.socket
    buffer: bytearray


class RemoteControlServer:
    def __init__(self, host: str, port: int, timeout_sec: float, verbose: bool = False):
        self.timeout_sec = timeout_sec
        self.verbose = verbose
        self._selector = selectors.DefaultSelector()
        self._clients: dict[int, _Client] = {}
        self._actions: set[str] = set()
        self._state = ProviderState()
        self._enabled = False
        self._last_drive_at = 0.0
        self._last_log = ""
        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind((host, port))
        self._server.listen()
        self._server.setblocking(False)
        self._selector.register(self._server, selectors.EVENT_READ, self._accept)

    def poll_actions(self, now: float) -> set[str]:
        for key, _ in self._selector.select(timeout=0.0):
            callback = key.data
            callback(key.fileobj)
        if self._enabled and now - self._last_drive_at > self.timeout_sec:
            self._enabled = False
            self._state = ProviderState()
            self._actions.add("coast_stop")
        actions = self._actions
        self._actions = set()
        return actions

    def state(self, now: float) -> ProviderState:
        if not self._enabled or now - self._last_drive_at > self.timeout_sec:
            return ProviderState()
        return self._state

    def close(self) -> None:
        for client in list(self._clients.values()):
            self._close_client(client.sock)
        self._selector.unregister(self._server)
        self._server.close()
        self._selector.close()

    def _accept(self, server: socket.socket) -> None:
        sock, _ = server.accept()
        sock.setblocking(False)
        client = _Client(sock=sock, buffer=bytearray())
        self._clients[sock.fileno()] = client
        self._selector.register(sock, selectors.EVENT_READ, self._read_client)

    def _read_client(self, sock: socket.socket) -> None:
        client = self._clients.get(sock.fileno())
        if client is None:
            return
        try:
            chunk = sock.recv(4096)
        except OSError:
            self._close_client(sock)
            return
        if not chunk:
            self._close_client(sock)
            return
        client.buffer.extend(chunk)
        while b"\n" in client.buffer:
            line, _, rest = client.buffer.partition(b"\n")
            client.buffer = bytearray(rest)
            self._handle_line(line)

    def _handle_line(self, line: bytes) -> None:
        try:
            msg = json.loads(line.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            return
        if not isinstance(msg, dict):
            return

        msg_type = msg.get("type")
        if msg_type == "enable":
            self._enabled = bool(msg.get("enabled"))
            if self._enabled:
                self._last_drive_at = time.monotonic()
            else:
                self._state = ProviderState()
                self._actions.add("coast_stop")
        elif msg_type == "drive":
            if not self._enabled:
                return
            state = ProviderState(
                throttle=self._axis(msg.get("throttle")),
                steering=self._axis(msg.get("steering")),
                arm=self._axis(msg.get("arm")),
            )
            self._state = state
            self._last_drive_at = time.monotonic()
            self._log(
                f"web control received: drive throttle={state.throttle:g} "
                f"steering={state.steering:g} arm={state.arm:g}"
            )
        elif msg_type == "action":
            action = msg.get("action")
            if isinstance(action, str):
                self._actions.add(action)
                self._log(f"web control received: action={action}")

    @staticmethod
    def _axis(value: object) -> float:
        if not isinstance(value, (int, float)):
            return 0.0
        return max(-1.0, min(1.0, float(value)))

    def _close_client(self, sock: socket.socket) -> None:
        fileno = sock.fileno()
        try:
            self._selector.unregister(sock)
        except Exception:
            pass
        self._clients.pop(fileno, None)
        sock.close()

    def _log(self, message: str) -> None:
        if self.verbose and message != self._last_log:
            print(message)
            self._last_log = message
