#!/usr/bin/env python3
"""
Bridge a libraspike-facing pseudo terminal to a real SPIKE serial device.

Default layout:

    libraspike -> /tmp/raspike-tty -> raspike-bridge -> /dev/USB_SPIKE
                                                   +-> /tmp/raspike.sock

The Unix domain socket accepts raw RasPike protocol bytes. Bytes received from
the real serial device are sent to the pseudo terminal and all Unix socket
clients. Bytes received from the pseudo terminal or socket clients are sent to
the real serial device.
"""

from __future__ import annotations

import argparse
import errno
import os
import selectors
import signal
import socket
import stat
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

try:
    import pty
    import termios
except ModuleNotFoundError:
    pty = None
    termios = None

SUPPORTED_BAUD_RATES = (9600, 19200, 38400, 57600, 115200, 230400)


def termios_baud_rates() -> dict[int, int]:
    assert termios is not None
    return {
        9600: termios.B9600,
        19200: termios.B19200,
        38400: termios.B38400,
        57600: termios.B57600,
        115200: termios.B115200,
        230400: termios.B230400,
    }


@dataclass
class Endpoint:
    name: str
    fd: int
    buffer: bytearray = field(default_factory=bytearray)


class Bridge:
    def __init__(
        self,
        serial_path: str,
        pty_link: str,
        socket_path: str,
        baud: int,
        write_lock_ms: int,
        pty_priority_ms: int,
        verbose: bool,
    ):
        self.serial_path = serial_path
        self.pty_link = pty_link
        self.socket_path = socket_path
        self.baud = baud
        self.write_lock_seconds = write_lock_ms / 1000.0
        self.pty_priority_seconds = pty_priority_ms / 1000.0
        self.verbose = verbose
        self.selector = selectors.DefaultSelector()
        self.running = True
        self.clients: dict[int, Endpoint] = {}
        self.ptym: Endpoint | None = None
        self.pty_slave_fd: int | None = None
        self.serial: Endpoint | None = None
        self.server: socket.socket | None = None
        self.serial_writer_fd: int | None = None
        self.serial_writer_until = 0.0
        self.last_pty_write_at = 0.0

    def log(self, message: str) -> None:
        if self.verbose:
            print(message, file=sys.stderr, flush=True)

    def setup(self) -> None:
        self.serial = Endpoint("serial", self.open_serial())
        self.ptym = Endpoint("pty", self.open_pty())
        self.server = self.open_unix_server()

        self.selector.register(self.serial.fd, selectors.EVENT_READ, self.serial)
        self.selector.register(self.ptym.fd, selectors.EVENT_READ, self.ptym)
        self.selector.register(self.server, selectors.EVENT_READ, "server")

    def open_serial(self) -> int:
        fd = os.open(self.serial_path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        self.configure_serial(fd, self.baud)
        self.log(f"opened real serial: {self.serial_path}")
        return fd

    def configure_serial(self, fd: int, baud: int) -> None:
        assert termios is not None
        baud_rates = termios_baud_rates()
        attrs = termios.tcgetattr(fd)
        speed = baud_rates.get(baud)
        if speed is None:
            raise ValueError(f"unsupported baud rate: {baud}")

        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[3] = 0
        attrs[4] = speed
        attrs[5] = speed
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)

    def open_pty(self) -> int:
        assert pty is not None
        master_fd, slave_fd = pty.openpty()
        slave_name = os.ttyname(slave_fd)
        self.configure_pty(slave_fd)
        os.set_blocking(master_fd, False)
        self.pty_slave_fd = slave_fd
        self.replace_symlink(self.pty_link, slave_name)
        self.log(f"created pty: {self.pty_link} -> {slave_name}")
        return master_fd

    def configure_pty(self, fd: int) -> None:
        assert termios is not None
        attrs = termios.tcgetattr(fd)
        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[3] = 0
        attrs[6][termios.VMIN] = 1
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)

    def open_unix_server(self) -> socket.socket:
        path = Path(self.socket_path)
        if path.exists() or path.is_symlink():
            mode = path.lstat().st_mode
            if stat.S_ISSOCK(mode):
                path.unlink()
            else:
                raise FileExistsError(f"{self.socket_path} exists and is not a socket")
        path.parent.mkdir(parents=True, exist_ok=True)

        server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        server.setblocking(False)
        server.bind(self.socket_path)
        server.listen()
        self.log(f"listening unix socket: {self.socket_path}")
        return server

    def replace_symlink(self, link: str, target: str) -> None:
        path = Path(link)
        path.parent.mkdir(parents=True, exist_ok=True)
        if path.exists() or path.is_symlink():
            if not path.is_symlink():
                raise FileExistsError(f"{link} exists and is not a symlink")
            path.unlink()
        os.symlink(target, link)

    def run(self) -> None:
        while self.running:
            for key, mask in self.selector.select(timeout=1.0):
                data = key.data
                if data == "server":
                    self.accept_client()
                    continue

                endpoint = data
                if mask & selectors.EVENT_READ:
                    self.read_endpoint(endpoint)
                if mask & selectors.EVENT_WRITE:
                    self.flush_endpoint(endpoint)

    def accept_client(self) -> None:
        assert self.server is not None
        conn, _ = self.server.accept()
        conn.setblocking(False)
        fd = conn.detach()
        endpoint = Endpoint(f"client:{fd}", fd)
        self.clients[fd] = endpoint
        self.selector.register(fd, selectors.EVENT_READ, endpoint)
        self.log(f"accepted unix client fd={fd}")

    def read_endpoint(self, endpoint: Endpoint) -> None:
        try:
            data = os.read(endpoint.fd, 4096)
        except OSError as exc:
            if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                return
            self.close_endpoint(endpoint)
            return

        if not data:
            self.close_endpoint(endpoint)
            return

        if endpoint is self.serial:
            assert self.ptym is not None
            self.queue_write(self.ptym, data)
            for client in list(self.clients.values()):
                self.queue_write(client, data)
        else:
            if endpoint is self.ptym:
                self.last_pty_write_at = time.monotonic()
            elif self.pty_has_priority():
                self.log(f"dropped {len(data)} bytes from {endpoint.name}: pty priority is active")
                return
            if not self.acquire_serial_writer(endpoint, len(data)):
                return
            assert self.serial is not None
            self.queue_write(self.serial, data)

    def pty_has_priority(self) -> bool:
        if self.pty_priority_seconds <= 0:
            return False
        return time.monotonic() - self.last_pty_write_at < self.pty_priority_seconds

    def acquire_serial_writer(self, endpoint: Endpoint, data_len: int) -> bool:
        now = time.monotonic()
        if self.serial_writer_fd is None or now >= self.serial_writer_until:
            self.serial_writer_fd = endpoint.fd
        if self.serial_writer_fd != endpoint.fd:
            self.log(f"dropped {data_len} bytes from {endpoint.name}: serial write lock is busy")
            return False
        self.serial_writer_until = now + self.write_lock_seconds
        return True

    def queue_write(self, endpoint: Endpoint, data: bytes) -> None:
        if not data:
            return
        endpoint.buffer.extend(data)
        self.update_interest(endpoint)

    def flush_endpoint(self, endpoint: Endpoint) -> None:
        if not endpoint.buffer:
            self.update_interest(endpoint)
            return
        try:
            written = os.write(endpoint.fd, endpoint.buffer)
        except OSError as exc:
            if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                return
            self.close_endpoint(endpoint)
            return
        del endpoint.buffer[:written]
        self.update_interest(endpoint)

    def update_interest(self, endpoint: Endpoint) -> None:
        events = selectors.EVENT_READ
        if endpoint.buffer:
            events |= selectors.EVENT_WRITE
        try:
            self.selector.modify(endpoint.fd, events, endpoint)
        except (KeyError, ValueError, OSError):
            pass

    def close_endpoint(self, endpoint: Endpoint) -> None:
        if endpoint is self.serial or endpoint is self.ptym:
            self.running = False
        self.clients.pop(endpoint.fd, None)
        try:
            self.selector.unregister(endpoint.fd)
        except (KeyError, ValueError, OSError):
            pass
        try:
            os.close(endpoint.fd)
        except OSError:
            pass
        self.log(f"closed {endpoint.name}")

    def cleanup(self) -> None:
        for endpoint in [self.serial, self.ptym, *list(self.clients.values())]:
            if endpoint is not None:
                try:
                    os.close(endpoint.fd)
                except OSError:
                    pass
        if self.server is not None:
            try:
                self.selector.unregister(self.server)
            except Exception:
                pass
            self.server.close()
        if self.pty_slave_fd is not None:
            try:
                os.close(self.pty_slave_fd)
            except OSError:
                pass
        for path in [self.pty_link, self.socket_path]:
            try:
                p = Path(path)
                if p.is_symlink() or stat.S_ISSOCK(p.lstat().st_mode):
                    p.unlink()
            except FileNotFoundError:
                pass


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="PTY/Unix-socket bridge for RasPike serial traffic")
    parser.add_argument("--serial", default="/dev/USB_SPIKE", help="real SPIKE serial device")
    parser.add_argument("--pty-link", default="/tmp/raspike-tty", help="symlink exposed to libraspike")
    parser.add_argument("--unix-socket", default="/tmp/raspike.sock", help="raw protocol Unix socket path")
    parser.add_argument("--baud", type=int, default=115200, choices=SUPPORTED_BAUD_RATES, help="serial baud rate")
    parser.add_argument(
        "--write-lock-ms",
        type=int,
        default=20,
        help="minimum time window that one input source owns serial writes",
    )
    parser.add_argument(
        "--pty-priority-ms",
        type=int,
        default=200,
        help="drop Unix-socket writes while libraspike/PTY has written recently",
    )
    parser.add_argument("-v", "--verbose", action="store_true", help="print bridge events")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if pty is None or termios is None:
        print("raspike_bridge.py requires a POSIX/Linux Python with pty and termios modules.", file=sys.stderr)
        return 2

    bridge = Bridge(
        args.serial,
        args.pty_link,
        args.unix_socket,
        args.baud,
        args.write_lock_ms,
        args.pty_priority_ms,
        args.verbose,
    )

    def stop(_signum: int, _frame: object) -> None:
        bridge.running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    try:
        bridge.setup()
        print(f"libraspike tty: {args.pty_link}", flush=True)
        print(f"real serial:    {args.serial}", flush=True)
        print(f"unix socket:    {args.unix_socket}", flush=True)
        bridge.run()
    finally:
        bridge.cleanup()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
