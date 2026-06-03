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
#!/usr/bin/env python3
from __future__ import annotations

import argparse
import errno
import os
import selectors
import signal
import socket
import stat
import struct
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

RP_CMD_INIT = 0xAE
RP_CMD_INIT_MAGIC = 0xCE
RP_CMD_START = 0xEA
RP_CMD_ID_ALL_STATUS = 0x01
RP_CMD_ID_ACK = 0x02
RP_CMD_ID_SHT_DWN = 0x03
RP_CMD_ID_RESTART = 0x04
RP_PORT_NONE = 0xFF
SPIKE_VERSION = bytes([0, 0, 6])

# *_CFG commands (RP_CMD_TYPE_*<<5 | 0). The SPIKE firmware grabs the port
# device on these and asserts (red LED) if the same port is configured twice.
# Re-running a libraspike-art program would trigger that, so the bridge proxies
# the ACK for ports it has already forwarded a config for.
RP_CMD_ID_COL_CFG = 0x20
RP_CMD_ID_FRC_CFG = 0x40
RP_CMD_ID_MOT_CFG = 0x60
RP_CMD_ID_MOT_STU = 0x61
RP_CMD_ID_MOT_RST = 0x62
RP_CMD_ID_US_CFG = 0x80
CONFIG_COMMANDS = frozenset(
    {RP_CMD_ID_COL_CFG, RP_CMD_ID_FRC_CFG, RP_CMD_ID_MOT_CFG, RP_CMD_ID_US_CFG}
)
RP_MOTOR_STU_INDEX_RESETCOUNT = 4

# Command types (cmd >> 5) that address a port device and therefore require the
# port to be configured first: COLOR(1), FORCE(2), MOTOR(3), ULTRASONIC(4).
# Their *_CFG command is (type << 5), i.e. the entries in CONFIG_COMMANDS.
PORT_DEVICE_TYPES = frozenset({1, 2, 3, 4})

SPIKE_STATUS_PORTS_OFFSET = 32
SPIKE_STATUS_PORTS_OFFSET_LEGACY = 40
SPIKE_STATUS_PORT_COUNT = 6
SPIKE_STATUS_PORT_SIZE = 16
SPIKE_STATUS_PORT_INDEX = 0
SPIKE_STATUS_PORT_CMD_INDEX = 1
SPIKE_STATUS_PORT_DATA_OFFSET = 4
SPIKE_STATUS_FORCE_VALUE_INDEX = 0
SPIKE_STATUS_FORCE_TOUCHED_INDEX = 8
SPIKE_STATUS_BUTTON_OFFSET = 28
SPIKE_STATUS_BUTTON_OFFSET_LEGACY = 36
RP_CMD_ID_BRIDGE_VBUTTON = 0xF0
RP_CMD_ID_BRIDGE_VFORCE = 0xF1
VIRTUAL_FORCE_NEWTONS = 10.0


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
    parse_buf: bytearray = field(default_factory=bytearray)


class Bridge:
    def __init__(
        self,
        serial_path: str,
        pty_link: str,
        socket_path: str,
        baud: int,
        write_lock_ms: int,
        pty_priority_ms: int,
        pty_mode: int,
        spike_handshake: bool,
        shutdown_spike_on_signal: bool,
        restart_spike_on_signal: bool,
        restart_spike_on_broken_pipe: bool,
        verbose: bool,
        handshake_timeout_sec: float,
    ):
        self.serial_path = serial_path
        self.pty_link = pty_link
        self.socket_path = socket_path
        self.baud = baud
        self.write_lock_seconds = write_lock_ms / 1000.0
        self.pty_priority_seconds = pty_priority_ms / 1000.0
        self.pty_mode = pty_mode
        self.spike_handshake = spike_handshake
        self.shutdown_spike_on_signal = shutdown_spike_on_signal
        self.restart_spike_on_signal = restart_spike_on_signal
        self.restart_spike_on_broken_pipe = restart_spike_on_broken_pipe
        self.verbose = verbose
        self.handshake_timeout_sec = handshake_timeout_sec

        self.selector = selectors.DefaultSelector()
        self.running = True
        self.stop_by_signal = False
        self.stop_by_broken_pipe = False
        self.system_command_sent = False
        self.clients: dict[int, Endpoint] = {}
        self.ptym: Endpoint | None = None
        self.pty_slave_fd: int | None = None
        self.serial: Endpoint | None = None
        self.server: socket.socket | None = None
        self.serial_writer_fd: int | None = None
        self.serial_writer_until = 0.0
        self.last_pty_write_at = 0.0
        self.virtual_button_bits = 0
        self.virtual_button_until = 0.0
        self.virtual_force_until = [0.0] * SPIKE_STATUS_PORT_COUNT
        # (port, cmd) pairs whose config has already been forwarded to the real
        # SPIKE. Subsequent configs for the same pair are answered locally.
        self.configured_cmds: set[tuple[int, int]] = set()
        self.pending_config_cmds: set[tuple[int, int]] = set()
        self.status_configured_cmds: set[tuple[int, int]] | None = None
        self.status_setup_motor_ports: set[int] = set()

    def log(self, message: str) -> None:
        if self.verbose:
            print(message, file=sys.stderr, flush=True)

    def setup(self) -> None:
        self.serial = Endpoint("serial", self.open_serial())

        if self.spike_handshake:
            self.handshake_spike()

        self.ptym = Endpoint("pty", self.open_pty())
        self.server = self.open_unix_server()

        self.selector.register(self.serial.fd, selectors.EVENT_READ, self.serial)
        self.selector.register(self.ptym.fd, selectors.EVENT_READ, self.ptym)
        self.selector.register(self.server, selectors.EVENT_READ, "server")
        self.flush_serial_parse_buf()

    def handshake_spike(self) -> None:
        assert self.serial is not None
        fd = self.serial.fd
        init = bytes([RP_CMD_INIT, RP_CMD_INIT_MAGIC])

        # The hub waits ~1s after the port opens (and may reset on open) before
        # it starts reading, so a single send is easily missed. Re-send the
        # INIT marker periodically and scan the reply, skipping any boot noise
        # (e.g. leading zero bytes), exactly like libraspike-art's read loop.
        deadline = time.monotonic() + self.handshake_timeout_sec
        next_send = 0.0
        saw_init = False
        collecting = False
        version = bytearray()

        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_send:
                try:
                    os.write(fd, init)
                except (BlockingIOError, InterruptedError):
                    pass
                except OSError as exc:
                    if exc.errno not in (errno.EAGAIN, errno.EWOULDBLOCK):
                        raise
                next_send = now + 0.25

            try:
                chunk = os.read(fd, 64)
            except BlockingIOError:
                chunk = b""
            except OSError as exc:
                if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                    chunk = b""
                else:
                    raise

            if not chunk:
                time.sleep(0.01)
                continue

            for i, b in enumerate(chunk):
                if collecting:
                    version.append(b)
                    if len(version) == 3:
                        # Keep anything that trailed the reply for the main loop.
                        self.serial.parse_buf.extend(chunk[i + 1:])
                        self.log(f"SPIKE handshake ok: version={tuple(version)}")
                        return
                elif b == RP_CMD_START:
                    # Hub is already past its boot handshake and streaming
                    # status frames. Treat it as alive and resume from here.
                    self.serial.parse_buf.extend(chunk[i:])
                    self.log("SPIKE already running (status frame seen); handshake skipped")
                    return
                elif saw_init and b == RP_CMD_INIT_MAGIC:
                    collecting = True
                else:
                    saw_init = b == RP_CMD_INIT

        raise RuntimeError(
            "SPIKE handshake timed out: no reply from the hub on "
            f"{self.serial_path}. Reboot/restart the SPIKE program and retry."
        )

    def open_serial(self) -> int:
        fd = os.open(self.serial_path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        self.configure_serial(fd, self.baud)
        self.log(f"opened real serial: {self.serial_path}")
        return fd

    def configure_serial(self, fd: int, baud: int) -> None:
        assert termios is not None
        speed = termios_baud_rates().get(baud)
        if speed is None:
            raise ValueError(f"unsupported baud rate: {baud}")

        attrs = termios.tcgetattr(fd)
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
        os.chmod(slave_name, self.pty_mode)
        os.set_blocking(master_fd, False)
        self.pty_slave_fd = slave_fd
        self.replace_symlink(self.pty_link, slave_name)
        self.log(f"created pty: {self.pty_link} -> {slave_name} mode={self.pty_mode:o}")
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
            # Fan out only complete frames so a proxied ACK (injected from the
            # client path) can never land inside a half-written status frame.
            endpoint.parse_buf.extend(data)
            frames = self.extract_frames(endpoint.parse_buf)
            if not frames:
                return
            self.learn_configured_cmds_from_acks(frames)
            self.learn_configured_cmds_from_status(frames)
            frames = self.apply_virtual_inputs(frames)
            self.queue_write(self.ptym, frames)
            for client in list(self.clients.values()):
                self.queue_write(client, frames)
        else:
            data = self.swallow_client_handshake(endpoint, data)
            if not data:
                return

            data = self.filter_config_frames(endpoint, data)
            if not data:
                return

            if endpoint is self.ptym:
                self.last_pty_write_at = time.monotonic()
            elif self.pty_has_priority():
                self.log(f"dropped {len(data)} bytes from {endpoint.name}: pty priority is active")
                return

            if not self.acquire_serial_writer(endpoint, len(data)):
                return

            assert self.serial is not None
            self.queue_write(self.serial, data)

    def swallow_client_handshake(self, endpoint: Endpoint, data: bytes) -> bytes:
        if len(data) < 2:
            return data
        if data[0] != RP_CMD_INIT or data[1] != RP_CMD_INIT_MAGIC:
            return data

        response = bytes([RP_CMD_INIT, RP_CMD_INIT_MAGIC]) + SPIKE_VERSION
        if endpoint is self.ptym:
            self.refresh_config_cache_for_pty_session()
        self.queue_write(endpoint, response)
        self.log(f"proxy handshake for {endpoint.name}: req={data[:2].hex()} resp={response.hex()}")
        return data[2:]

    def extract_frames(self, buf: bytearray) -> bytes:
        """Pull whole RasPike frames out of buf, leaving any partial tail behind."""
        out = bytearray()
        while buf:
            if buf[0] != RP_CMD_START:
                del buf[0]
                continue
            if len(buf) < 4:
                break
            total = 4 + buf[2]
            if len(buf) < total:
                break
            out += buf[:total]
            del buf[:total]
        return bytes(out)

    def iter_frames(self, frames: bytes):
        offset = 0
        while offset + 4 <= len(frames):
            if frames[offset] != RP_CMD_START:
                offset += 1
                continue
            total = 4 + frames[offset + 2]
            if offset + total > len(frames):
                break
            yield frames[offset:offset + total]
            offset += total

    def flush_serial_parse_buf(self) -> None:
        assert self.serial is not None
        assert self.ptym is not None
        frames = self.extract_frames(self.serial.parse_buf)
        if not frames:
            return
        self.learn_configured_cmds_from_acks(frames)
        self.learn_configured_cmds_from_status(frames)
        frames = self.apply_virtual_inputs(frames)
        self.queue_write(self.ptym, frames)

    def apply_virtual_inputs(self, frames: bytes) -> bytes:
        now = time.monotonic()
        if now >= self.virtual_button_until:
            self.virtual_button_bits = 0
        virtual_force_ports = {idx for idx, until in enumerate(self.virtual_force_until) if now < until}
        if self.virtual_button_bits == 0 and not virtual_force_ports:
            return frames

        out = bytearray()
        for frame in self.iter_frames(frames):
            if frame[1] != RP_CMD_ID_ALL_STATUS or frame[3] != RP_PORT_NONE:
                out += frame
                continue
            layout = self.status_layout(frame[2])
            if layout is None:
                out += frame
                continue
            ports_offset, button_offset = layout
            mutable = bytearray(frame)
            if frame[2] >= (button_offset + 4) and self.virtual_button_bits:
                bits_offset = 4 + button_offset
                current = struct.unpack_from("<I", mutable, bits_offset)[0]
                struct.pack_into("<I", mutable, bits_offset, current | self.virtual_button_bits)
            if frame[2] >= (ports_offset + SPIKE_STATUS_PORT_COUNT * SPIKE_STATUS_PORT_SIZE) and virtual_force_ports:
                for i in range(SPIKE_STATUS_PORT_COUNT):
                    base = 4 + ports_offset + i * SPIKE_STATUS_PORT_SIZE
                    status_port = mutable[base + SPIKE_STATUS_PORT_INDEX]
                    if status_port in virtual_force_ports:
                        data_idx = base + SPIKE_STATUS_PORT_DATA_OFFSET
                        struct.pack_into(
                            "<f",
                            mutable,
                            data_idx + SPIKE_STATUS_FORCE_VALUE_INDEX,
                            VIRTUAL_FORCE_NEWTONS,
                        )
                        mutable[data_idx + SPIKE_STATUS_FORCE_TOUCHED_INDEX] = 1
            out += mutable
        return bytes(out)

    def status_layout(self, payload_size: int) -> tuple[int, int] | None:
        if payload_size >= SPIKE_STATUS_PORTS_OFFSET + SPIKE_STATUS_PORT_COUNT * SPIKE_STATUS_PORT_SIZE:
            return SPIKE_STATUS_PORTS_OFFSET, SPIKE_STATUS_BUTTON_OFFSET
        if payload_size >= SPIKE_STATUS_PORTS_OFFSET_LEGACY + SPIKE_STATUS_PORT_COUNT * SPIKE_STATUS_PORT_SIZE:
            return SPIKE_STATUS_PORTS_OFFSET_LEGACY, SPIKE_STATUS_BUTTON_OFFSET_LEGACY
        return None

    def learn_configured_cmds_from_status(self, frames: bytes) -> None:
        """Infer already-configured devices from SPIKE status frames.

        If the bridge attaches to an already-running SPIKE program, its local
        configured_cmds cache starts empty even though the firmware may already
        own motor/sensor devices. Status frames include each port's active
        command, so use that to avoid forwarding duplicate *_CFG commands that
        would trip the firmware's "device must be unconfigured" assert.
        """
        for frame in self.iter_frames(frames):
            cmd = frame[1]
            size = frame[2]
            port = frame[3]
            if cmd != RP_CMD_ID_ALL_STATUS or port != RP_PORT_NONE:
                continue
            layout = self.status_layout(size)
            if layout is None:
                continue
            ports_offset, _button_offset = layout

            payload = frame[4:]
            status_configured_cmds: set[tuple[int, int]] = set()
            status_setup_motor_ports: set[int] = set()
            for i in range(SPIKE_STATUS_PORT_COUNT):
                offset = ports_offset + i * SPIKE_STATUS_PORT_SIZE
                status_port = payload[offset + SPIKE_STATUS_PORT_INDEX]
                status_cmd = payload[offset + SPIKE_STATUS_PORT_CMD_INDEX]
                cmd_type = status_cmd >> 5
                if status_port >= SPIKE_STATUS_PORT_COUNT or cmd_type not in PORT_DEVICE_TYPES:
                    continue

                config_cmd = cmd_type << 5
                key = (status_port, config_cmd)
                status_configured_cmds.add(key)
                if status_cmd == RP_CMD_ID_MOT_STU:
                    status_setup_motor_ports.add(status_port)
                if key not in self.configured_cmds:
                    self.configured_cmds.add(key)
                    self.pending_config_cmds.discard(key)
                    self.log(
                        "learned configured port from status: "
                        f"port={status_port} cmd=0x{config_cmd:02x}"
                    )
            self.status_configured_cmds = status_configured_cmds
            self.status_setup_motor_ports = status_setup_motor_ports

    def learn_configured_cmds_from_acks(self, frames: bytes) -> None:
        """Update config cache from real firmware ACKs.

        Keep *_CFG commands pending until the SPIKE reports success. This lets
        startup clients retry a transient failure instead of being handed a
        proxied success for a config the firmware rejected.
        """
        for frame in self.iter_frames(frames):
            if frame[1] != RP_CMD_ID_ACK or frame[2] < 8:
                continue
            ack_port = frame[3]
            ack_cmd, ack_data = struct.unpack("<ii", frame[4:12])
            if ack_cmd not in CONFIG_COMMANDS:
                continue
            key = (ack_port, ack_cmd)
            self.pending_config_cmds.discard(key)
            if ack_data == 1:
                if key not in self.configured_cmds:
                    self.log(f"learned configured port from ack: port={ack_port} cmd=0x{ack_cmd:02x}")
                self.configured_cmds.add(key)
            elif key in self.configured_cmds:
                self.configured_cmds.discard(key)
                self.log(f"removed failed config from cache: port={ack_port} cmd=0x{ack_cmd:02x}")

    def refresh_config_cache_for_pty_session(self) -> None:
        """Reset stale per-run config cache when libraspike reconnects.

        The SPIKE-side RasPike firmware can be restarted independently from this
        bridge, while the bridge process keeps running. When a new libraspike
        process connects through the PTY, prefer the latest observed status over
        the bridge's old forwarded-config cache so duplicate config protection
        still works without hiding configs that the restarted firmware needs.
        """
        before = set(self.configured_cmds)
        # Motor setup is visible in status frames, so it can be refreshed from
        # the observed SPIKE state. Sensor *_CFG state is not reflected in the
        # status command byte until a later sensor mode command, so keep the
        # bridge-owned record for those to avoid duplicate CFG asserts.
        preserved_sensor_cmds = {
            key for key in self.configured_cmds if key[1] != RP_CMD_ID_MOT_CFG
        }
        self.configured_cmds = set(self.status_configured_cmds or ()) | preserved_sensor_cmds
        if self.configured_cmds != before:
            self.log(
                "refreshed config cache for pty session: "
                f"before={sorted(before)} after={sorted(self.configured_cmds)}"
            )
        self.pending_config_cmds.clear()

    def filter_config_frames(self, endpoint: Endpoint, data: bytes) -> bytes:
        """
        Forward client frames to the serial, proxying duplicate *_CFG ACKs.

        Configured (port, cmd) pairs are learned from both forwarded config
        commands and SPIKE ALL_STATUS frames.

        Unknown configs are forwarded to the real SPIKE. If a port is already
        known to be configured, the bridge responds with a proxied ACK instead
        of forwarding the config command again. This prevents the firmware from
        re-grabbing an already configured device and triggering its duplicate
        configuration assert.

        This also allows the bridge to attach to an already-running SPIKE
        program and safely handle MOT_CFG requests for ports that were
        configured before the bridge started.
        """
        endpoint.parse_buf.extend(data)
        buf = endpoint.parse_buf
        forward = bytearray()
        while buf:
            if buf[0] != RP_CMD_START:
                del buf[0]
                continue
            if len(buf) < 4:
                break
            total = 4 + buf[2]
            if len(buf) < total:
                break
            frame = bytes(buf[:total])
            del buf[:total]

            cmd = frame[1]
            port = frame[3]
            cmd_type = cmd >> 5

            if cmd == RP_CMD_ID_BRIDGE_VBUTTON:
                if len(frame) >= 10:
                    button_bits, duration_ms = struct.unpack("<IH", frame[4:10])
                    self.virtual_button_bits = button_bits
                    self.virtual_button_until = time.monotonic() + (duration_ms / 1000.0)
                    self.log(
                        f"virtual button pulse from {endpoint.name}: bits=0x{button_bits:08x} "
                        f"duration_ms={duration_ms}"
                    )
                continue

            if cmd == RP_CMD_ID_BRIDGE_VFORCE:
                if len(frame) >= 8:
                    force_port, touched, duration_ms = struct.unpack("<BBH", frame[4:8])
                    if 0 <= force_port < SPIKE_STATUS_PORT_COUNT and touched:
                        self.virtual_force_until[force_port] = time.monotonic() + (duration_ms / 1000.0)
                        self.log(
                            f"virtual force pulse from {endpoint.name}: port={force_port} "
                            f"duration_ms={duration_ms}"
                        )
                continue

            if cmd in CONFIG_COMMANDS:
                # *_CFG: forward the first one (and remember it), proxy the rest.
                if (port, cmd) in self.configured_cmds:
                    self.send_proxy_ack(endpoint, port, cmd)
                    self.log(f"proxy config ack for {endpoint.name}: port={port} cmd=0x{cmd:02x}")
                    continue
                if (port, cmd) in self.pending_config_cmds:
                    self.log(f"dropped duplicate pending config from {endpoint.name}: port={port} cmd=0x{cmd:02x}")
                    continue
                self.pending_config_cmds.add((port, cmd))
                forward += frame
                continue

            if cmd == RP_CMD_ID_MOT_STU and port in self.status_setup_motor_ports:
                if len(frame) > 4 + RP_MOTOR_STU_INDEX_RESETCOUNT and frame[4 + RP_MOTOR_STU_INDEX_RESETCOUNT]:
                    forward += bytes([RP_CMD_START, RP_CMD_ID_MOT_RST, 0, port])
                    self.log(f"translated duplicate motor setup reset for {endpoint.name}: port={port}")
                self.send_proxy_ack(endpoint, port, cmd)
                self.log(f"proxy motor setup ack for {endpoint.name}: port={port}")
                continue

            # Any other per-port device command (color/force/motor/ultrasonic)
            # makes the firmware dereference a NULL device and crash if the port
            # was never configured for that type. Drop it as a safety net.
            if cmd_type in PORT_DEVICE_TYPES and (port, cmd_type << 5) not in self.configured_cmds:
                self.log(f"dropped cmd 0x{cmd:02x} from {endpoint.name}: port {port} not configured")
                continue

            forward += frame

        return bytes(forward)

    def send_proxy_ack(self, endpoint: Endpoint, port: int, cmd: int) -> None:
        # Mirrors the firmware's send_ack: payload is two little-endian int32s,
        # the acked command id and the result (1 == success).
        payload = struct.pack("<ii", cmd, 1)
        ack = bytes([RP_CMD_START, RP_CMD_ID_ACK, len(payload), port]) + payload
        self.queue_write(endpoint, ack)

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
            if exc.errno == errno.EPIPE:
                self.log(f"broken pipe on {endpoint.name}")
                self.stop_by_broken_pipe = True
                self.running = False
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

    def request_spike_system_command(self, cmd: int, label: str) -> None:
        if self.system_command_sent or self.serial is None:
            return
        self.system_command_sent = True
        packet = bytes([RP_CMD_START, cmd, 0, RP_PORT_NONE])
        deadline = time.monotonic() + 0.3
        offset = 0
        while offset < len(packet) and time.monotonic() < deadline:
            try:
                offset += os.write(self.serial.fd, packet[offset:])
            except (BlockingIOError, InterruptedError):
                time.sleep(0.01)
            except OSError as exc:
                if exc.errno not in (errno.EAGAIN, errno.EWOULDBLOCK):
                    self.log(f"failed to send SPIKE {label} command: {exc}")
                    return
                time.sleep(0.01)
        if offset == len(packet):
            self.log(f"sent SPIKE {label} command")
            try:
                termios.tcdrain(self.serial.fd)
            except (AttributeError, OSError):
                pass
        else:
            self.log(f"timed out sending SPIKE {label} command")

    def request_spike_shutdown(self) -> None:
        self.request_spike_system_command(RP_CMD_ID_SHT_DWN, "shutdown")

    def request_spike_restart(self) -> None:
        self.request_spike_system_command(RP_CMD_ID_RESTART, "restart")

    def cleanup(self) -> None:
        if self.restart_spike_on_signal and self.stop_by_signal:
            self.request_spike_restart()
        elif self.shutdown_spike_on_signal and self.stop_by_signal:
            self.request_spike_shutdown()
        elif self.restart_spike_on_broken_pipe and self.stop_by_broken_pipe:
            self.request_spike_restart()

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
    parser.add_argument("--baud", type=int, default=115200, choices=SUPPORTED_BAUD_RATES)
    parser.add_argument("--write-lock-ms", type=int, default=20)
    parser.add_argument("--pty-priority-ms", type=int, default=200)
    parser.add_argument("--pty-mode", type=lambda value: int(value, 8), default=0o666)
    parser.add_argument("--no-spike-handshake", action="store_true",
                        help="skip the one-time handshake with the real SPIKE (normally required)")
    parser.add_argument("--shutdown-spike-on-signal", action="store_true",
                        help="send RP_CMD_ID_SHT_DWN to the SPIKE when the bridge receives SIGINT/SIGTERM")
    parser.add_argument("--restart-spike-on-signal", action="store_true",
                        help="send RP_CMD_ID_RESTART to the SPIKE when the bridge receives SIGINT/SIGTERM")
    parser.add_argument("--restart-spike-on-broken-pipe", action="store_true",
                        help="send RP_CMD_ID_RESTART to the SPIKE when the bridge hits EPIPE while writing")
    parser.add_argument("--handshake-timeout-sec", type=float, default=10.0)
    parser.add_argument("-v", "--verbose", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if pty is None or termios is None:
        print("raspike_bridge.py requires POSIX/Linux Python.", file=sys.stderr)
        return 2

    bridge = Bridge(
        serial_path=args.serial,
        pty_link=args.pty_link,
        socket_path=args.unix_socket,
        baud=args.baud,
        write_lock_ms=args.write_lock_ms,
        pty_priority_ms=args.pty_priority_ms,
        pty_mode=args.pty_mode,
        spike_handshake=not args.no_spike_handshake,
        shutdown_spike_on_signal=args.shutdown_spike_on_signal,
        restart_spike_on_signal=args.restart_spike_on_signal,
        restart_spike_on_broken_pipe=args.restart_spike_on_broken_pipe,
        verbose=args.verbose,
        handshake_timeout_sec=args.handshake_timeout_sec,
    )

    def stop(_signum: int, _frame: object) -> None:
        bridge.stop_by_signal = True
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
