from __future__ import annotations

import os
import select
import sys
import termios
import time
import tty
from dataclasses import dataclass

try:
    from evdev import InputDevice, ecodes, list_devices
except ModuleNotFoundError:
    InputDevice = None
    ecodes = None
    list_devices = None


@dataclass
class AxisState:
    value: int = 0
    minimum: int = -32768
    maximum: int = 32767

    def normalized(self, deadzone: float) -> float:
        center = (self.maximum + self.minimum) / 2.0
        half_span = max((self.maximum - self.minimum) / 2.0, 1.0)
        value = max(-1.0, min(1.0, (self.value - center) / half_span))
        return 0.0 if abs(value) < deadzone else value

    def normalized_around(self, center: int, deadzone: float) -> float:
        if self.value >= center:
            span = max(self.maximum - center, 1)
        else:
            span = max(center - self.minimum, 1)
        value = max(-1.0, min(1.0, (self.value - center) / span))
        return 0.0 if abs(value) < deadzone else value

    def trigger(self) -> float:
        span = max(self.maximum - self.minimum, 1)
        return max(0.0, min(1.0, (self.value - self.minimum) / span))


class KeyboardInput:
    def __init__(self, enabled: bool):
        self.enabled = enabled and sys.stdin.isatty()
        self.fd = sys.stdin.fileno() if self.enabled else None
        self._old_term = None
        self._until: dict[str, float] = {}

    def __enter__(self) -> "KeyboardInput":
        if self.enabled and self.fd is not None:
            self._old_term = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self.enabled and self.fd is not None and self._old_term is not None:
            termios.tcsetattr(self.fd, termios.TCSANOW, self._old_term)

    def poll(self, now: float) -> set[str]:
        if not self.enabled or self.fd is None:
            return set()
        pressed: set[str] = set()
        readable, _, _ = select.select([self.fd], [], [], 0.0)
        if self.fd in readable:
            chars = os.read(self.fd, 32)
            for b in chars:
                ch = chr(b)
                if ch in ("w", "a", "s", "d"):
                    self._until[ch] = now + 0.2
                elif ch == " ":
                    pressed.add("emergency_stop")
                elif ch in ("r", "R"):
                    pressed.add("gyro_reset")
                elif ch in ("\n", "\r"):
                    pressed.add("start")
                elif ch in ("c", "C"):
                    pressed.add("coast_stop")

        return pressed

    def throttle(self, now: float) -> float:
        up = self._until.get("w", 0.0) > now
        down = self._until.get("s", 0.0) > now
        return (1.0 if up else 0.0) - (1.0 if down else 0.0)

    def steering(self, now: float) -> float:
        left = self._until.get("a", 0.0) > now
        right = self._until.get("d", 0.0) > now
        return (1.0 if right else 0.0) - (1.0 if left else 0.0)


def find_controller(path: str | None) -> InputDevice | None:
    if InputDevice is None or list_devices is None:
        return None
    if path:
        return InputDevice(path)

    for dev_path in list_devices():
        dev = InputDevice(dev_path)
        name = dev.name.lower()
        if (
            any(token in name for token in ("dualsense", "wireless controller", "playstation", "ps5"))
            and "touchpad" not in name
            and "motion" not in name
        ):
            return dev
        dev.close()
    return None


def axis_ranges(device: InputDevice) -> dict[int, AxisState]:
    assert ecodes is not None
    ranges: dict[int, AxisState] = {}
    for code, info in device.capabilities(absinfo=True).get(ecodes.EV_ABS, []):
        ranges[code] = AxisState(value=info.value, minimum=info.min, maximum=info.max)
    return ranges
