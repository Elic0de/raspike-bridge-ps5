from __future__ import annotations

import os
import select
import sys
import termios
import time
import tty
from dataclasses import dataclass

from .config import ControllerConfig
from .input_mapper import AxisState, axis_ranges, ecodes, find_controller


@dataclass
class ProviderState:
    throttle: float = 0.0
    steering: float = 0.0


class KeyboardProvider:
    def __init__(self, enabled: bool, cfg: ControllerConfig):
        self.enabled = enabled and sys.stdin.isatty()
        self.fd = sys.stdin.fileno() if self.enabled else None
        self._old_term = None
        self._until: dict[str, float] = {}
        self._pressed_at: dict[str, float] = {}
        self._hold_fired: set[tuple[str, str]] = set()
        self._active_prev: set[str] = set()
        self._last_state_at: float | None = None
        self._steering = 0.0
        self.cfg = cfg

    @staticmethod
    def _decode_key(byte: int) -> str:
        if byte in (10, 13):
            return "enter"
        if byte == 32:
            return "space"
        return chr(byte).lower()

    def __enter__(self):
        if self.enabled and self.fd is not None:
            self._old_term = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc, tb):
        if self.enabled and self.fd is not None and self._old_term is not None:
            termios.tcsetattr(self.fd, termios.TCSANOW, self._old_term)

    def poll_actions(self, now: float) -> set[str]:
        if not self.enabled or self.fd is None:
            return set()
        actions: set[str] = set()
        readable, _, _ = select.select([self.fd], [], [], 0.0)
        if self.fd in readable:
            chars = os.read(self.fd, 32)
            for b in chars:
                ch = self._decode_key(b)
                self._until[ch] = now + 0.2
                if ch not in self._pressed_at:
                    self._pressed_at[ch] = now

        active = {k for k, until in self._until.items() if until > now}
        for key_name, binding in self.cfg.bindings.buttons.items():
            key = key_name.lower()
            if key in active and key not in self._active_prev and binding.press:
                actions.add(binding.press)
            if binding.hold and key in active:
                start_at = self._pressed_at.get(key, now)
                elapsed_ms = int((now - start_at) * 1000)
                token = (key, binding.hold.action)
                if elapsed_ms >= binding.hold.duration_ms and token not in self._hold_fired:
                    actions.add(binding.hold.action)
                    self._hold_fired.add(token)
            elif binding.hold:
                self._hold_fired.discard((key, binding.hold.action))

        for combo in self.cfg.bindings.combos:
            if combo.keys and combo.action and all(k.lower() in active for k in combo.keys):
                actions.add(combo.action)

        released = self._active_prev - active
        for key in released:
            self._pressed_at.pop(key, None)
        self._active_prev = active
        return actions

    def state(self, now: float) -> ProviderState:
        up = self._until.get("w", 0.0) > now
        down = self._until.get("s", 0.0) > now
        left = self._until.get("a", 0.0) > now
        right = self._until.get("d", 0.0) > now
        steering_target = (1.0 if right else 0.0) - (1.0 if left else 0.0)
        dt = 0.0 if self._last_state_at is None else max(0.0, now - self._last_state_at)
        self._last_state_at = now

        if steering_target:
            self._steering = steering_target
        else:
            step = self.cfg.input.steering_return_speed * dt
            if abs(self._steering) <= step:
                self._steering = 0.0
            else:
                self._steering += -step if self._steering > 0 else step

        return ProviderState(
            throttle=(1.0 if up else 0.0) - (1.0 if down else 0.0),
            steering=self._steering,
        )


class GamepadProvider:
    def __init__(self, event_device: str | None, cfg: ControllerConfig):
        self.event_device = event_device
        self.cfg = cfg
        self.controller = None
        self.axes: dict[int, AxisState] = {}
        self._pressed_at: dict[int, float] = {}
        self._hold_fired: set[tuple[int, str]] = set()

    def ensure_connected(self) -> bool:
        if self.controller is not None:
            return True
        self.controller = find_controller(self.event_device)
        if self.controller is None:
            return False
        self.axes = axis_ranges(self.controller)
        return True

    def poll_actions(self, now: float) -> set[str]:
        actions: set[str] = set()
        if self.controller is None:
            return actions
        try:
            readable, _, _ = select.select([self.controller.fd], [], [], 0.0)
            if self.controller.fd not in readable:
                return actions
            for event in self.controller.read():
                if event.type == ecodes.EV_ABS and event.code in self.axes:
                    self.axes[event.code].value = event.value
                elif event.type == ecodes.EV_KEY:
                    if event.value == 1:
                        self._pressed_at[event.code] = now
                        for k, b in self.cfg.bindings.buttons.items():
                            if event.code == getattr(ecodes, k, -1) and b.press:
                                actions.add(b.press)
                    elif event.value == 0:
                        self._pressed_at.pop(event.code, None)

            for key_name, b in self.cfg.bindings.buttons.items():
                code = getattr(ecodes, key_name, -1)
                if b.hold and code in self._pressed_at:
                    elapsed_ms = int((now - self._pressed_at[code]) * 1000)
                    token = (code, b.hold.action)
                    if elapsed_ms >= b.hold.duration_ms and token not in self._hold_fired:
                        actions.add(b.hold.action)
                        self._hold_fired.add(token)
                elif b.hold:
                    self._hold_fired.discard((code, b.hold.action))

            for combo in self.cfg.bindings.combos:
                if not combo.keys or not combo.action:
                    continue
                if all(getattr(ecodes, k, -1) in self._pressed_at for k in combo.keys):
                    actions.add(combo.action)
        except OSError:
            if self.controller is not None:
                try:
                    self.controller.close()
                except Exception:
                    pass
            self.controller = None
            self.axes = {}
            self._pressed_at.clear()
            self._hold_fired.clear()
        return actions

    def state(self) -> ProviderState:
        if self.controller is None:
            return ProviderState()
        axis_x = self.axes.get(ecodes.ABS_X, AxisState())
        steering = axis_x.normalized_around(127, self.cfg.input.deadzone)
        accelerator = self.axes.get(ecodes.ABS_RZ, AxisState(minimum=0, maximum=255)).trigger()
        brake = self.axes.get(ecodes.ABS_Z, AxisState(minimum=0, maximum=255)).trigger()
        return ProviderState(throttle=accelerator - brake, steering=steering)
