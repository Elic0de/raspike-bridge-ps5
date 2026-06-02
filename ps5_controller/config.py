from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml


@dataclass
class ComboBinding:
    keys: list[str] = field(default_factory=list)
    action: str = ""


@dataclass
class HoldBinding:
    duration_ms: int = 1500
    action: str = ""


@dataclass
class ButtonBinding:
    press: str | None = None
    hold: HoldBinding | None = None


@dataclass
class BindingsConfig:
    combos: list[ComboBinding] = field(default_factory=list)
    buttons: dict[str, ButtonBinding] = field(default_factory=dict)


@dataclass
class InputConfig:
    deadzone: float = 0.06
    steering_curve: float = 2.1
    throttle_curve: float = 1.7
    steering_gain: float = 1.35
    steering_return_speed: float = 4.5
    low_speed_steer_gain: float = 1.35
    high_speed_steer_gain: float = 0.75


@dataclass
class PowerConfig:
    slew_rate: float = 180.0


@dataclass
class ControllerConfig:
    bindings: BindingsConfig = field(default_factory=BindingsConfig)
    input: InputConfig = field(default_factory=InputConfig)
    power: PowerConfig = field(default_factory=PowerConfig)


def _parse_bindings(raw: dict[str, Any]) -> BindingsConfig:
    cfg = BindingsConfig()
    combos = raw.get("combos") or []
    for item in combos:
        if isinstance(item, dict):
            cfg.combos.append(ComboBinding(keys=list(item.get("keys") or []), action=str(item.get("action") or "")))

    for key, spec in raw.items():
        if key == "combos" or not isinstance(spec, dict):
            continue
        press = None
        hold = None
        p = spec.get("press")
        if isinstance(p, dict) and p.get("action"):
            press = str(p["action"])
        h = spec.get("hold")
        if isinstance(h, dict) and h.get("action"):
            hold = HoldBinding(duration_ms=int(h.get("duration_ms", 1500)), action=str(h["action"]))
        cfg.buttons[key] = ButtonBinding(press=press, hold=hold)

    if not cfg.buttons:
        # Backward-compatible defaults
        cfg.buttons = {
            "BTN_SOUTH": ButtonBinding(press="emergency_stop"),
            "BTN_NORTH": ButtonBinding(press="gyro_reset"),
            "BTN_START": ButtonBinding(press="start"),
            "BTN_EAST": ButtonBinding(press="coast_stop"),
            "space": ButtonBinding(press="emergency_stop"),
            "r": ButtonBinding(press="gyro_reset"),
            "enter": ButtonBinding(press="start"),
            "c": ButtonBinding(press="coast_stop"),
        }
    return cfg


def _merge_dataclass(dc: Any, data: dict[str, Any]) -> Any:
    for key, value in data.items():
        if hasattr(dc, key):
            setattr(dc, key, value)
    return dc


def load_config(path: str | None) -> ControllerConfig:
    cfg = ControllerConfig()
    if not path:
        return cfg
    p = Path(path)
    if not p.exists():
        return cfg

    raw = yaml.safe_load(p.read_text(encoding="utf-8")) or {}
    if not isinstance(raw, dict):
        return cfg

    if isinstance(raw.get("bindings"), dict):
        cfg.bindings = _parse_bindings(raw["bindings"])
    if isinstance(raw.get("input"), dict):
        _merge_dataclass(cfg.input, raw["input"])
    if isinstance(raw.get("power"), dict):
        _merge_dataclass(cfg.power, raw["power"])
    return cfg
