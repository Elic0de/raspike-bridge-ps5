from __future__ import annotations

from dataclasses import dataclass


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def curve(value: float, exponent: float) -> float:
    if value == 0:
        return 0.0
    return (1.0 if value > 0 else -1.0) * (abs(value) ** exponent)


def slew(current: float, target: float, max_delta: float) -> float:
    delta = target - current
    if abs(delta) <= max_delta:
        return target
    return current + (max_delta if delta > 0 else -max_delta)


@dataclass
class MixerState:
    left: float = 0.0
    right: float = 0.0


class DriveMixer:
    def __init__(
        self,
        steering_curve: float,
        throttle_curve: float,
        steering_gain: float,
        low_speed_steer_gain: float,
        high_speed_steer_gain: float,
        slew_rate: float,
    ):
        self.steering_curve = steering_curve
        self.throttle_curve = throttle_curve
        self.steering_gain = steering_gain
        self.low_speed_steer_gain = low_speed_steer_gain
        self.high_speed_steer_gain = high_speed_steer_gain
        self.slew_rate = slew_rate
        self.state = MixerState()

    def mix(self, throttle_in: float, steering_in: float, power_limit: int, dt: float) -> tuple[int, int, float, float]:
        throttle = curve(clamp(throttle_in, -1.0, 1.0), self.throttle_curve)
        steering = curve(clamp(steering_in, -1.0, 1.0), self.steering_curve)

        speed_ratio = abs(throttle)
        dynamic_steer = self.low_speed_steer_gain + (self.high_speed_steer_gain - self.low_speed_steer_gain) * speed_ratio
        steer_scale = self.steering_gain * dynamic_steer

        target_left = clamp((throttle + steering * steer_scale) * power_limit, -power_limit, power_limit)
        target_right = clamp((throttle - steering * steer_scale) * power_limit, -power_limit, power_limit)

        max_delta = self.slew_rate * max(0.0, dt)
        self.state.left = slew(self.state.left, target_left, max_delta)
        self.state.right = slew(self.state.right, target_right, max_delta)

        left = int(round(clamp(self.state.left, -power_limit, power_limit)))
        right = int(round(clamp(self.state.right, -power_limit, power_limit)))
        return left, right, throttle, steering

    def reset(self) -> None:
        self.state.left = 0.0
        self.state.right = 0.0
