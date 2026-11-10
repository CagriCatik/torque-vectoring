"""Steering actuator: first-order lag with rate and angle limits, exact solution over a held command."""

from __future__ import annotations
from dataclasses import dataclass
from math import isfinite


@dataclass(frozen=True)
class SteeringActuator:
    time_constant_s: float = 0.12
    rate_limit_deg_s: float = 120.0
    max_angle_deg: float = 25.0

    def __post_init__(self):
        if not all(isfinite(x) and x > 0 for x in (self.time_constant_s, self.rate_limit_deg_s, self.max_angle_deg)):
            raise ValueError('Invalid steering actuator parameters')

    def derivative(self, command_deg: float, actual_deg: float) -> float:
        if not (isfinite(command_deg) and isfinite(actual_deg)):
            raise ValueError('Nonfinite actuator input')
        command = max(-self.max_angle_deg, min(self.max_angle_deg, command_deg))
        raw = (command - actual_deg) / self.time_constant_s
        return max(-self.rate_limit_deg_s, min(self.rate_limit_deg_s, raw))

    def predict_hold(self, command_deg: float, actual_deg: float, duration_s: float) -> tuple[float, float]:
        """Exact end and mean angle [deg] for a held command (Q07/Q09).

        Integrates the same first-order/rate-limited law as derivative(),
        including a possible switch from constant rate to exponential decay.
        """
        from math import exp

        if not isfinite(duration_s) or duration_s <= 0:
            raise ValueError('Positive finite prediction duration required')
        self.derivative(command_deg, actual_deg)  # finite input validation
        command = max(-self.max_angle_deg, min(self.max_angle_deg, command_deg))
        error = command - actual_deg
        sign = 1.0 if error >= 0 else -1.0
        magnitude = abs(error)
        threshold = self.rate_limit_deg_s * self.time_constant_s
        ramp_s = max(0.0, (magnitude - threshold) / self.rate_limit_deg_s)
        ramp = min(duration_s, ramp_s)
        at_switch = actual_deg + sign * self.rate_limit_deg_s * ramp
        integral = actual_deg * ramp + sign * self.rate_limit_deg_s * ramp * ramp / 2
        remainder = duration_s - ramp
        decay = exp(-remainder / self.time_constant_s)
        end = command + (at_switch - command) * decay
        integral += command * remainder + (at_switch - command) * self.time_constant_s * (1 - decay)
        return end, integral / duration_s
