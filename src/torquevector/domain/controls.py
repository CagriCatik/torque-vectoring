"""Control inputs: driver controls, per-wheel drive/brake torque commands, yaw references and policy types."""

from __future__ import annotations
from dataclasses import dataclass
from math import isfinite
from typing import Callable
from torquevector.domain.states import VehicleState


@dataclass(frozen=True)
class YawReference:
    """Desired yaw-rate, and zero/desired sideslip, SI units."""

    yaw_rate_rads: float
    sideslip_rad: float = 0.0

    def __post_init__(self):
        if not all(isfinite(z) for z in (self.yaw_rate_rads, self.sideslip_rad)):
            raise ValueError('Reference must be finite')


@dataclass(frozen=True)
class TorqueControls:
    steer_deg: float = 0.0
    drive_nm: tuple[float, float, float, float] = (0.0, 0.0, 80.0, 80.0)
    brake_nm: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    torque_limit_nm: float = 800.0

    def __post_init__(self):
        if not isfinite(self.steer_deg) or abs(self.steer_deg) > 25:
            raise ValueError('Invalid steer angle')
        if not isfinite(self.torque_limit_nm) or self.torque_limit_nm <= 0:
            raise ValueError('Invalid torque limit')
        for key, values in [('drive', self.drive_nm), ('brake', self.brake_nm)]:
            if len(values) != 4 or any(not isfinite(x) for x in values):
                raise ValueError(f'{key}: four finite values needed')
            if any(abs(x) > self.torque_limit_nm for x in values):
                raise ValueError(f'{key}: torque exceeds actuator limit')
        if any(x < 0 for x in self.brake_nm):
            raise ValueError('Brake torques must be nonnegative')


TorquePolicy = Callable[[float, VehicleState, TorqueControls], TorqueControls]


def active_control(t: float, s: VehicleState, base: TorqueControls, policy: TorquePolicy | None) -> TorqueControls:
    result = base if policy is None else policy(t, s, base)
    if not isinstance(result, TorqueControls):
        raise TypeError('Torque policy must return validated TorqueControls')
    return result


@dataclass(frozen=True)
class DriverControls:
    steer_deg: float = 2.0
    slip_ratios: tuple[float, float, float, float] = (0.0, 0.0, 0.01, 0.025)

    def __post_init__(self):
        if not isfinite(self.steer_deg) or abs(self.steer_deg) > 45:
            raise ValueError('Steer angle must be within +/-45 deg')
        if len(self.slip_ratios) != 4 or any(not isfinite(x) or abs(x) > 0.5 for x in self.slip_ratios):
            raise ValueError('Four finite slip ratios within +/-0.5 are required')
