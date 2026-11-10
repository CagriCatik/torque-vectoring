"""Proportional yaw-rate controller: additional yaw moment = K_p * (r_ref - r)."""

from __future__ import annotations
from dataclasses import dataclass
from math import isfinite
from torquevector.domain.states import VehicleState
from torquevector.domain.controls import YawReference


@dataclass(frozen=True)
class ProportionalYaw:
    gain_nm_per_rads: float = 2300.0

    def __post_init__(self):
        if not isfinite(self.gain_nm_per_rads) or self.gain_nm_per_rads <= 0:
            raise ValueError('Invalid proportional gain')

    def demand(self, state: VehicleState, reference: YawReference, steering_rad: float) -> float:
        return self.gain_nm_per_rads * (reference.yaw_rate_rads - state.yaw_rate_rads)
