"""Wheel parameters (effective radius, inertia, envelope limits) and the canonical wheel order FL, FR, RL, RR."""

from __future__ import annotations
from dataclasses import dataclass
from math import isfinite


@dataclass(frozen=True)
class WheelParameters:
    radius_m: float = 0.315
    inertia_kgm2: float = 1.45
    min_contact_speed_ms: float = 3.0
    max_abs_slip: float = 0.5

    def __post_init__(self):
        vals = (self.radius_m, self.inertia_kgm2, self.min_contact_speed_ms, self.max_abs_slip)
        if not all(isfinite(v) for v in vals) or not (
            0.15 <= self.radius_m <= 0.6
            and self.inertia_kgm2 > 0
            and self.min_contact_speed_ms >= 2
            and 0 < self.max_abs_slip <= 1
        ):
            raise ValueError('Invalid wheel parameters')


WHEEL_ORDER = ('FL', 'FR', 'RL', 'RR')
