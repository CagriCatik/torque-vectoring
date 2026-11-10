"""Vehicle parameters (mass, inertia, geometry, resistances) and wheel contact point geometry."""

from __future__ import annotations
from dataclasses import dataclass
from math import isfinite


@dataclass(frozen=True)
class VehicleParameters:
    mass_kg: float = 1600.0
    iz_kgm2: float = 2600.0
    wheelbase_m: float = 2.76
    cg_to_front_m: float = 1.18
    track_m: float = 1.62
    cg_height_m: float = 0.52
    cd_area_m2: float = 0.65
    rolling_coefficient: float = 0.012
    air_density_kgm3: float = 1.225
    gravity: float = 9.81
    load_transfer: bool = True

    def __post_init__(self):
        values = (
            self.mass_kg,
            self.iz_kgm2,
            self.wheelbase_m,
            self.cg_to_front_m,
            self.track_m,
            self.cg_height_m,
            self.cd_area_m2,
            self.rolling_coefficient,
            self.air_density_kgm3,
            self.gravity,
        )
        if not all(isfinite(value) for value in values):
            raise ValueError('Vehicle parameters must be finite')
        if not (
            self.mass_kg > 0
            and self.iz_kgm2 > 0
            and self.wheelbase_m > 0
            and 0 < self.cg_to_front_m < self.wheelbase_m
            and self.track_m > 0
            and self.cg_height_m >= 0
            and self.cd_area_m2 >= 0
            and self.rolling_coefficient >= 0
            and self.air_density_kgm3 > 0
            and self.gravity > 0
        ):
            raise ValueError('Invalid vehicle geometry or constants')

    def contact_points(self) -> dict[str, tuple[float, float]]:
        a = self.cg_to_front_m
        b = self.wheelbase_m - a
        half_t = self.track_m / 2
        return {'FL': (a, half_t), 'FR': (a, -half_t), 'RL': (-b, half_t), 'RR': (-b, -half_t)}
