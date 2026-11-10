"""Linear single-track (bicycle) model matrices and axle cornering stiffnesses for LQR/LTV-MPC."""

from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from scipy.linalg import expm
from torquevector.domain.vehicle import VehicleParameters
from torquevector.domain.tyre import TyreParameters
from torquevector.prediction.interfaces import DiscreteModel
from torquevector.physics.load_transfer import wheel_loads
from torquevector.physics.tyre_magic_formula import peak_grip


@dataclass(frozen=True)
class BicycleLinearizer:
    car: VehicleParameters
    tyre: TyreParameters

    def step(self, state, moment_knm, speed_ms, steering_rad, period_s):
        model = self.discrete(speed_ms, period_s)
        return model.a @ state + model.b * moment_knm + model.steering * steering_rad

    def discrete(self, speed_ms: float, period_s: float) -> DiscreteModel:
        cf, cr = _axle_cornering_stiffness(self.car, self.tyre)
        a, b, steer = bicycle_lateral_matrices(float(speed_ms), self.car, cf, cr)
        block = np.zeros((4, 4))
        block[:2, :2] = a
        block[:2, 2:3] = b
        block[:2, 3] = steer
        z = expm(block * period_s)
        return DiscreteModel(z[:2, :2], z[:2, 2], z[:2, 3])


def _axle_cornering_stiffness(car: VehicleParameters, tyre: TyreParameters) -> tuple[float, float]:
    """Small-signal axle stiffness of the fitted pure-slip MF at static load.

    dFy/dalpha = -D B C at alpha=0; *not* calibrated at combined slip.
    """
    loads = wheel_loads(car)
    cf = sum(peak_grip(loads[n], tyre) * tyre.lateral.b * tyre.lateral.c for n in ('FL', 'FR'))
    cr = sum(peak_grip(loads[n], tyre) * tyre.lateral.b * tyre.lateral.c for n in ('RL', 'RR'))
    return cf, cr


def bicycle_lateral_matrices(speed_ms: float, car: VehicleParameters, cf_n_rad: float, cr_n_rad: float):
    """State x=[beta, yaw_rate], input u=incremental yaw moment in kNm.

    Derivation assumes constant forward velocity, small slip, and rear fixed steer.
    """
    if not 3.0 <= speed_ms <= 80.0 or cf_n_rad <= 0 or cr_n_rad <= 0:
        raise ValueError('Bicycle linearization outside valid envelope')
    m, iz, a = car.mass_kg, car.iz_kgm2, car.cg_to_front_m
    b = car.wheelbase_m - a
    u = speed_ms
    aa = np.array(
        [
            [-(cf_n_rad + cr_n_rad) / (m * u), (b * cr_n_rad - a * cf_n_rad) / (m * u * u) - 1.0],
            [(b * cr_n_rad - a * cf_n_rad) / iz, -(a * a * cf_n_rad + b * b * cr_n_rad) / (iz * u)],
        ],
        dtype=float,
    )
    bb = np.array([[0.0], [1000.0 / iz]], dtype=float)
    b_delta = np.array([cf_n_rad / (m * u), a * cf_n_rad / iz], dtype=float)
    return aa, bb, b_delta
