"""LQR yaw-rate/side-slip controller on the linear single-track model, gain scheduled over a speed grid."""

from __future__ import annotations
from dataclasses import dataclass
from math import isfinite
from functools import lru_cache
import numpy as np
from scipy.linalg import solve_continuous_are
from torquevector.domain.vehicle import VehicleParameters
from torquevector.prediction.bicycle_linear import bicycle_lateral_matrices
from math import atan2
from torquevector.domain.states import VehicleState
from torquevector.domain.tyre import TyreParameters
from torquevector.domain.controls import YawReference
from torquevector.prediction.bicycle_linear import _axle_cornering_stiffness


@dataclass(frozen=True)
class LQRConfig:
    sideslip_scale_rad: float = 0.05
    yaw_rate_scale_rads: float = 0.08
    moment_scale_nm: float = 1500.0
    min_speed_ms: float = 5.0
    max_speed_ms: float = 45.0
    speed_grid_ms: float = 0.5
    feedforward_weight: float = 0.0

    def __post_init__(self):
        values = (
            self.sideslip_scale_rad,
            self.yaw_rate_scale_rads,
            self.moment_scale_nm,
            self.min_speed_ms,
            self.max_speed_ms,
            self.speed_grid_ms,
            self.feedforward_weight,
        )
        if not all(isfinite(v) for v in values):
            raise ValueError('LQR parameters must be finite')
        if (
            not 0.005 <= self.sideslip_scale_rad <= 0.5
            or not 0.005 <= self.yaw_rate_scale_rads <= 1.0
            or not 50 <= self.moment_scale_nm <= 10000
            or not 3 <= self.min_speed_ms < self.max_speed_ms <= 80
            or not 0.1 <= self.speed_grid_ms <= 5
            or not 0 <= self.feedforward_weight <= 1
        ):
            raise ValueError('Invalid LQR tuning or speed domain')


@lru_cache(maxsize=512)
def _lqr_gain(speed_ms: float, car: VehicleParameters, cf_n_rad: float, cr_n_rad: float, tuning: LQRConfig):
    aa, bb, _ = bicycle_lateral_matrices(speed_ms, car, cf_n_rad, cr_n_rad)
    qq = np.diag([1 / tuning.sideslip_scale_rad**2, 1 / tuning.yaw_rate_scale_rads**2])
    rr = np.array([[(1000.0 / tuning.moment_scale_nm) ** 2]])
    pp = solve_continuous_are(aa, bb, qq, rr)
    k = np.linalg.solve(rr, bb.T @ pp).reshape(2)
    if not np.all(np.isfinite(k)) or np.max(np.real(np.linalg.eigvals(aa - bb @ k[None, :]))) >= 0:
        raise ValueError('Invalid or unstable LQR gain')
    return tuple(float(z) for z in k)


@dataclass(frozen=True)
class LQRYawSideslip:
    car: VehicleParameters
    tyre: TyreParameters
    tuning: LQRConfig = LQRConfig()

    def demand(self, state: VehicleState, reference: YawReference, steering_rad: float) -> float:
        v = min(self.tuning.max_speed_ms, max(self.tuning.min_speed_ms, state.vx_ms))
        v = round(v / self.tuning.speed_grid_ms) * self.tuning.speed_grid_ms
        cf, cr = _axle_cornering_stiffness(self.car, self.tyre)
        k_beta, k_yaw = _lqr_gain(v, self.car, cf, cr, self.tuning)
        beta = atan2(state.vy_ms, max(0.1, state.vx_ms))
        # K returns kNm/rad. Feedback sign follows x_error = x - x_ref.
        demand_k_nm = -k_beta * (beta - reference.sideslip_rad) - k_yaw * (
            state.yaw_rate_rads - reference.yaw_rate_rads
        )
        if self.tuning.feedforward_weight:
            aa, _, b_delta = bicycle_lateral_matrices(v, self.car, cf, cr)
            desired = np.array([reference.sideslip_rad, reference.yaw_rate_rads])
            feedforward_nm = -self.car.iz_kgm2 * (aa[1] @ desired + b_delta[1] * steering_rad)
            demand_k_nm += self.tuning.feedforward_weight * feedforward_nm / 1000.0
        return 1000 * demand_k_nm
