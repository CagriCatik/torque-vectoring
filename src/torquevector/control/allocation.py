"""Bound-constrained linearized torque allocation, engineering prototype.

QP surrogate: minimize yaw-moment tracking, total-drive-torque deviation and
regularization. Fx ~= T/R is only a *local approximation*; the nonlinear
four-wheel simulation determines the actual realized tire forces.
"""

from __future__ import annotations
from dataclasses import dataclass
from math import isfinite, radians, tan
import numpy as np
from scipy.optimize import lsq_linear
from torquevector.domain.vehicle import VehicleParameters
from torquevector.domain.states import VehicleState
from torquevector.domain.wheels import WHEEL_ORDER
from torquevector.domain.controls import TorqueControls
from torquevector.domain.wheels import WheelParameters


@dataclass(frozen=True)
class AllocationOptions:
    yaw_weight: float = 1.0
    total_weight: float = 2.0
    intervention_penalty: float = 0.45
    max_delta_nm: float = 130.0
    mu_reference: float = 0.95
    yaw_kp_nm_per_rads: float = 2300.0

    def __post_init__(self):
        vals = (
            self.yaw_weight,
            self.total_weight,
            self.intervention_penalty,
            self.max_delta_nm,
            self.mu_reference,
            self.yaw_kp_nm_per_rads,
        )
        if not all(isfinite(z) and z > 0 for z in vals):
            raise ValueError('Invalid allocation options')


@dataclass(frozen=True)
class AllocationResult:
    torques_nm: tuple[float, float, float, float]
    requested_yaw_nm: float
    estimated_yaw_nm: float
    torque_sum_delta_nm: float
    yaw_residual_nm: float
    bound_saturated: bool


def allocate_yaw(
    base: tuple[float, float, float, float],
    requested_yaw_nm: float,
    car: VehicleParameters,
    wheel: WheelParameters,
    torque_limit_nm: float,
    options: AllocationOptions = AllocationOptions(),
):
    if len(base) != 4 or not all(isfinite(z) for z in base) or not isfinite(requested_yaw_nm):
        raise ValueError('Invalid torques or requested moment')
    # The longitudinal force lever arm -y/R generates yaw moment about the CG.
    lever = np.array([-car.contact_points()[n][1] / wheel.radius_m for n in WHEEL_ORDER])
    b = np.array(base, float)
    lower = np.maximum(-b, -options.max_delta_nm * np.ones(4))
    upper = np.minimum(torque_limit_nm - b, options.max_delta_nm * np.ones(4))
    if np.any(lower > upper):
        raise ValueError('Base torques outside admissible envelope')
    A = np.vstack(
        (options.yaw_weight * lever, options.total_weight * np.ones(4), options.intervention_penalty * np.eye(4))
    )
    target = np.concatenate(([options.yaw_weight * requested_yaw_nm, 0.0], np.zeros(4)))
    # lsq_linear requires strictly distinct bounds. Fix pinned wheels explicitly.
    pinned = (upper - lower) < 1e-10
    increments = lower.copy()
    if np.any(~pinned):
        reduced = A[:, ~pinned]
        residual_target = target - A[:, pinned] @ increments[pinned]
        solution = lsq_linear(
            reduced, residual_target, bounds=(lower[~pinned], upper[~pinned]), method='bvls', tol=1e-10, max_iter=100
        )
        if not solution.success:
            raise RuntimeError('Torque allocator did not converge')
        increments[~pinned] = solution.x
    actual = b + increments
    realized = float(lever @ increments)
    saturated = bool(np.any(np.abs(increments - lower) < 1e-5) | np.any(np.abs(increments - upper) < 1e-5))
    return AllocationResult(
        tuple(map(float, actual)),
        float(requested_yaw_nm),
        realized,
        float(sum(increments)),
        float(requested_yaw_nm - realized),
        saturated,
    )
