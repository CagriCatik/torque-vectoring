"""Reduced-order nonlinear single-track model with tanh axle saturation and RK4 step (NMPC predictor)."""

from __future__ import annotations
from dataclasses import dataclass
from math import isfinite
from torquevector.domain.vehicle import VehicleParameters
from torquevector.domain.tyre import TyreParameters
from torquevector.prediction.bicycle_linear import _axle_cornering_stiffness
from typing import Any


@dataclass(frozen=True)
class ReducedOrderTyre:
    cf_n_rad: float
    cr_n_rad: float
    front_limit_n: float
    rear_limit_n: float

    def __post_init__(self):
        if any(
            (not isfinite(v) or v <= 0) for v in (self.cf_n_rad, self.cr_n_rad, self.front_limit_n, self.rear_limit_n)
        ):
            raise ValueError('Invalid nominal tyre properties')


def tyre_from_nominal(car: VehicleParameters, tyre: TyreParameters, mu: float | None = None) -> ReducedOrderTyre:
    if mu is None:
        mu = tyre.mu
    if not isfinite(mu) or mu <= 0:
        raise ValueError('Positive friction required')
    cf, cr = _axle_cornering_stiffness(car, tyre)
    a = car.cg_to_front_m
    b = car.wheelbase_m - a
    return ReducedOrderTyre(
        cf,
        cr,
        mu * car.mass_kg * car.gravity * b / car.wheelbase_m,
        mu * car.mass_kg * car.gravity * a / car.wheelbase_m,
    )


def bicycle_derivative(
    beta: Any,
    yaw_rate: Any,
    moment_knm: Any,
    speed_ms: Any,
    steer_rad: Any,
    car: VehicleParameters,
    tyres: ReducedOrderTyre,
    ca: Any,
):
    """Return [beta_dot, yaw_rate_dot] with smooth nonlinear lateral tyre forces.

    Forward-speed-only; constant v_x over one integration stage, no wheel spin.
    State sideslip approximates vy=vx*tan(beta). u=additional yaw moment [kNm].
    """
    vx = speed_ms
    a = car.cg_to_front_m
    b = car.wheelbase_m - a
    vy = vx * ca.tan(beta)
    af = steer_rad - ca.atan((vy + a * yaw_rate) / vx)
    ar = -ca.atan((vy - b * yaw_rate) / vx)
    fy_f = tyres.front_limit_n * ca.tanh(tyres.cf_n_rad * af / tyres.front_limit_n)
    fy_r = tyres.rear_limit_n * ca.tanh(tyres.cr_n_rad * ar / tyres.rear_limit_n)
    front = fy_f * ca.cos(steer_rad)
    vy_dot = (front + fy_r) / car.mass_kg - vx * yaw_rate
    beta_dot = vy_dot / (vx * (1 + ca.tan(beta) ** 2))
    yaw_dot = (a * front - b * fy_r + 1000 * moment_knm) / car.iz_kgm2
    return ca.vertcat(beta_dot, yaw_dot)


def rk4_step(x: Any, u: Any, vx: Any, delta: Any, dt: float, car: VehicleParameters, tyres: ReducedOrderTyre, ca: Any):
    def fn(z):
        return bicycle_derivative(z[0], z[1], u, vx, delta, car, tyres, ca)

    k1 = fn(x)
    k2 = fn(x + dt / 2 * k1)
    k3 = fn(x + dt / 2 * k2)
    k4 = fn(x + dt * k3)
    return x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


@dataclass(frozen=True)
class NonlinearBicyclePrediction:
    """PredictionModel adapter for the unchanged symbolic RK4 equations."""

    car: VehicleParameters
    tyres: ReducedOrderTyre
    backend: Any

    def step(self, state, moment_knm, speed_ms, steering_rad, period_s):
        return rk4_step(state, moment_knm, speed_ms, steering_rad, period_s, self.car, self.tyres, self.backend)
