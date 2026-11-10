"""Four-wheel force evaluation: wheel kinematics, slip angles, tyre forces, body forces/moments and resistances."""

from __future__ import annotations
from math import cos
from math import sin
from math import atan2
from math import pi
from torquevector.domain.tyre import TyreParameters
from torquevector.domain.tyre import TyreResult
from torquevector.physics.tyre_magic_formula import tyre_forces
from typing import Callable
from torquevector.domain.controls import DriverControls
from torquevector.domain.states import VehicleForces
from torquevector.domain.vehicle import VehicleParameters
from torquevector.domain.states import VehicleState
from torquevector.domain.wheels import WHEEL_ORDER
from torquevector.physics.load_transfer import wheel_loads


def rotate_wheel_to_body(fx: float, fy: float, delta: float) -> tuple[float, float]:
    return fx * cos(delta) - fy * sin(delta), fx * sin(delta) + fy * cos(delta)


def evaluate(
    state: VehicleState,
    control: DriverControls,
    car: VehicleParameters,
    tyre: TyreParameters,
    t_s: float,
    load_guess: tuple[float, float] = (0.0, 0.0),
    tyre_model: Callable | None = None,
    steer_angle_deg_override: float | None = None,
) -> VehicleForces:
    loads = wheel_loads(car, *load_guess)
    contacts = car.contact_points()
    # A smooth steering input avoids an instantaneous yaw acceleration jump.
    steer_ramp = min(1.0, max(0.0, t_s / 0.55))
    delta_front = (
        (control.steer_deg * steer_ramp if steer_angle_deg_override is None else steer_angle_deg_override) * pi / 180.0
    )
    fx_sum = fy_sum = mz_sum = 0.0
    wheel_results: dict[str, TyreResult] = {}
    body_forces: dict[str, tuple[float, float]] = {}
    for index, name in enumerate(WHEEL_ORDER):
        x, y = contacts[name]
        vel_x = state.vx_ms - state.yaw_rate_rads * y
        vel_y = state.vy_ms + state.yaw_rate_rads * x
        delta = delta_front if name.startswith('F') else 0.0
        vx_w = cos(delta) * vel_x + sin(delta) * vel_y
        vy_w = -sin(delta) * vel_x + cos(delta) * vel_y
        # This simple model is meant for forward motion, not low-speed parking.
        slip_angle = atan2(vy_w, max(2.0, vx_w))
        if tyre_model is not None and hasattr(tyre_model, 'at_wheel'):
            result = tyre_model.at_wheel(name, t_s, control.slip_ratios[index], slip_angle, loads[name], tyre)
        else:
            result = (tyre_forces if tyre_model is None else tyre_model)(
                control.slip_ratios[index], slip_angle, loads[name], tyre
            )
        fx, fy = rotate_wheel_to_body(result.fx_n, result.fy_n, delta)
        wheel_results[name] = result
        body_forces[name] = (fx, fy)
        fx_sum += fx
        fy_sum += fy
        mz_sum += x * fy - y * fx

    drag_n = 0.5 * car.air_density_kgm3 * car.cd_area_m2 * state.vx_ms * abs(state.vx_ms)
    rolling_n = car.rolling_coefficient * car.mass_kg * car.gravity
    resistance_n = drag_n + rolling_n
    return VehicleForces(
        wheel_results,
        body_forces,
        fx_sum,
        fy_sum,
        mz_sum,
        (fx_sum - resistance_n) / car.mass_kg,
        fy_sum / car.mass_kg,
        resistance_n,
    )


def advance(state: VehicleState, forces: VehicleForces, car: VehicleParameters, dt: float) -> VehicleState:
    """Semi-implicit step; dt <= 0.02 s recommended for this demonstrator."""
    if not 0 < dt <= 0.05:
        raise ValueError('dt must be in (0, 0.05] seconds')
    du = forces.ax_ms2 + state.yaw_rate_rads * state.vy_ms
    dv = forces.ay_ms2 - state.yaw_rate_rads * state.vx_ms
    dr = forces.mz_nm / car.iz_kgm2
    vx = max(0.0, state.vx_ms + dt * du)
    vy = state.vy_ms + dt * dv
    yaw_rate = state.yaw_rate_rads + dt * dr
    yaw = state.yaw_rad + dt * yaw_rate
    x = state.x_m + dt * (vx * cos(yaw) - vy * sin(yaw))
    y = state.y_m + dt * (vx * sin(yaw) + vy * cos(yaw))
    return VehicleState(x, y, yaw, vx, vy, yaw_rate)
