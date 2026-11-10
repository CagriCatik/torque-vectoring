"""Wheel rotation dynamics (I_w*dw/dt = T_drive - T_brake - R*Fx), slip computation and the ODE right-hand side."""

from __future__ import annotations
from math import radians
from torquevector.domain.controls import TorqueControls
from math import sin
from math import cos
from torquevector.domain.vehicle import VehicleParameters
from torquevector.domain.states import VehicleState
from torquevector.domain.wheels import WHEEL_ORDER
from math import isfinite
from torquevector.domain.errors import OutsideEnvelopeError
from torquevector.domain.wheels import WheelParameters
from typing import Callable
from scipy.optimize import root
from torquevector.domain.states import VehicleForces
from torquevector.domain.controls import DriverControls
from torquevector.physics.four_wheel import evaluate
from torquevector.domain.tyre import TyreParameters
from torquevector.physics.steering import SteeringActuator
import numpy as np
from torquevector.domain.controls import TorquePolicy
from torquevector.domain.controls import active_control
from torquevector.domain.states import decode


def steering(t: float, controls: TorqueControls) -> float:
    return radians(controls.steer_deg) * min(1.0, max(0.0, t / 0.55))


def wheel_longitudinal_velocities(
    s: VehicleState, t: float, car: VehicleParameters, control: TorqueControls, steer_actual_deg: float | None = None
) -> tuple[float, ...]:
    delta = steering(t, control) if steer_actual_deg is None else radians(steer_actual_deg)
    out = []
    for name in WHEEL_ORDER:
        x, y = car.contact_points()[name]
        u = s.vx_ms - s.yaw_rate_rads * y
        v = s.vy_ms + s.yaw_rate_rads * x
        d = delta if name.startswith('F') else 0.0
        out.append(cos(d) * u + sin(d) * v)
    return tuple(out)


def wheel_slips(
    s: VehicleState,
    omega: tuple[float, ...],
    t: float,
    car: VehicleParameters,
    controls: TorqueControls,
    wheel: WheelParameters,
    steer_actual_deg: float | None = None,
) -> tuple[float, ...]:
    vxs = wheel_longitudinal_velocities(s, t, car, controls, steer_actual_deg)
    if not all(v >= wheel.min_contact_speed_ms for v in vxs):
        raise OutsideEnvelopeError('Wheel contact speed below forward-only domain threshold')
    kappa = tuple((wheel.radius_m * w - vx) / abs(vx) for w, vx in zip(omega, vxs))
    if not all(isfinite(k) and abs(k) <= wheel.max_abs_slip for k in kappa):
        raise OutsideEnvelopeError('Slip outside calibrated domain; no force extrapolation allowed')
    if min(omega) < -1e-6:
        raise OutsideEnvelopeError('Reverse wheel rotation not modeled')
    return kappa


def validate_no_lift(car: VehicleParameters, ax: float, ay: float):
    """Test raw quasi-static contact loads before wheel_loads applies clipping."""
    m, g = car.mass_kg, car.gravity
    if not car.load_transfer:
        return
    a, L = car.cg_to_front_m, car.wheelbase_m
    b = L - a
    front = m * (g * b - ax * car.cg_height_m) / L
    rear = m * g - front
    df = m * ay * car.cg_height_m / car.track_m * b / L
    dr = m * ay * car.cg_height_m / car.track_m * a / L
    raw = ((front - df) / 2, (front + df) / 2, (rear - dr) / 2, (rear + dr) / 2)
    if min(raw) <= 0:
        raise OutsideEnvelopeError('Quasi-static model predicts wheel lift: roll/contact dynamics required')


def forces_at(
    s: VehicleState,
    omega: tuple[float, ...],
    t: float,
    car: VehicleParameters,
    tyre: TyreParameters,
    controls: TorqueControls,
    wheel: WheelParameters,
    tyre_model: Callable | None = None,
    steer_actual_deg: float | None = None,
    load_solver: str = "fixed_point",
) -> tuple[VehicleForces, tuple[float, ...]]:
    slips = wheel_slips(s, omega, t, car, controls, wheel, steer_actual_deg)
    ctl = DriverControls(controls.steer_deg, slips)

    def residual_accel(estimate):
        f = evaluate(s, ctl, car, tyre, t, tuple(estimate), tyre_model, steer_actual_deg)
        return (float(estimate[0] - f.ax_ms2), float(estimate[1] - f.ay_ms2))

    if load_solver == 'root':
        # Two-variable root of the quasi-static load/acceleration coupling.
        # No stale state or cached accelerations across adaptive RHS evaluations.
        solution = root(residual_accel, (0.0, 0.0), method='hybr', options={'maxfev': 30, 'xtol': 1e-10})
        if max(abs(z) for z in residual_accel(solution.x)) > 1e-7:
            raise OutsideEnvelopeError('Quasi-static load root did not converge')
        guess = tuple(float(z) for z in solution.x)
    elif load_solver == 'fixed_point':
        guess = (0.0, 0.0)
        for _ in range(40):
            f = evaluate(s, ctl, car, tyre, t, guess, tyre_model, steer_actual_deg)
            next_guess = (0.5 * (guess[0] + f.ax_ms2), 0.5 * (guess[1] + f.ay_ms2))
            if max(abs(a - b) for a, b in zip(next_guess, guess)) < 1e-7:
                guess = next_guess
                break
            guess = next_guess
        else:
            raise OutsideEnvelopeError('Load-transfer fixed-point iteration did not converge')
    else:
        raise ValueError('Unknown load-solver')
    validate_no_lift(car, *guess)
    return evaluate(s, ctl, car, tyre, t, guess, tyre_model, steer_actual_deg), slips


def derivative(
    t: float,
    x: np.ndarray,
    car: VehicleParameters,
    tyre: TyreParameters,
    controls: TorqueControls,
    wheel: WheelParameters,
    policy: TorquePolicy | None = None,
    tyre_model: Callable | None = None,
    actuator: SteeringActuator | None = None,
    load_solver: str = "fixed_point",
) -> np.ndarray:
    s, omega = decode(x)
    controls = active_control(t, s, controls, policy)
    steer_actual = float(x[10]) if actuator is not None else None
    f, _ = forces_at(s, omega, t, car, tyre, controls, wheel, tyre_model, steer_actual, load_solver)
    du = f.ax_ms2 + s.yaw_rate_rads * s.vy_ms
    dv = f.ay_ms2 - s.yaw_rate_rads * s.vx_ms
    rdot = f.mz_nm / car.iz_kgm2
    psi = s.yaw_rad
    deriv = [
        s.vx_ms * cos(psi) - s.vy_ms * sin(psi),
        s.vx_ms * sin(psi) + s.vy_ms * cos(psi),
        s.yaw_rate_rads,
        du,
        dv,
        rdot,
    ]
    for index, name in enumerate(WHEEL_ORDER):
        local_fx = f.wheels[name].fx_n
        # Forward-running model: friction brake opposes positive rotation.
        net_torque = controls.drive_nm[index] - controls.brake_nm[index]
        deriv.append((net_torque - wheel.radius_m * local_fx) / wheel.inertia_kgm2)
    if actuator is not None:
        deriv.append(actuator.derivative(controls.steer_deg, steer_actual))
    arr = np.asarray(deriv, dtype=float)
    if not np.all(np.isfinite(arr)):
        raise OutsideEnvelopeError('Non-finite dynamics derivative')
    return arr
