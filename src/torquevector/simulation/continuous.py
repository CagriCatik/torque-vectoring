"""Continuous-time reference integration of the torque-driven four-wheel model (high-accuracy, no sampling)."""

from __future__ import annotations
from typing import Callable
from torquevector.physics.steering import SteeringActuator
import numpy as np
from scipy.integrate import solve_ivp
from torquevector.domain.vehicle import VehicleParameters
from torquevector.domain.tyre import TyreParameters
from torquevector.domain.errors import OutsideEnvelopeError
from torquevector.domain.controls import TorqueControls
from torquevector.domain.controls import TorquePolicy
from torquevector.domain.telemetry import TorqueSample
from torquevector.domain.wheels import WheelParameters
from torquevector.domain.controls import active_control
from torquevector.domain.states import decode
from torquevector.physics.wheel_rotation import derivative
from torquevector.physics.wheel_rotation import forces_at


def simulate_torque(
    car: VehicleParameters,
    tyre: TyreParameters,
    controls: TorqueControls,
    wheel: WheelParameters = WheelParameters(),
    speed_ms: float = 18.0,
    duration_s: float = 2.0,
    output_dt: float = 0.02,
    max_step: float = 0.02,
    rtol: float = 1e-7,
    method: str = 'DOP853',
    max_rhs_calls: int = 12000,
    policy: TorquePolicy | None = None,
    tyre_model: Callable | None = None,
    actuator: SteeringActuator | None = None,
    load_solver: str = "fixed_point",
) -> list[TorqueSample]:
    if not (
        3.5 <= speed_ms <= 75
        and 0 < duration_s <= 30
        and 0 < output_dt <= 0.1
        and 0 < max_step <= 0.1
        and 1e-11 <= rtol <= 1e-3
    ):
        raise ValueError('Invalid integration envelope, time or tolerance')
    if method not in ('DOP853', 'RK45', 'Radau'):
        raise ValueError('Unapproved solver')
    steps = int(np.ceil(duration_s / output_dt))
    t_eval = np.linspace(0.0, duration_s, steps + 1)
    x0 = np.asarray(
        (0, 0, 0, speed_ms, 0, 0) + (speed_ms / wheel.radius_m,) * 4 + ((0.0,) if actuator is not None else ()),
        dtype=float,
    )
    evaluations = [0]

    def rhs(t, x):
        evaluations[0] += 1
        if evaluations[0] > max_rhs_calls:
            raise OutsideEnvelopeError(
                'Solver RHS evaluation budget exceeded: potentially stiff or unsupported scenario'
            )
        return derivative(t, x, car, tyre, controls, wheel, policy, tyre_model, actuator, load_solver)

    sol = solve_ivp(rhs, (0, duration_s), x0, method=method, t_eval=t_eval, rtol=rtol, atol=1e-9, max_step=max_step)
    if not sol.success or not np.all(np.isfinite(sol.y)):
        raise OutsideEnvelopeError(f'Integration failed: {sol.message}')
    result = []
    for t, raw in zip(sol.t, sol.y.T):
        s, omegas = decode(raw)
        ctl = active_control(float(t), s, controls, policy)
        actual = float(raw[10]) if actuator is not None else None
        forces, kappa = forces_at(s, omegas, float(t), car, tyre, ctl, wheel, tyre_model, actual, load_solver)
        result.append(TorqueSample(float(t), s, forces, omegas, kappa, ctl.drive_nm, ctl.brake_nm, actual))
    return result
