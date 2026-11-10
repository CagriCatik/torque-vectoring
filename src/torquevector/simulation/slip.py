"""Legacy slip-driven simulation (prescribed slip ratios, semi-implicit Euler) used by the dashboard."""

from __future__ import annotations
from torquevector.domain.controls import DriverControls
from torquevector.domain.vehicle import VehicleParameters
from torquevector.domain.states import VehicleState
from torquevector.physics.four_wheel import evaluate
from torquevector.physics.four_wheel import advance
from torquevector.domain.tyre import TyreParameters
from torquevector.domain.telemetry import Sample


def simulate(
    car: VehicleParameters,
    tyre: TyreParameters,
    controls: DriverControls,
    speed_ms: float = 18.0,
    duration_s: float = 4.5,
    dt: float = 0.02,
) -> list[Sample]:
    if not (3 <= speed_ms <= 75 and 0 < duration_s <= 30 and 0 < dt <= 0.05):
        raise ValueError('Simulation must use speed 3..75 m/s, time <=30 s, dt<=0.05 s')
    steps = round(duration_s / dt)
    if steps < 1:
        raise ValueError('Simulation shorter than a step')
    actual_dt = duration_s / steps
    state = VehicleState(vx_ms=speed_ms)
    samples = []
    prev_guess = (0.0, 0.0)
    for i in range(steps + 1):
        t = i * actual_dt
        # Two passes to update the quasi-static load transfer using the current
        # acceleration, without coupling GUI concerns into the solver.
        first = evaluate(state, controls, car, tyre, t, prev_guess)
        forces = evaluate(state, controls, car, tyre, t, (first.ax_ms2, first.ay_ms2))
        samples.append(Sample(t, state, forces))
        prev_guess = (forces.ax_ms2, forces.ay_ms2)
        if i < steps:
            state = advance(state, forces, car, actual_dt)
    return samples
