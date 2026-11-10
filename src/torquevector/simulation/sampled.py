"""Sampled zero-order-hold (ZOH) simulation: controllers act only at sample instants, the plant is integrated in between."""

from __future__ import annotations

from dataclasses import dataclass
from math import ceil, isfinite
from typing import Callable

import numpy as np
from scipy.integrate import solve_ivp

from torquevector.domain.controls import TorqueControls, TorquePolicy, active_control
from torquevector.domain.errors import OutsideEnvelopeError
from torquevector.domain.states import decode
from torquevector.domain.telemetry import TorqueSample
from torquevector.domain.tyre import TyreParameters
from torquevector.domain.vehicle import VehicleParameters
from torquevector.domain.wheels import WheelParameters
from torquevector.physics.steering import SteeringActuator
from torquevector.physics.wheel_rotation import derivative, forces_at, wheel_slips


@dataclass(frozen=True)
class SampledSettings:
    period_s: float = 0.12
    max_torque_rate_nm_s: float = 3000.0
    max_steer_rate_command_deg_s: float = 120.0
    max_step_s: float = 0.06
    rtol: float = 2e-5
    max_rhs_per_period: int = 600
    atol: float = 1e-8
    method: str = 'DOP853'

    def __post_init__(self):
        x = (
            self.period_s,
            self.max_torque_rate_nm_s,
            self.max_steer_rate_command_deg_s,
            self.max_step_s,
            self.rtol,
            self.atol,
        )
        if not all(isfinite(z) and z > 0 for z in x) or not (
            0.025 <= self.period_s <= 0.5
            and self.max_step_s <= self.period_s
            and 1e-10 <= self.rtol <= 1e-3
            and 1e-14 <= self.atol <= 1e-3
            and self.method in ('DOP853', 'RK45', 'Radau')
            and isinstance(self.max_rhs_per_period, int)
            and self.max_rhs_per_period >= 30
        ):
            raise ValueError('Invalid sampled-loop configuration')


def slew_limit(request: TorqueControls, previous: TorqueControls, dt: float, cfg: SampledSettings) -> TorqueControls:
    """Hard per-wheel drive/brake torque and steering setpoint change limits."""
    dn = cfg.max_torque_rate_nm_s * dt
    ds = cfg.max_steer_rate_command_deg_s * dt

    def limit(values, prior):
        return tuple(float(np.clip(v, p - dn, p + dn)) for v, p in zip(values, prior))

    steer = float(np.clip(request.steer_deg, previous.steer_deg - ds, previous.steer_deg + ds))
    return TorqueControls(
        steer,
        limit(request.drive_nm, previous.drive_nm),
        limit(request.brake_nm, previous.brake_nm),
        request.torque_limit_nm,
    )


def simulate_sampled(
    car: VehicleParameters,
    tyre: TyreParameters,
    controls: TorqueControls,
    wheel: WheelParameters = WheelParameters(),
    *,
    speed_ms: float = 18.0,
    duration_s: float = 7.0,
    policy: TorquePolicy | None = None,
    tyre_model: Callable | None = None,
    actuator: SteeringActuator = SteeringActuator(),
    settings: SampledSettings = SampledSettings(),
    load_solver: str = 'root',
    initial_pose: tuple[float, float, float, float, float] | None = None,
    tyre_model_at: Callable | None = None,
    event_times: Callable | None = None,
    slip_controller: Callable | None = None,
    slip_period_s: float | None = None,
    stop_condition: Callable | None = None,
) -> list[TorqueSample]:
    """``initial_pose`` = (x_m, y_m, yaw_rad, vy_ms, yaw_rate_rads); vx = speed_ms, Räder frei rollend.

    ``tyre_model_at(state)`` (optional) liefert das Reifenmodell für genau diesen Zustand, z. B. mit
    lokalem Fahrbahnreibwert je Rad. Es wird bei *jeder* ODE-Auswertung neu gebunden, so dass ein Rad
    einen Reibwertübergang innerhalb eines Halteintervalls physikalisch korrekt überfährt; der Regler
    bleibt davon unberührt und wird nur zu den Abtastzeitpunkten aufgerufen.

    ``event_times(state, dt)`` (optional) liefert relative Zeitpunkte bekannter Unstetigkeiten der rechten
    Seite (z. B. scharfe Reibwertgrenzen). Das Halteintervall wird dort geteilt und der Integrator neu
    gestartet; das gehaltene Kommando bleibt unverändert.

    ``slip_controller(t, kappa, held) -> TorqueControls`` (optional, mit ``slip_period_s``) ist ein schneller
    innerer Abtastregler (ABS/ASR, Mehrraten-ZOH): Das Halteintervall wird in Teilintervalle der Länge
    ``slip_period_s`` geteilt; zu Beginn jedes Teilintervalls wird aus dem gemessenen Schlupf das tatsächlich
    angelegte Radmoment bestimmt und bis zum nächsten inneren Takt gehalten. Ohne ``slip_controller`` ist der
    Ablauf unverändert.

    ``stop_condition(state) -> bool`` (optional) wird zu Beginn jedes Regeltakts mit dem gemessenen Zustand
    ausgewertet; liefert sie ``True``, endet die Simulation regulär mit dem bis dahin berechneten Verlauf
    (z. B. Fahrgrenze erreicht: Schwimmwinkel über einer Schwelle). Ohne ``stop_condition`` unverändert.
    """

    def model_for(state):
        return tyre_model if tyre_model_at is None else tyre_model_at(state)

    if slip_controller is not None and not (
        slip_period_s is not None and isfinite(slip_period_s) and 0 < slip_period_s <= settings.period_s
    ):
        raise ValueError('slip_controller requires 0 < slip_period_s <= period_s')
    if not (3.5 <= speed_ms <= 75 and 0 < duration_s <= 30):
        raise ValueError('Invalid sample integration duration or speed')
    x0, y0, yaw0, vy0, r0 = (0.0, 0.0, 0.0, 0.0, 0.0) if initial_pose is None else map(float, initial_pose)
    if not all(isfinite(z) for z in (x0, y0, yaw0, vy0, r0)):
        raise ValueError('Initial pose must be finite')
    # Zero-order hold: every command is computed from the *previous* measured state.
    raw = np.asarray((x0, y0, yaw0, speed_ms, vy0, r0) + (speed_ms / wheel.radius_m,) * 4 + (0.0,), float)
    prev = controls
    result = []
    steps = int(ceil(duration_s / settings.period_s))
    end_s = duration_s
    for k in range(steps):
        t0 = k * settings.period_s
        t1 = min(duration_s, (k + 1) * settings.period_s)
        dt = t1 - t0
        if dt <= 1e-12:
            break
        state, omega = decode(raw)
        if stop_condition is not None and k > 0 and stop_condition(state):
            end_s = t0
            break
        observer = getattr(policy, 'observe_actuator', None)
        if observer is not None:
            observer(float(raw[10]), actuator)
        requested = active_control(t0, state, controls, policy)
        held = slew_limit(requested, prev, dt, settings)
        prev = held
        inner = 1 if slip_controller is None else max(1, int(ceil(dt / slip_period_s - 1e-9)))
        calls = [0]
        for j in range(inner):
            a = t0 + j * dt / inner
            b = t0 + (j + 1) * dt / inner
            sub_state, sub_omega = decode(raw)
            applied = (
                held
                if slip_controller is None
                else slip_controller(a, wheel_slips(sub_state, sub_omega, a, car, held, wheel, float(raw[10])), held)
            )
            if j == 0:
                # Store sample at the beginning of the *held* control interval.
                forces, slips = forces_at(
                    state, omega, t0, car, tyre, applied, wheel, model_for(state), float(raw[10]), load_solver
                )
                result.append(
                    TorqueSample(t0, state, forces, omega, slips, applied.drive_nm, applied.brake_nm, float(raw[10]))
                )
            raw = _hold(
                raw,
                applied,
                a,
                b,
                sub_state,
                b - a,
                calls,
                car,
                tyre,
                wheel,
                tyre_model,
                tyre_model_at,
                actuator,
                load_solver,
                settings,
                event_times,
                t0,
            )
        held = applied
    state, omega = decode(raw)
    forces, slips = forces_at(
        state, omega, end_s, car, tyre, held, wheel, model_for(state), float(raw[10]), load_solver
    )
    result.append(TorqueSample(end_s, state, forces, omega, slips, held.drive_nm, held.brake_nm, float(raw[10])))
    return result


def _hold(
    raw,
    held,
    a,
    b,
    state,
    dt,
    calls,
    car,
    tyre,
    wheel,
    tyre_model,
    tyre_model_at,
    actuator,
    load_solver,
    settings,
    event_times,
    t_tick,
):
    """Integriert ein Intervall mit konstantem Kommando ``held`` (geteilt an bekannten Unstetigkeiten)."""

    def rhs(t, x):
        calls[0] += 1
        if calls[0] > settings.max_rhs_per_period:
            raise OutsideEnvelopeError('Sampled integrator RHS budget exceeded')
        # Critical: policy=None; held command cannot change within this integration.
        model = tyre_model if tyre_model_at is None else tyre_model_at(decode(x)[0])
        return derivative(t, x, car, tyre, held, wheel, None, model, actuator, load_solver)

    cuts = [a] + [a + e for e in (event_times(state, dt) if event_times is not None else [])] + [b]
    for lo, hi in zip(cuts[:-1], cuts[1:]):
        try:
            sol = solve_ivp(
                rhs,
                (lo, hi),
                raw,
                method=settings.method,
                rtol=settings.rtol,
                atol=settings.atol,
                max_step=settings.max_step_s,
            )
        except OutsideEnvelopeError as exc:
            raise OutsideEnvelopeError(f'Sampled t={t_tick:.3f}s: {exc}') from exc
        if not sol.success or not np.all(np.isfinite(sol.y)):
            raise OutsideEnvelopeError('Sampled integrator failure: ' + sol.message)
        raw = sol.y[:, -1]
    return raw
