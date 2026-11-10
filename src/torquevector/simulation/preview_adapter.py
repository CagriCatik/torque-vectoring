"""Manoeuvre preview for predictive controllers (speed, steering, yaw reference, optional friction ratios)."""

from __future__ import annotations
from math import atan
from math import radians
from torquevector.physics.steering import SteeringActuator
import numpy as np
from torquevector.domain.vehicle import VehicleParameters
from torquevector.domain.states import VehicleState
from torquevector.domain.controls import TorqueControls
from torquevector.domain.scenarios import ManoeuvreSpec
from torquevector.simulation.driving import ManoeuvrePolicy
from torquevector.domain.scenarios import smoothstep
from torquevector.prediction.interfaces import PreviewConfig
from torquevector.prediction.interfaces import PredictionPreview


def make_maneuver_preview(
    spec: ManoeuvreSpec,
    car: VehicleParameters,
    *,
    base_policy: ManoeuvrePolicy | None = None,
    speed_time_constant_s: float = 2.0,
    engage_ramp_s: float = 0.55,
    friction_fraction: float = 0.75,
    friction_ratios=None,
    reference_source: str = 'path_curvature',
):
    """``friction_ratios(x, y, psi) -> (vorne, hinten)`` liefert optional Kartenwissen über die Fahrbahn.

    Abgefragt wird nur an Posen, die aus dem aktuellen gemessenen Zustand und der Sollbahn
    vorhergesagt werden – nie an zukünftigen simulierten Zuständen.
    """
    base_policy = ManoeuvrePolicy(spec, car) if base_policy is None else base_policy
    observation = [None, SteeringActuator()]

    def observe_actuator(actual_deg, actuator):
        observation[:] = [float(actual_deg), actuator]

    def forecast(t: float, state: VehicleState, base: TorqueControls, cfg: PreviewConfig):
        speed = max(cfg.min_speed_ms, min(cfg.max_speed_ms, state.vx_ms))
        cur = spec.reference_at_distance(state.x_m)
        cur_curvature = cur.curvature_inv_m if cur else 0.0
        vs = []
        steers = []
        betas = []
        yaws = []
        front = []
        rear = []
        actual, actuator = observation
        for k in range(cfg.horizon_steps):
            ahead = (k + 1) * cfg.sample_s
            vk = float(
                np.clip(
                    speed + (spec.speed_ms - speed) * (1 - np.exp(-ahead / speed_time_constant_s)),
                    cfg.min_speed_ms,
                    cfg.max_speed_ms,
                )
            )
            projected = state.x_m + speed * ahead
            ref = spec.reference_at_distance(projected)
            if ref is not None:
                curvature = ref.curvature_inv_m
                requested = vk * curvature * smoothstep((t + ahead) / engage_ramp_s)
                steer = base.steer_deg + np.rad2deg(
                    atan(car.wheelbase_m * curvature) - atan(car.wheelbase_m * cur_curvature)
                )
            else:
                steer = base_policy.steer_command(t + ahead, state)
                requested = vk * np.tan(radians(steer)) / car.wheelbase_m
            requested = float(
                np.clip(
                    requested,
                    -friction_fraction * cfg.mu_reference * car.gravity / max(vk, 3.0),
                    friction_fraction * cfg.mu_reference * car.gravity / max(vk, 3.0),
                )
            )
            vs.append(vk)
            if friction_ratios is not None:
                if ref is not None:
                    pose = (ref.x_m, ref.y_m, ref.heading_rad)
                else:
                    pose = (
                        state.x_m + speed * ahead * np.cos(state.yaw_rad),
                        state.y_m + speed * ahead * np.sin(state.yaw_rad),
                        state.yaw_rad,
                    )
                rf, rr = friction_ratios(*pose)
                front.append(float(rf))
                rear.append(float(rr))
            if actual is not None:
                # Inputs act over [k,k+1]; references above belong to k+1.
                start_ahead = k * cfg.sample_s
                start_ref = spec.reference_at_distance(state.x_m + speed * start_ahead)
                if start_ref is not None:
                    steer = base.steer_deg + np.rad2deg(
                        atan(car.wheelbase_m * start_ref.curvature_inv_m) - atan(car.wheelbase_m * cur_curvature)
                    )
                else:
                    steer = base_policy.steer_command(t + start_ahead, state)
                command = float(np.clip(steer, -spec.max_steer_deg, spec.max_steer_deg))
                actual, steer = actuator.predict_hold(command, actual, cfg.sample_s)
            steers.append(radians(float(np.clip(steer, -spec.max_steer_deg, spec.max_steer_deg))))
            if reference_source == 'driver_steering':
                # Kinematische Fahrerabsicht aus dem vorhergesagten Lenkwinkel dieser Stufe.
                requested = float(
                    np.clip(
                        vk * np.tan(steers[-1]) / car.wheelbase_m,
                        -friction_fraction * cfg.mu_reference * car.gravity / max(vk, 3.0),
                        friction_fraction * cfg.mu_reference * car.gravity / max(vk, 3.0),
                    )
                )
            betas.append(0.0)
            yaws.append(requested)
        if friction_ratios is None:
            return PredictionPreview(tuple(vs), tuple(steers), tuple(betas), tuple(yaws))
        return PredictionPreview(tuple(vs), tuple(steers), tuple(betas), tuple(yaws), tuple(front), tuple(rear))

    forecast.observe_actuator = observe_actuator
    return forecast
