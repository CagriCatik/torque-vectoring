"""Driver model: path-following steering and speed control producing base wheel torques."""

from __future__ import annotations
from dataclasses import dataclass
from math import atan
from math import degrees
from math import pi
from math import sin
from torquevector.domain.vehicle import VehicleParameters
from torquevector.domain.states import VehicleState
from torquevector.domain.controls import TorqueControls
from torquevector.domain.scenarios import ManoeuvreSpec
from torquevector.domain.scenarios import clamp
from torquevector.domain.scenarios import smoothstep


@dataclass(frozen=True)
class ManoeuvrePolicy:
    spec: ManoeuvreSpec
    car: VehicleParameters
    tyre_radius_m: float = 0.315
    max_drive_torque_nm: float = 250.0
    step_ramp_s: float = 0.4

    def steer_command(self, t: float, s: VehicleState) -> float:
        spec = self.spec
        if spec.kind == 'circle_open':
            # Open loop at the *road wheel*; speed is held by longitudinal feedback.
            return degrees(atan(self.car.wheelbase_m / spec.radius_m))
        if spec.kind in ('circle_track', 'double_lane_change', 'straight'):
            ref, e_y, epsi = spec.reference_at(s)
            delta = atan(self.car.wheelbase_m * ref.curvature_inv_m)
            delta += spec.heading_gain * epsi
            delta += atan(spec.cross_gain * e_y / max(3.0, s.vx_ms))
            return degrees(delta)
        if spec.kind == 'step_steer':
            return spec.steer_amplitude_deg * smoothstep((t - spec.excitation_start_s) / self.step_ramp_s)
        if spec.kind == 'ramp_steer':
            ramp = (t - spec.excitation_start_s) / (spec.duration_s - spec.excitation_start_s)
            return spec.steer_amplitude_deg * clamp(ramp, 0.0, 1.0)
        if spec.kind == 'sine_steer':
            if t < spec.excitation_start_s:
                return 0.0
            tau = t - spec.excitation_start_s
            return spec.steer_amplitude_deg * sin(2 * pi * spec.steer_frequency_hz * tau)
        raise ValueError('Unsupported kind')

    def __call__(self, t: float, s: VehicleState, base: TorqueControls) -> TorqueControls:
        steer = clamp(self.steer_command(t, s), -self.spec.max_steer_deg, self.spec.max_steer_deg)
        target = self.spec.speed_ms
        drag = (
            0.5 * self.car.air_density_kgm3 * self.car.cd_area_m2 * target * target
            + self.car.rolling_coefficient * self.car.mass_kg * self.car.gravity
        )
        nominal = drag * self.tyre_radius_m / 2.0
        rear = clamp(nominal + self.spec.speed_kp_nm_per_ms * (target - s.vx_ms), 0.0, self.max_drive_torque_nm)
        return TorqueControls(steer, (0.0, 0.0, rear, rear), (0.0, 0.0, 0.0, 0.0), base.torque_limit_nm)
