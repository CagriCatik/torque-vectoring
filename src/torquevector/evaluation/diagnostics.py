"""Energy and power audit of torque-driven simulations (kinetic energy, drive/brake/slip/resistance power)."""

from __future__ import annotations
from torquevector.domain.telemetry import TorqueSample
from torquevector.domain.wheels import WheelParameters
from torquevector.domain.vehicle import VehicleParameters
from math import cos
from math import sin
from math import radians
from torquevector.domain.controls import TorqueControls
from torquevector.physics.wheel_rotation import steering
from torquevector.domain.wheels import WHEEL_ORDER


def energy_j(s: TorqueSample, car: VehicleParameters, wheel: WheelParameters) -> float:
    v = s.state
    return (
        0.5 * car.mass_kg * (v.vx_ms**2 + v.vy_ms**2)
        + 0.5 * car.iz_kgm2 * v.yaw_rate_rads**2
        + 0.5 * wheel.inertia_kgm2 * sum(w * w for w in s.omega_rad_s)
    )


def power_w(s: TorqueSample, car: VehicleParameters, wheel: WheelParameters, ctrl: TorqueControls) -> dict:
    v = s.state
    delta = steering(s.t_s, ctrl) if s.steer_actual_deg is None else radians(s.steer_actual_deg)
    slip_power = 0.0
    for i, name in enumerate(WHEEL_ORDER):
        x, y = car.contact_points()[name]
        vel_x = v.vx_ms - v.yaw_rate_rads * y
        vel_y = v.vy_ms + v.yaw_rate_rads * x
        d = delta if name.startswith('F') else 0.0
        wx = cos(d) * vel_x + sin(d) * vel_y
        wy = -sin(d) * vel_x + cos(d) * vel_y
        tyre = s.forces.wheels[name]
        slip_power += tyre.fx_n * (wx - wheel.radius_m * s.omega_rad_s[i]) + tyre.fy_n * wy
    driv = sum(t * om for t, om in zip(s.drive_nm, s.omega_rad_s))
    brake = sum(t * om for t, om in zip(s.brake_nm, s.omega_rad_s))
    resist = s.forces.drag_n * v.vx_ms
    return {
        'drive_w': driv,
        'brake_w': brake,
        'resistance_w': resist,
        'slip_w': slip_power,
        'net_w': driv - brake - resist + slip_power,
    }


def audit_energy(
    samples: list[TorqueSample], car: VehicleParameters, wheel: WheelParameters, ctrl: TorqueControls
) -> dict:
    from numpy import trapezoid

    energies = [energy_j(s, car, wheel) for s in samples]
    powers = [power_w(s, car, wheel, ctrl) for s in samples]
    integrated = float(trapezoid([p['net_w'] for p in powers], [s.t_s for s in samples]))
    actual = energies[-1] - energies[0]
    scale = max(100.0, abs(actual), abs(integrated), max(energies) - min(energies))
    residual = actual - integrated
    return {
        'delta_energy_j': actual,
        'integrated_net_power_j': integrated,
        'residual_j': residual,
        'relative_residual': abs(residual) / scale,
        'max_contact_power_w': max(p['slip_w'] for p in powers),
        'min_energy_j': min(energies),
    }
