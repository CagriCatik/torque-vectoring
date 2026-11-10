"""Physikalische Invarianten je Zeitschritt mit automatischen Alarmen (Q08.1–Q08.5).

Unabhängige Nachrechnung aus gespeicherter Telemetrie (keine Simulation, kein Code-Sharing mit
``physics.four_wheel``):

- Kraft-/Momentenbilanz: Summe der Radkräfte im Fahrzeugsystem, Giermoment über die Aufstandspunkte,
  ax = (ΣFx − Fahrwiderstand)/m, ay = ΣFy/m, und die Rotation Rad → Fahrzeug.
- Kontakt: Fz > 0 an allen Rädern.
- Kraftgrenze: |F| ≤ µ(Fz)·Fz (relativ 1e-6).
- Leistungsvorzeichen: Die Reifenkontaktleistung ist dissipativ (≤ 0 bis auf Rundung).

Die Toleranzen sind die Akzeptanzwerte aus ``docs/quality/QUALITY_GATES.md``, keine Modellparameter.
"""

from __future__ import annotations

from math import cos, hypot, radians, sin
from typing import Sequence

from torquevector.domain.telemetry import TorqueSample
from torquevector.domain.vehicle import VehicleParameters
from torquevector.domain.wheels import WHEEL_ORDER, WheelParameters

BALANCE_TOLERANCE = 1e-6  # Q08.1 / Q05.1: relativ für algebraische Bilanzgleichungen
FORCE_LIMIT_TOLERANCE = 1e-6  # Q03.4: relativ zur Kraftschlussgrenze
CONTACT_POWER_TOLERANCE_W = 1e-6  # Q08.3: Rundung der Kontaktleistung


def _front_steer_rad(sample: TorqueSample, steer_rad_at) -> float:
    """Radlenkwinkel vorne: gemessener Aktuatorwinkel (getaktete Simulation) oder ``steer_rad_at(t)``."""
    if sample.steer_actual_deg is not None:
        return radians(sample.steer_actual_deg)
    if steer_rad_at is None:
        raise ValueError('Ohne Aktuatorwinkel in der Telemetrie wird steer_rad_at(t) benötigt')
    return steer_rad_at(sample.t_s)


def _wheel_velocity(sample: TorqueSample, car: VehicleParameters, name: str, steer_rad: float) -> tuple[float, float]:
    s = sample.state
    x, y = car.contact_points()[name]
    vel_x, vel_y = s.vx_ms - s.yaw_rate_rads * y, s.vy_ms + s.yaw_rate_rads * x
    delta = steer_rad if name.startswith('F') else 0.0
    return cos(delta) * vel_x + sin(delta) * vel_y, -sin(delta) * vel_x + cos(delta) * vel_y


def balance_residuals(sample: TorqueSample, car: VehicleParameters, steer_rad_at=None) -> dict:
    """Relative Residuen der algebraischen Bilanzgleichungen eines Zeitschritts."""
    f = sample.forces
    steer = _front_steer_rad(sample, steer_rad_at)
    contacts = car.contact_points()
    fx = sum(f.force_body[n][0] for n in WHEEL_ORDER)
    fy = sum(f.force_body[n][1] for n in WHEEL_ORDER)
    mz = sum(contacts[n][0] * f.force_body[n][1] - contacts[n][1] * f.force_body[n][0] for n in WHEEL_ORDER)
    rotation = 0.0
    for n in WHEEL_ORDER:
        delta = steer if n.startswith('F') else 0.0
        w = f.wheels[n]
        bx, by = w.fx_n * cos(delta) - w.fy_n * sin(delta), w.fx_n * sin(delta) + w.fy_n * cos(delta)
        rotation = max(rotation, hypot(bx - f.force_body[n][0], by - f.force_body[n][1]))
    scale_f = max(1.0, sum(hypot(*f.force_body[n]) for n in WHEEL_ORDER), car.mass_kg * car.gravity * 1e-3)
    scale_m = scale_f * max(car.wheelbase_m, car.track_m)
    return {
        'force_x': abs(fx - f.fx_n) / scale_f,
        'force_y': abs(fy - f.fy_n) / scale_f,
        'moment_z': abs(mz - f.mz_nm) / scale_m,
        'accel_x': abs((fx - f.drag_n) / car.mass_kg - f.ax_ms2) * car.mass_kg / scale_f,
        'accel_y': abs(fy / car.mass_kg - f.ay_ms2) * car.mass_kg / scale_f,
        'wheel_to_body_rotation': rotation / scale_f,
    }


def contact_power_w(sample: TorqueSample, car: VehicleParameters, wheel: WheelParameters, steer_rad_at=None) -> float:
    """Leistung der Reifenkontaktkräfte über die Gleitgeschwindigkeit (Schlupf); dissipativ heißt ≤ 0."""
    steer = _front_steer_rad(sample, steer_rad_at)
    power = 0.0
    for i, n in enumerate(WHEEL_ORDER):
        wx, wy = _wheel_velocity(sample, car, n, steer)
        t = sample.forces.wheels[n]
        power += t.fx_n * (wx - wheel.radius_m * sample.omega_rad_s[i]) + t.fy_n * wy
    return power


def invariant_report(
    samples: Sequence[TorqueSample], car: VehicleParameters, wheel: WheelParameters, steer_rad_at=None
) -> dict:
    """Bericht je Szenario: Maxima aller Residuen und Liste ausgelöster Alarme (leer = bestanden)."""
    if not samples:
        raise ValueError('Leere Telemetrie')
    worst = {}
    min_fz, force_excess, max_power = float('inf'), 0.0, float('-inf')
    for sample in samples:
        for key, value in balance_residuals(sample, car, steer_rad_at).items():
            worst[key] = max(worst.get(key, 0.0), value)
        for w in sample.forces.wheels.values():
            min_fz = min(min_fz, w.fz_n)
            if w.peak_n > 0:
                force_excess = max(force_excess, hypot(w.fx_n, w.fy_n) / w.peak_n - 1.0)
        max_power = max(max_power, contact_power_w(sample, car, wheel, steer_rad_at))
    alarms = [
        f'Bilanzresiduum {k} = {v:.3e} > {BALANCE_TOLERANCE:g}' for k, v in worst.items() if v > BALANCE_TOLERANCE
    ]
    if min_fz <= 0:
        alarms.append(f'Radlast {min_fz:.3f} N <= 0 im Kontaktzustand')
    if force_excess > FORCE_LIMIT_TOLERANCE:
        alarms.append(f'Kraftschlussgrenze um {force_excess:.3e} relativ überschritten')
    if max_power > CONTACT_POWER_TOLERANCE_W:
        alarms.append(f'Kontaktleistung {max_power:.3e} W > 0 (nicht dissipativ)')
    return {
        'samples': len(samples),
        'max_balance_residual': worst,
        'min_wheel_load_n': min_fz,
        'max_force_limit_excess': force_excess,
        'max_contact_power_w': max_power,
        'alarms': alarms,
        'passed': not alarms,
    }
