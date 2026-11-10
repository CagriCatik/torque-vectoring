"""Unabhängige theoretische Referenzmanöver mit tabellarischen Ergebnissen und Toleranzen (Q09.2–Q09.5).

Referenzen sind analytisch und teilen keine Kernformeln mit ``physics``:

- B1 Stationäre Kreisfahrt im linearen Bereich (|ay| <= 0,2 g) gegen das lineare Einspurmodell
  (``evaluation.benchmarks.linear_bicycle_stationary``) plus die Kinematik ay ≈ v·r und r ≈ v/R.
- B2 Radlasten: statisch (m·g, Momentenbilanz), dynamisch symmetrisch (Längsbeschleunigung) und
  asymmetrisch (stationäre Kreisfahrt mit Lastverlagerung), Handformeln aus Schwerpunkthöhe/Spur/Radstand.

Toleranzen sind die Akzeptanzwerte aus ``docs/quality/QUALITY_GATES.md``.
"""

from __future__ import annotations

from dataclasses import replace

import numpy as np

from torquevector.domain.controls import TorqueControls
from torquevector.domain.tyre import TyreParameters
from torquevector.domain.vehicle import VehicleParameters
from torquevector.domain.wheels import WheelParameters
from torquevector.evaluation.benchmarks import linear_bicycle_stationary, nominal_cornering_stiffness
from torquevector.simulation.continuous import simulate_torque

LINEAR_TOLERANCE = 0.05  # Q09.2
KINEMATIC_TOLERANCE = 0.02  # Q09.3 (stationäre Annahme: v̇y ≈ 0, v ≈ const)
LOAD_TOLERANCE = 1e-6  # Q09.4 (algebraische Handformel)


def _row(benchmark, quantity, simulated, reference, tolerance, unit):
    error = abs(simulated - reference) / max(abs(reference), 1e-12)
    return {
        'benchmark': benchmark,
        'quantity': quantity,
        'unit': unit,
        'simulated': float(simulated),
        'reference': float(reference),
        'relative_error': float(error),
        'tolerance': tolerance,
        'passed': bool(error <= tolerance),
    }


def _circle_radius(xs, ys) -> float:
    """Kreisausgleich (Kåsa) durch die Schwerpunktbahn."""
    a = np.column_stack((xs, ys, np.ones_like(xs)))
    b = xs**2 + ys**2
    cx, cy, c = np.linalg.lstsq(a, b, rcond=None)[0]
    return float(np.sqrt(c + (cx / 2) ** 2 + (cy / 2) ** 2))


def steady_cornering(
    car: VehicleParameters,
    tyre: TyreParameters,
    wheel: WheelParameters,
    tyre_model,
    *,
    speed_ms: float,
    steer_deg: float,
    duration_s: float,
) -> list[dict]:
    """B1: lineare stationäre Kreisfahrt (ohne Lastverlagerung und Fahrwiderstände, konstante Geschwindigkeit)."""
    linear_car = replace(car, load_transfer=False, cd_area_m2=0.0, rolling_coefficient=0.0)
    seq = simulate_torque(
        linear_car,
        tyre,
        TorqueControls(steer_deg=steer_deg, drive_nm=(0.0,) * 4),
        wheel,
        speed_ms=speed_ms,
        duration_s=duration_s,
        max_step=0.01,
        tyre_model=tyre_model,
    )
    final = seq[-1]
    m, g, a = linear_car.mass_kg, linear_car.gravity, linear_car.cg_to_front_m
    b, L = linear_car.wheelbase_m - a, linear_car.wheelbase_m
    axle = {'front': m * g * b / L / 2, 'rear': m * g * a / L / 2}  # statische Radlast je Rad (unabhängig)
    stiffness = {
        k: 2
        * nominal_cornering_stiffness(
            tyre.mu, fz, tyre.nominal_fz_n, tyre.load_exponent, tyre.lateral.b, tyre.lateral.c
        )
        for k, fz in axle.items()
    }
    ref = linear_bicycle_stationary(m, L, a, final.state.vx_ms, steer_deg, stiffness['front'], stiffness['rear'])
    tail = [s for s in seq if s.t_s >= duration_s - 2.0]
    radius = _circle_radius(np.array([s.state.x_m for s in tail]), np.array([s.state.y_m for s in tail]))
    v, r, ay = final.state.vx_ms, final.state.yaw_rate_rads, final.forces.ay_ms2
    if abs(ay) > 0.2 * g:
        raise ValueError('Reference case leaves the linear range |ay| <= 0.2 g')
    return [
        _row('B1 linear steady-state circle', 'yaw rate r', r, ref['yaw_rate_rads'], LINEAR_TOLERANCE, 'rad/s'),
        _row('B1 linear steady-state circle', 'lateral acceleration ay', ay, ref['ay_ms2'], LINEAR_TOLERANCE, 'm/s²'),
        _row('B1 linear steady-state circle', 'ay ≈ v·r', ay, v * r, KINEMATIC_TOLERANCE, 'm/s²'),
        _row(
            'B1 linear steady-state circle',
            'r ≈ v/R (R from path fit)',
            r,
            np.hypot(v, final.state.vy_ms) / radius,
            KINEMATIC_TOLERANCE,
            'rad/s',
        ),
    ]


def _hand_loads(car: VehicleParameters, ax: float, ay: float) -> dict:
    m, g, h = car.mass_kg, car.gravity, car.cg_height_m
    a, L, t = car.cg_to_front_m, car.wheelbase_m, car.track_m
    b = L - a
    front, rear = m * (g * b - ax * h) / L, m * (g * a + ax * h) / L
    d_front, d_rear = m * ay * h / t * b / L, m * ay * h / t * a / L
    return {
        'FL': (front - d_front) / 2,
        'FR': (front + d_front) / 2,
        'RL': (rear - d_rear) / 2,
        'RR': (rear + d_rear) / 2,
    }


def wheel_load_references(
    car: VehicleParameters, tyre: TyreParameters, wheel: WheelParameters, tyre_model
) -> list[dict]:
    """B2: statische, symmetrisch-dynamische und asymmetrisch-dynamische Radlasten gegen Handformeln."""
    rows = []
    cases = {
        'static': None,
        'symmetric (acceleration)': TorqueControls(0.0, (0.0, 0.0, 250.0, 250.0)),
        'asymmetric (cornering)': TorqueControls(4.0, (0.0, 0.0, 80.0, 80.0)),
    }
    for label, ctrl in cases.items():
        if ctrl is None:
            from torquevector.physics.load_transfer import wheel_loads

            loads, ax, ay = wheel_loads(car, 0.0, 0.0), 0.0, 0.0
        else:
            final = simulate_torque(
                car, tyre, ctrl, wheel, speed_ms=18.0, duration_s=3.0, tyre_model=tyre_model, load_solver='root'
            )[-1]
            loads = {n: w.fz_n for n, w in final.forces.wheels.items()}
            ax, ay = final.forces.ax_ms2, final.forces.ay_ms2
        hand = _hand_loads(car, ax, ay)
        rows += [_row(f'B2 wheel load {label}', f'Fz {n}', loads[n], hand[n], LOAD_TOLERANCE, 'N') for n in hand]
        rows.append(
            _row(
                f'B2 wheel load {label}',
                'ΣFz = m·g',
                sum(loads.values()),
                car.mass_kg * car.gravity,
                LOAD_TOLERANCE,
                'N',
            )
        )
    return rows


def run_reference_benchmarks(
    car: VehicleParameters, tyre: TyreParameters, wheel: WheelParameters, tyre_model
) -> list[dict]:
    """Alle Referenzmanöver; jede Zeile mit Simulation, Referenz, relativem Fehler, Toleranz und Ergebnis."""
    rows = []
    for speed, steer in ((20.0, 0.5), (15.0, 0.8)):
        rows += steady_cornering(car, tyre, wheel, tyre_model, speed_ms=speed, steer_deg=steer, duration_s=6.0)
    return rows + wheel_load_references(car, tyre, wheel, tyre_model)
