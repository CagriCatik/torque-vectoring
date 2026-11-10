"""Self-steering characteristic (understeer/oversteer) and handling limits.

Definitions (ISO 8855 / ISO 4138, road-wheel angle δ, wheelbase L, path radius R = v/r):

- **Steer excess** ``δ − L/R``: zero for a neutral-steer vehicle. Its slope over the lateral acceleration is the
  self-steering (understeer) gradient K [deg/g]: K > 0 understeer, K < 0 oversteer.
- **Linear reference**: single-track model with axle cornering stiffnesses C_f, C_r (independent analytic formulas,
  ``evaluation.benchmarks``): K = m/L·(b/C_f − a/C_r). Characteristic speed √(L/K) for K > 0, critical speed √(−L/K)
  for K < 0 (above it the linear vehicle is unstable).
- **Understeer index per time step**: |v·tan δ/L| − |r| [rad/s]; positive = the vehicle yaws less than a neutral-steer
  vehicle would with the same steering angle (understeer), negative = more (oversteer). The magnitude form stays
  continuous when steering and yaw rate change sign at slightly different times in transient manoeuvres.
- **Limit behaviour**: the axle whose mean friction utilisation reaches the saturation threshold first
  (front ⇒ limit understeer, "plough"; rear ⇒ limit oversteer, "spin").
- **Ideal g-g envelope**: for every direction in the (a_x, a_y) plane the largest acceleration whose quasi-static
  wheel loads still allow Σ µ(F_z,i)·F_z,i ≥ m·a. An upper bound (no yaw-moment balance, no aerodynamics).
"""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, degrees, hypot, isfinite, radians, sin, sqrt, tan
from typing import Sequence

import numpy as np

from torquevector.domain.telemetry import TorqueSample
from torquevector.domain.tyre import TyreParameters
from torquevector.domain.vehicle import VehicleParameters
from torquevector.domain.wheels import WHEEL_ORDER
from torquevector.evaluation.benchmarks import nominal_cornering_stiffness
from torquevector.physics.axle_tyre import TyreAxleScaling
from torquevector.physics.load_transfer import wheel_loads
from torquevector.physics.tyre_magic_formula import peak_grip


@dataclass(frozen=True)
class PhasePlaneSettings:
    speed_ms: float
    steer_deg: float
    sideslip_range_deg: float
    yaw_rate_range_rads: float
    grid_sideslip: int
    grid_yaw_rate: int
    duration_s: float
    settled_yaw_acceleration_rads2: float


@dataclass(frozen=True)
class HandlingSettings:
    speeds_ms: tuple[float, ...]
    controllers: tuple[str, ...]
    lateral_limit_estimate_factor: float
    ramp_margin_deg: float
    driver_max_wheel_torque_nm: float
    speed_kp_nm_per_ms: float
    linear_limit_g: float
    gradient_bands_g: tuple[float, ...]
    neutral_tolerance_deg_per_g: float
    saturation_utilisation: float
    stop_sideslip_deg: float
    gg_directions: int
    gg_brake_in_turn_speed_ms: float
    gg_brake_in_turn_nm: tuple[float, ...]
    gg_rear_brake_fraction: float
    gg_straight_drive_nm: tuple[float, ...]
    phase_plane: PhasePlaneSettings

    def __post_init__(self):
        bands = self.gradient_bands_g
        if list(bands) != sorted(bands) or len(bands) < 2 or bands[0] < 0:
            raise ValueError('gradient_bands_g must be ascending, non-negative, at least two edges')
        if not (0 < self.linear_limit_g < 1.5 and 0 < self.saturation_utilisation <= 1 and self.gg_directions >= 8):
            raise ValueError('Invalid handling analysis settings')


# ---------------------------------------------------------------- linear single-track reference


def axle_cornering_stiffness(car: VehicleParameters, tyre: TyreParameters, scaling: TyreAxleScaling):
    """Static axle cornering stiffnesses [N/rad] including per-axle scaling (λ_Ky·λ_µ scales B·C·D)."""
    m, g, a, L = car.mass_kg, car.gravity, car.cg_to_front_m, car.wheelbase_m
    b = L - a
    out = []
    for axle_load, scale in ((m * g * b / L, scaling.front), (m * g * a / L, scaling.rear)):
        per_wheel = nominal_cornering_stiffness(
            tyre.mu * scale.friction_scale,
            axle_load / 2,
            tyre.nominal_fz_n,
            tyre.load_exponent,
            tyre.lateral.b * scale.lateral_stiffness_scale,
            tyre.lateral.c,
        )
        out.append(2 * per_wheel)
    return tuple(out)


def linear_characteristic(
    car: VehicleParameters, tyre: TyreParameters, scaling: TyreAxleScaling, *, neutral_tolerance_deg_per_g: float
) -> dict:
    cf, cr = axle_cornering_stiffness(car, tyre, scaling)
    m, g, a, L = car.mass_kg, car.gravity, car.cg_to_front_m, car.wheelbase_m
    b = L - a
    k = m / L * (b / cf - a / cr)  # rad per m/s²
    result = {
        'front_cornering_stiffness_n_per_rad': cf,
        'rear_cornering_stiffness_n_per_rad': cr,
        'understeer_gradient_rad_per_ms2': k,
        'understeer_gradient_deg_per_g': degrees(k) * g,
        'classification': classify(degrees(k) * g, neutral_tolerance_deg_per_g),
        'characteristic_speed_ms': sqrt(L / k) if k > 0 else None,
        'critical_speed_ms': sqrt(-L / k) if k < 0 else None,
    }
    return result


def linear_yaw_gain(car: VehicleParameters, k_rad_per_ms2: float, speed_ms: float) -> float:
    """Steady-state yaw-rate gain r/δ [1/s] of the linear single-track model (∞ beyond the critical speed)."""
    denominator = car.wheelbase_m + k_rad_per_ms2 * speed_ms**2
    return speed_ms / denominator if denominator > 0 else float('inf')


def classify(gradient_deg_per_g: float, tolerance: float) -> str:
    if gradient_deg_per_g > tolerance:
        return 'understeer'
    if gradient_deg_per_g < -tolerance:
        return 'oversteer'
    return 'neutral'


# ---------------------------------------------------------------- per-time-step indicators


def axle_utilisation(sample: TorqueSample) -> tuple[float, float]:
    w = sample.forces.wheels
    return (w['FL'].utilization + w['FR'].utilization) / 2, (w['RL'].utilization + w['RR'].utilization) / 2


def understeer_index(sample: TorqueSample, car: VehicleParameters) -> float:
    """|v·tan δ/L| − |r| [rad/s] (> 0 understeer, < 0 oversteer)."""
    delta = radians(sample.steer_actual_deg or 0.0)
    v = hypot(sample.state.vx_ms, sample.state.vy_ms)
    return float(abs(v * tan(delta) / car.wheelbase_m) - abs(sample.state.yaw_rate_rads))


def handling_metrics(samples: Sequence[TorqueSample], car: VehicleParameters) -> dict:
    """Summary indicators of any run (added to the metrics of every simulated case)."""
    index = [understeer_index(s, car) for s in samples]
    front, rear = zip(*(axle_utilisation(s) for s in samples))
    return {
        'max_understeer_index_rads': max(0.0, max(index)),
        'max_oversteer_index_rads': max(0.0, -min(index)),
        'max_front_axle_utilisation': max(front),
        'max_rear_axle_utilisation': max(rear),
    }


# ---------------------------------------------------------------- quasi-steady-state ramp steer


def steady_state_curve(samples: Sequence[TorqueSample], car: VehicleParameters, *, start_s: float) -> dict:
    """Points of the handling diagram from a slow ramp steer (one entry per sample after ``start_s``)."""
    rows = []
    for s in samples:
        if s.t_s < start_s or abs(s.state.yaw_rate_rads) < 1e-4 or s.steer_actual_deg is None:
            continue
        v = hypot(s.state.vx_ms, s.state.vy_ms)
        r = s.state.yaw_rate_rads
        delta = radians(s.steer_actual_deg)
        front, rear = axle_utilisation(s)
        rows.append(
            (
                s.t_s,
                s.forces.ay_ms2 / car.gravity,  # lateral acceleration from the tyre forces (accelerometer) [g]
                degrees(delta),
                degrees(delta - car.wheelbase_m * r / v),  # steer excess δ − L/R
                degrees(atan2(s.state.vy_ms, s.state.vx_ms)),
                r / delta if abs(delta) > 1e-6 else float('nan'),
                front,
                rear,
            )
        )
    keys = ('t_s', 'ay_g', 'steer_deg', 'steer_excess_deg', 'sideslip_deg', 'yaw_gain_per_s', 'front_util', 'rear_util')
    return {k: np.array([row[i] for row in rows]) for i, k in enumerate(keys)}


def gradient(curve: dict, lo_g: float, hi_g: float) -> float | None:
    """Least-squares slope d(δ − L/R)/d a_y [deg/g] for |a_y| in [lo, hi) (left-hand ramp, δ > 0); None with too
    few points."""
    ay = np.abs(curve['ay_g'])
    mask = (ay >= lo_g) & (ay < hi_g)
    if mask.sum() < 10 or np.ptp(ay[mask]) < 0.25 * (hi_g - lo_g):
        return None
    return float(np.polyfit(ay[mask], curve['steer_excess_deg'][mask], 1)[0])


def characteristic(curve: dict, settings: HandlingSettings, *, stopped_by_sideslip: bool) -> dict:
    """Self-steering gradient per a_y band, linear gradient, lateral limit and limit behaviour."""
    if len(curve['ay_g']) < 10:
        raise ValueError('Too few quasi-steady samples for a handling characteristic')
    edges = settings.gradient_bands_g
    bands = []
    for lo, hi in zip(edges[:-1], edges[1:]):
        k = gradient(curve, lo, hi)
        bands.append(
            {
                'ay_g': [lo, hi],
                'gradient_deg_per_g': k,
                'classification': None if k is None else classify(k, settings.neutral_tolerance_deg_per_g),
            }
        )
    linear = gradient(curve, 0.05, settings.linear_limit_g)
    ay = np.abs(curve['ay_g'])
    sat = settings.saturation_utilisation
    first = {}
    for axle in ('front', 'rear'):
        hits = np.nonzero(curve[f'{axle}_util'] >= sat)[0]
        first[axle] = float(curve['t_s'][hits[0]]) if len(hits) else None
    if stopped_by_sideslip:
        limit = 'loss of stability (side slip exceeded)'
    elif first['front'] is not None and (first['rear'] is None or first['front'] <= first['rear']):
        limit = 'limit understeer (front axle saturates first)'
    elif first['rear'] is not None:
        limit = 'limit oversteer (rear axle saturates first)'
    else:
        limit = 'no axle saturated'
    i_max = int(np.argmax(ay))
    return {
        'linear_gradient_deg_per_g': linear,
        'linear_classification': None if linear is None else classify(linear, settings.neutral_tolerance_deg_per_g),
        'bands': bands,
        'max_lateral_acceleration_g': float(ay[i_max]),
        'steer_at_max_lateral_deg': float(abs(curve['steer_deg'][i_max])),
        'sideslip_at_max_lateral_deg': float(abs(curve['sideslip_deg'][i_max])),
        'front_saturation_time_s': first['front'],
        'rear_saturation_time_s': first['rear'],
        'limit_behaviour': limit,
        'stopped_by_sideslip': stopped_by_sideslip,
    }


# ---------------------------------------------------------------- handling limits


def gg_envelope(car: VehicleParameters, tyre: TyreParameters, scaling: TyreAxleScaling, directions: int) -> dict:
    """Ideal tyre-limited g-g envelope (upper bound): largest |a| per direction with Σ µ_i(F_z,i)·F_z,i ≥ m·|a|."""
    m, g = car.mass_kg, car.gravity
    tyres = {name: scaling.for_wheel(name).apply(tyre) for name in WHEEL_ORDER}

    def feasible(a, theta):
        loads = wheel_loads(car, a * cos(theta), a * sin(theta))
        return sum(peak_grip(loads[n], tyres[n]) for n in WHEEL_ORDER) >= m * a

    ax, ay = [], []
    for theta in np.linspace(0.0, 2 * np.pi, directions, endpoint=False):
        lo, hi = 0.0, 3.0 * g
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            lo, hi = (mid, hi) if feasible(mid, theta) else (lo, mid)
        ax.append(lo * cos(theta) / g)
        ay.append(lo * sin(theta) / g)
    return {'ax_g': ax, 'ay_g': ay, 'max_lateral_g': max(abs(v) for v in ay)}


def acceleration_points(samples: Sequence[TorqueSample], car: VehicleParameters) -> tuple[list, list]:
    """Achieved (a_x, a_y) of a run in g (body frame, from the tyre forces)."""
    return [s.forces.ax_ms2 / car.gravity for s in samples], [s.forces.ay_ms2 / car.gravity for s in samples]


def phase_plane_outcome(samples: Sequence[TorqueSample], *, stopped: bool, settled_rads2: float) -> str:
    """'stable' when the trajectory settles (|ṙ| < ``settled_rads2`` at the end), 'unstable' when it diverged or was
    stopped by the side-slip limit."""
    if stopped or len(samples) < 3:
        return 'unstable'
    r = [s.state.yaw_rate_rads for s in samples[-3:]]
    dt = samples[-1].t_s - samples[-3].t_s
    rate = abs(r[-1] - r[0]) / dt if dt > 0 else float('inf')
    return 'stable' if isfinite(rate) and rate < settled_rads2 else 'unstable'
