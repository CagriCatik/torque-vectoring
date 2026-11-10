"""Manoeuvre screening metrics and internal plausibility checks per manoeuvre type."""

from __future__ import annotations
import numpy as np
from torquevector.domain.scenarios import ManoeuvreSpec
from torquevector.domain.telemetry import TorqueSample


def compute_metrics(spec: ManoeuvreSpec, samples: list[TorqueSample]) -> dict:
    if len(samples) < 5:
        raise ValueError('At least 5 samples needed')
    a = np.asarray([s.state.yaw_rate_rads for s in samples])
    vy = np.asarray([s.state.vy_ms for s in samples])
    vx = np.asarray([s.state.vx_ms for s in samples])
    ax = np.asarray([s.forces.ax_ms2 for s in samples])
    ay = np.asarray([s.forces.ay_ms2 for s in samples])
    t = np.asarray([s.t_s for s in samples])
    x = np.asarray([s.state.x_m for s in samples])
    y = np.asarray([s.state.y_m for s in samples])
    utiliz = np.asarray([s.forces.wheels[n].utilization for s in samples for n in ('FL', 'FR', 'RL', 'RR')])
    refs = [spec.reference_at(s.state) for s in samples]
    e_y = np.asarray([q[1] for q in refs]) if spec.path else None
    epsi = np.asarray([q[2] for q in refs]) if spec.path else None
    tail = t >= 0.8 * t[-1]
    avg_yaw = float(np.mean(a[tail]))
    mean_speed = float(np.mean(vx[tail]))
    results = {
        'max_abs_ay_g': float(np.max(np.abs(ay)) / 9.81),
        'max_abs_ax_ms2': float(np.max(np.abs(ax))),
        'max_abs_yaw_rate_deg_s': float(np.max(np.abs(a)) * 180 / np.pi),
        'max_tyre_utilization': float(np.max(utiliz)),
        'min_forward_speed_ms': float(np.min(vx)),
        'max_speed_error_ms': float(np.max(np.abs(vx - spec.speed_ms))),
        'final_x_m': float(x[-1]),
        'final_y_m': float(y[-1]),
        'final_yaw_deg': float(samples[-1].state.yaw_rad * 180 / np.pi),
        'steady_yaw_rate_rads': avg_yaw,
        'steady_yaw_std_rads': float(np.std(a[tail])),
        'max_abs_sideslip_deg': float(np.max(np.abs(np.arctan2(vy, vx))) * 180 / np.pi),
        'finite_all_states': bool(np.all(np.isfinite([x, y, vx, vy, a, ax, ay]))),
    }
    if e_y is not None:
        results['cross_track_rmse_m'] = float(np.sqrt(np.mean(e_y * e_y)))
        results['cross_track_max_m'] = float(np.max(np.abs(e_y)))
        results['final_cross_track_m'] = float(e_y[-1])
        results['max_heading_error_deg'] = float(np.max(np.abs(epsi)) * 180 / np.pi)
    if spec.kind in ('circle_track', 'circle_open'):
        # Signed radius inferred from the steady-state yaw rate and speed.
        results['measured_radius_m'] = mean_speed / avg_yaw if abs(avg_yaw) > 1e-6 else None
        results['steady_yaw_coefficient_variation'] = float(np.std(a[tail]) / max(abs(avg_yaw), 1e-6))
        if spec.kind == 'circle_track':
            results['steady_circle_radius_error_m'] = float(np.mean(np.abs(e_y[tail])))
            results['steady_circle_radius_rmse_m'] = float(np.sqrt(np.mean(e_y[tail] ** 2)))
    if spec.kind == 'double_lane_change':
        path = spec.path
        # Only evaluate completed manoeuvre if actual position crosses return transition.
        results['completed_path'] = bool(x[-1] >= path.end_x_m)
        results['max_lateral_displacement_m'] = float(np.max(y))
        results['final_lateral_error_m'] = float(y[-1] - path.at_x(float(x[-1])).y_m)
    checks = {
        'finite_and_forward': results['finite_all_states'] and results['min_forward_speed_ms'] >= 3.0,
        'tyre_inside_kamm': results['max_tyre_utilization'] <= 1.0 + 1e-9,
        'speed_control': results['max_speed_error_ms'] <= 1.0,
    }
    if spec.kind == 'circle_open':
        checks['steady_yaw'] = results['steady_yaw_coefficient_variation'] < 0.06
        checks['radius_reported'] = results['measured_radius_m'] is not None
    elif spec.kind == 'circle_track':
        checks['steady_yaw'] = results['steady_yaw_coefficient_variation'] < 0.06
        checks['steady_radius_tracking'] = results['steady_circle_radius_rmse_m'] < 1.0
    elif spec.kind == 'double_lane_change':
        checks['completed_path'] = results['completed_path']
        checks['max_cross_track'] = results['cross_track_max_m'] < 1.0
        checks['final_lane_return'] = abs(results['final_lateral_error_m']) < 0.5
    elif spec.kind in ('step_steer', 'sine_steer'):
        checks['nonzero_yaw_response'] = results['max_abs_yaw_rate_deg_s'] > 0.1
    return {
        'kind': spec.kind,
        'status': 'PASS_INTERNAL_SCREENING' if all(checks.values()) else 'FAIL_INTERNAL_SCREENING',
        'checks': checks,
        'metrics': results,
        'warning': 'Not a standardized ISO test or an experimentally validated vehicle model.',
    }
