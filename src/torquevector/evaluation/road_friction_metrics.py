"""Kennzahlen für Fahrten über räumlich veränderliche Reibwerte (aus gespeicherter Telemetrie)."""

from __future__ import annotations

from math import atan2, degrees, sqrt
from typing import Mapping, Sequence

from torquevector.domain.scenarios import ManoeuvreSpec
from torquevector.domain.telemetry import TorqueSample
from torquevector.domain.wheels import WHEEL_ORDER


def contact_record(t_s: float, contacts: Mapping) -> dict:
    """Serialisierbare Fahrbahninformation eines Samples (Kontakte aus physics.road_friction.wheel_contacts)."""
    row = {'t_s': t_s}
    for name in WHEEL_ORDER:
        c = contacts[name]
        row[name] = {'x_m': c.x_m, 'y_m': c.y_m, 'road_mu': c.road_mu, 'tyre_mu': c.tyre_mu, 'surface_id': c.surface_id}
    return row


def road_metrics(
    spec: ManoeuvreSpec,
    seq: Sequence[TorqueSample],
    contacts: Sequence[dict],
    *,
    utilization_limit: float,
    slip_limit: float,
    brake_start_s: float | None = None,
    diagnostics: Sequence[dict] = (),
) -> dict:
    road = [[row[w]['road_mu'] for w in WHEEL_ORDER] for row in contacts]
    lateral = []
    for s in seq:
        reference = spec.reference_at(s.state)
        lateral.append(abs(reference[1]) if reference else abs(s.state.y_m))
    violations = sum(
        1
        for s in seq
        if max(w.utilization for w in s.forces.wheels.values()) > utilization_limit
        or max(abs(k) for k in s.kappa) > slip_limit
    )
    tv = [sqrt((s.drive_nm[3] - s.drive_nm[2]) ** 2 + (s.drive_nm[1] - s.drive_nm[0]) ** 2) for s in seq]
    metrics = {
        'min_road_mu': {w: min(r[i] for r in road) for i, w in enumerate(WHEEL_ORDER)},
        'max_left_right_mu_difference': max(max(abs(r[0] - r[1]), abs(r[2] - r[3])) for r in road),
        'time_on_reduced_mu_s': _time_reduced(seq, road),
        'max_lateral_deviation_m': max(lateral),
        'max_abs_beta_deg': max(abs(degrees(atan2(s.state.vy_ms, s.state.vx_ms))) for s in seq),
        'model_limit_violations': violations,
        'tv_differential_torque_rms_nm': sqrt(sum(v * v for v in tv) / len(tv)),
    }
    if diagnostics:
        requested = [d['requested_nm'] for d in diagnostics]
        metrics['requested_yaw_moment_rms_nm'] = sqrt(sum(v * v for v in requested) / len(requested))
    if brake_start_s is not None:
        start = min(seq, key=lambda s: abs(s.t_s - brake_start_s))
        end = seq[-1]
        metrics['braking'] = {
            'start_speed_ms': start.state.vx_ms,
            'end_speed_ms': end.state.vx_ms,
            'distance_m': end.state.x_m - start.state.x_m,
            'duration_s': end.t_s - start.t_s,
            'mean_deceleration_ms2': (start.state.vx_ms - end.state.vx_ms) / max(end.t_s - start.t_s, 1e-9),
        }
    return metrics


def _time_reduced(seq, road) -> dict:
    """Zeit je Rad auf Fahrbahn mit kleinerem Reibwert als dem Maximum der Fahrt."""
    top = max(max(r) for r in road)
    out = {}
    for i, w in enumerate(WHEEL_ORDER):
        total = 0.0
        for a, b, r in zip(seq[:-1], seq[1:], road[:-1]):
            if r[i] < top - 1e-12:
                total += b.t_s - a.t_s
        out[w] = total
    return out
