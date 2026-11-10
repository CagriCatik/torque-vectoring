"""CSV/JSON export of telemetry, including per-wheel quantities, road contact data and controller diagnostics."""

from __future__ import annotations
from csv import DictWriter
from pathlib import Path
from torquevector.domain.wheels import WHEEL_ORDER
from torquevector.domain.telemetry import Sample
from torquevector.domain.telemetry import TorqueSample


def export_csv(samples: list[Sample], filepath: str | Path) -> None:
    if not samples:
        raise ValueError('No samples')
    filepath = Path(filepath)
    filepath.parent.mkdir(parents=True, exist_ok=True)
    columns = [
        't_s',
        'x_m',
        'y_m',
        'yaw_deg',
        'vx_ms',
        'vy_ms',
        'yaw_rate_deg_s',
        'ax_ms2',
        'ay_ms2',
        'mz_nm',
        'fx_n',
        'fy_n',
    ]
    for name in WHEEL_ORDER:
        columns += [
            f'{name}_{field}' for field in ('fz_n', 'fx_n', 'fy_n', 'alpha_deg', 'kappa', 'utilization', 'clipped')
        ]
    with filepath.open('w', encoding='utf-8', newline='') as fh:
        writer = DictWriter(fh, fieldnames=columns)
        writer.writeheader()
        for item in samples:
            s, f = item.state, item.forces
            row = {
                't_s': item.t_s,
                'x_m': s.x_m,
                'y_m': s.y_m,
                'yaw_deg': s.yaw_rad * 180 / 3.141592653589793,
                'vx_ms': s.vx_ms,
                'vy_ms': s.vy_ms,
                'yaw_rate_deg_s': s.yaw_rate_rads * 180 / 3.141592653589793,
                'ax_ms2': f.ax_ms2,
                'ay_ms2': f.ay_ms2,
                'mz_nm': f.mz_nm,
                'fx_n': f.fx_n,
                'fy_n': f.fy_n,
            }
            for name in WHEEL_ORDER:
                w = f.wheels[name]
                row.update(
                    {
                        f'{name}_fz_n': w.fz_n,
                        f'{name}_fx_n': w.fx_n,
                        f'{name}_fy_n': w.fy_n,
                        f'{name}_alpha_deg': w.slip_angle_rad * 180 / 3.141592653589793,
                        f'{name}_kappa': w.slip_ratio,
                        f'{name}_utilization': w.utilization,
                        f'{name}_clipped': int(w.clipped),
                    }
                )
            writer.writerow(row)


def vehicle_samples(samples: list[TorqueSample]) -> list[Sample]:
    """Adapter: old dashboard cannot alter the new torque model."""
    return [Sample(s.t_s, s.state, s.forces) for s in samples]


def export_torque_csv(
    samples, filepath, *, contacts=None, reference_yaw=None, diagnostics=None, extra_columns=None
) -> None:
    """Telemetrie des Vier-Rad-Modells als CSV: Zustände, Radgrößen, lokale Reibwerte und Oberflächen.

    ``contacts``: Liste je Sample (aus evaluation.road_friction_metrics.contact_record);
    ``reference_yaw``: optionale Soll-Gierrate je Sample [rad/s];
    ``diagnostics``: Reglerdiagnose je Takt (time_s, requested_nm, allocated_estimate_nm, residual_nm,
    optional iterations, solve_time_ms). Zuordnung über den Taktzeitpunkt = Samplezeit (Beginn des
    Halteintervalls). ``solver_status``: SUCCESS je aufgezeichnetem Takt, NO_TICK ohne Reglertakt
    (OFF, Endzustand); gescheiterte Solves beenden den Lauf (FAIL_CLOSED) und erzeugen keine CSV.
    ``extra_columns``: weitere Spalten {name: Werte je Sample}, z. B. Handling-Indikatoren aus evaluation.handling.
    """
    from math import atan2

    filepath = Path(filepath)
    filepath.parent.mkdir(parents=True, exist_ok=True)
    columns = [
        't_s',
        'x_m',
        'y_m',
        'yaw_rad',
        'vx_ms',
        'vy_ms',
        'yaw_rate_rads',
        'yaw_rate_ref_rads',
        'beta_rad',
        'ax_ms2',
        'ay_ms2',
        'mz_nm',
        'steer_actual_deg',
        'yaw_moment_requested_nm',
        'yaw_moment_allocated_nm',
        'allocation_residual_nm',
        'yaw_moment_tyres_nm',
        'solver_status',
        'solver_iterations',
        'solve_time_ms',
    ]
    ticks = {round(d['time_s'], 9): d for d in (diagnostics or [])}
    for name in WHEEL_ORDER:
        n = name.lower()
        columns += [
            f'{n}_{k}'
            for k in (
                'fx_n',
                'fy_n',
                'fz_n',
                'alpha_rad',
                'kappa',
                'utilization',
                'omega_rad_s',
                'drive_nm',
                'brake_nm',
                'contact_x_m',
                'contact_y_m',
                'mu_tyre_eff',
            )
        ]
    columns += [f'mu_{n.lower()}' for n in WHEEL_ORDER] + [f'surface_id_{n.lower()}' for n in WHEEL_ORDER]
    extra = dict(extra_columns or {})
    if any(len(v) != len(samples) for v in extra.values()):
        raise ValueError('extra_columns need one value per sample')
    columns += list(extra)
    with filepath.open('w', encoding='utf-8', newline='') as fh:
        writer = DictWriter(fh, fieldnames=columns)
        writer.writeheader()
        for index, item in enumerate(samples):
            s, f = item.state, item.forces
            row = {
                't_s': item.t_s,
                'x_m': s.x_m,
                'y_m': s.y_m,
                'yaw_rad': s.yaw_rad,
                'vx_ms': s.vx_ms,
                'vy_ms': s.vy_ms,
                'yaw_rate_rads': s.yaw_rate_rads,
                'beta_rad': atan2(s.vy_ms, s.vx_ms),
                'ax_ms2': f.ax_ms2,
                'ay_ms2': f.ay_ms2,
                'mz_nm': f.mz_nm,
                'steer_actual_deg': item.steer_actual_deg,
                'yaw_rate_ref_rads': None if reference_yaw is None else reference_yaw[index],
                'yaw_moment_tyres_nm': f.mz_nm,
            }
            tick = ticks.get(round(item.t_s, 9))
            row['solver_status'] = 'SUCCESS' if tick else 'NO_TICK'
            if tick:
                row.update(
                    {
                        'yaw_moment_requested_nm': tick.get('requested_nm'),
                        'yaw_moment_allocated_nm': tick.get('allocated_estimate_nm'),
                        'allocation_residual_nm': tick.get('residual_nm'),
                        'solver_iterations': tick.get('iterations'),
                        'solve_time_ms': tick.get('solve_time_ms'),
                    }
                )
            for i, name in enumerate(WHEEL_ORDER):
                n, w = name.lower(), f.wheels[name]
                row.update(
                    {
                        f'{n}_fx_n': w.fx_n,
                        f'{n}_fy_n': w.fy_n,
                        f'{n}_fz_n': w.fz_n,
                        f'{n}_alpha_rad': w.slip_angle_rad,
                        f'{n}_kappa': item.kappa[i],
                        f'{n}_utilization': w.utilization,
                        f'{n}_omega_rad_s': item.omega_rad_s[i],
                        f'{n}_drive_nm': item.drive_nm[i],
                        f'{n}_brake_nm': item.brake_nm[i],
                    }
                )
                if contacts is not None:
                    c = contacts[index][name]
                    row.update(
                        {
                            f'{n}_contact_x_m': c['x_m'],
                            f'{n}_contact_y_m': c['y_m'],
                            f'{n}_mu_tyre_eff': c['tyre_mu'],
                            f'mu_{n}': c['road_mu'],
                            f'surface_id_{n}': c['surface_id'],
                        }
                    )
            row.update({name: values[index] for name, values in extra.items()})
            writer.writerow(row)
