"""Analyseplots und mehrseitiger PDF-Bericht aus gespeicherter Telemetrie.

Keine Simulation: Eingaben sind Telemetriesequenzen (``TorqueSample``), gespeicherte
Kennzahlen/Diagnostik (einfache Dicts) und für Reifenkennlinien ein Reifenmodell.
Jede Funktion liefert eine Matplotlib-Figur; ``write_report`` speichert PNG je Abschnitt
und fasst alles in ``report.pdf`` zusammen.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import degrees
from typing import Callable, Mapping, Sequence

import numpy as np

from torquevector.domain.scenarios import ManoeuvreSpec
from torquevector.domain.telemetry import TorqueSample
from torquevector.domain.tyre import TyreParameters
from torquevector.domain.wheels import WHEEL_ORDER
from torquevector.physics.tyre_magic_formula import peak_grip

SECTIONS = (
    'overview',
    'kpi',
    'tracking',
    'allocation',
    'wheels',
    'friction_circle',
    'handling',
    'solver',
    'tyre',
    'road_map',
    'road_mu',
)
LABEL = 'SYNTHETIC | NOT VEHICLE VALIDATED'
# Reifegradklasse nach QUALITY_GATES Q12.5 (DEMO / CALIBRATED / VALIDATED); ohne Messvalidierung nie VALIDATED.
STATUS = 'Status: DEMO (synthetic vehicle, tyre calibrated on synthetic data) – not VALIDATED'
CONTROLLER_COLORS = {'off': '#6b6b6b', 'p': '#1f77b4', 'lqr': '#2ca02c', 'ltv': '#ff7f0e', 'nmpc': '#d62728'}
WHEEL_COLORS = {'FL': '#1f77b4', 'FR': '#ff7f0e', 'RL': '#2ca02c', 'RR': '#d62728'}


def _plt():
    import matplotlib

    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    return plt


def color(kind: str, index: int = 0) -> str:
    return CONTROLLER_COLORS.get(kind, f'C{index % 10}')


def _t(seq):
    return np.array([s.t_s for s in seq])


def _finish(fig, title):
    fig.suptitle(f'{title}\n{LABEL}  ·  {STATUS}', fontsize=12)
    return fig


@dataclass(frozen=True)
class RunContext:
    """Begleitinformationen eines gespeicherten Laufs (für Titel und Übersicht)."""

    run_id: str
    spec: ManoeuvreSpec
    duration_s: float
    fingerprint: str | None
    case: str | None


def overview_figure(ctx: RunContext, records: Mapping[str, dict]):
    plt = _plt()
    fig = plt.figure(figsize=(11.7, 8.3))
    fig.text(0.06, 0.92, 'torquevector – Simulationsbericht', fontsize=20, weight='bold')
    lines = [
        f'Run: {ctx.run_id}' + (f'   Case: {ctx.case}' if ctx.case else ''),
        f'Manoeuvre: {ctx.spec.kind}, {ctx.spec.speed_ms:g} m/s, duration {ctx.duration_s:g} s',
        f'Konfigurations-Fingerprint: {ctx.fingerprint or "–"}',
        '',
        'Status per controller:',
    ]
    for kind, entry in records.items():
        extra = ''
        if entry.get('vs_off'):
            extra = (
                f"   yaw rate {entry['vs_off']['yaw_improvement_pct']:+.1f} %,"
                f" path {entry['vs_off']['path_improvement_pct']:+.1f} % vs OFF"
            )
        reason = f"   ({entry.get('reason', '')[:70]})" if entry['status'] != 'PASS' and 'reason' in entry else ''
        lines.append(f'  {kind.upper():5s} {entry["status"]}{extra}{reason}')
    lines += [
        '',
        'Note: synthetic simulation. No vehicle or tyre validation.',
        'Q10 REQUIRED_REAL_DATA, Q11 BLOCKED. Verbesserungen positiv = besser als OFF.',
    ]
    fig.text(0.06, 0.85, '\n'.join(lines), fontsize=11, va='top', family='monospace')
    return fig


def kpi_figure(records: Mapping[str, dict]):
    plt = _plt()
    passed = {k: v['metrics'] for k, v in records.items() if 'metrics' in v}
    metrics = [
        ('yaw_rmse_rads', 'Yaw-rate RMSE [rad/s]', 1.0),
        ('cross_rmse_m', 'Path RMSE [m]', 1.0),
        ('max_abs_beta_deg', 'max. |side-slip angle| [deg]', 1.0),
        ('max_tyre_utilization', 'max. tyre utilisation [-]', 1.0),
        ('diff_torque_rms_nm', 'RMS rear torque difference [Nm]', 1.0),
        ('max_abs_slip', 'max. |slip ratio| [-]', 1.0),
    ]
    fig, axs = plt.subplots(2, 4, figsize=(15, 7.5), constrained_layout=True)
    kinds = list(passed)
    for ax, (key, title, _) in zip(axs.flat, metrics):
        values = [passed[k].get(key) for k in kinds]
        ax.bar(
            [k.upper() for k in kinds],
            [np.nan if v is None else v for v in values],
            color=[color(k, i) for i, k in enumerate(kinds)],
        )
        ax.set_title(title, fontsize=10)
        ax.grid(axis='y', alpha=0.3)
    for ax, key, title in (
        (axs[1, 2], 'yaw_improvement_pct', 'Yaw-rate improvement vs OFF [%]'),
        (axs[1, 3], 'path_improvement_pct', 'Path improvement vs OFF [%]'),
    ):
        rows = [(k, records[k]['vs_off'][key]) for k in kinds if records[k].get('vs_off')]
        if rows:
            ax.bar(
                [k.upper() for k, _ in rows], [v for _, v in rows], color=[color(k, i) for i, (k, _) in enumerate(rows)]
            )
            ax.axhline(0, color='k', lw=0.8)
        else:
            ax.text(
                0.5,
                0.5,
                'no OFF comparison\n(no path manoeuvre or OFF missing)',
                ha='center',
                va='center',
                transform=ax.transAxes,
            )
        ax.set_title(title, fontsize=10)
        ax.grid(axis='y', alpha=0.3)
    return _finish(fig, 'Metrics per controller')


def tracking_figure(
    seqs: Mapping[str, Sequence[TorqueSample]], spec: ManoeuvreSpec, diagnostics: Mapping[str, list] | None = None
):
    plt = _plt()
    fig, axs = plt.subplots(2, 2, figsize=(14, 8.5), constrained_layout=True)
    for i, (kind, seq) in enumerate(seqs.items()):
        c = color(kind, i)
        t = _t(seq)
        yaw = np.array([s.state.yaw_rate_rads for s in seq])
        refs = [spec.reference_at(s.state) for s in seq]
        if refs[0] is not None:
            desired = np.array([s.state.vx_ms * r[0].curvature_inv_m for s, r in zip(seq, refs)])
            axs[0, 1].plot(t, np.degrees(yaw - desired), color=c, label=kind.upper())
            axs[1, 0].plot(t, [r[1] for r in refs], color=c, label=kind.upper())
            axs[1, 1].plot(t, np.degrees([r[2] for r in refs]), color=c, label=kind.upper())
            if i == 0:
                axs[0, 0].plot(t, np.degrees(desired), 'k--', lw=1.2, label='path reference v·κ')
        axs[0, 0].plot(t, np.degrees(yaw), color=c, label=kind.upper())
    for kind, ticks in (diagnostics or {}).items():
        refs = [(d['time_s'], d['reference_yaw_rads']) for d in ticks if 'reference_yaw_rads' in d]
        if refs:
            axs[0, 0].plot(
                *zip(*[(a, degrees(b)) for a, b in refs]),
                ':',
                color=color(kind),
                lw=1,
                label=f'{kind.upper()} controller reference (limited)',
            )
    axs[0, 0].set(title='Yaw rate', xlabel='t [s]', ylabel='r [deg/s]')
    axs[0, 1].set(title='Yaw-rate error to path reference', xlabel='t [s]', ylabel='[deg/s]')
    axs[1, 0].set(title='Lateral deviation e_y', xlabel='t [s]', ylabel='[m]')
    axs[1, 1].set(title='Heading error', xlabel='t [s]', ylabel='[deg]')
    for ax in axs.flat:
        ax.grid(alpha=0.3)
        if ax.lines:
            ax.legend(fontsize=8)
    if spec.path is None:
        for ax in (axs[0, 1], axs[1, 0], axs[1, 1]):
            ax.text(0.5, 0.5, 'no path manoeuvre', ha='center', va='center', transform=ax.transAxes)
    return _finish(fig, f'Tracking: {spec.kind}')


def allocation_figure(seqs: Mapping[str, Sequence[TorqueSample]], diagnostics: Mapping[str, list]):
    plt = _plt()
    fig, axs = plt.subplots(3, 2, figsize=(14, 10), constrained_layout=True)
    for i, (kind, seq) in enumerate(seqs.items()):
        c = color(kind, i)
        t = _t(seq)
        drive = np.array([s.drive_nm for s in seq])
        axs[1, 0].plot(t, drive[:, 3] - drive[:, 2], color=c, label=kind.upper())
        axs[1, 1].plot(t, drive[:, 1] - drive[:, 0], color=c, label=kind.upper())
        axs[2, 1].plot(t, [s.forces.mz_nm for s in seq], color=c, label=kind.upper())
        ticks = diagnostics.get(kind) or []
        if ticks:
            tt = [d['time_s'] for d in ticks]
            axs[0, 0].plot(tt, [d['requested_nm'] for d in ticks], color=c, label=f'{kind.upper()} requested')
            axs[0, 0].plot(
                tt,
                [d['allocated_estimate_nm'] for d in ticks],
                '--',
                color=c,
                label=f'{kind.upper()} allocated (estimate)',
            )
            axs[0, 1].plot(tt, [d['residual_nm'] for d in ticks], color=c, label=kind.upper())
    first = next(iter(seqs.values()), None)
    if first is not None:
        kind = next(iter(seqs))
        drive = np.array([s.drive_nm for s in first])
        for j, wheel in enumerate(WHEEL_ORDER):
            axs[2, 0].plot(_t(first), drive[:, j], color=WHEEL_COLORS[wheel], label=wheel)
        axs[2, 0].set_title(f'Wheel torques {kind.upper()} (per wheel)')
    axs[0, 0].set(title='Additional yaw moment: requested vs allocated', xlabel='t [s]', ylabel='[Nm]')
    axs[0, 1].set(title='Allocation residual (not realisable)', xlabel='t [s]', ylabel='[Nm]')
    axs[1, 0].set(title='Rear torque difference RR−RL', xlabel='t [s]', ylabel='[Nm]')
    axs[1, 1].set(title='Front torque difference FR−FL', xlabel='t [s]', ylabel='[Nm]')
    axs[2, 0].set(xlabel='t [s]', ylabel='[Nm]')
    axs[2, 1].set(title='Total yaw moment from tyre forces Mz', xlabel='t [s]', ylabel='[Nm]')
    for ax in axs.flat:
        ax.grid(alpha=0.3)
        if ax.lines:
            ax.legend(fontsize=7)
    if not any(diagnostics.values()):
        axs[0, 0].text(
            0.5, 0.5, 'no controller diagnostics stored', ha='center', va='center', transform=axs[0, 0].transAxes
        )
    return _finish(fig, 'Torque Allocation')


def wheel_figure(kind: str, seq: Sequence[TorqueSample]):
    plt = _plt()
    fig, axs = plt.subplots(2, 4, figsize=(19, 8), constrained_layout=True, sharex=True)
    t = _t(seq)
    deg = 180 / np.pi
    panels = [
        (lambda s, w, i: s.forces.wheels[w].fx_n, 'Longitudinal force Fx [N]'),
        (lambda s, w, i: s.forces.wheels[w].fy_n, 'Lateral force Fy [N]'),
        (lambda s, w, i: float(np.hypot(s.forces.wheels[w].fx_n, s.forces.wheels[w].fy_n)), 'Resultant force Fres [N]'),
        (lambda s, w, i: s.forces.wheels[w].fz_n, 'Wheel load Fz [N]'),
        (lambda s, w, i: s.forces.wheels[w].slip_ratio, 'Slip ratio κ [-]'),
        (lambda s, w, i: s.forces.wheels[w].slip_angle_rad * deg, 'Slip angle α [deg]'),
        (lambda s, w, i: s.forces.wheels[w].utilization, 'Friction utilisation [-]'),
        (lambda s, w, i: s.drive_nm[i] - s.brake_nm[i], 'Wheel torque drive − brake [Nm]'),
    ]
    for ax, (value, title) in zip(axs.flat, panels):
        for i, wheel in enumerate(WHEEL_ORDER):
            ax.plot(t, [value(s, wheel, i) for s in seq], color=WHEEL_COLORS[wheel], label=wheel)
        ax.set_title(title)
        ax.grid(alpha=0.3)
    axs[1, 2].axhline(1.0, color='k', ls='--', lw=0.8)
    for ax in axs[1]:
        ax.set_xlabel('t [s]')
    axs[0, 0].legend(ncol=4, fontsize=8)
    return _finish(fig, f'Wheel forces and slip – {kind.upper()}')


def friction_circle_figure(seqs: Mapping[str, Sequence[TorqueSample]]):
    plt = _plt()
    fig, axs = plt.subplots(2, 2, figsize=(10.5, 10), constrained_layout=True)
    phi = np.linspace(0, 2 * np.pi, 200)
    for ax, wheel in zip(axs.flat, WHEEL_ORDER):
        ax.plot(np.cos(phi), np.sin(phi), 'k--', lw=0.8, label='friction limit')
        for i, (kind, seq) in enumerate(seqs.items()):
            fx = [s.forces.wheels[wheel].fx_n / max(s.forces.wheels[wheel].peak_n, 1e-9) for s in seq]
            fy = [s.forces.wheels[wheel].fy_n / max(s.forces.wheels[wheel].peak_n, 1e-9) for s in seq]
            ax.plot(fy, fx, color=color(kind, i), lw=1.2, label=kind.upper())
            ax.plot(fy[-1:], fx[-1:], 'o', color=color(kind, i), ms=4)
        ax.set(
            title=f'{wheel}: normalised tyre forces',
            xlabel='Fy / (μ(Fz)·Fz)',
            ylabel='Fx / (μ(Fz)·Fz)',
            xlim=(-1.1, 1.1),
            ylim=(-1.1, 1.1),
            aspect='equal',
        )
        ax.grid(alpha=0.3)
    axs[0, 0].legend(fontsize=8)
    return _finish(fig, 'Friction circles per wheel')


def solver_figure(diagnostics: Mapping[str, list]):
    plt = _plt()
    predictive = {k: v for k, v in diagnostics.items() if v and 'solve_time_ms' in v[0]}
    fig, axs = plt.subplots(1, 3, figsize=(15, 4.8), constrained_layout=True)
    if not predictive:
        for ax in axs:
            ax.text(0.5, 0.5, 'no predictive controllers in this run', ha='center', va='center', transform=ax.transAxes)
        return _finish(fig, 'Solverstatistik')
    for i, (kind, ticks) in enumerate(predictive.items()):
        c = color(kind, i)
        times = [d['solve_time_ms'] for d in ticks]
        axs[0].plot([d['time_s'] for d in ticks], times, color=c, label=kind.upper())
        axs[1].plot([d['time_s'] for d in ticks], [d['iterations'] for d in ticks], color=c, label=kind.upper())
    axs[2].boxplot(
        [[d['solve_time_ms'] for d in v] for v in predictive.values()], tick_labels=[k.upper() for k in predictive]
    )
    axs[0].set(title='Solve time per control tick', xlabel='t [s]', ylabel='[ms]')
    axs[1].set(title='Solver iterations per tick', xlabel='t [s]', ylabel='[-]')
    axs[2].set(title='Solve-time distribution', ylabel='[ms]')
    for ax in axs:
        ax.grid(alpha=0.3)
    axs[0].legend(fontsize=8)
    axs[1].legend(fontsize=8)
    return _finish(fig, 'Solver statistics (measured on this machine, no real-time proof)')


def tyre_figure(tyre: TyreParameters, model: Callable, title: str, loads_n: Sequence[float] = (2000.0, 4000.0, 6000.0)):
    """Kennlinien des konfigurierten Reifenmodells ``model(kappa, alpha, fz, tyre)``."""
    plt = _plt()
    fig, axs = plt.subplots(2, 2, figsize=(13, 9.5), constrained_layout=True)
    kappa = np.linspace(-0.5, 0.5, 401)
    alpha = np.radians(np.linspace(-15, 15, 401))
    for i, fz in enumerate(loads_n):
        c = f'C{i}'
        axs[0, 0].plot(kappa, [model(float(k), 0.0, fz, tyre).fx_n for k in kappa], color=c, label=f'Fz={fz:.0f} N')
        axs[0, 1].plot(
            np.degrees(alpha), [model(0.0, float(a), fz, tyre).fy_n for a in alpha], color=c, label=f'Fz={fz:.0f} N'
        )
    fz = tyre.nominal_fz_n
    for j, k in enumerate((0.0, 0.02, 0.05, 0.1, 0.2)):
        res = [model(k, float(a), fz, tyre) for a in alpha]
        axs[1, 0].plot([r.fy_n for r in res], [r.fx_n for r in res], color=f'C{j}', label=f'κ={k:g}')
    peak = peak_grip(fz, tyre)
    phi = np.linspace(0, 2 * np.pi, 200)
    axs[1, 0].plot(peak * np.cos(phi), peak * np.sin(phi), 'k--', lw=0.8, label='μ(Fz)·Fz')
    loads = np.linspace(500.0, 8000.0, 200)
    axs[1, 1].plot(loads, [peak_grip(f, tyre) / f for f in loads], color='C3')
    axs[1, 1].axvline(tyre.nominal_fz_n, color='k', ls=':', lw=0.8, label='reference load')
    axs[0, 0].set(title='Longitudinal force vs slip ratio (α=0)', xlabel='κ [-]', ylabel='Fx [N]')
    axs[0, 1].set(title='Lateral force vs slip angle (κ=0)', xlabel='α [deg]', ylabel='Fy [N]')
    axs[1, 0].set(title=f'Combined slip at Fz={fz:.0f} N (α sweep per κ)', xlabel='Fy [N]', ylabel='Fx [N]')
    axs[1, 0].set_aspect('equal', adjustable='datalim')
    axs[1, 1].set(title='Load sensitivity μ_eff(Fz)', xlabel='Fz [N]', ylabel='μ_eff [-]')
    for ax in axs.flat:
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    return _finish(fig, f'Tyre curves: {title}')


def primary_controller(seqs, preferred: str | None = None) -> str:
    """Für Einzeldarstellungen: gewünschter Regler, sonst erster aktive Regler, sonst OFF."""
    if preferred:
        if preferred not in seqs:
            raise ValueError(f'Controller {preferred!r} not in telemetry ({", ".join(seqs)})')
        return preferred
    return next((k for k in seqs if k != 'off'), next(iter(seqs)))


def write_report(
    output_dir,
    *,
    ctx: RunContext,
    seqs: Mapping[str, Sequence[TorqueSample]],
    records: Mapping[str, dict],
    diagnostics: Mapping[str, list],
    sections: Sequence[str],
    tyre: tuple[TyreParameters, Callable, str] | None = None,
    road=None,
    handling_series=None,
    dpi: int = 120,
    formats: Sequence[str] = ('png',),
    prefix: str = '',
):
    """Speichert die gewählten Abschnitte als Einzelbilder und gemeinsam als ``report.pdf``."""
    from pathlib import Path
    from matplotlib.backends.backend_pdf import PdfPages

    plt = _plt()
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    unknown = set(sections) - set(SECTIONS)
    if unknown:
        raise ValueError(f'Unbekannte Berichtsabschnitte: {sorted(unknown)}')
    figures = []
    for section in sections:
        if section == 'overview':
            figures.append(('overview', overview_figure(ctx, records)))
        elif section == 'kpi':
            figures.append(('kpi', kpi_figure(records)))
        elif section == 'tracking':
            figures.append(('tracking', tracking_figure(seqs, ctx.spec, diagnostics)))
        elif section == 'allocation':
            figures.append(('allocation', allocation_figure(seqs, diagnostics)))
        elif section == 'wheels':
            figures += [(f'wheels_{kind}', wheel_figure(kind, seq)) for kind, seq in seqs.items()]
        elif section == 'friction_circle':
            figures.append(('friction_circle', friction_circle_figure(seqs)))
        elif section == 'handling' and handling_series:
            from torquevector.visualization.handling_plot import handling_timeseries_figure

            series = {k: {**v, 'color': color(k, i)} for i, (k, v) in enumerate(handling_series.items())}
            figures.append(('handling', handling_timeseries_figure(series)))
        elif section == 'solver':
            figures.append(('solver', solver_figure(diagnostics)))
        elif section == 'tyre' and tyre is not None:
            figures.append(('tyre', tyre_figure(*tyre)))
        elif section in ('road_map', 'road_mu') and road is not None and seqs:
            from torquevector.visualization.road_surface_plot import mu_timeseries_figure, road_map_figure

            field, contacts, car = road
            primary = primary_controller(seqs)
            title = f'{ctx.spec.kind} on {field.surface.id}'
            if section == 'road_map':
                figures.append(('road_map', road_map_figure(field, seqs, contacts, car, primary, title)))
            else:
                figures.append(('road_mu', mu_timeseries_figure(seqs, contacts, primary, title)))
    written = []
    pdf_path = output / f'{prefix}report.pdf'
    with PdfPages(pdf_path) as pdf:
        for name, fig in figures:
            for fmt in formats:
                path = output / f'{prefix}{name}.{fmt}'
                fig.savefig(path, dpi=dpi)
                written.append(path)
            pdf.savefig(fig)
            plt.close(fig)
    written.append(pdf_path)
    return written
