"""Figures of the handling analysis: handling diagram, yaw-rate gain, g-g diagram and β–r phase plane.

Input are plain arrays produced by ``evaluation.handling`` (no simulation here). Vehicle variants use a fixed
categorical order (validated palette, see the dataviz rules) and, as secondary encoding, a fixed line style.
"""

from __future__ import annotations

from typing import Mapping

import numpy as np

LABEL = 'SYNTHETIC | NOT VEHICLE VALIDATED'
VARIANT_COLORS = ('#2a78d6', '#eb6834', '#1baf7a')  # blue, orange, aqua – fixed order, never cycled
VARIANT_STYLES = ('-', '--', ':')
STABLE, UNSTABLE = '#2a78d6', '#eb6834'


def _plt():
    import matplotlib

    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    return plt


def _style(index: int) -> dict:
    return {'color': VARIANT_COLORS[index % len(VARIANT_COLORS)], 'ls': VARIANT_STYLES[index % len(VARIANT_STYLES)]}


def _finish(fig, title: str):
    fig.suptitle(f'{title}  –  {LABEL}', fontsize=12)
    for ax in fig.axes:
        ax.grid(alpha=0.3)
    return fig


def handling_diagram_figure(curves: Mapping[str, dict], linear: Mapping[str, float], *, title: str):
    """``curves``: variant → steady_state_curve arrays; ``linear``: variant → linear gradient [deg/g]."""
    plt = _plt()
    fig, axs = plt.subplots(2, 2, figsize=(13, 9.5), constrained_layout=True)
    for i, (name, c) in enumerate(curves.items()):
        st = _style(i)
        ay = c['ay_g']
        axs[0, 0].plot(ay, c['steer_excess_deg'], lw=2, label=name, **st)
        k = linear.get(name)
        if k is not None:
            grid = np.linspace(0, 0.4, 2)
            axs[0, 0].plot(grid, k * grid, lw=1, color=st['color'], alpha=0.6, ls='-.')
        axs[0, 1].plot(ay, c['sideslip_deg'], lw=2, label=name, **st)
        axs[1, 0].plot(ay, c['front_util'], lw=2, label=f'{name} front', **st)
        axs[1, 0].plot(ay, c['rear_util'], lw=1, alpha=0.7, label=f'{name} rear', color=st['color'], ls='-.')
        axs[1, 1].plot(ay, c['steer_deg'], lw=2, label=name, **st)
    axs[0, 0].axhline(0, color='k', lw=0.8)
    axs[0, 0].set(
        title='Handling diagram: δ − L/R over a_y (dash-dot: linear reference)',
        xlabel='lateral acceleration a_y [g]',
        ylabel='steer excess δ − L/R [deg]  (> 0 understeer)',
    )
    axs[0, 1].set(title='Side-slip angle', xlabel='a_y [g]', ylabel='β [deg]')
    axs[1, 0].axhline(1, color='k', ls='--', lw=0.8)
    axs[1, 0].set(title='Axle friction utilisation (solid front, dash-dot rear)', xlabel='a_y [g]', ylabel='[-]')
    axs[1, 1].set(title='Road-wheel steering angle', xlabel='a_y [g]', ylabel='δ [deg]')
    for ax in axs.flat:
        ax.legend(fontsize=8)
    return _finish(fig, title)


def yaw_gain_figure(linear: Mapping[str, list], measured: Mapping[str, list]):
    """``linear``: variant → [(v, r/δ)]; ``measured``: variant → [(v, r/δ at low a_y)]."""
    plt = _plt()
    fig, ax = plt.subplots(figsize=(9, 6), constrained_layout=True)
    for i, name in enumerate(linear):
        st = _style(i)
        v, gain = zip(*linear[name])
        gain = [np.nan if g is None or not np.isfinite(g) else g for g in gain]
        ax.plot(v, gain, lw=2, label=f'{name} (linear model)', **st)
        if measured.get(name):
            mv, mg = zip(*measured[name])
            ax.plot(mv, mg, 'o', ms=8, color=st['color'], mec='white', mew=2, label=f'{name} (simulation, a_y < 0.2 g)')
    ax.set(
        title='Steady-state yaw-rate gain r/δ (→ ∞ at the critical speed of an oversteering vehicle)',
        xlabel='speed v [m/s]',
        ylabel='r/δ [1/s]',
    )
    low_speed = [g for name in linear for v, g in linear[name] if v <= 20.0 and np.isfinite(g)]
    ax.set_ylim(0, 3 * max(low_speed) if low_speed else None)  # an oversteering gain diverges at v_crit
    ax.legend(fontsize=8)
    return _finish(fig, 'Yaw-rate gain')


def gg_figure(envelopes: Mapping[str, tuple]):
    """``envelopes``: variant → (ideal envelope dict, achieved a_x list, achieved a_y list)."""
    plt = _plt()
    names = list(envelopes)
    fig, axs = plt.subplots(1, len(names), figsize=(5.5 * len(names), 5.8), constrained_layout=True, squeeze=False)
    for i, (ax, name) in enumerate(zip(axs[0], names)):
        envelope, ax_pts, ay_pts = envelopes[name]
        st = _style(i)
        ex, ey = envelope['ax_g'] + envelope['ax_g'][:1], envelope['ay_g'] + envelope['ay_g'][:1]
        ax.plot(ey, ex, lw=2, label='ideal tyre-limited envelope', **st)
        if ax_pts:
            ax.scatter(ay_pts, ax_pts, s=4, alpha=0.35, color='#5f5e5a', label='achieved (simulation)')
        ax.set(title=name, xlabel='a_y [g]', ylabel='a_x [g]', aspect='equal', xlim=(-1.3, 1.3), ylim=(-1.3, 1.3))
        ax.axhline(0, color='k', lw=0.6)
        ax.axvline(0, color='k', lw=0.6)
        ax.legend(fontsize=8, loc='lower right')
    return _finish(fig, 'g-g diagram')


def phase_plane_figure(phases: Mapping[str, list], *, speed_ms: float, steer_deg: float):
    """``phases``: variant → [(β list [deg], r list [rad/s], outcome)]."""
    plt = _plt()
    names = list(phases)
    fig, axs = plt.subplots(1, len(names), figsize=(5.5 * len(names), 5.6), constrained_layout=True, squeeze=False)
    for ax, name in zip(axs[0], names):
        stable = unstable = 0
        for beta, r, outcome in phases[name]:
            if not beta:
                continue
            ok = outcome == 'stable'
            stable, unstable = stable + ok, unstable + (not ok)
            ax.plot(beta, r, lw=1.2, color=STABLE if ok else UNSTABLE, ls='-' if ok else '--')
            ax.plot(beta[0], r[0], 'o', ms=4, color=STABLE if ok else UNSTABLE)
        ax.plot([], [], color=STABLE, label=f'stable ({stable})')
        ax.plot([], [], color=UNSTABLE, ls='--', label=f'unstable / spin ({unstable})')
        ax.set(title=name, xlabel='side-slip angle β [deg]', ylabel='yaw rate r [rad/s]')
        ax.legend(fontsize=8)
    return _finish(fig, f'β–r phase plane, OFF, {speed_ms:g} m/s, constant road-wheel angle {steer_deg:g} deg')


def handling_timeseries_figure(series: Mapping[str, dict]):
    """Report section ``handling`` of a normal run: understeer index and axle utilisation over time per controller.

    ``series``: controller → {'t': …, 'understeer_index_rads': …, 'front': …, 'rear': …, 'color': …}.
    """
    plt = _plt()
    fig, axs = plt.subplots(2, 1, figsize=(12, 8), constrained_layout=True, sharex=True)
    for kind, d in series.items():
        axs[0].plot(d['t'], d['understeer_index_rads'], lw=1.6, color=d['color'], label=kind.upper())
        axs[1].plot(d['t'], d['front'], lw=1.6, color=d['color'], label=f'{kind.upper()} front')
        axs[1].plot(d['t'], d['rear'], lw=1.0, ls='--', color=d['color'], label=f'{kind.upper()} rear')
    axs[0].axhline(0, color='k', lw=0.8)
    axs[0].set(title='Understeer index |v·tan δ/L| − |r|  (> 0 understeer, < 0 oversteer)', ylabel='[rad/s]')
    axs[1].axhline(1, color='k', ls='--', lw=0.8)
    axs[1].set(title='Axle friction utilisation (solid front, dashed rear)', ylabel='[-]', xlabel='t [s]')
    axs[0].legend(fontsize=8, ncol=5)
    axs[1].legend(fontsize=7, ncol=6, loc='upper center', bbox_to_anchor=(0.5, -0.14))
    return _finish(fig, 'Self-steering behaviour')
