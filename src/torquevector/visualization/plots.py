"""Dashboard panels: vehicle with force arrows, per-wheel friction circles, trajectory and tyre curves."""

from __future__ import annotations
import numpy as np
from torquevector.visualization.vehicle_artwork import BG
from torquevector.visualization.vehicle_artwork import WHITE
from torquevector.visualization.vehicle_artwork import MUTED
from torquevector.visualization.vehicle_artwork import FX_COLOR
from torquevector.visualization.vehicle_artwork import FY_COLOR
from torquevector.visualization.vehicle_artwork import RES_COLOR
from torquevector.visualization.vehicle_artwork import WHEELS
from torquevector.visualization.vehicle_artwork import draw_car
from torquevector.visualization.vehicle_artwork import draw_road
from torquevector.visualization.vehicle_artwork import wheel_basis
from torquevector.visualization.vehicle_artwork import force_arrow
from torquevector.domain.wheels import WHEEL_ORDER
from torquevector.domain.telemetry import Sample
from matplotlib.patches import Circle
import matplotlib.pyplot as plt
from torquevector.domain.tyre import TyreParameters
from torquevector.physics.tyre_magic_formula import magic_formula
from math import degrees

PANEL = '#172334'


GOLD = '#F2C879'


def vehicle_panel(ax, sample: Sample, steer_deg: float, scale: float = 0.30) -> None:
    ax.clear()
    draw_road(ax)
    draw_car(ax, steer_deg)
    for name in WHEEL_ORDER:
        tyre = sample.forces.wheels[name]
        steer = steer_deg if name.startswith('F') else 0.0
        ex, ey = wheel_basis(steer)
        p0 = np.asarray(WHEELS[name], dtype=float)
        p1 = p0 + tyre.fx_n / 1000.0 * ex * scale
        p2 = p1 + tyre.fy_n / 1000.0 * ey * scale
        force_arrow(ax, p0, p2, RES_COLOR, 3.4, 7.0, alpha=0.95)
        force_arrow(ax, p0, p1, FX_COLOR, 2.4, 8.0)
        force_arrow(ax, p1, p2, FY_COLOR, 2.4, 8.0)
        side = -1 if name.endswith('L') else 1
        ax.text(
            WHEELS[name][0] + side * 0.47,
            WHEELS[name][1] - 0.66,
            f'{name}  {tyre.utilization * 100:.0f}%',
            fontsize=9.5,
            color=GOLD if tyre.clipped else WHITE,
            ha='center',
            va='center',
            fontweight='bold',
            zorder=12,
            bbox={
                'boxstyle': 'round,pad=0.25',
                'facecolor': BG,
                'edgecolor': GOLD if tyre.clipped else '#52677B',
                'linewidth': 0.9,
            },
        )
    ax.set_xlim(-3.55, 3.55)
    ax.set_ylim(-3.05, 3.13)
    ax.set_aspect('equal', adjustable='box')
    ax.axis('off')
    ax.text(0, 2.78, 'DRIVING DIRECTION  +x', ha='center', color=WHITE, fontsize=10.5, weight='bold')
    ax.annotate('', xy=(0, 3.02), xytext=(0, 2.88), arrowprops={'arrowstyle': '-|>', 'lw': 2, 'color': WHITE})
    ax.text(0, -2.92, 'ISO: +x forward  |  +y left  |  +delta left', color=MUTED, ha='center', fontsize=9)


def circle_panel(ax, name: str, sample: Sample) -> None:
    result = sample.forces.wheels[name]
    ax.clear()
    ax.set_facecolor(PANEL)
    ax.add_patch(Circle((0, 0), 1.0, facecolor='#213A3D', edgecolor='#7993A8', linewidth=1.35, alpha=0.92))
    ax.axhline(0, color='#97A9B7', alpha=0.28, lw=0.8)
    ax.axvline(0, color='#97A9B7', alpha=0.28, lw=0.8)
    if result.peak_n:
        x, y = result.fx_n / result.peak_n, result.fy_n / result.peak_n
        raw_x = result.fx_pure_n / result.peak_n
        raw_y = result.fy_pure_n / result.peak_n
    else:
        x = y = raw_x = raw_y = 0.0
    if result.clipped:
        ax.plot([0, raw_x], [0, raw_y], '--', color=GOLD, alpha=0.67, lw=1.5)
        ax.plot(raw_x, raw_y, marker='x', ms=7, color=GOLD, mew=1.7)
    ax.annotate(
        '',
        xy=(x, y),
        xytext=(0, 0),
        arrowprops={'arrowstyle': '-|>', 'color': RES_COLOR, 'lw': 2.6, 'mutation_scale': 13},
    )
    ax.scatter([x], [y], s=31, color=RES_COLOR, zorder=5)
    ax.set_xlim(-1.32, 1.32)
    ax.set_ylim(-1.28, 1.28)
    ax.set_aspect('equal', adjustable='box')
    ax.set_xticks([-1, 0, 1])
    ax.set_yticks([-1, 0, 1])
    ax.tick_params(colors=MUTED, labelsize=7.5, length=0)
    for spine in ax.spines.values():
        spine.set_visible(False)
    ax.set_title(
        f'{name}   {100 * result.utilization:.0f}%   Fz {result.fz_n / 1000:.2f} kN',
        color=GOLD if result.clipped else WHITE,
        fontsize=10,
        fontweight='bold',
        pad=6,
    )


def path_panel(ax, samples: list[Sample]) -> None:
    ax.clear()
    ax.set_facecolor(PANEL)
    x = [a.state.y_m for a in samples]  # vehicle-left on image horizontal axis
    y = [a.state.x_m for a in samples]  # global forward upwards
    ax.plot(x, y, color=FY_COLOR, linewidth=2.4)
    ax.scatter([x[0]], [y[0]], marker='o', s=19, color=WHITE, zorder=5)
    ax.scatter([x[-1]], [y[-1]], marker='o', s=36, color=RES_COLOR, zorder=5)
    ax.set_title('FAHRTRAJEKTORIE / CG', fontsize=10, color=WHITE, weight='bold', pad=8)
    ax.set_xlabel('World y: left [m]', color=MUTED, fontsize=9)
    ax.set_ylabel('Welt x [m]', color=MUTED, fontsize=9)
    ax.tick_params(colors=MUTED, labelsize=8)
    ax.grid(alpha=0.12)
    for spine in ax.spines.values():
        spine.set_color('#4D6070')
    ax.margins(0.13)


def header(fig, torque_mode: bool = False) -> None:
    fig.text(
        0.028,
        0.954,
        'PACEJKA  /  FRICTION CIRCLE  /  FOUR-WHEEL DYNAMICS',
        color=WHITE,
        fontsize=19,
        weight='bold',
        va='center',
    )
    fig.text(
        0.029,
        0.914,
        'Fx longitudinal force     Fy lateral force     Fres resultant    |'
        '    ' + ('Torque-driven demonstration model' if torque_mode else 'Slip-driven demonstration model'),
        color=MUTED,
        fontsize=10,
    )
    for label, color, x in (('Fx', FX_COLOR, 0.030), ('Fy', FY_COLOR, 0.085), ('Fres', RES_COLOR, 0.139)):
        fig.text(x, 0.872, '>>>', color=color, fontsize=13, weight='bold')
        fig.text(x + 0.021, 0.872, label, color=WHITE, fontsize=10)


def curves_figure(tyre: TyreParameters, fz_n: float = 4000.0):
    """Separate 1D pure-slip characteristic curves with given tire shape."""
    from torquevector.physics.tyre_magic_formula import peak_grip

    peak = peak_grip(fz_n, tyre)
    kappa = np.linspace(-0.3, 0.3, 400)
    alpha_deg = np.linspace(-20, 20, 400)
    fig, axes = plt.subplots(1, 2, figsize=(13, 5.5), facecolor=BG)
    for ax in axes:
        ax.set_facecolor(PANEL)
        ax.tick_params(colors=WHITE)
        ax.grid(alpha=0.16)
        for spine in ax.spines.values():
            spine.set_color('#415A6C')
    axes[0].plot(
        kappa * 100, [magic_formula(float(x), peak, tyre.longitudinal) / 1000 for x in kappa], color=FX_COLOR, lw=2.8
    )
    axes[0].set(xlabel='Slip ratio kappa [%]', ylabel='Fx [kN]')
    axes[1].plot(
        alpha_deg,
        [-magic_formula(float(np.deg2rad(x)), peak, tyre.lateral) / 1000 for x in alpha_deg],
        color=FY_COLOR,
        lw=2.8,
    )
    axes[1].set(xlabel='Schraeglaufwinkel alpha [deg]', ylabel='Fy [kN]')
    for ax in axes:
        ax.xaxis.label.set_color(WHITE)
        ax.yaxis.label.set_color(WHITE)
        ax.axhline(+peak / 1000, color=MUTED, lw=1, ls='--')
        ax.axhline(-peak / 1000, color=MUTED, lw=1, ls='--')
    fig.suptitle(
        f'MAGIC FORMULA (simplified pure-slip curves)   Fz={fz_n / 1000:.1f} kN',
        color=WHITE,
        fontsize=15,
        weight='bold',
    )
    fig.tight_layout(rect=(0, 0, 1, 0.91))
    return fig


def history_figure(samples: list[Sample]):
    """Vehicle dynamics time history: speed, lateral acceleration, yaw rate, yaw moment."""
    times = [s.t_s for s in samples]
    speed = [np.hypot(s.state.vx_ms, s.state.vy_ms) * 3.6 for s in samples]
    ay = [s.forces.ay_ms2 for s in samples]
    yaw_rate = [degrees(s.state.yaw_rate_rads) for s in samples]
    yaw_moment = [s.forces.mz_nm / 1000 for s in samples]
    fig, axes = plt.subplots(2, 2, figsize=(12.5, 8), facecolor=BG)
    series = (
        (speed, 'Speed [km/h]', WHITE),
        (ay, 'Querbeschleunigung ay [m/s2]', FY_COLOR),
        (yaw_rate, 'Yaw rate [deg/s]', RES_COLOR),
        (yaw_moment, 'Yaw moment Mz [kNm]', GOLD),
    )
    for ax, (values, title, color) in zip(axes.flat, series):
        ax.set_facecolor(PANEL)
        ax.plot(times, values, lw=2.4, color=color)
        ax.set_title(title, color=WHITE, fontsize=12, weight='bold')
        ax.set_xlabel('Time [s]', color=MUTED)
        ax.tick_params(colors=MUTED)
        ax.grid(alpha=0.15)
        for spine in ax.spines.values():
            spine.set_color('#425468')
    fig.suptitle('FAHRDYNAMIK / ZEITVERLAEUFE', color=WHITE, fontsize=16, weight='bold')
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    return fig
