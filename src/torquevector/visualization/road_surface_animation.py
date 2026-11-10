"""GIF: Fahrzeug auf farbiger Reibwertkarte mit Kraftvektoren und synchronen Zeitverläufen.

Panels: Draufsicht (Karte, Fahrzeug, vier Kraftvektoren, µ je Rad), Gierrate, Schwimmwinkel,
lokale Reibwerte, Radmomentverteilung, maximale Kraftschlussausnutzung – alle mit derselben
Simulationszeit. Headless (Agg + Pillow), keine erneute Simulation.
"""

from __future__ import annotations

from math import atan2, degrees
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np

from torquevector.domain.telemetry import TorqueSample
from torquevector.domain.vehicle import VehicleParameters
from torquevector.domain.wheels import WHEEL_ORDER
from torquevector.physics.road_friction import RoadFrictionMap
from torquevector.visualization.road_surface_plot import (
    CONTROLLER_COLORS,
    FORCE_SCALE_M_PER_N,
    LABEL,
    MU_CMAP,
    MU_RANGE,
    WHEEL_COLORS,
    draw_forces,
    draw_friction_map,
    draw_vehicle,
    map_extent,
    vehicle_outline,
    zoom_window,
)


def render_road_gif(
    field: RoadFrictionMap,
    seqs: Mapping[str, Sequence[TorqueSample]],
    contacts: Mapping[str, Sequence[dict]],
    car: VehicleParameters,
    primary: str,
    output,
    *,
    title: str,
    fps: int,
    max_frames: int,
    dpi: int,
    tail_s: float = 1.0,
) -> Path:
    import matplotlib

    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    seq, rows = seqs[primary], contacts[primary]
    n = len(seq)
    if n < 2:
        raise ValueError('At least two samples are required for an animation')
    t = np.array([s.t_s for s in seq])
    yaw = np.degrees([s.state.yaw_rate_rads for s in seq])
    beta = np.array([degrees(atan2(s.state.vy_ms, s.state.vx_ms)) for s in seq])
    mu = {w: np.array([r[w]['road_mu'] for r in rows]) for w in WHEEL_ORDER}
    drive = np.array([np.array(s.drive_nm) - np.array(s.brake_nm) for s in seq])
    util = np.array([max(w.utilization for w in s.forces.wheels.values()) for s in seq])
    step = max(1, int(np.ceil(n / max_frames)))
    frames = list(range(0, n, step)) + ([n - 1] if (n - 1) % step else []) + [n - 1] * int(round(tail_s * fps))

    extent = map_extent(seqs, margin_m=4.0)
    strip = 13.0 * (extent[3] - extent[2]) / (extent[1] - extent[0])
    fig = plt.figure(figsize=(14, strip + 8.2))
    grid = fig.add_gridspec(
        3,
        3,
        height_ratios=(strip + 0.5, 3.4, 2.6),
        hspace=0.5,
        wspace=0.28,
        left=0.06,
        right=0.97,
        top=0.93,
        bottom=0.07,
    )
    ax_map = fig.add_subplot(grid[0, :])
    ax_cam, ax_yaw, ax_beta = (fig.add_subplot(grid[1, i]) for i in range(3))
    ax_mu, ax_torque, ax_util = (fig.add_subplot(grid[2, i]) for i in range(3))
    image = draw_friction_map(ax_map, field, extent)
    ax_map.set_aspect('equal')
    fig.colorbar(image, ax=ax_map, label='road µ', fraction=0.02, pad=0.01)
    window0 = zoom_window(seq[0].state)
    cam_extent = (
        min(s.state.x_m for s in seq) - 6,
        max(s.state.x_m for s in seq) + 6,
        min(s.state.y_m for s in seq) - 4,
        max(s.state.y_m for s in seq) + 4,
    )
    draw_friction_map(ax_cam, field, cam_extent, resolution_m=0.1, labels=False)
    ax_cam.plot(
        [s.state.x_m for s in seq],
        [s.state.y_m for s in seq],
        color=CONTROLLER_COLORS.get(primary, 'k'),
        lw=1,
        alpha=0.6,
    )
    ax_cam.set(
        xlim=window0[:2],
        ylim=window0[2:],
        aspect='equal',
        title=f'Follow camera (true scale)\narrows: tyre force, 1 m ≙ {1 / FORCE_SCALE_M_PER_N:.0f} N',
    )
    ax_cam.tick_params(labelsize=7)
    for kind, other in seqs.items():
        if kind != primary:
            ax_map.plot(
                [s.state.x_m for s in other],
                [s.state.y_m for s in other],
                lw=0.8,
                alpha=0.5,
                color=CONTROLLER_COLORS.get(kind, 'k'),
                label=kind.upper(),
            )
    (path_line,) = ax_map.plot([], [], color=CONTROLLER_COLORS.get(primary, 'k'), lw=2, label=primary.upper())
    (body,) = ax_map.plot([], [], color='k', lw=1.2)
    mu_text = ax_cam.text(
        0.5,
        0.02,
        '',
        transform=ax_cam.transAxes,
        ha='center',
        va='bottom',
        fontsize=7,
        family='monospace',
        bbox=dict(fc='white', alpha=0.85, lw=0),
        zorder=20,
    )
    dynamic = []
    ax_map.set(
        xlim=extent[:2],
        ylim=extent[2:],
        xlabel='x [m]',
        ylabel='y [m]',
    )
    ax_map.set_title(f'{title} – {primary.upper()}', loc='left')
    ax_map.legend(loc='lower right', bbox_to_anchor=(1.0, 1.0), ncol=len(seqs), fontsize=7, frameon=False)

    def series(ax, y, ylabel, label, color='C0'):
        ax.plot(t, y, color=color, alpha=0.25, lw=1)
        (line,) = ax.plot([], [], color=color, lw=1.8, label=label)
        ax.set(xlim=(t[0], t[-1]), ylabel=ylabel, xlabel='t [s]')
        ax.grid(alpha=0.3)
        return line

    lines = [
        series(ax_yaw, yaw, 'r [deg/s]', 'yaw rate'),
        series(ax_beta, beta, 'β [deg]', 'side slip', 'C3'),
        series(ax_util, util, '[-]', 'max. utilisation', 'C4'),
    ]
    ax_yaw.set_title('Yaw rate')
    ax_beta.set_title('Side-slip angle')
    ax_util.set_title('Max. friction utilisation')
    ax_util.axhline(1, color='k', ls='--', lw=0.8)
    mu_lines = [series(ax_mu, mu[w], 'µ [-]', w, WHEEL_COLORS[w]) for w in WHEEL_ORDER]
    ax_mu.set_title('Local road µ per wheel')
    ax_mu.set_ylim(0, 1.05)
    ax_mu.legend(fontsize=7, ncol=2)
    torque_lines = [series(ax_torque, drive[:, i], '[Nm]', w, WHEEL_COLORS[w]) for i, w in enumerate(WHEEL_ORDER)]
    ax_torque.set_title('Wheel torques (drive − brake)')
    ax_torque.legend(fontsize=7, ncol=2)
    clock = fig.text(0.5, 0.015, '', ha='center', fontsize=10)
    fig.suptitle(LABEL, fontsize=9)

    def update(frame):
        i = min(frame, n - 1)
        sample, row = seq[i], rows[i]
        path_line.set_data([s.state.x_m for s in seq[: i + 1]], [s.state.y_m for s in seq[: i + 1]])
        body.set_data(*vehicle_outline(sample.state, car))
        for artist in dynamic:
            artist.remove()
        dynamic[:] = draw_vehicle(
            ax_cam, sample.state, car, sample.steer_actual_deg or 0.0, {w: row[w]['road_mu'] for w in WHEEL_ORDER}
        ) + draw_forces(ax_cam, sample, row)
        window = zoom_window(sample.state)
        ax_cam.set_xlim(*window[:2])
        ax_cam.set_ylim(*window[2:])
        mu_text.set_text('  '.join(f"{w} µ={row[w]['road_mu']:.2f}" for w in WHEEL_ORDER))
        for line, y in zip(lines, (yaw, beta, util)):
            line.set_data(t[: i + 1], y[: i + 1])
        for line, w in zip(mu_lines, WHEEL_ORDER):
            line.set_data(t[: i + 1], mu[w][: i + 1])
        for j, line in enumerate(torque_lines):
            line.set_data(t[: i + 1], drive[: i + 1, j])
        clock.set_text(f't = {t[i]:.2f} s')
        return []

    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    FuncAnimation(fig, update, frames=frames, interval=1000 / fps, blit=False).save(
        output, writer=PillowWriter(fps=fps), dpi=dpi
    )
    plt.close(fig)
    return output
