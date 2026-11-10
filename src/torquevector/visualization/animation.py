"""GIF animation comparing controllers on the same manoeuvre with full 2D planar vehicle motion."""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, degrees, hypot, sin
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np

from torquevector.domain.telemetry import TorqueSample
from torquevector.domain.vehicle import VehicleParameters
from torquevector.domain.wheels import WHEEL_ORDER
from torquevector.visualization.plots import circle_panel, vehicle_panel
from torquevector.visualization.report import CONTROLLER_COLORS
from torquevector.visualization.road_surface_plot import (
    FORCE_SCALE_M_PER_N,
    draw_forces,
    draw_ghost_vehicle,
    draw_vehicle,
    draw_velocity_arrow,
    vehicle_outline,
)
from torquevector.visualization.vehicle_artwork import TRACK


@dataclass(frozen=True)
class AnimationConfig:
    fps: int = 12
    max_frames: int = 140
    tail_seconds: float = 1.0
    dpi: int = 110
    figure_size_in: tuple[float, float] = (11.5, 8.0)
    colors: tuple[str, ...] | None = None

    def __post_init__(self):
        if not (
            4 <= self.fps <= 30
            and 10 <= self.max_frames <= 400
            and 0 <= self.tail_seconds <= 5
            and 60 <= self.dpi <= 200
        ):
            raise ValueError('Invalid animation settings')
        if len(self.figure_size_in) != 2 or not all(0 < v <= 60 for v in self.figure_size_in):
            raise ValueError('Invalid animation figure size')
        if self.colors is not None and not self.colors:
            raise ValueError('Animation colors must be None or non-empty')


def _aligned_length(seqs: Mapping[str, Sequence[TorqueSample]]) -> int:
    if not seqs:
        raise ValueError('No sequences')
    n = min(len(v) for v in seqs.values())
    if n < 2:
        raise ValueError('Need at least two samples per sequence')
    return n


def _select_primary(names: Sequence[str], preferred: str | None) -> str:
    if preferred is not None:
        if preferred not in names:
            raise ValueError(f'Controller {preferred!r} not in telemetry ({", ".join(names)})')
        return preferred
    for candidate in ('lqr', 'nmpc', 'ltv', 'p'):
        if candidate in names:
            return candidate
    return names[0]


def _controller_color(name: str, index: int, custom: Sequence[str] | None, cycle: Sequence[str]) -> str:
    if custom:
        return custom[index % len(custom)]
    if name in CONTROLLER_COLORS:
        return CONTROLLER_COLORS[name]
    return cycle[index % len(cycle)]


def _arrays(samples: Sequence[TorqueSample], count: int) -> dict[str, np.ndarray]:
    ss = samples[:count]
    t = np.array([s.t_s for s in ss], float)
    x = np.array([s.state.x_m for s in ss], float)
    y = np.array([s.state.y_m for s in ss], float)
    yaw = np.array([s.state.yaw_rate_rads * 180 / np.pi for s in ss], float)
    beta = np.array([degrees(atan2(s.state.vy_ms, max(s.state.vx_ms, 1e-9))) for s in ss], float)
    util = np.array([max(w.utilization for w in s.forces.wheels.values()) for s in ss], float)
    dyaw = np.array([s.drive_nm[3] - s.drive_nm[2] for s in ss], float)
    return {'t': t, 'x': x, 'y': y, 'yaw': yaw, 'beta': beta, 'util': util, 'dyaw': dyaw}


def _world_contact_points(state, car: VehicleParameters) -> dict[str, dict[str, float]]:
    c, s = cos(state.yaw_rad), sin(state.yaw_rad)
    return {
        name: {'x_m': state.x_m + c * lx - s * ly, 'y_m': state.y_m + s * lx + c * ly}
        for name, (lx, ly) in car.contact_points().items()
    }


def _bounds(values: np.ndarray, minimum: float) -> tuple[float, float]:
    lo = float(values.min())
    hi = float(values.max())
    span = max(minimum, hi - lo)
    pad = 0.12 * span
    return lo - pad, hi + pad


def _draw_follow_camera(
    ax,
    *,
    sample: TorqueSample,
    off_sample: TorqueSample | None,
    car: VehicleParameters,
    primary_name: str,
    half_x: float,
    half_y: float,
    dynamic: list,
) -> None:
    for art in dynamic:
        art.remove()
    dynamic.clear()

    state = sample.state
    if off_sample is not None and primary_name != 'off':
        if (
            abs(off_sample.state.x_m - state.x_m) < half_x + 4.0
            and abs(off_sample.state.y_m - state.y_m) < half_y + 4.0
        ):
            dynamic.extend(draw_ghost_vehicle(ax, off_sample.state, car))

    steer_deg = sample.steer_actual_deg or 0.0
    dynamic.extend(draw_vehicle(ax, state, car, steer_deg, zorder=6))
    dynamic.extend(draw_forces(ax, sample, _world_contact_points(state, car), zorder=9))
    dynamic.extend(draw_velocity_arrow(ax, state, zorder=8))

    ax.set_xlim(state.x_m - half_x, state.x_m + half_x)
    ax.set_ylim(state.y_m - half_y, state.y_m + half_y)
    speed = hypot(state.vx_ms, state.vy_ms)
    beta_deg = degrees(atan2(state.vy_ms, max(state.vx_ms, 1e-9)))
    ax.set_title(
        f'2D Follow Camera ({primary_name.upper()}) | v={speed:.1f} m/s  '
        f'δ={steer_deg:+.1f}°  β={beta_deg:+.1f}°\n'
        f'Tyre forces: navy (1 m = {1 / FORCE_SCALE_M_PER_N:.0f} N)  |  Velocity: cyan',
        fontsize=8.5,
        pad=5,
    )


def render_controller_comparison_gif(
    seqs: Mapping[str, Sequence[TorqueSample]],
    output_path: str | Path,
    *,
    title: str,
    config: AnimationConfig = AnimationConfig(),
    car: VehicleParameters | None = None,
    primary: str | None = None,
) -> Path:
    """Render a multi-panel 2D planar vehicle and controller comparison GIF from stored telemetry."""
    import matplotlib

    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import ticker
    from matplotlib.animation import FuncAnimation, PillowWriter
    from matplotlib.patches import FancyBboxPatch, Polygon, Rectangle

    vehicle = car or VehicleParameters()
    names = list(seqs)
    primary_name = _select_primary(names, primary)
    count = _aligned_length(seqs)
    data = {name: _arrays(samples, count) for name, samples in seqs.items()}
    t = data[primary_name]['t']

    step = max(1, int(np.ceil(count / config.max_frames)))
    frames = list(range(0, count, step))
    if frames[-1] != count - 1:
        frames.append(count - 1)
    hold = int(round(config.tail_seconds * config.fps))
    frames.extend([count - 1] * max(0, hold))

    all_x = np.concatenate([d['x'] for d in data.values()])
    all_y = np.concatenate([d['y'] for d in data.values()])
    all_yaw = np.concatenate([d['yaw'] for d in data.values()])
    all_beta = np.concatenate([d['beta'] for d in data.values()])
    all_util = np.concatenate([d['util'] for d in data.values()])
    all_dyaw = np.concatenate([d['dyaw'] for d in data.values()])

    span_x = max(6.0, float(all_x.max() - all_x.min()))
    span_y = max(3.0, float(all_y.max() - all_y.min()))
    wide_map = (span_x / span_y) > 4.5

    fig = plt.figure(figsize=tuple(config.figure_size_in), facecolor='#ffffff')
    fig.suptitle(title, fontsize=11, fontweight='bold', y=0.975)

    if wide_map:
        strip_ratio = max(1.15, min(2.0, 13.0 * (span_y + 5.5) / (span_x + 8.0)))
        outer = fig.add_gridspec(
            3,
            1,
            height_ratios=(strip_ratio, 3.65, 2.25),
            left=0.055,
            right=0.985,
            top=0.915,
            bottom=0.088,
            hspace=0.40,
        )
        ax_xy = fig.add_subplot(outer[0])
        mid = outer[1].subgridspec(1, 3, width_ratios=(1.15, 0.92, 0.98), wspace=0.22)
        ax_cam = fig.add_subplot(mid[0])
        ax_car = fig.add_subplot(mid[1])
        circle_grid = mid[2].subgridspec(2, 2, hspace=0.45, wspace=0.22)
        bot = outer[2].subgridspec(1, 4, wspace=0.33)
        half_x, half_y = 5.4, 3.4
    else:
        outer = fig.add_gridspec(
            2,
            1,
            height_ratios=(3.75, 2.35),
            left=0.055,
            right=0.985,
            top=0.91,
            bottom=0.088,
            hspace=0.38,
        )
        top_row = outer[0].subgridspec(1, 4, width_ratios=(0.98, 1.04, 0.95, 1.03), wspace=0.25)
        ax_xy = fig.add_subplot(top_row[0])
        ax_cam = fig.add_subplot(top_row[1])
        ax_car = fig.add_subplot(top_row[2])
        circle_grid = top_row[3].subgridspec(2, 2, hspace=0.45, wspace=0.22)
        bot = outer[1].subgridspec(1, 4, wspace=0.33)
        half_x, half_y = 4.9, 4.3

    ax_yaw, ax_beta, ax_dt, ax_util = (fig.add_subplot(bot[i]) for i in range(4))
    circle_axes = {
        name: fig.add_subplot(circle_grid[row, col])
        for name, row, col in (('FL', 0, 0), ('FR', 0, 1), ('RL', 1, 0), ('RR', 1, 1))
    }

    cycle = plt.rcParams['axes.prop_cycle'].by_key().get('color', []) or ['C0', 'C1', 'C2', 'C3', 'C4']
    color_map = {name: _controller_color(name, i, config.colors, cycle) for i, name in enumerate(names)}

    padx = max(4.0, 0.05 * span_x)
    pady = max(2.8, 0.12 * span_y)
    ax_xy.set(
        title='2D Trajectory & Footprints',
        xlabel='x [m]',
        ylabel='y [m]',
        xlim=(float(all_x.min()) - padx, float(all_x.max()) + padx),
        ylim=(float(all_y.min()) - pady, float(all_y.max()) + pady),
    )
    ax_xy.title.set_fontsize(9.5)
    ax_xy.set_aspect('equal', adjustable='datalim')

    ax_cam.set_facecolor('#f1f5f9')
    ax_cam.set_aspect('equal')
    ax_cam.xaxis.set_major_locator(ticker.MultipleLocator(2.0))
    ax_cam.yaxis.set_major_locator(ticker.MultipleLocator(2.0))
    ax_cam.grid(True, which='both', color='#cbd5e1', lw=0.6, alpha=0.8)
    ax_cam.set_xlabel('x [m]', fontsize=8)
    ax_cam.set_ylabel('y [m]', fontsize=8)
    ax_cam.tick_params(labelsize=7)

    cam_box = Rectangle((0, 0), 2 * half_x, 2 * half_y, fill=False, edgecolor='#0f172a', ls='--', lw=0.9, zorder=10)
    ax_xy.add_patch(cam_box)

    artists = {}
    map_cars = {}
    for name in names:
        c = color_map[name]
        d = data[name]
        lw_main = 2.0 if name == primary_name else 1.4
        ax_xy.plot(d['x'], d['y'], lw=1.0, alpha=0.18, color=c)
        ax_cam.plot(
            d['x'],
            d['y'],
            lw=1.6 if name == primary_name else 1.0,
            alpha=0.55 if name == primary_name else 0.35,
            color=c,
        )
        (traj_line,) = ax_xy.plot([], [], lw=lw_main, label=name.upper(), color=c)
        ox0, oy0 = vehicle_outline(seqs[name][0].state, vehicle)
        body = Polygon(
            list(zip(ox0, oy0)),
            closed=True,
            facecolor=c,
            alpha=0.35 if name == primary_name else 0.18,
            edgecolor=c,
            lw=1.3 if name == primary_name else 0.9,
            zorder=8 if name == primary_name else 6,
        )
        ax_xy.add_patch(body)
        map_cars[name] = body

        for ax_ts, key in ((ax_yaw, 'yaw'), (ax_beta, 'beta'), (ax_dt, 'dyaw'), (ax_util, 'util')):
            ax_ts.plot(d['t'], d[key], lw=0.9, alpha=0.20, color=c)
        (yaw_line,) = ax_yaw.plot([], [], lw=lw_main, label=name.upper(), color=c)
        (yaw_dot,) = ax_yaw.plot([], [], marker='o', ms=3.5, ls='None', color=c)
        (beta_line,) = ax_beta.plot([], [], lw=lw_main, color=c)
        (beta_dot,) = ax_beta.plot([], [], marker='o', ms=3.5, ls='None', color=c)
        (dt_line,) = ax_dt.plot([], [], lw=lw_main, color=c)
        (dt_dot,) = ax_dt.plot([], [], marker='o', ms=3.5, ls='None', color=c)
        (util_line,) = ax_util.plot([], [], lw=lw_main, color=c)
        (util_dot,) = ax_util.plot([], [], marker='o', ms=3.5, ls='None', color=c)
        artists[name] = (
            traj_line,
            yaw_line,
            yaw_dot,
            beta_line,
            beta_dot,
            dt_line,
            dt_dot,
            util_line,
            util_dot,
        )

    ax_yaw.set(title='Yaw rate r', xlabel='t [s]', ylabel='r [deg/s]', xlim=(t[0], t[-1]), ylim=_bounds(all_yaw, 5.0))
    ax_beta.set(
        title='Side-slip angle β', xlabel='t [s]', ylabel='β [deg]', xlim=(t[0], t[-1]), ylim=_bounds(all_beta, 3.0)
    )
    ax_dt.set(
        title='Rear ΔDrive torque (RR − RL)',
        xlabel='t [s]',
        ylabel='ΔT [Nm]',
        xlim=(t[0], t[-1]),
        ylim=_bounds(all_dyaw, 20.0),
    )
    ax_util.set(
        title='Max. tyre friction utilisation',
        xlabel='t [s]',
        ylabel='max η [-]',
        xlim=(t[0], t[-1]),
        ylim=(0.0, max(1.08, float(all_util.max()) * 1.1)),
    )
    ax_util.axhline(1.0, color='#0f172a', ls='--', lw=0.9, label='Kamm limit')

    cursors = []
    for ax in (ax_yaw, ax_beta, ax_dt, ax_util):
        ax.grid(alpha=0.28)
        cursors.append(ax.axvline(t[0], color='#334155', ls='--', lw=0.8, alpha=0.6))
    ax_xy.grid(alpha=0.25)
    ax_xy.legend(
        loc='upper right' if wide_map else 'upper left',
        ncol=min(5, len(names)) if wide_map else min(2, len(names)),
        fontsize=7.0,
        framealpha=0.88,
    )
    ax_yaw.legend(loc='best', fontsize=7.0, framealpha=0.85)
    ax_util.legend(loc='best', fontsize=7.0, framealpha=0.85)

    footer = fig.text(
        0.5,
        0.013,
        '',
        ha='center',
        va='bottom',
        fontsize=9.5,
        family='monospace',
        color='#0f172a',
    )
    cam_dynamic: list = []

    def update(frame_idx: int):
        idx = min(frame_idx, count - 1)
        for name in names:
            d = data[name]
            sample_n = seqs[name][idx]
            (
                traj_line,
                yaw_line,
                yaw_dot,
                beta_line,
                beta_dot,
                dt_line,
                dt_dot,
                util_line,
                util_dot,
            ) = artists[name]
            traj_line.set_data(d['x'][: idx + 1], d['y'][: idx + 1])
            ox, oy = vehicle_outline(sample_n.state, vehicle)
            map_cars[name].set_xy(list(zip(ox, oy)))
            yaw_line.set_data(d['t'][: idx + 1], d['yaw'][: idx + 1])
            yaw_dot.set_data([d['t'][idx]], [d['yaw'][idx]])
            beta_line.set_data(d['t'][: idx + 1], d['beta'][: idx + 1])
            beta_dot.set_data([d['t'][idx]], [d['beta'][idx]])
            dt_line.set_data(d['t'][: idx + 1], d['dyaw'][: idx + 1])
            dt_dot.set_data([d['t'][idx]], [d['dyaw'][idx]])
            util_line.set_data(d['t'][: idx + 1], d['util'][: idx + 1])
            util_dot.set_data([d['t'][idx]], [d['util'][idx]])

        for cursor in cursors:
            cursor.set_xdata([t[idx], t[idx]])

        primary_sample = seqs[primary_name][idx]
        off_sample = seqs['off'][idx] if 'off' in seqs else None
        cam_box.set_xy((primary_sample.state.x_m - half_x, primary_sample.state.y_m - half_y))

        _draw_follow_camera(
            ax_cam,
            sample=primary_sample,
            off_sample=off_sample,
            car=vehicle,
            primary_name=primary_name,
            half_x=half_x,
            half_y=half_y,
            dynamic=cam_dynamic,
        )

        steer_deg = primary_sample.steer_actual_deg or 0.0
        vehicle_panel(ax_car, primary_sample, steer_deg)
        for txt in ax_car.texts:
            label_txt = txt.get_text()
            if 'DRIVING' in label_txt:
                txt.set_position((0.0, 2.48))
                txt.set_fontsize(7.2)
            elif 'ISO' in label_txt:
                txt.set_fontsize(6.8)
            elif label_txt != 'S':
                txt.set_fontsize(7.2)
            else:
                txt.set_fontsize(8.0)
        # Restore dark track backdrop inside ax_car since ax.axis('off') hides the axis background patch.
        ax_car.add_patch(
            FancyBboxPatch(
                (-3.55, -3.05),
                7.10,
                6.18,
                boxstyle='round,pad=0,rounding_size=0.22',
                facecolor=TRACK,
                edgecolor='#475569',
                lw=1.1,
                zorder=-5,
            )
        )
        ax_car.set_title(
            f'2D Tyre Forces ({primary_name.upper()})\nFx orange · Fy cyan · Fres lime',
            fontsize=8.0,
            fontweight='bold',
            color='#0f172a',
            pad=4,
        )

        for w_name in WHEEL_ORDER:
            ax_c = circle_axes[w_name]
            circle_panel(ax_c, w_name, primary_sample)
            clipped = primary_sample.forces.wheels[w_name].clipped
            ax_c.title.set_color('#b45309' if clipped else '#0f172a')
            ax_c.title.set_fontsize(7.4)
            ax_c.title.set_y(1.01)
            ax_c.tick_params(colors='#475569', labelsize=6.0)

        ay_g = primary_sample.forces.ay_ms2 / max(vehicle.gravity, 1e-6)
        ax_g = primary_sample.forces.ax_ms2 / max(vehicle.gravity, 1e-6)
        footer.set_text(
            f't = {t[idx]:.2f} s   |   Primary: {primary_name.upper()}   |   '
            f'ax = {ax_g:+.2f} g   ay = {ay_g:+.2f} g   Mz = {primary_sample.forces.mz_nm:+.0f} Nm'
        )
        return []

    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    anim = FuncAnimation(fig, update, frames=frames, interval=1000 / config.fps, blit=False, repeat=False)
    anim.save(output, writer=PillowWriter(fps=config.fps), dpi=config.dpi)
    plt.close(fig)
    return output
