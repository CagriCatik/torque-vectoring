"""GIF animation comparing controllers on the same manoeuvre from stored telemetry."""

from __future__ import annotations
from dataclasses import dataclass
from typing import Mapping
from typing import Sequence
from torquevector.domain.telemetry import TorqueSample
import numpy as np
from pathlib import Path


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


def _arrays(samples: Sequence[TorqueSample], count: int) -> dict[str, np.ndarray]:
    ss = samples[:count]
    t = np.array([s.t_s for s in ss], float)
    x = np.array([s.state.x_m for s in ss], float)
    y = np.array([s.state.y_m for s in ss], float)
    yaw = np.array([s.state.yaw_rate_rads * 180 / np.pi for s in ss], float)
    beta = np.array([np.degrees(np.arctan2(s.state.vy_ms, max(s.state.vx_ms, 1e-9))) for s in ss], float)
    util = np.array([max(w.utilization for w in s.forces.wheels.values()) for s in ss], float)
    dyaw = np.array([s.drive_nm[3] - s.drive_nm[2] for s in ss], float)
    return {'t': t, 'x': x, 'y': y, 'yaw': yaw, 'beta': beta, 'util': util, 'dyaw': dyaw}


def render_controller_comparison_gif(
    seqs: Mapping[str, Sequence[TorqueSample]],
    output_path: str | Path,
    *,
    title: str,
    config: AnimationConfig = AnimationConfig(),
) -> Path:
    """Render a compact animated comparison GIF from controller sample sequences."""
    import matplotlib

    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    names = list(seqs)
    count = _aligned_length(seqs)
    data = {name: _arrays(samples, count) for name, samples in seqs.items()}
    ref = next(iter(data.values()))
    t = ref['t']
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

    padx = max(3.0, 0.08 * max(1.0, float(all_x.max() - all_x.min())))
    pady = max(2.0, 0.10 * max(1.0, float(all_y.max() - all_y.min())))

    fig, axs = plt.subplots(2, 2, figsize=tuple(config.figure_size_in), constrained_layout=True)
    ax_xy, ax_yaw, ax_beta, ax_util = axs[0, 0], axs[0, 1], axs[1, 0], axs[1, 1]
    fig.suptitle(title)

    colors = list(config.colors) if config.colors else plt.rcParams['axes.prop_cycle'].by_key().get('color', [])
    if not colors:
        colors = ['C0', 'C1', 'C2', 'C3', 'C4']

    artists: dict[str, tuple] = {}
    for i, name in enumerate(names):
        c = colors[i % len(colors)]
        d = data[name]
        ax_xy.plot(d['x'], d['y'], lw=1.1, alpha=0.18, color=c)
        (traj_line,) = ax_xy.plot([], [], lw=2.0, label=name.upper(), color=c)
        (traj_dot,) = ax_xy.plot([], [], marker='o', ms=5, linestyle='None', color=c)
        (yaw_line,) = ax_yaw.plot([], [], lw=2.0, label=name.upper(), color=c)
        (beta_line,) = ax_beta.plot([], [], lw=2.0, label=name.upper(), color=c)
        (util_line,) = ax_util.plot([], [], lw=2.0, label=name.upper(), color=c)
        artists[name] = (traj_line, traj_dot, yaw_line, beta_line, util_line)

    ax_xy.set(
        title='Trajectory',
        xlabel='x [m]',
        ylabel='y [m]',
        xlim=(all_x.min() - padx, all_x.max() + padx),
        ylim=(all_y.min() - pady, all_y.max() + pady),
    )
    ax_yaw.set(title='Yaw rate', xlabel='t [s]', ylabel='r [deg/s]', xlim=(t[0], t[-1]))
    ax_beta.set(title='Side-slip angle', xlabel='t [s]', ylabel='beta [deg]', xlim=(t[0], t[-1]))
    ax_util.set(title='ΔDrive torque / tyre utilisation', xlabel='t [s]', ylabel='Nm / [-]', xlim=(t[0], t[-1]))

    def _bounds(values: np.ndarray, minimum: float) -> tuple[float, float]:
        lo = float(values.min())
        hi = float(values.max())
        span = max(minimum, hi - lo)
        pad = 0.1 * span
        return lo - pad, hi + pad

    ax_yaw.set_ylim(*_bounds(all_yaw, 5.0))
    ax_beta.set_ylim(*_bounds(all_beta, 3.0))
    lo = min(float(all_dyaw.min()), 0.0)
    hi = max(float(all_dyaw.max()), 1.0)
    ax_util.set_ylim(min(lo - 0.1 * max(1.0, hi - lo), -10), max(1.05, hi + 0.1 * max(1.0, hi - lo)))
    util_right = ax_util.twinx()
    util_right.set_ylabel('max tire util [-]')
    util_right.set_ylim(0.0, max(1.05, float(all_util.max()) * 1.08))

    for ax in (ax_xy, ax_yaw, ax_beta, ax_util, util_right):
        ax.grid(alpha=0.25)
    for ax in (ax_xy, ax_yaw, ax_beta, ax_util):
        ax.legend(loc='best', fontsize=8)

    time_text = fig.text(0.5, 0.012, '', ha='center', va='bottom', fontsize=10)

    util_secondary = {}
    for i, name in enumerate(names):
        c = colors[i % len(colors)]
        (util_secondary[name],) = util_right.plot([], [], ls='--', lw=1.2, alpha=0.8, color=c)

    def init():
        time_text.set_text('')
        out = [time_text]
        for name in names:
            for art in artists[name]:
                art.set_data([], [])
                out.append(art)
            util_secondary[name].set_data([], [])
            out.append(util_secondary[name])
        return out

    def update(frame_idx: int):
        idx = min(frame_idx, count - 1)
        out = [time_text]
        for name in names:
            d = data[name]
            traj_line, traj_dot, yaw_line, beta_line, util_line = artists[name]
            traj_line.set_data(d['x'][: idx + 1], d['y'][: idx + 1])
            traj_dot.set_data([d['x'][idx]], [d['y'][idx]])
            yaw_line.set_data(d['t'][: idx + 1], d['yaw'][: idx + 1])
            beta_line.set_data(d['t'][: idx + 1], d['beta'][: idx + 1])
            util_line.set_data(d['t'][: idx + 1], d['dyaw'][: idx + 1])
            util_secondary[name].set_data(d['t'][: idx + 1], d['util'][: idx + 1])
            out.extend([traj_line, traj_dot, yaw_line, beta_line, util_line, util_secondary[name]])
        time_text.set_text(f't = {t[idx]:.2f} s')
        return out

    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    anim = FuncAnimation(
        fig, update, init_func=init, frames=frames, interval=1000 / config.fps, blit=False, repeat=False
    )
    anim.save(output, writer=PillowWriter(fps=config.fps), dpi=config.dpi)
    plt.close(fig)
    return output
