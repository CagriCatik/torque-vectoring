"""Stored telemetry comparison plot; no simulation imports."""

from dataclasses import dataclass
import numpy as np

PANELS = ('trajectory', 'yaw_rate', 'sideslip', 'tyre_utilization', 'torque_allocation')


@dataclass(frozen=True)
class ChartStyle:
    """Darstellung, keine Physik. Default = bisheriges 2x2-Vergleichsbild."""

    panels: tuple[str, ...] = ('trajectory', 'yaw_rate', 'sideslip', 'tyre_utilization')
    figure_size_in: tuple[float, float] = (12.0, 8.0)
    dpi: int = 140
    formats: tuple[str, ...] = ('png',)
    legend: bool = True
    grid_alpha: float = 0.25
    colors: tuple[str, ...] | None = None
    utilization_ylim_max: float = 1.05

    def __post_init__(self):
        if not self.panels or len(self.panels) > 6 or any(p not in PANELS for p in self.panels):
            raise ValueError('Invalid chart panels')
        if not set(self.formats) <= {'png', 'svg'} or not self.formats:
            raise ValueError('Invalid chart formats')


def _series(samples, panel):
    t = [s.t_s for s in samples]
    if panel == 'trajectory':
        return [s.state.x_m for s in samples], [s.state.y_m for s in samples]
    if panel == 'yaw_rate':
        return t, [s.state.yaw_rate_rads * 180 / np.pi for s in samples]
    if panel == 'sideslip':
        return t, [np.rad2deg(np.arctan2(s.state.vy_ms, s.state.vx_ms)) for s in samples]
    if panel == 'tyre_utilization':
        return t, [max(w.utilization for w in s.forces.wheels.values()) for s in samples]
    return t, [s.drive_nm[3] - s.drive_nm[2] for s in samples]


LABELS = {
    'trajectory': dict(title='Trajectory', xlabel='x [m]', ylabel='y [m]'),
    'yaw_rate': dict(title='Yaw rate', xlabel='t [s]', ylabel='r [deg/s]'),
    'sideslip': dict(title='Side-slip angle', xlabel='t [s]', ylabel='beta [deg]'),
    'tyre_utilization': dict(title='Max. tyre force utilisation', xlabel='t [s]', ylabel='[-]'),
    'torque_allocation': dict(title='Torque Allocation: Antriebsmoment RR-RL', xlabel='t [s]', ylabel='[Nm]'),
}


def chart(seqs, spec, file, style=None):
    style = style or ChartStyle()
    import matplotlib

    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    rows = (len(style.panels) + 1) // 2
    fig, axs = plt.subplots(rows, 2, figsize=style.figure_size_in, constrained_layout=True, squeeze=False)
    axes = list(axs.flat)
    for extra in axes[len(style.panels) :]:
        extra.set_visible(False)
    for index, (name, samples) in enumerate(seqs.items()):
        color = {} if style.colors is None else {'color': style.colors[index % len(style.colors)]}
        for ax, panel in zip(axes, style.panels):
            ax.plot(*_series(samples, panel), label=name, **color)
    for ax, panel in zip(axes, style.panels):
        ax.set(**LABELS[panel])
        if panel == 'tyre_utilization':
            ax.set_ylim(0, style.utilization_ylim_max)
        ax.grid(alpha=style.grid_alpha)
        if style.legend:
            ax.legend()
    fig.suptitle(f'Controller A/B: {spec.kind} | SYNTHETIC | NOT VEHICLE VALIDATED')
    fig.savefig(file, dpi=style.dpi)
    plt.close(fig)
