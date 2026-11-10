"""Interactive matplotlib dashboard: vehicle top view, force arrows, friction circles and sliders for steering, slip and tyre parameters."""

from __future__ import annotations
from dataclasses import replace
from math import degrees
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from matplotlib.widgets import Button
import numpy as np
from torquevector.visualization.vehicle_artwork import BG
from torquevector.visualization.vehicle_artwork import TRACK
from torquevector.visualization.vehicle_artwork import WHITE
from torquevector.visualization.vehicle_artwork import MUTED
from torquevector.visualization.vehicle_artwork import FX_COLOR
from torquevector.domain.vehicle import VehicleParameters
from torquevector.domain.controls import DriverControls
from torquevector.domain.wheels import WHEEL_ORDER
from torquevector.domain.tyre import TyreParameters
from torquevector.simulation.slip import simulate
from torquevector.infrastructure.telemetry_io import export_csv
from torquevector.domain.telemetry import Sample
from torquevector.visualization.plots import GOLD
from torquevector.visualization.plots import PANEL
from torquevector.visualization.plots import circle_panel
from torquevector.visualization.plots import header
from torquevector.visualization.plots import path_panel
from torquevector.visualization.plots import vehicle_panel


def dashboard(
    car: VehicleParameters,
    tyre: TyreParameters,
    control: DriverControls,
    speed_ms: float,
    duration_s: float,
    interactive: bool = False,
    dt: float = 0.02,
    samples_override: list[Sample] | None = None,
    figure_size_in: tuple[float, float] = (18, 10.3),
):
    fig = plt.figure(figsize=tuple(figure_size_in), facecolor=BG)
    header(fig, torque_mode=samples_override is not None)
    main = fig.add_axes([0.018, 0.11, 0.43, 0.745], facecolor=TRACK)
    circle_axes = {
        'FL': fig.add_axes([0.465, 0.625, 0.147, 0.190], facecolor=PANEL),
        'FR': fig.add_axes([0.655, 0.625, 0.147, 0.190], facecolor=PANEL),
        'RL': fig.add_axes([0.465, 0.370, 0.147, 0.190], facecolor=PANEL),
        'RR': fig.add_axes([0.655, 0.370, 0.147, 0.190], facecolor=PANEL),
    }
    route_ax = fig.add_axes([0.500, 0.115, 0.277, 0.164], facecolor=PANEL)
    fig.text(0.49, 0.858, 'FRICTION CIRCLE / normalised to mu(Fz) * Fz', color=WHITE, fontsize=10.5, weight='bold')
    summary = fig.text(0.030, 0.075, '', color=WHITE, fontsize=11, family='monospace')
    note = fig.text(0.485, 0.315, '', color=MUTED, fontsize=9)

    def redraw(
        current_control: DriverControls, current_tyres: TyreParameters, current_speed: float, current_duration: float
    ):
        samples = (
            samples_override
            if samples_override is not None
            else simulate(car, current_tyres, current_control, current_speed, current_duration, dt=dt)
        )
        last = samples[-1]
        vehicle_panel(main, last, current_control.steer_deg)
        for name, axis in circle_axes.items():
            circle_panel(axis, name, last)
        path_panel(route_ax, samples)
        s, f = last.state, last.forces
        summary.set_text(
            f'Sum Fx={f.fx_n / 1000:+.2f} kN  Fy={f.fy_n / 1000:+.2f} kN '
            f'  Mz={f.mz_nm / 1000:+.2f} kNm\n'
            f'ax={f.ax_ms2:+.2f} m/s2  ay={f.ay_ms2:+.2f} m/s2  '
            f'yaw rate={degrees(s.yaw_rate_rads):+.1f} deg/s  '
            f'v={np.hypot(s.vx_ms, s.vy_ms) * 3.6:.1f} km/h'
        )
        note.set_text(
            'X = pure MF outside the limit circle, arrow = limited force'
            if any(x.clipped for x in f.wheels.values())
            else 'All wheel force vectors lie inside the limit circle'
        )
        fig.canvas.draw_idle()
        return samples

    samples_ref = [redraw(control, tyre, speed_ms, duration_s)]
    if interactive and samples_override is not None:
        raise ValueError("Live slip control is incompatible with torque-mode samples")
    if interactive:
        fig.text(0.824, 0.843, 'PARAMETER / LIVE', color=WHITE, fontsize=14, weight='bold')
        fig.text(0.824, 0.811, 'kappa: commanded slip ratio', color=MUTED, fontsize=9)
        specs = [
            ('Steering [deg]', -20, 20, control.steer_deg, GOLD),
            ('v0 [km/h]', 15, 140, speed_ms * 3.6, '#9BAEFF'),
            ('mu [-]', 0.2, 1.4, tyre.mu, '#D6A5FF'),
            ('kappa FL', -0.22, 0.22, control.slip_ratios[0], FX_COLOR),
            ('kappa FR', -0.22, 0.22, control.slip_ratios[1], FX_COLOR),
            ('kappa RL', -0.22, 0.22, control.slip_ratios[2], FX_COLOR),
            ('kappa RR', -0.22, 0.22, control.slip_ratios[3], FX_COLOR),
            ('Simdauer [s]', 1.0, 9.0, duration_s, '#86BFC5'),
        ]
        sliders = []
        for idx, (label, vmin, vmax, initial, color) in enumerate(specs):
            y = 0.766 - idx * 0.080
            fig.text(0.824, y + 0.031, label, color=MUTED, fontsize=9)
            s_ax = fig.add_axes([0.828, y + 0.008, 0.131, 0.017], facecolor=PANEL)
            slider = Slider(
                s_ax, '', vmin, vmax, valinit=initial, color=color, valfmt='%.2f' if idx >= 2 and idx < 7 else '%.1f'
            )
            slider.valtext.set_color(WHITE)
            slider.valtext.set_fontsize(8)
            sliders.append(slider)

        def current():
            c = DriverControls(float(sliders[0].val), tuple(float(s.val) for s in sliders[3:7]))
            p = replace(tyre, mu=float(sliders[2].val))
            return c, p, float(sliders[1].val) / 3.6, float(sliders[7].val)

        def changed(_):
            c, p, speed, duration = current()
            samples_ref[0] = redraw(c, p, speed, duration)

        for slider in sliders:
            slider.on_changed(changed)
        buttons = []
        for label, x, file_name in (
            ('PNG', 0.832, 'pacejka_dashboard.png'),
            ('SVG', 0.905, 'pacejka_dashboard.svg'),
        ):
            ax_button = fig.add_axes([x, 0.066, 0.066, 0.047])
            btn = Button(ax_button, label, color='#2E4960', hovercolor='#406785')
            btn.label.set_color(WHITE)
            btn.on_clicked(lambda _ev, fn=file_name: fig.savefig(fn, dpi=180, facecolor=BG, bbox_inches='tight'))
            buttons.append(btn)
        export_axis = fig.add_axes([0.854, 0.012, 0.10, 0.043])
        csv_button = Button(export_axis, 'CSV Telemetrie', color='#2E4960', hovercolor='#406785')
        csv_button.label.set_color(WHITE)
        csv_button.on_clicked(lambda _ev: export_csv(samples_ref[0], 'telemetrie.csv'))
        fig._widgets = sliders + buttons + [csv_button]
    else:
        end = samples_ref[0][-1]
        fig.text(0.825, 0.823, 'SCENARIO', color=WHITE, fontsize=14, weight='bold')
        lines = (
            f'Steering angle: {control.steer_deg:+.1f} deg',
            f'v0: {speed_ms * 3.6:.1f} km/h',
            f'mu: {tyre.mu:.2f}',
            f'Duration: {duration_s:.1f} s',
            '',
            ('Computed instantaneous slip:' if samples_override is not None else 'Slip per wheel:'),
            *(f'{n}: {k:+.3f}' for n, k in zip(WHEEL_ORDER, control.slip_ratios)),
            '',
            'Current wheel forces:',
            *(
                f'{n}  Fx {end.forces.wheels[n].fx_n / 1000:+.2f} kN\n'
                f'    Fy {end.forces.wheels[n].fy_n / 1000:+.2f} kN'
                for n in WHEEL_ORDER
            ),
        )
        fig.text(
            0.827, 0.772, '\n'.join(lines), color=MUTED, fontsize=10.2, linespacing=1.55, family='monospace', va='top'
        )
    return fig, samples_ref[0]
