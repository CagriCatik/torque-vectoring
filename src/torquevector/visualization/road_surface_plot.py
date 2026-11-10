"""Draufsicht auf die Reibwertkarte mit Trajektorien, Radpositionen, lokalen Reibwerten und Reifenkräften.

Alle Größen stammen aus gespeicherter Telemetrie bzw. den mit der Physikfunktion berechneten
Radkontakten (``telemetry/road_contact.json``); die Karte wird nur zur Darstellung abgetastet.
"""

from __future__ import annotations

from math import cos, sin
from typing import Mapping, Sequence

import numpy as np

from torquevector.domain.telemetry import TorqueSample
from torquevector.domain.vehicle import VehicleParameters
from torquevector.domain.wheels import WHEEL_ORDER
from torquevector.physics.road_friction import RoadFrictionMap

MU_CMAP = 'RdYlGn'
MU_RANGE = (0.2, 1.0)
WHEEL_COLORS = {'FL': '#1f77b4', 'FR': '#ff7f0e', 'RL': '#2ca02c', 'RR': '#d62728'}
CONTROLLER_COLORS = {'off': '#3b3b3b', 'p': '#1f77b4', 'lqr': '#2ca02c', 'ltv': '#ff7f0e', 'nmpc': '#d62728'}
FORCE_SCALE_M_PER_N = 1.0 / 2500.0  # Pfeillänge 1 m je 2500 N
LABEL = 'SYNTHETIC | NOT VEHICLE VALIDATED'


def _plt():
    import matplotlib

    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    return plt


def map_extent(seqs: Mapping[str, Sequence[TorqueSample]], margin_m: float = 6.0) -> tuple[float, float, float, float]:
    xs = [s.state.x_m for seq in seqs.values() for s in seq]
    ys = [s.state.y_m for seq in seqs.values() for s in seq]
    return min(xs) - margin_m, max(xs) + margin_m, min(min(ys), -2.0) - margin_m / 2, max(max(ys), 2.0) + margin_m / 2


def draw_friction_map(ax, field: RoadFrictionMap, extent, resolution_m: float = 0.25, labels: bool = True):
    """Reibwert als Farbbild; Regionsgrenzen gestrichelt und beschriftet."""
    x0, x1, y0, y1 = extent
    xs = np.arange(x0, x1 + resolution_m, resolution_m)
    ys = np.arange(y0, y1 + resolution_m, resolution_m)
    grid = np.array([[field.friction_at(float(x), float(y)) for x in xs] for y in ys])
    image = ax.imshow(
        grid,
        origin='lower',
        extent=(xs[0], xs[-1], ys[0], ys[-1]),
        cmap=MU_CMAP,
        vmin=MU_RANGE[0],
        vmax=MU_RANGE[1],
        aspect='auto',
        interpolation='nearest',
        alpha=0.85,
    )
    for region in field.surface.regions:
        rx0 = x0 if region.x_min_m is None else max(region.x_min_m, x0)
        rx1 = x1 if region.x_max_m is None else min(region.x_max_m, x1)
        ry0 = y0 if region.y_min_m is None else max(region.y_min_m, y0)
        ry1 = y1 if region.y_max_m is None else min(region.y_max_m, y1)
        if rx0 >= rx1 or ry0 >= ry1:
            continue
        ax.plot([rx0, rx1, rx1, rx0, rx0], [ry0, ry0, ry1, ry1, ry0], ls='--', lw=0.8, color='k', alpha=0.6)
        if labels:
            ax.text((rx0 + rx1) / 2, ry1 - 0.3, f'{region.id}\nµ={region.mu:g}', ha='center', va='top', fontsize=7)
    return image


# Karosseriemaße nur zur Darstellung (keine Modellgrößen): Überhänge und Breitenzuschlag.
FRONT_OVERHANG_M = 0.90
REAR_OVERHANG_M = 0.95
BODY_EXTRA_WIDTH_M = 0.25
TYRE_LENGTH_M = 0.65
TYRE_WIDTH_M = 0.23


def vehicle_outline(state, car: VehicleParameters):
    """Karosserieumriss in Weltkoordinaten (für Übersichtsdarstellungen)."""
    a = car.cg_to_front_m + FRONT_OVERHANG_M
    b = car.wheelbase_m - car.cg_to_front_m + REAR_OVERHANG_M
    w = car.track_m / 2 + BODY_EXTRA_WIDTH_M
    corners = [(a, w), (a, -w), (-b, -w), (-b, w), (a, w)]
    c, s = cos(state.yaw_rad), sin(state.yaw_rad)
    return ([state.x_m + c * x - s * y for x, y in corners], [state.y_m + s * x + c * y for x, y in corners])


def draw_vehicle(
    ax, state, car: VehicleParameters, steer_deg: float, wheel_mu: Mapping[str, float] | None = None, zorder: int = 6
) -> list:
    """Maßstäbliche Draufsicht im Weltsystem: Karosserie, Scheiben, vier Räder (vorne eingeschlagen).

    Räder sind nach lokalem Fahrbahnreibwert eingefärbt. Rückgabe: erzeugte Artists (für Animationen).
    """
    import matplotlib.colors as mcolors
    from matplotlib import transforms
    from matplotlib.patches import FancyBboxPatch, Polygon, Rectangle
    import matplotlib.pyplot as plt

    pose = transforms.Affine2D().rotate(state.yaw_rad).translate(state.x_m, state.y_m) + ax.transData
    a = car.cg_to_front_m + FRONT_OVERHANG_M
    b = car.wheelbase_m - car.cg_to_front_m + REAR_OVERHANG_M
    w = car.track_m / 2 + BODY_EXTRA_WIDTH_M
    artists = [
        FancyBboxPatch(
            (-b, -w),
            a + b,
            2 * w,
            boxstyle='round,pad=0,rounding_size=0.45',
            facecolor='#eef2f5',
            edgecolor='#2b3a46',
            lw=1.2,
            zorder=zorder,
            transform=pose,
        ),
        Polygon(
            [
                (car.cg_to_front_m - 0.95, w - 0.18),
                (car.cg_to_front_m - 0.95, -w + 0.18),
                (car.cg_to_front_m - 0.35, -w + 0.3),
                (car.cg_to_front_m - 0.35, w - 0.3),
            ],
            closed=True,
            facecolor='#3c4f5c',
            edgecolor='none',
            zorder=zorder + 1,
            transform=pose,
        ),
        Polygon(
            [
                (-car.wheelbase_m + car.cg_to_front_m - 0.2, w - 0.25),
                (-car.wheelbase_m + car.cg_to_front_m - 0.2, -w + 0.25),
                (-car.wheelbase_m + car.cg_to_front_m + 0.2, -w + 0.3),
                (-car.wheelbase_m + car.cg_to_front_m + 0.2, w - 0.3),
            ],
            closed=True,
            facecolor='#4d6570',
            edgecolor='none',
            zorder=zorder + 1,
            transform=pose,
        ),
    ]
    cmap, norm = plt.get_cmap(MU_CMAP), mcolors.Normalize(*MU_RANGE)
    for name, (lx, ly) in car.contact_points().items():
        angle = steer_deg if name.startswith('F') else 0.0
        tyre_pose = transforms.Affine2D().rotate_deg_around(lx, ly, angle) + pose
        colour = '#101820' if wheel_mu is None else cmap(norm(wheel_mu[name]))
        artists.append(
            Rectangle(
                (lx - TYRE_LENGTH_M / 2, ly - TYRE_WIDTH_M / 2),
                TYRE_LENGTH_M,
                TYRE_WIDTH_M,
                facecolor=colour,
                edgecolor='#101820',
                lw=1.0,
                zorder=zorder + 2,
                transform=tyre_pose,
            )
        )
    for artist in artists:
        ax.add_patch(artist)
    artists.append(ax.scatter([state.x_m], [state.y_m], s=14, c='k', zorder=zorder + 3))
    return artists


def force_vectors(sample: TorqueSample):
    """Weltkoordinaten-Kraftvektoren je Rad (fahrzeugfeste Kraft rotiert mit dem Gierwinkel)."""
    c, s = cos(sample.state.yaw_rad), sin(sample.state.yaw_rad)
    out = {}
    for name in WHEEL_ORDER:
        fx, fy = sample.forces.force_body[name]
        out[name] = (c * fx - s * fy, s * fx + c * fy)
    return out


def zoom_window(state, half_length_m: float = 5.5, half_width_m: float = 3.0):
    return (state.x_m - half_length_m, state.x_m + half_length_m, state.y_m - half_width_m, state.y_m + half_width_m)


def draw_ghost_vehicle(ax, state, car: VehicleParameters, label: str = 'OFF', zorder: int = 4) -> list:
    """Halbtransparenter Gestrichelter Karosserieumriss als Vergleichsfahrzeug in derselben Kamera."""
    from matplotlib.patches import Polygon

    ox, oy = vehicle_outline(state, car)
    poly = Polygon(
        list(zip(ox, oy)),
        closed=True,
        facecolor='#94a3b8',
        alpha=0.22,
        edgecolor='#475569',
        ls='--',
        lw=1.3,
        zorder=zorder,
    )
    ax.add_patch(poly)
    tag = ax.text(
        state.x_m,
        state.y_m - 1.35,
        label,
        fontsize=7,
        color='#475569',
        ha='center',
        va='top',
        weight='bold',
        zorder=zorder + 1,
    )
    return [poly, tag]


def draw_velocity_arrow(ax, state, length_m: float = 2.9, zorder: int = 8) -> list:
    """Gestrichelte Fahrzeuglängsachse + cyanfarbener Geschwindigkeitsvektor (macht β sichtbar)."""
    from math import hypot

    c, s = cos(state.yaw_rad), sin(state.yaw_rad)
    artists = list(
        ax.plot(
            [state.x_m, state.x_m + length_m * c],
            [state.y_m, state.y_m + length_m * s],
            ls=':',
            lw=1.1,
            color='#334155',
            zorder=zorder,
        )
    )
    vx_w = c * state.vx_ms - s * state.vy_ms
    vy_w = s * state.vx_ms + c * state.vy_ms
    speed = hypot(vx_w, vy_w)
    if speed > 0.5:
        scale = length_m / speed
        artists.append(
            ax.annotate(
                '',
                xy=(state.x_m + vx_w * scale, state.y_m + vy_w * scale),
                xytext=(state.x_m, state.y_m),
                zorder=zorder + 2,
                arrowprops={'arrowstyle': '-|>', 'color': '#0284c7', 'lw': 1.8, 'mutation_scale': 10},
            )
        )
    return artists


def draw_forces(ax, sample: TorqueSample, row: dict, zorder: int = 9) -> list:
    artists = []
    for name, (fx, fy) in force_vectors(sample).items():
        px, py = row[name]['x_m'], row[name]['y_m']
        artists.append(
            ax.annotate(
                '',
                xy=(px + fx * FORCE_SCALE_M_PER_N, py + fy * FORCE_SCALE_M_PER_N),
                xytext=(px, py),
                zorder=zorder,
                arrowprops=dict(arrowstyle='-|>', color='#0b2a6f', lw=1.4, mutation_scale=9),
            )
        )
    return artists


def road_map_figure(
    field: RoadFrictionMap,
    seqs: Mapping[str, Sequence[TorqueSample]],
    contacts: Mapping[str, Sequence[dict]],
    car: VehicleParameters,
    primary: str,
    title: str,
    snapshots: int = 4,
):
    """Oben: Gesamtstrecke maßstäblich (gleiches Achsenverhältnis). Unten: Zoom-Panels mit Fahrzeugdraufsicht,
    vier Rädern im lokalen µ, Lenkeinschlag und Reifenkraftvektoren zu ausgewählten Zeitpunkten."""
    plt = _plt()
    extent = map_extent(seqs, margin_m=4.0)
    x0, x1, y0, y1 = extent
    strip_height = 15.6 * (y1 - y0) / (x1 - x0)  # Höhe des maßstäblichen Streifens in Zoll
    fig = plt.figure(figsize=(16, strip_height + 4.6))
    grid = fig.add_gridspec(
        2,
        snapshots,
        height_ratios=(strip_height + 0.4, 3.2),
        left=0.05,
        right=0.93,
        top=0.92,
        bottom=0.09,
        hspace=0.42,
        wspace=0.18,
    )
    ax = fig.add_subplot(grid[0, :])
    image = draw_friction_map(ax, field, extent)
    for kind, seq in seqs.items():
        ax.plot(
            [s.state.x_m for s in seq],
            [s.state.y_m for s in seq],
            color=CONTROLLER_COLORS.get(kind, 'k'),
            lw=2.0 if kind == primary else 0.9,
            alpha=1.0 if kind == primary else 0.6,
            label=kind.upper(),
        )
    seq, rows = seqs[primary], contacts[primary]
    picks = _snapshot_indices(seq, rows, snapshots)
    for number, index in enumerate(picks, start=1):
        sample = seq[index]
        ax.plot(*vehicle_outline(sample.state, car), color='k', lw=0.9, zorder=6)
        ax.text(
            sample.state.x_m,
            sample.state.y_m + car.track_m,
            str(number),
            ha='center',
            va='bottom',
            fontsize=8,
            weight='bold',
            zorder=7,
        )
    ax.set(
        xlim=extent[:2],
        ylim=extent[2:],
        xlabel='x [m]',
        ylabel='y [m]',
        aspect='equal',
    )
    ax.set_title(f'{title} – friction map and trajectories (numbers = zoom panels, {primary.upper()})', loc='left')
    ax.legend(loc='lower right', bbox_to_anchor=(1.0, 1.0), fontsize=7, ncol=len(seqs), frameon=False)
    colorbar_axis = fig.add_axes([0.945, 0.12, 0.012, 0.74])
    fig.colorbar(image, cax=colorbar_axis, label='road µ [-]')
    for number, index in enumerate(picks, start=1):
        sample, row = seq[index], rows[index]
        zoom = fig.add_subplot(grid[1, number - 1])
        window = zoom_window(sample.state)
        draw_friction_map(zoom, field, window, resolution_m=0.1, labels=False)
        zoom.plot(
            [s.state.x_m for s in seq],
            [s.state.y_m for s in seq],
            color=CONTROLLER_COLORS.get(primary, 'k'),
            lw=1.2,
            alpha=0.7,
        )
        draw_vehicle(
            zoom, sample.state, car, sample.steer_actual_deg or 0.0, {n: row[n]['road_mu'] for n in WHEEL_ORDER}
        )
        draw_forces(zoom, sample, row)
        c, s = cos(sample.state.yaw_rad), sin(sample.state.yaw_rad)
        for name in WHEEL_ORDER:
            # Beschriftung außerhalb der Karosserie, im fahrzeugfesten System versetzt (vorne/hinten, links/rechts)
            dx, dy = (0.9 if name.startswith('F') else -0.9), (1.05 if name.endswith('L') else -1.05)
            px, py = row[name]['x_m'] + c * dx - s * dy, row[name]['y_m'] + s * dx + c * dy
            zoom.text(
                px,
                py,
                f"{name}\nµ {row[name]['road_mu']:.2f}",
                ha='center',
                va='center',
                fontsize=7,
                zorder=12,
                bbox=dict(fc='white', alpha=0.8, lw=0, pad=0.8),
            )
        zoom.set(
            xlim=window[:2],
            ylim=window[2:],
            aspect='equal',
            xlabel='x [m]',
            title=f'({number}) t = {sample.t_s:.2f} s, v = {sample.state.vx_ms:.1f} m/s',
        )
        zoom.tick_params(labelsize=7)
        zoom.set_title(zoom.get_title(), fontsize=10)
    fig.suptitle(
        f'{LABEL}  |  arrows: resultant tyre force per wheel (1 m ≙ {1 / FORCE_SCALE_M_PER_N:.0f} N)', fontsize=9
    )
    return fig


def _snapshot_indices(seq, rows, count: int) -> list[int]:
    """Ein Zeitpunkt je Reibwertereignis (Vorder- und Hinterachse auf verschiedenem µ), aufgefüllt gleichverteilt."""

    def axles_differ(i):
        return (rows[i]['FL']['road_mu'], rows[i]['FR']['road_mu']) != (
            rows[i]['RL']['road_mu'],
            rows[i]['RR']['road_mu'],
        )

    events, previous = [], None
    for i in range(len(rows)):
        if axles_differ(i) and (previous is None or i - previous > 1):
            events.append(i)
        if axles_differ(i):
            previous = i
    chosen = [min(i + 1, len(rows) - 1) if axles_differ(min(i + 1, len(rows) - 1)) else i for i in events][:count]
    spacing = max(1, len(seq) // (2 * count))
    for index in np.linspace(0, len(seq) - 1, count).astype(int):
        if len(chosen) >= count:
            break
        if all(abs(int(index) - c) > spacing for c in chosen):
            chosen.append(int(index))
    return sorted(chosen)


def mu_timeseries_figure(
    seqs: Mapping[str, Sequence[TorqueSample]], contacts: Mapping[str, Sequence[dict]], primary: str, title: str
):
    plt = _plt()
    fig, axs = plt.subplots(3, 1, figsize=(13, 9), sharex=True, constrained_layout=True)
    seq, rows = seqs[primary], contacts[primary]
    t = [r['t_s'] for r in rows]
    for name in WHEEL_ORDER:
        axs[0].step(t, [r[name]['road_mu'] for r in rows], where='post', color=WHEEL_COLORS[name], label=name)
        axs[1].plot(t, [s.forces.wheels[name].utilization for s in seq], color=WHEEL_COLORS[name], label=name)
        axs[2].plot(t, [s.forces.wheels[name].fx_n / 1000 for s in seq], color=WHEEL_COLORS[name], label=f'{name} Fx')
        axs[2].plot(t, [s.forces.wheels[name].fy_n / 1000 for s in seq], ':', color=WHEEL_COLORS[name])
    axs[0].set(title=f'Local road µ per wheel ({primary.upper()})', ylabel='µ [-]')
    axs[1].set(title='Friction utilisation per wheel', ylabel='[-]')
    axs[1].axhline(1, color='k', ls='--', lw=0.8)
    axs[2].set(title='Longitudinal force Fx (solid) and lateral force Fy (dotted)', ylabel='[kN]', xlabel='t [s]')
    for ax in axs:
        ax.grid(alpha=0.3)
        ax.legend(ncol=4, fontsize=8)
    fig.suptitle(f'{title}\n{LABEL}', fontsize=11)
    return fig
