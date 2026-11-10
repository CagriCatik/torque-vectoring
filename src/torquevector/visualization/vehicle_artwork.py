"""Vehicle and road artwork for the dashboard top view."""

from __future__ import annotations
import numpy as np
from matplotlib.patches import FancyBboxPatch
from matplotlib import transforms
from matplotlib.patches import Polygon
from matplotlib.patches import Rectangle
from matplotlib.patches import FancyArrowPatch

WHEELS = {'FL': (-1.03, 1.36), 'FR': (1.03, 1.36), 'RL': (-1.03, -1.36), 'RR': (1.03, -1.36)}


BG = '#0B1220'


TRACK = '#1B2834'


WHITE = '#F1F6FA'


MUTED = '#A4B4C4'


FX_COLOR = '#FFAC4C'


FY_COLOR = '#57D2E4'


RES_COLOR = '#C7FF69'


def wheel_basis(steer_deg):
    angle = np.deg2rad(steer_deg)
    return np.array([-np.sin(angle), np.cos(angle)]), np.array([-np.cos(angle), -np.sin(angle)])


def draw_road(ax) -> None:
    ax.set_facecolor(TRACK)
    y = np.linspace(-3.5, 3.5, 150)
    bend = 0.03 * (y + 1.5) ** 2
    for side in (-1, +1):
        x = side * 2.82 + bend * 0.40
        ax.plot(x, y, color="#66798A", linewidth=2, alpha=0.45, zorder=0)
        ax.plot(x + side * 0.18, y, color="#D0A450", linewidth=3.2, alpha=0.8, zorder=0)
    for side in (-1, +1):
        x = side * 2.45 + bend * 0.40
        ax.plot(x, y, linestyle=(0, (1, 13)), color="#E1EDFD", linewidth=3.1, alpha=0.25, zorder=0)

    # Leichte Bodenkonturen als rein optische Orientierung.
    for y0 in np.arange(-2.6, 3.2, 0.95):
        ax.plot([-0.22, 0, 0.22], [y0, y0 + 0.13, y0], color="#8CA8AE", alpha=0.06, linewidth=5, zorder=0)


def rounded_box(ax, xy, w, h, radius, color, edge="none", zorder=3, lw=1):
    p = FancyBboxPatch(
        xy,
        w,
        h,
        boxstyle=f"round,pad=0,rounding_size={radius}",
        linewidth=lw,
        facecolor=color,
        edgecolor=edge,
        zorder=zorder,
    )
    ax.add_patch(p)
    return p


def draw_car(ax, steer_deg: float) -> None:
    # Bodenschatten, Karosserie, Scheiben, Front/Heck und Lichtsignatur.
    rounded_box(ax, (-1.00, -2.28), 2.03, 4.60, 0.58, "#080C11", zorder=1)
    rounded_box(ax, (-0.91, -2.23), 1.82, 4.46, 0.50, "#EEF2F5", edge="#B8C3CE", zorder=2, lw=2.2)

    # Motorhaube und Heckdeckel.
    ax.plot([-0.73, 0.73], [1.28, 1.28], color="#C7CED4", lw=1.9, zorder=3)
    ax.plot([-0.72, 0.72], [-1.54, -1.54], color="#C1CAD4", lw=1.8, zorder=3)
    ax.plot([-0.58, -0.72], [1.50, 1.99], color="#D4DAE0", lw=1.1, zorder=3)
    ax.plot([+0.58, +0.72], [1.50, 1.99], color="#D4DAE0", lw=1.1, zorder=3)

    # Dunkles Glasdach mit auslaufender Frontscheibe.
    roof = Polygon(
        [(-0.68, -1.26), (0.68, -1.26), (0.77, 0.72), (0.64, 1.20), (-0.64, 1.20), (-0.77, 0.72)],
        closed=True,
        facecolor="#26343E",
        edgecolor="#8F9EAA",
        linewidth=1.2,
        zorder=3,
    )
    ax.add_patch(roof)
    ax.add_patch(
        Polygon([(-0.63, 1.10), (+0.63, 1.10), (+0.72, 0.64), (-0.72, 0.64)], color="#526673", alpha=0.85, zorder=4)
    )
    ax.add_patch(
        Polygon([(-0.67, -0.97), (+0.67, -0.97), (+0.65, -1.22), (-0.65, -1.22)], color="#4D6570", alpha=0.7, zorder=4)
    )
    ax.plot([0, 0], [-0.83, 0.56], color="#485B67", lw=0.9, alpha=0.65, zorder=4)

    for side in (-1, +1):
        # Seitenspiegel.
        rounded_box(ax, (side * 0.91 - 0.13, 0.55), 0.27, 0.26, 0.08, "#DAE4E9", zorder=4)
        # Licht vorne, rot hinten.
        rounded_box(ax, (side * 0.65 - 0.13, 2.02), 0.25, 0.11, 0.04, "#EFFFD8", zorder=4)
        rounded_box(ax, (side * 0.58 - 0.17, -2.15), 0.33, 0.08, 0.035, "#FF6965", zorder=4)

    # Reifen mit Drehung der beiden Vorderraeder.
    for wheel, (cx, cy) in WHEELS.items():
        angle = steer_deg if wheel in ("FL", "FR") else 0.0
        tire_transform = transforms.Affine2D().rotate_deg_around(cx, cy, angle) + ax.transData
        tire = FancyBboxPatch(
            (cx - 0.20, cy - 0.42),
            0.40,
            0.84,
            boxstyle="round,pad=0,rounding_size=0.09",
            facecolor="#101820",
            edgecolor="#657581",
            lw=1.5,
            zorder=5,
            transform=tire_transform,
        )
        ax.add_patch(tire)
        hub = Rectangle(
            (cx - 0.105, cy - 0.25),
            0.21,
            0.50,
            facecolor="#6E8190",
            alpha=0.7,
            edgecolor="none",
            zorder=5,
            transform=tire_transform,
        )
        ax.add_patch(hub)

    # Schwerpunkt im Fahrzeugzentrum (S).
    ax.scatter([0], [0], s=32, c=WHITE, edgecolors=BG, linewidths=0.5, zorder=6)
    ax.text(0.11, -0.07, "S", fontsize=10, fontweight="bold", color=WHITE, zorder=7)


def force_arrow(
    ax,
    a: np.ndarray,
    b: np.ndarray,
    color: str,
    linewidth: float,
    zorder: float,
    alpha: float = 1.0,
    linestyle: str = "-",
) -> None:
    if np.linalg.norm(b - a) < 0.035:
        return
    patch = FancyArrowPatch(
        tuple(a),
        tuple(b),
        arrowstyle="-|>",
        mutation_scale=14 if linewidth < 3 else 19,
        linewidth=linewidth,
        color=color,
        linestyle=linestyle,
        alpha=alpha,
        zorder=zorder,
        shrinkA=0,
        shrinkB=0,
        joinstyle="round",
        capstyle="round",
    )
    ax.add_patch(patch)
