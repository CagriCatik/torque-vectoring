"""Manoeuvre definitions (circle, double lane change, step/sine/ramp steer, straight) with reference paths and presets.

``ramp_steer`` is the quasi-steady-state ISO 4138 manoeuvre (constant speed, slowly increasing steer): the road-wheel
angle rises linearly from 0 at ``excitation_start_s`` to ``steer_amplitude_deg`` at ``duration_s``.
"""

from __future__ import annotations
from typing import Literal
from math import pi
from dataclasses import dataclass
from math import atan2
from math import cos
from math import hypot
from math import isfinite
from math import sin
from torquevector.domain.states import VehicleState
from math import atan

Kind = Literal[
    'circle_open', 'circle_track', 'double_lane_change', 'step_steer', 'sine_steer', 'straight', 'ramp_steer'
]
KINDS = ('circle_open', 'circle_track', 'double_lane_change', 'step_steer', 'sine_steer', 'straight', 'ramp_steer')


def clamp(value: float, lo: float, hi: float) -> float:
    return min(hi, max(lo, value))


def wrap_pi(angle: float) -> float:
    return (angle + pi) % (2 * pi) - pi


def smoothstep(z: float) -> float:
    z = clamp(z, 0.0, 1.0)
    return z * z * z * (z * (6 * z - 15) + 10)


def smoothstep_d(z: float) -> float:
    if not 0 < z < 1:
        return 0.0
    return 30 * z * z * (z - 1) * (z - 1)


def smoothstep_dd(z: float) -> float:
    if not 0 < z < 1:
        return 0.0
    return 60 * z * (z - 1) * (2 * z - 1)


@dataclass(frozen=True)
class PathPoint:
    x_m: float
    y_m: float
    heading_rad: float
    curvature_inv_m: float


@dataclass(frozen=True)
class CirclePath:
    radius_m: float = 90.0

    def __post_init__(self):
        if not isfinite(self.radius_m) or not 30 <= abs(self.radius_m) <= 10000:
            raise ValueError('Signed radius must have absolute value >=30 m')

    def at_progress(self, distance_m: float) -> PathPoint:
        theta = distance_m / self.radius_m
        return PathPoint(self.radius_m * sin(theta), self.radius_m * (1 - cos(theta)), theta, 1 / self.radius_m)

    def reference_at_state(self, s: VehicleState) -> tuple[PathPoint, float, float]:
        r = self.radius_m
        # The path is a circle centred at (0, R). Positive radius = left turn.
        centre_x, centre_y = 0.0, r
        radial = hypot(s.x_m - centre_x, s.y_m - centre_y)
        # Signed cross-track error (positive left of desired travel direction).
        e_y = (radial - abs(r)) * (1 if r > 0 else -1)
        # sin(theta)=x/R, cos(theta)=1-y/R works for either turn direction.
        theta = atan2(s.x_m / r, 1.0 - s.y_m / r)
        point = self.at_progress(r * theta)
        return point, e_y, wrap_pi(theta - s.yaw_rad)


@dataclass(frozen=True)
class DoubleLaneChangePath:
    offset_m: float = 3.5
    entry_x_m: float = 20.0
    transition_m: float = 42.0
    plateau_m: float = 25.0

    def __post_init__(self):
        vals = (self.offset_m, self.entry_x_m, self.transition_m, self.plateau_m)
        if (
            not all(isfinite(v) for v in vals)
            or not 0.5 <= abs(self.offset_m) <= 5
            or self.entry_x_m < 0
            or self.transition_m < 10
            or self.plateau_m < 0
        ):
            raise ValueError('Invalid lane change dimensions')

    def at_x(self, x_m: float) -> PathPoint:
        length = self.transition_m
        start2 = self.entry_x_m + length + self.plateau_m
        z1 = (x_m - self.entry_x_m) / length
        z2 = (x_m - start2) / length
        y = self.offset_m * (smoothstep(z1) - smoothstep(z2))
        yp = self.offset_m / length * (smoothstep_d(z1) - smoothstep_d(z2))
        ypp = self.offset_m / length**2 * (smoothstep_dd(z1) - smoothstep_dd(z2))
        return PathPoint(x_m, y, atan(yp), ypp / (1 + yp * yp) ** 1.5)

    def reference_at_state(self, s: VehicleState) -> tuple[PathPoint, float, float]:
        ref = self.at_x(s.x_m)
        # Cross-track deviation in reference Frenet normal, not global y only.
        e_y = (ref.y_m - s.y_m) * cos(ref.heading_rad)
        return ref, e_y, wrap_pi(ref.heading_rad - s.yaw_rad)

    @property
    def end_x_m(self) -> float:
        return self.entry_x_m + 2 * self.transition_m + self.plateau_m


@dataclass(frozen=True)
class StraightPath:
    """Geradeausfahrt entlang y = 0 in +x (Spurhaltung, z. B. über Reibwertabschnitte)."""

    def at_x(self, x_m: float) -> PathPoint:
        return PathPoint(x_m, 0.0, 0.0, 0.0)

    def reference_at_state(self, s: VehicleState) -> tuple[PathPoint, float, float]:
        return self.at_x(s.x_m), -s.y_m, wrap_pi(-s.yaw_rad)


@dataclass(frozen=True)
class ManoeuvreSpec:
    kind: Kind
    speed_ms: float = 18.0
    duration_s: float = 8.0
    radius_m: float = 90.0
    offset_m: float = 3.5
    entry_m: float = 20.0
    transition_m: float = 42.0
    plateau_m: float = 25.0
    steer_amplitude_deg: float = 1.5
    steer_frequency_hz: float = 0.5
    excitation_start_s: float = 1.0
    max_steer_deg: float = 12.0
    cross_gain: float = 0.85
    heading_gain: float = 0.95
    speed_kp_nm_per_ms: float = 65.0

    def __post_init__(self):
        if self.kind not in KINDS:
            raise ValueError('Unknown manoeuvre')
        vals = (
            self.speed_ms,
            self.duration_s,
            self.radius_m,
            self.offset_m,
            self.entry_m,
            self.transition_m,
            self.plateau_m,
            self.steer_amplitude_deg,
            self.steer_frequency_hz,
            self.excitation_start_s,
            self.max_steer_deg,
            self.cross_gain,
            self.heading_gain,
            self.speed_kp_nm_per_ms,
        )
        if not all(isfinite(x) for x in vals) or not (
            5 <= self.speed_ms <= 40
            and 0 < self.duration_s <= 30
            and 30 <= abs(self.radius_m) <= 10000
            and 0 < self.steer_frequency_hz <= 3
            and 0 <= self.excitation_start_s < self.duration_s
            and 0 < self.max_steer_deg <= 25
            and 0 <= self.cross_gain <= 10
            and 0 <= self.heading_gain <= 5
            and 0 <= self.speed_kp_nm_per_ms <= 250
        ):
            raise ValueError('Manoeuvre outside valid operating envelope')
        if self.kind == 'double_lane_change':
            DoubleLaneChangePath(self.offset_m, self.entry_m, self.transition_m, self.plateau_m)
        if self.kind in ('circle_open', 'circle_track'):
            CirclePath(self.radius_m)

    @property
    def path(self) -> CirclePath | DoubleLaneChangePath | StraightPath | None:
        if self.kind == 'straight':
            return StraightPath()
        if self.kind in ('circle_open', 'circle_track'):
            return CirclePath(self.radius_m)
        if self.kind == 'double_lane_change':
            return DoubleLaneChangePath(self.offset_m, self.entry_m, self.transition_m, self.plateau_m)
        return None

    def reference_at(self, s: VehicleState) -> tuple[PathPoint, float, float] | None:
        return self.path.reference_at_state(s) if self.path else None

    def reference_at_distance(self, distance_m: float) -> PathPoint | None:
        path = self.path
        if isinstance(path, CirclePath):
            return path.at_progress(distance_m)
        if isinstance(path, (DoubleLaneChangePath, StraightPath)):
            return path.at_x(distance_m)
        return None


def presets() -> dict[str, ManoeuvreSpec]:
    return {
        'circle_open': ManoeuvreSpec('circle_open', speed_ms=15.0, duration_s=8.0, radius_m=90.0),
        'circle_track': ManoeuvreSpec('circle_track', speed_ms=15.0, duration_s=8.0, radius_m=90.0),
        'double_lane_change': ManoeuvreSpec(
            'double_lane_change', speed_ms=17.0, duration_s=9.0, offset_m=3.5, transition_m=42.0, plateau_m=25.0
        ),
        'step_steer': ManoeuvreSpec('step_steer', speed_ms=17.0, duration_s=5.0, steer_amplitude_deg=1.6),
        'sine_steer': ManoeuvreSpec(
            'sine_steer', speed_ms=17.0, duration_s=6.0, steer_amplitude_deg=1.2, steer_frequency_hz=0.45
        ),
    }
