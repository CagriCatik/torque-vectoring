"""Radaufstandspunkte im globalen Koordinatensystem.

[x_i, y_i] = [x_CG, y_CG] + R(psi) [x_i,lokal, y_i,lokal] mit R(psi) = [[cos, -sin], [sin, cos]].
Lokale Lagen aus ``VehicleParameters.contact_points`` (x vorwärts, y links; Reihenfolge FL, FR, RL, RR).
Der Lenkeinschlag verschiebt den Aufstandspunkt in diesem Modell nicht.
"""

from __future__ import annotations

from math import cos, sin

from torquevector.domain.states import VehicleState
from torquevector.domain.vehicle import VehicleParameters
from torquevector.domain.wheels import WHEEL_ORDER


def wheel_contact_positions(state: VehicleState, car: VehicleParameters) -> dict[str, tuple[float, float]]:
    c, s = cos(state.yaw_rad), sin(state.yaw_rad)
    local = car.contact_points()
    return {
        name: (state.x_m + c * local[name][0] - s * local[name][1], state.y_m + s * local[name][0] + c * local[name][1])
        for name in WHEEL_ORDER
    }
