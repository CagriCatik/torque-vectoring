"""Quasi-static longitudinal and lateral wheel load transfer (fixed front/rear split, no roll stiffness model)."""

from __future__ import annotations
from torquevector.domain.vehicle import VehicleParameters


def wheel_loads(params: VehicleParameters, ax_ms2: float = 0.0, ay_ms2: float = 0.0) -> dict[str, float]:
    """Instantaneous quasi-static axle and side-to-side transfer, no lift model."""
    m, g, a = params.mass_kg, params.gravity, params.cg_to_front_m
    b, l = params.wheelbase_m - a, params.wheelbase_m
    front = m * (g * b - ax_ms2 * params.cg_height_m) / l
    front = max(0.0, min(m * g, front))
    rear = m * g - front
    # Simple fixed front/rear lateral transfer distribution by static axle share.
    front_fraction = b / l
    total_difference = m * ay_ms2 * params.cg_height_m / params.track_m
    for_axles = ((front, total_difference * front_fraction), (rear, total_difference * (1 - front_fraction)))
    if not params.load_transfer:
        for_axles = ((m * g * front_fraction, 0.0), (m * g * (1 - front_fraction), 0.0))
    result = {}
    for prefix, (axle_load, diff) in zip(('F', 'R'), for_axles):
        left = max(0.0, min(axle_load, (axle_load - diff) / 2))
        result[prefix + 'L'] = left
        result[prefix + 'R'] = axle_load - left
    return result
