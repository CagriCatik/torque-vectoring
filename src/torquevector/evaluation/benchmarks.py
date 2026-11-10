"""Independent analytic references: stationary linear single-track model and nominal cornering stiffness."""

from __future__ import annotations
from math import radians
import numpy as np


def linear_bicycle_stationary(
    mass_kg: float,
    wheelbase_m: float,
    a_m: float,
    speed_ms: float,
    steer_deg: float,
    cf_n_per_rad: float,
    cr_n_per_rad: float,
) -> dict:
    if not (
        mass_kg > 0
        and 0 < a_m < wheelbase_m
        and speed_ms > 0
        and cf_n_per_rad > 0
        and cr_n_per_rad > 0
        and abs(steer_deg) < 5
    ):
        raise ValueError('Invalid bicycle assumptions')
    b = wheelbase_m - a_m
    v = speed_ms
    cf = cf_n_per_rad
    cr = cr_n_per_rad
    matrix = np.array(
        [
            [(cf + cr) / v, mass_kg * v + (cf * a_m - cr * b) / v],
            [(a_m * cf - b * cr) / v, (a_m * a_m * cf + b * b * cr) / v],
        ]
    )
    rhs = np.array([cf * radians(steer_deg), a_m * cf * radians(steer_deg)])
    vy, r = np.linalg.solve(matrix, rhs)
    return {'vy_ms': float(vy), 'yaw_rate_rads': float(r), 'ay_ms2': float(v * r)}


def nominal_cornering_stiffness(
    mu: float, load_n: float, load_nom_n: float, load_exponent: float, b: float, c: float
) -> float:
    """Independent analytic derivative at zero: dF/dalpha = mu(Fz)*Fz*B*C."""
    if not (load_n > 0 and load_nom_n > 0):
        raise ValueError('Invalid nominal load')
    return mu * (load_n / load_nom_n) ** load_exponent * load_n * b * c
