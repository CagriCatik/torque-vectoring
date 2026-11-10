"""Vehicle state and force containers plus decoding of the integrator state vector."""

from __future__ import annotations
import numpy as np
from dataclasses import dataclass
from torquevector.domain.tyre import TyreResult


def decode(x: np.ndarray) -> tuple[VehicleState, tuple[float, ...]]:
    return VehicleState(*map(float, x[:6])), tuple(map(float, x[6:10]))


@dataclass(frozen=True)
class VehicleState:
    x_m: float = 0.0
    y_m: float = 0.0
    yaw_rad: float = 0.0
    vx_ms: float = 18.0
    vy_ms: float = 0.0
    yaw_rate_rads: float = 0.0


@dataclass(frozen=True)
class VehicleForces:
    wheels: dict[str, TyreResult]
    force_body: dict[str, tuple[float, float]]
    fx_n: float
    fy_n: float
    mz_nm: float
    ax_ms2: float
    ay_ms2: float
    drag_n: float
