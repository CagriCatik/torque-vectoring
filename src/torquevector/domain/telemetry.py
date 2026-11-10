"""Telemetry samples written by the simulators (state, tyre forces, wheel speeds, slip, torques)."""

from __future__ import annotations
from dataclasses import dataclass
from torquevector.domain.states import VehicleState
from torquevector.domain.states import VehicleForces


@dataclass(frozen=True)
class Sample:
    t_s: float
    state: VehicleState
    forces: VehicleForces


@dataclass(frozen=True)
class TorqueSample:
    t_s: float
    state: VehicleState
    forces: VehicleForces
    omega_rad_s: tuple[float, float, float, float]
    kappa: tuple[float, float, float, float]
    drive_nm: tuple[float, float, float, float]
    brake_nm: tuple[float, float, float, float]
    steer_actual_deg: float | None = None
