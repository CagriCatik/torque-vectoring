"""Yaw-rate reference providers derived from the driver steering angle."""

from __future__ import annotations
from math import radians
from math import tan
from torquevector.domain.vehicle import VehicleParameters
from torquevector.domain.states import VehicleState
from torquevector.domain.controls import TorqueControls
from torquevector.control.interfaces import ReferenceProvider
from torquevector.domain.controls import YawReference


def steering_reference(car: VehicleParameters) -> ReferenceProvider:
    """Kinematic target for open-loop steering tests (limited by road grip elsewhere)."""

    def reference(t: float, state: VehicleState, base: TorqueControls) -> YawReference:
        return YawReference(state.vx_ms * tan(radians(base.steer_deg)) / car.wheelbase_m)

    return reference
