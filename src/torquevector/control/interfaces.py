"""Controller port: ``Controller.command(ControllerRequest) -> MomentCommand`` and its data types."""

from __future__ import annotations
from typing import Protocol
from torquevector.domain.states import VehicleState
from torquevector.domain.controls import YawReference
from typing import Callable
from torquevector.domain.controls import TorqueControls


class YawMomentStrategy(Protocol):
    def demand(self, state: VehicleState, reference: YawReference, steering_rad: float) -> float:
        """Return desired incremental yaw moment in Nm."""


ReferenceProvider = Callable[[float, VehicleState, TorqueControls], YawReference]

from dataclasses import dataclass
from typing import Any, runtime_checkable
from torquevector.prediction.interfaces import PredictionPreview


@dataclass(frozen=True)
class ControllerRequest:
    """Observed SI state; current reference and end-stage preview stay distinct."""

    state: VehicleState
    reference: YawReference = YawReference(0.0)
    steering_rad: float = 0.0
    preview: PredictionPreview | None = None
    previous_nm: float = 0.0
    period_s: float | None = None


@dataclass(frozen=True)
class MomentCommand:
    command_nm: float
    diagnostics: dict
    native_result: Any = None


@runtime_checkable
class Controller(Protocol):
    def command(self, request: ControllerRequest) -> MomentCommand: ...
