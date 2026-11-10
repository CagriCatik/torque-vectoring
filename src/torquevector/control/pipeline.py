"""Feedback pipeline for P/LQR: yaw reference -> yaw moment demand -> limit -> torque allocation."""

from __future__ import annotations
from dataclasses import dataclass
from math import radians
from torquevector.domain.vehicle import VehicleParameters
from torquevector.domain.states import VehicleState
from torquevector.control.allocation import AllocationOptions
from torquevector.control.allocation import allocate_yaw
from torquevector.domain.controls import TorqueControls
from torquevector.domain.wheels import WheelParameters
from torquevector.control.interfaces import ReferenceProvider
from torquevector.control.interfaces import YawMomentStrategy
from torquevector.domain.controls import YawReference


@dataclass(frozen=True)
class YawControlPipeline:
    """Strategy -> incremental yaw moment -> common four-wheel allocator.

    The reference and base torque policy are supplied *externally* so all
    strategies can run exactly the same mission, actuator and tyre pipeline.
    """

    moment_strategy: YawMomentStrategy
    reference: ReferenceProvider
    car: VehicleParameters
    wheel: WheelParameters = WheelParameters()
    allocation: AllocationOptions = AllocationOptions()
    reference_mu: float = 0.95
    yaw_limit_nm: float = 2000.0
    reference_friction_fraction: float = 0.75
    diagnostics: list | None = None  # optional: je Takt angefordertes/allokiertes Moment (nur ZOH-Pfad)
    allocator: object = None  # optional: (base, moment, state, limit) -> AllocationResult, z. B. reibwertbewusst

    def __call__(self, t: float, state: VehicleState, base: TorqueControls) -> TorqueControls:
        ref = self.reference(t, state, base)
        cap = self.reference_friction_fraction * self.reference_mu * self.car.gravity / max(abs(state.vx_ms), 3.0)
        bounded = YawReference(max(-cap, min(cap, ref.yaw_rate_rads)), ref.sideslip_rad)
        moment = self.moment_strategy.demand(state, bounded, radians(base.steer_deg))
        moment = max(-self.yaw_limit_nm, min(self.yaw_limit_nm, moment))
        if self.allocator is None:
            assigned = allocate_yaw(base.drive_nm, moment, self.car, self.wheel, base.torque_limit_nm, self.allocation)
        else:
            assigned = self.allocator(base.drive_nm, moment, state, base.torque_limit_nm)
        if self.diagnostics is not None:
            self.diagnostics.append(
                {
                    'time_s': float(t),
                    'requested_nm': float(moment),
                    'allocated_estimate_nm': assigned.estimated_yaw_nm,
                    'residual_nm': assigned.yaw_residual_nm,
                    'reference_yaw_rads': bounded.yaw_rate_rads,
                }
            )
        return TorqueControls(base.steer_deg, assigned.torques_nm, base.brake_nm, base.torque_limit_nm)
