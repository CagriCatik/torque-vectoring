"""Sampled (zero-order-hold) policy wrapping a predictive controller, preview and torque allocation."""

from __future__ import annotations
from time import perf_counter
from torquevector.domain.states import VehicleState
from torquevector.domain.controls import TorqueControls
from torquevector.domain.wheels import WheelParameters
from torquevector.control.allocation import AllocationOptions
from torquevector.control.allocation import allocate_yaw


class SampledYawMomentPolicy:
    """Only invoked at control ticks, never from adaptive ODE RHS."""

    def __init__(
        self,
        controller,
        base_policy,
        preview_provider,
        wheel: WheelParameters = WheelParameters(),
        allocation: AllocationOptions = AllocationOptions(),
        allocator=None,
    ):
        self.controller = controller
        self.base_policy = base_policy
        self.preview_provider = preview_provider
        self.wheel = wheel
        self.allocation = allocation
        self.allocator = allocator
        self.previous_nm = 0.0
        self.previous_time = None
        self.calls = 0
        self.last_result = None
        self.max_state_excess = 0.0
        self.max_allocation_residual_nm = 0.0
        self.diagnostics = []

    def observe_actuator(self, actual_deg, actuator):
        """Simulation-truth actuator port; not a sensor/observer claim."""
        observer = getattr(self.preview_provider, 'observe_actuator', None)
        if observer is not None:
            observer(actual_deg, actuator)

    def __call__(self, t: float, state: VehicleState, controls: TorqueControls) -> TorqueControls:
        dt = self.controller.config.sample_s
        if self.previous_time is not None and abs(t - self.previous_time - dt) > 1e-7:
            raise RuntimeError('Yaw policy used outside fixed-period sampled runner')
        base = self.base_policy(t, state, controls)
        preview = self.preview_provider(t, state, base, self.controller.config)
        started = perf_counter()
        result = self.controller.solve(state, preview, self.previous_nm, dt)
        solve_time_ms = 1000 * (perf_counter() - started)
        if self.allocator is None:
            allocation = allocate_yaw(
                base.drive_nm, result.command_nm, self.controller.car, self.wheel, base.torque_limit_nm, self.allocation
            )
        else:
            allocation = self.allocator(base.drive_nm, result.command_nm, state, base.torque_limit_nm)
        self.max_allocation_residual_nm = max(self.max_allocation_residual_nm, abs(allocation.yaw_residual_nm))
        self.max_state_excess = max(self.max_state_excess, result.max_state_excess)
        self.diagnostics.append(
            {
                "time_s": float(t),
                "solve_time_ms": solve_time_ms,
                "iterations": result.iterations,
                "requested_nm": result.command_nm,
                "allocated_estimate_nm": allocation.estimated_yaw_nm,
                "residual_nm": allocation.yaw_residual_nm,
                "predicted_beta_rad": result.predicted_beta_rad[0],
                "predicted_yaw_rads": result.predicted_yaw_rads[0],
            }
        )
        self.last_result = result
        self.previous_nm = result.command_nm
        self.previous_time = float(t)
        self.calls += 1
        return TorqueControls(base.steer_deg, allocation.torques_nm, base.brake_nm, base.torque_limit_nm)
