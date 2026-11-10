"""Construction of controllers and yaw strategies by name (off, p, lqr, ltv, nmpc)."""

from __future__ import annotations
from torquevector.domain.vehicle import VehicleParameters
from torquevector.domain.tyre import TyreParameters
from torquevector.control.controllers.lqr import LQRConfig
from torquevector.control.controllers.lqr import LQRYawSideslip
from torquevector.control.controllers.p import ProportionalYaw
from torquevector.control.interfaces import YawMomentStrategy


def make_strategy(
    kind: str,
    car: VehicleParameters,
    tyre: TyreParameters,
    *,
    proportional_gain: float = 2300,
    lqr_tuning: LQRConfig = LQRConfig(),
) -> YawMomentStrategy:
    if kind == 'p':
        return ProportionalYaw(proportional_gain)
    if kind == 'lqr':
        return LQRYawSideslip(car, tyre, lqr_tuning)
    raise ValueError(f'Unknown yaw controller: {kind}')


CONTROLLERS = ('off', 'p', 'lqr', 'ltv', 'nmpc')


def make_controller(
    kind,
    car,
    tyre,
    *,
    lqr_config=None,
    ltv_config=None,
    nmpc_config=None,
    proportional_gain=2300.0,
    nmpc_solver=None,
    friction_preview=False,
):
    """Common Controller port; each call creates independent controller state."""
    from torquevector.control.adapters import FeedbackController, PredictiveController

    if kind == 'off':
        from torquevector.control.controllers.off import OffYaw

        return FeedbackController(OffYaw())
    if kind in ('p', 'lqr'):
        return FeedbackController(
            make_strategy(kind, car, tyre, proportional_gain=proportional_gain, lqr_tuning=lqr_config or LQRConfig())
        )
    if kind == 'ltv':
        from torquevector.control.controllers.ltv_mpc import LTVMPC, LTVConfig
        from torquevector.prediction.bicycle_linear import BicycleLinearizer

        if friction_preview:
            from torquevector.control.controllers.friction_preview import FrictionScheduledLTV

            return PredictiveController(FrictionScheduledLTV(car, tyre, ltv_config or LTVConfig()))
        return PredictiveController(LTVMPC(BicycleLinearizer(car, tyre), car, ltv_config or LTVConfig()))
    if kind == 'nmpc':
        from torquevector.control.controllers.nmpc import NMPC, NMPCConfig

        return PredictiveController(
            NMPC(car, tyre, nmpc_config or NMPCConfig(), solver_settings=nmpc_solver, friction_preview=friction_preview)
        )
    raise ValueError(f'Unknown controller {kind}')
