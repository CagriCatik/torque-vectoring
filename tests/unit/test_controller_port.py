import unittest
from dataclasses import replace
import numpy as np
from torquevector.control.interfaces import Controller, ControllerRequest
from torquevector.control.factory import make_controller, CONTROLLERS
from torquevector.domain.vehicle import VehicleParameters
from torquevector.domain.states import VehicleState
from torquevector.domain.tyre import TyreParameters
from torquevector.domain.controls import YawReference
from torquevector.prediction.interfaces import PredictionPreview
from torquevector.control.adapters import PredictiveController
from torquevector.prediction.bicycle_linear import BicycleLinearizer


class ControllerPortTests(unittest.TestCase):
    def test_all_controllers_share_active_command_contract_and_independent_state(self):
        state = VehicleState(vx_ms=15.0)
        for name in CONTROLLERS:
            first = make_controller(name, VehicleParameters(), TyreParameters())
            second = make_controller(name, VehicleParameters(), TyreParameters())
            self.assertIsInstance(first, Controller)
            self.assertIsNot(first, second)
            n = getattr(getattr(first, 'config', None), 'horizon_steps', 2)
            preview = PredictionPreview((15.0,) * n, (0.0,) * n, (0.0,) * n, (0.0,) * n)
            request = ControllerRequest(state, preview=preview, period_s=0.04)
            result = first.command(request)
            self.assertLess(abs(result.command_nm), 1e-4)
            self.assertEqual(result.diagnostics['status'], 'SUCCESS')

    def test_feedback_adapter_preserves_original_tuning_and_output(self):
        from torquevector.control.controllers.lqr import LQRYawSideslip
        from torquevector.control.controllers.p import ProportionalYaw

        car = VehicleParameters()
        tyre = TyreParameters()
        s = VehicleState(vx_ms=17, vy_ms=0.1, yaw_rate_rads=0.02)
        request = ControllerRequest(s, YawReference(0.08), 0.01)
        for name, old in [('p', ProportionalYaw()), ('lqr', LQRYawSideslip(car, tyre))]:
            self.assertEqual(
                make_controller(name, car, tyre).command(request).command_nm, old.demand(s, request.reference, 0.01)
            )

    def test_predictive_adapter_propagates_failure(self):
        class Failing:
            car = VehicleParameters()
            config = None

            def solve(self, *args):
                raise RuntimeError('intentional solver failure')

        controller = PredictiveController(Failing())
        with self.assertRaisesRegex(ValueError, 'requires aligned'):
            controller.command(ControllerRequest(VehicleState()))
        with self.assertRaisesRegex(RuntimeError, 'intentional'):
            controller.command(
                ControllerRequest(
                    VehicleState(), preview=PredictionPreview((15.0,) * 2, (0.0,) * 2, (0.0,) * 2, (0.0,) * 2)
                )
            )

    def test_linear_prediction_port_matches_exact_discrete_step(self):
        model = BicycleLinearizer(VehicleParameters(), TyreParameters())
        x = np.array([0.01, 0.02])
        m = model.discrete(17.0, 0.04)
        np.testing.assert_array_equal(model.step(x, 0.1, 17.0, 0.01, 0.04), m.a @ x + m.b * 0.1 + m.steering * 0.01)
