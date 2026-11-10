"""Unabhängige Q09/Q11/Q13-Diagnoseverträge, keine Messvalidierung."""

import unittest
from unittest.mock import patch
from types import SimpleNamespace
import numpy as np
import casadi as ca
from torquevector.domain.vehicle import VehicleParameters
from torquevector.domain.states import VehicleState
from torquevector.domain.tyre import TyreParameters
from torquevector.prediction.nonlinear_bicycle import bicycle_derivative, tyre_from_nominal
from torquevector.prediction.bicycle_linear import bicycle_lateral_matrices
from torquevector.control.sampling import SampledYawMomentPolicy
from torquevector.control.controllers.ltv_mpc import SampledLTVPolicy
from torquevector.control.controllers.nmpc import SampledNMPCPolicy
from torquevector.domain.scenarios import presets
from torquevector.domain.controls import TorqueControls


class PredictionContracts(unittest.TestCase):
    def test_symbolic_jacobian_against_central_difference(self):
        car = VehicleParameters()
        tyres = tyre_from_nominal(car, TyreParameters())
        z = ca.SX.sym('z', 5)
        f = bicycle_derivative(z[0], z[1], z[2], z[3], z[4], car, tyres, ca)
        fun = ca.Function('f', [z], [f])
        jac = ca.Function('j', [z], [ca.jacobian(f, z)])
        for point in ([0, 0, 0, 10, 0], [0.03, 0.12, 0.2, 17, 0.04], [-0.08, -0.3, -0.5, 30, -0.1]):
            point = np.array(point, float)
            numerical = np.empty((2, 5))
            eps = 1e-6
            for i in range(5):
                step = np.eye(5)[i] * eps
                numerical[:, i] = np.array((fun(point + step) - fun(point - step)) / (2 * eps)).ravel()
            np.testing.assert_allclose(np.array(jac(point)), numerical, rtol=2e-6, atol=1e-7)

    def test_small_signal_nonlinear_matches_bicycle(self):
        car = VehicleParameters()
        tyres = tyre_from_nominal(car, TyreParameters())
        for speed in (10.0, 17.0, 30.0):
            a, b, d = bicycle_lateral_matrices(speed, car, tyres.cf_n_rad, tyres.cr_n_rad)
            x = np.array([2e-7, -3e-7])
            u = 1e-7
            delta = 4e-7
            actual = np.array(bicycle_derivative(*x, u, speed, delta, car, tyres, ca)).ravel()
            np.testing.assert_allclose(actual, a @ x + b.ravel() * u + d * delta, rtol=1e-7, atol=1e-12)

    def test_nmpc_has_neutral_policy_parent(self):
        self.assertTrue(issubclass(SampledNMPCPolicy, SampledYawMomentPolicy))
        self.assertFalse(issubclass(SampledNMPCPolicy, SampledLTVPolicy))

    def test_v11_same_initial_torque_distribution_for_all_controllers(self):
        """Observe pre-allocation base input, without four-wheel integration."""
        import torquevector.simulation.runner as run_v11
        from torquevector.control.factory import CONTROLLERS
        from torquevector.infrastructure.configuration import load_model

        car = VehicleParameters()
        tyre, model = load_model()
        observed = {}

        class Capture(Exception):
            pass

        for kind in CONTROLLERS:

            def simulate(*args, policy, **kwargs):
                state = VehicleState(vx_ms=15.0)
                if isinstance(policy, SampledYawMomentPolicy):
                    base = policy.base_policy(0, state, TorqueControls())
                else:
                    with patch(
                        'torquevector.control.pipeline.allocate_yaw',
                        side_effect=lambda base, *a, **k: SimpleNamespace(torques_nm=base),
                    ):
                        base = policy(0, state, TorqueControls())
                observed[kind] = base.drive_nm
                raise Capture

            with patch.object(run_v11, 'simulate_sampled', side_effect=simulate):
                with self.assertRaises(Capture):
                    run_v11.simulate_controller(kind, presets()['circle_track'], 0.1, car=car, tyre=tyre, model=model)
        for kind, drive in observed.items():
            np.testing.assert_allclose(drive, observed['off'], err_msg=kind)


if __name__ == '__main__':
    unittest.main()


class ActuatorPreviewContracts(unittest.TestCase):
    def test_exact_hold_against_independent_ode_integral(self):
        from scipy.integrate import solve_ivp
        from torquevector.physics.steering import SteeringActuator

        actuator = SteeringActuator()
        for command, actual, duration in (
            (3.0, 0.0, 0.04),
            (25.0, -10.0, 0.3),
            (-25.0, 10.0, 0.5),
            (40.0, 0.0, 0.4),
            (0.0, 0.0, 0.04),
        ):
            end, mean = actuator.predict_hold(command, actual, duration)
            sol = solve_ivp(
                lambda t, x: [actuator.derivative(command, x[0]), x[0]],
                (0, duration),
                [actual, 0.0],
                rtol=1e-11,
                atol=1e-12,
                max_step=0.0005,
            )
            np.testing.assert_allclose([end, mean], [sol.y[0, -1], sol.y[1, -1] / duration], atol=1e-7, rtol=1e-8)
        for duration in (0.0, -1.0, float('nan')):
            with self.assertRaises(ValueError):
                actuator.predict_hold(1.0, 0.0, duration)

    def test_observed_steering_is_delayed_and_refreshes_each_tick(self):
        from torquevector.simulation.preview_adapter import make_maneuver_preview
        from torquevector.control.controllers.ltv_mpc import LTVConfig
        from torquevector.physics.steering import SteeringActuator

        car = VehicleParameters()
        forecast = make_maneuver_preview(presets()['circle_track'], car)
        state = VehicleState(vx_ms=15.0)
        base = TorqueControls(steer_deg=2.0)
        cfg = LTVConfig()
        forecast.observe_actuator(0.0, SteeringActuator())
        predicted = forecast(0.0, state, base, cfg)
        self.assertGreater(predicted.steering_rad[0], 0.0)
        self.assertLess(predicted.steering_rad[0], np.deg2rad(2.0))
        forecast.observe_actuator(2.0, SteeringActuator())
        self.assertAlmostEqual(forecast(0.04, state, base, cfg).steering_rad[0], np.deg2rad(2.0), places=12)
