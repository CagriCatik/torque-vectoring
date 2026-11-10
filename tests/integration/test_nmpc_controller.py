"""NMPC boundary tests; deterministic synthetic-only, no vehicle validation."""

from __future__ import annotations
import unittest
import numpy as np
from torquevector.control.controllers.nmpc import NMPC, NMPCConfig
from torquevector.prediction.nonlinear_bicycle import tyre_from_nominal
from torquevector.prediction.interfaces import PredictionPreview
from torquevector.domain.vehicle import VehicleParameters
from torquevector.domain.states import VehicleState
from torquevector.domain.tyre import TyreParameters


def preview(n, speed=15.0, steer=0.0, target=0.0):
    return PredictionPreview((speed,) * n, (steer,) * n, (0.0,) * n, (target,) * n)


class NMPCUnitTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.car = VehicleParameters()
        cls.tyre = TyreParameters()
        cls.cfg = NMPCConfig(horizon_steps=5, max_iterations=50)
        cls.controller = NMPC(cls.car, cls.tyre, cls.cfg)

    def test_zero_state_zero_target(self):
        sol = self.controller.solve(VehicleState(vx_ms=15.0), preview(5), 0.0)
        self.assertLess(abs(sol.command_nm), 1e-4)
        self.assertEqual(len(sol.planned_nm), 5)
        self.assertTrue(np.all(np.isfinite(sol.predicted_beta_rad)))

    def test_yaw_correction_sign(self):
        sol = self.controller.solve(VehicleState(vx_ms=15.0, yaw_rate_rads=0.12), preview(5), 0.0)
        self.assertLess(sol.command_nm, 0)

    def test_hard_torque_and_slew(self):
        target = preview(5, target=0.3)
        sol = self.controller.solve(VehicleState(vx_ms=15.0), target, 0.0)
        u = np.array(sol.planned_nm)
        self.assertTrue(np.all(np.abs(u) <= self.cfg.moment_limit_nm + 1e-3))
        self.assertTrue(np.all(np.abs(np.diff(np.r_[0.0, u])) <= self.cfg.moment_slew_nm_s * self.cfg.sample_s + 1e-3))

    def test_previous_command_slew(self):
        sol = self.controller.solve(VehicleState(vx_ms=15.0), preview(5), 300.0)
        self.assertLessEqual(abs(sol.command_nm - 300), self.cfg.moment_slew_nm_s * self.cfg.sample_s + 1e-3)

    def test_invalid_preview_and_period(self):
        with self.assertRaises(ValueError):
            self.controller.solve(VehicleState(vx_ms=15.0), preview(4), 0.0)
        with self.assertRaises(ValueError):
            self.controller.solve(VehicleState(vx_ms=15.0), preview(5), 0.0, period_s=0.08)
        with self.assertRaises(ValueError):
            self.controller.solve(VehicleState(vx_ms=15.0), preview(5, speed=50.0), 0.0)

    def test_parameter_validation(self):
        with self.assertRaises(ValueError):
            NMPCConfig(horizon_steps=0)
        with self.assertRaises(ValueError):
            NMPCConfig(moment_slew_nm_s=-1)
        with self.assertRaises(ValueError):
            tyre_from_nominal(self.car, self.tyre, -0.2)

    def test_reproducible_new_controller(self):
        state = VehicleState(vx_ms=15.0, yaw_rate_rads=0.05)
        target = preview(5, steer=0.01, target=0.08)
        first = NMPC(self.car, self.tyre, self.cfg).solve(state, target, 0.0)
        second = NMPC(self.car, self.tyre, self.cfg).solve(state, target, 0.0)
        self.assertAlmostEqual(first.command_nm, second.command_nm, places=4)

    def test_offline_pso_not_imported_into_nmpc(self):
        from torquevector.control.controllers import nmpc

        self.assertNotIn('minimize_swarm', nmpc.__dict__)
        self.assertNotIn('pso', nmpc.__dict__)


if __name__ == '__main__':
    unittest.main()


class NMPCIntegrationTests(unittest.TestCase):
    def test_sampled_policy_in_four_wheel_plant(self):
        from torquevector.simulation.runner import simulate_controller
        from torquevector.domain.scenarios import presets

        m, samples = simulate_controller(
            'nmpc', presets()['circle_track'], 0.48, nmpc_config=NMPCConfig(horizon_steps=5)
        )
        self.assertEqual(m['control_ticks'], 12)
        self.assertEqual(len(samples), 13)
        self.assertTrue(np.isfinite(m['yaw_rmse_rads']))
        self.assertLessEqual(m['max_wheel_torque_nm'], 800)

    def test_unknown_strategy_fails(self):
        from torquevector.simulation.runner import simulate_controller
        from torquevector.domain.scenarios import presets

        with self.assertRaises(ValueError):
            simulate_controller('nonexistent', presets()['circle_track'], 0.1)
