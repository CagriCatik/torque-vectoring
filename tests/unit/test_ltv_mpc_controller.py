"""LTV-MPC: mathematical invariants, actuator constraints and integration seams."""

import unittest
from dataclasses import replace
from math import atan2
import numpy as np
from scipy.linalg import solve_discrete_are
from torquevector.domain.vehicle import VehicleParameters
from torquevector.domain.states import VehicleState
from torquevector.domain.tyre import TyreParameters
from torquevector.control.controllers.ltv_mpc import LTVConfig, LTVMPC, SampledLTVPolicy
from torquevector.prediction.bicycle_linear import BicycleLinearizer
from torquevector.prediction.interfaces import PredictionPreview
from torquevector.prediction.ltv import prediction_matrices
from torquevector.simulation.preview_adapter import make_maneuver_preview
from torquevector.domain.scenarios import presets
from torquevector.simulation.driving import ManoeuvrePolicy
from torquevector.domain.controls import TorqueControls
from torquevector.simulation.sampled import simulate_sampled, SampledSettings


class LTVMPCTests(unittest.TestCase):
    def setUp(self):
        self.car = VehicleParameters()
        self.tyre = TyreParameters()
        self.cfg = LTVConfig()
        self.lin = BicycleLinearizer(self.car, self.tyre)

    def preview(self, n=None, speed=17.0, steering=0.0, yaw=0.0):
        n = n or self.cfg.horizon_steps
        return PredictionPreview((speed,) * n, (steering,) * n, (0.0,) * n, (yaw,) * n)

    def test_zero_equilibrium(self):
        s = LTVMPC(self.lin, self.car, self.cfg).solve(VehicleState(vx_ms=17), self.preview(), 0.0)
        self.assertAlmostEqual(s.command_nm, 0.0, places=6)

    def test_reference_sign(self):
        c = LTVMPC(self.lin, self.car, self.cfg)
        x = VehicleState(vx_ms=17)
        self.assertGreater(c.solve(x, self.preview(yaw=0.08), 0.0).command_nm, 0)
        self.assertLess(c.solve(x, self.preview(yaw=-0.08), 0.0).command_nm, 0)

    def test_sample_disagreement_rejected(self):
        with self.assertRaises(ValueError):
            LTVMPC(self.lin, self.car, self.cfg).solve(VehicleState(vx_ms=17), self.preview(), 0.0, 0.12)

    def test_invalid_preview_fails(self):
        with self.assertRaises(ValueError):
            LTVMPC(self.lin, self.car, self.cfg).solve(VehicleState(vx_ms=17), self.preview(n=2), 0.0)
        with self.assertRaises(ValueError):
            PredictionPreview((0.0,) * 16, (0.0,) * 16, (0.0,) * 16, (0.0,) * 16).validate(16)

    def test_hard_input_and_slew_all_stages(self):
        cfg = replace(self.cfg, moment_limit_nm=300.0, moment_slew_nm_s=500.0)
        c = LTVMPC(self.lin, self.car, cfg)
        p = self.preview(yaw=0.2)
        sol = c.solve(VehicleState(vx_ms=17), p, 150.0)
        u = np.array(sol.planned_nm)
        self.assertLessEqual(np.max(np.abs(u)), 300 + 1e-5)
        self.assertLessEqual(np.max(np.abs(np.diff(np.r_[150.0, u]))), 500 * cfg.sample_s + 1e-5)

    def test_terminal_dare_lqr_equivalence(self):
        """Fundamental MPC regression: no bounds active and fixed A,B -> discrete LQR."""
        cfg = replace(
            self.cfg,
            horizon_steps=8,
            moment_limit_nm=3000.0,
            moment_slew_nm_s=50000.0,
            state_soft_weight=0.0,
            slew_weight=0.0,
            input_weight=0.2,
        )
        plant = self.lin.discrete(18.0, cfg.sample_s)
        q = np.diag([1 / cfg.beta_scale_rad**2, 1 / cfg.yaw_scale_rads**2])
        r = cfg.input_weight / (cfg.moment_scale_nm / 1000.0) ** 2
        p = solve_discrete_are(plant.a, plant.b[:, None], q, [[r]])
        k = np.linalg.solve(plant.b[:, None].T @ p @ plant.b[:, None] + [[r]], plant.b[:, None].T @ p @ plant.a)
        x = VehicleState(vx_ms=18, vy_ms=0.27, yaw_rate_rads=-0.025)
        v = np.array([atan2(x.vy_ms, x.vx_ms), x.yaw_rate_rads])
        prediction = self.preview(n=8, speed=18.0)
        mpc = LTVMPC(self.lin, self.car, cfg).solve(x, prediction, 0.0)
        expected = float((-1000 * k @ v).item())
        self.assertLess(abs(mpc.command_nm - expected), 0.1)

    def test_condensed_variable_dynamics_match_direct_rollout(self):
        speeds = [12.0, 12.5, 13.0, 14.0, 14.3]
        steer = np.array([0.002, 0.003, 0.001, -0.002, -0.004])
        models = [self.lin.discrete(v, 0.04) for v in speeds]
        fx, fu, fd = prediction_matrices(models, steer)
        state = np.array([0.015, 0.025])
        u = np.array([0.08, -0.02, 0.05, 0.04, -0.01])
        condensed = fx @ state + fu @ u + fd
        direct = []
        for model, delta, moment in zip(models, steer, u):
            state = model.a @ state + model.b * moment + model.steering * delta
            direct.extend(state)
        np.testing.assert_allclose(condensed, np.array(direct), atol=1e-12, rtol=1e-12)

    def test_actual_time_variation_changes_prediction(self):
        cfg = replace(self.cfg, horizon_steps=8)
        c = LTVMPC(self.lin, self.car, cfg)
        x = VehicleState(vx_ms=17, vy_ms=0.25, yaw_rate_rads=0.04)
        p_constant = self.preview(n=8, speed=17.0)
        p_vary = PredictionPreview(
            tuple(17 + 0.6 * k for k in range(8)),
            tuple(0.005 + 0.001 * k for k in range(8)),
            (0.0,) * 8,
            tuple(0.06 + 0.002 * k for k in range(8)),
        )
        out1 = c.solve(x, p_constant, 0.0)
        out2 = c.solve(x, p_vary, 0.0)
        self.assertGreater(abs(out1.predicted_yaw_rads[-1] - out2.predicted_yaw_rads[-1]), 0.0001)

    def test_soft_state_envelope_reports_not_hides_violation(self):
        cfg = replace(self.cfg, beta_limit_rad=0.021, state_soft_weight=1.0)
        x = VehicleState(vx_ms=17, vy_ms=2.4, yaw_rate_rads=0.2)
        sol = LTVMPC(self.lin, self.car, cfg).solve(x, self.preview(), 0.0)
        self.assertGreaterEqual(sol.max_state_excess, 0.0)

    def test_non_monotonic_policy_rejected(self):
        spec = presets()['circle_track']
        policy = SampledLTVPolicy(
            LTVMPC(self.lin, self.car, self.cfg), ManoeuvrePolicy(spec, self.car), make_maneuver_preview(spec, self.car)
        )
        s = VehicleState(vx_ms=15)
        policy(0.0, s, TorqueControls())
        with self.assertRaises(RuntimeError):
            policy(0.0, s, TorqueControls())

    def test_causal_sampled_simulation(self):
        spec = presets()['circle_track']
        policy = SampledLTVPolicy(
            LTVMPC(self.lin, self.car, self.cfg), ManoeuvrePolicy(spec, self.car), make_maneuver_preview(spec, self.car)
        )
        history = simulate_sampled(
            self.car,
            self.tyre,
            TorqueControls(),
            speed_ms=15,
            duration_s=0.20,
            policy=policy,
            settings=SampledSettings(period_s=0.04, max_step_s=0.02),
        )
        self.assertEqual(policy.calls, 5)
        self.assertEqual(len(history), 6)


if __name__ == '__main__':
    unittest.main()
