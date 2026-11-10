"""Geometry, controller, integration and screening tests for new manoeuvres."""

import json
import tempfile
import unittest
from dataclasses import replace
from math import pi, radians
from pathlib import Path
import numpy as np
from torquevector.domain.scenarios import (
    CirclePath,
    DoubleLaneChangePath,
    ManoeuvreSpec,
    presets,
    smoothstep,
    smoothstep_d,
)
from torquevector.simulation.driving import ManoeuvrePolicy
from torquevector.evaluation.maneuver_metrics import compute_metrics
from torquevector.domain.vehicle import VehicleParameters
from torquevector.domain.states import VehicleState
from torquevector.domain.tyre import TyreParameters
from torquevector.domain.controls import TorqueControls
from torquevector.simulation.continuous import simulate_torque


class TrajectoryGeometryTests(unittest.TestCase):
    def test_smoothstep_has_no_end_slope(self):
        self.assertEqual(smoothstep(-2), 0)
        self.assertEqual(smoothstep(0), 0)
        self.assertEqual(smoothstep(1), 1)
        self.assertEqual(smoothstep(2), 1)
        self.assertEqual(smoothstep_d(0), 0)
        self.assertEqual(smoothstep_d(1), 0)
        self.assertAlmostEqual(smoothstep(0.5), 0.5)

    def test_circle_geometry_both_directions(self):
        for radius in (75.0, -75.0):
            with self.subTest(radius=radius):
                path = CirclePath(radius)
                q = path.at_progress(30.0)
                self.assertAlmostEqual(np.hypot(q.x_m, q.y_m - radius), abs(radius))
                self.assertGreater(q.x_m, 0)
                self.assertEqual(np.sign(q.y_m), np.sign(radius))
                state = VehicleState(q.x_m, q.y_m, q.heading_rad, 15, 0, 0)
                point, e_y, e_heading = path.reference_at_state(state)
                self.assertAlmostEqual(e_y, 0, places=9)
                self.assertAlmostEqual(e_heading, 0, places=9)
                self.assertAlmostEqual(point.x_m, q.x_m, places=9)

    def test_circle_error_has_correct_sign(self):
        for r in (90.0, -90.0):
            path = CirclePath(r)
            # Positive Frenet left-error convention: inside left-turn is negative; inside right-turn positive.
            state = VehicleState(0.0, 0.1 if r > 0 else -0.1, 0, 15, 0, 0)
            _, ey, _ = path.reference_at_state(state)
            self.assertLess(ey, 0) if r > 0 else self.assertGreater(ey, 0)

    def test_double_lane_change_start_middle_end_continuous(self):
        path = DoubleLaneChangePath()
        for x in (-1.0, 0.0, 20.0, path.end_x_m, path.end_x_m + 30):
            q = path.at_x(x)
            self.assertAlmostEqual(q.y_m, 0, places=12)
            self.assertAlmostEqual(q.curvature_inv_m, 0, places=9)
        self.assertAlmostEqual(path.at_x(20 + 42 + 12).y_m, 3.5)
        for x in (20.0, 62.0, 87.0, 129.0):
            qminus = path.at_x(x - 1e-5)
            qplus = path.at_x(x + 1e-5)
            self.assertLess(abs(qplus.y_m - qminus.y_m), 1e-6)
            self.assertLess(abs(qplus.heading_rad - qminus.heading_rad), 1e-6)

    def test_malformed_specs_fail(self):
        with self.assertRaises(ValueError):
            ManoeuvreSpec('unknown')
        with self.assertRaises(ValueError):
            ManoeuvreSpec('double_lane_change', transition_m=1.0)
        with self.assertRaises(ValueError):
            ManoeuvreSpec('circle_open', radius_m=0.0)
        with self.assertRaises(ValueError):
            ManoeuvreSpec('sine_steer', speed_ms=-1.0)


class SteeringPolicyTests(unittest.TestCase):
    car = VehicleParameters()

    def test_circle_open_constant_steering(self):
        p = ManoeuvrePolicy(ManoeuvreSpec('circle_open', radius_m=90), self.car)
        s = VehicleState(vx_ms=18)
        self.assertAlmostEqual(p.steer_command(0, s), p.steer_command(3, s))
        self.assertGreater(p.steer_command(2, s), 0)

    def test_speed_feedback_and_saturation(self):
        p = ManoeuvrePolicy(ManoeuvreSpec('step_steer', speed_ms=18), self.car)
        base = TorqueControls()
        fast = p(2, VehicleState(vx_ms=20), base)
        slow = p(2, VehicleState(vx_ms=12), base)
        self.assertGreater(slow.drive_nm[2], fast.drive_nm[2])
        self.assertGreaterEqual(fast.drive_nm[2], 0.0)
        self.assertLessEqual(slow.drive_nm[2], 250.0)

    def test_step_and_sine_no_excitation_before_start(self):
        s = VehicleState(vx_ms=18)
        for kind in ('sine_steer', 'step_steer'):
            policy = ManoeuvrePolicy(ManoeuvreSpec(kind), self.car)
            self.assertEqual(policy.steer_command(0, s), 0)
        step = ManoeuvrePolicy(ManoeuvreSpec('step_steer'), self.car)
        self.assertAlmostEqual(step.steer_command(1.4, s), 1.5)

    def test_double_lane_change_feedback_has_correct_sign(self):
        spec = ManoeuvreSpec('double_lane_change')
        p = ManoeuvrePolicy(spec, self.car)
        base = VehicleState(x_m=75.0, y_m=3.5, vx_ms=18)
        left = replace(base, y_m=4.5)
        right = replace(base, y_m=2.5)
        self.assertLess(p.steer_command(4, left), p.steer_command(4, right))


class FullRunTests(unittest.TestCase):
    car = VehicleParameters()
    tyre = TyreParameters()

    def test_circle_following_within_internal_tolerance(self):
        spec = replace(presets()['circle_track'], duration_s=5.0)
        policy = ManoeuvrePolicy(spec, self.car)
        seq = simulate_torque(
            self.car,
            self.tyre,
            TorqueControls(),
            speed_ms=spec.speed_ms,
            duration_s=spec.duration_s,
            policy=policy,
            max_step=0.05,
            output_dt=0.05,
        )
        r = compute_metrics(spec, seq)
        self.assertEqual(r['status'], 'PASS_INTERNAL_SCREENING')
        self.assertLess(r['metrics']['steady_circle_radius_rmse_m'], 1.0)


if __name__ == '__main__':
    unittest.main()
