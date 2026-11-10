"""Q11: Sollwertquelle, reibwertbewusste Allokation und Bewertungslogik."""

import unittest
from dataclasses import replace
from torquevector.control.allocation import AllocationOptions, allocate_yaw
from torquevector.control.allocation_friction import (
    FrictionLimit,
    allocate_yaw_bounded,
    friction_torque_limits,
    make_allocator,
)
from torquevector.domain.controls import TorqueControls
from torquevector.domain.scenarios import presets
from torquevector.domain.states import VehicleState
from torquevector.domain.vehicle import VehicleParameters
from torquevector.domain.wheels import WheelParameters
from torquevector.evaluation.q11_assessment import assess_degraded, assess_nominal
from torquevector.physics.load_transfer import wheel_loads
from torquevector.simulation.preview_adapter import make_maneuver_preview

CAR = VehicleParameters()
WHEEL = WheelParameters()
OPTIONS = AllocationOptions()
STATE = VehicleState(vx_ms=17.0, yaw_rate_rads=0.3)


class FrictionAwareAllocation(unittest.TestCase):
    def test_identical_to_frozen_allocator_when_inactive(self):
        base = (30.0, 30.0, 120.0, 120.0)
        frozen = allocate_yaw(base, 800.0, CAR, WHEEL, 800.0, OPTIONS)
        bounded = allocate_yaw_bounded(base, 800.0, CAR, WHEEL, 800.0, OPTIONS, (1e4,) * 4)
        self.assertEqual(frozen, bounded)

    def test_per_wheel_friction_limit_respected(self):
        base = (30.0, 30.0, 120.0, 120.0)
        limits = (200.0, 40.0, 130.0, 250.0)
        result = allocate_yaw_bounded(base, -2500.0, CAR, WHEEL, 800.0, OPTIONS, limits)
        for torque, limit, b in zip(result.torques_nm, limits, base):
            self.assertLessEqual(torque, max(limit, b) + 1e-9)
            self.assertGreaterEqual(torque, -1e-12)
        self.assertGreater(abs(result.yaw_residual_nm), 0)  # Sättigung wird als Residuum ausgewiesen

    def test_limits_follow_wheel_load_and_mu(self):
        dry = friction_torque_limits(STATE, CAR, WHEEL, 0.8, (1.0,) * 4)
        wet = friction_torque_limits(STATE, CAR, WHEEL, 0.8, (0.45,) * 4)
        loads = wheel_loads(CAR, 0.0, STATE.vx_ms * STATE.yaw_rate_rads)
        self.assertAlmostEqual(dry[0], WHEEL.radius_m * 0.8 * loads['FL'], 9)
        self.assertGreater(dry[1], dry[0])  # Linkskurve: rechte Räder höher belastet
        for d, w in zip(dry, wet):
            self.assertAlmostEqual(w / d, 0.45, 12)

    def test_allocator_factory_requires_one_mu_source(self):
        with self.assertRaises(ValueError):
            make_allocator(CAR, WHEEL, OPTIONS, FrictionLimit(0.8, None))
        with self.assertRaises(ValueError):
            make_allocator(CAR, WHEEL, OPTIONS, FrictionLimit(0.8, 1.0), lambda s: (1.0,) * 4)
        allocate = make_allocator(CAR, WHEEL, OPTIONS, FrictionLimit(0.8, None), lambda s: (0.05,) * 4)
        result = allocate((30.0, 30.0, 120.0, 120.0), 900.0, STATE, 800.0)
        self.assertTrue(result.bound_saturated)


class DriverSteeringReference(unittest.TestCase):
    def test_preview_uses_kinematic_driver_intent(self):
        from torquevector.control.controllers.ltv_mpc import LTVConfig

        cfg = LTVConfig()
        spec = presets()['double_lane_change']
        state = VehicleState(x_m=35.0, vx_ms=17.0)
        path = make_maneuver_preview(spec, CAR)(2.0, state, TorqueControls(steer_deg=1.5), cfg)
        driver = make_maneuver_preview(spec, CAR, reference_source='driver_steering')(
            2.0, state, TorqueControls(steer_deg=1.5), cfg
        )
        from math import tan

        for v, steer, yaw in zip(driver.speed_ms, driver.steering_rad, driver.desired_yaw_rads):
            self.assertAlmostEqual(yaw, v * tan(steer) / CAR.wheelbase_m, 12)
        self.assertNotEqual(path.desired_yaw_rads, driver.desired_yaw_rads)
        self.assertEqual(path.steering_rad, driver.steering_rad)


class Q11Assessment(unittest.TestCase):
    def entry(self, yaw, path, status='PASS'):
        return {'status': status, 'metrics': {'yaw_rmse_rads': yaw, 'cross_rmse_m': path}}

    def test_nominal_requires_every_case(self):
        matrix = {
            'a': {'off': self.entry(1.0, 1.0), 'lqr': self.entry(0.7, 1.05), 'p': self.entry(0.85, 1.0)},
            'b': {'off': self.entry(1.0, 1.0), 'lqr': self.entry(0.75, 1.09), 'p': self.entry(0.7, 1.0)},
        }
        result = assess_nominal(matrix, min_yaw_pct=20, min_path_pct=-10)
        self.assertEqual(result['qualified_controllers'], ['lqr'])
        self.assertTrue(result['passed'])
        matrix['b']['lqr'] = self.entry(0.75, 1.11)
        self.assertFalse(assess_nominal(matrix, min_yaw_pct=20, min_path_pct=-10)['passed'])
        matrix['b']['lqr'] = self.entry(0.5, 0.5, 'INVALID_MODEL_ENVELOPE')
        self.assertFalse(assess_nominal(matrix, min_yaw_pct=20, min_path_pct=-10)['passed'])

    def test_degraded_requires_defined_endings(self):
        good = {
            'c': {
                'off': {'status': 'FAIL_CLOSED', 'reason': 'slip'},
                'lqr': self.entry(0.5, 0.5, 'INVALID_MODEL_ENVELOPE'),
            }
        }
        self.assertTrue(assess_degraded(good, {}, {'lqr': 2000.0})['passed'])
        bad = {'c': {'lqr': {'status': 'FAIL_CLOSED', 'reason': ''}}}
        self.assertFalse(assess_degraded(bad, {}, {})['passed'])
        over = {'c': {'lqr': self.entry(0.5, 0.5)}}
        self.assertFalse(assess_degraded(over, {'c': {'lqr': [{'requested_nm': 2500.0}]}}, {'lqr': 2000.0})['passed'])


if __name__ == '__main__':
    unittest.main()
