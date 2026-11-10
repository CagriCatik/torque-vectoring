"""Self-steering characteristic: per-axle tyre scaling, linear reference, curve analysis, g-g envelope, stop condition."""

import unittest
from dataclasses import replace
from math import degrees, hypot, radians, sqrt

import numpy as np

from torquevector.domain.controls import TorqueControls
from torquevector.domain.scenarios import ManoeuvreSpec
from torquevector.domain.states import VehicleState
from torquevector.domain.tyre import TyreParameters
from torquevector.domain.vehicle import VehicleParameters
from torquevector.evaluation.handling import (
    HandlingSettings,
    PhasePlaneSettings,
    characteristic,
    classify,
    gg_envelope,
    linear_characteristic,
    linear_yaw_gain,
    understeer_index,
)
from torquevector.physics.axle_tyre import AxleScale, AxleScaledTyre, TyreAxleScaling, scaled_tyre_model
from torquevector.physics.combined_slip import CombinedSlipTyre
from torquevector.physics.tyre_magic_formula import peak_grip
from torquevector.simulation.driving import ManoeuvrePolicy

CAR, TYRE = VehicleParameters(), TyreParameters()
NEUTRAL = TyreAxleScaling(AxleScale(1.0, 1.0), AxleScale(1.0, 1.0))
SETTINGS = HandlingSettings(
    speeds_ms=(20.0,),
    controllers=('off',),
    lateral_limit_estimate_factor=1.2,
    ramp_margin_deg=11.0,
    driver_max_wheel_torque_nm=700.0,
    speed_kp_nm_per_ms=250.0,
    linear_limit_g=0.4,
    gradient_bands_g=(0.1, 0.3, 0.5),
    neutral_tolerance_deg_per_g=0.25,
    saturation_utilisation=0.95,
    stop_sideslip_deg=10.0,
    gg_directions=36,
    gg_brake_in_turn_speed_ms=20.0,
    gg_brake_in_turn_nm=(200.0,),
    gg_rear_brake_fraction=0.6,
    gg_straight_drive_nm=(400.0,),
    phase_plane=PhasePlaneSettings(20.0, 1.0, 5.0, 0.3, 3, 3, 2.0, 0.05),
)


class AxleScaling(unittest.TestCase):
    def test_neutral_scaling_is_not_wrapped(self):
        model = CombinedSlipTyre()
        self.assertIs(scaled_tyre_model(model, NEUTRAL), model)

    def test_scaling_changes_only_the_selected_axle(self):
        scaling = TyreAxleScaling(AxleScale(0.8, 0.95), AxleScale(1.0, 1.0))
        model = AxleScaledTyre(CombinedSlipTyre(), scaling)
        base = CombinedSlipTyre()(0.0, 0.01, 4000.0, TYRE)
        front = model.at_wheel('FL', 0.0, 0.0, 0.01, 4000.0, TYRE)
        rear = model.at_wheel('RR', 0.0, 0.0, 0.01, 4000.0, TYRE)
        self.assertEqual(rear, base)
        self.assertAlmostEqual(front.fy_n / base.fy_n, 0.8 * 0.95, delta=0.01)  # small α: stiffness λ_Ky·λ_µ
        self.assertAlmostEqual(front.peak_n, peak_grip(4000.0, replace(TYRE, mu=TYRE.mu * 0.95)), 9)
        self.assertEqual(model(0.0, 0.01, 4000.0, TYRE), base)  # axle-independent call = nominal tyre

    def test_invalid_factor_rejected(self):
        for bad in (0.0, 0.2, 5.0, float('nan')):
            with self.assertRaises(ValueError):
                AxleScale(bad, 1.0)


class LinearReference(unittest.TestCase):
    def test_gradient_formula_and_speeds(self):
        lin = linear_characteristic(CAR, TYRE, NEUTRAL, neutral_tolerance_deg_per_g=0.25)
        m, a, L = CAR.mass_kg, CAR.cg_to_front_m, CAR.wheelbase_m
        cf, cr = lin['front_cornering_stiffness_n_per_rad'], lin['rear_cornering_stiffness_n_per_rad']
        k = m / L * ((L - a) / cf - a / cr)
        self.assertAlmostEqual(lin['understeer_gradient_rad_per_ms2'], k, 15)
        self.assertEqual(lin['classification'], 'neutral')  # +0.17 deg/g lies inside the ±0.25 band
        weak_front = TyreAxleScaling(AxleScale(0.8, 1.0), AxleScale(1.0, 1.0))
        us = linear_characteristic(CAR, TYRE, weak_front, neutral_tolerance_deg_per_g=0.25)
        self.assertEqual(us['classification'], 'understeer')
        self.assertAlmostEqual(us['characteristic_speed_ms'], sqrt(L / us['understeer_gradient_rad_per_ms2']), 12)
        weak_rear = TyreAxleScaling(AxleScale(1.0, 1.0), AxleScale(0.8, 1.0))
        os = linear_characteristic(CAR, TYRE, weak_rear, neutral_tolerance_deg_per_g=0.25)
        self.assertEqual(os['classification'], 'oversteer')
        self.assertIsNone(os['characteristic_speed_ms'])
        v_crit = os['critical_speed_ms']
        self.assertEqual(linear_yaw_gain(CAR, os['understeer_gradient_rad_per_ms2'], 1.01 * v_crit), float('inf'))
        self.assertGreater(linear_yaw_gain(CAR, os['understeer_gradient_rad_per_ms2'], 0.9 * v_crit), 0)

    def test_classify_band(self):
        self.assertEqual(classify(0.2, 0.25), 'neutral')
        self.assertEqual(classify(0.3, 0.25), 'understeer')
        self.assertEqual(classify(-0.3, 0.25), 'oversteer')


class CurveAnalysis(unittest.TestCase):
    def synthetic_curve(self, k_deg_per_g, saturate):
        ay = np.linspace(0.0, 0.9, 400)
        excess = k_deg_per_g * ay
        front = ay * (1.1 if saturate == 'front' else 0.9)
        rear = ay * (1.1 if saturate == 'rear' else 0.9)
        return {
            't_s': np.linspace(1, 30, 400),
            'ay_g': ay,
            'steer_excess_deg': excess,
            'steer_deg': excess + 3 * ay,
            'sideslip_deg': -ay,
            'yaw_gain_per_s': np.ones(400),
            'front_util': front,
            'rear_util': rear,
        }

    def test_gradient_bands_and_limit_behaviour(self):
        result = characteristic(self.synthetic_curve(2.0, 'front'), SETTINGS, stopped_by_sideslip=False)
        self.assertAlmostEqual(result['linear_gradient_deg_per_g'], 2.0, 9)
        self.assertTrue(all(abs(b['gradient_deg_per_g'] - 2.0) < 1e-9 for b in result['bands']))
        self.assertEqual(result['linear_classification'], 'understeer')
        self.assertTrue(result['limit_behaviour'].startswith('limit understeer'))
        rear = characteristic(self.synthetic_curve(-1.0, 'rear'), SETTINGS, stopped_by_sideslip=False)
        self.assertTrue(rear['limit_behaviour'].startswith('limit oversteer'))
        self.assertEqual(rear['linear_classification'], 'oversteer')
        spin = characteristic(self.synthetic_curve(-1.0, 'rear'), SETTINGS, stopped_by_sideslip=True)
        self.assertTrue(spin['limit_behaviour'].startswith('loss of stability'))
        self.assertAlmostEqual(result['max_lateral_acceleration_g'], 0.9, 9)

    def test_too_few_points_rejected(self):
        curve = {k: v[:5] for k, v in self.synthetic_curve(1.0, 'front').items()}
        with self.assertRaises(ValueError):
            characteristic(curve, SETTINGS, stopped_by_sideslip=False)


class Indicators(unittest.TestCase):
    def sample(self, steer_deg, yaw_rate):
        from torquevector.domain.telemetry import TorqueSample

        return TorqueSample(
            0.0,
            VehicleState(vx_ms=20.0, yaw_rate_rads=yaw_rate),
            None,
            (0,) * 4,
            (0,) * 4,
            (0,) * 4,
            (0,) * 4,
            steer_deg,
        )

    def test_understeer_index_sign_and_continuity(self):
        neutral = 20.0 * np.tan(radians(2.0)) / CAR.wheelbase_m
        self.assertAlmostEqual(understeer_index(self.sample(2.0, neutral), CAR), 0.0, 12)
        self.assertGreater(understeer_index(self.sample(2.0, 0.8 * neutral), CAR), 0)  # yaws less: understeer
        self.assertLess(understeer_index(self.sample(-2.0, -1.2 * neutral), CAR), 0)  # yaws more: oversteer
        self.assertAlmostEqual(understeer_index(self.sample(0.0, 0.0), CAR), 0.0, 12)


class GGEnvelope(unittest.TestCase):
    def test_symmetry_and_friction_bound(self):
        env = gg_envelope(CAR, TYRE, NEUTRAL, 72)
        ay, ax = np.array(env['ay_g']), np.array(env['ax_g'])
        radius = np.hypot(ax, ay)
        self.assertTrue(np.all(radius > 0.8) and np.all(radius < 1.2))  # µ ≈ 1 with load sensitivity
        self.assertAlmostEqual(env['max_lateral_g'], max(abs(ay)), 12)
        self.assertAlmostEqual(abs(ay[18]), abs(ay[54]), 6)  # left/right symmetric (90° and 270°)
        weak = gg_envelope(CAR, TYRE, TyreAxleScaling(AxleScale(1.0, 0.8), AxleScale(1.0, 0.8)), 72)
        self.assertLess(weak['max_lateral_g'], env['max_lateral_g'])


class RampSteerManoeuvre(unittest.TestCase):
    def test_linear_steer_ramp(self):
        spec = ManoeuvreSpec(
            'ramp_steer',
            speed_ms=20.0,
            duration_s=21.0,
            steer_amplitude_deg=10.0,
            excitation_start_s=1.0,
            max_steer_deg=12.0,
        )
        driver = ManoeuvrePolicy(spec, CAR)
        state = VehicleState(vx_ms=20.0)
        self.assertEqual(driver.steer_command(0.5, state), 0.0)
        self.assertAlmostEqual(driver.steer_command(11.0, state), 5.0, 12)
        self.assertAlmostEqual(driver.steer_command(21.0, state), 10.0, 12)
        self.assertAlmostEqual(driver.steer_command(30.0, state), 10.0, 12)


class StopCondition(unittest.TestCase):
    def test_stop_ends_run_with_samples(self):
        from torquevector.simulation.sampled import SampledSettings, simulate_sampled

        settings = SampledSettings(period_s=0.04, max_step_s=0.02)
        controls = TorqueControls(2.0, (0.0, 0.0, 80.0, 80.0))
        full = simulate_sampled(CAR, TYRE, controls, speed_ms=15.0, duration_s=1.0, settings=settings)
        stopped = simulate_sampled(
            CAR,
            TYRE,
            controls,
            speed_ms=15.0,
            duration_s=1.0,
            settings=settings,
            stop_condition=lambda s: abs(s.yaw_rate_rads) > 0.05,
        )
        self.assertLess(stopped[-1].t_s, full[-1].t_s)
        self.assertGreater(abs(stopped[-1].state.yaw_rate_rads), 0.05)
        self.assertEqual(stopped[:-1], full[: len(stopped) - 1])  # identical until the stop
        never = simulate_sampled(
            CAR, TYRE, controls, speed_ms=15.0, duration_s=1.0, settings=settings, stop_condition=lambda s: False
        )
        self.assertEqual(never, full)


if __name__ == '__main__':
    unittest.main()
