"""Core regression, invariants, combined-slip and integration tests."""

import csv
from math import hypot, pi
from pathlib import Path
import tempfile
import unittest
from torquevector.domain.tyre import MagicFormulaAxis, TyreParameters
from torquevector.physics.tyre_magic_formula import magic_formula, tyre_forces, peak_grip
from torquevector.domain.vehicle import VehicleParameters
from torquevector.domain.controls import DriverControls
from torquevector.domain.states import VehicleState
from torquevector.physics.load_transfer import wheel_loads
from torquevector.physics.four_wheel import evaluate
from torquevector.domain.wheels import WHEEL_ORDER
from torquevector.simulation.slip import simulate
from torquevector.infrastructure.telemetry_io import export_csv


class TireCoreTests(unittest.TestCase):
    def setUp(self):
        self.p = TyreParameters()

    def test_zero_slip_is_zero_force(self):
        tyre = tyre_forces(0, 0, 3900, self.p)
        self.assertEqual((tyre.fx_n, tyre.fy_n, tyre.utilization), (0.0, -0.0, 0.0))

    def test_unloaded_tire_has_no_force(self):
        for k, a in ((0.01, 0.2), (-0.3, -0.2), (0, 0)):
            tyre = tyre_forces(k, a, 0, self.p)
            self.assertEqual(tyre.fx_n, 0)
            self.assertEqual(tyre.fy_n, 0)

    def test_signed_slips(self):
        self.assertGreater(tyre_forces(+0.04, -0.1, 4000, self.p).fx_n, 0)
        self.assertLess(tyre_forces(-0.04, +0.1, 4000, self.p).fx_n, 0)
        self.assertGreater(tyre_forces(0, -0.1, 4000, self.p).fy_n, 0)
        self.assertLess(tyre_forces(0, +0.1, 4000, self.p).fy_n, 0)

    def test_kamm_constraint_holds_under_combined_slip(self):
        for fz in (0, 50, 1500, 4000, 9500):
            for kappa in (-0.5, -0.2, -0.05, 0, 0.05, 0.2, 0.5):
                for alpha in (-0.8, -0.2, 0, 0.1, 0.5):
                    t = tyre_forces(kappa, alpha, fz, self.p)
                    self.assertLessEqual(hypot(t.fx_n, t.fy_n), t.peak_n + 1e-8)
                    self.assertLessEqual(t.utilization, 1 + 1e-12)

    def test_combined_slip_enforces_saturation(self):
        t = tyre_forces(0.16, 0.3, 4000, self.p)
        self.assertTrue(t.clipped)
        self.assertAlmostEqual(t.utilization, 1.0, places=10)
        self.assertLess(abs(t.fx_n), abs(t.fx_pure_n))
        self.assertLess(abs(t.fy_n), abs(t.fy_pure_n))

    def test_magic_formula_at_zero_is_zero(self):
        self.assertEqual(magic_formula(0, 1234, MagicFormulaAxis()), 0)

    def test_peak_grip_respects_load_sensitivity(self):
        p = self.p
        self.assertGreater(peak_grip(6000, p), peak_grip(3000, p))
        self.assertLess(peak_grip(6000, p) / 6000, peak_grip(3000, p) / 3000)

    def test_reject_invalid_inputs(self):
        with self.assertRaises(ValueError):
            tyre_forces(float('nan'), 0, 3000, self.p)
        with self.assertRaises(ValueError):
            tyre_forces(0, 0, -3, self.p)
        with self.assertRaises(ValueError):
            MagicFormulaAxis(b=-1)
        with self.assertRaises(ValueError):
            TyreParameters(mu=-2)


class DynamicsTests(unittest.TestCase):
    def setUp(self):
        self.car = VehicleParameters()
        self.tyre = TyreParameters()

    def test_load_conservation(self):
        for ax in (-12, -3, 0, 3, 12):
            for ay in (-20, -3, 0, 3, 20):
                loads = wheel_loads(self.car, ax, ay)
                self.assertAlmostEqual(sum(loads.values()), self.car.mass_kg * self.car.gravity)
                self.assertTrue(all(fz >= 0 for fz in loads.values()))

    def test_acceleration_transfers_load_rearward(self):
        static = wheel_loads(self.car)
        accelerating = wheel_loads(self.car, 3, 0)
        self.assertLess(accelerating['FL'], static['FL'])
        self.assertGreater(accelerating['RL'], static['RL'])

    def test_left_turn_transfers_load_to_right(self):
        loads = wheel_loads(self.car, 0, 6)
        self.assertGreater(loads['FR'], loads['FL'])
        self.assertGreater(loads['RR'], loads['RL'])

    def test_equal_rear_slip_does_not_generate_yaw(self):
        c = DriverControls(0, (0.0, 0.0, 0.06, 0.06))
        f = evaluate(VehicleState(), c, self.car, self.tyre, 1)
        self.assertAlmostEqual(f.mz_nm, 0, delta=1e-8)
        self.assertAlmostEqual(f.fy_n, 0, delta=1e-8)

    def test_differential_right_drive_generates_positive_yaw(self):
        c = DriverControls(0, (0, 0, 0.01, 0.06))
        f = evaluate(VehicleState(), c, self.car, self.tyre, 1)
        self.assertGreater(f.mz_nm, 0)

    def test_negative_steer_opposite_lateral_force(self):
        left = evaluate(VehicleState(), DriverControls(7, (0, 0, 0, 0)), self.car, self.tyre, 1)
        right = evaluate(VehicleState(), DriverControls(-7, (0, 0, 0, 0)), self.car, self.tyre, 1)
        self.assertGreater(left.fy_n, 0)
        self.assertLess(right.fy_n, 0)
        self.assertAlmostEqual(left.fy_n, -right.fy_n, delta=1e-5)

    def test_symmetric_drive_goes_straight(self):
        samples = simulate(self.car, self.tyre, DriverControls(0, (0.01, 0.01, 0.01, 0.01)), duration_s=2)
        self.assertAlmostEqual(samples[-1].state.y_m, 0, delta=1e-7)
        self.assertAlmostEqual(samples[-1].state.yaw_rad, 0, delta=1e-7)

    def test_step_size_validation(self):
        with self.assertRaises(ValueError):
            simulate(self.car, self.tyre, DriverControls(), dt=0.1)

    def test_scenario_wheel_data_and_export(self):
        samples = simulate(self.car, self.tyre, DriverControls(), duration_s=0.2)
        self.assertEqual(len(samples), 11)
        self.assertEqual(tuple(samples[-1].forces.wheels), WHEEL_ORDER)
        with tempfile.TemporaryDirectory() as folder:
            outfile = Path(folder) / 'result.csv'
            export_csv(samples, outfile)
            with outfile.open(newline='') as fh:
                rows = list(csv.DictReader(fh))
            self.assertEqual(len(rows), len(samples))
            self.assertIn('FL_alpha_deg', rows[0])
            self.assertIn('RR_utilization', rows[0])


if __name__ == '__main__':
    unittest.main()
