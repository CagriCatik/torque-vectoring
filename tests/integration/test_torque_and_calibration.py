"""Adversarial and independent checks for the torque-based V3 path."""

import json
import tempfile
import unittest
from dataclasses import replace
from math import isfinite
from pathlib import Path
import numpy as np
from torquevector.domain.tyre import TyreParameters
from torquevector.domain.vehicle import VehicleParameters
from torquevector.domain.states import VehicleState
from torquevector.physics.load_transfer import wheel_loads
from torquevector.domain.wheels import WheelParameters
from torquevector.domain.controls import TorqueControls
from torquevector.domain.errors import OutsideEnvelopeError
from torquevector.physics.wheel_rotation import wheel_slips, derivative, forces_at
from torquevector.simulation.continuous import simulate_torque
from torquevector.evaluation.diagnostics import audit_energy, energy_j, power_w
from torquevector.evaluation.benchmarks import linear_bicycle_stationary, nominal_cornering_stiffness
from torquevector.calibration.tyre_fit import generate_demo, fit, read_measurements


class TorqueEquationTests(unittest.TestCase):
    car = VehicleParameters()
    tyre = TyreParameters()
    wheel = WheelParameters()

    def test_all_four_wheel_accelerations_obey_torque_balance(self):
        speed = 18.0
        omega = speed / self.wheel.radius_m
        x = np.asarray([0, 0, 0, speed, 0, 0] + [omega] * 4, dtype=float)
        ctrl = TorqueControls(drive_nm=(10.0, 20.0, 50.0, 70.0), brake_nm=(0.0, 0.0, 5.0, 8.0))
        out = derivative(0, x, self.car, self.tyre, ctrl, self.wheel)
        st = VehicleState(vx_ms=speed)
        f, _ = forces_at(st, (omega,) * 4, 0, self.car, self.tyre, ctrl, self.wheel)
        for i, name in enumerate(('FL', 'FR', 'RL', 'RR')):
            lhs = self.wheel.inertia_kgm2 * out[6 + i]
            rhs = ctrl.drive_nm[i] - ctrl.brake_nm[i] - self.wheel.radius_m * f.wheels[name].fx_n
            self.assertAlmostEqual(lhs, rhs, places=9)
        self.assertGreater(out[9], out[8])

    def test_slip_is_derived_from_angular_speed(self):
        st = VehicleState(vx_ms=18.0)
        omega = 18 / self.wheel.radius_m
        c = TorqueControls()
        self.assertAlmostEqual(wheel_slips(st, (omega,) * 4, 0, self.car, c, self.wheel)[0], 0, places=12)
        self.assertAlmostEqual(wheel_slips(st, (1.1 * omega,) * 4, 0, self.car, c, self.wheel)[0], 0.1, places=12)

    def test_low_speed_and_excess_slip_fail_closed(self):
        st = VehicleState(vx_ms=2.0)
        with self.assertRaisesRegex(OutsideEnvelopeError, 'contact speed'):
            wheel_slips(st, (12,) * 4, 0, self.car, TorqueControls(), self.wheel)
        with self.assertRaisesRegex(OutsideEnvelopeError, 'Slip'):
            wheel_slips(VehicleState(vx_ms=18), (500,) * 4, 0, self.car, TorqueControls(), self.wheel)
        with self.assertRaises(ValueError):
            TorqueControls(brake_nm=(0, -1, 0, 0))
        with self.assertRaises(ValueError):
            TorqueControls(drive_nm=(0, 900, 0, 0))

    def test_wheel_lift_is_rejected(self):
        from torquevector.physics.wheel_rotation import validate_no_lift

        with self.assertRaisesRegex(OutsideEnvelopeError, 'wheel lift'):
            validate_no_lift(self.car, 0, 40)

    def test_symmetric_torque_has_no_yaw(self):
        samples = simulate_torque(self.car, self.tyre, TorqueControls(0, (0, 0, 75, 75)), duration_s=1.0)
        self.assertAlmostEqual(samples[-1].state.yaw_rate_rads, 0, delta=1e-10)
        self.assertAlmostEqual(samples[-1].state.y_m, 0, delta=1e-10)
        self.assertEqual(len(samples[-1].omega_rad_s), 4)

    def test_asymmetric_drive_produces_expected_yaw(self):
        samples = simulate_torque(self.car, self.tyre, TorqueControls(0, (0, 0, 40, 110)), duration_s=1.0)
        self.assertGreater(samples[-1].state.yaw_rate_rads, 0)
        self.assertTrue(all(isfinite(x) for x in samples[-1].omega_rad_s))

    def test_energy_and_contact_dissipation(self):
        ctl = TorqueControls(steer_deg=1, drive_nm=(0, 0, 85, 80))
        samples = simulate_torque(self.car, self.tyre, ctl, duration_s=1.5)
        audit = audit_energy(samples, self.car, self.wheel, ctl)
        self.assertLess(audit['relative_residual'], 0.002)
        self.assertLess(audit['max_contact_power_w'], 1e-5)

    def test_unpowered_coasting_does_not_generate_energy(self):
        ctl = TorqueControls(0, (0, 0, 0, 0))
        samples = simulate_torque(self.car, self.tyre, ctl, duration_s=0.8)
        self.assertLess(energy_j(samples[-1], self.car, self.wheel), energy_j(samples[0], self.car, self.wheel))

    def test_numerical_step_refinement(self):
        ctl = TorqueControls(1, (0, 0, 75, 85))
        a = simulate_torque(self.car, self.tyre, ctl, duration_s=1.5, max_step=0.02)
        b = simulate_torque(self.car, self.tyre, ctl, duration_s=1.5, max_step=0.002)
        errors = []
        for name, floor in [('vx_ms', 1), ('vy_ms', 0.2), ('yaw_rate_rads', 0.05), ('yaw_rad', 0.1)]:
            va = np.asarray([getattr(x.state, name) for x in a])
            vb = np.asarray([getattr(x.state, name) for x in b])
            errors.append(float(np.sqrt(np.mean((va - vb) ** 2)) / max(floor, np.max(np.abs(vb)))))
        self.assertLess(max(errors), 0.02)

    def test_independent_stationary_bicycle_benchmark(self):
        car = replace(self.car, load_transfer=False, cd_area_m2=0, rolling_coefficient=0)
        loads = wheel_loads(car)
        p = self.tyre
        C = lambda key: sum(
            nominal_cornering_stiffness(p.mu, loads[n], p.nominal_fz_n, p.load_exponent, p.lateral.b, p.lateral.c)
            for n in key
        )
        cf = C(('FL', 'FR'))
        cr = C(('RL', 'RR'))
        c = TorqueControls(steer_deg=0.2, drive_nm=(0, 0, 0, 0))
        result = simulate_torque(car, p, c, duration_s=4, max_step=0.03)[-1]
        expected = linear_bicycle_stationary(
            car.mass_kg, car.wheelbase_m, car.cg_to_front_m, result.state.vx_ms, 0.2, cf, cr
        )
        self.assertLess(abs(result.state.yaw_rate_rads / expected['yaw_rate_rads'] - 1), 0.05)
        self.assertLess(abs(result.forces.ay_ms2 / expected['ay_ms2'] - 1), 0.05)


class CalibrationTests(unittest.TestCase):
    def test_synthetic_holdout_and_provenance(self):
        with tempfile.TemporaryDirectory() as root:
            csv = Path(root) / 'input.csv'
            meta = Path(root) / 'input.json'
            generate_demo(csv, meta)
            report = fit(csv, meta, restarts=3)
            self.assertEqual(report['status'], 'SYNTHETIC_IDENTIFICATION_CHECK_ONLY')
            self.assertTrue(report['holdout_under_10pct'])
            self.assertTrue(report['optimizer']['success'])
            for axis in ('Fx', 'Fy'):
                self.assertLess(report['metrics'][axis]['holdout_nrmse'], 0.03)
            self.assertGreater(report['optimizer']['jacobian_condition_number'], 1)

    def test_metadata_and_holdout_leakage(self):
        with tempfile.TemporaryDirectory() as root:
            csv = Path(root) / 'input.csv'
            meta = Path(root) / 'input.json'
            generate_demo(csv, meta)
            obj = json.loads(meta.read_text())
            obj['slip_units']['lateral'] = 'degrees'
            meta.write_text(json.dumps(obj))
            with self.assertRaisesRegex(ValueError, 'units'):
                read_measurements(csv, meta)
            obj['slip_units']['lateral'] = 'rad'
            meta.write_text(json.dumps(obj))
            with csv.open('a') as f:
                f.write('longitudinal,-0.320000000,2500,12,train\n')
            with self.assertRaisesRegex(ValueError, 'leakage'):
                read_measurements(csv, meta)


if __name__ == '__main__':
    unittest.main()
