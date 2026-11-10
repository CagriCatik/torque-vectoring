import json
import tempfile
import unittest
from pathlib import Path
from math import hypot
import numpy as np
from torquevector.domain.tyre import TyreParameters
from torquevector.physics.tyre_magic_formula import tyre_forces
from torquevector.physics.combined_slip import CombinedSlipTyre
from torquevector.physics.steering import SteeringActuator
from torquevector.calibration.synthetic_generation import generate, brush_surrogate
from torquevector.calibration.surrogate import read_dataset, predict, P0, fit
from torquevector.control.allocation import allocate_yaw, AllocationOptions
from torquevector.domain.vehicle import VehicleParameters
from torquevector.domain.states import VehicleState
from torquevector.domain.wheels import WheelParameters
from torquevector.domain.controls import TorqueControls
from torquevector.simulation.continuous import simulate_torque


class CombinedModelTests(unittest.TestCase):
    def test_zero_load_and_zero_slip(self):
        model = CombinedSlipTyre()
        self.assertEqual(model(0.1, 0.2, 0, TyreParameters()).fx_n, 0.0)
        self.assertEqual(model(0, 0, 4500, TyreParameters()).fy_n, 0.0)

    def test_pure_slip_matches_previous_model_before_saturation(self):
        p = TyreParameters()
        model = CombinedSlipTyre()
        for k, a in [(0.02, 0), (0, 0.03), (-0.02, 0), (0, -0.03)]:
            x = model(k, a, 4000, p)
            old = tyre_forces(k, a, 4000, p)
            self.assertAlmostEqual(x.fx_n, old.fx_n, places=8)
            self.assertAlmostEqual(x.fy_n, old.fy_n, places=8)

    def test_combined_reduction_and_ellipse_bound(self):
        p = TyreParameters()
        m = CombinedSlipTyre()
        for k in np.linspace(-0.4, 0.4, 15):
            for a in np.linspace(-0.35, 0.35, 15):
                res = m(float(k), float(a), 4300, p)
                self.assertLessEqual(res.utilization, 1 + 1e-10)
        self.assertLess(abs(m(0.12, 0.1, 4000, p).fx_n), abs(m(0.12, 0, 4000, p).fx_n))
        self.assertLess(abs(m(0.12, 0.1, 4000, p).fy_n), abs(m(0, 0.1, 4000, p).fy_n))

    def test_symmetries(self):
        m = CombinedSlipTyre()
        p = TyreParameters()
        x = m(0.12, 0.13, 4000, p)
        y = m(-0.12, -0.13, 4000, p)
        self.assertAlmostEqual(x.fx_n, -y.fx_n)
        self.assertAlmostEqual(x.fy_n, -y.fy_n)


class ActuatorTests(unittest.TestCase):
    def test_limit_and_delay(self):
        act = SteeringActuator(0.2, 30)
        self.assertAlmostEqual(act.derivative(20, 0), 30)
        self.assertAlmostEqual(act.derivative(-20, 0), -30)
        self.assertAlmostEqual(act.derivative(5, 5), 0)
        with self.assertRaises(ValueError):
            SteeringActuator(-0.1, 30)

    def test_solver_integrates_steering_and_advanced_tire(self):
        car = VehicleParameters()
        tyre = TyreParameters()
        out = simulate_torque(
            car,
            tyre,
            TorqueControls(steer_deg=2, drive_nm=(0, 0, 70, 70)),
            duration_s=0.5,
            max_step=0.025,
            output_dt=0.05,
            tyre_model=CombinedSlipTyre(),
            actuator=SteeringActuator(0.12, 50),
        )
        self.assertEqual(len(out), 11)
        self.assertEqual(out[0].steer_actual_deg, 0.0)
        self.assertTrue(0 < out[-1].steer_actual_deg < 2.01)
        self.assertTrue(all(abs(z.steer_actual_deg) < 3 for z in out))


class AllocationTests(unittest.TestCase):
    def test_bounds_sign_and_conservation(self):
        car = VehicleParameters()
        w = WheelParameters()
        base = (0.0, 0.0, 75.0, 75.0)
        pos = allocate_yaw(base, 160, car, w, 300)
        neg = allocate_yaw(base, -160, car, w, 300)
        self.assertGreater(pos.estimated_yaw_nm, 0)
        self.assertLess(neg.estimated_yaw_nm, 0)
        self.assertTrue(all(0 <= t <= 300 for t in pos.torques_nm))
        self.assertLess(abs(pos.torque_sum_delta_nm), 10.0)
        self.assertGreater(pos.torques_nm[3], pos.torques_nm[2])

    def test_constraints_strict(self):
        car = VehicleParameters()
        w = WheelParameters()
        result = allocate_yaw((0.0, 0.0, 20.0, 20.0), 1e5, car, w, 40)
        self.assertTrue(all(0 <= v <= 40 for v in result.torques_nm))
        self.assertTrue(result.bound_saturated)
        with self.assertRaises(ValueError):
            allocate_yaw((0, 0, 0, float('nan')), 1, car, w, 800)


class DataTests(unittest.TestCase):
    def test_generator_is_seed_reproducible_and_holdouts_distinct(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            generate(root / 'a.csv', root / 'a.json', 17)
            generate(root / 'b.csv', root / 'b.json', 17)
            self.assertEqual((root / 'a.csv').read_bytes(), (root / 'b.csv').read_bytes())
            data, split, meta = read_dataset(root / 'a.csv', root / 'a.json')
            self.assertGreater(len(data), 2500)
            self.assertTrue(all(np.isfinite(predict(P0, data)).flat))
            report = fit(root / 'a.csv', root / 'a.json', restarts=1)
            self.assertFalse(report['experimental_validation'])
            self.assertLess(report['metrics']['holdout']['joint_nrmse'], 0.12)

    def test_rejects_split_leakage(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            generate(root / 's.csv', root / 'meta.json')
            rows = (root / 's.csv').read_text().splitlines()
            cols = rows[1].split(',')
            cols[1] = 'holdout'
            rows[1] = ','.join(cols)
            (root / 's.csv').write_text('\n'.join(rows) + '\n')
            with self.assertRaisesRegex(ValueError, 'leakage'):
                read_dataset(root / 's.csv', root / 'meta.json')

    def test_synthetic_force_resists_positive_slip(self):
        x, y = brush_surrogate(np.array([0.1]), np.array([0.08]), np.array([4000.0]))
        self.assertGreater(x[0], 0)
        self.assertLess(y[0], 0)
        self.assertLess(hypot(x[0], y[0]), 4000)


if __name__ == '__main__':
    unittest.main()


class SolverTests(unittest.TestCase):
    def test_root_agrees_with_fixed_point(self):
        from torquevector.physics.wheel_rotation import forces_at
        from torquevector.physics.combined_slip import CombinedSlipTyre

        car = VehicleParameters()
        w = WheelParameters()
        p = TyreParameters()
        s = VehicleState(vx_ms=17, vy_ms=0.3, yaw_rate_rads=0.08)
        ctl = TorqueControls(1.8, (15, 15, 55, 55))
        om = (17 / 0.315,) * 4
        a, _ = forces_at(s, om, 1, car, p, ctl, w, CombinedSlipTyre(), 1.4, 'fixed_point')
        b, _ = forces_at(s, om, 1, car, p, ctl, w, CombinedSlipTyre(), 1.4, 'root')
        for attribute in ('fx_n', 'fy_n', 'mz_nm', 'ax_ms2', 'ay_ms2'):
            self.assertAlmostEqual(getattr(a, attribute), getattr(b, attribute), delta=1e-4)

    def test_convergence_and_energy_audit(self):
        from torquevector.evaluation.diagnostics import audit_energy

        car = VehicleParameters()
        w = WheelParameters()
        p = TyreParameters()
        cmd = TorqueControls(1.2, (10, 10, 50, 50))

        def run(maxstep):
            return simulate_torque(
                car,
                p,
                cmd,
                w,
                duration_s=1.0,
                output_dt=0.05,
                max_step=maxstep,
                rtol=1e-7,
                tyre_model=CombinedSlipTyre(),
                actuator=SteeringActuator(),
                load_solver='root',
            )

        coarse = run(0.04)
        fine = run(0.015)
        for attr in ('yaw_rate_rads', 'vx_ms', 'vy_ms', 'x_m', 'y_m'):
            v1 = np.asarray([getattr(s.state, attr) for s in coarse])
            v2 = np.asarray([getattr(s.state, attr) for s in fine])
            self.assertLess(np.max(abs(v1 - v2)), 0.005)
        audit = audit_energy(fine, car, w, cmd)
        self.assertLess(audit['relative_residual'], 0.002)


class FrictionRoadTests(unittest.TestCase):
    def test_split_mu_reduces_left_force_capacity(self):
        from torquevector.physics.road import FrictionMap

        road = FrictionMap(0.4, 0.95)
        p = TyreParameters()
        l = road.at_wheel('FL', 0, 0.12, 0, 4000, p)
        r = road.at_wheel('FR', 0, 0.12, 0, 4000, p)
        self.assertLess(l.peak_n, r.peak_n)
        self.assertLess(l.fx_n, r.fx_n)

    def test_mu_step(self):
        from torquevector.physics.road import FrictionMap

        road = FrictionMap(0.95, 0.95, after_s=1.0, after_mu=0.4)
        p = TyreParameters()
        self.assertGreater(
            road.at_wheel('FL', 0.5, 0.1, 0, 4000, p).peak_n, road.at_wheel('FL', 1.5, 0.1, 0, 4000, p).peak_n
        )
