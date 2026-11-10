import unittest
from math import radians
import numpy as np
from torquevector.domain.vehicle import VehicleParameters
from torquevector.domain.states import VehicleState
from torquevector.domain.controls import TorqueControls
from torquevector.domain.wheels import WheelParameters
from torquevector.simulation.continuous import simulate_torque
from torquevector.domain.tyre import TyreParameters
from torquevector.domain.controls import YawReference
from torquevector.control.controllers.p import ProportionalYaw
from torquevector.control.controllers.lqr import LQRYawSideslip, LQRConfig, _lqr_gain
from torquevector.control.factory import make_strategy
from torquevector.control.pipeline import YawControlPipeline
from torquevector.prediction.bicycle_linear import bicycle_lateral_matrices, _axle_cornering_stiffness


class AlternativeControllerTests(unittest.TestCase):
    def setUp(self):
        self.car = VehicleParameters()
        self.tyre = TyreParameters()
        self.s = VehicleState(vx_ms=15, yaw_rate_rads=0.1, vy_ms=0.15)

    def test_polarity_of_proportional(self):
        p = ProportionalYaw(2000)
        self.assertGreater(p.demand(self.s, YawReference(0.2), 0), 0)
        self.assertLess(p.demand(self.s, YawReference(0), 0), 0)

    def test_lqr_stable_across_speed_grid(self):
        cf, cr = _axle_cornering_stiffness(self.car, self.tyre)
        for v in [5, 10, 15, 25, 35, 45]:
            a, b, _ = bicycle_lateral_matrices(v, self.car, cf, cr)
            k = np.asarray(_lqr_gain(float(v), self.car, cf, cr, LQRConfig()))
            self.assertTrue(np.all(np.real(np.linalg.eigvals(a - b @ k[None, :])) < 0))

    def test_lqr_zero_equilibrium(self):
        s = VehicleState(vx_ms=15, vy_ms=0.0, yaw_rate_rads=0.15)
        c = LQRYawSideslip(self.car, self.tyre)
        self.assertAlmostEqual(c.demand(s, YawReference(0.15), 0), 0.0, places=8)

    def test_lqr_yaw_polarity(self):
        c = LQRYawSideslip(self.car, self.tyre)
        s = VehicleState(vx_ms=15, vy_ms=0.0, yaw_rate_rads=0.0)
        self.assertGreater(c.demand(s, YawReference(0.1), 0), 0.0)
        self.assertLess(c.demand(s, YawReference(-0.1), 0), 0.0)

    def test_unknown_controller_rejected(self):
        with self.assertRaises(ValueError):
            make_strategy('nn', self.car, self.tyre)

    def test_bounds_and_shared_pipeline(self):
        base = TorqueControls(0, (30, 30, 30, 30))
        ref = lambda t, s, b: YawReference(0.5)
        for name in ('p', 'lqr'):
            control = YawControlPipeline(make_strategy(name, self.car, self.tyre), ref, self.car, WheelParameters())
            out = control(1.0, self.s, base)
            self.assertEqual(len(out.drive_nm), 4)
            self.assertTrue(all(0 <= v <= out.torque_limit_nm for v in out.drive_nm))
            self.assertEqual(out.steer_deg, base.steer_deg)
            self.assertEqual(out.brake_nm, base.brake_nm)

    def test_nonfinite_and_bad_config_rejected(self):
        with self.assertRaises(ValueError):
            LQRConfig(moment_scale_nm=-2.0)
        with self.assertRaises(ValueError):
            YawReference(float('nan'))

    def test_simulation_integration(self):
        car = self.car
        tyre = self.tyre
        base = TorqueControls(steer_deg=0.4, drive_nm=(35, 35, 45, 45))
        strategy = make_strategy('lqr', car, tyre)
        ctrl = YawControlPipeline(strategy, lambda t, s, b: YawReference(0.05), car)
        seq = simulate_torque(
            car,
            tyre,
            base,
            speed_ms=15,
            duration_s=0.35,
            output_dt=0.07,
            max_step=0.05,
            rtol=1e-5,
            policy=ctrl,
            load_solver='root',
        )
        self.assertEqual(len(seq), 6)
        self.assertTrue(all(np.isfinite(x.state.yaw_rate_rads) for x in seq))


if __name__ == '__main__':
    unittest.main()
