"""Q05 – Fahrzeugdynamik und Radlast."""

import unittest
from dataclasses import replace

import numpy as np

from quality.common import ROOT, runtime
from torquevector.domain.controls import TorqueControls
from torquevector.domain.errors import OutsideEnvelopeError
from torquevector.domain.states import VehicleState
from torquevector.domain.wheels import WHEEL_ORDER


class Q05VehicleDynamics(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rt = runtime()
        cls.car, cls.tyre, cls.wheel, cls.model = rt.vehicle, rt.tyre, rt.wheel, rt.tyre_model

    def test_Q05_1_newton_euler_against_hand_calculation(self):
        """Q05.1 (kritisch): Längs-, Quer- und Gierbilanz gegen unabhängige Handrechnung <= 1e-6 relativ."""
        from torquevector.physics.wheel_rotation import derivative, forces_at

        car = self.car
        for vx, vy, r, steer, drive in (
            (18.0, 0.3, 0.2, 3.0, (20.0, 20.0, 120.0, 90.0)),
            (25.0, -0.5, -0.15, -2.0, (0.0, 0.0, 60.0, 60.0)),
        ):
            state = VehicleState(vx_ms=vx, vy_ms=vy, yaw_rate_rads=r)
            omega = (vx / self.wheel.radius_m,) * 4
            ctrl = TorqueControls(steer_deg=steer, drive_nm=drive)
            x = np.array([0.0, 0.0, 0.0, vx, vy, r] + list(omega))
            d = derivative(5.0, x, car, self.tyre, ctrl, self.wheel, tyre_model=self.model, load_solver='root')
            f, _ = forces_at(state, omega, 5.0, car, self.tyre, ctrl, self.wheel, self.model, None, 'root')
            fx = sum(f.force_body[n][0] for n in WHEEL_ORDER)
            fy = sum(f.force_body[n][1] for n in WHEEL_ORDER)
            mz = sum(
                car.contact_points()[n][0] * f.force_body[n][1] - car.contact_points()[n][1] * f.force_body[n][0]
                for n in WHEEL_ORDER
            )
            drag = (
                0.5 * car.air_density_kgm3 * car.cd_area_m2 * vx * vx
                + car.rolling_coefficient * car.mass_kg * car.gravity
            )
            hand = ((fx - drag) / car.mass_kg + r * vy, fy / car.mass_kg - r * vx, mz / car.iz_kgm2)
            for got, ref in zip(d[3:6], hand):
                self.assertLessEqual(abs(got - ref), 1e-6 * max(1.0, abs(ref)))

    def test_Q05_2_static_loads_sum_and_moment_balance(self):
        """Q05.2: statische Radlasten summieren sich zu m*g und erfüllen die Schwerpunkt-Momentenbilanz."""
        from torquevector.physics.load_transfer import wheel_loads

        car = self.car
        loads = wheel_loads(car, 0.0, 0.0)
        mg = car.mass_kg * car.gravity
        self.assertAlmostEqual(sum(loads.values()), mg, 9)
        front, rear = loads['FL'] + loads['FR'], loads['RL'] + loads['RR']
        b = car.wheelbase_m - car.cg_to_front_m
        self.assertAlmostEqual(front * car.cg_to_front_m, rear * b, 6)
        self.assertAlmostEqual(loads['FL'], loads['FR'], 9)

    def test_Q05_3_load_transfer_signs_and_reference_values(self):
        """Q05.3: Längs- und Querlastverlagerung mit korrekten Vorzeichen und Referenzwerten."""
        from torquevector.physics.load_transfer import wheel_loads

        car = self.car
        L, h, t, m = car.wheelbase_m, car.cg_height_m, car.track_m, car.mass_kg
        static = wheel_loads(car, 0.0, 0.0)
        accel = wheel_loads(car, 3.0, 0.0)
        self.assertAlmostEqual((accel['RL'] + accel['RR']) - (static['RL'] + static['RR']), m * 3.0 * h / L, 6)
        brake = wheel_loads(car, -6.0, 0.0)
        self.assertGreater(brake['FL'], static['FL'])
        left = wheel_loads(car, 0.0, 5.0)  # ay > 0: Linkskurve
        self.assertAlmostEqual((left['FR'] + left['RR']) - (left['FL'] + left['RL']), m * 5.0 * h / t, 6)
        self.assertGreater(left['FR'], left['FL'])
        self.assertGreater(left['RR'], left['RL'])
        self.assertAlmostEqual(sum(left.values()), m * car.gravity, 9)

    def test_Q05_4_roll_pitch_lift_marked_as_model_limits(self):
        """Q05.4 (kritisch): Rollsteifigkeit, Nick-/Wank-Näherung und Radabheben als Modellgrenze gekennzeichnet."""
        from torquevector.physics.wheel_rotation import validate_no_lift

        spec = (ROOT / 'docs/model/specification.md').read_text()
        section = spec.split('## 6 Wheel loads and body')[1]
        for word in ('Roll stiffness distribution', 'not modelled', 'Wheel lift', 'fail-fast', 'pitch', 'roll'):
            self.assertIn(word, section)
        with self.assertRaisesRegex(OutsideEnvelopeError, 'wheel lift'):
            validate_no_lift(self.car, 0.0, 1.05 * self.car.gravity * self.car.track_m / self.car.cg_height_m)
        validate_no_lift(
            self.car, 0.0, 0.95 * self.car.gravity * self.car.track_m / self.car.cg_height_m
        )  # Grenze ay = g·t/h

    def test_Q05_5_edge_cases_reproducible_and_plausible(self):
        """Q05.5: Geradeaus, Freilauf, konstante Kreisfahrt und Lastwechsel reproduzierbar und plausibel."""
        from torquevector.physics.wheel_rotation import steering
        from torquevector.simulation.continuous import simulate_torque

        def run(ctrl, **kw):
            return simulate_torque(
                self.car, self.tyre, ctrl, self.wheel, tyre_model=self.model, load_solver='root', **kw
            )

        cases = {
            'straight': TorqueControls(0.0, (0.0, 0.0, 60.0, 60.0)),
            'coast': TorqueControls(0.0, (0.0,) * 4),
            'circle': TorqueControls(2.0, (0.0, 0.0, 75.0, 75.0)),
            'lift_off': TorqueControls(2.0, (0.0,) * 4),
        }
        for name, ctrl in cases.items():
            a, b = run(ctrl, duration_s=3.0), run(ctrl, duration_s=3.0)
            with self.subTest(case=name):
                self.assertEqual(a, b)  # bitgleich reproduzierbar
                self.assertTrue(all(np.isfinite(s.state.vx_ms) for s in a))
        straight = run(cases['straight'], duration_s=3.0)
        self.assertAlmostEqual(straight[-1].state.y_m, 0.0, 12)
        self.assertAlmostEqual(straight[-1].state.yaw_rad, 0.0, 12)
        coast = run(cases['coast'], duration_s=3.0)
        self.assertTrue(all(b.state.vx_ms < a.state.vx_ms for a, b in zip(coast, coast[1:])))  # verzögert monoton
        circle = run(cases['circle'], duration_s=6.0)
        late = [s.state.yaw_rate_rads for s in circle if s.t_s > 4.0]
        self.assertLess(np.std(late) / np.mean(late), 0.02)  # stationär
        lift = run(cases['lift_off'], duration_s=6.0)
        self.assertLess(lift[-1].state.vx_ms, circle[-1].state.vx_ms)  # Lastwechsel: Gas weg ⇒ langsamer


if __name__ == '__main__':
    unittest.main()
