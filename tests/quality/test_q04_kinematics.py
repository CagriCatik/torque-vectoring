"""Q04 – Radkinematik und Schlupf (unabhängige Vektor-Referenzrechnung mit numpy)."""

import unittest
from math import atan2, cos, radians, sin

import numpy as np

from quality.common import ROOT, runtime
from torquevector.domain.controls import DriverControls, TorqueControls
from torquevector.domain.errors import OutsideEnvelopeError
from torquevector.domain.states import VehicleState
from torquevector.domain.wheels import WHEEL_ORDER

STATES = [
    VehicleState(vx_ms=v, vy_ms=vy, yaw_rate_rads=r)
    for v, vy, r in ((18.0, 0.0, 0.0), (15.0, 0.6, 0.35), (25.0, -0.9, -0.25), (8.0, 0.3, 0.5))
]


def reference_wheel_velocity(state, car, name, delta):
    """v_P = v_CG + ω × r_P im Fahrzeugsystem, dann Drehung um −δ ins Radsystem (Matrizenform)."""
    x, y = car.contact_points()[name]
    v = np.array([state.vx_ms, state.vy_ms, 0.0]) + np.cross([0.0, 0.0, state.yaw_rate_rads], [x, y, 0.0])
    rot = np.array([[cos(delta), sin(delta)], [-sin(delta), cos(delta)]])
    return rot @ v[:2]


class Q04Kinematics(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rt = runtime()
        cls.car, cls.tyre, cls.wheel, cls.model = rt.vehicle, rt.tyre, rt.wheel, rt.tyre_model

    def test_Q04_1_wheel_positions_and_velocities_against_independent_reference(self):
        """Q04.1 (kritisch): Radpositionen und lokale Geschwindigkeiten aus vx, vy, r unabhängig geprüft."""
        from torquevector.physics.four_wheel import evaluate
        from torquevector.physics.wheel_rotation import wheel_longitudinal_velocities

        a, L, t = self.car.cg_to_front_m, self.car.wheelbase_m, self.car.track_m
        self.assertEqual(
            self.car.contact_points(),
            {'FL': (a, t / 2), 'FR': (a, -t / 2), 'RL': (a - L, t / 2), 'RR': (a - L, -t / 2)},
        )
        for state in STATES:
            for steer in (0.0, 4.0, -7.0):
                u = wheel_longitudinal_velocities(state, 1.0, self.car, TorqueControls(steer_deg=steer), steer)
                forces = evaluate(
                    state, DriverControls(steer, (0.0,) * 4), self.car, self.tyre, 1.0, (0.0, 0.0), self.model, steer
                )
                for i, name in enumerate(WHEEL_ORDER):
                    delta = radians(steer) if name.startswith('F') else 0.0
                    ref = reference_wheel_velocity(state, self.car, name, delta)
                    self.assertAlmostEqual(u[i], ref[0], 12)
                    self.assertAlmostEqual(forces.wheels[name].slip_angle_rad, atan2(ref[1], ref[0]), 12)

    def test_Q04_2_frames_documented_and_back_transformation_exact(self):
        """Q04.2: Koordinaten dokumentiert; Rücktransformation erhält Norm und Vorzeichen bis 1e-10."""
        from torquevector.physics.four_wheel import rotate_wheel_to_body

        spec = (ROOT / 'docs/model/specification.md').read_text()
        self.assertIn('## 4 Coordinate systems and sign conventions', spec)
        rng = np.random.default_rng(4)
        for fx, fy, delta in rng.uniform([-8e3, -8e3, -0.45], [8e3, 8e3, 0.45], (2000, 3)):
            bx, by = rotate_wheel_to_body(fx, fy, delta)
            back = (cos(delta) * bx + sin(delta) * by, -sin(delta) * bx + cos(delta) * by)
            self.assertLess(abs(np.hypot(bx, by) - np.hypot(fx, fy)), 1e-10 * max(1.0, np.hypot(fx, fy)))
            self.assertLess(max(abs(back[0] - fx), abs(back[1] - fy)), 1e-10 * max(1.0, np.hypot(fx, fy)))
        self.assertEqual(rotate_wheel_to_body(1.0, 0.0, 0.0), (1.0, 0.0))

    def test_Q04_3_slip_angle_sign_and_parallel_steering(self):
        """Q04.3: konsistente alpha-Vorzeichenkonvention; dokumentierte Parallellenkung (kein Ackermann)."""
        from torquevector.physics.four_wheel import evaluate

        spec = (ROOT / 'docs/model/specification.md').read_text()
        self.assertIn('parallel steering', spec)
        straight = VehicleState(vx_ms=20.0)
        f = evaluate(straight, DriverControls(3.0, (0.0,) * 4), self.car, self.tyre, 1.0, (0.0, 0.0), self.model, 3.0)
        self.assertLess(f.wheels['FL'].slip_angle_rad, 0)  # Lenkung links ⇒ alpha < 0
        self.assertEqual(f.wheels['FL'].slip_angle_rad, f.wheels['FR'].slip_angle_rad)  # gleicher Radwinkel
        self.assertGreater(f.wheels['FL'].fy_n, 0)  # ⇒ Seitenkraft nach links
        self.assertGreater(f.mz_nm, 0)  # ⇒ Linksgieren
        self.assertEqual(f.wheels['RL'].slip_angle_rad, 0.0)  # keine Hinterradlenkung

    def test_Q04_4_kappa_from_wheel_speed_with_defined_special_cases(self):
        """Q04.4 (kritisch): kappa aus Raddrehzahl und Längsgeschwindigkeit; Stillstand, Rollen, Anfahren definiert."""
        from torquevector.physics.wheel_rotation import wheel_longitudinal_velocities, wheel_slips

        c = TorqueControls()
        for state in STATES[:3]:
            u = wheel_longitudinal_velocities(state, 1.0, self.car, c, 0.0)
            rolling = tuple(v / self.wheel.radius_m for v in u)
            self.assertTrue(all(abs(k) < 1e-12 for k in wheel_slips(state, rolling, 1.0, self.car, c, self.wheel, 0.0)))
            spun = tuple(1.08 * w for w in rolling)
            for k in wheel_slips(state, spun, 1.0, self.car, c, self.wheel, 0.0):
                self.assertAlmostEqual(k, 0.08, 12)
        for still in (VehicleState(vx_ms=0.0), VehicleState(vx_ms=1.0)):  # Stillstand / Anfahren: explizit abgewiesen
            with self.assertRaisesRegex(OutsideEnvelopeError, 'contact speed'):
                wheel_slips(still, (0.0,) * 4, 0.0, self.car, c, self.wheel, 0.0)

    def test_Q04_5_reverse_and_near_zero_rejected_without_nan(self):
        """Q04.5: rückwärts und nahe 0 m/s keine NaN/Inf und kein unerkannter Modellwechsel."""
        import warnings
        from torquevector.physics.wheel_rotation import derivative

        with warnings.catch_warnings():
            warnings.simplefilter('error')  # Division durch 0 würde warnen
            for vx, omega in ((-5.0, -15.0), (0.0, 0.0), (1e-9, 0.0), (2.99, 9.4), (18.0, -1.0)):
                x = np.array([0.0, 0.0, 0.0, vx, 0.0, 0.0] + [omega] * 4)
                with self.subTest(vx=vx, omega=omega), self.assertRaises(OutsideEnvelopeError):
                    derivative(0.0, x, self.car, self.tyre, TorqueControls(), self.wheel, tyre_model=self.model)
            x = np.array([0.0, 0.0, 0.0, 3.5, 0.0, 0.0] + [3.5 / self.wheel.radius_m] * 4)  # knapp gültig: endlich
            self.assertTrue(
                np.all(
                    np.isfinite(
                        derivative(0.0, x, self.car, self.tyre, TorqueControls(), self.wheel, tyre_model=self.model)
                    )
                )
            )


if __name__ == '__main__':
    unittest.main()
