"""Reibwertkarte, Radaufstandspunkte, Übergänge, Validierung (ohne Fahrzeugsimulation)."""

import math, unittest
from pathlib import Path
from torquevector.domain.road_surface import RoadFrictionField, RoadSurface, SurfaceRegion
from torquevector.domain.states import VehicleState
from torquevector.domain.vehicle import VehicleParameters
from torquevector.physics.road_friction import RoadFrictionMap, boundary_crossing_times
from torquevector.physics.wheel_contact import wheel_contact_positions

CAR = VehicleParameters()


def surface(regions=(), transition='sharp', width=0.0, model='scale_tyre_mu', default=0.9):
    return RoadSurface('t', 'dry', default, tuple(regions), transition, width, model, 0.9)


def rect(i, mu, x0=None, x1=None, y0=None, y1=None):
    return SurfaceRegion(i, 'wet', mu, x0, x1, y0, y1)


class WheelGeometry(unittest.TestCase):
    def test_contact_points_at_zero_and_rotated_yaw(self):
        a, b, h = CAR.cg_to_front_m, CAR.wheelbase_m - CAR.cg_to_front_m, CAR.track_m / 2
        p = wheel_contact_positions(VehicleState(x_m=10.0, y_m=2.0, yaw_rad=0.0), CAR)
        self.assertEqual(p['FL'], (10 + a, 2 + h))
        self.assertEqual(p['RR'], (10 - b, 2 - h))
        q = wheel_contact_positions(VehicleState(x_m=10.0, y_m=2.0, yaw_rad=math.pi / 2), CAR)
        for name, (lx, ly) in CAR.contact_points().items():
            self.assertAlmostEqual(q[name][0], 10 - ly, 12)
            self.assertAlmostEqual(q[name][1], 2 + lx, 12)
        r = wheel_contact_positions(VehicleState(yaw_rad=0.3), CAR)
        for name, (lx, ly) in CAR.contact_points().items():
            self.assertAlmostEqual(math.hypot(*r[name]), math.hypot(lx, ly), 12)


class FrictionMapQueries(unittest.TestCase):
    def test_protocol_and_four_different_wheel_values(self):
        field = RoadFrictionMap(
            surface(
                [
                    rect('q1', 0.3, 0, None, 0, None),
                    rect('q2', 0.5, 0, None, None, 0),
                    rect('q3', 0.7, None, 0, 0, None),
                ]
            )
        )
        self.assertIsInstance(field, RoadFrictionField)
        mus = {
            n: field.friction_at(*xy) for n, xy in wheel_contact_positions(VehicleState(x_m=0.0, y_m=0.0), CAR).items()
        }
        self.assertEqual(mus, {'FL': 0.3, 'FR': 0.5, 'RL': 0.7, 'RR': 0.9})
        self.assertEqual(len(set(mus.values())), 4)

    def test_half_open_boundaries_priority_and_repeatability(self):
        field = RoadFrictionMap(surface([rect('wet', 0.4, 30, 45, -4, 4), rect('patch', 0.2, 40, 50, -4, 4)]))
        self.assertEqual(field.friction_at(30.0, 0.0), 0.4)  # x_min inklusiv
        self.assertEqual(field.friction_at(29.999999, 0.0), 0.9)
        self.assertEqual(field.friction_at(45.0, 0.0), 0.2)  # spätere Region gewinnt
        self.assertEqual(field.friction_at(50.0, 0.0), 0.9)  # x_max exklusiv
        self.assertEqual(field.friction_at(35.0, 4.0), 0.9)  # y_max exklusiv
        self.assertEqual(field.surface_at(42.0, 0.0), 'patch')
        self.assertEqual(field.surface_at(10.0, 0.0), 'dry')
        self.assertEqual([field.friction_at(44.9, 1.0) for _ in range(5)], [0.2] * 5)

    def test_smooth_transition_is_continuous_and_centred(self):
        field = RoadFrictionMap(surface([rect('wet', 0.4, 30, None)], 'smooth', 2.0))
        self.assertAlmostEqual(field.friction_at(30.0, 0.0), 0.65, 12)
        self.assertEqual(field.friction_at(28.9, 0.0), 0.9)
        self.assertEqual(field.friction_at(31.1, 0.0), 0.4)
        values = [field.friction_at(29 + 0.01 * i, 0.0) for i in range(201)]
        self.assertTrue(all(b <= a + 1e-15 for a, b in zip(values, values[1:])))
        self.assertLess(max(abs(b - a) for a, b in zip(values, values[1:])), 0.02)

    def test_tyre_coupling_modes_and_identity(self):
        from torquevector.domain.tyre import TyreParameters

        tyre = TyreParameters(mu=1.0)
        self.assertAlmostEqual(RoadFrictionMap(surface()).tyre_mu(tyre, 0.4), 0.4 / 0.9)
        self.assertEqual(RoadFrictionMap(surface(model='replace_tyre_mu')).tyre_mu(tyre, 0.4), 0.4)
        self.assertTrue(RoadFrictionMap(surface()).is_identity)
        self.assertFalse(RoadFrictionMap(surface([rect('w', 0.4, 0, 1)])).is_identity)
        self.assertFalse(RoadFrictionMap(surface(default=0.4)).is_identity)

    def test_invalid_values_rejected(self):
        for kwargs in ({'mu': -0.1}, {'mu': float('nan')}, {'mu': float('inf')}, {'mu': 3.0}):
            with self.subTest(**kwargs), self.assertRaises(ValueError):
                rect('x', kwargs['mu'], 0, 1)
        with self.assertRaises(ValueError):
            rect('x', 0.4, 5, 5)
        with self.assertRaises(ValueError):
            surface(transition='smooth', width=0.0)
        with self.assertRaises(ValueError):
            surface(transition='sharp', width=1.0)
        with self.assertRaises(ValueError):
            surface([rect('a', 0.4, 0, 1), rect('a', 0.5, 1, 2)])

    def test_boundary_crossing_events(self):
        field = RoadFrictionMap(surface([rect('wet', 0.4, 30, 45)]))
        times = boundary_crossing_times(field, VehicleState(x_m=28.0, vx_ms=10.0), CAR, 0.5)
        a, b = CAR.cg_to_front_m, CAR.wheelbase_m - CAR.cg_to_front_m
        self.assertTrue(any(abs(t - (30 - 28 - a) / 10) < 1e-9 for t in times))
        self.assertTrue(any(abs(t - (30 - 28 + b) / 10) < 1e-9 for t in times))
        self.assertEqual(boundary_crossing_times(field, VehicleState(x_m=0.0, vx_ms=10.0), CAR, 0.04), [])
        smooth = RoadFrictionMap(surface([rect('wet', 0.4, 30, 45)], 'smooth', 1.0))
        self.assertEqual(boundary_crossing_times(smooth, VehicleState(x_m=28.0, vx_ms=10.0), CAR, 0.5), [])


class RoadYamlValidation(unittest.TestCase):
    def issues(self, text):
        import tempfile
        from torquevector.infrastructure.config import ConfigurationError
        from torquevector.infrastructure.config.resolver import resolve_component

        base = (Path(__file__).parents[3] / 'configs/roads/mixed_surface.yaml').read_text()
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / 'r.yaml'
            p.write_text(text(base))
            try:
                resolve_component(p, template_ok=False)
            except ConfigurationError as exc:
                return '\n'.join(exc.issues)
        return ''

    def test_reference_files_valid_and_errors_detected(self):
        self.assertEqual(self.issues(lambda s: s), '')
        cases = [
            (lambda s: s.replace('mu: 0.40', 'mu: -0.40', 1), 'unterschreitet'),
            (lambda s: s.replace('mu: 0.40', 'mu: .nan', 1), 'endlich'),
            (lambda s: s.replace('units: meters', 'units: feet'), 'nicht zulässig'),
            (lambda s: s.replace('x_min_m: 30.0', 'x_min_m: 50.0'), 'kleiner als'),
            (lambda s: s.replace('type: rectangle', 'type: split_mu', 1), 'split_mu benötigt'),
            (lambda s: s.replace('mode: sharp', 'mode: smooth'), 'smooth erfordert'),
            (lambda s: s.replace('id: split_mu_right_dry', 'id: wet_section_1'), 'doppelte'),
            (
                lambda s: s.replace('    - id: wet_section_1', '    - id: wet_section_1\n      colour: blue'),
                'unbekannter',
            ),
        ]
        for edit, expected in cases:
            with self.subTest(expected=expected):
                self.assertIn(expected, self.issues(edit))


if __name__ == '__main__':
    unittest.main()
