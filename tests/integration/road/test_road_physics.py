"""Wirkungskette Reibwertkarte -> Radkontakt -> Reifenkraft -> Fahrzeugbeschleunigung."""

import unittest
from dataclasses import replace
from math import hypot
from torquevector.domain.controls import TorqueControls
from torquevector.domain.road_surface import RoadSurface, SurfaceRegion
from torquevector.domain.states import VehicleState
from torquevector.domain.vehicle import VehicleParameters
from torquevector.domain.wheels import WHEEL_ORDER, WheelParameters
from torquevector.infrastructure.configuration import load_model
from torquevector.physics.road_friction import RoadFrictionMap, spatial_tyre_binding, wheel_contacts
from torquevector.physics.tyre_magic_formula import peak_grip, tyre_forces
from torquevector.physics.wheel_rotation import forces_at

CAR = VehicleParameters()
WHEEL = WheelParameters()
TYRE, MODEL = load_model()
CONTROLS = TorqueControls(2.0, (120.0, 120.0, 180.0, 180.0), (0.0, 0.0, 0.0, 0.0), 800.0)


def road(regions=(), default=0.9, model='scale_tyre_mu'):
    return RoadFrictionMap(RoadSurface('t', 'dry', default, tuple(regions), 'sharp', 0.0, model, 0.9))


def wet(x0=None, x1=None, y0=None, y1=None, mu=0.4):
    return SurfaceRegion('wet', 'wet', mu, x0, x1, y0, y1)


def forces(state, field, base_model=MODEL):
    omega = tuple((1.04 * v) / WHEEL.radius_m for v in (state.vx_ms,) * 4)
    bind = spatial_tyre_binding(field, CAR, TYRE, base_model)
    return forces_at(state, omega, 1.0, CAR, TYRE, CONTROLS, WHEEL, bind(state), 2.0, 'root')


STATE = VehicleState(x_m=0.0, y_m=0.0, yaw_rad=0.0, vx_ms=15.0, vy_ms=0.2, yaw_rate_rads=0.05)


class UniformReproducesConstantModel(unittest.TestCase):
    def test_neutral_uniform_map_is_bitidentical(self):
        omega = tuple((1.04 * 15.0) / WHEEL.radius_m for _ in range(4))
        reference, _ = forces_at(STATE, omega, 1.0, CAR, TYRE, CONTROLS, WHEEL, MODEL, 2.0, 'root')
        mapped, _ = forces(STATE, road())
        self.assertEqual(reference, mapped)

    def test_replace_mode_with_tyre_mu_is_identical(self):
        mapped, _ = forces(STATE, road(default=TYRE.mu, model='replace_tyre_mu'))
        omega = tuple((1.04 * 15.0) / WHEEL.radius_m for _ in range(4))
        reference, _ = forces_at(STATE, omega, 1.0, CAR, TYRE, CONTROLS, WHEEL, MODEL, 2.0, 'root')
        self.assertEqual(reference, mapped)


class LocalFrictionChangesForces(unittest.TestCase):
    def test_lower_mu_reduces_available_force_at_same_slip_and_load(self):
        for model in (MODEL, tyre_forces):
            for kappa, alpha in ((0.3, 0.0), (0.0, 0.2), (0.15, 0.12)):
                with self.subTest(model=getattr(model, '__name__', 'combined'), kappa=kappa, alpha=alpha):
                    dry = model(kappa, alpha, 4000.0, TYRE)
                    w = model(kappa, alpha, 4000.0, replace(TYRE, mu=TYRE.mu * 0.4 / 0.9))
                    self.assertLess(hypot(w.fx_n, w.fy_n), hypot(dry.fx_n, dry.fy_n))
                    self.assertAlmostEqual(
                        w.peak_n / dry.peak_n, 0.4 / 0.9, 12
                    )  # genau eine Skalierung, keine doppelte
                    self.assertLessEqual(w.utilization, 1 + 1e-12)

    def test_each_wheel_uses_its_contact_point_not_the_cg(self):
        # Grenze zwischen den Achsen: Vorderräder nass, Hinterräder und Schwerpunkt trocken
        field = road([wet(x0=0.5)])
        state = replace(STATE, x_m=0.0)
        f, _ = forces(state, field)
        contacts = wheel_contacts(field, state, CAR, TYRE)
        self.assertEqual(field.friction_at(state.x_m, state.y_m), 0.9)
        for name in WHEEL_ORDER:
            expected = peak_grip(f.wheels[name].fz_n, replace(TYRE, mu=contacts[name].tyre_mu))
            self.assertAlmostEqual(f.wheels[name].peak_n, expected, 9)
        self.assertEqual([contacts[n].road_mu for n in WHEEL_ORDER], [0.4, 0.4, 0.9, 0.9])

    def test_split_mu_changes_left_wheels_and_vehicle_response(self):
        dry, _ = forces(STATE, road())
        split, _ = forces(STATE, road([wet(y0=0.0)]))
        for name in ('FL', 'RL'):
            self.assertLess(split.wheels[name].peak_n, dry.wheels[name].peak_n)
        # rechts unveränderter Reifenreibwert bei der (durch Lastverlagerung geänderten) aktuellen Radlast
        for name in ('FR', 'RR'):
            self.assertAlmostEqual(split.wheels[name].peak_n, peak_grip(split.wheels[name].fz_n, TYRE), 9)
        self.assertNotAlmostEqual(split.wheels['FR'].fz_n, dry.wheels['FR'].fz_n, 3)  # Lastverlagerung wirkt mit
        self.assertNotAlmostEqual(split.mz_nm, dry.mz_nm, 3)  # Kräfte -> Giermoment
        self.assertAlmostEqual(sum(w.fz_n for w in split.wheels.values()), CAR.mass_kg * CAR.gravity, 6)

    def test_combined_slip_bound_holds_on_wet(self):
        state = replace(STATE, vy_ms=1.5, yaw_rate_rads=0.3)
        f, _ = forces(state, road(default=0.4))
        for w in f.wheels.values():
            self.assertLessEqual(w.utilization, 1 + 1e-12)

    def test_tyre_parameters_not_mutated(self):
        before = replace(TYRE)
        forces(STATE, road([wet(y0=0.0)]))
        self.assertEqual(TYRE, before)


if __name__ == '__main__':
    unittest.main()
