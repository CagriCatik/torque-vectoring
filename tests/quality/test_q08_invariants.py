"""Q08 – Physikalische Invarianten (je Zeitschritt, getaktete Simulation mit Reglern)."""

import unittest
from dataclasses import replace

from quality.common import run_case, runtime
from torquevector.domain.controls import TorqueControls
from torquevector.domain.errors import OutsideEnvelopeError
from torquevector.evaluation.invariants import BALANCE_TOLERANCE, balance_residuals, contact_power_w, invariant_report

SCENARIOS = (
    ('double_lane_change', 'lqr'),
    ('circle_track', 'nmpc'),
    ('mixed_mu_lane_change', 'ltv'),
    ('straight_split_mu_braking', 'p'),
)


class Q08Invariants(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rt = runtime()
        cls.car, cls.wheel = rt.vehicle, rt.wheel
        cls.reports = {}
        for scenario, kind in SCENARIOS:
            _, entry, seq, _ = run_case(scenario, kind, (('scenario.simulation_duration_s', 4.0),))
            cls.reports[(scenario, kind)] = (entry, seq, invariant_report(seq, cls.car, cls.wheel))

    def test_Q08_1_force_and_moment_balance_per_time_step(self):
        """Q08.1 (kritisch): Kraft- und Momentenbilanz je Zeitschritt, Residuen <= 1e-6 relativ."""
        for key, (entry, seq, report) in self.reports.items():
            with self.subTest(case=key):
                self.assertEqual(report['samples'], len(seq))
                self.assertLessEqual(max(report['max_balance_residual'].values()), BALANCE_TOLERANCE)

    def test_Q08_2_no_negative_wheel_load_and_lift_fail_fast(self):
        """Q08.2 (kritisch): keine negativen Fz im Kontakt; Radabheben mit Fail-Fast."""
        for key, (_, _, report) in self.reports.items():
            with self.subTest(case=key):
                self.assertGreater(report['min_wheel_load_n'], 0)
        from torquevector.physics.wheel_rotation import validate_no_lift

        with self.assertRaisesRegex(OutsideEnvelopeError, 'wheel lift'):
            validate_no_lift(self.car, -0.9 * self.car.gravity * self.car.wheelbase_m / self.car.cg_height_m, 0.0)

    def test_Q08_3_force_limits_and_power_signs_consistent(self):
        """Q08.3: Reifenkraftgrenzen sowie Reifen-/Rad-Leistungsvorzeichen in allen Testfällen konsistent."""
        for key, (_, seq, report) in self.reports.items():
            with self.subTest(case=key):
                self.assertLessEqual(report['max_force_limit_excess'], 1e-6)
                self.assertLessEqual(report['max_contact_power_w'], 1e-6)
                for s in seq:  # Bremse entzieht immer Leistung
                    self.assertTrue(all(b * w >= 0 for b, w in zip(s.brake_nm, s.omega_rad_s)))

    def test_Q08_4_kinetic_energy_does_not_increase_when_coasting(self):
        """Q08.4: Freilauf ohne Energiequelle erhöht die kinetische Energie nicht (inkl. Rotationsenergie)."""
        from torquevector.evaluation.diagnostics import energy_j
        from torquevector.simulation.continuous import simulate_torque

        rt = runtime()
        for steer in (0.0, 2.0, -4.0):
            for car in (self.car, replace(self.car, cd_area_m2=0.0, rolling_coefficient=0.0)):  # auch ohne Verlustterme
                seq = simulate_torque(
                    car,
                    rt.tyre,
                    TorqueControls(steer, (0.0,) * 4),
                    self.wheel,
                    duration_s=3.0,
                    output_dt=0.01,
                    tyre_model=rt.tyre_model,
                )
                energy = [energy_j(s, car, self.wheel) for s in seq]
                with self.subTest(steer=steer, losses=car.cd_area_m2 > 0):
                    self.assertTrue(all(b <= a * (1 + 1e-9) for a, b in zip(energy, energy[1:])))

    def test_Q08_5_residual_reports_and_automatic_alarms_per_scenario(self):
        """Q08.5: Impuls-/Energie-Residualberichte und automatische Alarme je Szenario."""
        for key, (_, _, report) in self.reports.items():
            with self.subTest(case=key):
                self.assertEqual(report['alarms'], [])
                self.assertTrue(report['passed'])
        _, seq, _ = self.reports[SCENARIOS[0]]
        broken = seq[:5] + [replace(seq[5], forces=replace(seq[5].forces, mz_nm=seq[5].forces.mz_nm + 50.0))]
        alarm = invariant_report(broken, self.car, self.wheel)
        self.assertFalse(alarm['passed'])
        self.assertTrue(any('moment_z' in a for a in alarm['alarms']))  # Alarm benennt die verletzte Bilanz
        self.assertGreater(balance_residuals(broken[-1], self.car)['moment_z'], BALANCE_TOLERANCE)
        self.assertIsInstance(contact_power_w(seq[3], self.car, self.wheel), float)


if __name__ == '__main__':
    unittest.main()
