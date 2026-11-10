"""Die vier Multi-mu-Szenarien: Wirkungskette bis zur Fahrzeugbewegung, Reproduzierbarkeit, Schrittweite."""

import unittest
from torquevector.domain.wheels import WHEEL_ORDER
from torquevector.physics.tyre_magic_formula import peak_grip
from dataclasses import replace
from integration.road.helpers import CONFIGS, run


class StraightMultiMu(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.rt, cls.entry, cls.seq, cls.contacts, _ = run('straight_multi_mu')

    def test_completes_and_wheels_pass_sections_in_order(self):
        self.assertEqual(self.entry['status'], 'PASS')

        def first_wet(name):
            return next(r['t_s'] for r in self.contacts if r[name]['road_mu'] < 0.5)

        a, b = self.rt.vehicle.cg_to_front_m, self.rt.vehicle.wheelbase_m
        self.assertAlmostEqual(first_wet('RL') - first_wet('FL'), b / 15.0, delta=0.05)  # Radstand / Geschwindigkeit
        split = [r for r in self.contacts if 72 <= r['FL']['x_m'] < 84 and 72 <= r['RL']['x_m'] < 84]
        self.assertTrue(split)
        for row in split:
            self.assertEqual(
                (row['FL']['road_mu'], row['RL']['road_mu'], row['FR']['road_mu'], row['RR']['road_mu']),
                (0.4, 0.4, 0.9, 0.9),
            )

    def test_telemetry_peak_matches_local_mu(self):
        for sample, row in zip(self.seq, self.contacts):
            for name in WHEEL_ORDER:
                w = sample.forces.wheels[name]
                self.assertAlmostEqual(w.peak_n, peak_grip(w.fz_n, replace(self.rt.tyre, mu=row[name]['tyre_mu'])), 6)

    def test_road_metrics_present(self):
        road = self.entry['metrics']['road']
        self.assertEqual(road['min_road_mu'], {'FL': 0.4, 'FR': 0.4, 'RL': 0.4, 'RR': 0.4})
        self.assertAlmostEqual(road['max_left_right_mu_difference'], 0.5)
        self.assertGreater(road['time_on_reduced_mu_s']['FL'], road['time_on_reduced_mu_s']['FR'])


class SplitMuBrakingAndAcceleration(unittest.TestCase):
    def test_braking_decelerates_with_asymmetric_utilisation(self):
        _, entry, seq, contacts, _ = run('straight_split_mu_braking')
        self.assertEqual(entry['status'], 'PASS')
        late = seq[-1].forces.wheels
        self.assertGreater(late['FL'].utilization, 1.8 * late['FR'].utilization)
        self.assertAlmostEqual(late['FL'].fx_n, late['FR'].fx_n, delta=20.0)  # momentgesteuerte Bremse, kein ABS
        braking = entry['metrics']['road']['braking']
        self.assertLess(braking['end_speed_ms'], braking['start_speed_ms'] - 5)
        self.assertGreater(braking['distance_m'], 0)

    def test_acceleration_gains_speed_on_split_mu(self):
        _, entry, seq, _, _ = run('straight_split_mu_acceleration')
        self.assertEqual(entry['status'], 'PASS')
        self.assertGreater(seq[-1].state.vx_ms, seq[0].state.vx_ms + 5)
        late = seq[-1].forces.wheels
        self.assertGreater(late['RL'].utilization, 1.8 * late['RR'].utilization)

    def test_dry_reference_is_symmetric(self):
        _, entry, seq, contacts, _ = run(
            'straight_split_mu_braking', replacements={'road': str(CONFIGS / 'roads/dry_asphalt.yaml')}
        )
        self.assertEqual(entry['status'], 'PASS')
        self.assertAlmostEqual(seq[-1].forces.wheels['FL'].utilization, seq[-1].forces.wheels['FR'].utilization, 3)
        self.assertEqual({r[n]['road_mu'] for r in contacts for n in WHEEL_ORDER}, {0.9})

    def test_lane_change_on_mixed_road(self):
        _, entry, _, contacts, _ = run('mixed_mu_lane_change', 'lqr')
        self.assertEqual(entry['status'], 'PASS')
        self.assertLess(min(r[n]['road_mu'] for r in contacts for n in WHEEL_ORDER), 0.5)


class ReproducibilityAndStepSize(unittest.TestCase):
    def test_identical_repeat(self):
        a = run('straight_multi_mu', overrides=[('scenario.simulation_duration_s', 4.0)])[1]
        b = run('straight_multi_mu', overrides=[('scenario.simulation_duration_s', 4.0)])[1]
        self.assertEqual(a, b)

    def test_step_size_refinement(self):
        coarse = run('mixed_mu_lane_change', 'off', [('scenario.simulation_duration_s', 5.0)])[2]
        fine = run(
            'mixed_mu_lane_change',
            'off',
            [
                ('scenario.simulation_duration_s', 5.0),
                ('simulation.integrator.max_step_s', 0.005),
                ('simulation.integrator.max_rhs_calls_per_period', 3000),
            ],
        )[2]
        for a, b in zip(coarse, fine):
            self.assertLess(abs(a.state.y_m - b.state.y_m), 0.01)
            self.assertLess(abs(a.state.yaw_rate_rads - b.state.yaw_rate_rads), 0.005)


if __name__ == '__main__':
    unittest.main()
