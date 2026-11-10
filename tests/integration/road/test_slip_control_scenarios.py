"""Split-mu über der Haftgrenze: ohne ABS/ASR FAIL_CLOSED, mit ABS/ASR definierter Lauf mit begrenztem Schlupf."""

import unittest
from integration.road.helpers import run

ON = [('powertrain.slip_control.abs_enabled', True), ('powertrain.slip_control.asr_enabled', True)]


class AboveAdhesionLimit(unittest.TestCase):
    def check(self, scenario, wet_wheels):
        _, without, *_ = run(scenario)
        self.assertEqual(without['status'], 'FAIL_CLOSED')
        self.assertIn('Slip outside calibrated domain', without['reason'])
        _, entry, seq, _, _ = run(scenario, overrides=ON)
        self.assertEqual(entry['status'], 'PASS')
        m = entry['metrics']
        self.assertLess(m['max_abs_slip'], 0.2)
        for i in wet_wheels:
            self.assertGreater(m['slip_control']['max_reduction'][i], 0.1)
        return entry, seq

    def test_abs_split_mu_braking(self):
        entry, _ = self.check('straight_split_mu_braking_above_limit', (0, 2))
        braking = entry['metrics']['road']['braking']
        self.assertGreater(braking['mean_deceleration_ms2'], 3.0)
        self.assertLess(entry['metrics']['road']['max_lateral_deviation_m'], 0.5)

    def test_asr_split_mu_acceleration(self):
        entry, seq = self.check('straight_split_mu_acceleration_above_limit', (0, 2))
        self.assertGreater(seq[-1].state.vx_ms, seq[0].state.vx_ms + 8.0)

    def test_inactive_slip_control_leaves_regular_scenario_unchanged(self):
        _, a, seq_a, _, _ = run('straight_split_mu_braking')
        _, b, seq_b, _, _ = run('straight_split_mu_braking', overrides=[('powertrain.slip_control.kp', 9.0)])
        self.assertEqual(seq_a, seq_b)
