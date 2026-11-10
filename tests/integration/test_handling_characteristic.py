"""Ramp steer (ISO 4138) on the four-wheel model vs the linear single-track reference, for three vehicle variants."""

import unittest
from functools import partial
from pathlib import Path

from torquevector.cli.composition import SideslipStop, build_runtime, ramp_steer_runtime, run_case
from torquevector.evaluation.handling import characteristic, linear_characteristic, steady_state_curve
from torquevector.infrastructure.config.resolver import resolve
from torquevector.simulation.sampled import simulate_sampled

CONFIGS = Path(__file__).parents[2] / 'configs'
VARIANTS = {
    'reference': CONFIGS / 'default.yaml',
    'understeer': CONFIGS / 'profiles/vehicle_understeer.yaml',
    'oversteer': CONFIGS / 'profiles/vehicle_oversteer.yaml',
}
SPEED = 15.0  # well below the critical speed of the oversteer variant (≈ 30 m/s): quasi-steady ramp valid


def ramp(config):
    base = build_runtime(resolve(config))
    s, car, tyre = base.validation.handling, base.vehicle, base.tyre
    rt, _ = ramp_steer_runtime(config, SPEED)
    stop = SideslipStop(s.stop_sideslip_deg)
    _, seq, _ = run_case(
        'off',
        rt.scenario,
        rt.plant,
        rt.controllers,
        rt.driver,
        simulator=partial(simulate_sampled, stop_condition=stop),
    )
    curve = steady_state_curve(seq, car, start_s=rt.scenario.spec.excitation_start_s)
    linear = linear_characteristic(car, tyre, base.tyre_axle_scaling, neutral_tolerance_deg_per_g=0.25)
    return linear, characteristic(curve, s, stopped_by_sideslip=stop.triggered)


class RampSteerAgainstLinearModel(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.results = {name: ramp(path) for name, path in VARIANTS.items()}

    def test_linear_gradient_matches_single_track_reference(self):
        """Q09-type benchmark: K from the nonlinear four-wheel model in 0.05–0.4 g vs the analytic linear K."""
        for name, (linear, result) in self.results.items():
            k_lin, k_sim = linear['understeer_gradient_deg_per_g'], result['linear_gradient_deg_per_g']
            with self.subTest(variant=name, linear=round(k_lin, 3), simulated=round(k_sim, 3)):
                self.assertLessEqual(abs(k_sim - k_lin), max(0.15 * abs(k_lin), 0.25))

    def test_variants_are_classified_and_ordered(self):
        k = {name: result['linear_gradient_deg_per_g'] for name, (_, result) in self.results.items()}
        self.assertGreater(k['understeer'], k['reference'])
        self.assertGreater(k['reference'], k['oversteer'])
        self.assertEqual(self.results['understeer'][1]['linear_classification'], 'understeer')
        self.assertEqual(self.results['oversteer'][1]['linear_classification'], 'oversteer')
        self.assertEqual(self.results['reference'][1]['linear_classification'], 'neutral')

    def test_limit_behaviour(self):
        self.assertTrue(self.results['understeer'][1]['limit_behaviour'].startswith('limit understeer'))
        self.assertTrue(self.results['reference'][1]['limit_behaviour'].startswith('limit understeer'))
        self.assertIn(
            self.results['oversteer'][1]['limit_behaviour'].split(' (')[0], ('loss of stability', 'limit oversteer')
        )
        for name, (_, result) in self.results.items():
            with self.subTest(variant=name):
                self.assertGreater(result['max_lateral_acceleration_g'], 0.8)
                self.assertLess(result['max_lateral_acceleration_g'], 1.05)


if __name__ == '__main__':
    unittest.main()
