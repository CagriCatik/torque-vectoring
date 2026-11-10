"""Eingefrorene Multi-mu-Referenz (tests/fixtures/road_multi_mu_reference.json) und neutrale Fahrbahn."""

import json, unittest
from pathlib import Path
import numpy as np
from torquevector.domain.scenarios import presets
from torquevector.simulation.runner import simulate_controller

ROOT = Path(__file__).parents[3]


class FrozenMultiMuReference(unittest.TestCase):
    def test_reference_cases_unchanged(self):
        from integration.road.helpers import run

        for row in json.loads((ROOT / 'tests/fixtures/road_multi_mu_reference.json').read_text()):
            with self.subTest(scenario=row['scenario'], controller=row['controller']):
                _, entry, seq, _, _ = run(row['scenario'], row['controller'])
                self.assertEqual(entry['status'], row['status'])
                for key, value in row['metrics'].items():
                    np.testing.assert_allclose(entry['metrics'][key], value, rtol=1e-9, atol=1e-10, err_msg=key)
                np.testing.assert_allclose(
                    [seq[-1].state.x_m, seq[-1].state.y_m], [row['final_x_m'], row['final_y_m']], rtol=1e-9, atol=1e-10
                )

    def test_uniform_dry_road_reproduces_constant_mu_model(self):
        from integration.road.helpers import run

        rt, entry, seq, _, _ = run(
            'double_lane_change',
            'lqr',
            [
                ('scenario.simulation_duration_s', 3.0),
                ('simulation.driver.yaw_reference.source', 'path_curvature'),
                ('allocation.friction_limit.mode', 'off'),
            ],
        )
        metrics, legacy = simulate_controller('lqr', presets()['double_lane_change'], 3.0)
        self.assertTrue(rt.scenario.surface.is_identity)
        for key in ('yaw_rmse_rads', 'cross_rmse_m', 'max_tyre_utilization', 'final_speed_ms'):
            self.assertEqual(entry['metrics'][key], metrics[key])
        self.assertEqual(seq[-1].state, legacy[-1].state)


if __name__ == '__main__':
    unittest.main()
