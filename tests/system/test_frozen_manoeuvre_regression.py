"""Frozen pre-migration complete maneuvers; tolerances fixed before extraction."""

import json, os, unittest
from pathlib import Path
import numpy as np
from torquevector.domain.scenarios import presets
from torquevector.simulation.runner import simulate_controller


class StructureNumericalRegression(unittest.TestCase):
    def test_complete_nominal_maneuvers_unchanged(self):
        rows = json.loads((Path(__file__).parents[1] / 'fixtures/structure_nominal_reference.json').read_text())
        actual = []
        for row in rows:
            with self.subTest(scenario=row['scenario'], controller=row['controller']):
                metrics, seq = simulate_controller(row['controller'], presets()[row['scenario']], row['duration_s'])
                actual.append({**row, 'metrics': metrics, 'final_x_m': seq[-1].state.x_m})
                for key, expected in row['metrics'].items():
                    np.testing.assert_allclose(metrics[key], expected, rtol=1e-9, atol=1e-10, err_msg=key)
                if row['scenario'] == 'double_lane_change':
                    self.assertGreaterEqual(seq[-1].state.x_m, 129.0)
        if os.environ.get('TORQUEVECTOR_TEST_RESULTS'):
            out = Path(os.environ['TORQUEVECTOR_TEST_RESULTS'])
            out.mkdir(parents=True, exist_ok=True)
            (out / 'numerical_regression.json').write_text(json.dumps(actual, indent=2) + '\n')
