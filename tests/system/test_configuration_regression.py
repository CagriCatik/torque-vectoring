"""YAML-Pfad reproduziert die vor der Konfigurationsmigration eingefrorenen Vollmanöver.

Gleiche Toleranzen wie tests/system/test_frozen_manoeuvre_regression.py (rtol=1e-9, atol=1e-10),
aber vollständig über configs/profiles/regression.yaml -> Loader -> Composition Root.
"""

import json, unittest
from pathlib import Path
import numpy as np
from torquevector.cli.composition import build_runtime, evaluate_case
from torquevector.infrastructure.config.resolver import matrix_cases, resolve, resolve_case

ROOT = Path(__file__).parents[2]


class YamlRegressionEqualsFrozenReference(unittest.TestCase):
    def test_regression_profile_reproduces_reference(self):
        rows = json.loads((ROOT / 'tests/fixtures/structure_nominal_reference.json').read_text())
        resolved = resolve(ROOT / 'configs/profiles/regression.yaml')
        cases = {case['id']: resolve_case(resolved, case) for case in matrix_cases(resolved, 'regression')}
        ids = {'circle_track': 'circle_track_15', 'double_lane_change': 'double_lane_change_17'}
        for row in rows:
            with self.subTest(scenario=row['scenario'], controller=row['controller']):
                case = cases[ids[row['scenario']]]
                self.assertEqual(case.get('scenario.simulation_duration_s'), row['duration_s'])
                entry, seq, _ = evaluate_case(build_runtime(case), row['controller'])
                self.assertEqual(entry['status'], 'PASS')
                for key, expected in row['metrics'].items():
                    np.testing.assert_allclose(entry['metrics'][key], expected, rtol=1e-9, atol=1e-10, err_msg=key)


if __name__ == '__main__':
    unittest.main()
