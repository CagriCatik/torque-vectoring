"""Offline-PSO aus YAML: Konfigurationswechsel, Hold-out-Trennung, Checkpoint, versionierter Export."""

import hashlib, json, shutil, tempfile, unittest
from pathlib import Path
from torquevector.cli.composition import run_optimization
from torquevector.infrastructure.config.resolver import resolve, resolve_component
from torquevector.infrastructure.config.yaml_io import load_with_origins

ROOT = Path(__file__).parents[2]


def tree_hash(path):
    return hashlib.sha256(b''.join(p.read_bytes() for p in sorted(Path(path).rglob('*.yaml')))).hexdigest()


class StubEvaluator:
    """Deterministische Kennzahlen ohne Simulation; protokolliert jeden Aufruf."""

    def __init__(self):
        self.calls = []

    def __call__(self, kind, case, params):
        self.calls.append((kind, case.id, None if params is None else dict(params)))
        if kind == 'off':
            return {
                'status': 'PASS',
                'metrics': {
                    'yaw_rmse_rads': 0.05,
                    'cross_rmse_m': 0.3,
                    'max_abs_beta_deg': 1.0,
                    'predictive_state_excess': 0.0,
                    'maximum_allocation_residual_nm': 100.0,
                },
            }
        p = params or {
            'weights.beta_scale_rad': 0.06,
            'weights.yaw_scale_rads': 0.07,
            'weights.input_weight': 0.12,
            'weights.slew_weight': 0.06,
        }
        yaw = 0.03 + abs(p['weights.yaw_scale_rads'] - 0.05) + 0.01 * p['weights.input_weight']
        return {
            'status': 'PASS',
            'metrics': {
                'yaw_rmse_rads': yaw,
                'cross_rmse_m': 0.28 + 0.1 * p['weights.slew_weight'],
                'max_abs_beta_deg': 1.0,
                'predictive_state_excess': 0.0,
                'maximum_allocation_residual_nm': 80.0,
            },
        }


class ConfiguredTuning(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.configs = Path(self.tmp.name) / 'configs'
        shutil.copytree(ROOT / 'configs', self.configs, ignore=shutil.ignore_patterns('*.json'))
        self.results = Path(self.tmp.name) / 'results'

    def tearDown(self):
        self.tmp.cleanup()

    def optimize(self, evaluator, **kwargs):
        lines = []
        code = run_optimization(
            self.configs / 'optimization/pso.yaml',
            results_dir=self.results,
            evaluate=evaluator,
            echo=lines.append,
            **kwargs,
        )
        return code, Path(json.loads(lines[-1])['path'])

    def test_export_is_versioned_loadable_and_never_touches_configs(self):
        before = tree_hash(self.configs)
        evaluator = StubEvaluator()
        code, run = self.optimize(evaluator)
        self.assertEqual(code, 0)
        self.assertEqual(tree_hash(self.configs), before)
        self.assertRegex(run.name, r'^\d{8}-\d{6}_nmpc_weights$')
        for name in ('best_parameters.yaml', 'convergence.csv', 'evaluation.json', 'metadata.yaml', 'checkpoint.jsonl'):
            self.assertTrue((run / name).is_file(), name)
        component, section = resolve_component(run / 'best_parameters.yaml', template_ok=False)
        self.assertEqual(component.controller_type, 'nmpc')
        self.assertFalse(section['provenance']['experimentally_validated'])
        report = json.loads((run / 'evaluation.json').read_text())
        self.assertEqual(report['evaluations'], 3 * 2)
        self.assertFalse(report['holdout_used_in_objective'])
        for key, value in report['best_parameters'].items():
            node = section
            for part in key.split('.'):
                node = node[part]
            self.assertEqual(node, value)
        tuned = resolve(
            self.configs / 'default.yaml', replacements={'controllers.nmpc': str(run / 'best_parameters.yaml')}
        )
        self.assertEqual(
            tuned.get('controllers.nmpc.weights.yaw_scale_rads'), report['best_parameters']['weights.yaw_scale_rads']
        )
        metadata, _ = load_with_origins(run / 'metadata.yaml')
        self.assertEqual(metadata['status'], 'FEASIBLE')
        self.assertFalse(metadata['pso_online'])

    def test_holdout_never_evaluated_with_candidates_during_search(self):
        evaluator = StubEvaluator()
        self.optimize(evaluator)
        candidate_calls = [i for i, (kind, _, params) in enumerate(evaluator.calls) if kind == 'nmpc']
        search_calls = candidate_calls[: 6 * 2]
        self.assertTrue(all(evaluator.calls[i][1].startswith('train/') for i in search_calls))
        holdout = [
            i for i, (kind, case, _) in enumerate(evaluator.calls) if case.startswith('holdout/') and kind == 'nmpc'
        ]
        self.assertGreater(min(holdout), max(search_calls))

    def test_pso_configuration_switch_and_checkpoint_resume(self):
        text = (self.configs / 'optimization/pso.yaml').read_text().replace('particles: 3', 'particles: 4')
        (self.configs / 'optimization/pso.yaml').write_text(text)
        code, run = self.optimize(StubEvaluator())
        report = json.loads((run / 'evaluation.json').read_text())
        self.assertEqual(report['evaluations'], 4 * 2)
        resumed = StubEvaluator()
        code2, run2 = self.optimize(resumed, resume=str(run))
        self.assertEqual(run2, run)
        again = json.loads((run / 'evaluation.json').read_text())
        self.assertEqual(again['best_fitness'], report['best_fitness'])
        self.assertTrue(all(c['from_checkpoint'] for c in again['candidates']))
        searched = [c for c in resumed.calls if c[0] == 'nmpc' and c[1].startswith('train/') and c[2] is not None]
        self.assertEqual(len(searched), 2)  # nur die abschließende Trainingsauswertung des Gewinners

    def test_results_inside_configs_are_rejected(self):
        with self.assertRaises(ValueError):
            run_optimization(
                self.configs / 'optimization/pso.yaml',
                results_dir=self.configs / 'x',
                evaluate=StubEvaluator(),
                echo=lambda s: None,
            )


if __name__ == '__main__':
    unittest.main()
