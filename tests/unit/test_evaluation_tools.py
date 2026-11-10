"""Auswertungswerkzeuge: Testsuite-Adapter, Vergleichsmatrix, Manöverkennzahlen, Gate-CLI (Q13.3-Abdeckung)."""

import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from torquevector.domain.scenarios import presets
from torquevector.evaluation.comparisons import assess, compare
from torquevector.evaluation.maneuver_metrics import compute_metrics

ROOT = Path(__file__).parents[2]


class SuiteAdapter(unittest.TestCase):
    def run_suite(self, body):
        from torquevector.evaluation.test_runner import run_tests

        with tempfile.TemporaryDirectory() as root:
            (Path(root) / 'tests/unit').mkdir(parents=True)
            (Path(root) / 'tests/unit/test_probe.py').write_text(body)
            with mock.patch('builtins.print'):
                code = run_tests('smoke', Path(root) / 'results', test_root=Path(root) / 'tests')
            manifest = json.loads(next((Path(root) / 'results').rglob('manifest.json')).read_text())
        return code, manifest

    def test_pass_fail_and_skip_are_explicit(self):
        ok = 'import unittest\nclass T(unittest.TestCase):\n    def test_a(self):pass\n'
        code, manifest = self.run_suite(ok)
        self.assertEqual((code, manifest['status'], manifest['test_count']), (0, 'PASS', 1))
        code, manifest = self.run_suite(ok.replace('pass', 'self.fail()'))
        self.assertEqual((code, manifest['status']), (1, 'FAIL'))
        code, manifest = self.run_suite(ok.replace('pass', "self.skipTest('x')"))
        self.assertEqual(manifest['status'], 'FAIL')  # Skip ist kein Erfolg

    def test_missing_test_tree_rejected(self):
        from torquevector.evaluation.test_runner import run_tests

        with tempfile.TemporaryDirectory() as root, self.assertRaises(ValueError):
            run_tests('smoke', root, test_root=Path(root) / 'nope')


class ComparisonAndMetrics(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.records, cls.evidence = compare(
            [('double_lane_change', 17.0, 2.0), ('circle_track', 15.0, 2.0)],
            controllers=('off', 'p'),
            collect_samples=True,
        )

    def test_comparison_records_and_relative_annotation(self):
        self.assertEqual([r['scenario'] for r in self.records], ['double_lane_change', 'circle_track'])
        for row in self.records:
            self.assertEqual(set(row['controllers']), {'off', 'p'})
            self.assertIn('status', row['controllers']['p'])
        self.assertEqual(set(self.evidence['circle_track']), {'off', 'p'})

    def test_failure_is_fail_closed_with_reason(self):
        from torquevector.control.controllers.nmpc import NMPCConfig

        entry, seq = assess('off', presets()['double_lane_change'], -1.0, NMPCConfig())
        self.assertEqual(entry['status'], 'FAIL_CLOSED')
        self.assertTrue(entry['reason'])
        self.assertIsNone(seq)

    def test_manoeuvre_metrics_for_each_kind(self):
        for scenario in ('double_lane_change', 'circle_track'):
            result = compute_metrics(presets()[scenario], self.evidence[scenario]['p'])
            self.assertIn(result['status'], ('PASS_INTERNAL_SCREENING', 'FAIL_INTERNAL_SCREENING'))
            self.assertTrue(result['metrics']['finite_all_states'])
        circle = compute_metrics(presets()['circle_track'], self.evidence['circle_track']['off'])['metrics']
        self.assertIn('steady_circle_radius_rmse_m', circle)
        for kind in ('circle_open', 'step_steer'):
            seq = self.evidence['circle_track']['off']
            self.assertIn('checks', compute_metrics(presets()[kind], seq))
        with self.assertRaises(ValueError):
            compute_metrics(presets()['circle_track'], self.evidence['circle_track']['off'][:3])


class GateCommandLine(unittest.TestCase):
    def test_main_reports_blocked_and_writes_json(self):
        from torquevector.evaluation import quality_gates

        with tempfile.TemporaryDirectory() as root:
            argv = [
                'quality_gates',
                '--scorecard',
                str(ROOT / 'docs/quality/quality_scorecard.csv'),
                '--manifest',
                str(ROOT / 'docs/quality/quality_gate_manifest.json'),
                '--json',
                f'{root}/r.json',
            ]
            with mock.patch.object(sys, 'argv', argv), mock.patch('builtins.print') as out:
                self.assertEqual(quality_gates.main(), 1)
            report = json.loads(Path(f'{root}/r.json').read_text())
        self.assertFalse(report['pass'])
        self.assertIn(('OVERALL:', 'BLOCKED'), [c.args for c in out.call_args_list])

    def test_tampered_scorecard_rejected(self):
        from torquevector.evaluation.quality_gates import assess as gates

        with tempfile.TemporaryDirectory() as root:
            text = (ROOT / 'docs/quality/quality_scorecard.csv').read_text().replace('Q01.1,', 'Q01.9,', 1)
            (Path(root) / 's.csv').write_text(text)
            with self.assertRaises(ValueError):
                gates(Path(root) / 's.csv', ROOT / 'docs/quality/quality_gate_manifest.json')


if __name__ == '__main__':
    unittest.main()
