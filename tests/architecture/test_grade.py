"""Self-tests for the quality gate evaluator, independent of simulation tests."""

from __future__ import annotations

import csv
import tempfile
import unittest
from pathlib import Path
from torquevector.evaluation.quality_gates import assess


from torquevector.infrastructure.configuration import resource_path

BASE = resource_path('quality')


class QualityGateTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.csv_path = Path(self.tmp.name) / 'score.csv'
        with (BASE / 'quality_scorecard.csv').open(encoding='utf-8', newline='') as stream:
            r = csv.DictReader(stream)
            self.columns = r.fieldnames
            self.rows = list(r)

    def run_score(self):
        with self.csv_path.open('w', encoding='utf-8', newline='') as stream:
            w = csv.DictWriter(stream, self.columns)
            w.writeheader()
            w.writerows(self.rows)
        return assess(self.csv_path, BASE / 'quality_gate_manifest.json')

    def test_blank_is_not_evaluated(self):
        report, passed = self.run_score()
        self.assertFalse(passed)
        self.assertIsNone(report['categories']['Q01']['score'])

    def test_complete_evidence_passes(self):
        for row in self.rows:
            row['points'] = '2'
            row['evidence_path'] = 'reports/sample.json'
        report, passed = self.run_score()
        self.assertTrue(passed)
        self.assertEqual(report['category_count'], 13)

    def test_critical_one_is_blocker_even_with_nine(self):
        for row in self.rows:
            row['points'] = '2'
            row['evidence_path'] = 'reports/sample.json'
        critical = next(r for r in self.rows if r['category'] == 'Q01' and r['critical'] == 'true')
        critical['points'] = '1'
        report, passed = self.run_score()
        self.assertFalse(passed)
        self.assertEqual(report['categories']['Q01']['score'], 9)
        self.assertFalse(report['categories']['Q01']['passed'])

    def test_noncritical_zero_still_allows_eight(self):
        for row in self.rows:
            row['points'] = '2'
            row['evidence_path'] = 'reports/sample.json'
        row = next(r for r in self.rows if r['category'] == 'Q01' and r['critical'] == 'false')
        row['points'] = '0'
        report, passed = self.run_score()
        self.assertTrue(passed)
        self.assertEqual(report['categories']['Q01']['score'], 8)

    def test_missing_evidence_blocks(self):
        for row in self.rows:
            row['points'] = '2'
            row['evidence_path'] = 'reports/sample.json'
        self.rows[0]['evidence_path'] = ''
        report, passed = self.run_score()
        self.assertFalse(passed)
        self.assertIsNone(report['categories']['Q01']['score'])

    def test_deleted_category_is_error(self):
        self.rows = [r for r in self.rows if r['category'] != 'Q01']
        with self.assertRaisesRegex(ValueError, 'Category mismatch'):
            self.run_score()


if __name__ == '__main__':
    unittest.main()
