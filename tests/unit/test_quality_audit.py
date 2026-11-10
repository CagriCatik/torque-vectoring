"""Bewertungsregel des Qualitäts-Audits: konservativ, nie mehr als belegt, Original-Scorecard unberührt."""

import csv
import tempfile
import unittest
from pathlib import Path

from torquevector.evaluation.quality_audit import DATA_LIMITED, check_id, propose_points, write_scorecard

ROOT = Path(__file__).parents[2]


class ProposalRule(unittest.TestCase):
    def test_check_id_from_test_name(self):
        self.assertEqual(check_id('quality.test_q07_numerics.Q07Numerics.test_Q07_2_dt_versus'), 'Q07.2')
        self.assertIsNone(check_id('quality.test_q10.X.test_Q10_status_required_real_data_declared'))

    def test_points(self):
        p = propose_points({'Q01.1': 'passed', 'Q01.2': 'failed', 'Q02.1': 'passed', 'Q13.3': 'passed'}, 85.0)
        self.assertEqual(p['Q01.1']['points'], 2)
        self.assertEqual(p['Q01.2']['points'], 0)
        self.assertEqual(p['Q02.1']['points'], 1)  # synthetische Daten: nie volle Punkte
        self.assertEqual(p['Q13.3']['points'], 1)  # Coverage < 90 %
        self.assertEqual(propose_points({'Q13.3': 'passed'}, 91.0)['Q13.3']['points'], 2)
        self.assertTrue(all(p[f'Q10.{i}']['points'] == 0 for i in range(1, 6)))  # ohne Messdaten immer 0
        self.assertTrue(set(DATA_LIMITED) <= {'Q02.1', 'Q02.2', 'Q02.3', 'Q03.2'})

    def test_scorecard_written_separately_with_pending_review(self):
        original = (ROOT / 'docs/quality/quality_scorecard.csv').read_bytes()
        with tempfile.TemporaryDirectory() as root:
            target = Path(root) / 'proposed.csv'
            write_scorecard(
                ROOT / 'docs/quality/quality_scorecard.csv',
                target,
                propose_points({'Q01.1': 'passed'}, None),
                {'Q01.1': 'tests/quality/x.py::t'},
            )
            with target.open(encoding='utf-8') as fh:
                rows = {r['check_id']: r for r in csv.DictReader(fh)}
        self.assertEqual(rows['Q01.1']['points'], '2')
        self.assertEqual(rows['Q01.1']['reviewer'], 'PENDING_REVIEW')
        self.assertEqual(rows['Q10.1']['evidence_path'], '')
        self.assertEqual((ROOT / 'docs/quality/quality_scorecard.csv').read_bytes(), original)


if __name__ == '__main__':
    unittest.main()
