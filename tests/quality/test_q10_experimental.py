"""Q10 – Experimentelle Validierung: KEINE realen Messdaten vorhanden.

Diese Tests weisen nichts nach, sie sichern nur ab, dass das Projekt nirgends eine experimentelle
Validierung behauptet (Blocker B5). Alle Q10-Checks erhalten 0 Punkte (DATA_LIMITED, REQUIRED_REAL_DATA)."""

import json
import unittest

from quality.common import ROOT


class Q10NoFalseValidationClaim(unittest.TestCase):
    def test_Q10_status_required_real_data_declared(self):
        """Q10.1–Q10.5: Status REQUIRED_REAL_DATA in Daten, Läufen, Konfiguration und Doku ausgewiesen."""
        report = json.loads((ROOT / 'data/synthetic/fit_report.json').read_text())
        self.assertFalse(report['experimental_validation'])
        self.assertIn('NO PHYSICAL MEASUREMENTS', report['metadata']['measurement_status'])
        for path in ROOT.glob('configs/vehicles/*.yaml'):
            self.assertIn('validated_against_real_vehicle: false', path.read_text())
        import tempfile

        from torquevector.infrastructure.experiment_store import ExperimentStore

        with tempfile.TemporaryDirectory() as root:  # jedes Lauf-Manifest kennzeichnet den Status
            manifest = ExperimentStore(root, {}).manifest
        self.assertIs(manifest['real_vehicle_validation'], False)
        self.assertEqual(manifest['q10'], 'BLOCKED_REQUIRED_REAL_DATA')
        self.assertIn('REQUIRED_REAL_DATA', (ROOT / 'README.md').read_text())
        self.assertFalse(list(ROOT.glob('data/measurements/**/*.csv')))  # kein Messdatensatz eingecheckt


if __name__ == '__main__':
    unittest.main()
