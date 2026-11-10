import csv, hashlib, unittest, zipfile
from pathlib import Path


class HistoricalPreservation(unittest.TestCase):
    def test_every_original_file_remains_traceable_by_hash(self):
        root = Path(__file__).parents[2]
        migration_map = root / 'archive/reports/takeover/MIGRATION_MAP.csv'
        if migration_map.is_file():
            with migration_map.open() as stream:
                rows = list(csv.DictReader(stream))
            for row in rows:
                with self.subTest(source=row['source']):
                    evidence = row['preservation_evidence']
                    if '::' in evidence:
                        archive, entry = evidence.split('::', 1)
                        with zipfile.ZipFile(root / archive) as z:
                            data = z.read(entry)
                    else:
                        data = (root / evidence).read_bytes()
                    self.assertEqual(hashlib.sha256(data).hexdigest(), row['sha256_before'])
        else:
            for rel in (
                'tests/fixtures/structure_definition_hashes.json',
                'tests/fixtures/core_numeric_defaults.json',
                'tests/fixtures/structure_nominal_reference.json',
                'tests/fixtures/road_multi_mu_reference.json',
                'data/synthetic/fit_report.json',
                'data/synthetic/metadata.json',
                'data/synthetic/synthetic_brush_measurements.csv',
                'docs/quality/quality_gate_manifest.json',
                'docs/quality/quality_scorecard.csv',
            ):
                with self.subTest(file=rel):
                    self.assertTrue((root / rel).is_file(), rel)
                    self.assertGreater((root / rel).stat().st_size, 0, rel)

    def test_packaged_defaults_match_repository_sources(self):
        from torquevector.infrastructure.configuration import resource_path

        root = Path(__file__).parents[2]
        for relative, source in [
            ('synthetic/fit_report.json', 'data/synthetic/fit_report.json'),
            ('quality/quality_gate_manifest.json', 'docs/quality/quality_gate_manifest.json'),
            ('quality/quality_scorecard.csv', 'docs/quality/quality_scorecard.csv'),
        ]:
            self.assertEqual(resource_path(relative).read_bytes(), (root / source).read_bytes(), relative)
