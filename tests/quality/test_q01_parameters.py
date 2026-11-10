"""Q01 – Parameter und Geltungsbereich."""

import json
import re
import tempfile
import unittest
from dataclasses import replace
from math import inf, nan

from quality.common import CONFIGS, ROOT, runtime

SPEC = (ROOT / 'docs/model/specification.md').read_text(encoding='utf-8')


def table_value(label):
    """Zahlenwert der Tabellenzeile, deren erste Spalte ``label`` enthält."""
    for line in SPEC.splitlines():
        cells = [c.strip() for c in line.strip('|').split('|')]
        if len(cells) >= 3 and label in cells[0]:
            return float(cells[1].replace(' ', '').replace('−', '-'))
    raise AssertionError(f'{label} not in docs/model/specification.md')


class Q01Parameters(unittest.TestCase):
    def test_Q01_1_vehicle_parameters_documented_with_source_unit_uncertainty(self):
        """Q01.1 (kritisch): Masse, Trägheit, Radstand, Spurweite, Schwerpunkt mit Quelle, Einheit, Unsicherheit."""
        car = runtime().vehicle
        for label, value in (
            ('mass_kg', car.mass_kg),
            ('yaw_kgm2', car.iz_kgm2),
            ('wheelbase_m', car.wheelbase_m),
            ('track_m', car.track_m),
            ('cg_to_front_axle_m', car.cg_to_front_m),
            ('cg_height_m', car.cg_height_m),
        ):
            self.assertAlmostEqual(table_value(label), value, 9, label)
        section = SPEC.split('## 1 Vehicle')[1].split('## 2')[0]
        rows = [l for l in section.splitlines() if l.startswith('| ') and 'Parameter' not in l]
        for row in rows:
            cells = [c.strip() for c in row.strip('|').split('|')]
            self.assertEqual(len(cells), 5, row)
            self.assertTrue(all(cells), f'leere Zelle (Quelle/Einheit/Unsicherheit): {row}')
        self.assertIn('engineering assumptions', SPEC)  # ehrliche Kennzeichnung der Unsicherheit

    def test_Q01_2_tyre_parameters_documented_with_origin(self):
        """Q01.2 (kritisch): Abmessung, Nenndruck, Reibwert, Referenzlast und Herkunft der Reifenparameter."""
        rt = runtime()
        self.assertAlmostEqual(table_value('Effective rolling radius'), rt.wheel.radius_m, 9)
        self.assertAlmostEqual(table_value('Nominal friction coefficient'), round(rt.tyre.mu, 4), 9)
        self.assertAlmostEqual(table_value('Reference load'), rt.tyre.nominal_fz_n, 9)
        self.assertAlmostEqual(table_value('Load sensitivity exponent'), round(rt.tyre.load_exponent, 4), 9)
        for word in ('Inflation pressure', 'Tyre size', 'brush'):
            self.assertIn(word, SPEC)
        tyre_yaml = (CONFIGS / 'tyres/pacejka_fitted_synthetic.yaml').read_text()
        for key in ('provenance:', 'dataset_id:', 'fit_method:', 'validity_domain:'):
            self.assertIn(key, tyre_yaml)

    def test_Q01_3_operating_ranges_defined_and_enforced(self):
        """Q01.3: Bereiche für v, alpha, kappa, Fz, mu; Temperatur explizit nicht modelliert."""
        from torquevector.cli.composition import validity_envelope

        envelope = validity_envelope(runtime())
        section = SPEC.split('## 3 Operating envelope')[1].split('## 4')[0]
        for word in (
            'Vehicle speed',
            'Longitudinal slip κ',
            'Slip angle α',
            'Wheel load Fz',
            'friction coefficient',
            'Temperature',
        ):
            self.assertIn(word, section)
        self.assertIn(f"|κ| ≤ {envelope['abs_slip_ratio_max']:g}", section.replace('\\|', '|'))
        self.assertIn(f"≥ {envelope['wheel_contact_speed_ms']['min']:.1f}", section)
        self.assertEqual(envelope['temperature'], 'not_modelled')

    def test_Q01_4_inputs_validated_for_units_signs_ranges_nan(self):
        """Q01.4: Einheiten, Vorzeichen, Plausibilitätsbereiche und NaN/Inf werden abgewiesen."""
        from torquevector.domain.controls import TorqueControls
        from torquevector.domain.tyre import TyreParameters
        from torquevector.domain.vehicle import VehicleParameters
        from torquevector.domain.wheels import WheelParameters
        from torquevector.infrastructure.config.errors import ConfigurationError
        from torquevector.infrastructure.config.resolver import resolve

        for bad in (dict(mass_kg=nan), dict(mass_kg=-1.0), dict(iz_kgm2=inf), dict(cg_to_front_m=3.0)):
            with self.assertRaises(ValueError):
                VehicleParameters(**bad)
        with self.assertRaises(ValueError):
            TyreParameters(mu=0.0)
        with self.assertRaises(ValueError):
            WheelParameters(radius_m=nan)
        with self.assertRaises(ValueError):
            TorqueControls(brake_nm=(0.0, -1.0, 0.0, 0.0))
        for path, value in (
            ('vehicle.mass_kg', -1600.0),
            ('vehicle.mass_kg', 'schwer'),
            ('vehicle.mass_kg', float('nan')),
            ('tyre.friction.mu', 9.0),
            ('wheel.effective_radius_m', 0.0),
        ):
            with self.subTest(path=path, value=value), self.assertRaises(ConfigurationError):
                resolve(CONFIGS / 'default.yaml', overrides=[(path, value)])

    def test_Q01_5_run_records_parameter_files_version_revision_validity(self):
        """Q01.5: Jeder Lauf protokolliert Parameterdateien, Version, Git-Revision und Gültigkeitsbereich."""
        from torquevector.cli.composition import run_configured
        from torquevector.infrastructure.config.resolver import resolve

        with tempfile.TemporaryDirectory() as root:
            resolved = resolve(CONFIGS / 'default.yaml', overrides=[('scenario.simulation_duration_s', 0.5)])
            run_configured(resolved, results_dir=root, controllers=('off',), echo=lambda *_: None, report=False)
            manifest = json.loads(next(p for p in __import__('pathlib').Path(root).rglob('manifest.json')).read_text())
        config = manifest['configuration']
        roles = {f['role'] for f in config['parameter_files']}
        self.assertTrue({'application', 'vehicle', 'tyre'} <= roles, roles)
        self.assertTrue(all(re.fullmatch('[0-9a-f]{64}', f['sha256']) for f in config['parameter_files']))
        self.assertTrue(manifest['torquevector_version'])
        self.assertTrue(manifest['git_revision'])  # Hash oder expliziter UNAVAILABLE_-Grund
        self.assertIn('abs_slip_ratio_max', config['validity_envelope'])
        self.assertEqual(manifest['source_kind'], 'SYNTHETIC_SIMULATION_ONLY')


if __name__ == '__main__':
    unittest.main()
