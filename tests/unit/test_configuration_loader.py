"""YAML-Loader, Schema-Validierung, Konsistenz- und Override-Regeln (inkl. Negativfälle)."""

import shutil, tempfile, unittest
from pathlib import Path
from torquevector.infrastructure.config import ConfigurationError
from torquevector.infrastructure.config.resolver import (
    load_snapshot,
    matrix_cases,
    resolve,
    resolve_case,
    resolve_component,
    resolve_study,
    with_overrides,
)
from torquevector.infrastructure.config.yaml_io import load_with_origins, parse_scalar

CONFIGS = Path(__file__).parents[2] / 'configs'


class ConfigCopy:
    def __enter__(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name) / 'configs'
        shutil.copytree(CONFIGS, self.root, ignore=shutil.ignore_patterns('*.json'))
        return self

    def __exit__(self, *exc):
        self.tmp.cleanup()

    def edit(self, relative, old, new):
        path = self.root / relative
        text = path.read_text()
        self_check = text.count(old)
        if self_check != 1:
            raise AssertionError(f'{relative}: {old!r} found {self_check}x')
        path.write_text(text.replace(old, new))

    def write(self, relative, text):
        path = self.root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)
        return path

    @property
    def default(self):
        return self.root / 'default.yaml'


def issues_of(callable_):
    try:
        callable_()
    except ConfigurationError as exc:
        return '\n'.join(exc.issues)
    raise AssertionError('ConfigurationError expected')


class StrictYamlTests(unittest.TestCase):
    def test_off_stays_string_scientific_notation_is_float_and_duplicates_fail(self):
        with ConfigCopy() as c:
            path = c.write('x.yaml', 'a: off\nb: 1e-5\nc: yes\nd: true\n')
            data, origins = load_with_origins(path)
            self.assertEqual(data, {'a': 'off', 'b': 1e-5, 'c': 'yes', 'd': True})
            self.assertTrue(origins[('b',)].endswith('x.yaml:2'))
            dup = c.write('dup.yaml', 'a: 1\na: 2\n')
            self.assertIn('doppelter Schlüssel', issues_of(lambda: load_with_origins(dup)))
        self.assertEqual(parse_scalar('[off, p]'), ['off', 'p'])


class SchemaValidationTests(unittest.TestCase):
    def test_reference_configuration_is_valid_and_typed(self):
        resolved = resolve(CONFIGS / 'default.yaml')
        self.assertEqual(resolved.get('vehicle.mass_kg'), 1600.0)
        self.assertIsInstance(resolved.get('controllers.ltv_mpc.horizon_steps'), int)
        self.assertEqual(resolved.get('run.controllers'), ('off', 'p', 'lqr', 'ltv', 'nmpc'))
        with self.assertRaises(TypeError):
            resolved.tree['vehicle']['mass_kg'] = 1.0

    def test_unknown_missing_type_range_and_null_are_rejected_with_origin(self):
        cases = [
            (
                'vehicles/generic_passenger_car.yaml',
                '  mass_kg: 1600.0\n',
                '  mass_kg: 1600.0\n  mass_lb: 3527.0\n',
                'unbekannter Parameter',
            ),
            (
                'vehicles/generic_passenger_car.yaml',
                '  mass_kg: 1600.0\n',
                '',
                'vehicle.mass_kg: Pflichtparameter fehlt',
            ),
            ('vehicles/generic_passenger_car.yaml', 'mass_kg: 1600.0', 'mass_kg: "1600"', 'erwartet float'),
            ('vehicles/generic_passenger_car.yaml', 'mass_kg: 1600.0', 'mass_kg: -5.0', 'unterschreitet'),
            ('controllers/ltv_mpc.yaml', 'horizon_steps: 16', 'horizon_steps: 16.5', 'erwartet int'),
            ('vehicles/generic_passenger_car.yaml', 'enabled: true', 'enabled: yes', 'erwartet bool'),
            (
                'tyres/pacejka_fitted_synthetic.yaml',
                '    mu: 1.0049454296189615',
                '    mu: null',
                'Pflichtwert ist null',
            ),
        ]
        for relative, old, new, expected in cases:
            with self.subTest(expected=expected), ConfigCopy() as c:
                c.edit(relative, old, new)
                text = issues_of(lambda: resolve(c.default))
                self.assertIn(expected, text)
                self.assertIn(relative.split('/')[-1], text)

    def test_units_are_checked_without_conversion(self):
        with ConfigCopy() as c:
            c.edit('vehicles/generic_passenger_car.yaml', 'mass_kg: 1600.0', 'mass_kg: {value: 1600.0, unit: kg}')
            self.assertEqual(resolve(c.default).get('vehicle.mass_kg'), 1600.0)
        with ConfigCopy() as c:
            c.edit('vehicles/generic_passenger_car.yaml', 'mass_kg: 1600.0', 'mass_kg: {value: 1.6, unit: t}')
            self.assertIn('keine automatische Umrechnung', issues_of(lambda: resolve(c.default)))

    def test_schema_version_and_fixed_code_values(self):
        with ConfigCopy() as c:
            c.edit('wheels/standard_wheel.yaml', 'schema_version: 1', 'schema_version: 2')
            self.assertIn('schema_version 2 nicht unterstützt', issues_of(lambda: resolve(c.default)))
        with ConfigCopy() as c:
            c.edit('controllers/ltv_mpc.yaml', 'method: SLSQP', 'method: trust-constr')
            self.assertIn('nicht zulässig', issues_of(lambda: resolve(c.default)))
        with ConfigCopy() as c:
            c.edit('powertrain/four_wheel_independent.yaml', 'dead_time_s: 0.0', 'dead_time_s: 0.02')
            self.assertIn('im Code fest', issues_of(lambda: resolve(c.default)))


class ConsistencyTests(unittest.TestCase):
    def test_cross_field_inconsistencies(self):
        cases = [
            (
                'controllers/nmpc.yaml',
                '  sample_s: 0.04',
                '  sample_s: 0.05',
                'widerspricht simulation.sampling.control_period_s',
            ),
            (
                'vehicles/generic_passenger_car.yaml',
                'cg_to_front_axle_m: 1.18',
                'cg_to_front_axle_m: 2.9',
                'kleiner als wheelbase_m',
            ),
            ('powertrain/four_wheel_independent.yaml', '    RR: 0.3\n', '    RR: 0.4\n', 'Summe muss 1'),
            (
                'vehicles/generic_passenger_car.yaml',
                'validated_against_real_vehicle: false',
                'validated_against_real_vehicle: true',
                'synthetische Quelle',
            ),
            (
                'tyres/pacejka_fitted_synthetic.yaml',
                'experimentally_validated: false',
                'experimentally_validated: true',
                'synthetische Quelle',
            ),
            (
                'tyres/pacejka_fitted_synthetic.yaml',
                'kappa_scale: 0.13953873041633566',
                'kappa_scale: null',
                'benötigt',
            ),
            ('simulation/integrator.yaml', 'max_step_s: 0.02', 'max_step_s: 0.05', 'max_step_s'),
            ('validation/quality_gates.yaml', 'Q12, Q13]', 'Q12]', 'weicht vom Manifest ab'),
            ('validation/quality_gates.yaml', 'min_points_per_gate: 8', 'min_points_per_gate: 7', 'im Code fest'),
            ('controllers/p.yaml', 'mu_source: tyre', 'mu_source: explicit', 'mu genau dann'),
        ]
        for relative, old, new, expected in cases:
            with self.subTest(expected=expected), ConfigCopy() as c:
                c.edit(relative, old, new)
                self.assertIn(expected, issues_of(lambda: resolve(c.default)))

    def test_templates_cannot_be_used_but_keep_schema_structure(self):
        for name in ('vehicles/vehicle_template.yaml', 'tyres/tyre_template.yaml'):
            component, _ = resolve_component(CONFIGS / name)
            self.assertTrue(component.template)
            with self.assertRaises(ConfigurationError):
                resolve_component(CONFIGS / name, template_ok=False)
        with ConfigCopy() as c:
            c.edit('default.yaml', 'vehicles/generic_passenger_car.yaml', 'vehicles/vehicle_template.yaml')
            self.assertIn('Vorlage', issues_of(lambda: resolve(c.default)))
        with ConfigCopy() as c:
            c.edit('vehicles/vehicle_template.yaml', '  mass_kg: null', '  mass_kg: null\n  wheel_count: null')
            self.assertIn(
                'unbekannter Parameter', issues_of(lambda: resolve_component(c.root / 'vehicles/vehicle_template.yaml'))
            )

    def test_extends_merges_and_cycles_fail(self):
        component, result = resolve_component(CONFIGS / 'tyres/combined_slip.yaml')
        self.assertEqual(result['longitudinal']['B'], 11.0)
        self.assertEqual(result['combined_slip']['model'], 'elliptic_damping_surrogate')
        self.assertEqual(len(component.files), 2)
        with ConfigCopy() as c:
            c.edit('tyres/pacejka_baseline.yaml', 'schema_version: 1', 'schema_version: 1\nextends: combined_slip.yaml')
            self.assertIn(
                'zyklische extends-Kette', issues_of(lambda: resolve_component(c.root / 'tyres/combined_slip.yaml'))
            )

    def test_component_kind_must_match_slot(self):
        with ConfigCopy() as c:
            c.edit('default.yaml', 'tyre: tyres/pacejka_fitted_synthetic.yaml', 'tyre: wheels/standard_wheel.yaml')
            self.assertIn('passt nicht in tyre', issues_of(lambda: resolve(c.default)))


class LayeringTests(unittest.TestCase):
    def test_priority_profile_then_cli_and_origins(self):
        resolved = resolve(CONFIGS / 'profiles/development.yaml', overrides=[('scenario.simulation_duration_s', 1.5)])
        self.assertEqual(resolved.get('scenario.simulation_duration_s'), 1.5)
        self.assertEqual(resolved.get('run.controllers'), ('off', 'p', 'lqr'))
        self.assertIn('CLI', resolved.origins['scenario.simulation_duration_s'])
        self.assertIn('development', resolved.origins['run.controllers'])
        self.assertIn('generic_passenger_car.yaml:', resolved.origins['vehicle.mass_kg'])

    def test_conflicting_profiles_and_cli_vs_case_are_rejected(self):
        with ConfigCopy() as c:
            other = c.write(
                'profiles/other.yaml',
                'schema_version: 1\nprofile:\n  id: other\n  overrides:\n    run.controllers: [off]\n',
            )
            self.assertIn(
                'Widersprüchliche Werte',
                issues_of(lambda: resolve(c.root / 'profiles/development.yaml', profiles=[other])),
            )
        base = resolve(CONFIGS / 'default.yaml', overrides=[('vehicle.mass_kg', 1700.0)])
        robust = [case for case in matrix_cases(base, 'robustness') if 'mass' in case['id']][0]
        self.assertIn('Widersprüchliche Werte', issues_of(lambda: resolve_case(base, robust)))
        self.assertIn(
            'mehrfach',
            issues_of(
                lambda: resolve(
                    CONFIGS / 'default.yaml', overrides=[('vehicle.mass_kg', 1.0), ('vehicle.mass_kg', 2.0)]
                )
            ),
        )
        self.assertIn(
            'kein Blattparameter',
            issues_of(lambda: resolve(CONFIGS / 'default.yaml', overrides=[('vehicle.colour', 'red')])),
        )

    def test_fingerprint_is_value_based_and_snapshot_roundtrips(self):
        a = resolve(CONFIGS / 'default.yaml')
        b = resolve(CONFIGS / 'default.yaml')
        self.assertEqual(a.fingerprint, b.fingerprint)
        c = with_overrides(a, [('vehicle.mass_kg', 1600.0000001)])
        self.assertNotEqual(a.fingerprint, c.fingerprint)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'snapshot.yaml'
            path.write_text(a.export_yaml())
            self.assertEqual(load_snapshot(path), a.plain())
            self.assertIn(a.fingerprint, path.read_text())

    def test_matrix_case_overrides(self):
        base = resolve(CONFIGS / 'default.yaml')
        cases = {case['id']: resolve_case(base, case) for case in matrix_cases(base, 'robustness')}
        self.assertEqual(cases['mass_plus_10pct_circle'].get('vehicle.mass_kg'), 1760.0)
        self.assertEqual(cases['mu_minus_10pct_dlc'].get('scenario.manoeuvre.kind'), 'double_lane_change')
        self.assertEqual(cases['mu_minus_10pct_dlc'].get('scenario.simulation_duration_s'), 8.0)


class StudyResolutionTests(unittest.TestCase):
    def test_studies_and_leakage_checks(self):
        definition = resolve_study(CONFIGS / 'optimization/pso.yaml')
        self.assertEqual(definition.study['controller'], 'nmpc')
        self.assertEqual(resolve_study(CONFIGS / 'optimization/ltv_mpc_tuning.yaml').study['controller'], 'ltv')
        with ConfigCopy() as c:
            c.edit('optimization/nmpc_tuning.yaml', '    holdout: holdout', '    holdout: train')
            self.assertIn(
                'Hold-out-Leckage', issues_of(lambda: resolve_study(c.root / 'optimization/nmpc_tuning.yaml'))
            )
        with ConfigCopy() as c:
            c.edit('optimization/nmpc_tuning.yaml', 'lower: 0.02, upper: 2.0', 'lower: 0.0, upper: 2.0')
            self.assertIn('offenen Rand', issues_of(lambda: resolve_study(c.root / 'optimization/nmpc_tuning.yaml')))
        self.assertIn(
            'tuning.',
            issues_of(lambda: resolve_study(CONFIGS / 'optimization/pso.yaml', overrides=[('pso.particles', 4)])),
        )


if __name__ == '__main__':
    unittest.main()
