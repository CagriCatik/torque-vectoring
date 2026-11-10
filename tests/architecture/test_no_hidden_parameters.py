"""Architekturregeln gegen neu eingeführte versteckte Modellparameter und YAML in Fachschichten."""

import ast, importlib.util, json, tempfile, unittest
from pathlib import Path
import torquevector
from torquevector.evaluation.architecture import scan
from torquevector.infrastructure.config.resolver import (
    all_leaf_paths,
    load_component,
    resolve,
    resolve_component,
    resolve_study,
)
from torquevector.infrastructure.config.schema import Field, iter_fields
from torquevector.infrastructure.config.schemas import COMPONENTS, CONTROLLERS, PSO, TUNING, tree_schema

ROOT = Path(__file__).parents[2]
PACKAGE = Path(torquevector.__file__).parent
_spec = importlib.util.spec_from_file_location('audit_hardcoded', ROOT / 'scripts/configuration/audit_hardcoded.py')
_audit = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_audit)
core_defaults, audit_scan = _audit.core_defaults, _audit.scan


class NoHiddenDefaults(unittest.TestCase):
    def test_no_new_numeric_defaults_in_core_layers(self):
        """Neue Default-Zahlen in domain/physics/prediction/control/simulation erfordern Review + YAML-Bindung."""
        baseline = json.loads((ROOT / 'tests/fixtures/core_numeric_defaults.json').read_text())
        allowed = {(r['module'], r['symbol'], r['name'], repr(r['value'])) for r in baseline}
        current = {(r['module'], r['symbol'], r['name'], repr(r['value'])) for r in core_defaults(audit_scan(PACKAGE))}
        self.assertEqual(sorted(current - allowed), [], 'Neue fest verdrahtete Defaults in Kernschichten')

    def test_configured_setups_have_no_defaults(self):
        for relative in (
            'simulation/configured.py',
            'cli/composition.py',
            'optimization/configured_tuning.py',
            'evaluation/envelope.py',
        ):
            tree = ast.parse((PACKAGE / relative).read_text())
            for node in ast.walk(tree):
                if isinstance(node, ast.ClassDef) and any(
                    ast.unparse(d).endswith('dataclass(frozen=True)') for d in node.decorator_list
                ):
                    defaults = [
                        ast.unparse(i.target) for i in node.body if isinstance(i, ast.AnnAssign) and i.value is not None
                    ]
                    with self.subTest(cls=node.name):
                        self.assertEqual(defaults, [])
        legacy = ast.parse((PACKAGE / 'evaluation/envelope.py').read_text())
        self.assertTrue(
            any(isinstance(n, ast.Assign) and ast.unparse(n.targets[0]) == 'LEGACY_THRESHOLDS' for n in legacy.body)
        )

    def test_yaml_only_in_infrastructure_and_cli(self):
        self.assertEqual(scan(PACKAGE)['violations'], [])
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            for layer in ('control', 'infrastructure'):
                (root / layer).mkdir()
            (root / 'control/a.py').write_text('import yaml\n')
            (root / 'infrastructure/b.py').write_text('import yaml\n')
            violations = scan(root)['violations']
        self.assertTrue(any('control.a' in v and 'YAML' in v for v in violations))
        self.assertFalse(any('infrastructure.b' in v for v in violations))


class SchemaHygiene(unittest.TestCase):
    def all_sections(self):
        yield tree_schema()
        yield PSO
        yield TUNING

    def test_numeric_fields_have_units_and_no_structural_defaults(self):
        for section in self.all_sections():
            for path, spec in iter_fields(section):
                if spec.type in ('float', 'int', 'wheel_map', 'float_list'):
                    with self.subTest(path=path):
                        self.assertTrue(spec.unit)
                        self.assertFalse(spec.has_structural_default)

    def test_unit_suffix_convention_for_si_fields(self):
        suffix = {
            'kg': '_kg',
            'm': '_m',
            's': '_s',
            'm/s': '_ms',
            'rad': '_rad',
            'rad/s': '_rads',
            'deg': '_deg',
            'N': '_n',
            'N*m': '_nm',
            'deg/s': '_deg_s',
            'Hz': '_hz',
            'in': '_in',
            'm^2': '_m2',
            'kg*m^2': '_kgm2',
            'kg/m^3': '_kgm3',
            'm/s^2': '_ms2',
            'N*m/s': '_nm_s',
            '%': '_pct',
        }
        for path, spec in all_leaf_paths().items():
            if spec.unit in suffix and not spec.is_fixed:
                with self.subTest(path=path):
                    self.assertTrue(path.split('.')[-1].endswith(suffix[spec.unit]), f'{path} [{spec.unit}]')


class EveryConfigFileIsUsable(unittest.TestCase):
    def test_every_yaml_validates_and_components_plug_into_default(self):
        for path in sorted((ROOT / 'configs').rglob('*.yaml')):
            relative = path.relative_to(ROOT / 'configs')
            with self.subTest(file=str(relative)):
                text = path.read_text()
                if relative.parts[0] == 'profiles' or relative.name == 'default.yaml':
                    resolve(path)
                    continue
                if relative.parts[0] == 'optimization':
                    resolve_study(path)
                    continue
                component, _ = resolve_component(path)
                if component.template:
                    continue
                slot = '.'.join(component.slot)
                resolve(ROOT / 'configs/default.yaml', replacements={slot: str(path)})


if __name__ == '__main__':
    unittest.main()
