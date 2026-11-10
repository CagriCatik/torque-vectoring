"""Jedes Feld der Laufzeit-Dataclasses stammt aus YAML, und jede gebundene YAML-Größe wirkt.

Verhindert versteckte Modellparameter: Ein neues Dataclass-Feld (etwa in LTVConfig)
ohne Schema-Bindung lässt ``test_every_runtime_field_is_bound`` scheitern; ein YAML-Wert,
den die Composition Root ignoriert, lässt ``test_every_binding_changes_runtime`` scheitern.
"""

import copy, dataclasses, unittest
from pathlib import Path
from torquevector.cli.composition import RuntimeConfiguration, build_runtime
from torquevector.infrastructure.config.resolver import all_leaf_paths, resolve, with_overrides
from torquevector.infrastructure.config.schema import Field

CONFIGS = Path(__file__).parents[2] / 'configs'
# Nicht aus YAML, sondern abgeleitet bzw. Aliase desselben Objekts (siehe composition.build_runtime).
DERIVED = {
    'fingerprint',
    'tyre_model',
    'plant.car',
    'plant.tyre',
    'plant.wheel',
    'plant.tyre_model',
    'scenario.road.base_model',
}
ALIASES = {'plant.car': 'vehicle', 'plant.tyre': 'tyre', 'plant.wheel': 'wheel', 'plant.tyre_model': 'tyre_model'}
# Enum-Umschaltungen mit abhängigen Pflichtfeldern: über Datei-/Szenariowechsel in
# tests/integration/test_configuration_runs.py geprüft.
SPECIAL = {
    'scenario.manoeuvre.kind',
    'scenario.road.model',
    'tyre.combined_slip.model',
    'road.regions',
}  # Regionsliste: eigene Tests in tests/unit/road/
ACCEL = CONFIGS / 'scenarios/straight_split_mu_acceleration.yaml'
COUPLED = ('controllers.ltv_mpc.sample_s', 'controllers.nmpc.sample_s', 'simulation.sampling.control_period_s')


def runtime_leaves(obj, prefix=''):
    if not dataclasses.is_dataclass(obj) or callable(obj) and not hasattr(obj, '__dataclass_fields__'):
        yield prefix
        return
    for f in dataclasses.fields(obj):
        path = f'{prefix}.{f.name}' if prefix else f.name
        value = getattr(obj, f.name)
        if path in DERIVED:
            yield path
            continue
        if dataclasses.is_dataclass(value):
            yield from runtime_leaves(value, path)
        else:
            yield path


def attr(obj, path):
    for part in path.split('.'):
        obj = getattr(obj, part)
    return obj


def bindings():
    return {path: spec for path, spec in all_leaf_paths().items() if isinstance(spec, Field) and spec.binds}


class BindingCompleteness(unittest.TestCase):
    def configurations(self):
        return {
            'default': resolve(CONFIGS / 'default.yaml'),
            'mu_step': resolve(
                CONFIGS / 'default.yaml', replacements={'scenario': str(CONFIGS / 'scenarios/mu_step.yaml')}
            ),
            'brake': resolve(
                CONFIGS / 'default.yaml', replacements={'scenario': str(CONFIGS / 'scenarios/brake_in_turn.yaml')}
            ),
            'accel': resolve(CONFIGS / 'default.yaml', replacements={'scenario': str(ACCEL)}),
            'kamm': resolve(
                CONFIGS / 'default.yaml', replacements={'tyre': str(CONFIGS / 'tyres/pacejka_baseline.yaml')}
            ),
        }

    def test_every_runtime_field_is_bound(self):
        bound = {target for spec in bindings().values() for target in spec.binds}
        runtimes = [build_runtime(r) for r in self.configurations().values()]
        leaves = set()
        for runtime in runtimes:
            leaves |= set(runtime_leaves(runtime))
        missing = sorted(
            p
            for p in leaves - bound - DERIVED
            if not any(p.startswith(b + '.') or b.startswith(p + '.') for b in bound)
        )
        self.assertEqual(missing, [], 'Laufzeitfelder ohne YAML-Bindung (versteckte Defaults)')
        self.assertIn('scenario.road.after_mu', leaves)
        self.assertIn('scenario.brake_phase.brake_nm', leaves)
        for alias, target in ALIASES.items():
            self.assertIs(attr(runtimes[0], alias), attr(runtimes[0], target))

    def test_every_binding_target_exists(self):
        runtime = build_runtime(
            resolve(CONFIGS / 'default.yaml', replacements={'scenario': str(CONFIGS / 'scenarios/brake_in_turn.yaml')})
        )
        friction = build_runtime(
            resolve(CONFIGS / 'default.yaml', replacements={'scenario': str(CONFIGS / 'scenarios/mu_step.yaml')})
        )
        accel = build_runtime(resolve(CONFIGS / 'default.yaml', replacements={'scenario': str(ACCEL)}))
        for path, spec in bindings().items():
            for target in spec.binds:
                with self.subTest(yaml=path, target=target):
                    source = friction if target.startswith('scenario.road.') else runtime
                    if target.startswith(('scenario.drive_phase', 'scenario.surface.surface.regions')):
                        source = accel
                    attr(source, target)

    def test_every_binding_changes_runtime(self):
        configs = {
            'default': resolve(CONFIGS / 'default.yaml'),
            'mu_step': resolve(
                CONFIGS / 'default.yaml', replacements={'scenario': str(CONFIGS / 'scenarios/mu_step.yaml')}
            ),
            'brake': resolve(
                CONFIGS / 'default.yaml', replacements={'scenario': str(CONFIGS / 'scenarios/brake_in_turn.yaml')}
            ),
            'accel': resolve(CONFIGS / 'default.yaml', replacements={'scenario': str(ACCEL)}),
        }
        checked = 0
        for path, spec in bindings().items():
            if path in SPECIAL:
                continue
            base = next((r for r in configs.values() if self._get(r, path) is not None), None)
            if base is None:
                base = self._enable(configs['default'], path)
            value = self._get(base, path)
            new = self._perturb(spec, value, path)
            overrides = [(path, new)]
            if path in COUPLED:
                overrides = [(p, 0.05) for p in COUPLED]
            if path.endswith('mu_source'):
                overrides = [(path, 'explicit'), (path.rsplit('.', 1)[0] + '.mu', 0.9)]
            if path == 'road.transition.mode':
                overrides = [(path, 'smooth'), ('road.transition.smoothing_distance_m', 0.5)]
            if path == 'road.transition.smoothing_distance_m':
                overrides = [(path, 0.5), ('road.transition.mode', 'smooth')]
            with self.subTest(yaml=path):
                changed = with_overrides(base, overrides)
                before, after = build_runtime(base), build_runtime(changed)
                for target in spec.binds:
                    self.assertNotEqual(self._norm(attr(before, target)), self._norm(attr(after, target)), target)
                checked += 1
        self.assertGreater(checked, 150)

    def _get(self, resolved, path):
        node = resolved.tree
        for key in path.split('.'):
            if node is None or key not in node:
                return None
            node = node[key]
        return node

    def _enable(self, base, path):
        if path in (
            'controllers.p.yaw_reference.mu',
            'controllers.lqr.yaw_reference.mu',
            'allocation.friction_limit.mu',
        ):
            prefix = path.rsplit('.', 1)[0]
            return with_overrides(base, [(prefix + '.mu_source', 'explicit'), (prefix + '.mu', 0.9)])
        if path.startswith('controllers.nmpc.solver.max_cpu_time_s'):
            return with_overrides(base, [(path, 5.0)])
        if path == 'scenario.simulation_duration_s':
            return with_overrides(base, [(path, 3.0)])
        if path.startswith('visualization.') and path.endswith('colors'):
            return with_overrides(base, [(path, ['C0', 'C1'])])
        if path == 'run.matrix_set':
            return with_overrides(base, [(path, 'nominal')])
        raise AssertionError(f'Kein Basiswert für {path}')

    def _perturb(self, spec, value, path):
        if spec.type == 'bool':
            return not value
        if spec.type == 'enum':
            return next(c for c in spec.choices if c != value)
        if spec.type == 'int':
            return value + 1 if spec.maximum is None or value + 1 <= spec.maximum else value - 1
        if spec.type == 'float':
            candidate = value * 1.01 if value else 0.01
            if spec.maximum is not None and candidate > spec.maximum:
                candidate = value * 0.99 if value else -0.01
            return candidate
        if spec.type == 'wheel_map':
            out = dict(value)
            key = 'RL' if value['RL'] > 0 else 'FL'
            out[key] = value[key] * 0.5 if value[key] else 1.0
            if path.endswith('drive_distribution'):
                out = {'FL': 0.25, 'FR': 0.25, 'RL': 0.25, 'RR': 0.25}
            return out
        if spec.type == 'float_list':
            return [v * 1.1 for v in value]
        if spec.type == 'str_list':
            if spec.choices:
                rest = [c for c in spec.choices if c not in value]
                return list(value) + rest[:1] if rest else list(value)[:-1]
            return list(value) + ['C9']
        if path == 'run.matrix_set':
            return 'regression' if value != 'regression' else 'nominal'
        if spec.type in ('str', 'path'):
            return value + '_x'
        raise AssertionError(f'Unperturbierbarer Typ {spec.type} ({path})')

    def _norm(self, value):
        return tuple(value) if isinstance(value, (list, tuple)) else value


if __name__ == '__main__':
    unittest.main()
