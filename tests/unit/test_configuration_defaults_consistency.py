"""Keine widersprüchlichen Standardwerte: Referenz-YAML == historische Code-Defaults/Literale.

Die Defaults der Fachklassen sind teilweise AST-eingefroren und bleiben als
Kompatibilitäts-API bestehen. Dieser Test stellt sicher, dass beide Quellen identisch sind.
"""

import dataclasses, json, unittest
from dataclasses import asdict, replace
from pathlib import Path
from torquevector.cli.composition import build_runtime
from torquevector.control.allocation import AllocationOptions
from torquevector.control.controllers.lqr import LQRConfig
from torquevector.control.controllers.ltv_mpc import LTVConfig
from torquevector.control.controllers.nmpc import NMPCConfig, NMPCSolverSettings
from torquevector.domain.scenarios import presets
from torquevector.domain.tyre import TyreParameters
from torquevector.domain.vehicle import VehicleParameters
from torquevector.domain.wheels import WheelParameters
from torquevector.evaluation.envelope import LEGACY_THRESHOLDS
from torquevector.infrastructure.config.resolver import resolve, resolve_study
from torquevector.infrastructure.configuration import load_model
from torquevector.optimization.configured_tuning import ObjectiveSpec
from torquevector.optimization.pso import PSOSettings
from torquevector.physics.combined_slip import CombinedSlipShape
from torquevector.simulation.runner import LEGACY_DRIVER, legacy_setups
from torquevector.visualization.animation import AnimationConfig
from torquevector.visualization.comparison import ChartStyle

CONFIGS = Path(__file__).parents[2] / 'configs'
# Eingefrorene Werte der archivierten Offline-Tuningmodule V10/V11 (archive/legacy_src/.../optimization).
LEGACY_BOUNDS = ((0.018, 0.14), (0.025, 0.16), (0.02, 2.0), (0.0, 0.9))
LEGACY_TRAIN = (('circle_track', 15.0, 6.0), ('double_lane_change', 17.0, 6.0))
LEGACY_HOLDOUT = (('circle_track', 19.0, 6.0), ('double_lane_change', 21.0, 6.0))
LEGACY_NMPC_FITNESS_SAMPLE = 0.5794024224965707
V11_PINS = [('simulation.driver.yaw_reference.source', 'path_curvature'), ('allocation.friction_limit.mode', 'off')]


class ReferenceYamlEqualsLegacyDefaults(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Seit V12.1 weichen zwei Defaults bewusst vom eingefrorenen V11-Stand ab (Q11); gepinnt wie im
        # Regressionsprofil, damit alle übrigen Werte weiterhin gegen die Code-Literale geprüft werden.
        cls.runtime = build_runtime(resolve(CONFIGS / 'default.yaml', overrides=V11_PINS))

    def test_intended_v12_1_differences(self):
        current = build_runtime(resolve(CONFIGS / 'default.yaml'))
        self.assertEqual(current.driver.reference_source, 'driver_steering')
        self.assertEqual(current.controllers.allocation_friction_mode, 'friction_aware')
        self.assertEqual(replace(current.driver, reference_source='path_curvature'), LEGACY_DRIVER)
        pinned = resolve(CONFIGS / 'profiles/regression.yaml')
        self.assertEqual(pinned.get('simulation.driver.yaw_reference.source'), 'path_curvature')
        self.assertEqual(pinned.get('allocation.friction_limit.mode'), 'off')

    def test_vehicle_wheel_and_fitted_tyre(self):
        tyre, model = load_model()
        self.assertEqual(self.runtime.vehicle, VehicleParameters())
        self.assertEqual(self.runtime.wheel, WheelParameters())
        self.assertEqual(self.runtime.tyre, tyre)
        self.assertEqual(self.runtime.tyre_model, model)

    def test_baseline_and_combined_slip_tyres_equal_code_defaults(self):
        base = build_runtime(
            resolve(CONFIGS / 'default.yaml', replacements={'tyre': str(CONFIGS / 'tyres/pacejka_baseline.yaml')})
        )
        self.assertEqual(base.tyre, TyreParameters())
        self.assertIsNone(base.combined_slip)
        cs = build_runtime(
            resolve(CONFIGS / 'default.yaml', replacements={'tyre': str(CONFIGS / 'tyres/combined_slip.yaml')})
        )
        self.assertEqual(cs.tyre, TyreParameters())
        self.assertEqual(cs.combined_slip, CombinedSlipShape())

    def test_canonical_runner_literals(self):
        tyre, model = load_model()
        plant, controllers, scenario = legacy_setups(
            presets()['circle_track'],
            8.0,
            nmpc_config=NMPCConfig(),
            ltv_config=LTVConfig(),
            car=VehicleParameters(),
            tyre=tyre,
            model=model,
        )
        self.assertEqual(self.runtime.plant, plant)
        self.assertEqual(self.runtime.controllers, controllers)
        self.assertEqual(self.runtime.driver, LEGACY_DRIVER)
        # Fahrbahn: YAML dry_asphalt und Legacy-Neutralfahrbahn sind verschieden benannt, aber beide neutral.
        for field in dataclasses.fields(scenario):
            if field.name != 'surface':
                self.assertEqual(getattr(self.runtime.scenario, field.name), getattr(scenario, field.name), field.name)
        self.assertTrue(self.runtime.scenario.surface.is_identity and scenario.surface.is_identity)
        self.assertEqual(self.runtime.controllers.lqr, LQRConfig())
        self.assertEqual(self.runtime.controllers.nmpc_solver, NMPCSolverSettings())
        self.assertEqual(self.runtime.controllers.allocation, AllocationOptions())

    def test_validation_and_visualization(self):
        self.assertEqual(self.runtime.validation.envelope, LEGACY_THRESHOLDS)
        self.assertEqual(self.runtime.visualization.chart, ChartStyle())
        self.assertEqual(self.runtime.visualization.animation, AnimationConfig(fps=8, max_frames=100, dpi=80))

    def test_scenarios_equal_presets_and_legacy_json(self):
        for name, spec in presets().items():
            with self.subTest(scenario=name):
                runtime = build_runtime(
                    resolve(
                        CONFIGS / 'default.yaml', replacements={'scenario': str(CONFIGS / f'scenarios/{name}.yaml')}
                    )
                )
                self.assertEqual(runtime.scenario.spec, spec)
                legacy = json.loads((CONFIGS / f'scenarios/{name}.json').read_text())
                self.assertEqual(asdict(spec), legacy)

    def test_controller_json_and_yaml_agree(self):
        self.assertEqual(
            asdict(self.runtime.controllers.ltv), json.loads((CONFIGS / 'controllers/ltv.json').read_text())
        )
        self.assertEqual(
            asdict(self.runtime.controllers.nmpc), json.loads((CONFIGS / 'controllers/nmpc.json').read_text())
        )

    def test_pso_and_tuning_match_offline_modules(self):
        for path in ('optimization/nmpc_tuning.yaml', 'optimization/ltv_mpc_tuning.yaml'):
            with self.subTest(study=path):
                d = resolve_study(CONFIGS / path)
                self.assertEqual(tuple((x['lower'], x['upper']) for x in d.study['search_space']), LEGACY_BOUNDS)
                legacy = json.loads((CONFIGS / 'optimizations/nmpc_pso.json').read_text())
                self.assertEqual({k: d.pso[k] for k in legacy}, legacy)
                defaults = asdict(PSOSettings())
                for key in ('cognitive', 'social', 'inertia_start', 'inertia_end'):
                    self.assertEqual(d.pso[key], defaults[key])
        sets = json.loads(
            json.dumps(resolve(CONFIGS / 'default.yaml').plain()['validation']['scenario_matrix']['sets'])
        )

        def cases(name):
            return tuple((Path(c['scenario']).stem, c['speed_ms'], c['duration_s']) for c in sets[name])

        self.assertEqual(cases('train'), LEGACY_TRAIN)
        self.assertEqual(cases('holdout'), LEGACY_HOLDOUT)

    def test_objective_equals_nmpc_fitness(self):
        o = resolve_study(CONFIGS / 'optimization/nmpc_tuning.yaml').study['objective']
        spec = ObjectiveSpec(
            **o['weights'],
            beta_norm_deg=o['normalization']['max_beta_deg'],
            soft_state_norm=o['normalization']['soft_state_excess'],
            allocation_norm_nm=o['normalization']['allocation_residual_nm'],
            invalid_cost=o['invalid_candidate_cost'],
        )
        m = {
            'yaw_rmse_rads': 0.03,
            'cross_rmse_m': 0.25,
            'max_abs_beta_deg': 1.7,
            'predictive_state_excess': 0.004,
            'maximum_allocation_residual_nm': 210.0,
        }
        b = {'yaw_rmse_rads': 0.04, 'cross_rmse_m': 0.27}
        self.assertEqual(spec.fitness(m, b), LEGACY_NMPC_FITNESS_SAMPLE)


if __name__ == '__main__':
    unittest.main()
