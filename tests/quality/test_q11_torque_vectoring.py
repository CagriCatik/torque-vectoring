"""Q11 – Torque Vectoring und Regelung."""

import ast
import tempfile
import unittest
from pathlib import Path

import numpy as np

from quality.common import CONFIGS, ROOT, runtime
from torquevector.domain.states import VehicleState


class Q11TorqueVectoring(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from torquevector.cli.composition import q11_assessment
        from torquevector.infrastructure.config.resolver import resolve

        cls.tmp = tempfile.TemporaryDirectory()
        cls.report = q11_assessment(
            lambda name: resolve(CONFIGS / 'default.yaml', overrides=[('run.matrix_set', name)]), Path(cls.tmp.name)
        )

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_Q11_1_reference_observation_strategy_specified_and_baseline_exists(self):
        """Q11.1 (kritisch): Sollwert, Zustandsbeobachtung, Regelstrategie spezifiziert; Baseline ohne TV existiert."""
        spec = (ROOT / 'docs/control/specification.md').read_text()
        for section in (
            '## 1 Baseline OFF',
            '## 2 Yaw reference',
            '## 3 State observation',
            '## 4 Control laws',
            '## 5 Torque allocation',
        ):
            self.assertIn(section, spec)
        rt = runtime()
        self.assertEqual(rt.driver.reference_source, 'driver_steering')
        self.assertIn('off', rt.run.controllers)
        for case in self.report['Q11.3_nominal']['controllers']['lqr']['cases']:
            self.assertIn('yaw_improvement_pct', self.report['Q11.3_nominal']['controllers']['lqr']['cases'][case])

    def test_Q11_2_allocator_respects_actuator_friction_and_load_limits(self):
        """Q11.2 (kritisch): vier zulässige Momente unter Aktuator-, Reib- und Radlastgrenzen."""
        from torquevector.control.allocation_friction import FrictionLimit, friction_torque_limits, make_allocator

        rt = runtime()
        c = rt.controllers
        self.assertEqual(c.allocation_friction_mode, 'friction_aware')
        limit = c.friction_limit(rt.tyre)
        rng = np.random.default_rng(11)
        for mu in (1.0, 0.4):
            allocate = make_allocator(rt.vehicle, rt.wheel, c.allocation, FrictionLimit(limit.reserve_fraction, mu))
            for _ in range(300):
                state = VehicleState(vx_ms=rng.uniform(8, 30), yaw_rate_rads=rng.uniform(-0.5, 0.5))
                base = tuple(rng.uniform(0, 300, 4))
                result = allocate(base, rng.uniform(-3000, 3000), state, 800.0)
                wheel_limit = friction_torque_limits(state, rt.vehicle, rt.wheel, limit.reserve_fraction, (mu,) * 4)
                for t, b, w in zip(result.torques_nm, base, wheel_limit):
                    self.assertGreaterEqual(t, -1e-9)
                    self.assertLessEqual(t, 800.0 + 1e-9)
                    self.assertLessEqual(t, max(b, w) + 1e-6)
                    self.assertLessEqual(abs(t - b), c.allocation.max_delta_nm + 1e-9)

    def test_Q11_3_tv_improves_yaw_tracking_by_twenty_percent(self):
        """Q11.3: bei machbaren Referenzmanövern >= 20 % bessere Gierratenfolge gegen Baseline (jeder Fall)."""
        nominal = self.report['Q11.3_nominal']
        self.assertTrue(nominal['passed'])
        self.assertIn('lqr', nominal['qualified_controllers'])
        self.assertIn('lqr', self.report['robustness']['qualified_controllers'])

    def test_Q11_4_degradation_defined_and_stable(self):
        """Q11.4: unmögliche Anforderung, niedriger Reibwert, Aktuatorsättigung degradieren definiert und stabil."""
        degraded = self.report['Q11.4_degraded']
        self.assertTrue(degraded['passed'], degraded)
        cases = degraded['cases']
        self.assertEqual(len(cases), 5)
        self.assertTrue(all(len(v) == 5 for v in cases.values()))  # alle fünf Regler je Fall

    def test_Q11_5_controller_path_uses_torque_only(self):
        """Q11.5: Drehmoment als Eingang; keine direkte, nicht-kausale Schlupf- oder Radkraftvorgabe."""
        control = ROOT / 'src/torquevector/control'
        for path in control.rglob('*.py'):
            tree = ast.parse(path.read_text())
            names = {n.id for n in ast.walk(tree) if isinstance(n, ast.Name)} | {
                a.name for n in ast.walk(tree) if isinstance(n, ast.ImportFrom) for a in n.names
            }
            with self.subTest(module=str(path.relative_to(control))):
                self.assertNotIn('DriverControls', names)  # Schlupfvorgabe-Typ
                self.assertFalse({'derivative', 'forces_at', 'simulate_sampled'} & names)  # kein Blick in die Strecke
                if path.name != 'slip_control.py':
                    self.assertNotIn('kappa', path.read_text())
        from torquevector.control.interfaces import Controller, MomentCommand

        self.assertEqual(Controller.command.__annotations__['return'], 'MomentCommand')
        self.assertTrue(MomentCommand)


if __name__ == '__main__':
    unittest.main()
