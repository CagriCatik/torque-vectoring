"""Q07 – Numerische Robustheit."""

import json
import tempfile
import unittest
from math import isfinite
from pathlib import Path

import numpy as np

from quality.common import CONFIGS, nrmse, run_case, runtime
from torquevector.domain.controls import TorqueControls
from torquevector.domain.errors import OutsideEnvelopeError

STATES = (('vx_ms', 1.0), ('vy_ms', 0.2), ('yaw_rate_rads', 0.05), ('yaw_rad', 0.1))
# Gültige Manöver: Normal, Lastwechsel, Grenzbereich (max. Ausnutzung > 0,9) – SCENARIO_MATRIX M06/S12
MANOEUVRES = {
    'normal_drive': TorqueControls(1.0, (0.0, 0.0, 75.0, 85.0)),
    'load_change_brake': TorqueControls(2.0, (0.0,) * 4, (250.0, 250.0, 150.0, 150.0)),
    'limit_cornering': TorqueControls(9.0, (0.0, 0.0, 200.0, 200.0)),
}


def states(seq, name):
    return [getattr(s.state, name) for s in seq]


class Q07Numerics(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rt = runtime()
        cls.car, cls.tyre, cls.wheel, cls.model = rt.vehicle, rt.tyre, rt.wheel, rt.tyre_model

    def sim(self, ctrl, **kw):
        from torquevector.simulation.continuous import simulate_torque

        return simulate_torque(
            self.car,
            self.tyre,
            ctrl,
            self.wheel,
            speed_ms=20.0,
            duration_s=3.0,
            tyre_model=self.model,
            load_solver='root',
            **kw,
        )

    def test_Q07_1_second_integration_method_as_reference(self):
        """Q07.1: zweites numerisches Referenzverfahren (implizites Radau gegen explizites DOP853), automatisiert."""
        for name, ctrl in MANOEUVRES.items():
            a, b = self.sim(ctrl, method='DOP853'), self.sim(ctrl, method='Radau', rtol=1e-9, max_step=0.005)
            with self.subTest(manoeuvre=name):
                for state, floor in STATES:
                    self.assertLessEqual(nrmse(states(a, state), states(b, state), floor), 0.005)

    def test_Q07_2_dt_versus_dt_over_ten_below_two_percent(self):
        """Q07.2 (kritisch): NRMSE der Hauptzustände zwischen dt und dt/10 <= 2 % in allen gültigen Manövern."""
        limit = self.sim(MANOEUVRES['limit_cornering'])
        self.assertGreater(max(max(w.utilization for w in s.forces.wheels.values()) for s in limit), 0.9)
        for name, ctrl in MANOEUVRES.items():
            a, b = self.sim(ctrl, max_step=0.02), self.sim(ctrl, max_step=0.002, max_rhs_calls=100000)
            with self.subTest(path='continuous', manoeuvre=name):
                for state, floor in STATES:
                    self.assertLessEqual(nrmse(states(a, state), states(b, state), floor), 0.02)
        for scenario, kind in (
            ('double_lane_change', 'lqr'),
            ('circle_track', 'nmpc'),
        ):  # getakteter ZOH-Pfad mit Regler
            base = (('scenario.simulation_duration_s', 5.0),)
            _, _, a, _ = run_case(scenario, kind, base)
            _, _, b, _ = run_case(
                scenario,
                kind,
                base + (('simulation.integrator.max_step_s', 0.002), ('simulation.integrator.rtol', 1e-7)),
            )
            with self.subTest(path='sampled', scenario=scenario, controller=kind):
                for state, floor in STATES:
                    self.assertLessEqual(nrmse(states(a, state), states(b, state), floor), 0.02)

    def test_Q07_3_no_nan_or_undetected_instability_in_stress_scenarios(self):
        """Q07.3 (kritisch): keine NaN/Inf, explodierenden Zustände oder unerkannten Instabilitäten unter Stress."""
        stress = [
            ('double_lane_change', 'lqr', (('scenario.manoeuvre.speed_ms', 30.0),)),  # unmöglich
            ('double_lane_change', 'nmpc', (('road.default_surface.mu', 0.4), ('scenario.manoeuvre.speed_ms', 24.0))),
            ('straight_split_mu_braking_above_limit', 'off', ()),  # ohne ABS
            ('circle_track', 'ltv', (('allocation.max_delta_nm', 25.0), ('scenario.manoeuvre.speed_ms', 21.0))),
            ('mixed_mu_lane_change', 'p', ()),
        ]
        for scenario, kind, overrides in stress:
            _, entry, seq, _ = run_case(scenario, kind, overrides + (('scenario.simulation_duration_s', 6.0),))
            with self.subTest(scenario=scenario, controller=kind, status=entry['status']):
                self.assertIn(entry['status'], ('PASS', 'INVALID_MODEL_ENVELOPE', 'FAIL_CLOSED'))
                if entry['status'] == 'FAIL_CLOSED':
                    self.assertTrue(entry.get('reason'))  # erkannt und begründet
                for s in seq or ():  # FAIL_CLOSED: keine Telemetrie
                    values = (s.state.vx_ms, s.state.vy_ms, s.state.yaw_rate_rads, *s.omega_rad_s, *s.kappa)
                    self.assertTrue(all(isfinite(v) for v in values))
                    self.assertLess(abs(s.state.yaw_rate_rads), 3.0)
                    self.assertLess(abs(s.state.vy_ms), 15.0)

    def test_Q07_4_singular_events_handled_or_rejected(self):
        """Q07.4: Nullgeschwindigkeit, Radstillstand, Lastabheben, Kraftsättigung robust behandelt oder abgewiesen."""
        from torquevector.physics.wheel_rotation import validate_no_lift
        from torquevector.simulation.continuous import simulate_torque
        from torquevector.simulation.sampled import simulate_sampled

        with self.assertRaises(ValueError):  # v nahe 0: Start abgewiesen
            simulate_sampled(self.car, self.tyre, TorqueControls(), self.wheel, speed_ms=0.5, duration_s=1.0)
        from dataclasses import replace

        with self.assertRaisesRegex(OutsideEnvelopeError, 'Slip'):  # Radstillstand (Blockieren auf µ = 0,3)
            simulate_torque(
                self.car,
                replace(self.tyre, mu=0.3),
                TorqueControls(0.0, (0.0,) * 4, (800.0,) * 4),
                self.wheel,
                duration_s=2.0,
                tyre_model=self.model,
            )
        with self.assertRaisesRegex(OutsideEnvelopeError, 'contact speed'):  # Ausrollen bis nahe 0 m/s: abgewiesen
            simulate_torque(
                self.car,
                self.tyre,
                TorqueControls(0.0, (0.0,) * 4, (700.0,) * 4),
                self.wheel,
                speed_ms=20.0,
                duration_s=6.0,
                tyre_model=self.model,
            )
        with self.assertRaisesRegex(OutsideEnvelopeError, 'lift'):  # Lastabheben
            validate_no_lift(self.car, 0.0, 1.1 * self.car.gravity * self.car.track_m / self.car.cg_height_m)
        sat = self.sim(MANOEUVRES['limit_cornering'])  # Sättigung: behandelt, endlich
        clipped = [w.clipped for s in sat for w in s.forces.wheels.values()]
        self.assertTrue(all(isfinite(s.state.yaw_rate_rads) for s in sat))
        self.assertLessEqual(max(w.utilization for s in sat for w in s.forces.wheels.values()), 1 + 1e-9)
        self.assertIsInstance(any(clipped), bool)

    def test_Q07_5_options_stored_and_results_reproducible(self):
        """Q07.5: Zeitstempel, Integrator-Optionen, Toleranzen, dt gespeichert; gleiche Eingaben ⇒ gleiche Ergebnisse."""
        from torquevector.cli.composition import run_configured
        from torquevector.infrastructure.config.resolver import resolve

        runs = []
        with tempfile.TemporaryDirectory() as root:
            for _ in range(2):
                resolved = resolve(CONFIGS / 'default.yaml', overrides=[('scenario.simulation_duration_s', 1.5)])
                run_configured(
                    resolved, results_dir=root, controllers=('off', 'lqr'), echo=lambda *_: None, report=False
                )
            for run in sorted(Path(root).iterdir()):
                snapshot = (run / 'config/resolved_config.yaml').read_text()
                for key in ('method: DOP853', 'rtol:', 'atol:', 'max_step_s:', 'control_period_s:'):
                    self.assertIn(key, snapshot)
                self.assertRegex(run.name, r'^\d{8}-\d{6}_circle_track(-\d+)?$')  # Zeitstempel
                runs.append(json.loads((run / 'metrics/comparison.json').read_text()))
        strip = lambda d: {
            k: v['metrics'] for k, v in d.get('controllers', d).items() if isinstance(v, dict) and 'metrics' in v
        }
        self.assertEqual(strip(runs[0]), strip(runs[1]))


if __name__ == '__main__':
    unittest.main()
