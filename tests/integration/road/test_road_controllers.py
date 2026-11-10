"""Regler auf der Reibwertkarte: gleiche Eingangsdaten, kein unerlaubtes Vorwissen, Grenzen, Fehler."""

import unittest
from unittest.mock import patch
from torquevector.domain.states import VehicleState
from torquevector.domain.controls import TorqueControls
from torquevector.simulation.configured import friction_ratio_function
from torquevector.simulation.preview_adapter import make_maneuver_preview
from integration.road.helpers import run, runtime

SHORT = [('scenario.simulation_duration_s', 3.0)]


class SameScenarioForAllControllers(unittest.TestCase):
    def test_all_controllers_run_on_identical_road_and_initial_state(self):
        starts = set()
        for kind in ('off', 'p', 'lqr', 'ltv', 'nmpc'):
            with self.subTest(kind=kind):
                rt, entry, seq, contacts, ticks = run('mixed_mu_lane_change', kind, SHORT)
                self.assertIn(entry['status'], ('PASS', 'INVALID_MODEL_ENVELOPE'))
                starts.add((rt.scenario.surface, seq[0].state, tuple(sorted(contacts[0]['FL'].items()))))
                self.assertLessEqual(
                    max(abs(z) for s in seq for z in s.drive_nm), rt.plant.initial_controls.torque_limit_nm
                )
                if kind in ('ltv', 'nmpc'):
                    limit = getattr(rt.controllers, kind).moment_limit_nm
                    self.assertLessEqual(max(abs(d['requested_nm']) for d in ticks), limit + 1e-6)
        self.assertEqual(len(starts), 1)


class FrictionKnowledge(unittest.TestCase):
    def preview(self, mode, state):
        rt = runtime('mixed_mu_lane_change', [('controllers.nmpc.friction_knowledge.mode', mode)])
        ratios = friction_ratio_function(rt.scenario.surface, rt.vehicle, rt.tyre) if mode == 'map_preview' else None
        forecast = make_maneuver_preview(rt.scenario.spec, rt.vehicle, friction_ratios=ratios)
        return forecast(1.0, state, TorqueControls(), rt.controllers.nmpc)

    def test_unknown_mode_contains_no_road_information(self):
        p = self.preview('unknown', VehicleState(x_m=25.0, vx_ms=17.0))
        self.assertIsNone(p.front_friction_ratio)
        self.assertIsNone(p.rear_friction_ratio)

    def test_map_preview_only_sees_horizon_from_current_state(self):
        far = self.preview('map_preview', VehicleState(x_m=10.0, vx_ms=17.0))  # Horizont 0.4 s ≈ 7 m: nur trocken
        self.assertEqual(set(far.front_friction_ratio + far.rear_friction_ratio), {1.0})
        near = self.preview('map_preview', VehicleState(x_m=26.0, vx_ms=17.0))  # Nassabschnitt ab 30 m im Horizont
        self.assertLess(min(near.front_friction_ratio), 1.0)
        self.assertEqual(near.rear_friction_ratio[0], 1.0)  # Hinterachse erreicht ihn später

    def test_map_preview_changes_predictive_commands_but_not_the_road(self):
        base = run('mixed_mu_lane_change', 'nmpc', SHORT)
        known = run(
            'mixed_mu_lane_change', 'nmpc', SHORT + [('controllers.nmpc.friction_knowledge.mode', 'map_preview')]
        )
        self.assertEqual(base[0].scenario.surface, known[0].scenario.surface)
        self.assertNotEqual([d['requested_nm'] for d in base[4]], [d['requested_nm'] for d in known[4]])
        ltv = run(
            'mixed_mu_lane_change', 'ltv', SHORT + [('controllers.ltv_mpc.friction_knowledge.mode', 'map_preview')]
        )
        self.assertEqual(ltv[1]['status'], 'PASS')

    def test_solver_failure_is_reported_not_counted_as_success(self):
        with patch(
            'torquevector.control.controllers.friction_preview.FrictionScheduledLTV.solve',
            side_effect=RuntimeError('injected'),
        ):
            entry = run(
                'mixed_mu_lane_change', 'ltv', SHORT + [('controllers.ltv_mpc.friction_knowledge.mode', 'map_preview')]
            )[1]
        self.assertEqual(entry['status'], 'FAIL_CLOSED')
        self.assertIn('injected', entry['reason'])


if __name__ == '__main__':
    unittest.main()
