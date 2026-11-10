"""YAML-gesteuerte Läufe: Regler-, Fahrzeug-, Reifen- und Szenariowechsel ohne Codeänderung."""

import json, shutil, subprocess, sys, tempfile, unittest
from pathlib import Path
from torquevector.cli.composition import build_runtime, evaluate_case
from torquevector.cli.main import main
from torquevector.infrastructure.config.resolver import resolve

ROOT = Path(__file__).parents[2]
CONFIGS = ROOT / 'configs'


def run_metrics(kind='off', **kwargs):
    resolved = resolve(CONFIGS / 'default.yaml', **kwargs)
    entry, seq, _ = evaluate_case(build_runtime(resolved), kind)
    return entry, seq, resolved


class ComponentSwitching(unittest.TestCase):
    def test_vehicle_switch_via_yaml_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            heavy = Path(tmp) / 'heavy.yaml'
            text = (
                (CONFIGS / 'vehicles/generic_passenger_car.yaml')
                .read_text()
                .replace('mass_kg: 1600.0', 'mass_kg: 1900.0')
            )
            heavy.write_text(text.replace('id: generic_passenger_car', 'id: heavy_variant'))
            base, _, a = run_metrics(overrides=[('scenario.simulation_duration_s', 0.8)])
            other, _, b = run_metrics(
                replacements={'vehicle': str(heavy)}, overrides=[('scenario.simulation_duration_s', 0.8)]
            )
        self.assertEqual(b.get('vehicle.mass_kg'), 1900.0)
        self.assertNotEqual(a.fingerprint, b.fingerprint)
        self.assertNotEqual(base['metrics']['yaw_rmse_rads'], other['metrics']['yaw_rmse_rads'])

    def test_tyre_switch_changes_force_model(self):
        fitted, seq_fitted, _ = run_metrics(overrides=[('scenario.simulation_duration_s', 0.8)])
        for name in ('pacejka_baseline', 'combined_slip'):
            with self.subTest(tyre=name):
                other, seq, resolved = run_metrics(
                    replacements={'tyre': str(CONFIGS / f'tyres/{name}.yaml')},
                    overrides=[('scenario.simulation_duration_s', 0.8)],
                )
                self.assertEqual(other['status'], 'PASS')
                self.assertNotEqual(seq[-1].forces.wheels['FL'].fy_n, seq_fitted[-1].forces.wheels['FL'].fy_n)

    def test_controller_switch_via_yaml(self):
        with tempfile.TemporaryDirectory() as tmp:
            for controller in ('p', 'lqr', 'ltv_mpc', 'nmpc'):
                with self.subTest(controller=controller):
                    code = main(
                        [
                            '--quiet',
                            '--log-dir',
                            tmp + '/logs',
                            'simulate',
                            '--config',
                            str(CONFIGS / 'default.yaml'),
                            '--controller',
                            str(CONFIGS / f'controllers/{controller}.yaml'),
                            '--duration',
                            '.16',
                            '--results-dir',
                            tmp + '/results',
                            '--no-report',
                        ]
                    )
                    self.assertEqual(code, 0)
            manifests = [json.loads(p.read_text()) for p in Path(tmp).glob('results/*/manifest.json')]
            self.assertEqual(
                sorted(m['configuration']['controllers'][0] for m in manifests), ['lqr', 'ltv', 'nmpc', 'p']
            )
            self.assertTrue(all(m['config_fingerprint'] for m in manifests))

    def test_disturbance_scenarios_run_with_every_basic_controller(self):
        def braking(seq):
            # 4 x 45 Nm / 0.315 m = 571 N -> etwa 0.36 m/s^2 Verzögerung nach dem 0.35-s-Übergang
            self.assertEqual(seq[-1].brake_nm, (45.0, 45.0, 45.0, 45.0))
            self.assertLess(seq[-1].state.vx_ms, max(s.state.vx_ms for s in seq if s.t_s <= 1.5) - 0.1)

        for scenario, check in (
            ('brake_in_turn', braking),
            (
                'split_mu',
                lambda seq: self.assertNotEqual(seq[-1].forces.wheels['FL'].peak_n, seq[-1].forces.wheels['FR'].peak_n),
            ),
            ('mu_step', None),
        ):
            for kind in ('off', 'lqr'):
                with self.subTest(scenario=scenario, controller=kind):
                    overrides = [('scenario.simulation_duration_s', 2.4 if scenario == 'brake_in_turn' else 1.2)]
                    if scenario == 'mu_step':
                        overrides.append(('scenario.road.step_time_s', 0.4))
                    entry, seq, _ = run_metrics(
                        kind,
                        replacements={'scenario': str(CONFIGS / f'scenarios/{scenario}.yaml')},
                        overrides=overrides,
                    )
                    self.assertIn(entry['status'], ('PASS', 'INVALID_MODEL_ENVELOPE', 'FAIL_CLOSED'))
                    if seq is not None and check:
                        check(seq)


class CliConfiguredRuns(unittest.TestCase):
    def test_component_flags_without_config_snapshot_and_render_settings(self):
        with tempfile.TemporaryDirectory() as tmp:
            command = [
                sys.executable,
                '-m',
                'torquevector.cli.main',
                '--log-dir',
                tmp + '/logs',
                'simulate',
                '--vehicle',
                'configs/vehicles/generic_passenger_car.yaml',
                '--tyre',
                'configs/tyres/pacejka_fitted_synthetic.yaml',
                '--controller',
                'off',
                '--scenario',
                'configs/scenarios/double_lane_change.yaml',
                '--duration',
                '2.0',
                '--results-dir',
                tmp,
                '--set',
                'visualization.gif_export.max_frames=10',
                '--set',
                'visualization.static_plots.formats=[svg]',
                '--set',
                'visualization.static_plots.report_sections=[kpi, allocation]',
            ]
            process = subprocess.run(command, cwd=ROOT, capture_output=True, text=True, timeout=120)
            self.assertEqual(process.returncode, 0, process.stderr)
            record = json.loads(process.stdout.splitlines()[-1])
            run = Path(record['path'])
            snapshot = (run / 'config/resolved_config.yaml').read_text()
            self.assertIn(record['config_fingerprint'], snapshot)
            self.assertIn('max_frames: 10', snapshot)
            origins = json.loads((run / 'config/resolved_config.origins.json').read_text())
            self.assertIn('override (CLI)', origins['visualization.gif_export.max_frames'])
            self.assertEqual(
                main(
                    [
                        '--quiet',
                        '--log-dir',
                        tmp + '/logs',
                        'render',
                        '--run-id',
                        record['run_id'],
                        '--format',
                        'gif',
                        '--results-dir',
                        tmp,
                    ]
                ),
                0,
            )
            self.assertEqual(
                main(
                    [
                        '--quiet',
                        '--log-dir',
                        tmp + '/logs',
                        'render',
                        '--run-id',
                        record['run_id'],
                        '--format',
                        'svg',
                        '--results-dir',
                        tmp,
                    ]
                ),
                0,
            )
            from PIL import Image

            with Image.open(run / 'animations/comparison.gif') as gif:
                self.assertLessEqual(gif.n_frames, 10 + 1 + 8)
            self.assertTrue((run / 'plots/comparison.svg').is_file())
            self.assertTrue((run / 'plots/report.pdf').is_file())  # automatischer Bericht
            self.assertTrue((run / 'plots/allocation.svg').is_file())

    def test_matrix_profile_and_case_render(self):
        with tempfile.TemporaryDirectory() as tmp:
            code = main(
                [
                    '--quiet',
                    '--log-dir',
                    tmp + '/logs',
                    'simulate',
                    '--config',
                    str(CONFIGS / 'default.yaml'),
                    '--set',
                    'run.matrix_set=disturbance',
                    '--set',
                    'run.controllers=[off]',
                    '--results-dir',
                    tmp + '/results',
                    '--no-report',
                ]
            )
            run = next(Path(tmp, 'results').iterdir())
            matrix = json.loads((run / 'metrics/matrix.json').read_text())
            self.assertEqual(sorted(matrix), ['brake_in_turn', 'mu_step', 'split_mu'])
            self.assertEqual(code, 0 if all(c['off']['status'] == 'PASS' for c in matrix.values()) else 1)
            self.assertTrue((run / 'config/cases/split_mu.yaml').is_file())
            if matrix['split_mu']['off']['status'] == 'PASS':
                self.assertEqual(
                    main(
                        [
                            '--quiet',
                            '--log-dir',
                            tmp + '/logs',
                            'render',
                            '--run-id',
                            run.name,
                            '--case',
                            'split_mu',
                            '--format',
                            'report',
                            '--results-dir',
                            tmp + '/results',
                        ]
                    ),
                    0,
                )
                self.assertTrue((run / 'plots/split_mu_report.pdf').is_file())
                self.assertTrue((run / 'plots/split_mu_wheels_off.png').is_file())

    def test_config_commands(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / 'resolved.yaml'
            self.assertEqual(
                main(
                    [
                        '--quiet',
                        '--log-dir',
                        tmp + '/logs',
                        'config',
                        'validate',
                        '--config',
                        str(CONFIGS / 'profiles/robustness.yaml'),
                    ]
                ),
                0,
            )
            self.assertEqual(
                main(
                    [
                        '--quiet',
                        '--log-dir',
                        tmp + '/logs',
                        'config',
                        'show',
                        '--config',
                        str(CONFIGS / 'default.yaml'),
                        '--origins',
                        '--output',
                        str(out),
                    ]
                ),
                0,
            )
            self.assertIn('vehicle.mass_kg: ', out.read_text())
            bad = Path(tmp) / 'bad.yaml'
            bad.write_text(
                (CONFIGS / 'controllers/nmpc.yaml').read_text().replace('horizon_steps: 10', 'horizon_steps: 99')
            )
            self.assertEqual(
                main(['--quiet', '--log-dir', tmp + '/logs', 'config', 'validate', '--config', str(bad)]), 1
            )

    def test_live_plot_configuration(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / 'dash.png'
            code = main(
                [
                    '--quiet',
                    '--log-dir',
                    tmp + '/logs',
                    'dashboard',
                    '--config',
                    str(CONFIGS / 'default.yaml'),
                    '--no-gui',
                    '--output',
                    str(out),
                    '--set',
                    'visualization.live_plots.figure_size_in=[8.0, 5.0]',
                    '--set',
                    'scenario.simulation_duration_s=0.4',
                ]
            )
            self.assertEqual(code, 0)
            from PIL import Image

            with Image.open(out) as image:
                self.assertEqual(image.size, (800, 500))


if __name__ == '__main__':
    unittest.main()
