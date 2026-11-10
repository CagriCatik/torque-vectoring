"""Fahrbahndarstellung: Dateien, Farbzuordnung, Übereinstimmung von Telemetrie und Darstellung."""

import csv, json, tempfile, unittest
from pathlib import Path
from torquevector.cli.composition import build_road, build_vehicle, load_run
from torquevector.cli.main import main
from torquevector.domain.wheels import WHEEL_ORDER
from torquevector.physics.wheel_contact import wheel_contact_positions
from integration.road.helpers import CONFIGS


class RoadVisualization(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        root = Path(cls.tmp.name)
        code = main(
            [
                '--quiet',
                '--log-dir',
                str(root / 'logs'),
                'compare',
                '--config',
                str(CONFIGS / 'scenarios/straight_multi_mu.yaml'),
                '--controllers',
                'off',
                'lqr',
                '--duration',
                '6',
                '--results-dir',
                str(root / 'results'),
                '--set',
                'visualization.static_plots.report_sections=[kpi, road_map, road_mu]',
                '--set',
                'visualization.gif_export.max_frames=12',
            ]
        )
        assert code == 0
        cls.run_dir = next((root / 'results').iterdir())
        cls.root = root

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_report_gif_and_csv_files(self):
        for name in ('road_map.png', 'road_mu.png', 'report.pdf'):
            self.assertTrue((self.run_dir / 'plots' / name).is_file(), name)
        self.assertEqual(
            main(
                [
                    '--quiet',
                    '--log-dir',
                    str(self.root / 'logs'),
                    'render',
                    '--result',
                    str(self.run_dir),
                    '--format',
                    'road-gif',
                    '--controller',
                    'lqr',
                ]
            ),
            0,
        )
        from PIL import Image

        with Image.open(self.run_dir / 'animations/road_lqr.gif') as gif:
            self.assertGreaterEqual(gif.n_frames, 10)
        rows = list(csv.DictReader((self.run_dir / 'telemetry/lqr.csv').open()))
        self.assertIn('mu_fl', rows[0])
        self.assertIn('surface_id_rr', rows[0])
        diag = json.loads((self.run_dir / 'metrics/controller_diagnostics.json').read_text())['lqr']
        ticked = [r for r in rows if r['solver_status'] == 'SUCCESS']
        self.assertEqual(len(ticked), len(diag))
        self.assertEqual(rows[-1]['solver_status'], 'NO_TICK')
        self.assertAlmostEqual(float(ticked[5]['yaw_moment_requested_nm']), diag[5]['requested_nm'], 9)
        self.assertAlmostEqual(float(ticked[5]['allocation_residual_nm']), diag[5]['residual_nm'], 9)
        off = list(csv.DictReader((self.run_dir / 'telemetry/off.csv').open()))
        self.assertEqual({r['solver_status'] for r in off}, {'NO_TICK'})
        self.assertEqual(off[3]['yaw_moment_requested_nm'], '')
        self.assertTrue(any(float(r['mu_fl']) < 0.5 for r in rows))

    def test_telemetry_contacts_match_vehicle_state(self):
        data = load_run(self.run_dir)
        car = build_vehicle(data['tree']['vehicle'])
        field = build_road(data['tree']['road'])
        for sample, row in zip(data['seqs']['lqr'], data['road_contacts']['lqr']):
            positions = wheel_contact_positions(sample.state, car)
            for name in WHEEL_ORDER:
                self.assertAlmostEqual(row[name]['x_m'], positions[name][0], 9)
                self.assertEqual(row[name]['road_mu'], field.friction_at(*positions[name]))

    def test_colour_map_shows_correct_mu_per_region(self):
        import matplotlib

        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from torquevector.visualization.road_surface_plot import MU_RANGE, draw_friction_map

        field = build_road(load_run(self.run_dir)['tree']['road'])
        fig, ax = plt.subplots()
        image = draw_friction_map(ax, field, (0.0, 100.0, -5.0, 5.0), resolution_m=0.5)
        grid = image.get_array()
        x0, x1, y0, y1 = image.get_extent()

        def at(x, y):
            return grid[round((y - y0) / 0.5), round((x - x0) / 0.5)]

        self.assertEqual(at(37.0, 0.0), 0.4)
        self.assertEqual(at(77.0, 2.0), 0.4)
        self.assertEqual(at(77.0, -2.0), 0.9)
        self.assertEqual(at(10.0, 0.0), 0.9)
        self.assertEqual(image.get_clim(), MU_RANGE)
        plt.close(fig)


if __name__ == '__main__':
    unittest.main()
