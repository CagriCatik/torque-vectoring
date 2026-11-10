"""Q12 – 2D-Visualisierung und Telemetrie."""

import csv
import tempfile
import unittest
from dataclasses import replace
from math import cos, radians, sin
from pathlib import Path

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from quality.common import run_case, runtime
from torquevector.domain.wheels import WHEEL_ORDER


class Q12Visualization(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        _, _, cls.seq, cls.ticks = run_case('double_lane_change', 'lqr', (('scenario.simulation_duration_s', 3.0),))

    def tearDown(self):
        plt.close('all')

    def test_Q12_1_all_wheel_quantities_labelled(self):
        """Q12.1: alle vier Räder mit Fx, Fy, Fres, Fz, alpha, kappa und Ausnutzung eindeutig beschriftet."""
        from torquevector.visualization.plots import circle_panel
        from torquevector.visualization.report import wheel_figure

        fig = wheel_figure('lqr', self.seq)
        titles = ' | '.join(ax.get_title() for ax in fig.axes)
        for label in ('Fx [N]', 'Fy [N]', 'Fres [N]', 'Fz [N]', 'κ [-]', 'α [deg]', 'Friction utilisation'):
            self.assertIn(label, titles)
        self.assertEqual([t.get_text() for t in fig.axes[0].get_legend().get_texts()], list(WHEEL_ORDER))
        sample = self.seq[40]
        fres = [l for l in fig.axes[2].lines if l.get_label() == 'FR'][0].get_ydata()[40]
        self.assertAlmostEqual(fres, np.hypot(sample.forces.wheels['FR'].fx_n, sample.forces.wheels['FR'].fy_n), 9)
        ax = plt.figure().add_subplot()
        circle_panel(ax, 'RL', sample)
        self.assertIn('RL', ax.get_title())
        self.assertIn('%', ax.get_title())
        self.assertIn('Fz', ax.get_title())

    def test_Q12_2_arrow_direction_transformation_and_length_within_one_percent(self):
        """Q12.2 (kritisch): Pfeilrichtung, Koordinatentransformation, Längenverhältnis gegen bekannte Vektoren <= 1 %."""
        from torquevector.visualization.road_surface_plot import FORCE_SCALE_M_PER_N, draw_forces, force_vectors

        sample = self.seq[30]
        known = {'FL': (1000.0, 0.0), 'FR': (0.0, 2000.0), 'RL': (-1500.0, 500.0), 'RR': (3000.0, -3000.0)}
        for yaw_deg in (0.0, 30.0, -120.0):
            test = replace(
                sample,
                state=replace(sample.state, yaw_rad=radians(yaw_deg)),
                forces=replace(sample.forces, force_body=known),
            )
            vectors = force_vectors(test)
            c, s = cos(radians(yaw_deg)), sin(radians(yaw_deg))
            row = {n: {'x_m': 10.0 + i, 'y_m': -3.0 * i} for i, n in enumerate(WHEEL_ORDER)}
            ax = plt.figure().add_subplot()
            arrows = draw_forces(ax, test, row)
            for arrow, n in zip(arrows, WHEEL_ORDER):
                fx, fy = known[n]
                expected = np.array([c * fx - s * fy, s * fx + c * fy])
                np.testing.assert_allclose(vectors[n], expected, rtol=1e-12, atol=1e-9)
                drawn = (np.array(arrow.xy) - np.array(arrow.xyann)) / FORCE_SCALE_M_PER_N
                self.assertLessEqual(np.linalg.norm(drawn - expected) / np.linalg.norm(expected), 0.01)
                self.assertAlmostEqual(
                    np.linalg.norm(np.array(arrow.xy) - np.array(arrow.xyann)),
                    np.hypot(fx, fy) * FORCE_SCALE_M_PER_N,
                    12,
                )  # Länge ∝ |F|

    def test_Q12_3_friction_circle_uses_true_numerical_limit(self):
        """Q12.3 (kritisch): Kammscher Kreis zeigt echte numerische Kraftgrenzen, keine dekorative Skalierung."""
        from torquevector.physics.tyre_magic_formula import peak_grip
        from torquevector.visualization.plots import circle_panel
        from torquevector.visualization.report import friction_circle_figure

        rt = runtime()
        sample = max(self.seq, key=lambda s: s.forces.wheels['FR'].utilization)
        w = sample.forces.wheels['FR']
        self.assertAlmostEqual(w.peak_n, peak_grip(w.fz_n, rt.tyre), 6)  # Grenze = µ(Fz)·Fz
        ax = plt.figure().add_subplot()
        circle_panel(ax, 'FR', sample)
        tip = [c for c in ax.collections][-1].get_offsets()[0]
        np.testing.assert_allclose(tip, (w.fx_n / w.peak_n, w.fy_n / w.peak_n), rtol=1e-12)
        self.assertAlmostEqual(np.hypot(*tip), w.utilization, 9)
        fig = friction_circle_figure({'lqr': self.seq})
        line = [l for l in fig.axes[1].lines if l.get_label() == 'LQR'][0]
        np.testing.assert_allclose(
            line.get_xdata(), [s.forces.wheels['FR'].fy_n / s.forces.wheels['FR'].peak_n for s in self.seq], rtol=1e-12
        )
        limit = [l for l in fig.axes[1].lines if l.get_label() == 'friction limit'][0]
        self.assertAlmostEqual(float(np.max(np.hypot(limit.get_xdata(), limit.get_ydata()))), 1.0, 12)

    def test_Q12_4_csv_svg_png_export_match_internal_state(self):
        """Q12.4: CSV/SVG/PNG-Export stimmt innerhalb der Rundung mit dem internen Zustand überein."""
        from torquevector.infrastructure.telemetry_io import export_torque_csv
        from torquevector.visualization.report import wheel_figure

        with tempfile.TemporaryDirectory() as root:
            path = Path(root) / 'lqr.csv'
            export_torque_csv(self.seq, path, diagnostics=self.ticks)
            with path.open(encoding='utf-8') as fh:
                rows = list(csv.DictReader(fh))
            self.assertEqual(len(rows), len(self.seq))
            for row, s in zip(rows, self.seq):
                for key, value in (
                    ('t_s', s.t_s),
                    ('vx_ms', s.state.vx_ms),
                    ('yaw_rate_rads', s.state.yaw_rate_rads),
                    ('fl_fx_n', s.forces.wheels['FL'].fx_n),
                    ('rr_fz_n', s.forces.wheels['RR'].fz_n),
                    ('fr_kappa', s.kappa[1]),
                    ('rl_drive_nm', s.drive_nm[2]),
                ):
                    self.assertAlmostEqual(float(row[key]), value, delta=1e-9 * max(1.0, abs(value)))
            fig = wheel_figure('lqr', self.seq)
            for suffix in ('png', 'svg'):
                fig.savefig(Path(root) / f'wheels.{suffix}')
                self.assertGreater((Path(root) / f'wheels.{suffix}').stat().st_size, 10_000)
            fx_line = [l for l in fig.axes[0].lines if l.get_label() == 'FL'][0]
            np.testing.assert_allclose(fx_line.get_ydata(), [s.forces.wheels['FL'].fx_n for s in self.seq], rtol=0)

    def test_Q12_5_legend_units_and_maturity_label_and_gui_regression(self):
        """Q12.5: Legende, Einheiten, Skalen, Kennzeichnung DEMO/CALIBRATED/VALIDATED; GUI-Regressionstest."""
        from torquevector.cli.dashboard import dashboard
        from torquevector.domain.controls import DriverControls
        from torquevector.visualization.report import STATUS, friction_circle_figure, wheel_figure

        for fig in (wheel_figure('lqr', self.seq), friction_circle_figure({'lqr': self.seq})):
            title = fig._suptitle.get_text()
            self.assertIn('NOT VEHICLE VALIDATED', title)
            self.assertIn('DEMO', title)
            self.assertIn(STATUS, title)
            for ax in fig.axes:
                self.assertRegex(ax.get_title() + ax.get_xlabel() + ax.get_ylabel(), r'\[|/')  # Einheit oder Normierung
        rt = runtime()
        fig, _ = dashboard(rt.vehicle, rt.tyre, DriverControls(), speed_ms=18, duration_s=0.3, interactive=True)
        self.assertTrue(any('demonstration model' in t.get_text() for t in fig.texts))
        fig.canvas.draw()
        before = np.asarray(fig.canvas.buffer_rgba()).copy()
        fig._widgets[0].set_val(-4)  # Lenk-Slider: Szene neu berechnet
        fig.canvas.draw()
        changed = np.mean(np.any(np.asarray(fig.canvas.buffer_rgba()) != before, axis=2))
        self.assertGreater(changed, 0.005)  # sichtbare Änderung, kein Absturz


if __name__ == '__main__':
    unittest.main()
