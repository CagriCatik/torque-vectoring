"""Q03 – Combined Slip und Kammscher Kreis."""

import itertools
import unittest
from dataclasses import replace
from math import hypot

import numpy as np

from quality.common import ROOT, nrmse, run_case, runtime
from quality.tyre_data import model_forces, rows


class Q03CombinedSlip(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rt = runtime()
        cls.tyre, cls.model = rt.tyre, rt.tyre_model

    def test_Q03_1_limit_model_documented_and_utilisation_visualised(self):
        """Q03.1: Grenzkreis/Reibellipse mit Fz- und mu-Abhängigkeit dokumentiert; Ausnutzung je Rad sichtbar."""
        import matplotlib

        matplotlib.use('Agg')
        from torquevector.physics.tyre_magic_formula import peak_grip
        from torquevector.visualization.report import friction_circle_figure

        spec = (ROOT / 'docs/model/specification.md').read_text()
        self.assertIn('## 7 Combined slip and friction limit', spec)
        r = self.model(0.1, 0.08, 4200.0, self.tyre)
        cap = peak_grip(4200.0, self.tyre)
        self.assertAlmostEqual(r.peak_n, cap, 9)
        self.assertAlmostEqual(r.utilization, hypot(r.fx_n, r.fy_n) / cap, 9)  # c_x = c_y = 1
        wet = self.model(0.1, 0.08, 4200.0, replace(self.tyre, mu=self.tyre.mu * 0.4))
        self.assertAlmostEqual(wet.peak_n / r.peak_n, 0.4, 12)
        _, _, seq, _ = run_case(None, 'off', (('scenario.simulation_duration_s', 1.0),))
        fig = friction_circle_figure({'off': seq})
        self.assertEqual(len(fig.axes), 4)
        self.assertEqual([ax.get_title()[:2] for ax in fig.axes], ['FL', 'FR', 'RL', 'RR'])

    def test_Q03_2_combined_holdout_nrmse(self):
        """Q03.2 (kritisch, DATENBEGRENZT): Combined-Slip-Hold-out-NRMSE je Komponente <= 10 %.

        Nur gegen den synthetischen Brush-Datensatz ⇒ höchstens 1 Punkt (Blocker B1)."""
        for split in ('holdout', 'stress'):
            data = rows(split)
            combined = (np.abs(data['kappa']) > 0.03) & (np.abs(data['alpha_rad']) > 0.03)
            self.assertGreater(combined.sum(), 100)
            fx, fy = model_forces(self.model, self.tyre, data)
            with self.subTest(split=split):
                self.assertLessEqual(nrmse(fx[combined], data['fx_n'][combined], 100.0), 0.10)
                self.assertLessEqual(nrmse(fy[combined], data['fy_n'][combined], 100.0), 0.10)

    def test_Q03_3_pure_slip_limits_reproduced(self):
        """Q03.3 (kritisch): alpha = 0 und kappa = 0 reproduzieren die Pure-Slip-Kurven."""
        from torquevector.physics.tyre_magic_formula import magic_formula, peak_grip

        for fz in (2500.0, 4000.0, 5600.0):
            peak = peak_grip(fz, self.tyre)
            for s in np.linspace(-0.5, 0.5, 101):
                self.assertEqual(self.model(s, 0.0, fz, self.tyre).fx_n, magic_formula(s, peak, self.tyre.longitudinal))
                self.assertEqual(self.model(s, 0.0, fz, self.tyre).fy_n, 0.0)
                self.assertEqual(self.model(0.0, s, fz, self.tyre).fy_n, -magic_formula(s, peak, self.tyre.lateral))
                self.assertEqual(self.model(0.0, s, fz, self.tyre).fx_n, 0.0)

    def test_Q03_4_force_limit_never_exceeded(self):
        """Q03.4: keine Überschreitung der Kraftgrenze > 1e-6 relativ für alle gültigen Testfälle."""
        from torquevector.physics.tyre_magic_formula import peak_grip

        worst = 0.0
        for kappa, alpha, fz, mu in itertools.product(
            np.linspace(-0.5, 0.5, 41),
            np.linspace(-0.4, 0.4, 33),
            (500.0, 2500.0, 4000.0, 5600.0, 9000.0),
            (0.1, 0.4, 1.0, 1.4),
        ):
            tyre = replace(self.tyre, mu=mu)
            r = self.model(kappa, alpha, fz, tyre)
            worst = max(worst, hypot(r.fx_n, r.fy_n) / peak_grip(fz, tyre) - 1)
        self.assertLessEqual(worst, 1e-6)

    def test_Q03_5_no_jumps_or_sign_flips_between_drive_brake_cornering(self):
        """Q03.5: Übergang Antrieb ↔ Bremsung ↔ Querführung ohne Sprünge oder unphysikalische Vorzeichenwechsel."""
        kappa = np.linspace(-0.3, 0.3, 6001)
        for alpha in (-0.12, -0.03, 0.03, 0.12):
            r = [self.model(k, alpha, 4000.0, self.tyre) for k in kappa]
            fx, fy = np.array([x.fx_n for x in r]), np.array([x.fy_n for x in r])
            with self.subTest(alpha=alpha):
                self.assertTrue(np.all(np.sign(fx[kappa > 1e-9]) == 1) and np.all(np.sign(fx[kappa < -1e-9]) == -1))
                self.assertTrue(np.all(np.sign(fy) == -np.sign(alpha)))  # Seitenkraft wechselt nie das Vorzeichen
                for f in (fx, fy):  # Schritt 1e-4: kein Sprung > 1 % von max|F|
                    self.assertLess(np.max(np.abs(np.diff(f))), 0.01 * np.max(np.abs(f)))


if __name__ == '__main__':
    unittest.main()
