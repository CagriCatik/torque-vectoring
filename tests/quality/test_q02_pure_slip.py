"""Q02 – Pacejka Pure Slip (kanonisches Reifenmodell der Referenzkonfiguration)."""

import json
import unittest

import numpy as np

from quality.common import ROOT, nrmse, runtime
from quality.tyre_data import model_forces, rows

LOADS_N = (2500.0, 4000.0, 5600.0)  # Fz-Stützstellen = Fitdatenbereich (docs/model/specification.md)


class Q02PureSlip(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rt = runtime()
        cls.tyre, cls.model = rt.tyre, rt.tyre_model

    def pure(self, kappa=0.0, alpha=0.0, fz=4000.0):
        return self.model(kappa, alpha, fz, self.tyre)

    def test_Q02_1_identification_with_separated_train_holdout(self):
        """Q02.1 (kritisch, DATENBEGRENZT): Identifikation mit getrennten Trainings-/Testdaten.

        Erfüllt nur synthetisch: unabhängiges Wahrheitsmodell (Brush/tanh, nicht MF), getrennte
        Messläufe. Keine realen Messdaten ⇒ höchstens 1 Punkt (Blocker B1)."""
        train, hold = rows('train'), rows('holdout')
        self.assertFalse(set(train['run_id']) & set(hold['run_id']))
        report = json.loads((ROOT / 'data/synthetic/fit_report.json').read_text())
        self.assertNotIn('Magic Formula', report['independent_truth_model'])
        self.assertFalse(report['experimental_validation'])

    def test_Q02_2_holdout_nrmse_per_axis_and_load_band(self):
        """Q02.2 (kritisch, DATENBEGRENZT): Hold-out-NRMSE Fx und Fy je Betriebsbereich <= 10 %."""
        for split in ('holdout', 'stress'):  # je unabhängigem Messlauf = Betriebsbereich
            data = rows(split)
            fx, fy = model_forces(self.model, self.tyre, data)
            runs = np.array(data['run_id'])
            for run in sorted(set(runs)):
                band = runs == run
                with self.subTest(
                    split=split, run=run, fz=(int(data['fz_n'][band].min()), int(data['fz_n'][band].max()))
                ):
                    self.assertLessEqual(nrmse(fx[band], data['fx_n'][band], 100.0), 0.10)
                    self.assertLessEqual(nrmse(fy[band], data['fy_n'][band], 100.0), 0.10)

    def test_Q02_3_peak_and_initial_stiffness_against_reference(self):
        """Q02.3 (DATENBEGRENZT): Peak <= 10 %, Anfangssteifigkeit <= 15 % gegen die (synthetische) Referenz."""
        from torquevector.calibration.synthetic_generation import brush_surrogate

        slip = np.linspace(0, 0.28, 400)
        for fz in LOADS_N:
            ref_x = brush_surrogate(slip, 0 * slip, fz)[0]
            ref_y = -brush_surrogate(0 * slip, slip, fz)[1]
            mod_x = np.array([self.pure(kappa=s, fz=fz).fx_n for s in slip])
            mod_y = -np.array([self.pure(alpha=s, fz=fz).fy_n for s in slip])
            for mod, ref, axis in ((mod_x, ref_x, 'x'), (mod_y, ref_y, 'y')):
                with self.subTest(fz=fz, axis=axis):
                    self.assertLessEqual(abs(mod.max() / ref.max() - 1), 0.10)
                    self.assertLessEqual(abs((mod[1] / slip[1]) / (ref[1] / slip[1]) - 1), 0.15)

    def test_Q02_4_three_load_points_and_both_slip_signs(self):
        """Q02.4: >= 3 Fz-Stützstellen, antreibend/bremsend bzw. positive/negative Schräglaufwinkel."""
        from torquevector.physics.tyre_magic_formula import peak_grip

        peaks = []
        for fz in LOADS_N:
            for s in (0.02, 0.08, 0.15, 0.28):
                drive, brake = self.pure(kappa=s, fz=fz).fx_n, self.pure(kappa=-s, fz=fz).fx_n
                left, right = self.pure(alpha=s, fz=fz).fy_n, self.pure(alpha=-s, fz=fz).fy_n
                self.assertGreater(drive, 0)
                self.assertAlmostEqual(drive, -brake, 9)
                self.assertLess(left, 0)
                self.assertAlmostEqual(left, -right, 9)
            peaks.append(peak_grip(fz, self.tyre))
        self.assertTrue(peaks[0] < peaks[1] < peaks[2])  # Kraft steigt mit Fz
        self.assertTrue(peaks[0] / LOADS_N[0] > peaks[2] / LOADS_N[2])  # Reibwertdegression

    def test_Q02_5_curves_continuous_without_unexplained_jumps(self):
        """Q02.5: Nullschlupf, kleine Winkel, Peak und Post-Peak stetig, keine Kraftsprünge."""
        self.assertEqual(self.pure().fx_n, 0.0)
        self.assertEqual(self.pure().fy_n, 0.0)
        slip = np.linspace(-0.5, 0.5, 20001)
        step = slip[1] - slip[0]
        for fz in LOADS_N:
            for axis in ('x', 'y'):
                f = np.array(
                    [(self.pure(kappa=s, fz=fz).fx_n if axis == 'x' else self.pure(alpha=s, fz=fz).fy_n) for s in slip]
                )
                slope0 = abs(f[10001] - f[10000]) / step
                with self.subTest(fz=fz, axis=axis):
                    self.assertTrue(np.all(np.isfinite(f)))
                    self.assertLessEqual(np.max(np.abs(np.diff(f))) / step, 1.0001 * slope0)  # Lipschitz ⇒ stetig
                    curvature = np.abs(np.diff(f, 2))
                    self.assertLess(curvature.max(), 1e-2 * slope0 * step)  # keine Knicke


if __name__ == '__main__':
    unittest.main()
