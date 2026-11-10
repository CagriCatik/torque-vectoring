"""Q09 – Unabhängige theoretische Benchmarks."""

import ast
import inspect
import unittest

from quality.common import runtime
from torquevector.evaluation import benchmarks, reference_benchmarks


class Q09Benchmarks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rt = runtime()
        cls.rows = reference_benchmarks.run_reference_benchmarks(rt.vehicle, rt.tyre, rt.wheel, rt.tyre_model)

    def select(self, benchmark, quantity=None):
        rows = [
            r
            for r in self.rows
            if r['benchmark'].startswith(benchmark) and (quantity is None or quantity in r['quantity'])
        ]
        self.assertTrue(rows, (benchmark, quantity))
        return rows

    def test_Q09_1_linear_bicycle_independent_without_shared_core_formulas(self):
        """Q09.1 (kritisch): lineares Einspurmodell unabhängig implementiert (kein Code-Sharing der Kernformeln)."""
        tree = ast.parse(inspect.getsource(benchmarks))
        imported = {n.module for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)} | {
            a.name for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names
        }
        self.assertFalse({m for m in imported if m and m.startswith('torquevector')})
        self.assertEqual(
            benchmarks.linear_bicycle_stationary(1500.0, 2.7, 1.2, 20.0, 1.0, 9e4, 1e5)['ay_ms2'],
            20.0 * benchmarks.linear_bicycle_stationary(1500.0, 2.7, 1.2, 20.0, 1.0, 9e4, 1e5)['yaw_rate_rads'],
        )

    def test_Q09_2_linear_range_yaw_rate_and_lateral_acceleration_within_five_percent(self):
        """Q09.2 (kritisch): |ay| <= 0,2 g, stationäre Gierrate und Querbeschleunigung <= 5 %."""
        for row in self.select('B1', 'yaw rate') + self.select('B1', 'lateral acceleration'):
            self.assertLessEqual(row['relative_error'], 0.05, row)
            self.assertLessEqual(abs(row['simulated']), 0.2 * 9.81 if 'lateral' in row['quantity'] else 1.0)

    def test_Q09_3_circle_kinematics_ay_equals_v_r_and_r_equals_v_over_radius(self):
        """Q09.3: Kreisfahrt ay ≈ v·r und r ≈ v/R innerhalb der stationären Annahmen."""
        for row in self.select('B1', 'ay ≈ v·r') + self.select('B1', 'r ≈ v/R'):
            self.assertTrue(row['passed'], row)

    def test_Q09_4_static_and_dynamic_wheel_loads_symmetric_and_asymmetric(self):
        """Q09.4: statische und dynamische Radlastreferenzen für symmetrische und asymmetrische Manöver."""
        for label in ('B2 wheel load static', 'B2 wheel load symmetric', 'B2 wheel load asymmetric'):
            rows = self.select(label)
            self.assertEqual(len(rows), 5)
            for row in rows:
                self.assertLessEqual(row['relative_error'], 1e-6, row)
        asym = {r['quantity']: r['simulated'] for r in self.select('B2 wheel load asymmetric')}
        self.assertGreater(asym['Fz FR'], asym['Fz FL'])  # Linkskurve belastet rechts

    def test_Q09_5_two_reference_manoeuvres_tabulated_with_tolerances(self):
        """Q09.5: mindestens zwei unabhängige Referenzmanöver tabellarisch mit Toleranzen und automatischer Regression."""
        keys = {'benchmark', 'quantity', 'unit', 'simulated', 'reference', 'relative_error', 'tolerance', 'passed'}
        self.assertTrue(all(set(r) == keys for r in self.rows))
        self.assertGreaterEqual(len({r['benchmark'].split()[0] for r in self.rows}), 2)
        self.assertTrue(all(r['passed'] for r in self.rows), [r for r in self.rows if not r['passed']])


if __name__ == '__main__':
    unittest.main()
