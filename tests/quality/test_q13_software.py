"""Q13 – Softwarearchitektur, Tests und Reproduzierbarkeit."""

import ast
import csv
import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from quality.common import CONFIGS, ROOT

SKIP_CALLS = {'skipTest', 'skip', 'skipIf', 'skipUnless', 'expectedFailure'}


class Q13Software(unittest.TestCase):
    def test_Q13_1_separated_modules_and_domain_without_gui(self):
        """Q13.1 (kritisch): getrennte Module; Domain importiert keine GUI."""
        from torquevector.evaluation.architecture import scan

        package = ROOT / 'src/torquevector'
        for layer in ('domain', 'physics', 'control', 'simulation', 'evaluation', 'visualization', 'cli'):
            self.assertTrue((package / layer).is_dir(), layer)
        result = scan(package)
        self.assertEqual(result['violations'], [])
        self.assertEqual(result['cycles'], [])
        for path in (package / 'domain').rglob('*.py'):
            tree = ast.parse(path.read_text())
            modules = {a.name for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names} | {
                n.module or '' for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)
            }
            self.assertFalse(
                {m for m in modules if m.split('.')[0] in ('matplotlib', 'PIL', 'tkinter')}
                or {m for m in modules if '.visualization' in m or '.cli' in m},
                path,
            )

    def test_Q13_2_test_kinds_and_all_critical_checks_in_ci(self):
        """Q13.2 (kritisch): Unit-, Integrations-, Invarianten-, Grenztests; 100 % der kritischen Checks laufen in CI."""
        for kind in ('unit', 'integration', 'regression', 'architecture', 'system', 'quality'):
            self.assertTrue(list((ROOT / 'tests' / kind).rglob('test_*.py')), kind)
        workflow = (ROOT / '.github/workflows/tests.yml').read_text()
        self.assertIn('unittest discover -s tests', workflow)  # entdeckt auch tests/quality
        with (ROOT / 'docs/quality/quality_scorecard.csv').open() as fh:
            critical = [r['check_id'] for r in csv.DictReader(fh) if r['critical'] == 'true']
        names = ' '.join(p.read_text() for p in (ROOT / 'tests/quality').glob('test_q*.py'))
        missing = [c for c in critical if f"test_{c.replace('.', '_')}_" not in names and not c.startswith('Q10')]
        self.assertEqual(missing, [])
        self.assertIn('test_Q10_status_required_real_data_declared', names)

    def test_Q13_3_no_unexpected_skips_and_critical_branch_tests(self):
        """Q13.3: keine unerwarteten Skips; explizite Tests der kritischen Fail-Closed-Zweige.

        Die Core-Line-Coverage (>= 90 %) misst torquevector quality-audit mit coverage.py (.coveragerc)."""
        for path in (ROOT / 'tests').rglob('test_*.py'):  # echte Aufrufe/Dekoratoren, keine Strings
            tree = ast.parse(path.read_text())
            calls = {
                (n.func.attr if isinstance(n.func, ast.Attribute) else getattr(n.func, 'id', ''))
                for n in ast.walk(tree)
                if isinstance(n, ast.Call)
            }
            decorators = {
                ast.unparse(d).split('(')[0].split('.')[-1]
                for n in ast.walk(tree)
                if isinstance(n, (ast.FunctionDef, ast.ClassDef))
                for d in n.decorator_list
            }
            with self.subTest(file=str(path.relative_to(ROOT))):
                self.assertFalse((calls | decorators) & SKIP_CALLS)
        branches = ' '.join(p.read_text() for p in (ROOT / 'tests').rglob('test_*.py'))
        for message in ('contact speed', 'Slip', 'wheel lift', 'FAIL_CLOSED', 'OutsideEnvelopeError'):
            self.assertIn(message, branches)
        self.assertTrue((ROOT / '.coveragerc').is_file())

    def test_Q13_4_cli_reproducible_json_csv_and_machine_readable_gate_report(self):
        """Q13.4: CLI-Lauf mit fixer Konfiguration ⇒ reproduzierbare JSON/CSV-Ergebnisse und Gate-Report."""
        env = dict(os.environ, MPLBACKEND='Agg', TORQUEVECTOR_LOG_DIR=tempfile.gettempdir())
        with tempfile.TemporaryDirectory() as root:
            outputs = []
            for _ in range(2):
                cmd = [
                    sys.executable,
                    '-m',
                    'torquevector.cli.main',
                    '--quiet',
                    'simulate',
                    '--controller',
                    'lqr',
                    '--results-dir',
                    root,
                    '--no-report',
                    '--set',
                    'scenario.simulation_duration_s=1.5',
                ]
                done = subprocess.run(cmd, capture_output=True, text=True, env=env, cwd=ROOT, timeout=300)
                self.assertEqual(done.returncode, 0, done.stderr[-2000:])
                outputs.append(Path(json.loads(done.stdout.strip().splitlines()[-1])['path']))
            csv_files = [sorted((o / 'telemetry').glob('*.csv')) for o in outputs]
            self.assertTrue(csv_files[0])
            for a, b in zip(*csv_files):
                self.assertEqual(a.read_bytes(), b.read_bytes())
            metrics = [json.loads((o / 'metrics/comparison.json').read_text()) for o in outputs]
            strip = lambda d: {k: v.get('metrics') for k, v in d.get('controllers', d).items() if isinstance(v, dict)}
            self.assertEqual(strip(metrics[0]), strip(metrics[1]))
            gate = subprocess.run(
                [
                    sys.executable,
                    '-m',
                    'torquevector.evaluation.quality_gates',
                    '--scorecard',
                    str(ROOT / 'docs/quality/quality_scorecard.csv'),
                    '--manifest',
                    str(ROOT / 'docs/quality/quality_gate_manifest.json'),
                    '--json',
                    f'{root}/gates.json',
                ],
                capture_output=True,
                text=True,
                cwd=ROOT,
                timeout=120,
            )
            report = json.loads(Path(f'{root}/gates.json').read_text())
            self.assertEqual(gate.returncode, 1)  # ehrlich: aktuell BLOCKED
            self.assertEqual(report['category_count'], 13)

    def test_Q13_5_docs_pinned_dependencies_and_fresh_setup(self):
        """Q13.5: README, Parameter-/Quellenverzeichnis, Validierungsgrenzen, Changelog, feste Versionen, frisches Setup."""
        for doc in (
            'README.md',
            'docs/configuration/PARAMETER_REFERENCE.md',
            'docs/model/specification.md',
            'docs/limitations.md',
            'CHANGELOG.md',
            'docs/development.md',
            'docs/quality/QUALITY_GATES.md',
        ):
            self.assertGreater((ROOT / doc).stat().st_size, 1000, doc)
        lock_path = (
            ROOT / 'requirements-lock-macos-arm64-py313.txt'
            if (ROOT / 'requirements-lock-macos-arm64-py313.txt').is_file()
            else ROOT / 'requirements.txt'
        )
        lock = lock_path.read_text().splitlines()
        pins = [l for l in lock if l and not l.startswith('#')]
        self.assertTrue(pins and all('==' in l for l in pins))
        build_existed = (ROOT / 'build').exists()
        self.addCleanup(lambda: build_existed or shutil.rmtree(ROOT / 'build', ignore_errors=True))
        with tempfile.TemporaryDirectory() as root:  # frisches Setup aus dem Quellbaum
            wheel = subprocess.run(
                [
                    sys.executable,
                    '-m',
                    'pip',
                    'wheel',
                    str(ROOT),
                    '--no-deps',
                    '--no-build-isolation',
                    '-q',
                    '-w',
                    root,
                ],
                capture_output=True,
                text=True,
                timeout=600,
            )
            self.assertEqual(wheel.returncode, 0, wheel.stderr[-2000:])
            base = getattr(sys, '_base_executable', sys.executable)
            venv = Path(root) / 'venv'
            subprocess.run([base, '-m', 'venv', '--system-site-packages', str(venv)], check=True, timeout=300)
            python = venv / 'bin/python'
            install = subprocess.run(
                [
                    str(python),
                    '-m',
                    'pip',
                    'install',
                    '-q',
                    '-c',  # Abhängigkeiten aufgelöst,
                    str(lock_path),  # gegen feste Versionen
                    *map(str, Path(root).glob('torquevector-*.whl')),
                ],
                capture_output=True,
                text=True,
                timeout=900,
            )
            self.assertEqual(install.returncode, 0, install.stderr[-2000:])
            env = dict(os.environ, MPLBACKEND='Agg', TORQUEVECTOR_LOG_DIR=root, PYTHONPATH='')
            check = subprocess.run(
                [str(venv / 'bin/torquevector'), 'config', 'validate', '--config', str(CONFIGS / 'default.yaml')],
                capture_output=True,
                text=True,
                env=env,
                cwd=root,
                timeout=300,
            )
            self.assertEqual(check.returncode, 0, check.stdout[-1000:] + check.stderr[-2000:])
            self.assertIn('VALID', check.stdout)


if __name__ == '__main__':
    unittest.main()
