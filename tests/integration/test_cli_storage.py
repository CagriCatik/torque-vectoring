import json, subprocess, sys, tempfile, unittest
from pathlib import Path
from unittest.mock import patch
from torquevector.infrastructure.telemetry_store import read_telemetry, write_telemetry
from torquevector.infrastructure.experiment_store import ExperimentStore, resolve_run
from torquevector.cli.main import main

CONFIG = Path(__file__).parents[2] / 'configs/default.yaml'


class CLITelemetryTests(unittest.TestCase):
    def test_arbitrary_cwd_and_stored_render_never_resimulate(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            command = [
                sys.executable,
                '-m',
                'torquevector.cli.main',
                '--log-dir',
                str(root / 'logs'),
                'compare',
                '--config',
                str(CONFIG),
                '--scenario',
                'circle_track',
                '--controllers',
                'off',
                'nmpc',
                '--duration',
                '.16',
                '--results-dir',
                str(root),
                '--no-report',
            ]
            process = subprocess.run(command, cwd='/tmp', capture_output=True, text=True, timeout=60)
            self.assertEqual(process.returncode, 0, process.stderr)
            record = json.loads(process.stdout.splitlines()[-1])
            run = resolve_run(root, record['run_id'])
            samples = read_telemetry(run / 'telemetry/samples.json')
            copy = root / 'roundtrip.json'
            write_telemetry(copy, samples)
            self.assertEqual(json.loads(copy.read_text()), json.loads((run / 'telemetry/samples.json').read_text()))
            before = (run / 'telemetry/samples.json').read_bytes()
            with patch(
                'torquevector.simulation.runner.simulate_controller', side_effect=AssertionError('render resimulated')
            ):
                for fmt in ('png', 'gif'):
                    self.assertEqual(
                        main(
                            [
                                'render',
                                '--run-id',
                                record['run_id'],
                                '--format',
                                fmt,
                                '--results-dir',
                                str(root),
                                '--max-frames',
                                '10',
                                '--dpi',
                                '60',
                            ]
                        ),
                        0,
                    )
            self.assertEqual(before, (run / 'telemetry/samples.json').read_bytes())
            self.assertGreater((run / 'animations/comparison.gif').stat().st_size, 1000)
            from PIL import Image

            with Image.open(run / 'animations/comparison.gif') as gif:
                self.assertGreaterEqual(gif.n_frames, 2)

    def test_run_ids_and_partial_failure_are_explicit(self):
        with tempfile.TemporaryDirectory() as root:
            first = ExperimentStore(root, {})
            second = ExperimentStore(root, {})
            self.assertNotEqual(first.run_id, second.run_id)
            self.assertEqual(json.loads((first.path / 'manifest.json').read_text())['status'], 'INCOMPLETE')
            first.finish('FAIL', 1, error='injected')
            self.assertEqual(json.loads((first.path / 'manifest.json').read_text())['exit_code'], 1)
            with self.assertRaises(ValueError):
                resolve_run(root, '../bad')

    def test_logs_are_written_to_log_directory_and_run_folder(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            code = main(
                [
                    '--quiet',
                    '--log-dir',
                    str(root / 'logs'),
                    'simulate',
                    '--config',
                    str(CONFIG),
                    '--controller',
                    'off',
                    '--duration',
                    '.16',
                    '--results-dir',
                    str(root / 'results'),
                    '--no-report',
                ]
            )
            self.assertEqual(code, 0)
            logs = list((root / 'logs').glob('*_simulate.log'))
            self.assertEqual(len(logs), 1)
            text = logs[0].read_text()
            self.assertIn('Lauf ', text)
            self.assertIn('OFF', text)
            run = next((root / 'results').iterdir())
            self.assertIn('beendet', (run / 'logs/run.log').read_text())
            code = main(
                [
                    '--quiet',
                    '--log-dir',
                    str(root / 'logs'),
                    'simulate',
                    '--config',
                    str(CONFIG),
                    '--set',
                    'vehicle.mass_kg=-1',
                ]
            )
            self.assertEqual(code, 1)
            self.assertIn('ERROR', sorted((root / 'logs').glob('*_simulate.log'))[-1].read_text())


class OptionalDependencyTests(unittest.TestCase):
    def test_basic_controllers_do_not_import_nmpc_or_visualization_dependencies(self):
        code = '''
import sys,importlib.abc
class BlockOptional(importlib.abc.MetaPathFinder):
    def find_spec(self,fullname,path=None,target=None):
        if fullname.split('.')[0] in ('casadi','matplotlib','PIL'):
            raise AssertionError('unexpected optional import '+fullname)
sys.meta_path.insert(0,BlockOptional())
from torquevector.simulation.runner import simulate_controller
from torquevector.domain.scenarios import presets
for kind in ('off','p','lqr','ltv'):
    metrics,samples=simulate_controller(kind,presets()['circle_track'],.08)
    assert len(samples)==3
'''
        process = subprocess.run(
            [sys.executable, '-I', '-c', code], cwd='/tmp', capture_output=True, text=True, timeout=30
        )
        self.assertEqual(process.returncode, 0, process.stderr)
