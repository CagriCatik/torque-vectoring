"""Repository test-suite adapter; explicit status/timeout, no silent skips."""

import json, os, re, subprocess, sys, time
from pathlib import Path
from torquevector.infrastructure.experiment_store import ExperimentStore


def run_tests(suite, results_root, test_root=None):
    root = Path(test_root) if test_root else Path(__file__).resolve().parents[3] / 'tests'
    if not root.is_dir():
        raise ValueError('Tests are development sources; run validate from an editable checkout')
    store = ExperimentStore(results_root, {'suite': suite}, label=f'tests_{suite}')
    selected = root if suite == 'full' else root / 'unit'
    command = [sys.executable, '-m', 'unittest', 'discover', '-s', str(selected), '-v']
    started = time.perf_counter()
    env = dict(
        os.environ,
        MPLBACKEND='Agg',
        MPLCONFIGDIR='/tmp/torquevector-matplotlib',
        OPENBLAS_NUM_THREADS='1',
        TORQUEVECTOR_TEST_RESULTS=str(store.path / 'metrics'),
    )
    with (store.path / 'logs/tests.log').open('w') as stream:
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, env=env)
        try:
            code = process.wait(timeout=180)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
            store.finish('TIMEOUT', 1, command=command, duration_s=time.perf_counter() - started)
            return 1
    text = (store.path / 'logs/tests.log').read_text()
    count = re.search(r'Ran (\d+) tests?', text)
    status = 'PASS' if code == 0 and 'skipped=' not in text else 'FAIL'
    store.finish(
        status,
        code if code else (0 if status == 'PASS' else 1),
        command=command,
        duration_s=time.perf_counter() - started,
        test_count=int(count[1]) if count else None,
    )
    print(json.dumps({'run_id': store.run_id, 'status': status, 'path': str(store.path)}))
    return store.manifest['exit_code']
