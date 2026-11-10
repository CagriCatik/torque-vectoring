"""Run every test file in its own interpreter with a timeout and write a JSON evidence report.

Isolation catches order dependencies and leaked global state that a single ``unittest discover`` run can hide.
Skips, failures and timeouts are never counted as passed.

    python scripts/run_isolated_tests.py --timeout 300 --output results/test_runs/<run>/report.json
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
import os
import platform
import re
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PACKAGES = ('numpy', 'scipy', 'matplotlib', 'pillow', 'casadi', 'pyyaml')


def source_digests() -> dict[str, str]:
    return {
        str(f.relative_to(ROOT)): hashlib.sha256(f.read_bytes()).hexdigest()
        for folder in ('src', 'tests', 'scripts')
        for f in sorted((ROOT / folder).rglob('*.py'))
    }


def package_versions() -> dict[str, str | None]:
    versions = {}
    for name in PACKAGES:
        try:
            versions[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            versions[name] = None
    return versions


def run_file(path: Path, timeout_s: float, log: Path) -> dict:
    command = [sys.executable, '-m', 'unittest', 'discover', '-s', 'tests', '-p', path.name, '-v']
    env = dict(os.environ, MPLBACKEND='Agg', PYTHONHASHSEED='0', OPENBLAS_NUM_THREADS='1')
    started = time.monotonic()
    with log.open('w') as stream:
        process = subprocess.Popen(command, cwd=ROOT, env=env, stdout=stream, stderr=subprocess.STDOUT)
        try:
            code = process.wait(timeout=timeout_s)
            status = 'PASS' if code == 0 else 'FAIL'
        except subprocess.TimeoutExpired:
            process.kill()
            code = process.wait()
            status = 'TIMEOUT'
    text = log.read_text()
    count = re.search(r'Ran (\d+) tests?', text)
    return {
        'file': str(path.relative_to(ROOT)),
        'status': status,
        'exit_code': code,
        'duration_s': round(time.monotonic() - started, 3),
        'test_count': int(count[1]) if count else None,
        'skips': re.findall(r'skipped (.*)', text),
        'log': str(log.relative_to(ROOT)),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--timeout', type=float, default=300.0, help='seconds per test file')
    parser.add_argument('--pattern', default='test_*.py')
    parser.add_argument('--output', default=f"results/test_runs/{time.strftime('%Y%m%d-%H%M%S')}/report.json")
    args = parser.parse_args()
    output = ROOT / args.output
    output.parent.mkdir(parents=True, exist_ok=True)
    digests = source_digests()
    report = {
        'python': sys.version,
        'platform': platform.platform(),
        'packages': package_versions(),
        'source_sha256': digests,
        'runs': [],
        'status': 'INCOMPLETE',
    }
    for path in sorted((ROOT / 'tests').rglob(args.pattern)):
        row = run_file(path, args.timeout, output.parent / f'{path.stem}.log')
        report['runs'].append(row)
        output.write_text(json.dumps(report, indent=2) + '\n')
        print(f"{row['file']:<60} {row['status']:<8} {row['duration_s']:8.1f} s", flush=True)
    report['source_changed_during_run'] = [
        n for n, d in digests.items() if hashlib.sha256((ROOT / n).read_bytes()).hexdigest() != d
    ]
    if any(r['status'] == 'TIMEOUT' for r in report['runs']) or report['source_changed_during_run']:
        report['status'] = 'INCOMPLETE'
    else:
        report['status'] = 'PASS' if all(r['status'] == 'PASS' and not r['skips'] for r in report['runs']) else 'FAIL'
    output.write_text(json.dumps(report, indent=2) + '\n')
    print(f"OVERALL {report['status']} -> {output}")
    return 0 if report['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
