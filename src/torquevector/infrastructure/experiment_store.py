"""Unique run directories, provenance, explicit incomplete/failure states."""

import hashlib, importlib.metadata, json, platform, re, subprocess, sys
from datetime import datetime
from pathlib import Path

TIMESTAMP_FORMAT = '%Y%m%d-%H%M%S'  # local time, sorts chronologically


def timestamp() -> str:
    return datetime.now().strftime(TIMESTAMP_FORMAT)


def slug(text) -> str:
    return re.sub(r'[^A-Za-z0-9]+', '_', str(text)).strip('_').lower() or 'run'


def unique_directory(root, name: str) -> Path:
    """Create ``root/name``; on a collision append -2, -3, … (never overwrite an existing result)."""
    root = Path(root)
    root.mkdir(parents=True, exist_ok=True)
    for index in range(1, 1000):
        path = root / (name if index == 1 else f'{name}-{index}')
        try:
            path.mkdir()
            return path
        except FileExistsError:
            continue
    raise FileExistsError(f'No free result directory for {name} under {root}')


class ExperimentStore:
    """One result directory per run.

    ``name`` sets the directory name exactly (used for runs nested in a collection, e.g. one folder per scenario);
    otherwise the name is ``<local timestamp>_<label>``, e.g. ``20261026-101530_double_lane_change``.
    """

    def __init__(self, root, configuration, *, seed=None, label=None, name=None):
        self.root = Path(root).expanduser().resolve()
        self.path = unique_directory(self.root, slug(name) if name else f'{timestamp()}_{slug(label or "run")}')
        self.run_id = self.path.name
        for name in ('telemetry', 'metrics', 'plots', 'animations', 'logs'):
            (self.path / name).mkdir()
        packages = {}
        for name in ('numpy', 'scipy', 'casadi', 'matplotlib', 'pillow'):
            try:
                packages[name] = importlib.metadata.version(name)
            except importlib.metadata.PackageNotFoundError:
                packages[name] = None
        package_root = Path(__file__).resolve().parents[1]
        hashes = {
            str(p.relative_to(package_root)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(package_root.rglob('*.py'))
        }
        hashes['resources/synthetic/fit_report.json'] = hashlib.sha256(
            (package_root / 'resources/synthetic/fit_report.json').read_bytes()
        ).hexdigest()
        self.manifest = {
            'run_id': self.run_id,
            'status': 'INCOMPLETE',
            'configuration': configuration,
            'seed': seed,
            'source_kind': 'SYNTHETIC_SIMULATION_ONLY',
            'real_vehicle_validation': False,
            'python': sys.version,
            'platform': platform.platform(),
            'packages': packages,
            'source_sha256': hashes,
            'git_revision': git_revision(package_root),
            'torquevector_version': package_version(),
            'exit_code': None,
            'q10': 'BLOCKED_REQUIRED_REAL_DATA',
            'q11': 'BLOCKED',
        }
        self.save()

    def save(self):
        temporary = self.path / 'manifest.json.tmp'
        temporary.write_text(json.dumps(self.manifest, indent=2, allow_nan=False) + '\n')
        temporary.replace(self.path / 'manifest.json')

    def finish(self, status, exit_code=0, **details):
        self.manifest.update(status=status, exit_code=exit_code, **details)
        self.save()


def git_revision(path):
    """Commit-Hash (mit Markierung lokaler Änderungen) oder expliziter Grund, warum keiner existiert."""
    try:
        head = subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=path, capture_output=True, text=True, timeout=10)
        if head.returncode != 0:
            return 'UNAVAILABLE_NOT_A_GIT_REPOSITORY'
        dirty = subprocess.run(['git', 'status', '--porcelain'], cwd=path, capture_output=True, text=True, timeout=10)
        return head.stdout.strip() + ('+dirty' if dirty.stdout.strip() else '')
    except (OSError, subprocess.SubprocessError):
        return 'UNAVAILABLE_GIT_NOT_INSTALLED'


def package_version():
    try:
        return importlib.metadata.version('torquevector')
    except importlib.metadata.PackageNotFoundError:
        return None


def resolve_run(root, run_id):
    base = Path(root).resolve()
    run = (base / run_id).resolve()
    if run.parent != base:
        raise ValueError('Run ID must identify one direct run directory')
    if not (run / 'manifest.json').is_file():
        raise FileNotFoundError(f'Unknown run: {run_id}')
    return run
