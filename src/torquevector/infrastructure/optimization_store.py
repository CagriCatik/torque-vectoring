"""Versionierte Offline-Optimierungsergebnisse unter results/optimization/.

Regler-YAML-Dateien unter configs/ werden nie überschrieben: Ergebnisse landen in
einem eindeutigen Ordner ``<zeit>_<studie>`` und enthalten eine eigenständig
ladbare ``best_parameters.yaml``.
"""

from __future__ import annotations

import csv
import importlib.metadata
import json
import platform
import sys
from pathlib import Path

from torquevector.infrastructure.config.yaml_io import dump
from torquevector.infrastructure.experiment_store import timestamp, unique_directory


class OptimizationRun:
    def __init__(self, root, study_id: str, *, resume: str | None = None, forbidden_roots=()):
        root = Path(root).expanduser().resolve()
        for forbidden in forbidden_roots:
            forbidden = Path(forbidden).resolve()
            if root == forbidden or forbidden in root.parents:
                raise ValueError(f'Ergebnisse dürfen nicht unter {forbidden} geschrieben werden')
        if resume:
            self.path = Path(resume).expanduser().resolve()
            if not (self.path / 'metadata.yaml').is_file():
                raise FileNotFoundError(f'Kein wiederaufnehmbarer Optimierungslauf: {self.path}')
        else:
            self.path = unique_directory(root, f'{timestamp()}_{study_id}')
        self.run_id = self.path.name

    def write_metadata(self, metadata: dict):
        packages = {}
        for name in ('numpy', 'scipy', 'casadi', 'pyyaml'):
            try:
                packages[name] = importlib.metadata.version(name)
            except importlib.metadata.PackageNotFoundError:
                packages[name] = None
        payload = {
            'run_id': self.run_id,
            'python': sys.version.split()[0],
            'platform': platform.platform(),
            'packages': packages,
            **metadata,
        }
        (self.path / 'metadata.yaml').write_text(
            dump(payload, header='torquevector Offline-PSO-Metadaten'), encoding='utf-8'
        )

    def write_results(self, report: dict, best_controller: dict | None):
        with (self.path / 'convergence.csv').open('w', newline='', encoding='utf-8') as stream:
            writer = csv.writer(stream)
            writer.writerow(['iteration', 'best_fitness'])
            for index, value in enumerate(report['best_history'], start=1):
                writer.writerow([index, repr(float(value))])
        (self.path / 'evaluation.json').write_text(
            json.dumps(report, indent=2, allow_nan=False, default=str) + '\n', encoding='utf-8'
        )
        target = self.path / 'best_parameters.yaml'
        if best_controller is not None:
            target.write_text(
                dump(
                    {'schema_version': 1, 'controller': best_controller},
                    header='Optimierte Reglerparameter (SYNTHETISCH, nicht fahrzeugvalidiert)',
                ),
                encoding='utf-8',
            )
        elif target.exists():
            target.unlink()


class CandidateCheckpoint:
    """JSONL-Bewertungscache. Gleicher Seed + Cache = deterministische Wiederaufnahme."""

    def __init__(self, path, fingerprint: str):
        self.path = Path(path)
        self.fingerprint = fingerprint
        self.records = {}
        if self.path.exists():
            lines = self.path.read_text(encoding='utf-8').splitlines()
            header = json.loads(lines[0]) if lines else {}
            if header.get('fingerprint') != fingerprint:
                raise ValueError('Checkpoint gehört zu einer anderen Studienkonfiguration (Fingerprint abweichend)')
            for line in lines[1:]:
                record = json.loads(line)
                self.records[record['key']] = record
        else:
            self.path.write_text(json.dumps({'fingerprint': fingerprint}) + '\n', encoding='utf-8')

    def get(self, key):
        return self.records.get(key)

    def put(self, key, record):
        self.records[key] = record
        with self.path.open('a', encoding='utf-8') as stream:
            stream.write(json.dumps(record, allow_nan=False) + '\n')
            stream.flush()
