"""Lossless JSON adapter for immutable telemetry records."""

import json
from dataclasses import asdict
from pathlib import Path
from torquevector.domain.telemetry import TorqueSample
from torquevector.domain.states import VehicleState, VehicleForces
from torquevector.domain.tyre import TyreResult


def write_telemetry(path, sequences):
    Path(path).write_text(
        json.dumps({kind: [asdict(s) for s in seq] for kind, seq in sequences.items()}, allow_nan=False) + '\n'
    )


def read_telemetry(path):
    result = {}
    for kind, rows in json.loads(Path(path).read_text()).items():
        seq = []
        for row in rows:
            f = row['forces']
            f['wheels'] = {name: TyreResult(**w) for name, w in f['wheels'].items()}
            f['force_body'] = {name: tuple(v) for name, v in f['force_body'].items()}
            row['forces'] = VehicleForces(**f)
            row['state'] = VehicleState(**row['state'])
            for name in ('omega_rad_s', 'kappa', 'drive_nm', 'brake_nm'):
                row[name] = tuple(row[name])
            seq.append(TorqueSample(**row))
        result[kind] = seq
    return result
