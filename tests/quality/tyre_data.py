"""Synthetischer Brush-Datensatz (data/synthetic) für die datenbegrenzten Reifenchecks Q02/Q03."""

import csv
from functools import lru_cache

import numpy as np

from quality.common import ROOT


@lru_cache(maxsize=None)
def rows(split):
    with (ROOT / 'data/synthetic/synthetic_brush_measurements.csv').open() as fh:
        data = [r for r in csv.DictReader(fh) if r['split'] == split]
    return {k: np.array([float(r[k]) for r in data]) for k in ('kappa', 'alpha_rad', 'fz_n', 'fx_n', 'fy_n')} | {
        'run_id': [r['run_id'] for r in data]
    }


def model_forces(model, tyre, data):
    out = [model(k, a, f, tyre) for k, a, f in zip(data['kappa'], data['alpha_rad'], data['fz_n'])]
    return np.array([r.fx_n for r in out]), np.array([r.fy_n for r in out])
