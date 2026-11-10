"""Generic pure-slip Magic Formula identification from CSV measurements with unit checks and train/hold-out leakage protection."""

from __future__ import annotations
import numpy as np
import csv
import json
from pathlib import Path
from dataclasses import asdict
from scipy.optimize import least_squares
from torquevector.domain.tyre import MagicFormulaAxis
from torquevector.domain.tyre import TyreParameters

LB = np.array([2, 1.05, -1.5, 2, 1.05, -1.5, 0.25, -0.3])


UB = np.array([40, 2.4, 1.0, 40, 2.4, 1.0, 1.8, 0.0])


INITIAL = np.array([11, 1.65, 0.45, 7, 1.45, 0.35, 0.95, -0.06])


def read_measurements(csv_path: str | Path, metadata_path: str | Path):
    metadata = json.loads(Path(metadata_path).read_text(encoding='utf-8'))
    if metadata.get('dataset_kind') not in ('synthetic_demo', 'measured'):
        raise ValueError('Metadata must declare dataset_kind explicitly')
    for k in ('source', 'dataset_id', 'slip_units', 'force_units', 'load_units'):
        if not str(metadata.get(k, '')).strip():
            raise ValueError(f'Missing provenance field: {k}')
    if (
        metadata['slip_units'] != {'longitudinal': 'ratio', 'lateral': 'rad'}
        or metadata['force_units'] != 'N'
        or metadata['load_units'] != 'N'
    ):
        raise ValueError('Unsupported units: explicit dimensionless/radians/Newton required')
    with Path(csv_path).open(encoding='utf-8', newline='') as stream:
        reader = csv.DictReader(stream)
        if not {'axis', 'slip', 'fz_n', 'force_n', 'split'}.issubset(set(reader.fieldnames or [])):
            raise ValueError('Missing required measurement columns')
        raw = list(reader)
    rows = []
    for r in raw:
        if r['axis'] not in ('longitudinal', 'lateral') or r['split'] not in ('train', 'holdout'):
            raise ValueError('Unknown axis or split')
        try:
            s = float(r['slip'])
            fz = float(r['fz_n'])
            force = float(r['force_n'])
        except (ValueError, TypeError) as exc:
            raise ValueError('Non-numeric measurement') from exc
        if not all(np.isfinite(x) for x in (s, fz, force)) or not (fz > 100):
            raise ValueError('Nonfinite or invalid load/measurement')
        if abs(s) > (0.5 if r['axis'] == 'longitudinal' else 0.45):
            raise ValueError('Pure-slip measurement outside declared domain')
        rows.append((0 if r['axis'] == 'longitudinal' else 1, s, fz, force, 0 if r['split'] == 'train' else 1))
    if not rows:
        raise ValueError('No measurements')
    a = np.asarray(rows, dtype=float)
    for axis in (0, 1):
        train = a[(a[:, 0] == axis) & (a[:, 4] == 0)]
        test = a[(a[:, 0] == axis) & (a[:, 4] == 1)]
        if len(train) < 30 or len(test) < 12 or len(set(train[:, 2])) < 3:
            raise ValueError('Require >=30 train, >=12 holdout and >=3 training loads per axis')
        for sign in (-1, 1):
            if np.count_nonzero(np.sign(train[:, 1]) == sign) < 8:
                raise ValueError('Both signed slip domains must be represented')
    # Reject identical operating points across train/test; prevents leakage.
    keys = lambda b: {(int(row[0]), round(row[1], 9), round(row[2], 3)) for row in b}
    if keys(a[a[:, 4] == 0]) & keys(a[a[:, 4] == 1]):
        raise ValueError('Train/holdout operating-point leakage detected')
    return a, metadata


def predict(x: np.ndarray, a: np.ndarray) -> np.ndarray:
    b = np.where(a[:, 0] == 0, x[0], x[3])
    c = np.where(a[:, 0] == 0, x[1], x[4])
    e = np.where(a[:, 0] == 0, x[2], x[5])
    bs = b * a[:, 1]
    shape = np.sin(c * np.arctan(bs - e * (bs - np.arctan(bs))))
    peak = x[6] * a[:, 2] * (a[:, 2] / 4000.0) ** x[7]
    return np.where(a[:, 0] == 0, 1.0, -1.0) * peak * shape


def nrmse(ref: np.ndarray, pred: np.ndarray) -> float:
    return float(np.sqrt(np.mean((pred - ref) ** 2)) / max(100.0, np.percentile(np.abs(ref), 95)))


def fit(csv_path: str | Path, metadata_path: str | Path, restarts: int = 5, seed: int = 2026) -> dict:
    a, metadata = read_measurements(csv_path, metadata_path)
    tr = a[a[:, 4] == 0]
    te = a[a[:, 4] == 1]
    rng = np.random.default_rng(seed)
    candidates = [INITIAL]
    for i in range(restarts - 1):
        candidates.append(np.clip(INITIAL * rng.uniform(0.72, 1.28, size=8), LB + 1e-6, UB - 1e-6))
    # Training-only objective, independent test-set report, multi-start robust regression.
    solutions = []
    for initial in candidates:
        result = least_squares(
            lambda p: (predict(p, tr) - tr[:, 3]) / 100.0,
            initial,
            bounds=(LB, UB),
            loss='soft_l1',
            f_scale=0.5,
            max_nfev=2500,
            xtol=1e-11,
            ftol=1e-11,
            gtol=1e-11,
        )
        solutions.append(result)
    best = min(solutions, key=lambda r: r.cost)
    x = best.x
    metrics = {}
    for axis, name in ((0, 'Fx'), (1, 'Fy')):
        axis_test = te[te[:, 0] == axis]
        axis_train = tr[tr[:, 0] == axis]
        metrics[name] = {
            'n_train': len(axis_train),
            'n_holdout': len(axis_test),
            'train_nrmse': nrmse(axis_train[:, 3], predict(x, axis_train)),
            'holdout_nrmse': nrmse(axis_test[:, 3], predict(x, axis_test)),
            'initial_holdout_nrmse': nrmse(axis_test[:, 3], predict(INITIAL, axis_test)),
        }
    singular = np.linalg.svd(best.jac, compute_uv=False)
    cond = float(singular[0] / singular[-1]) if singular[-1] > 1e-15 else float('inf')
    # Numerical conditioning is indicative, not an uncertainty bound.
    params = TyreParameters(
        longitudinal=MagicFormulaAxis(float(x[0]), float(x[1]), float(x[2])),
        lateral=MagicFormulaAxis(float(x[3]), float(x[4]), float(x[5])),
        mu=float(x[6]),
        load_exponent=float(x[7]),
    )
    return {
        'dataset': metadata,
        'parameters': asdict(params),
        'optimizer': {
            'method': 'scipy.optimize.least_squares TRF',
            'loss': 'soft_l1',
            'seed': seed,
            'restarts': restarts,
            'success': bool(best.success),
            'nfev': int(best.nfev),
            'jacobian_condition_number': cond,
            'condition_warning': bool(cond > 1e7),
        },
        'metrics': metrics,
        'status': (
            'SYNTHETIC_IDENTIFICATION_CHECK_ONLY'
            if metadata['dataset_kind'] == 'synthetic_demo'
            else 'MEASUREMENT_FIT_NOT_INDEPENDENT_VEHICLE_VALIDATION'
        ),
        'holdout_under_10pct': all(v['holdout_nrmse'] <= 0.1 for v in metrics.values()),
    }


def generate_demo(out_csv: Path, metadata_path: Path, seed: int = 721):
    """Only generated data; cannot count as real measurements or Q10 evidence."""
    rng = np.random.default_rng(seed)
    p = np.array([12.0, 1.54, 0.32, 8.1, 1.42, 0.29, 0.99, -0.085])
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    with out_csv.open('w', encoding='utf-8', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['axis', 'slip', 'fz_n', 'force_n', 'split'])
        w.writeheader()
        for axis, name in ((0, 'longitudinal'), (1, 'lateral')):
            # Exact signed-slip groups split into train and test to prevent leakage.
            grid = np.linspace(-0.32 if axis == 0 else -0.28, 0.32 if axis == 0 else 0.28, 65)
            for idx, slip in enumerate(grid):
                for fz in (2500.0, 4000.0, 5500.0):
                    a = np.asarray([[axis, slip, fz, 0, 0]], dtype=float)
                    true = predict(p, a)[0]
                    noise = rng.normal(0, 18)
                    # Sparse gross outliers, robust fit should not overfit these.
                    if rng.random() < 0.025:
                        noise += rng.choice((-1, 1)) * rng.uniform(140, 280)
                    w.writerow(
                        {
                            'axis': name,
                            'slip': format(slip, '.9f'),
                            'fz_n': fz,
                            'force_n': float(true + noise),
                            'split': 'holdout' if idx % 5 == 0 else 'train',
                        }
                    )
    metadata = {
        'dataset_kind': 'synthetic_demo',
        'source': 'generated in torquevector.calibration.generate_demo',
        'dataset_id': f'synthetic_seed_{seed}',
        'slip_units': {'longitudinal': 'ratio', 'lateral': 'rad'},
        'force_units': 'N',
        'load_units': 'N',
        'notes': 'Synthetic data: NOT independent experimental validation. Truth coefficients not passed to optimizer.',
    }
    metadata_path.write_text(json.dumps(metadata, indent=2) + '\n', encoding='utf-8')
