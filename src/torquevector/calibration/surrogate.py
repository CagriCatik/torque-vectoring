"""Fit of the canonical Magic Formula tyre (plus elliptic combined-slip damping) to the synthetic brush dataset."""

from __future__ import annotations
import numpy as np
import csv
import json
from pathlib import Path
from torquevector.calibration.synthetic_generation import FIELDS
from scipy.optimize import least_squares

LO = np.array([2.0, 1.05, -1.5, 2.0, 1.05, -1.5, 0.30, -0.27, 0.04, 0.04])


HI = np.array([36.0, 2.4, 1.0, 36.0, 2.4, 1.0, 1.6, 0.0, 0.7, 0.7])


P0 = np.array([11.0, 1.65, 0.45, 7.0, 1.45, 0.35, 0.95, -0.06, 0.17, 0.19])


def read_dataset(csv_path: Path, meta_path: Path):
    metadata = json.loads(meta_path.read_text())
    if metadata.get('dataset_kind') != 'synthetic_independent_surrogate':
        raise ValueError('This calibration pipeline accepts explicitly synthetic surrogate data only')
    with csv_path.open(newline='', encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    if not rows or set(rows[0]) != set(FIELDS):
        raise ValueError('Unexpected data schema')
    groups = {}
    values = []
    for r in rows:
        if r['split'] not in ('train', 'holdout', 'stress'):
            raise ValueError('Invalid split')
        groups.setdefault(r['run_id'], set()).add(r['split'])
        vals = [
            float(r[k])
            for k in ('kappa', 'alpha_rad', 'fz_n', 'temperature_c', 'pressure_kpa', 'speed_ms', 'fx_n', 'fy_n')
        ]
        if not all(np.isfinite(vals)) or vals[2] <= 0:
            raise ValueError('Nonfinite or invalid sample')
        values.append(vals)
    if any(len(s) > 1 for s in groups.values()):
        raise ValueError('Run-id leakage across splits')
    split = np.array([r['split'] for r in rows])
    if sum(split == 'train') < 100 or sum(split == 'holdout') < 40:
        raise ValueError('Insufficient independent holdout runs')
    return np.asarray(values, float), split, metadata


def predict(parameters, data):
    k, a, fz = data[:, 0], data[:, 1], data[:, 2]
    bx, cx, ex, by, cy, ey, mu, load, ascale, kscale = parameters
    d = mu * fz * (fz / 4000.0) ** load

    def mf(z, b, c, e):
        bz = b * z
        return np.sin(c * np.arctan(bz - e * (bz - np.arctan(bz))))

    fx = d * mf(k, bx, cx, ex) / np.sqrt(1 + (a / ascale) ** 2)
    fy = -d * mf(a, by, cy, ey) / np.sqrt(1 + (k / kscale) ** 2)
    util = np.hypot(fx / d, fy / d)
    factor = np.minimum(1.0, 1.0 / np.maximum(util, 1e-10))
    return np.column_stack([fx * factor, fy * factor])


def errors(actual, pred):
    denom = max(100.0, float(np.percentile(np.linalg.norm(actual, axis=1), 95)))
    return {
        'joint_nrmse': float(np.sqrt(np.mean(np.sum((actual - pred) ** 2, axis=1))) / denom),
        'fx_rmse_n': float(np.sqrt(np.mean((actual[:, 0] - pred[:, 0]) ** 2))),
        'fy_rmse_n': float(np.sqrt(np.mean((actual[:, 1] - pred[:, 1]) ** 2))),
        'n': int(len(actual)),
    }


def fit(csv_path: Path, meta_path: Path, seed=903, restarts=3):
    data, split, meta = read_dataset(csv_path, meta_path)
    tr = data[split == 'train']
    rng = np.random.default_rng(seed)
    # No optimizer ever reads holdout/stress rows.
    seeds = [P0] + [np.clip(P0 * rng.uniform(0.80, 1.20, 10), LO + 1e-5, HI - 1e-5) for _ in range(restarts - 1)]
    results = []
    for initial in seeds:
        res = least_squares(
            lambda p: ((predict(p, tr) - tr[:, 6:8]) / 4000).ravel(),
            initial,
            bounds=(LO, HI),
            loss='soft_l1',
            f_scale=0.05,
            max_nfev=800,
        )
        results.append(res)
    best = min(results, key=lambda r: r.cost)
    scores = {}
    for name in ('train', 'holdout', 'stress'):
        subset = data[split == name]
        scores[name] = errors(subset[:, 6:8], predict(best.x, subset))
    return {
        'source_kind': 'SYNTHETIC_ONLY',
        'independent_truth_model': 'brush-inspired tanh force surrogate',
        'fitted_model': 'pure MF B/C/E plus non-MF smooth opposing-slip damping',
        'metadata': meta,
        'fit_parameters': dict(
            zip(
                ['Bx', 'Cx', 'Ex', 'By', 'Cy', 'Ey', 'mu', 'load_exponent', 'alpha_scale_rad', 'kappa_scale'],
                map(float, best.x),
            )
        ),
        'metrics': scores,
        'optimization': {
            'method': 'TRF soft_l1 multi-start',
            'restarts': restarts,
            'seed': seed,
            'success': bool(best.success),
            'evaluations': int(best.nfev),
            'jacobian_condition_number': float(np.linalg.cond(best.jac)),
            'parameters_at_bounds': [
                i for i, (v, l, h) in enumerate(zip(best.x, LO, HI)) if min(abs(v - l), abs(h - v)) < 1e-4
            ],
        },
        'experimental_validation': False,
        'quality_gate_q10': 'BLOCKED_NO_REAL_MEASUREMENTS',
    }


def fitted_tyre_from_report(report):
    """Return a model pair that exactly follows the fitted surrogate equation."""
    from torquevector.domain.tyre import TyreParameters
    from torquevector.domain.tyre import MagicFormulaAxis
    from torquevector.physics.combined_slip import CombinedSlipTyre
    from torquevector.physics.combined_slip import CombinedSlipShape

    if report.get('source_kind') != 'SYNTHETIC_ONLY':
        raise ValueError('Unexpected fit provenance')
    a = report['fit_parameters']
    tyre = TyreParameters(
        longitudinal=MagicFormulaAxis(a['Bx'], a['Cx'], a['Ex']),
        lateral=MagicFormulaAxis(a['By'], a['Cy'], a['Ey']),
        mu=a['mu'],
        load_exponent=a['load_exponent'],
    )
    model = CombinedSlipTyre(CombinedSlipShape(a['alpha_scale_rad'], a['kappa_scale']))
    return tyre, model
