"""Synthetic tyre "measurements" from an independent brush/tanh truth model (not Magic Formula), with train/hold-out/stress runs."""

from __future__ import annotations
import numpy as np
import csv
import json
from pathlib import Path

FIELDS = ('run_id', 'split', 'kappa', 'alpha_rad', 'fz_n', 'temperature_c', 'pressure_kpa', 'speed_ms', 'fx_n', 'fy_n')


def brush_surrogate(kappa, alpha, fz, temp=28.0, pressure=240.0, speed=18.0):
    a = np.asarray(kappa, dtype=float)
    b = np.asarray(alpha, dtype=float)
    fz = np.asarray(fz, dtype=float)
    # Plausible *assumptions*, chosen to challenge MF calibration, NOT real tire coefficients.
    mu = (
        0.98
        * (fz / 4000.0) ** (-0.07)
        * np.clip(1 - 0.0025 * np.abs(np.asarray(temp) - 28), 0.75, 1.0)
        * np.clip(1 - 0.0006 * np.abs(np.asarray(pressure) - 240), 0.85, 1.0)
        * np.clip(1 - 0.0008 * (np.asarray(speed) - 18), 0.85, 1.0)
    )
    peak = mu * fz
    xx = 11.5 * a
    yy = 8.3 * b
    mag = np.hypot(xx, yy)
    factor = np.divide(np.tanh(mag), mag, out=np.ones_like(mag, dtype=float), where=mag > 1e-10)
    return peak * factor * xx, -peak * factor * yy


def generate(out_csv: Path, meta: Path, seed: int = 20261025) -> dict:
    rng = np.random.default_rng(seed)
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    records = []
    # Runs never straddle splits; group-wise separation avoids sample leakage.
    designs = [
        ('train', 2500.0, 28.0, 240.0, 18.0),
        ('train', 3600.0, 29.0, 242.0, 17.0),
        ('train', 4800.0, 27.0, 238.0, 19.0),
        ('train', 5600.0, 28.0, 240.0, 18.0),
        ('holdout', 3100.0, 28.0, 240.0, 18.0),
        ('holdout', 4400.0, 28.0, 240.0, 18.0),
        ('stress', 2800.0, 8.0, 210.0, 27.0),
        ('stress', 5100.0, 47.0, 270.0, 29.0),
    ]
    for n, (split, fz0, temp0, press0, speed0) in enumerate(designs):
        # Each acquisition run includes pure and combined slip coverage, randomized order.
        run_bias = rng.normal(0.0, 0.006, size=2)
        points = (
            [(k, 0.0) for k in np.linspace(-0.28, 0.28, 35)]
            + [(0.0, a) for a in np.linspace(-0.25, 0.25, 35)]
            + [(k, a) for k in np.linspace(-0.23, 0.23, 17) for a in np.linspace(-0.22, 0.22, 17)]
        )
        rng.shuffle(points)
        for j, (k, a) in enumerate(points):
            fz = fz0 * (1 + rng.normal(0, 0.012))
            temp = temp0 + rng.normal(0, 0.5)
            pressure = press0 + rng.normal(0, 1.2)
            speed = max(5.0, speed0 + rng.normal(0, 0.4))
            fx, fy = brush_surrogate(k, a, fz, temp, pressure, speed)
            # Reproducible instrument noise, drift, and occasional outliers.
            sigma = max(12.0, 0.008 * fz)
            error = rng.normal(0.0, sigma, 2) + run_bias * fz
            if rng.random() < 0.008:
                error += rng.normal(0.0, 0.04 * fz, size=2)
            records.append(
                dict(
                    run_id=f'R{n:02d}',
                    split=split,
                    kappa=f'{k:.8f}',
                    alpha_rad=f'{a:.8f}',
                    fz_n=f'{fz:.5f}',
                    temperature_c=f'{temp:.4f}',
                    pressure_kpa=f'{pressure:.4f}',
                    speed_ms=f'{speed:.4f}',
                    fx_n=f'{float(fx+error[0]):.6f}',
                    fy_n=f'{float(fy+error[1]):.6f}',
                )
            )
    with out_csv.open('w', newline='', encoding='utf-8') as f:
        wr = csv.DictWriter(f, fieldnames=FIELDS)
        wr.writeheader()
        wr.writerows(records)
    info = {
        'dataset_kind': 'synthetic_independent_surrogate',
        'source': 'tireviz.synthetic_lab.generate',
        'seed': seed,
        'model': 'tanh-based brush-inspired surrogate (NOT Magic Formula)',
        'assumed_road': 'dry, homogeneous',
        'measurement_status': 'NO PHYSICAL MEASUREMENTS',
        'units': {
            'slip_ratio': '1',
            'alpha': 'rad',
            'forces': 'N',
            'fz': 'N',
            'temperature': 'degC',
            'pressure': 'kPa',
            'speed': 'm/s',
        },
        'split_protocol': 'Entire independent acquisition runs held out; no shared run_id',
        'known_omissions': [
            'camber',
            'pneumatic trail',
            'thermal dynamics',
            'road texture',
            'frequency dependence',
            'hysteresis',
        ],
        'rows_by_split': {key: sum(x['split'] == key for x in records) for key in ('train', 'holdout', 'stress')},
    }
    meta.write_text(json.dumps(info, indent=2) + '\n', encoding='utf-8')
    return info
