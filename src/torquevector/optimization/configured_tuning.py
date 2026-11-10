"""Konfigurierte Offline-PSO-Studie. Ausschließlich offline; nie in der Regelschleife.

Die Studie kennt weder YAML noch Dateipfade: Bewertungsfunktion, Checkpoint-Speicher
und Fallbeschreibungen werden von der Composition Root injiziert. Hold-out- und
Robustheitsfälle werden erst nach Abschluss der Suche ausgewertet.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from typing import Callable, Protocol

import numpy as np

from torquevector.evaluation.comparisons import annotate_relative
from torquevector.evaluation.envelope import QualificationCriteria
from torquevector.optimization.pso import PSOSettings, minimize_swarm


@dataclass(frozen=True)
class SearchDimension:
    parameter: str
    lower: float
    upper: float

    def __post_init__(self):
        if not (isfinite(self.lower) and isfinite(self.upper) and self.lower < self.upper):
            raise ValueError(f'Invalid search bounds for {self.parameter}')


@dataclass(frozen=True)
class ObjectiveSpec:
    yaw_rmse: float
    cross_rmse: float
    max_beta: float
    soft_state: float
    allocation_residual: float
    beta_norm_deg: float
    soft_state_norm: float
    allocation_norm_nm: float
    invalid_cost: float

    def fitness(self, metrics: dict, baseline: dict) -> float:
        """sum w_i*(Kennzahl_i/Normierung_i)^2; Gier-/Bahnfehler relativ zur OFF-Baseline."""
        if baseline['yaw_rmse_rads'] < 1e-6 or baseline['cross_rmse_m'] < 1e-6:
            raise ValueError('Degenerate baseline')
        y = metrics['yaw_rmse_rads'] / baseline['yaw_rmse_rads']
        p = metrics['cross_rmse_m'] / baseline['cross_rmse_m']
        beta = metrics['max_abs_beta_deg'] / self.beta_norm_deg
        soft = metrics.get('predictive_state_excess', 0.0) / self.soft_state_norm
        allocation = metrics.get('maximum_allocation_residual_nm', 0.0) / self.allocation_norm_nm
        return float(
            self.yaw_rmse * y * y
            + self.cross_rmse * p * p
            + self.max_beta * beta * beta
            + self.soft_state * soft * soft
            + self.allocation_residual * allocation * allocation
        )


@dataclass(frozen=True)
class TuningCase:
    id: str
    payload: object


@dataclass(frozen=True)
class TuningStudy:
    study_id: str
    controller: str
    dimensions: tuple[SearchDimension, ...]
    objective: ObjectiveSpec
    pso: PSOSettings
    base_position: tuple[float, ...]
    train: tuple[TuningCase, ...]
    holdout: tuple[TuningCase, ...]
    robustness: tuple[TuningCase, ...]
    qualification: QualificationCriteria

    def __post_init__(self):
        if self.controller not in ('ltv', 'nmpc'):
            raise ValueError('Only predictive controllers are tuned offline')
        if not self.dimensions or len(self.base_position) != len(self.dimensions):
            raise ValueError('Base position must match search dimensions')
        for value, dim in zip(self.base_position, self.dimensions):
            if not dim.lower <= value <= dim.upper:
                raise ValueError(
                    f'Current {dim.parameter}={value} lies outside search bounds ' f'[{dim.lower}, {dim.upper}]'
                )
        if {c.id for c in self.train} & {c.id for c in self.holdout}:
            raise ValueError('Train and hold-out case ids overlap')

    def parameters(self, position) -> dict:
        p = np.asarray(position, float)
        if p.shape != (len(self.dimensions),) or not np.all(np.isfinite(p)):
            raise ValueError('Candidate dimension mismatch')
        for value, dim in zip(p, self.dimensions):
            if not dim.lower <= value <= dim.upper:
                raise ValueError('Candidate outside search box')
        return {dim.parameter: float(value) for value, dim in zip(p, self.dimensions)}


class CandidateStore(Protocol):
    def get(self, key: str): ...
    def put(self, key: str, record: dict) -> None: ...


Evaluator = Callable[[str, TuningCase, dict | None], dict]


def candidate_key(position) -> str:
    return ','.join(repr(float(v)) for v in position)


def _baselines(study, evaluate, label, cases, required):
    rows = []
    for case in cases:
        entry = evaluate('off', case, None)
        if required and entry['status'] != 'PASS':
            raise RuntimeError(f'Invalid {label} baseline {case.id}: {entry}')
        rows.append(entry)
    return rows


def run_study(
    study: TuningStudy, evaluate: Evaluator, store: CandidateStore | None = None, log: Callable[[str], None] = print
) -> dict:
    baselines = {
        'train': _baselines(study, evaluate, 'train', study.train, True),
        'holdout': _baselines(study, evaluate, 'holdout', study.holdout, True),
    }
    candidates = []

    def objective(x):
        key = candidate_key(x)
        cached = store.get(key) if store is not None else None
        if cached is not None:
            candidates.append({**cached, 'index': len(candidates) + 1, 'from_checkpoint': True})
            return cached['fitness']
        params = study.parameters(x)
        statuses, costs = [], []
        for case, baseline in zip(study.train, baselines['train']):
            entry = evaluate(study.controller, case, params)
            statuses.append(entry['status'])
            if entry['status'] != 'PASS':
                break
            costs.append(study.objective.fitness(entry['metrics'], baseline['metrics']))
        cost = float(np.mean(costs)) if len(costs) == len(study.train) else study.objective.invalid_cost
        record = {'key': key, 'parameters': params, 'fitness': cost, 'statuses': statuses}
        if store is not None:
            store.put(key, record)
        candidates.append({**record, 'index': len(candidates) + 1, 'from_checkpoint': False})
        log(f'{study.study_id} candidate {len(candidates)} cost={cost:.5f}')
        return cost

    search = minimize_swarm(
        objective, tuple((d.lower, d.upper) for d in study.dimensions), study.pso, initial_position=study.base_position
    )
    feasible = search.best_fitness < study.objective.invalid_cost
    tuned = study.parameters(search.best_position) if feasible else None
    evaluation = {}
    for label, cases in (('train', study.train), ('holdout', study.holdout), ('robustness', study.robustness)):
        rows = []
        for case in cases:
            entries = {'off': evaluate('off', case, None), study.controller: evaluate(study.controller, case, tuned)}
            annotate_relative(entries)
            entry = entries[study.controller]
            rows.append(
                {
                    'case': case.id,
                    'controllers': entries,
                    'qualifies': entry['status'] == 'PASS' and study.qualification.qualifies(entry.get('vs_off')),
                }
            )
        evaluation[label] = rows
    qualified = feasible and all(r['qualifies'] for label in ('train', 'holdout') for r in evaluation[label])
    return {
        'data_provenance': 'SYNTHETIC_SIMULATION_ONLY',
        'experimental_validation': False,
        'pso_online': False,
        'q10': 'BLOCKED_NO_REAL_MEASUREMENTS',
        'q11': 'SYNTHETIC_CANDIDATE_NOT_VALIDATED' if qualified else 'BLOCKED',
        'study_id': study.study_id,
        'controller': study.controller,
        'status': 'FEASIBLE' if feasible else 'NO_FEASIBLE_CANDIDATE',
        'dimensions': [d.__dict__ for d in study.dimensions],
        'base_position': list(study.base_position),
        'evaluations': search.evaluations,
        'best_fitness': search.best_fitness,
        'best_history': list(search.best_history),
        'best_parameters': tuned,
        'train_cases': [c.id for c in study.train],
        'holdout_cases': [c.id for c in study.holdout],
        'robustness_cases': [c.id for c in study.robustness],
        'holdout_used_in_objective': False,
        'candidates': candidates,
        'evaluation': evaluation,
    }
