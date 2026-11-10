"""Deterministic global-best particle swarm optimiser in a normalised box (offline tuning only)."""

from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from typing import Callable


@dataclass(frozen=True)
class PSOSettings:
    particles: int = 12
    iterations: int = 15
    seed: int = 42
    cognitive: float = 1.5
    social: float = 1.5
    inertia_start: float = 0.75
    inertia_end: float = 0.40

    def __post_init__(self):
        if not 3 <= self.particles <= 1000 or not 1 <= self.iterations <= 10000:
            raise ValueError('Invalid PSO evaluation budget')
        v = (self.cognitive, self.social, self.inertia_start, self.inertia_end)
        if not all(np.isfinite(vv) and 0 <= vv <= 3 for vv in v):
            raise ValueError('Invalid PSO gains')


@dataclass(frozen=True)
class PSOResult:
    best_position: tuple[float, ...]
    best_fitness: float
    best_history: tuple[float, ...]
    evaluations: int


def minimize_swarm(
    objective: Callable[[np.ndarray], float],
    bounds: tuple[tuple[float, float], ...],
    settings: PSOSettings = PSOSettings(),
    initial_position: tuple[float, ...] | None = None,
) -> PSOResult:
    box = np.array(bounds, dtype=float)
    if (
        box.ndim != 2
        or box.shape[1] != 2
        or len(box) == 0
        or not np.all(np.isfinite(box))
        or not np.all(box[:, 1] > box[:, 0])
    ):
        raise ValueError('Need finite increasing parameter bounds')
    rng = np.random.default_rng(settings.seed)
    lo, hi = box[:, 0], box[:, 1]
    width = hi - lo
    # Optimize in normalized [0,1] space: parameters have different physical units.
    pos = rng.uniform(size=(settings.particles, len(lo)))
    if initial_position is None:
        pos[0] = 0.5  # deterministic reference if no feasible seed is known
    else:
        initial = np.asarray(initial_position, dtype=float)
        if (
            initial.shape != lo.shape
            or not np.all(np.isfinite(initial))
            or np.any(initial < lo)
            or np.any(initial > hi)
        ):
            raise ValueError('Initial PSO position outside parameter box')
        pos[0] = (initial - lo) / width
    vel = np.zeros_like(pos)
    best_pos = pos.copy()
    best_cost = np.full(settings.particles, np.inf)
    leader = pos[0].copy()
    leader_cost = np.inf
    history = []
    evaluations = 0
    for step in range(settings.iterations):
        for i in range(settings.particles):
            candidate = lo + pos[i] * width
            try:
                cost = float(objective(candidate.copy()))
            except (ValueError, ArithmeticError, RuntimeError, FloatingPointError):
                cost = float('inf')
            if not np.isfinite(cost):
                cost = float('inf')
            evaluations += 1
            if cost < best_cost[i]:
                best_pos[i] = pos[i].copy()
                best_cost[i] = cost
            if cost < leader_cost:
                leader = pos[i].copy()
                leader_cost = cost
        history.append(float(leader_cost))
        if step == settings.iterations - 1:
            break
        ratio = step / max(1, settings.iterations - 1)
        inertia = (1 - ratio) * settings.inertia_start + ratio * settings.inertia_end
        vel = (
            inertia * vel
            + settings.cognitive * rng.random(pos.shape) * (best_pos - pos)
            + settings.social * rng.random(pos.shape) * (leader - pos)
        )
        vel = np.clip(vel, -0.3, 0.3)
        pos = np.clip(pos + vel, 0, 1)
        clipped = ((pos == 0) & (vel < 0)) | ((pos == 1) & (vel > 0))
        vel[clipped] = 0.0
    return PSOResult(tuple(map(float, lo + leader * width)), float(leader_cost), tuple(history), evaluations)
