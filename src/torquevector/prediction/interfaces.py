"""Prediction ports and data types: preview over the horizon, discrete models, linearisers."""

from __future__ import annotations
from dataclasses import dataclass
from math import isfinite
import numpy as np
from typing import Protocol


@dataclass(frozen=True)
class PredictionPreview:
    """One value per prediction stage; referenced yaw is at end of each stage."""

    speed_ms: tuple[float, ...]
    steering_rad: tuple[float, ...]
    desired_beta_rad: tuple[float, ...]
    desired_yaw_rads: tuple[float, ...]
    # Optionale Reibwertvorschau (Kartenwissen): Verhältnis angenommener/nominaler Reifenreibwert
    # je Stufe für Vorder- und Hinterachse. None = Regler kennt die Fahrbahn nicht (Modus unknown).
    front_friction_ratio: tuple[float, ...] | None = None
    rear_friction_ratio: tuple[float, ...] | None = None

    def validate(self, n: int):
        rows = (self.speed_ms, self.steering_rad, self.desired_beta_rad, self.desired_yaw_rads)
        if any(len(row) != n for row in rows):
            raise ValueError('Prediction preview has wrong number of stages')
        if (self.front_friction_ratio is None) != (self.rear_friction_ratio is None):
            raise ValueError('Friction preview requires front and rear ratios')
        if self.front_friction_ratio is not None:
            ratios = self.front_friction_ratio + self.rear_friction_ratio
            if len(self.front_friction_ratio) != n or len(self.rear_friction_ratio) != n:
                raise ValueError('Friction preview has wrong number of stages')
            if not all(isfinite(float(v)) and v > 0 for v in ratios):
                raise ValueError('Friction preview ratios must be positive and finite')
        if not all(isfinite(float(v)) for row in rows for v in row):
            raise ValueError('Non-finite preview')
        if any(v <= 0 for v in self.speed_ms):
            raise ValueError('Prediction preview requires positive speed')


@dataclass(frozen=True)
class DiscreteModel:
    a: np.ndarray
    b: np.ndarray
    steering: np.ndarray


class PredictionLinearizer(Protocol):
    def discrete(self, speed_ms: float, period_s: float) -> DiscreteModel: ...


class PredictionModel(Protocol):
    """SI state beta/r, yaw input kNm; validity checked by concrete model owner."""

    def step(self, state, moment_knm, speed_ms, steering_rad, period_s): ...


class PreviewConfig(Protocol):
    sample_s: float
    horizon_steps: int
    min_speed_ms: float
    max_speed_ms: float
    mu_reference: float
