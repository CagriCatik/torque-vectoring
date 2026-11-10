"""Modell-Gültigkeitsprüfung eines abgeschlossenen Laufs (PASS / INVALID_MODEL_ENVELOPE)."""

from __future__ import annotations
from dataclasses import dataclass
from math import isfinite


@dataclass(frozen=True)
class EnvelopeThresholds:
    max_abs_slip: float
    max_tyre_utilization: float
    max_abs_beta_deg: float
    max_final_speed_deviation_ms: float

    def __post_init__(self):
        values = (
            self.max_abs_slip,
            self.max_tyre_utilization,
            self.max_abs_beta_deg,
            self.max_final_speed_deviation_ms,
        )
        if not all(isfinite(v) and v > 0 for v in values):
            raise ValueError('Envelope thresholds must be positive and finite')


LEGACY_THRESHOLDS = EnvelopeThresholds(0.5, 0.98, 10.0, 3.0)


def envelope_status(
    metrics: dict, target_speed_ms: float, thresholds: EnvelopeThresholds, check_final_speed: bool = True
) -> str:
    if (
        metrics['max_abs_slip'] > thresholds.max_abs_slip
        or metrics['max_tyre_utilization'] > thresholds.max_tyre_utilization
        or metrics['max_abs_beta_deg'] > thresholds.max_abs_beta_deg
        or (
            check_final_speed
            and abs(metrics['final_speed_ms'] - target_speed_ms) > thresholds.max_final_speed_deviation_ms
        )
    ):
        return 'INVALID_MODEL_ENVELOPE'
    return 'PASS'


@dataclass(frozen=True)
class QualificationCriteria:
    """Projektkriterium (Q11-Vorprüfung); keine Freigabe und keine Gate-Wertung."""

    min_yaw_improvement_pct: float
    min_path_improvement_pct: float

    def qualifies(self, vs_off: dict | None) -> bool:
        return bool(vs_off) and (
            vs_off.get('yaw_improvement_pct') is not None
            and vs_off['yaw_improvement_pct'] >= self.min_yaw_improvement_pct
            and vs_off.get('path_improvement_pct') is not None
            and vs_off['path_improvement_pct'] >= self.min_path_improvement_pct
        )
