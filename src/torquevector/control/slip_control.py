"""Radindividuelle Schlupfregelung (ABS beim Bremsen, ASR beim Antreiben) – erste Version.

Schneller innerer Abtastregler (Mehrraten-ZOH, z. B. 5 ms innerhalb des 40-ms-Torque-Vectoring-Takts):
Zu jedem inneren Takt wird der gemessene Längsschlupf kappa_i ausgewertet und das vom Fahrer bzw.
Torque Vectoring angeforderte Moment je Rad höchstens **reduziert**, nie erhöht:

    e_i = |kappa_i| - kappa_ziel            (nur wirksam, wenn das Rad bremst bzw. antreibt)
    r_i <- clip(r_i + Kp*(e_i - e_i,alt) + Ki*T*e_i, 0, 1)      (Reduktionsanteil, PI in Geschwindigkeitsform)
    T_i  = (1 - r_i) * T_i,soll             mit Änderungsrate |dT_i/dt| <= Rate

Keine Kenntnis von Reibwert oder Reifenmodell; Eingang ist nur der gemessene Schlupf (Simulationswahrheit,
kein Schätzer). Das ist ein bewusst einfaches Ersatzmodell, kein Serien-ABS (keine Druckmodulation,
keine Phasenlogik, keine Fahrzeugreferenzgeschwindigkeitsschätzung).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from math import isfinite

from torquevector.domain.controls import TorqueControls
from torquevector.domain.wheels import WHEEL_ORDER


@dataclass(frozen=True)
class SlipControlSettings:
    abs_enabled: bool
    asr_enabled: bool
    period_s: float
    abs_target_slip: float
    asr_target_slip: float
    kp: float
    ki_per_s: float
    torque_rate_nm_s: float

    def __post_init__(self):
        values = (
            self.period_s,
            self.abs_target_slip,
            self.asr_target_slip,
            self.kp,
            self.ki_per_s,
            self.torque_rate_nm_s,
        )
        if not all(isfinite(v) for v in values) or not (
            0 < self.period_s <= 0.04
            and 0 < self.abs_target_slip < 0.5
            and 0 < self.asr_target_slip < 0.5
            and self.kp >= 0
            and self.ki_per_s >= 0
            and self.torque_rate_nm_s > 0
        ):
            raise ValueError('Invalid slip control settings')

    @property
    def active(self) -> bool:
        return self.abs_enabled or self.asr_enabled


@dataclass
class SlipController:
    """Zustandsbehafteter innerer Regler (ein Exemplar je Simulationslauf)."""

    settings: SlipControlSettings
    reduction: list = field(init=False)
    last_error: list = field(init=False)
    applied: tuple | None = field(init=False)
    log: list = field(init=False)

    def __post_init__(self):
        # Zustand, keine Parameter: Start ohne Reduktion und ohne Vorgeschichte.
        self.reduction = [0.0 for _ in WHEEL_ORDER]
        self.last_error = [0.0 for _ in WHEEL_ORDER]
        self.applied, self.log = None, []

    def __call__(self, t: float, kappa, requested: TorqueControls) -> TorqueControls:
        s = self.settings
        drive, brake = list(requested.drive_nm), list(requested.brake_nm)
        for i in range(len(WHEEL_ORDER)):
            braking = brake[i] > drive[i]
            enabled = s.abs_enabled if braking else s.asr_enabled
            target = s.abs_target_slip if braking else s.asr_target_slip
            slip = -kappa[i] if braking else kappa[i]  # positiv = in Wirkrichtung überschrittener Schlupf
            error = (slip - target) if enabled and max(drive[i], brake[i]) > 0 else -1.0
            if not enabled:
                self.reduction[i] = 0.0
            else:
                step = s.kp * (error - self.last_error[i]) + s.ki_per_s * s.period_s * error
                self.reduction[i] = min(1.0, max(0.0, self.reduction[i] + step))
            self.last_error[i] = error if enabled else 0.0
            factor = 1.0 - self.reduction[i]
            if braking:
                brake[i] *= factor
            else:
                drive[i] *= factor
        command = self._rate_limited(drive, brake)
        self.log.append(
            {
                'time_s': float(t),
                'kappa': [float(k) for k in kappa],
                'reduction': list(self.reduction),
                'drive_nm': list(command[0]),
                'brake_nm': list(command[1]),
            }
        )
        return TorqueControls(requested.steer_deg, tuple(command[0]), tuple(command[1]), requested.torque_limit_nm)

    def _rate_limited(self, drive, brake):
        if self.applied is None:
            self.applied = (tuple(drive), tuple(brake))
            return self.applied
        step = self.settings.torque_rate_nm_s * self.settings.period_s

        def limit(new, old):
            return tuple(min(o + step, max(o - step, n)) for n, o in zip(new, old))

        self.applied = (limit(drive, self.applied[0]), limit(brake, self.applied[1]))
        return self.applied


def slip_control_metrics(log: list) -> dict:
    """Kennzahlen des inneren Reglers: Eingriffsanteil je Rad und maximale Reduktion."""
    if not log:
        return {'inner_ticks': 0, 'intervention_fraction': [0.0] * 4, 'max_reduction': [0.0] * 4}
    return {
        'inner_ticks': len(log),
        'intervention_fraction': [sum(r['reduction'][i] > 1e-9 for r in log) / len(log) for i in range(4)],
        'max_reduction': [max(r['reduction'][i] for r in log) for i in range(4)],
    }
