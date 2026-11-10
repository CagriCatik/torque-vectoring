"""Reibwert- und radlastbewusste Giermoment-Allokation (Q11.2).

``control.allocation.allocate_yaw`` ist AST-eingefroren und kennt nur eine skalare Aktuatorgrenze.
Diese Variante verwendet dasselbe beschränkte Ausgleichsproblem (identische Zielfunktion, Gewichte
und Solveroptionen), zusätzlich mit einer **radindividuellen** oberen Momentgrenze aus dem
übertragbaren Kraftschluss:

    T_max,i = R * reserve * mu_i * Fz_i,     Fz_i = quasi-statische Radlast aus (ax, ay) ≈ (0, vx*r)

Ist keine Reibgrenze aktiv, sind alle Arrays identisch zu ``allocate_yaw`` und das Ergebnis ist
bitgleich. Grenzen des Ansatzes: Fz aus quasi-statischer Schätzung ohne Längsbeschleunigung, mu ist
die Annahme des Reglers (nicht die wahre Fahrbahn), Seitenkraftbedarf wird über ``reserve`` pauschal
berücksichtigt.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite

import numpy as np
from scipy.optimize import lsq_linear

from torquevector.control.allocation import AllocationOptions, AllocationResult
from torquevector.domain.states import VehicleState
from torquevector.domain.vehicle import VehicleParameters
from torquevector.domain.wheels import WHEEL_ORDER, WheelParameters
from torquevector.physics.load_transfer import wheel_loads


@dataclass(frozen=True)
class FrictionLimit:
    """Parameter der Kraftschlussgrenze (aus YAML, ohne Defaults).

    ``mu``: konstante Reibwertannahme; ``None`` bedeutet, dass die Reibwerte je Rad zur Laufzeit über
    ``mu_at(state)`` geliefert werden (z. B. Kartenreibwert am aktuellen Aufstandspunkt).
    """

    reserve_fraction: float
    mu: float | None

    def __post_init__(self):
        if not (isfinite(self.reserve_fraction) and 0 < self.reserve_fraction <= 1):
            raise ValueError('reserve_fraction muss in (0, 1] liegen')
        if self.mu is not None and not (isfinite(self.mu) and 0.05 <= self.mu <= 2):
            raise ValueError('Reibwert der Kraftschlussgrenze ungültig')


def friction_torque_limits(
    state: VehicleState, car: VehicleParameters, wheel: WheelParameters, reserve_fraction: float, mus
) -> tuple[float, float, float, float]:
    loads = wheel_loads(car, 0.0, state.vx_ms * state.yaw_rate_rads)
    return tuple(wheel.radius_m * reserve_fraction * mu * loads[name] for name, mu in zip(WHEEL_ORDER, mus))


def allocate_yaw_bounded(
    base,
    requested_yaw_nm: float,
    car: VehicleParameters,
    wheel: WheelParameters,
    torque_limit_nm: float,
    options: AllocationOptions,
    wheel_limits_nm: tuple[float, float, float, float],
) -> AllocationResult:
    """Wie ``allocate_yaw``, zusätzlich T_i <= wheel_limits_nm[i] (Basis oberhalb der Grenze bleibt erlaubt,
    wird aber nicht weiter erhöht)."""
    if len(base) != 4 or not all(isfinite(z) for z in base) or not isfinite(requested_yaw_nm):
        raise ValueError('Invalid torques or requested moment')
    if len(wheel_limits_nm) != 4 or not all(isfinite(z) and z >= 0 for z in wheel_limits_nm):
        raise ValueError('Invalid wheel torque limits')
    lever = np.array([-car.contact_points()[n][1] / wheel.radius_m for n in WHEEL_ORDER])
    b = np.array(base, float)
    lower = np.maximum(-b, -options.max_delta_nm * np.ones(4))
    upper = np.minimum(torque_limit_nm - b, options.max_delta_nm * np.ones(4))
    upper = np.minimum(upper, np.maximum(np.asarray(wheel_limits_nm, float) - b, 0.0))
    if np.any(lower > upper):
        raise ValueError('Base torques outside admissible envelope')
    A = np.vstack(
        (options.yaw_weight * lever, options.total_weight * np.ones(4), options.intervention_penalty * np.eye(4))
    )
    target = np.concatenate(([options.yaw_weight * requested_yaw_nm, 0.0], np.zeros(4)))
    pinned = (upper - lower) < 1e-10
    increments = lower.copy()
    if np.any(~pinned):
        reduced = A[:, ~pinned]
        residual_target = target - A[:, pinned] @ increments[pinned]
        solution = lsq_linear(
            reduced, residual_target, bounds=(lower[~pinned], upper[~pinned]), method='bvls', tol=1e-10, max_iter=100
        )
        if not solution.success:
            raise RuntimeError('Torque allocator did not converge')
        increments[~pinned] = solution.x
    actual = b + increments
    realized = float(lever @ increments)
    saturated = bool(np.any(np.abs(increments - lower) < 1e-5) | np.any(np.abs(increments - upper) < 1e-5))
    return AllocationResult(
        tuple(map(float, actual)),
        float(requested_yaw_nm),
        realized,
        float(sum(increments)),
        float(requested_yaw_nm - realized),
        saturated,
    )


def make_allocator(
    car: VehicleParameters, wheel: WheelParameters, options: AllocationOptions, limit: FrictionLimit | None, mu_at=None
):
    """Allokator mit Signatur (base, requested, state, torque_limit) -> AllocationResult."""
    from torquevector.control.allocation import allocate_yaw

    if limit is not None and (limit.mu is None) == (mu_at is None):
        raise ValueError('Kraftschlussgrenze benötigt genau eine Reibwertquelle (konstant oder mu_at)')

    def allocate(base, requested, state, torque_limit_nm):
        if limit is None:
            return allocate_yaw(base, requested, car, wheel, torque_limit_nm, options)
        mus = (limit.mu,) * 4 if limit.mu is not None else tuple(mu_at(state))
        return allocate_yaw_bounded(
            base,
            requested,
            car,
            wheel,
            torque_limit_nm,
            options,
            friction_torque_limits(state, car, wheel, limit.reserve_fraction, mus),
        )

    return allocate
