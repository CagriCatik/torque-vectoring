"""Per-axle tyre scaling (Pacejka-style scaling factors) for the self-steering balance of the vehicle.

Real vehicles get their understeer/oversteer balance from different front/rear tyres and pressures, roll stiffness
distribution and compliance steer. The four-wheel model uses one tyre for all wheels and a fixed lateral load-transfer
split, so these effects are represented in aggregate by two scaling factors per axle:

- ``lateral_stiffness_scale`` (λ_Ky): multiplies the lateral Magic Formula stiffness factor B_y, i.e. the cornering
  stiffness B·C·D, without changing the peak force;
- ``friction_scale`` (λ_µ): multiplies the friction coefficient, i.e. the peak force D of both directions.

Scaling acts through the ``at_wheel`` hook of ``physics.four_wheel.evaluate``; the frozen four-wheel equations are
unchanged. With all factors equal to 1 no wrapper is created, so the model is bit-identical to the unscaled tyre.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from math import isfinite
from typing import Callable

from torquevector.domain.tyre import TyreParameters


@dataclass(frozen=True)
class AxleScale:
    lateral_stiffness_scale: float
    friction_scale: float

    def __post_init__(self):
        for value in (self.lateral_stiffness_scale, self.friction_scale):
            if not (isfinite(value) and 0.3 <= value <= 3.0):
                raise ValueError('Axle tyre scaling factors must lie in [0.3, 3.0]')

    @property
    def is_neutral(self) -> bool:
        return self.lateral_stiffness_scale == 1.0 and self.friction_scale == 1.0

    def apply(self, tyre: TyreParameters) -> TyreParameters:
        lateral = replace(tyre.lateral, b=tyre.lateral.b * self.lateral_stiffness_scale)
        return replace(tyre, lateral=lateral, mu=tyre.mu * self.friction_scale)


@dataclass(frozen=True)
class TyreAxleScaling:
    front: AxleScale
    rear: AxleScale

    @property
    def is_neutral(self) -> bool:
        return self.front.is_neutral and self.rear.is_neutral

    def for_wheel(self, name: str) -> AxleScale:
        return self.front if name.startswith('F') else self.rear


@dataclass(frozen=True)
class AxleScaledTyre:
    """Tyre model with per-axle scaling; ``base_model(kappa, alpha, fz, tyre)`` is the configured tyre model."""

    base_model: Callable
    scaling: TyreAxleScaling

    def at_wheel(self, name: str, t: float, kappa: float, alpha: float, fz: float, base: TyreParameters):
        return self.base_model(kappa, alpha, fz, self.scaling.for_wheel(name).apply(base))

    def __call__(self, kappa: float, alpha: float, fz: float, tyre: TyreParameters):
        """Axle-independent call (tyre curve plots): the nominal, unscaled tyre."""
        return self.base_model(kappa, alpha, fz, tyre)


def scaled_tyre_model(base_model: Callable, scaling: TyreAxleScaling) -> Callable:
    """The configured tyre model, wrapped only if the scaling is not neutral."""
    return base_model if scaling.is_neutral else AxleScaledTyre(base_model, scaling)
