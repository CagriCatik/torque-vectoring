"""Legacy time-based friction map (split-mu sides, mu step); spatial road friction lives in ``physics.road_friction``."""

from __future__ import annotations
from dataclasses import dataclass
from dataclasses import replace
from math import isfinite
from torquevector.domain.tyre import TyreParameters
from torquevector.physics.combined_slip import CombinedSlipTyre


@dataclass(frozen=True)
class FrictionMap:
    left_mu: float = 0.95
    right_mu: float = 0.95
    after_s: float | None = None
    after_mu: float | None = None
    base_model: object = CombinedSlipTyre()

    def __post_init__(self):
        if not all(isfinite(z) and 0.05 <= z <= 2 for z in (self.left_mu, self.right_mu)):
            raise ValueError('Invalid lateral friction')
        if self.after_s is not None:
            if self.after_mu is None or not (
                isfinite(self.after_s) and self.after_s >= 0 and isfinite(self.after_mu) and 0.05 <= self.after_mu <= 2
            ):
                raise ValueError('Invalid mu-step configuration')

    def at_wheel(self, name: str, t: float, k: float, alpha: float, fz: float, base: TyreParameters):
        if name not in ('FL', 'FR', 'RL', 'RR'):
            raise ValueError('Unknown wheel')
        mu = self.left_mu if name.endswith('L') else self.right_mu
        if self.after_s is not None and t >= self.after_s:
            mu = self.after_mu
        return self.base_model(k, alpha, fz, replace(base, mu=mu))
