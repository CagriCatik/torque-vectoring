"""Combined-slip surrogate: mutual Fx/Fy damping plus elliptic projection onto the friction limit (not MF Gxa/Gyk)."""

from __future__ import annotations
from dataclasses import dataclass
from math import isfinite
from math import hypot
from math import sqrt
from torquevector.domain.tyre import TyreParameters
from torquevector.domain.tyre import TyreResult
from torquevector.physics.tyre_magic_formula import magic_formula
from torquevector.physics.tyre_magic_formula import peak_grip


@dataclass(frozen=True)
class CombinedSlipShape:
    alpha_scale_rad: float = 0.17
    kappa_scale: float = 0.19
    lateral_peak_fraction: float = 1.0
    longitudinal_peak_fraction: float = 1.0

    def __post_init__(self):
        if not all(
            isfinite(v) and v > 0
            for v in (
                self.alpha_scale_rad,
                self.kappa_scale,
                self.lateral_peak_fraction,
                self.longitudinal_peak_fraction,
            )
        ):
            raise ValueError('Invalid combined-slip scaling parameters')
        if not (
            0.025 <= self.alpha_scale_rad <= 0.7
            and 0.025 <= self.kappa_scale <= 0.9
            and 0.5 <= self.lateral_peak_fraction <= 1.4
            and 0.5 <= self.longitudinal_peak_fraction <= 1.4
        ):
            raise ValueError('Combined-slip shape outside demonstration envelope')


@dataclass(frozen=True)
class CombinedSlipTyre:
    shape: CombinedSlipShape = CombinedSlipShape()

    def __call__(self, kappa: float, alpha_rad: float, fz_n: float, p: TyreParameters) -> TyreResult:
        if not all(isfinite(v) for v in (kappa, alpha_rad, fz_n)):
            raise ValueError('Non-finite slip or load')
        peak = peak_grip(fz_n, p)
        if peak == 0:
            return TyreResult(0.0, 0.0, 0.0, 0.0, fz_n, 0.0, 0.0, kappa, alpha_rad, False)
        fxp = magic_formula(kappa, peak * self.shape.longitudinal_peak_fraction, p.longitudinal)
        fyp = -magic_formula(alpha_rad, peak * self.shape.lateral_peak_fraction, p.lateral)
        fx = fxp / sqrt(1 + (alpha_rad / self.shape.alpha_scale_rad) ** 2)
        fy = fyp / sqrt(1 + (kappa / self.shape.kappa_scale) ** 2)
        # Elliptical utilization rather than Kamm's isotropic circle. This is
        # an explicit *surrogate*, not Pacejka's Gx_alpha / Gy_kappa equations.
        capx = peak * self.shape.longitudinal_peak_fraction
        capy = peak * self.shape.lateral_peak_fraction
        util = hypot(fx / capx, fy / capy)
        scale = min(1.0, 1.0 / util) if util else 1.0
        fx *= scale
        fy *= scale
        # Utilization relative to ellipse, preserves peak calibration per axis.
        usage = hypot(fx / capx, fy / capy)
        return TyreResult(fx, fy, fxp, fyp, fz_n, peak, usage, kappa, alpha_rad, scale < 1 - 1e-12)
