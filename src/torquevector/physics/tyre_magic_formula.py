"""Pure-slip Magic Formula (B, C, D, E), load-sensitive peak grip and Kamm-circle combined slip."""

from __future__ import annotations
from math import atan
from math import isfinite
from math import sin
from torquevector.domain.tyre import MagicFormulaAxis
from torquevector.domain.tyre import TyreParameters
from math import hypot
from torquevector.domain.tyre import TyreResult


def magic_formula(slip: float, peak_force: float, shape: MagicFormulaAxis) -> float:
    """Y = D sin(C atan(B s - E (B s - atan(B s))))."""
    if not isfinite(slip) or not isfinite(peak_force):
        raise ValueError('Non-finite input')
    bs = shape.b * slip
    return peak_force * sin(shape.c * atan(bs - shape.e * (bs - atan(bs))))


def peak_grip(fz_n: float, parameters: TyreParameters) -> float:
    if not isfinite(fz_n) or fz_n < 0:
        raise ValueError('Fz must be finite and nonnegative')
    if fz_n == 0:
        return 0.0
    # Slight load sensitivity: effective mu declines under heavier load.
    mu_eff = parameters.mu * (fz_n / parameters.nominal_fz_n) ** parameters.load_exponent
    return max(0.0, mu_eff * fz_n)


def tyre_forces(kappa: float, alpha_rad: float, fz_n: float, parameters: TyreParameters) -> TyreResult:
    """Pure MF + isotropic Kamm-circle radial projection for combined slip.

    This IS NOT a calibrated MF5.2/MF6.x combined-slip implementation. Kamm
    projection respects |F|<=mu(Fz)*Fz but is a simplified coupling law.
    Positive alpha (velocity toward wheel-left) causes negative lateral force.
    """
    if not all(map(isfinite, (kappa, alpha_rad))):
        raise ValueError('Slip inputs must be finite')
    peak = peak_grip(fz_n, parameters)
    fx_pure = magic_formula(kappa, peak, parameters.longitudinal)
    fy_pure = -magic_formula(alpha_rad, peak, parameters.lateral)
    norm = hypot(fx_pure, fy_pure)
    scale = min(1.0, peak / norm) if norm > 0.0 else 1.0
    fx = fx_pure * scale
    fy = fy_pure * scale
    return TyreResult(
        fx,
        fy,
        fx_pure,
        fy_pure,
        fz_n,
        peak,
        hypot(fx, fy) / peak if peak else 0.0,
        kappa,
        alpha_rad,
        scale < 1.0 - 1e-12,
    )
