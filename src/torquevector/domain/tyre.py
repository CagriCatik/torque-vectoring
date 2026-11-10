"""Tyre parameter types (Magic Formula B/C/E per axis, friction, load sensitivity) and tyre result record."""

from __future__ import annotations
from dataclasses import dataclass


@dataclass(frozen=True)
class MagicFormulaAxis:
    b: float = 11.0
    c: float = 1.65
    e: float = 0.45

    def __post_init__(self):
        if not (0 < self.b <= 200 and 0 < self.c <= 3 and -5 <= self.e <= 1):
            raise ValueError('Invalid demonstrator Magic Formula shape parameters')


@dataclass(frozen=True)
class TyreParameters:
    longitudinal: MagicFormulaAxis = MagicFormulaAxis(11.0, 1.65, 0.45)
    lateral: MagicFormulaAxis = MagicFormulaAxis(7.0, 1.45, 0.35)
    mu: float = 0.95
    nominal_fz_n: float = 4000.0
    load_exponent: float = -0.06

    def __post_init__(self):
        if not (0.05 <= self.mu <= 2 and self.nominal_fz_n > 0):
            raise ValueError('mu and nominal Fz must be positive and within demo limits')
        if not (-0.3 <= self.load_exponent <= 0.0):
            raise ValueError('load_exponent must be in [-0.3, 0]')


@dataclass(frozen=True)
class TyreResult:
    fx_n: float
    fy_n: float
    fx_pure_n: float
    fy_pure_n: float
    fz_n: float
    peak_n: float
    utilization: float
    slip_ratio: float
    slip_angle_rad: float
    clipped: bool
