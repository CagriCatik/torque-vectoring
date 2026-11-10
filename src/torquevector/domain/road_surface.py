"""Fahrbahnoberfläche als räumliches Reibwertfeld mu(x, y) – reine, validierte Datentypen.

Koordinaten: globales Simulationssystem (x vorwärts in Startrichtung, y links, Meter).
Regionen werden in Listenreihenfolge überlagert: spätere Regionen überschreiben frühere.
Rechteckgrenzen sind halboffen: x_min <= x < x_max und y_min <= y < y_max (deterministisch,
keine Doppelzuordnung an gemeinsamen Kanten). ``None`` als Grenze bedeutet unbegrenzt.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from typing import Protocol, runtime_checkable

MU_MIN = 0.05
MU_MAX = 2.0
FRICTION_MODELS = ('scale_tyre_mu', 'replace_tyre_mu')
TRANSITIONS = ('sharp', 'smooth')


@runtime_checkable
class RoadFrictionField(Protocol):
    """Kleine Physik-Schnittstelle: lokaler Fahrbahnreibwert und Oberflächenkennung."""

    def friction_at(self, x_m: float, y_m: float) -> float: ...

    def surface_at(self, x_m: float, y_m: float) -> str: ...


def _check_mu(value: float, what: str) -> None:
    if (
        not isinstance(value, (int, float))
        or isinstance(value, bool)
        or not isfinite(value)
        or not MU_MIN <= value <= MU_MAX
    ):
        raise ValueError(f'{what}: Reibwert muss endlich in [{MU_MIN}, {MU_MAX}] liegen, erhalten {value!r}')


@dataclass(frozen=True)
class SurfaceRegion:
    """Rechteckige Region (Längssegmente: y-Grenzen None, Halbebenen: einzelne Grenzen None)."""

    id: str
    material: str
    mu: float
    x_min_m: float | None
    x_max_m: float | None
    y_min_m: float | None
    y_max_m: float | None

    def __post_init__(self):
        _check_mu(self.mu, f'Region {self.id}')
        for lo, hi, axis in ((self.x_min_m, self.x_max_m, 'x'), (self.y_min_m, self.y_max_m, 'y')):
            for value in (lo, hi):
                if value is not None and not isfinite(value):
                    raise ValueError(f'Region {self.id}: {axis}-Grenze nicht endlich')
            if lo is not None and hi is not None and not lo < hi:
                raise ValueError(f'Region {self.id}: {axis}_min muss kleiner als {axis}_max sein')
        if not self.id or not self.material:
            raise ValueError('Region benötigt id und material')


@dataclass(frozen=True)
class RoadSurface:
    """Vollständige Fahrbahnbeschreibung (typisiertes Ergebnis der YAML-Konfiguration).

    ``friction_model`` legt fest, wie der lokale Fahrbahnreibwert in den Reifen eingeht:
    ``scale_tyre_mu``: mu_Reifen,eff = mu_Reifen * mu_Fahrbahn / reference_mu (Pacejka-lambda_mu);
    ``replace_tyre_mu``: mu_Reifen,eff = mu_Fahrbahn (Semantik der historischen FrictionMap).
    """

    id: str
    default_material: str
    default_mu: float
    regions: tuple[SurfaceRegion, ...]
    transition: str
    smoothing_distance_m: float
    friction_model: str
    reference_mu: float

    def __post_init__(self):
        _check_mu(self.default_mu, 'default_surface')
        _check_mu(self.reference_mu, 'reference_mu')
        if self.friction_model not in FRICTION_MODELS:
            raise ValueError(f'Unbekanntes Reibwertmodell {self.friction_model}')
        if self.transition not in TRANSITIONS:
            raise ValueError(f'Unbekannter Übergang {self.transition}')
        if not isfinite(self.smoothing_distance_m) or self.smoothing_distance_m < 0:
            raise ValueError('smoothing_distance_m muss endlich und >= 0 sein')
        if (self.transition == 'smooth') != (self.smoothing_distance_m > 0):
            raise ValueError('smooth erfordert smoothing_distance_m > 0, sharp erfordert 0')
        ids = [r.id for r in self.regions]
        if len(set(ids)) != len(ids):
            raise ValueError('Regions-IDs müssen eindeutig sein')

    @property
    def is_uniform(self) -> bool:
        return not self.regions
