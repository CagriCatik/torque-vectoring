"""Räumliche Reibwertkarte und ortsabhängiges Reifenmodell (eine Reibwertquelle je Rad).

Die Karte ist eine reine Funktion der Position (keine Zustände, keine Seiteneffekte).
``SpatialFrictionTyre`` wird für genau einen Fahrzeugzustand gebildet und liefert je Rad
die Reifenkraft mit dem lokalen Reibwert am Aufstandspunkt. Der Reibwert geht ausschließlich
über ``TyreParameters.mu`` in den Peak-Faktor D = mu(Fz)*Fz ein; Längs-, Querkraft und die
Combined-Slip-Begrenzung des jeweiligen Reifenmodells skalieren damit konsistent und genau
einmal (keine zweite Sättigung). Reifenparameter werden nicht verändert, sondern kopiert.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from math import cos, sin
from typing import Callable

from torquevector.domain.road_surface import RoadSurface, SurfaceRegion
from torquevector.domain.states import VehicleState
from torquevector.domain.tyre import TyreParameters
from torquevector.physics.road import FrictionMap
from torquevector.domain.vehicle import VehicleParameters
from torquevector.physics.wheel_contact import wheel_contact_positions


def _smoothstep(z: float) -> float:
    z = min(1.0, max(0.0, z))
    return z * z * z * (z * (6 * z - 15) + 10)


def _axis_membership(value: float, lo: float | None, hi: float | None, width: float) -> float:
    """1 innerhalb [lo, hi), 0 außerhalb; bei width > 0 Smoothstep-Übergang zentriert auf die Grenze."""
    if width <= 0:
        return float((lo is None or value >= lo) and (hi is None or value < hi))
    rise = 1.0 if lo is None else _smoothstep((value - (lo - width / 2)) / width)
    fall = 1.0 if hi is None else 1.0 - _smoothstep((value - (hi - width / 2)) / width)
    return rise * fall


@dataclass(frozen=True)
class RoadFrictionMap:
    """Implementierung von ``RoadFrictionField`` für eine ``RoadSurface``."""

    surface: RoadSurface

    def _width(self) -> float:
        return self.surface.smoothing_distance_m if self.surface.transition == 'smooth' else 0.0

    def membership(self, region: SurfaceRegion, x_m: float, y_m: float) -> float:
        w = self._width()
        return _axis_membership(x_m, region.x_min_m, region.x_max_m, w) * _axis_membership(
            y_m, region.y_min_m, region.y_max_m, w
        )

    def friction_at(self, x_m: float, y_m: float) -> float:
        mu = self.surface.default_mu
        for region in self.surface.regions:  # Listenreihenfolge = Priorität (spätere gewinnen)
            weight = self.membership(region, x_m, y_m)
            if weight:
                mu = (1.0 - weight) * mu + weight * region.mu
        return mu

    def surface_at(self, x_m: float, y_m: float) -> str:
        surface = self.surface.default_material
        for region in self.surface.regions:
            if self.membership(region, x_m, y_m) >= 0.5:
                surface = region.id
        return surface

    def tyre_mu(self, tyre: TyreParameters, road_mu: float) -> float:
        """Effektiver Reifenreibwert für den lokalen Fahrbahnreibwert."""
        if self.surface.friction_model == 'replace_tyre_mu':
            return road_mu
        return tyre.mu * road_mu / self.surface.reference_mu

    @property
    def is_identity(self) -> bool:
        """Gleichförmige Fahrbahn, die den Reifenreibwert exakt unverändert lässt."""
        return (
            self.surface.is_uniform
            and self.surface.friction_model == 'scale_tyre_mu'
            and self.surface.default_mu == self.surface.reference_mu
        )


@dataclass(frozen=True)
class WheelContact:
    """Lokale Fahrbahninformation eines Rades zu einem Fahrzeugzustand."""

    x_m: float
    y_m: float
    road_mu: float
    tyre_mu: float
    surface_id: str


def wheel_contacts(
    field: RoadFrictionMap, state: VehicleState, car: VehicleParameters, tyre: TyreParameters
) -> dict[str, WheelContact]:
    contacts = {}
    for name, (x, y) in wheel_contact_positions(state, car).items():
        road = field.friction_at(x, y)
        contacts[name] = WheelContact(x, y, road, field.tyre_mu(tyre, road), field.surface_at(x, y))
    return contacts


@dataclass(frozen=True)
class SpatialFrictionTyre:
    """Reifenmodell mit lokalem Reibwert je Rad für genau einen Fahrzeugzustand.

    Wird von ``physics.four_wheel.evaluate`` über die Schnittstelle ``at_wheel`` genutzt.
    ``base_model(kappa, alpha, fz, tyre)`` ist das konfigurierte Reifenmodell.
    """

    base_model: Callable
    contacts: dict

    def at_wheel(self, name: str, t: float, kappa: float, alpha: float, fz: float, base: TyreParameters):
        local = replace(base, mu=self.contacts[name].tyre_mu)
        if hasattr(self.base_model, 'at_wheel'):  # wheel-dependent base model, e.g. per-axle tyre scaling
            return self.base_model.at_wheel(name, t, kappa, alpha, fz, local)
        return self.base_model(kappa, alpha, fz, local)


def boundary_crossing_times(
    field: RoadFrictionMap, state: VehicleState, car: VehicleParameters, horizon_s: float
) -> list[float]:
    """Vorhergesagte Zeitpunkte in (0, horizon_s), zu denen ein Aufstandspunkt eine scharfe Regionsgrenze
    überfährt (lineare Extrapolation der Kontaktpunktgeschwindigkeit aus dem aktuellen Zustand).

    Dient der Ereignisbehandlung: Die Simulation startet den Integrator an diesen Zeitpunkten neu,
    damit keine Runge-Kutta-Zwischenstufe über die Unstetigkeit hinweg ausgewertet wird. Bei glatten
    Übergängen ist die rechte Seite stetig; dann werden keine Zeitpunkte geliefert.
    """
    if field.surface.transition != 'sharp' or not field.surface.regions:
        return []
    c, s = cos(state.yaw_rad), sin(state.yaw_rad)
    vx_world = state.vx_ms * c - state.vy_ms * s
    vy_world = state.vx_ms * s + state.vy_ms * c
    edges_x = {v for r in field.surface.regions for v in (r.x_min_m, r.x_max_m) if v is not None}
    edges_y = {v for r in field.surface.regions for v in (r.y_min_m, r.y_max_m) if v is not None}
    times = set()
    for (x, y), (lx, ly) in zip(wheel_contact_positions(state, car).values(), car.contact_points().values()):
        # Kontaktpunktgeschwindigkeit = v_CG + r x omega (in Weltkoordinaten)
        rx, ry = c * lx - s * ly, s * lx + c * ly
        vx, vy = vx_world - state.yaw_rate_rads * ry, vy_world + state.yaw_rate_rads * rx
        for pos, vel, edges in ((x, vx, edges_x), (y, vy, edges_y)):
            if abs(vel) < 1e-9:
                continue
            for edge in edges:
                t = (edge - pos) / vel
                if 1e-6 < t < horizon_s - 1e-6:
                    times.add(round(t, 9))
    return sorted(times)


def spatial_tyre_binding(
    field: RoadFrictionMap, car: VehicleParameters, tyre: TyreParameters, base_model: Callable
) -> Callable[[VehicleState], SpatialFrictionTyre]:
    """Liefert ``state -> Reifenmodell``; von der Simulation bei jeder ODE-Auswertung aufgerufen."""
    if isinstance(base_model, FrictionMap):
        raise ValueError('Räumliche Reibwertkarte und zeit-/seitenbasierte FrictionMap schließen sich aus')

    def bind(state: VehicleState) -> SpatialFrictionTyre:
        return SpatialFrictionTyre(base_model, wheel_contacts(field, state, car, tyre))

    return bind
