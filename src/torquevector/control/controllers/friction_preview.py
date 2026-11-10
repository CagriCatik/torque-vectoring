"""LTV-MPC mit kartenbasierter Reibwertvorschau (Modus ``map_preview``), per Komposition.

``LTVMPC.solve`` ist AST-eingefroren und kennt nur einen Reibwert. Der Adapter plant deshalb je
Regeltakt konservativ mit dem kleinsten vorhergesagten Reibwertverhältnis r_min über Horizont und
Achsen: Linearisierung mit mu_Reifen*r_min (Schräglaufsteifigkeit B*C*D skaliert mit D) und
Zustandsgrenzen mit mu_reference*r_min. Ohne Vorschau im Preview wird der Fehler offen gemeldet.
"""

from __future__ import annotations

from dataclasses import replace

from torquevector.control.controllers.ltv_mpc import LTVConfig, LTVMPC
from torquevector.domain.tyre import TyreParameters
from torquevector.domain.vehicle import VehicleParameters
from torquevector.prediction.bicycle_linear import BicycleLinearizer
from torquevector.prediction.interfaces import PredictionPreview

MIN_MU_REFERENCE = 0.1  # untere Gültigkeitsgrenze von LTVConfig.mu_reference


class FrictionScheduledLTV:
    def __init__(self, car: VehicleParameters, tyre: TyreParameters, config: LTVConfig):
        self.car = car
        self.tyre = tyre
        self.config = config
        self._nominal = LTVMPC(BicycleLinearizer(car, tyre), car, config)

    def solve(self, state, preview: PredictionPreview, previous_nm: float, period_s: float | None = None):
        if preview.front_friction_ratio is None:
            raise ValueError('LTV map_preview benötigt eine Reibwertvorschau im Preview')
        ratio = min(min(preview.front_friction_ratio), min(preview.rear_friction_ratio))
        if ratio == 1.0:
            return self._nominal.solve(state, preview, previous_nm, period_s)
        tyre = replace(self.tyre, mu=self.tyre.mu * ratio)
        config = replace(self.config, mu_reference=max(MIN_MU_REFERENCE, self.config.mu_reference * ratio))
        return LTVMPC(BicycleLinearizer(self.car, tyre), self.car, config).solve(state, preview, previous_nm, period_s)
