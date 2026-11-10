"""Vollständig parametrisierter ZOH-Simulationslauf ohne eingebaute Modellparameter.

Alle Größen kommen über unveränderliche Setup-Objekte ohne Default-Werte. Die
Composition Root (``torquevector.cli.composition``) baut sie aus YAML; die historische
API ``simulation.runner.simulate_controller`` baut sie aus ihren bisherigen
Literalen und delegiert hierher, damit beide Wege dieselbe Orchestrierung nutzen.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from math import isfinite
from time import perf_counter
from typing import Callable

from torquevector.control.allocation import AllocationOptions
from torquevector.control.allocation_friction import FrictionLimit, make_allocator
from torquevector.control.controllers.lqr import LQRConfig
from torquevector.control.controllers.ltv_mpc import LTVConfig
from torquevector.control.controllers.nmpc import NMPCConfig, NMPCSolverSettings
from torquevector.control.factory import make_controller
from torquevector.control.pipeline import YawControlPipeline
from torquevector.control.sampling import SampledYawMomentPolicy
from torquevector.domain.controls import TorqueControls
from torquevector.domain.scenarios import ManoeuvreSpec, smoothstep
from torquevector.domain.tyre import TyreParameters
from torquevector.domain.vehicle import VehicleParameters
from torquevector.domain.wheels import WheelParameters
from torquevector.evaluation.metrics import compute_case
from torquevector.domain.states import VehicleState
from torquevector.physics.road_friction import (
    RoadFrictionMap,
    boundary_crossing_times,
    spatial_tyre_binding,
    wheel_contacts,
)
from torquevector.physics.steering import SteeringActuator
from torquevector.simulation.driving import ManoeuvrePolicy
from torquevector.simulation.policies import make_reference, policy_4wd
from torquevector.simulation.preview_adapter import make_maneuver_preview
from torquevector.control.slip_control import SlipControlSettings, SlipController, slip_control_metrics
from torquevector.evaluation.handling import handling_metrics
from torquevector.simulation.sampled import SampledSettings, simulate_sampled

LOG = logging.getLogger(__name__)


def _positive(*values):
    return all(isfinite(v) and v > 0 for v in values)


@dataclass(frozen=True)
class DriverSettings:
    max_drive_torque_nm: float
    step_steer_ramp_s: float
    reference_engage_ramp_s: float
    reference_friction_fraction: float
    preview_speed_time_constant_s: float
    reference_source: str

    def __post_init__(self):
        if not _positive(
            self.max_drive_torque_nm,
            self.step_steer_ramp_s,
            self.reference_engage_ramp_s,
            self.reference_friction_fraction,
            self.preview_speed_time_constant_s,
        ):
            raise ValueError('Invalid driver/reference settings')
        if self.reference_source not in ('path_curvature', 'driver_steering'):
            raise ValueError(f'Unknown yaw reference source {self.reference_source}')


@dataclass(frozen=True)
class BrakePhase:
    start_s: float
    ramp_s: float
    brake_nm: tuple[float, float, float, float]
    cut_drive: bool

    def __post_init__(self):
        if (
            not isfinite(self.start_s)
            or self.start_s < 0
            or not _positive(self.ramp_s)
            or len(self.brake_nm) != 4
            or not all(isfinite(b) and b >= 0 for b in self.brake_nm)
        ):
            raise ValueError('Invalid brake phase')


@dataclass(frozen=True)
class DrivePhase:
    """Antriebsphase: Radmomente werden per Smoothstep vom Fahrerwunsch auf feste Werte überblendet."""

    start_s: float
    ramp_s: float
    drive_nm: tuple[float, float, float, float]

    def __post_init__(self):
        if (
            not isfinite(self.start_s)
            or self.start_s < 0
            or not _positive(self.ramp_s)
            or len(self.drive_nm) != 4
            or not all(isfinite(v) and v >= 0 for v in self.drive_nm)
        ):
            raise ValueError('Invalid drive phase')


@dataclass(frozen=True)
class InitialPose:
    x_m: float
    y_m: float
    yaw_rad: float
    vy_ms: float
    yaw_rate_rads: float

    def as_tuple(self) -> tuple[float, float, float, float, float]:
        return (self.x_m, self.y_m, self.yaw_rad, self.vy_ms, self.yaw_rate_rads)


@dataclass(frozen=True)
class PlantSetup:
    car: VehicleParameters
    tyre: TyreParameters
    tyre_model: Callable
    wheel: WheelParameters
    actuator: SteeringActuator
    sampled: SampledSettings
    load_solver: str
    initial_controls: TorqueControls
    drive_distribution: tuple[float, float, float, float]
    slip_control: SlipControlSettings

    def __post_init__(self):
        if (
            len(self.drive_distribution) != 4
            or abs(sum(self.drive_distribution) - 1) > 1e-9
            or min(self.drive_distribution) < 0
        ):
            raise ValueError('Drive distribution must be four nonnegative shares summing to one')
        if self.load_solver not in ('root', 'fixed_point'):
            raise ValueError('Unknown load solver')


@dataclass(frozen=True)
class ControllerSetup:
    proportional_gain: float
    p_yaw_limit_nm: float
    p_reference_mu_source: str
    p_reference_mu_explicit: float | None
    lqr: LQRConfig
    lqr_yaw_limit_nm: float
    lqr_reference_mu_source: str
    lqr_reference_mu_explicit: float | None
    ltv: LTVConfig
    nmpc: NMPCConfig
    nmpc_solver: NMPCSolverSettings
    allocation: AllocationOptions
    ltv_friction_preview: str
    nmpc_friction_preview: str
    allocation_friction_mode: str
    allocation_reserve_fraction: float
    allocation_mu_source: str
    allocation_mu_explicit: float | None

    def __post_init__(self):
        for mode in (self.ltv_friction_preview, self.nmpc_friction_preview):
            if mode not in ('unknown', 'map_preview'):
                raise ValueError(f'Unknown friction knowledge mode {mode}')

    def friction_limit(self, tyre: TyreParameters) -> FrictionLimit | None:
        if self.allocation_friction_mode == 'off':
            return None
        if self.allocation_friction_mode != 'friction_aware':
            raise ValueError(f'Unknown allocation friction mode {self.allocation_friction_mode}')
        mu = {'tyre': tyre.mu, 'explicit': self.allocation_mu_explicit, 'road_map': None}[self.allocation_mu_source]
        return FrictionLimit(self.allocation_reserve_fraction, mu)

    def friction_preview(self, kind: str) -> bool:
        return kind in ('ltv', 'nmpc') and getattr(self, f'{kind}_friction_preview') == 'map_preview'

    def reference_mu(self, kind: str, tyre: TyreParameters) -> float:
        source = getattr(self, f'{kind}_reference_mu_source')
        if source == 'tyre':
            return tyre.mu
        if source == 'explicit':
            return getattr(self, f'{kind}_reference_mu_explicit')
        raise ValueError(f'Unknown reference mu source {source}')

    def yaw_limit(self, kind: str) -> float:
        return getattr(self, f'{kind}_yaw_limit_nm')


@dataclass(frozen=True)
class ScenarioSetup:
    spec: ManoeuvreSpec
    duration_s: float
    initial_state: InitialPose
    road_model: str
    road: Callable | None
    brake_phase: BrakePhase | None
    check_final_speed: bool
    surface: RoadFrictionMap | None
    initial_speed_ms: float
    drive_phase: DrivePhase | None

    def __post_init__(self):
        if self.road_model == 'friction_map':
            if self.road is None or (self.surface is not None and not self.surface.is_identity):
                raise ValueError('friction_map benötigt eine FrictionMap und eine neutrale Fahrbahnkarte')
        elif self.road_model == 'road_surface':
            if self.road is not None or self.surface is None:
                raise ValueError('road_surface benötigt eine Fahrbahnkarte und keine FrictionMap')
        else:
            raise ValueError(f'Unknown road model {self.road_model}')
        if not isfinite(self.initial_speed_ms) or self.initial_speed_ms <= 0:
            raise ValueError('Initial speed must be positive')


def brake_policy(base, phase: BrakePhase):
    """Bremsphase über der Fahrerpolicy; Smoothstep-Übergang wie im historischen V5-Grip-Test."""

    def policy(t, s, c):
        drive = base(t, s, c)
        z = smoothstep((t - phase.start_s) / phase.ramp_s)
        if z <= 0:
            return drive
        scale = (1 - z) if phase.cut_drive else 1.0
        return TorqueControls(
            drive.steer_deg,
            tuple(scale * v for v in drive.drive_nm),
            tuple(z * b for b in phase.brake_nm),
            drive.torque_limit_nm,
        )

    return policy


def drive_policy(base, phase: DrivePhase):
    def policy(t, s, c):
        drive = base(t, s, c)
        z = smoothstep((t - phase.start_s) / phase.ramp_s)
        if z <= 0:
            return drive
        return TorqueControls(
            drive.steer_deg,
            tuple((1 - z) * v + z * p for v, p in zip(drive.drive_nm, phase.drive_nm)),
            drive.brake_nm,
            drive.torque_limit_nm,
        )

    return policy


def friction_ratio_function(field: RoadFrictionMap, car: VehicleParameters, tyre: TyreParameters):
    """(x, y, psi) -> (r_vorne, r_hinten): Kartenreibwert an den Radpositionen einer vorhergesagten Pose,
    relativ zum nominalen Reifenreibwert; Achsmittel aus linkem und rechtem Rad."""

    def ratios(x, y, psi):
        contacts = wheel_contacts(field, VehicleState(x_m=x, y_m=y, yaw_rad=psi), car, tyre)
        r = {name: c.tyre_mu / tyre.mu for name, c in contacts.items()}
        return (r['FL'] + r['FR']) / 2, (r['RL'] + r['RR']) / 2

    return ratios


def build_policy(
    kind: str,
    scenario: ScenarioSetup,
    plant: PlantSetup,
    controllers: ControllerSetup,
    driver: DriverSettings,
    ticks: list | None = None,
):
    spec, car, tyre = scenario.spec, plant.car, plant.tyre
    manoeuvre = ManoeuvrePolicy(spec, car, plant.wheel.radius_m, driver.max_drive_torque_nm, driver.step_steer_ramp_s)
    base = policy_4wd(manoeuvre, plant.drive_distribution)
    if scenario.drive_phase is not None:
        base = drive_policy(base, scenario.drive_phase)
    if scenario.brake_phase is not None:
        base = brake_policy(base, scenario.brake_phase)
    if kind == 'off':
        return base
    limit = controllers.friction_limit(tyre)
    mu_at = None
    if limit is not None and limit.mu is None:
        if scenario.surface is None:
            raise ValueError('Allokation mit mu_source=road_map benötigt eine Fahrbahnkarte')
        field = scenario.surface
        mu_at = lambda state: tuple(c.tyre_mu for c in wheel_contacts(field, state, car, tyre).values())  # noqa: E731
    allocator = make_allocator(car, plant.wheel, controllers.allocation, limit, mu_at)
    if kind in ('p', 'lqr'):
        strategy = make_controller(
            kind, car, tyre, lqr_config=controllers.lqr, proportional_gain=controllers.proportional_gain
        )
        mode = 'path' if driver.reference_source == 'path_curvature' else 'steering'
        pipeline = YawControlPipeline(
            strategy,
            make_reference(spec, car, mode, driver.reference_engage_ramp_s),
            car,
            plant.wheel,
            controllers.allocation,
            reference_mu=controllers.reference_mu(kind, tyre),
            yaw_limit_nm=controllers.yaw_limit(kind),
            reference_friction_fraction=driver.reference_friction_fraction,
            diagnostics=ticks,
            allocator=allocator,
        )
        return lambda t, s, c: pipeline(t, s, base(t, s, c))
    if kind in ('ltv', 'nmpc'):
        knows_map = controllers.friction_preview(kind)
        controller = make_controller(
            kind,
            car,
            tyre,
            ltv_config=controllers.ltv,
            nmpc_config=controllers.nmpc,
            nmpc_solver=controllers.nmpc_solver,
            friction_preview=knows_map,
        )
        ratios = None
        if knows_map:
            if scenario.surface is None:
                raise ValueError('map_preview benötigt eine räumliche Fahrbahnkarte (road_surface)')
            ratios = friction_ratio_function(scenario.surface, car, tyre)
        preview = make_maneuver_preview(
            spec,
            car,
            base_policy=manoeuvre,
            speed_time_constant_s=driver.preview_speed_time_constant_s,
            engage_ramp_s=driver.reference_engage_ramp_s,
            friction_fraction=driver.reference_friction_fraction,
            friction_ratios=ratios,
            reference_source=driver.reference_source,
        )
        return SampledYawMomentPolicy(controller, base, preview, plant.wheel, controllers.allocation, allocator)
    raise ValueError(f'Unknown controller {kind}')


def run_case(
    kind: str,
    scenario: ScenarioSetup,
    plant: PlantSetup,
    controllers: ControllerSetup,
    driver: DriverSettings,
    *,
    simulator=simulate_sampled,
    feedback_diagnostics: bool = True,
):
    """Ein Regler, ein Szenario: Kennzahlen, Telemetrie und Taktdiagnostik.

    ``simulator`` ist eine Testnaht (Signatur von ``simulate_sampled``), kein Konfigurationsparameter;
    ``feedback_diagnostics`` zeichnet für P/LQR angefordertes und allokiertes Moment je Takt auf.
    """
    if kind in ('ltv', 'nmpc'):
        period = getattr(controllers, kind).sample_s
        if abs(period - plant.sampled.period_s) > 1e-9:
            raise ValueError('All sampled controllers must share control period')
    feedback_ticks = [] if kind in ('p', 'lqr') and feedback_diagnostics else None
    policy = build_policy(kind, scenario, plant, controllers, driver, feedback_ticks)
    LOG.debug('Start %s: %s, %.2f m/s, %.2f s', kind, scenario.spec.kind, scenario.spec.speed_ms, scenario.duration_s)
    started = perf_counter()
    tyre_model = scenario.road if scenario.road is not None else plant.tyre_model
    extra = {}
    if scenario.surface is not None and not scenario.surface.is_identity:
        extra['tyre_model_at'] = spatial_tyre_binding(scenario.surface, plant.car, plant.tyre, plant.tyre_model)
        extra['event_times'] = lambda state, dt: boundary_crossing_times(scenario.surface, state, plant.car, dt)
    slip = SlipController(plant.slip_control) if plant.slip_control.active else None
    if slip is not None:
        extra['slip_controller'], extra['slip_period_s'] = slip, plant.slip_control.period_s
    seq = simulator(
        plant.car,
        plant.tyre,
        plant.initial_controls,
        plant.wheel,
        speed_ms=scenario.initial_speed_ms,
        duration_s=scenario.duration_s,
        policy=policy,
        tyre_model=tyre_model,
        actuator=plant.actuator,
        settings=plant.sampled,
        load_solver=plant.load_solver,
        initial_pose=scenario.initial_state.as_tuple(),
        **extra,
    )
    metric = compute_case(scenario.spec, seq)
    metric['max_abs_slip'] = max(abs(k) for s in seq for k in s.kappa)
    metric['max_wheel_torque_nm'] = max(abs(z) for s in seq for z in s.drive_nm)
    metric.update(handling_metrics(seq, plant.car))
    if slip is not None:
        metric['slip_control'] = slip_control_metrics(slip.log)
    diagnostics = list(feedback_ticks or [])
    if kind in ('ltv', 'nmpc'):
        diagnostics = list(policy.diagnostics)
        metric['control_ticks'] = policy.calls
        metric['predictive_state_excess'] = policy.max_state_excess
        metric['maximum_allocation_residual_nm'] = policy.max_allocation_residual_nm
    LOG.debug(
        'Ende %s: %d Samples, %.2f s Rechenzeit, yaw_rmse=%.5f rad/s',
        kind,
        len(seq),
        perf_counter() - started,
        metric['yaw_rmse_rads'],
    )
    return metric, seq, diagnostics
