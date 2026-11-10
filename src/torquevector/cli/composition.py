"""Composition Root: validierter YAML-Baum -> typisierte, unveränderliche Laufzeitobjekte.

Einzige Stelle, an der aufgelöste Konfigurationswerte auf Fachklassen abgebildet
werden. Jedes Feld der gebundenen Dataclasses wird explizit gesetzt; Defaults der
Fachklassen (historische Kompatibilität, teils AST-eingefroren) werden hier nie
verwendet. Die Abbildung ist im Schema über ``binds`` dokumentiert und wird durch
``tests/architecture/test_configuration_bindings.py`` vollständig geprüft.
"""

from __future__ import annotations

import json
import logging
from math import atan2, degrees, radians
import os
import time
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

from torquevector.control.allocation import AllocationOptions
from torquevector.evaluation.handling import HandlingSettings, PhasePlaneSettings
from torquevector.infrastructure.config.errors import ConfigurationError
from torquevector.physics.axle_tyre import AxleScale, TyreAxleScaling, scaled_tyre_model
from torquevector.domain.road_surface import MU_MAX, MU_MIN
from torquevector.control.controllers.lqr import LQRConfig
from torquevector.control.controllers.ltv_mpc import LTVConfig
from torquevector.control.controllers.nmpc import NMPCConfig, NMPCSolverSettings
from torquevector.domain.controls import TorqueControls
from torquevector.domain.scenarios import ManoeuvreSpec
from torquevector.domain.tyre import MagicFormulaAxis, TyreParameters
from torquevector.domain.vehicle import VehicleParameters
from torquevector.domain.wheels import WHEEL_ORDER, WheelParameters
from torquevector.evaluation.comparisons import annotate_relative, assess_run
from torquevector.evaluation.envelope import EnvelopeThresholds, QualificationCriteria
from torquevector.infrastructure.telemetry_io import export_torque_csv
from torquevector.infrastructure.config.resolver import (
    ResolvedConfiguration,
    matrix_cases,
    resolve,
    resolve_case,
    resolve_study,
    with_overrides,
)
from torquevector.optimization.configured_tuning import (
    ObjectiveSpec,
    SearchDimension,
    TuningCase,
    TuningStudy,
    run_study,
)
from torquevector.optimization.pso import PSOSettings
from torquevector.physics.combined_slip import CombinedSlipShape, CombinedSlipTyre
from torquevector.domain.road_surface import RoadSurface, SurfaceRegion
from torquevector.physics.road import FrictionMap
from torquevector.physics.road_friction import RoadFrictionMap
from torquevector.physics.steering import SteeringActuator
from torquevector.physics.tyre_magic_formula import tyre_forces
from torquevector.control.slip_control import SlipControlSettings
from torquevector.simulation.configured import (
    BrakePhase,
    ControllerSetup,
    DrivePhase,
    DriverSettings,
    InitialPose,
    PlantSetup,
    ScenarioSetup,
    run_case,
)
from torquevector.simulation.sampled import SampledSettings, simulate_sampled
from torquevector.visualization.animation import AnimationConfig
from torquevector.visualization.comparison import ChartStyle

LOG = logging.getLogger(__name__)
CONTROLLER_SLOTS = {'p': 'p', 'lqr': 'lqr', 'ltv': 'ltv_mpc', 'nmpc': 'nmpc'}


def wheel_tuple(mapping) -> tuple[float, float, float, float]:
    return tuple(float(mapping[name]) for name in WHEEL_ORDER)


@dataclass(frozen=True)
class LivePlotSettings:
    enabled: bool
    figure_size_in: tuple[float, float]
    dt_s: float


@dataclass(frozen=True)
class VisualizationSettings:
    live: LivePlotSettings
    chart: ChartStyle
    chart_file_stem: str
    animation: AnimationConfig
    animation_file_name: str
    auto_report: bool
    report_sections: tuple[str, ...]


@dataclass(frozen=True)
class ValidationSettings:
    envelope: EnvelopeThresholds
    qualification: QualificationCriteria
    handling: HandlingSettings


@dataclass(frozen=True)
class RunSettings:
    controllers: tuple[str, ...]
    matrix_set: str | None
    results_dir: str
    store_controller_diagnostics: bool


@dataclass(frozen=True)
class RuntimeConfiguration:
    fingerprint: str
    vehicle: VehicleParameters
    tyre: TyreParameters
    combined_slip_model: str
    combined_slip: CombinedSlipShape | None
    tyre_axle_scaling: TyreAxleScaling
    tyre_model: object
    wheel: WheelParameters
    plant: PlantSetup
    controllers: ControllerSetup
    driver: DriverSettings
    scenario: ScenarioSetup
    visualization: VisualizationSettings
    validation: ValidationSettings
    run: RunSettings


def build_vehicle(v) -> VehicleParameters:
    g = v['geometry']
    return VehicleParameters(
        mass_kg=v['mass_kg'],
        iz_kgm2=v['inertia']['yaw_kgm2'],
        wheelbase_m=g['wheelbase_m'],
        cg_to_front_m=g['cg_to_front_axle_m'],
        track_m=g['track_m'],
        cg_height_m=g['cg_height_m'],
        cd_area_m2=v['aerodynamics']['drag_area_m2'],
        rolling_coefficient=v['rolling_resistance']['coefficient'],
        air_density_kgm3=v['aerodynamics']['air_density_kgm3'],
        gravity=v['environment']['gravity_ms2'],
        load_transfer=v['load_transfer']['enabled'],
    )


def build_axle_scaling(t) -> TyreAxleScaling:
    a = t['axle_scaling']
    return TyreAxleScaling(**{axle: AxleScale(**a[axle]) for axle in ('front', 'rear')})


def build_tyre(t):
    def axis(a):
        return MagicFormulaAxis(b=a['B'], c=a['C'], e=a['E'])

    f = t['friction']
    tyre = TyreParameters(
        longitudinal=axis(t['longitudinal']),
        lateral=axis(t['lateral']),
        mu=f['mu'],
        nominal_fz_n=f['reference_load_n'],
        load_exponent=f['load_sensitivity_exponent'],
    )
    cs = t['combined_slip']
    if cs['model'] == 'elliptic_damping_surrogate':
        shape = CombinedSlipShape(
            alpha_scale_rad=cs['alpha_scale_rad'],
            kappa_scale=cs['kappa_scale'],
            lateral_peak_fraction=cs['lateral_peak_fraction'],
            longitudinal_peak_fraction=cs['longitudinal_peak_fraction'],
        )
        return tyre, cs['model'], shape, CombinedSlipTyre(shape=shape)
    return tyre, cs['model'], None, tyre_forces


def build_predictive(c, cls):
    w, lim, val = c['weights'], c['limits'], c['model_validity']
    common = dict(
        sample_s=c['sample_s'],
        horizon_steps=c['horizon_steps'],
        beta_scale_rad=w['beta_scale_rad'],
        yaw_scale_rads=w['yaw_scale_rads'],
        moment_scale_nm=w['moment_scale_nm'],
        moment_limit_nm=lim['moment_limit_nm'],
        moment_slew_nm_s=lim['moment_slew_nm_s'],
        input_weight=w['input_weight'],
        slew_weight=w['slew_weight'],
        terminal_weight=w['terminal_weight'],
        state_soft_weight=w['state_soft_weight'],
        beta_limit_rad=lim['beta_limit_rad'],
        mu_reference=lim['mu_reference'],
        yaw_limit_fraction=lim['yaw_limit_fraction'],
        min_speed_ms=val['min_speed_ms'],
        max_speed_ms=val['max_speed_ms'],
        max_iterations=c['solver']['max_iterations'],
    )
    if cls is NMPCConfig:
        common['tolerance'] = c['solver']['tolerance']
    return cls(**common)


def build_controllers(tree) -> ControllerSetup:
    c = tree['controllers']
    p, lqr = c['p'], c['lqr']
    s = c['nmpc']['solver']
    a = tree['allocation']
    return ControllerSetup(
        proportional_gain=p['gain_nm_per_rads'],
        p_yaw_limit_nm=p['output']['yaw_moment_limit_nm'],
        p_reference_mu_source=p['yaw_reference']['mu_source'],
        p_reference_mu_explicit=p['yaw_reference']['mu'],
        lqr=LQRConfig(
            sideslip_scale_rad=lqr['weights']['sideslip_scale_rad'],
            yaw_rate_scale_rads=lqr['weights']['yaw_rate_scale_rads'],
            moment_scale_nm=lqr['weights']['moment_scale_nm'],
            min_speed_ms=lqr['scheduling']['min_speed_ms'],
            max_speed_ms=lqr['scheduling']['max_speed_ms'],
            speed_grid_ms=lqr['scheduling']['speed_grid_ms'],
            feedforward_weight=lqr['feedforward_weight'],
        ),
        lqr_yaw_limit_nm=lqr['output']['yaw_moment_limit_nm'],
        lqr_reference_mu_source=lqr['yaw_reference']['mu_source'],
        lqr_reference_mu_explicit=lqr['yaw_reference']['mu'],
        ltv=build_predictive(c['ltv_mpc'], LTVConfig),
        nmpc=build_predictive(c['nmpc'], NMPCConfig),
        nmpc_solver=NMPCSolverSettings(
            print_level=s['print_level'],
            print_time=s['print_time'],
            acceptable_tol_floor=s['acceptable_tolerance_floor'],
            acceptable_tol_factor=s['acceptable_tolerance_factor'],
            suppress_banner=s['suppress_banner'],
            max_cpu_time_s=s['max_cpu_time_s'],
        ),
        allocation=AllocationOptions(
            yaw_weight=a['weights']['yaw_weight'],
            total_weight=a['weights']['total_weight'],
            intervention_penalty=a['weights']['intervention_penalty'],
            max_delta_nm=a['max_delta_nm'],
            mu_reference=a['legacy_four_wheel_controller']['mu_reference'],
            yaw_kp_nm_per_rads=a['legacy_four_wheel_controller']['yaw_kp_nm_per_rads'],
        ),
        allocation_friction_mode=a['friction_limit']['mode'],
        allocation_reserve_fraction=a['friction_limit']['reserve_fraction'],
        allocation_mu_source=a['friction_limit']['mu_source'],
        allocation_mu_explicit=a['friction_limit']['mu'],
        ltv_friction_preview=c['ltv_mpc']['friction_knowledge']['mode'],
        nmpc_friction_preview=c['nmpc']['friction_knowledge']['mode'],
    )


def build_road(r) -> RoadFrictionMap:
    """Fahrbahn-YAML -> RoadSurface; split_mu wird in zwei halboffene Rechtecke aufgelöst."""
    regions = []
    for item in r['regions']:
        b = item['bounds']
        if item['type'] == 'split_mu':
            regions.append(
                SurfaceRegion(
                    f"{item['id']}_left",
                    item['left']['material'],
                    item['left']['mu'],
                    b['x_min_m'],
                    b['x_max_m'],
                    item['split_y_m'],
                    None,
                )
            )
            regions.append(
                SurfaceRegion(
                    f"{item['id']}_right",
                    item['right']['material'],
                    item['right']['mu'],
                    b['x_min_m'],
                    b['x_max_m'],
                    None,
                    item['split_y_m'],
                )
            )
        else:
            regions.append(
                SurfaceRegion(
                    item['id'], item['material'], item['mu'], b['x_min_m'], b['x_max_m'], b['y_min_m'], b['y_max_m']
                )
            )
    return RoadFrictionMap(
        RoadSurface(
            id=r['id'],
            default_material=r['default_surface']['material'],
            default_mu=r['default_surface']['mu'],
            regions=tuple(regions),
            transition=r['transition']['mode'],
            smoothing_distance_m=r['transition']['smoothing_distance_m'],
            friction_model=r['friction_model']['mode'],
            reference_mu=r['friction_model']['reference_mu'],
        )
    )


def build_scenario(s, tyre_model, road=None) -> ScenarioSetup:
    m = s['manoeuvre']
    spec = ManoeuvreSpec(
        kind=m['kind'],
        speed_ms=m['speed_ms'],
        duration_s=m['duration_s'],
        radius_m=m['path']['radius_m'],
        offset_m=m['path']['offset_m'],
        entry_m=m['path']['entry_m'],
        transition_m=m['path']['transition_m'],
        plateau_m=m['path']['plateau_m'],
        steer_amplitude_deg=m['steering']['amplitude_deg'],
        steer_frequency_hz=m['steering']['frequency_hz'],
        excitation_start_s=m['steering']['excitation_start_s'],
        max_steer_deg=m['steering']['max_steer_deg'],
        cross_gain=m['path_following']['cross_gain'],
        heading_gain=m['path_following']['heading_gain'],
        speed_kp_nm_per_ms=m['speed_control']['kp_nm_per_ms'],
    )
    i = s['initial_state']
    road_map = road
    road = s['road']
    friction = None
    if road['model'] == 'friction_map':
        friction = FrictionMap(
            left_mu=road['left_mu'],
            right_mu=road['right_mu'],
            after_s=road['step_time_s'],
            after_mu=road['step_mu'],
            base_model=tyre_model,
        )
    drive = s.get('drive_phase')
    drive_phase = (
        None
        if drive is None
        else DrivePhase(
            start_s=drive['start_s'], ramp_s=drive['ramp_s'], drive_nm=wheel_tuple(drive['drive_torque_nm'])
        )
    )
    brake = s.get('brake_phase')
    phase = (
        None
        if brake is None
        else BrakePhase(
            start_s=brake['start_s'],
            ramp_s=brake['ramp_s'],
            brake_nm=wheel_tuple(brake['brake_torque_nm']),
            cut_drive=brake['cut_drive'],
        )
    )
    duration = m['duration_s'] if s['simulation_duration_s'] is None else s['simulation_duration_s']
    return ScenarioSetup(
        spec=spec,
        duration_s=duration,
        initial_state=InitialPose(
            x_m=i['x_m'], y_m=i['y_m'], yaw_rad=i['yaw_rad'], vy_ms=i['vy_ms'], yaw_rate_rads=i['yaw_rate_rads']
        ),
        road_model=road['model'],
        road=friction,
        brake_phase=phase,
        check_final_speed=s['envelope']['check_final_speed'],
        surface=road_map,
        initial_speed_ms=m['speed_ms'] if m.get('initial_speed_ms') is None else m['initial_speed_ms'],
        drive_phase=drive_phase,
    )


def visualization_from_tree(tree) -> VisualizationSettings:
    viz = tree['visualization']
    live, static, gif = viz['live_plots'], viz['static_plots'], viz['gif_export']
    return VisualizationSettings(
        live=LivePlotSettings(
            enabled=live['enabled'], figure_size_in=tuple(live['figure_size_in']), dt_s=live['integration_step_s']
        ),
        chart=ChartStyle(
            panels=tuple(static['panels']),
            figure_size_in=tuple(static['figure_size_in']),
            dpi=static['dpi'],
            formats=tuple(static['formats']),
            legend=static['legend'],
            grid_alpha=static['grid_alpha'],
            colors=None if static['colors'] is None else tuple(static['colors']),
            utilization_ylim_max=static['utilization_ylim_max'],
        ),
        chart_file_stem=static['file_stem'],
        animation=AnimationConfig(
            fps=gif['fps'],
            max_frames=gif['max_frames'],
            tail_seconds=gif['tail_s'],
            dpi=gif['dpi'],
            figure_size_in=tuple(gif['figure_size_in']),
            colors=None if gif['colors'] is None else tuple(gif['colors']),
        ),
        animation_file_name=gif['file_name'],
        auto_report=static['auto_report'],
        report_sections=tuple(static['report_sections']),
    )


def build_runtime(resolved: ResolvedConfiguration) -> RuntimeConfiguration:
    tree = resolved.tree
    vehicle = build_vehicle(tree['vehicle'])
    tyre, cs_model, shape, base_tyre_model = build_tyre(tree['tyre'])
    axle_scaling = build_axle_scaling(tree['tyre'])
    if not axle_scaling.is_neutral and tree['scenario']['road']['model'] == 'friction_map':
        raise ConfigurationError(
            'tyre.axle_scaling ist mit scenario.road.model friction_map (zeitbasierte Legacy-Karte) nicht kombinierbar; '
            'road_surface verwenden'
        )
    tyre_model = scaled_tyre_model(base_tyre_model, axle_scaling)
    w = tree['wheel']
    wheel = WheelParameters(
        radius_m=w['effective_radius_m'],
        inertia_kgm2=w['rotational_inertia_kgm2'],
        min_contact_speed_ms=w['min_contact_speed_ms'],
        max_abs_slip=tree['tyre']['operating_limits']['max_abs_longitudinal_slip'],
    )
    pt = tree['powertrain']
    st = pt['steering_actuator']
    sim = tree['simulation']
    integ, slew = sim['integrator'], sim['limits']['command_slew']
    plant = PlantSetup(
        car=vehicle,
        tyre=tyre,
        tyre_model=tyre_model,
        wheel=wheel,
        actuator=SteeringActuator(
            time_constant_s=st['time_constant_s'],
            rate_limit_deg_s=st['rate_limit_deg_s'],
            max_angle_deg=st['max_angle_deg'],
        ),
        sampled=SampledSettings(
            period_s=sim['sampling']['control_period_s'],
            max_torque_rate_nm_s=slew['wheel_torque_rate_nm_s'],
            max_steer_rate_command_deg_s=slew['steer_command_rate_deg_s'],
            max_step_s=integ['max_step_s'],
            rtol=integ['rtol'],
            max_rhs_per_period=integ['max_rhs_calls_per_period'],
            atol=integ['atol'],
            method=integ['method'],
        ),
        load_solver=integ['load_transfer_solver'],
        initial_controls=TorqueControls(
            steer_deg=pt['initial_state']['steer_command_deg'],
            drive_nm=wheel_tuple(pt['initial_state']['drive_torque_nm']),
            brake_nm=wheel_tuple(pt['initial_state']['brake_torque_nm']),
            torque_limit_nm=pt['wheel_torque_limit_nm'],
        ),
        drive_distribution=wheel_tuple(pt['drive_distribution']),
        slip_control=SlipControlSettings(**pt['slip_control']),
    )
    d = sim['driver']
    driver = DriverSettings(
        max_drive_torque_nm=d['base_policy']['max_base_wheel_torque_nm'],
        step_steer_ramp_s=d['base_policy']['step_steer_ramp_s'],
        reference_engage_ramp_s=d['yaw_reference']['engage_ramp_s'],
        reference_friction_fraction=d['yaw_reference']['friction_fraction'],
        preview_speed_time_constant_s=d['preview']['speed_time_constant_s'],
        reference_source=d['yaw_reference']['source'],
    )
    visualization = visualization_from_tree(tree)
    th = tree['validation']['thresholds']
    h = th['handling']
    validation = ValidationSettings(
        envelope=EnvelopeThresholds(**th['envelope']),
        qualification=QualificationCriteria(**th['qualification']),
        handling=HandlingSettings(
            **{k: tuple(v) if isinstance(v, (list, tuple)) else v for k, v in h.items() if k != 'phase_plane'},
            phase_plane=PhasePlaneSettings(**h['phase_plane']),
        ),
    )
    run = tree['run']
    return RuntimeConfiguration(
        fingerprint=resolved.fingerprint,
        vehicle=vehicle,
        tyre=tyre,
        combined_slip_model=cs_model,
        combined_slip=shape,
        tyre_axle_scaling=axle_scaling,
        tyre_model=tyre_model,
        wheel=wheel,
        plant=plant,
        controllers=build_controllers(tree),
        driver=driver,
        scenario=build_scenario(tree['scenario'], tyre_model, build_road(tree['road'])),
        visualization=visualization,
        validation=validation,
        run=RunSettings(
            controllers=tuple(run['controllers']),
            matrix_set=run['matrix_set'],
            results_dir=run['results_dir'],
            store_controller_diagnostics=run['telemetry']['store_controller_diagnostics'],
        ),
    )


def evaluate_case(runtime: RuntimeConfiguration, kind: str):
    """Ein Regler auf dem konfigurierten Szenario mit konfigurierter Gültigkeitsprüfung."""
    s = runtime.scenario
    return assess_run(
        lambda: run_case(kind, s, runtime.plant, runtime.controllers, runtime.driver),
        s.spec.speed_ms,
        runtime.validation.envelope,
        s.check_final_speed,
    )


def results_root(cli_value, runtime: RuntimeConfiguration | None = None) -> Path:
    """Priorität: CLI --results-dir > TORQUEVECTOR_RESULTS_DIR > run.results_dir (relativ zum Arbeitsverzeichnis)."""
    value = (
        cli_value or os.environ.get('TORQUEVECTOR_RESULTS_DIR') or (runtime.run.results_dir if runtime else 'results')
    )
    return Path(value).expanduser().resolve()


def write_snapshot(directory: Path, resolved: ResolvedConfiguration, stem='resolved_config'):
    directory.mkdir(parents=True, exist_ok=True)
    (directory / f'{stem}.yaml').write_text(resolved.export_yaml(), encoding='utf-8')
    (directory / f'{stem}.origins.json').write_text(resolved.origins_json(), encoding='utf-8')
    (directory / f'{stem}.sources.json').write_text(resolved.sources_json(), encoding='utf-8')
    (directory / f'{stem}.fingerprint').write_text(resolved.fingerprint + '\n', encoding='utf-8')


def _handling_series(seq, car: VehicleParameters) -> dict:
    columns = handling_columns(seq, car)
    return {
        'understeer_index_rads': columns['understeer_index_rads'],
        'front': columns['front_axle_utilisation'],
        'rear': columns['rear_axle_utilisation'],
    }


def handling_columns(seq, car: VehicleParameters) -> dict:
    """Per-sample handling indicators for the CSV (evaluation.handling)."""
    from torquevector.evaluation.handling import axle_utilisation, understeer_index

    axles = [axle_utilisation(s) for s in seq]
    return {
        'understeer_index_rads': [understeer_index(s, car) for s in seq],
        'front_axle_utilisation': [a[0] for a in axles],
        'rear_axle_utilisation': [a[1] for a in axles],
    }


def validity_envelope(runtime: RuntimeConfiguration) -> dict:
    """Gültigkeitsbereich des Laufs (Q01.3/Q01.5): Grenzen, außerhalb derer der Lauf FAIL_CLOSED endet."""
    c = runtime.controllers
    return {
        'wheel_contact_speed_ms': {'min': runtime.wheel.min_contact_speed_ms, 'reverse': 'not_modelled'},
        'abs_slip_ratio_max': runtime.wheel.max_abs_slip,
        'tyre_mu': runtime.tyre.mu,
        'road_mu_range': [MU_MIN, MU_MAX],
        'wheel_load_n': {'min_exclusive': 0.0, 'wheel_lift': 'fail_closed'},
        'temperature': 'not_modelled',
        'inflation_pressure': 'not_modelled',
        'predictive_controller_speed_ms': {
            'ltv': [c.ltv.min_speed_ms, c.ltv.max_speed_ms],
            'nmpc': [c.nmpc.min_speed_ms, c.nmpc.max_speed_ms],
        },
        'source_kind': 'SYNTHETIC_SIMULATION_ONLY',
    }


def run_configured(
    resolved: ResolvedConfiguration,
    *,
    results_dir=None,
    controllers=None,
    echo=print,
    report: bool | None = None,
    run_name: str | None = None,
) -> int:
    """Einzelszenario oder Matrixmenge ausführen; Snapshot, Telemetrie, Log und optional Bericht speichern.

    Ergebnisordner: ``<zeit>_<szenario oder matrixmenge>`` unter der Ergebniswurzel, oder exakt ``run_name``
    (für Läufe innerhalb einer Sammlung wie multi-mu oder q11).
    """
    from torquevector.infrastructure.experiment_store import ExperimentStore
    from torquevector.infrastructure.logging_setup import run_log
    from torquevector.infrastructure.telemetry_store import write_telemetry

    runtime = build_runtime(resolved)
    kinds = tuple(controllers or runtime.run.controllers)
    if runtime.run.matrix_set:
        cases = [(case['id'], resolve_case(resolved, case)) for case in matrix_cases(resolved, runtime.run.matrix_set)]
    else:
        cases = [(None, resolved)]
    runtimes = {case_id: (case_resolved, build_runtime(case_resolved)) for case_id, case_resolved in cases}
    first = next(iter(runtimes.values()))[1]
    configuration = {
        'source': 'yaml',
        'fingerprint': resolved.fingerprint,
        'application': str(resolved.application_path),
        'layers': list(resolved.layers),
        'controllers': list(kinds),
        'scenario': asdict(first.scenario.spec),
        'duration_s': first.scenario.duration_s,
        'matrix_set': runtime.run.matrix_set,
        'cases': {
            cid: {
                'scenario': asdict(rt.scenario.spec),
                'duration_s': rt.scenario.duration_s,
                'fingerprint': rt.fingerprint,
            }
            for cid, (_, rt) in runtimes.items()
            if cid
        },
        'warnings': list(resolved.warnings),
        'parameter_files': json.loads(resolved.sources_json()),
        'validity_envelope': validity_envelope(first),
    }
    label = runtime.run.matrix_set or resolved.tree['scenario']['id']
    store = ExperimentStore(results_root(results_dir, runtime), configuration, label=label, name=run_name)
    write_snapshot(store.path / 'config', resolved)
    started = time.perf_counter()
    status = 'FAIL'
    with run_log(store.path / 'logs'):
        LOG.info(
            'Lauf %s: %d Fall/Fälle, Regler %s, Ergebnisse %s',
            store.run_id,
            len(runtimes),
            ', '.join(kinds),
            store.path,
        )
        LOG.info('Konfiguration %s (Fingerprint %s)', resolved.application_path, resolved.fingerprint[:16])
        for warning in resolved.warnings:
            LOG.warning(warning)
        try:
            matrix_records = {}
            for case_id, (case_resolved, case_runtime) in runtimes.items():
                if case_id:
                    write_snapshot(store.path / 'config' / 'cases', case_resolved, stem=case_id)
                spec = case_runtime.scenario.spec
                LOG.info(
                    'Fall %s: %s bei %.1f m/s über %.2f s',
                    case_id or '-',
                    spec.kind,
                    spec.speed_ms,
                    case_runtime.scenario.duration_s,
                )
                records, sequences, diagnostics, road_contacts, tick_records = {}, {}, {}, {}, {}
                for kind in kinds:
                    tick = time.perf_counter()
                    entry, seq, ticks = evaluate_case(case_runtime, kind)
                    records[kind] = entry
                    detail = (
                        f"yaw_rmse={entry['metrics']['yaw_rmse_rads']:.5f} rad/s"
                        if 'metrics' in entry
                        else entry.get('reason', '')
                    )
                    log = LOG.info if entry['status'] == 'PASS' else LOG.warning
                    log('  %-5s %-22s %6.1f s  %s', kind.upper(), entry['status'], time.perf_counter() - tick, detail)
                    if seq is not None:
                        sequences[kind] = seq
                        road_contacts[kind] = road_contact_rows(case_runtime, seq)
                        entry['metrics']['road'] = road_summary(case_runtime, seq, road_contacts[kind], ticks)
                    tick_records[kind] = ticks
                    if ticks and case_runtime.run.store_controller_diagnostics:
                        diagnostics[kind] = ticks
                if spec.path is not None:
                    annotate_relative(records)
                telemetry = store.path / 'telemetry' / ('samples.json' if not case_id else f'cases/{case_id}.json')
                telemetry.parent.mkdir(parents=True, exist_ok=True)
                write_telemetry(telemetry, sequences)
                prefix = f'{case_id}_' if case_id else ''
                (
                    telemetry.parent / f'{prefix}road_contact.json'
                    if case_id
                    else store.path / 'telemetry' / 'road_contact.json'
                ).write_text(json.dumps(road_contacts, allow_nan=False) + '\n')
                for kind, seq in sequences.items():
                    export_torque_csv(
                        seq,
                        store.path / 'telemetry' / f'{prefix}{kind}.csv',
                        contacts=road_contacts[kind],
                        reference_yaw=reference_yaw(spec, seq),
                        diagnostics=tick_records.get(kind),
                        extra_columns=handling_columns(seq, case_runtime.vehicle),
                    )
                if diagnostics:
                    name = 'controller_diagnostics.json' if not case_id else f'controller_diagnostics_{case_id}.json'
                    (store.path / 'metrics' / name).write_text(json.dumps(diagnostics, allow_nan=False) + '\n')
                matrix_records[case_id or 'single'] = records
            target = 'comparison.json' if len(runtimes) == 1 and None in runtimes else 'matrix.json'
            payload = matrix_records['single'] if target == 'comparison.json' else matrix_records
            (store.path / 'metrics' / target).write_text(json.dumps(payload, indent=2, allow_nan=False) + '\n')
            statuses = [r['status'] for records in matrix_records.values() for r in records.values()]
            status = 'PASS' if all(s == 'PASS' for s in statuses) else 'FAIL'
            store.finish(
                status,
                0 if status == 'PASS' else 1,
                duration_s=time.perf_counter() - started,
                config_fingerprint=resolved.fingerprint,
            )
        except Exception as exc:
            LOG.exception('Lauf %s abgebrochen', store.run_id)
            store.finish(
                'FAIL',
                1,
                error=f'{type(exc).__name__}: {exc}',
                duration_s=time.perf_counter() - started,
                config_fingerprint=resolved.fingerprint,
            )
            raise
        LOG.info('Lauf %s beendet: %s in %.1f s', store.run_id, status, time.perf_counter() - started)
        if runtime.visualization.auto_report if report is None else report:
            for case_id in runtimes:
                try:
                    written = render_report(store.path, case=case_id)
                    LOG.info('Bericht: %s', [p.name for p in written if p.suffix == '.pdf'][0])
                except ImportError as exc:
                    LOG.warning('Bericht übersprungen (Visualisierungs-Extra fehlt: %s)', exc)
    echo(
        json.dumps(
            {
                'run_id': store.run_id,
                'status': status,
                'path': str(store.path),
                'config_fingerprint': resolved.fingerprint,
            }
        )
    )
    return store.manifest['exit_code']


def road_contact_rows(runtime: RuntimeConfiguration, seq) -> list[dict]:
    """Lokaler Reibwert, Reifen-mu und Oberfläche je Rad und Sample – dieselbe Funktion wie in der Physik."""
    from torquevector.evaluation.road_friction_metrics import contact_record
    from torquevector.physics.road_friction import wheel_contacts

    field = runtime.scenario.surface
    if field is None:
        return []
    return [contact_record(s.t_s, wheel_contacts(field, s.state, runtime.vehicle, runtime.tyre)) for s in seq]


def road_summary(runtime: RuntimeConfiguration, seq, contacts, ticks) -> dict | None:
    from torquevector.evaluation.road_friction_metrics import road_metrics

    if not contacts:
        return None
    brake = runtime.scenario.brake_phase
    return road_metrics(
        runtime.scenario.spec,
        seq,
        contacts,
        utilization_limit=runtime.validation.envelope.max_tyre_utilization,
        slip_limit=runtime.validation.envelope.max_abs_slip,
        brake_start_s=None if brake is None else brake.start_s,
        diagnostics=[d for d in ticks if 'requested_nm' in d],
    )


def reference_yaw(spec, seq) -> list | None:
    if spec.path is None:
        return None
    return [s.state.vx_ms * spec.reference_at(s.state)[0].curvature_inv_m for s in seq]


def load_run(run_dir: Path, case: str | None = None):
    """Gespeicherte Telemetrie, Kennzahlen, Diagnostik und Konfigurationssnapshot eines Laufs."""
    from torquevector.infrastructure.config.resolver import load_snapshot
    from torquevector.infrastructure.telemetry_store import read_telemetry

    manifest = json.loads((run_dir / 'manifest.json').read_text())
    configuration = manifest['configuration']
    if case:
        if case not in configuration.get('cases', {}):
            raise ValueError(f'Fall {case!r} nicht im Lauf')
        scenario, duration = configuration['cases'][case]['scenario'], configuration['cases'][case]['duration_s']
        telemetry = run_dir / 'telemetry' / 'cases' / f'{case}.json'
        records = json.loads((run_dir / 'metrics/matrix.json').read_text())[case]
        diag_file = run_dir / 'metrics' / f'controller_diagnostics_{case}.json'
        road_file = run_dir / 'telemetry' / 'cases' / f'{case}_road_contact.json'
        snapshot = run_dir / 'config' / 'cases' / f'{case}.yaml'
    else:
        scenario, duration = configuration['scenario'], configuration['duration_s']
        telemetry = run_dir / 'telemetry' / 'samples.json'
        if not telemetry.is_file():
            raise ValueError('Matrixlauf: Fall mit --case wählen')
        records = json.loads((run_dir / 'metrics/comparison.json').read_text())
        diag_file = run_dir / 'metrics' / 'controller_diagnostics.json'
        road_file = run_dir / 'telemetry' / 'road_contact.json'
        snapshot = run_dir / 'config' / 'resolved_config.yaml'
    seqs = read_telemetry(telemetry)
    diagnostics = json.loads(diag_file.read_text()) if diag_file.is_file() else {}
    tree = load_snapshot(snapshot) if snapshot.is_file() else None
    return {
        'manifest': manifest,
        'spec': ManoeuvreSpec(**scenario),
        'duration_s': duration,
        'seqs': seqs,
        'records': records,
        'diagnostics': diagnostics,
        'tree': tree,
        'road_contacts': json.loads(road_file.read_text()) if road_file.is_file() else {},
    }


def road_inputs(data):
    """(Reibwertkarte, Radkontakte, Fahrzeug) für Fahrbahndarstellungen; None bei neutraler Fahrbahn."""
    field = build_road(data['tree']['road'])
    if field.is_identity or not data['road_contacts']:
        return None
    contacts = {k: v for k, v in data['road_contacts'].items() if k in data['seqs']}
    return field, contacts, build_vehicle(data['tree']['vehicle'])


def render_road_animation(run_dir, *, case: str | None = None, controller: str | None = None, output=None) -> Path:
    """GIF auf der Reibwertkarte aus gespeicherter Telemetrie."""
    from torquevector.visualization.report import primary_controller
    from torquevector.visualization.road_surface_animation import render_road_gif

    run_dir = Path(run_dir)
    data = load_run(run_dir, case)
    if data['tree'] is None or not data['road_contacts']:
        raise ValueError('Lauf ohne Fahrbahnkontakte: Fahrbahnanimation nicht möglich')
    field = build_road(data['tree']['road'])
    viz = visualization_from_tree(data['tree'])
    primary = primary_controller(data['seqs'], controller)
    target = Path(output) if output else run_dir / 'animations' / f"{case + '_' if case else ''}road_{primary}.gif"
    gif = viz.animation
    return render_road_gif(
        field,
        data['seqs'],
        data['road_contacts'],
        build_vehicle(data['tree']['vehicle']),
        primary,
        target,
        title=f"{data['spec'].kind} on {field.surface.id}",
        fps=gif.fps,
        max_frames=gif.max_frames,
        dpi=gif.dpi,
        tail_s=gif.tail_seconds,
    )


def render_report(run_dir, *, case: str | None = None, sections=None) -> list[Path]:
    """Analysebericht (PNG je Abschnitt + report.pdf) unter ``<run>/plots/`` erzeugen."""
    from torquevector.visualization.report import RunContext, write_report

    run_dir = Path(run_dir)
    data = load_run(run_dir, case)
    if data['tree'] is None:
        raise ValueError('Lauf ohne Konfigurationssnapshot: Bericht nicht möglich')
    viz = visualization_from_tree(data['tree'])
    tyre, model_name, _, model = build_tyre(data['tree']['tyre'])
    ctx = RunContext(
        run_id=run_dir.name,
        spec=data['spec'],
        duration_s=data['duration_s'],
        fingerprint=data['manifest'].get('config_fingerprint'),
        case=case,
    )
    written = write_report(
        run_dir / 'plots',
        ctx=ctx,
        seqs=data['seqs'],
        records=data['records'],
        diagnostics=data['diagnostics'],
        sections=sections or viz.report_sections,
        tyre=(tyre, model, f"{data['tree']['tyre']['id']} ({model_name})"),
        road=road_inputs(data),
        handling_series={
            kind: {'t': [s.t_s for s in seq], **_handling_series(seq, build_vehicle(data['tree']['vehicle']))}
            for kind, seq in data['seqs'].items()
        },
        dpi=viz.chart.dpi,
        formats=viz.chart.formats,
        prefix=f'{case}_' if case else '',
    )
    LOG.debug('Berichtsdateien: %s', [str(p) for p in written])
    return written


def build_study(definition, case_configs):
    study, pso = definition.study, definition.pso
    o = study['objective']
    objective = ObjectiveSpec(
        yaw_rmse=o['weights']['yaw_rmse'],
        cross_rmse=o['weights']['cross_rmse'],
        max_beta=o['weights']['max_beta'],
        soft_state=o['weights']['soft_state'],
        allocation_residual=o['weights']['allocation_residual'],
        beta_norm_deg=o['normalization']['max_beta_deg'],
        soft_state_norm=o['normalization']['soft_state_excess'],
        allocation_norm_nm=o['normalization']['allocation_residual_nm'],
        invalid_cost=o['invalid_candidate_cost'],
    )
    settings = PSOSettings(
        particles=pso['particles'],
        iterations=pso['iterations'],
        seed=pso['seed'],
        cognitive=pso['cognitive'],
        social=pso['social'],
        inertia_start=pso['inertia_start'],
        inertia_end=pso['inertia_end'],
    )
    slot = CONTROLLER_SLOTS[study['controller']]
    base = case_configs['base'].tree['controllers'][slot]
    dims = tuple(SearchDimension(d['parameter'], d['lower'], d['upper']) for d in study['search_space'])

    def lookup(parameter):
        node = base
        for key in parameter.split('.'):
            node = node[key]
        return float(node)

    thresholds = case_configs['base'].tree['validation']['thresholds']['qualification']

    def cases(label):
        return tuple(TuningCase(cid, resolved) for cid, resolved in case_configs[label])

    return TuningStudy(
        study_id=study['id'],
        controller=study['controller'],
        dimensions=dims,
        objective=objective,
        pso=settings,
        base_position=tuple(lookup(d.parameter) for d in dims),
        train=cases('train'),
        holdout=cases('holdout'),
        robustness=cases('robustness'),
        qualification=QualificationCriteria(**thresholds),
    )


def resolve_study_cases(definition, *, profiles=(), overrides=None):
    base = resolve(definition.application_path, profiles=profiles, overrides=overrides)
    from torquevector.infrastructure.config.resolver import load_component

    matrix = load_component(definition.matrix_path).section['sets']
    configs = {'base': base}
    for label in ('train', 'holdout', 'robustness'):
        name = definition.study['cases'].get(label)
        configs[label] = []
        if not name:
            continue
        for raw in matrix[name]:
            case = {
                'label': f'{name}/{raw["id"]}',
                'id': raw['id'],
                'replacements': {'scenario': str((definition.matrix_path.parent / raw['scenario']).resolve())},
                'overrides': (
                    [('scenario.manoeuvre.speed_ms', raw['speed_ms'])] if raw.get('speed_ms') is not None else []
                )
                + ([('scenario.simulation_duration_s', raw['duration_s'])] if raw.get('duration_s') is not None else [])
                + list((raw.get('overrides') or {}).items()),
            }
            configs[label].append((f'{name}/{raw["id"]}', resolve_case(base, case)))
    return configs


def make_evaluator(controller):
    slot = CONTROLLER_SLOTS[controller]

    def evaluate(kind, case, params):
        resolved = case.payload
        if params:
            resolved = with_overrides(resolved, [(f'controllers.{slot}.{k}', v) for k, v in params.items()])
        entry, _, _ = evaluate_case(build_runtime(resolved), kind)
        return entry

    return evaluate


def tuned_controller_section(base_section, params, provenance):
    import copy

    section = copy.deepcopy(base_section)
    for parameter, value in (params or {}).items():
        node = section
        keys = parameter.split('.')
        for key in keys[:-1]:
            node = node[key]
        node[keys[-1]] = value
    section['provenance'] = provenance
    return section


def run_optimization(
    config_path, *, study_overrides=None, results_dir=None, resume=None, evaluate=None, echo=print
) -> int:
    from torquevector.infrastructure.config.resolver import thaw
    from torquevector.infrastructure.optimization_store import CandidateCheckpoint, OptimizationRun

    definition = resolve_study(config_path, overrides=study_overrides)
    case_configs = resolve_study_cases(definition)
    study = build_study(definition, case_configs)
    root = (
        Path(results_dir).expanduser().resolve() if results_dir else Path(definition.study['output']['root']).resolve()
    )
    configs_root = definition.study_path.parent.parent
    run = OptimizationRun(root, study.study_id, resume=resume, forbidden_roots=(configs_root,))
    started = time.time()
    base_meta = {
        'study': str(definition.study_path),
        'pso': str(definition.pso_path),
        'application': str(definition.application_path),
        'matrix': str(definition.matrix_path),
        'study_fingerprint': definition.fingerprint,
        'base_config_fingerprint': case_configs['base'].fingerprint,
        'sources': [{'role': r, 'path': str(p), 'sha256': d} for r, p, d in definition.sources],
        'pso_settings': asdict(study.pso),
        'controller': study.controller,
        'seed': study.pso.seed,
        'source_kind': 'SYNTHETIC_SIMULATION_ONLY',
        'experimental_validation': False,
        'pso_online': False,
        'q10': 'BLOCKED_REQUIRED_REAL_DATA',
        'resumed': bool(resume),
    }
    run.write_metadata({**base_meta, 'status': 'INCOMPLETE'})
    checkpoint = None
    if definition.study['checkpoint']['enabled']:
        checkpoint = CandidateCheckpoint(run.path / definition.study['checkpoint']['file_name'], definition.fingerprint)
    from torquevector.infrastructure.logging_setup import run_log

    with run_log(run.path / 'logs'):
        LOG.info(
            'Offline-PSO %s (%s): %d Partikel x %d Iterationen, Seed %d, Ergebnisse %s',
            study.study_id,
            study.controller,
            study.pso.particles,
            study.pso.iterations,
            study.pso.seed,
            run.path,
        )
        try:
            report = run_study(study, evaluate or make_evaluator(study.controller), checkpoint, log=LOG.info)
        except Exception as exc:
            LOG.exception('Offline-PSO abgebrochen')
            run.write_metadata(
                {
                    **base_meta,
                    'status': 'FAIL',
                    'error': f'{type(exc).__name__}: {exc}',
                    'duration_s': time.time() - started,
                }
            )
            raise
        LOG.info(
            'Offline-PSO beendet: %s, beste Kosten %.5f, %d Bewertungen, Q11 %s',
            report['status'],
            report['best_fitness'],
            report['evaluations'],
            report['q11'],
        )
    slot = CONTROLLER_SLOTS[study.controller]
    best = None
    if report['best_parameters'] is not None:
        base_file = case_configs['base'].include_files[f'controllers.{slot}']
        best = tuned_controller_section(
            thaw(case_configs['base'].tree['controllers'][slot]),
            report['best_parameters'],
            {
                'source_type': 'synthetic_calibrated',
                'experimentally_validated': False,
                'origin': f'Offline-PSO {study.study_id}, Lauf {run.run_id}, Basis {base_file}',
                'notes': 'Nur synthetische Simulationsoptimierung; keine Fahrzeugvalidierung, keine Q11-Freigabe.',
                'calibration': {
                    'study_fingerprint': definition.fingerprint,
                    'base_config_fingerprint': case_configs['base'].fingerprint,
                    'best_fitness': report['best_fitness'],
                    'seed': study.pso.seed,
                },
            },
        )
    run.write_results(report, best)
    run.write_metadata(
        {
            **base_meta,
            'status': report['status'],
            'q11': report['q11'],
            'best_fitness': report['best_fitness'],
            'evaluations': report['evaluations'],
            'duration_s': time.time() - started,
        }
    )
    echo(json.dumps({'run_id': run.run_id, 'status': report['status'], 'path': str(run.path)}))
    return 0 if report['status'] == 'FEASIBLE' else 1


Q11_SETS = ('q11_nominal', 'q11_robust', 'q11_degraded')


def q11_assessment(resolve_set, output: Path) -> dict:
    """Q11-Mengen rechnen (Läufe unter ``output/runs``) und bewerten; ``resolve_set(name)`` liefert die Konfiguration."""
    from torquevector.evaluation.q11_assessment import assess_degraded, assess_nominal, assess_robust

    runs, matrices, diagnostics = {}, {}, {}
    for name in Q11_SETS:
        lines = []
        run_configured(resolve_set(name), results_dir=output, report=False, echo=lines.append, run_name=name)
        record = json.loads(lines[-1])
        run = Path(record['path'])
        runs[name] = {'run_id': record['run_id'], 'path': str(run), 'config_fingerprint': record['config_fingerprint']}
        matrices[name] = json.loads((run / 'metrics/matrix.json').read_text())
        diagnostics[name] = {
            case: json.loads(p.read_text())
            for case in matrices[name]
            for p in [run / 'metrics' / f'controller_diagnostics_{case}.json']
            if p.is_file()
        }
        LOG.info('%s: %s', name, record['status'])
    runtime = build_runtime(resolve_set(Q11_SETS[0]))
    q = runtime.validation.qualification
    c = runtime.controllers
    limits = {
        'p': c.p_yaw_limit_nm,
        'lqr': c.lqr_yaw_limit_nm,
        'ltv': c.ltv.moment_limit_nm,
        'nmpc': c.nmpc.moment_limit_nm,
    }
    return {
        'source_kind': 'SYNTHETIC_SIMULATION_ONLY',
        'experimental_validation': False,
        'runs': runs,
        'yaw_reference_source': runtime.driver.reference_source,
        'allocation_friction_mode': c.allocation_friction_mode,
        'Q11.3_nominal': assess_nominal(
            matrices['q11_nominal'], min_yaw_pct=q.min_yaw_improvement_pct, min_path_pct=q.min_path_improvement_pct
        ),
        'robustness': assess_robust(
            matrices['q11_robust'], min_yaw_pct=q.min_yaw_improvement_pct, min_path_pct=q.min_path_improvement_pct
        ),
        'Q11.4_degraded': assess_degraded(matrices['q11_degraded'], diagnostics['q11_degraded'], limits),
    }


# ---------------------------------------------------------------- handling characteristic and limits


def _finite(value):
    """JSON-safe: None instead of ±inf/NaN (e.g. the linear yaw gain beyond the critical speed)."""
    return float(value) if np.isfinite(value) else None


class SideslipStop:
    """Stop condition for the sampled simulation: |β| above the limit (loss of stability)."""

    def __init__(self, limit_deg: float):
        self.limit_rad, self.triggered = radians(limit_deg), False

    def __call__(self, state) -> bool:
        self.triggered = self.triggered or abs(atan2(state.vy_ms, state.vx_ms)) > self.limit_rad
        return self.triggered


def ramp_steer_runtime(config, speed_ms: float):
    """Runtime of the ISO 4138 ramp steer for one variant and speed, plus the final road-wheel angle [deg].

    The ramp ends at the Ackermann angle of ``lateral_limit_estimate_factor``·µ·g plus ``ramp_margin_deg`` (capped at
    25 deg); speed is held with the stronger handling speed controller (ISO 4138: constant speed).
    """
    from torquevector.infrastructure.config.resolver import resolve

    base = build_runtime(resolve(config))
    s, car, tyre = base.validation.handling, base.vehicle, base.tyre
    end_deg = min(
        25.0,
        degrees(car.wheelbase_m * s.lateral_limit_estimate_factor * tyre.mu * car.gravity / speed_ms**2)
        + s.ramp_margin_deg,
    )
    configs = Path(__file__).resolve().parents[3] / 'configs'
    resolved = resolve(
        config,
        replacements={'scenario': str(configs / 'scenarios/ramp_steer.yaml')},
        overrides=[
            ('scenario.manoeuvre.speed_ms', speed_ms),
            ('scenario.manoeuvre.steering.amplitude_deg', round(end_deg, 3)),
            ('scenario.manoeuvre.steering.max_steer_deg', round(end_deg, 3)),
            ('scenario.manoeuvre.speed_control.kp_nm_per_ms', s.speed_kp_nm_per_ms),
            ('simulation.driver.base_policy.max_base_wheel_torque_nm', s.driver_max_wheel_torque_nm),
        ],
    )
    return build_runtime(resolved), end_deg


def handling_analysis(variants: dict, output: Path, *, controllers=None) -> dict:
    """Self-steering characteristic and handling limits for each vehicle variant.

    ``variants``: name → application/profile YAML. Per variant: linear single-track reference, ISO 4138 ramp steer per
    speed and controller (stopped when |β| exceeds the limit), β–r phase plane (OFF) and the g-g diagram. Every
    simulated run writes its telemetry CSV below ``output/<variant>/``.
    """
    from functools import partial

    from torquevector.evaluation.handling import (
        acceleration_points,
        characteristic,
        gg_envelope,
        linear_characteristic,
        linear_yaw_gain,
        steady_state_curve,
    )
    from torquevector.infrastructure.config.resolver import resolve
    from torquevector.infrastructure.telemetry_io import export_torque_csv

    configs = Path(__file__).resolve().parents[3] / 'configs'
    report = {'source_kind': 'SYNTHETIC_SIMULATION_ONLY', 'variants': {}}
    curves, phase, gg = {}, {}, {}
    for name, config in variants.items():
        base = build_runtime(resolve(config))
        settings, car, tyre = base.validation.handling, base.vehicle, base.tyre
        linear = linear_characteristic(
            car, tyre, base.tyre_axle_scaling, neutral_tolerance_deg_per_g=settings.neutral_tolerance_deg_per_g
        )
        entry = {'config': str(config), 'linear': linear, 'ramp_steer': {}, 'phase_plane': [], 'gg': {}}
        kinds = tuple(controllers or settings.controllers)
        points_ax, points_ay = [], []
        for speed in settings.speeds_ms:
            rt, end_deg = ramp_steer_runtime(config, speed)
            for kind in kinds:
                stop = SideslipStop(settings.stop_sideslip_deg)
                key = f'{kind}@{speed:g}'
                try:
                    _, seq, _ = run_case(
                        kind,
                        rt.scenario,
                        rt.plant,
                        rt.controllers,
                        rt.driver,
                        simulator=partial(simulate_sampled, stop_condition=stop),
                    )
                except Exception as exc:  # documented failure, never silently dropped
                    entry['ramp_steer'][key] = {'status': 'FAIL_CLOSED', 'reason': str(exc)}
                    LOG.warning('Handling %s %s: FAIL_CLOSED (%s)', name, key, exc)
                    continue
                curve = steady_state_curve(seq, car, start_s=rt.scenario.spec.excitation_start_s)
                result = characteristic(curve, settings, stopped_by_sideslip=stop.triggered)
                result.update(status='PASS', speed_ms=speed, controller=kind, ramp_end_deg=end_deg)
                entry['ramp_steer'][key] = result
                curves[(name, key)] = curve
                ax, ay = acceleration_points(seq, car)
                points_ax += ax
                points_ay += ay
                export_torque_csv(
                    seq, output / name / f'ramp_steer_{kind}_{speed:g}ms.csv', extra_columns=handling_columns(seq, car)
                )
                LOG.info(
                    'Handling %s %s: K=%s deg/g, ay_max=%.2f g, %s',
                    name,
                    key,
                    result['linear_gradient_deg_per_g'],
                    result['max_lateral_acceleration_g'],
                    result['limit_behaviour'],
                )
        entry['yaw_gain'] = [
            {
                'speed_ms': float(v),
                'linear_per_s': _finite(linear_yaw_gain(car, linear['understeer_gradient_rad_per_ms2'], v)),
            }
            for v in np.linspace(5.0, 40.0, 36)
        ]
        gx, gy = _gg_runs(config, settings, configs)
        points_ax += gx
        points_ay += gy
        envelope = gg_envelope(car, tyre, base.tyre_axle_scaling, settings.gg_directions)
        entry['gg'] = {'ideal_max_lateral_g': envelope['max_lateral_g']}
        gg[name] = (envelope, points_ax, points_ay)
        entry['phase_plane'], phase[name] = _phase_plane(config, settings, configs)
        report['variants'][name] = entry
    report['_figures'] = {'curves': curves, 'phase': phase, 'gg': gg}
    return report


def _gg_runs(config, settings, configs):
    """Combined-acceleration runs (OFF) for the achieved points of the g-g diagram: braking in a circle (front torque
    per wheel from ``gg_brake_in_turn_nm``, rear ``gg_rear_brake_fraction`` of it) and straight all-wheel acceleration
    on dry asphalt (``gg_straight_drive_nm``). Failed runs (wheel lock, envelope) are skipped and logged."""
    wheels = ('FL', 'FR', 'RL', 'RR')
    from torquevector.evaluation.handling import acceleration_points
    from torquevector.infrastructure.config.resolver import resolve

    rear = settings.gg_rear_brake_fraction
    cases = [
        (
            'brake_in_turn.yaml',
            [
                ('scenario.manoeuvre.speed_ms', settings.gg_brake_in_turn_speed_ms),
                ('scenario.brake_phase.brake_torque_nm', {w: t * (1.0 if w[0] == 'F' else rear) for w in wheels}),
            ],
        )
        for t in settings.gg_brake_in_turn_nm
    ] + [
        (
            'straight_split_mu_acceleration.yaml',
            [
                ('scenario.road.surface', '../roads/dry_asphalt.yaml'),
                ('scenario.drive_phase.drive_torque_nm', {w: t for w in wheels}),
            ],
        )
        for t in settings.gg_straight_drive_nm
    ]
    ax, ay = [], []
    for scenario, overrides in cases:
        rt = build_runtime(
            resolve(config, replacements={'scenario': str(configs / 'scenarios' / scenario)}, overrides=overrides)
        )
        try:
            _, seq, _ = run_case('off', rt.scenario, rt.plant, rt.controllers, rt.driver)
        except Exception as exc:
            LOG.debug('g-g run %s %s skipped: %s', scenario, overrides[-1], exc)
            continue
        x, y = acceleration_points(seq, rt.vehicle)
        ax += x
        ay += y
    return ax, ay


def _phase_plane(config, settings, configs):
    """β–r phase plane with OFF: constant steer, initial (β0, r0) grid; outcome per trajectory."""
    from functools import partial

    from torquevector.evaluation.handling import phase_plane_outcome
    from torquevector.infrastructure.config.resolver import resolve

    pp = settings.phase_plane
    outcomes, trajectories = [], []
    for beta0 in np.linspace(-pp.sideslip_range_deg, pp.sideslip_range_deg, pp.grid_sideslip):
        for r0 in np.linspace(-pp.yaw_rate_range_rads, pp.yaw_rate_range_rads, pp.grid_yaw_rate):
            resolved = resolve(
                config,
                replacements={'scenario': str(configs / 'scenarios/step_steer.yaml')},
                overrides=[
                    ('scenario.manoeuvre.speed_ms', pp.speed_ms),
                    ('scenario.manoeuvre.duration_s', pp.duration_s),
                    ('scenario.manoeuvre.steering.amplitude_deg', pp.steer_deg),
                    ('scenario.manoeuvre.steering.excitation_start_s', 0.0),
                    ('scenario.initial_state.vy_ms', float(pp.speed_ms * np.tan(np.radians(beta0)))),
                    ('scenario.initial_state.yaw_rate_rads', float(r0)),
                ],
            )
            rt = build_runtime(resolved)
            stop = SideslipStop(settings.stop_sideslip_deg)
            try:
                _, seq, _ = run_case(
                    'off',
                    rt.scenario,
                    rt.plant,
                    rt.controllers,
                    rt.driver,
                    simulator=partial(simulate_sampled, stop_condition=stop),
                )
                outcome = phase_plane_outcome(
                    seq, stopped=stop.triggered, settled_rads2=pp.settled_yaw_acceleration_rads2
                )
            except Exception as exc:
                seq, outcome = [], 'fail_closed'
                LOG.debug('Phase plane β0=%.1f r0=%.2f: %s', beta0, r0, exc)
            beta = [degrees(atan2(s.state.vy_ms, s.state.vx_ms)) for s in seq]
            r = [s.state.yaw_rate_rads for s in seq]
            outcomes.append({'sideslip0_deg': float(beta0), 'yaw_rate0_rads': float(r0), 'outcome': outcome})
            trajectories.append((beta, r, outcome))
    return outcomes, trajectories
