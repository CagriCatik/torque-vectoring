"""Historische Runner-API; delegiert an den vollständig parametrisierten ``run_case``.

Die Literale unten sind die eingefrorenen V11-Referenzwerte dieser Kompatibilitäts-API.
Kanonisch werden dieselben Werte aus ``configs/*.yaml`` geladen; der Test
``tests/unit/test_configuration_defaults_consistency.py`` stellt Gleichheit sicher.
"""

from __future__ import annotations
from torquevector.infrastructure.configuration import load_model
from torquevector.physics.steering import SteeringActuator
from torquevector.domain.vehicle import VehicleParameters
from torquevector.control.controllers.ltv_mpc import LTVConfig
from torquevector.control.controllers.nmpc import NMPCConfig, NMPCSolverSettings
from torquevector.control.allocation import AllocationOptions
from torquevector.control.slip_control import SlipControlSettings
from torquevector.domain.road_surface import RoadSurface
from torquevector.physics.road_friction import RoadFrictionMap
from torquevector.simulation.sampled import SampledSettings
from torquevector.simulation.sampled import simulate_sampled
from torquevector.domain.controls import TorqueControls
from torquevector.domain.wheels import WheelParameters
from torquevector.control.controllers.lqr import LQRConfig
from torquevector.simulation.configured import (
    ControllerSetup,
    DriverSettings,
    InitialPose,
    PlantSetup,
    ScenarioSetup,
    run_case,
)

# Gleichförmige Fahrbahn, die den Reifenreibwert unverändert lässt (= bisheriges Modell).
# ABS/ASR aus (V11-Verhalten); Reglerparameter wie configs/powertrain/four_wheel_independent.yaml.
SLIP_CONTROL_OFF = SlipControlSettings(
    abs_enabled=False,
    asr_enabled=False,
    period_s=0.005,
    abs_target_slip=0.1,
    asr_target_slip=0.1,
    kp=4.0,
    ki_per_s=60.0,
    torque_rate_nm_s=20000.0,
)
NEUTRAL_ROAD = RoadFrictionMap(RoadSurface('neutral', 'tyre_reference', 1.0, (), 'sharp', 0.0, 'scale_tyre_mu', 1.0))

LEGACY_DRIVER = DriverSettings(
    max_drive_torque_nm=250.0,
    step_steer_ramp_s=0.4,
    reference_engage_ramp_s=0.55,
    reference_friction_fraction=0.75,
    preview_speed_time_constant_s=2.0,
    reference_source='path_curvature',
)


def legacy_setups(spec, duration, *, nmpc_config, ltv_config, car, tyre, model):
    """Setup-Objekte mit den unveränderten Literalen der historischen API."""
    plant = PlantSetup(
        car=car,
        tyre=tyre,
        tyre_model=model,
        wheel=WheelParameters(),
        actuator=SteeringActuator(0.12, 120.0),
        sampled=SampledSettings(
            period_s=nmpc_config.sample_s,
            max_step_s=min(0.02, nmpc_config.sample_s),
            rtol=2e-5,
            max_torque_rate_nm_s=1500.0,
        ),
        load_solver='root',
        initial_controls=TorqueControls(),
        drive_distribution=(0.2, 0.2, 0.3, 0.3),
        slip_control=SLIP_CONTROL_OFF,
    )
    controllers = ControllerSetup(
        proportional_gain=2300.0,
        p_yaw_limit_nm=2000.0,
        p_reference_mu_source='tyre',
        p_reference_mu_explicit=None,
        lqr=LQRConfig(),
        lqr_yaw_limit_nm=2000.0,
        lqr_reference_mu_source='tyre',
        lqr_reference_mu_explicit=None,
        ltv=ltv_config,
        nmpc=nmpc_config,
        nmpc_solver=NMPCSolverSettings(),
        allocation=AllocationOptions(),
        ltv_friction_preview='unknown',
        nmpc_friction_preview='unknown',
        allocation_friction_mode='off',
        allocation_reserve_fraction=0.8,
        allocation_mu_source='tyre',
        allocation_mu_explicit=None,
    )
    scenario = ScenarioSetup(
        spec=spec,
        duration_s=duration,
        initial_state=InitialPose(0.0, 0.0, 0.0, 0.0, 0.0),
        road_model='road_surface',
        road=None,
        brake_phase=None,
        check_final_speed=True,
        surface=NEUTRAL_ROAD,
        initial_speed_ms=spec.speed_ms,
        drive_phase=None,
    )
    return plant, controllers, scenario


def simulate_controller(
    kind,
    spec,
    duration,
    *,
    nmpc_config=NMPCConfig(),
    ltv_config=None,
    car=None,
    tyre=None,
    model=None,
    diagnostics=None,
):
    car = car or VehicleParameters()
    if tyre is None or model is None:
        tyre, model = load_model()
    cfg = nmpc_config
    if ltv_config is None:
        ltv_config = LTVConfig(sample_s=cfg.sample_s)
    if abs(ltv_config.sample_s - cfg.sample_s) > 1e-9:
        raise ValueError('All sampled controllers must share control period')
    plant, controllers, scenario = legacy_setups(
        spec, duration, nmpc_config=cfg, ltv_config=ltv_config, car=car, tyre=tyre, model=model
    )
    metric, seq, ticks = run_case(
        kind, scenario, plant, controllers, LEGACY_DRIVER, simulator=simulate_sampled, feedback_diagnostics=False
    )
    if diagnostics is not None and kind in ('ltv', 'nmpc'):
        diagnostics.extend(ticks)
    return metric, seq
