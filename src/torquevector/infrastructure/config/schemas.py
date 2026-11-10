"""Konkrete Schemata aller YAML-Konfigurationsarten.

``binds`` nennt den Attributpfad im typisierten ``RuntimeConfiguration``-Objekt
der Composition Root (``torquevector.cli.composition``). Architekturtests prüfen, dass
jedes Feld der gebundenen Dataclasses genau aus YAML stammt und jede gebundene
YAML-Größe das Laufzeitobjekt tatsächlich verändert.

Bereiche spiegeln die Plausibilitätsprüfungen der Fachklassen; diese prüfen
zusätzlich (Defense in Depth). Werte mit ``fixed`` sind im Code verankert
(AST-eingefrorene Definitionen oder nicht implementierte Alternativen) und
werden sichtbar gemacht, aber nicht konfigurierbar.
"""

from __future__ import annotations

from torquevector.infrastructure.config.schema import Field, Section

SCHEMA_VERSION = 1
CONTROLLER_KINDS = ('off', 'p', 'lqr', 'ltv', 'nmpc')
FROZEN = 'AST-eingefrorene Definition (tests/fixtures/structure_definition_hashes.json); Änderung nur mit Freigabe'
NOT_IMPLEMENTED = 'Alternative im Modell nicht implementiert'

SIM = ('torquevector.simulation.configured', 'torquevector.simulation.sampled')
CTRL = ('torquevector.control',)


def num(
    description,
    unit,
    lo=None,
    hi=None,
    *,
    binds=(),
    used=(),
    xlo=False,
    xhi=False,
    kind='float',
    nullable=False,
    required=True,
    fixed=None,
    reason='',
):
    extra = {} if fixed is None else {'fixed': fixed, 'fixed_reason': reason}
    return Field(
        kind,
        description,
        unit=unit,
        minimum=lo,
        maximum=hi,
        exclusive_minimum=xlo,
        exclusive_maximum=xhi,
        binds=tuple(binds) if isinstance(binds, (list, tuple)) else (binds,),
        used_by=tuple(used),
        nullable=nullable,
        required=required,
        **extra,
    )


def integer(description, lo=None, hi=None, *, binds=(), used=(), fixed=None, reason=''):
    return num(description, '1', lo, hi, binds=binds, used=used, kind='int', fixed=fixed, reason=reason)


def text(description, *, required=True, nullable=False):
    return Field('str', description, required=required, nullable=nullable)


def path(description, *, required=True, nullable=False, structural_default=None):
    extra = {} if structural_default is None else {'structural_default': structural_default}
    return Field('path', description, required=required, nullable=nullable, **extra)


def enum(description, choices, *, binds=(), used=(), fixed=None, reason='', required=True):
    extra = {} if fixed is None else {'fixed': fixed, 'fixed_reason': reason}
    return Field(
        'enum',
        description,
        choices=tuple(choices),
        binds=tuple(binds) if isinstance(binds, (list, tuple)) else (binds,),
        used_by=tuple(used),
        required=required,
        **extra,
    )


def flag(description, *, binds=(), used=(), fixed=None, reason=''):
    extra = {} if fixed is None else {'fixed': fixed, 'fixed_reason': reason}
    return Field(
        'bool',
        description,
        binds=tuple(binds) if isinstance(binds, (list, tuple)) else (binds,),
        used_by=tuple(used),
        **extra,
    )


def wheels(description, unit, lo=None, hi=None, *, binds=(), used=()):
    return Field(
        'wheel_map',
        description,
        unit=unit,
        minimum=lo,
        maximum=hi,
        binds=(binds,) if binds else (),
        used_by=tuple(used),
    )


def provenance(validation_key: str) -> Section:
    return Section(
        {
            'source_type': enum(
                'Art der Datenquelle',
                ('synthetic', 'synthetic_calibrated', 'engineering_assumption', 'literature', 'measured'),
            ),
            validation_key: flag('Gegen reale Messungen validiert (synthetische Quellen müssen false sein)'),
            'origin': text('Ursprung der Werte (Datei/Symbol/Bericht im Repository)'),
            'notes': text('Hinweise, Einschränkungen', required=False, nullable=True),
            'calibration': Section(
                {}, 'Kalibrierungsmetadaten (nicht modellwirksam)', required=False, nullable=True, open_mapping=True
            ),
        },
        'Herkunft und Validierungsstatus',
    )


def header(validation_key='experimentally_validated') -> dict:
    return {
        'id': text('Eindeutige Kennung'),
        'description': text('Beschreibung', required=False, nullable=True),
        'provenance': provenance(validation_key),
    }


VEHICLE = Section(
    {
        **header('validated_against_real_vehicle'),
        'mass_kg': num(
            'Fahrzeugmasse',
            'kg',
            0,
            xlo=True,
            binds='vehicle.mass_kg',
            used=('torquevector.physics.four_wheel', 'torquevector.physics.load_transfer', 'torquevector.prediction'),
        ),
        'geometry': Section(
            {
                'wheelbase_m': num(
                    'Radstand', 'm', 0, xlo=True, binds='vehicle.wheelbase_m', used=('torquevector.physics',)
                ),
                'track_m': num(
                    'Spurweite (vorne = hinten; getrennte Spurweiten nicht modelliert)',
                    'm',
                    0,
                    xlo=True,
                    binds='vehicle.track_m',
                    used=('torquevector.physics',),
                ),
                'cg_to_front_axle_m': num(
                    'Abstand Schwerpunkt-Vorderachse; Hinterachse = Radstand - Wert',
                    'm',
                    0,
                    xlo=True,
                    binds='vehicle.cg_to_front_m',
                    used=('torquevector.physics',),
                ),
                'cg_height_m': num(
                    'Schwerpunktshöhe',
                    'm',
                    0,
                    binds='vehicle.cg_height_m',
                    used=('torquevector.physics.load_transfer',),
                ),
            },
            'Geometrie',
        ),
        'inertia': Section(
            {
                'yaw_kgm2': num(
                    'Gierträgheitsmoment',
                    'kg*m^2',
                    0,
                    xlo=True,
                    binds='vehicle.iz_kgm2',
                    used=('torquevector.physics', 'torquevector.prediction'),
                ),
            },
            'Trägheit',
        ),
        'aerodynamics': Section(
            {
                'drag_area_m2': num(
                    'Luftwiderstandsfläche c_w*A',
                    'm^2',
                    0,
                    binds='vehicle.cd_area_m2',
                    used=('torquevector.physics.four_wheel', 'torquevector.simulation.driving'),
                ),
                'air_density_kgm3': num(
                    'Luftdichte',
                    'kg/m^3',
                    0,
                    xlo=True,
                    binds='vehicle.air_density_kgm3',
                    used=('torquevector.physics.four_wheel', 'torquevector.simulation.driving'),
                ),
            },
            'Aerodynamik',
        ),
        'rolling_resistance': Section(
            {
                'coefficient': num(
                    'Rollwiderstandsbeiwert f_R',
                    '1',
                    0,
                    binds='vehicle.rolling_coefficient',
                    used=('torquevector.physics.four_wheel', 'torquevector.simulation.driving'),
                ),
            },
            'Rollwiderstand',
        ),
        'environment': Section(
            {
                'gravity_ms2': num(
                    'Gravitationsbeschleunigung',
                    'm/s^2',
                    0,
                    xlo=True,
                    binds='vehicle.gravity',
                    used=('torquevector.physics', 'torquevector.control', 'torquevector.prediction'),
                ),
            },
            'Umgebung',
        ),
        'load_transfer': Section(
            {
                'enabled': flag(
                    'Quasi-statische Radlastverlagerung längs/quer',
                    binds='vehicle.load_transfer',
                    used=('torquevector.physics.load_transfer',),
                ),
                'model': enum(
                    'Lastverlagerungsmodell',
                    ('quasi_static_fixed_split',),
                    fixed='quasi_static_fixed_split',
                    reason=FROZEN + '; Querverteilung nach statischem Achslastanteil, kein Wankmodell',
                ),
            },
            'Radlastverlagerung',
        ),
    },
    'Fahrzeugparameter (VehicleParameters)',
)

AXIS = {
    'B': num('Steifigkeitsfaktor B', '1', 0, 200, xlo=True),
    'C': num('Formfaktor C', '1', 0, 3, xlo=True),
    'E': num('Krümmungsfaktor E', '1', -5, 1),
}


def axis(name):
    return Section(
        {
            key: num(
                spec.description,
                '1',
                spec.minimum,
                spec.maximum,
                xlo=spec.exclusive_minimum,
                binds=f'tyre.{name}.{key.lower()}',
                used=('torquevector.physics.tyre_magic_formula', 'torquevector.prediction.bicycle_linear'),
            )
            for key, spec in AXIS.items()
        },
        f'Magic-Formula-Formparameter {name}; D = mu(Fz)*Fz wird abgeleitet, kein freier Parameter',
    )


TYRE = Section(
    {
        **header(),
        'model': enum(
            'Reinschlupf-Reifenmodell',
            ('magic_formula_bce',),
            fixed='magic_formula_bce',
            reason='Einzige Implementierung Y=D sin(C atan(Bs-E(Bs-atan Bs))); keine MF5.2/MF6.x-Koeffizienten',
        ),
        'friction': Section(
            {
                'mu': num(
                    'Reibwert bei Referenzlast (wird durch Straßen-Reibwertkarte ersetzt, falls aktiv)',
                    '1',
                    0.05,
                    2,
                    binds='tyre.mu',
                    used=('torquevector.physics.tyre_magic_formula',),
                ),
                'reference_load_n': num(
                    'Referenzradlast Fz0 der Lastabhängigkeit',
                    'N',
                    0,
                    xlo=True,
                    binds='tyre.nominal_fz_n',
                    used=('torquevector.physics.tyre_magic_formula',),
                ),
                'load_sensitivity_exponent': num(
                    'Exponent mu_eff = mu*(Fz/Fz0)^k',
                    '1',
                    -0.3,
                    0,
                    binds='tyre.load_exponent',
                    used=('torquevector.physics.tyre_magic_formula',),
                ),
            },
            'Reibwert und Vertikallastabhängigkeit',
        ),
        'longitudinal': axis('longitudinal'),
        'lateral': axis('lateral'),
        'axle_scaling': Section(
            {
                axle: Section(
                    {
                        'lateral_stiffness_scale': num(
                            f'Skalierung der Schräglaufsteifigkeit {label} (λ_Ky, über B_y; Peak unverändert)',
                            '1',
                            0.3,
                            3.0,
                            binds=f'tyre_axle_scaling.{axle}.lateral_stiffness_scale',
                            used=('torquevector.physics.axle_tyre',),
                        ),
                        'friction_scale': num(
                            f'Skalierung des Reibwerts {label} (λ_µ, Peak Fx und Fy)',
                            '1',
                            0.3,
                            3.0,
                            binds=f'tyre_axle_scaling.{axle}.friction_scale',
                            used=('torquevector.physics.axle_tyre',),
                        ),
                    },
                    f'Achsskalierung {label}',
                )
                for axle, label in (('front', 'vorne'), ('rear', 'hinten'))
            },
            'Achsweise Reifenskalierung (Eigenlenkverhalten; 1/1 = bitgleich zum unskalierten Reifen)',
        ),
        'combined_slip': Section(
            {
                'model': enum(
                    'Kopplung kombinierter Schlupf',
                    ('kamm_circle', 'elliptic_damping_surrogate'),
                    binds='combined_slip_model',
                    used=('torquevector.physics.tyre_magic_formula', 'torquevector.physics.combined_slip'),
                ),
                'alpha_scale_rad': num(
                    'Dämpfungsskala Schräglauf auf Fx (nur Surrogat)',
                    'rad',
                    0.025,
                    0.7,
                    binds='combined_slip.alpha_scale_rad',
                    nullable=True,
                    used=('torquevector.physics.combined_slip',),
                ),
                'kappa_scale': num(
                    'Dämpfungsskala Längsschlupf auf Fy (nur Surrogat)',
                    '1',
                    0.025,
                    0.9,
                    binds='combined_slip.kappa_scale',
                    nullable=True,
                    used=('torquevector.physics.combined_slip',),
                ),
                'lateral_peak_fraction': num(
                    'Seitenkraft-Ellipsenhalbachse relativ zu mu*Fz (nur Surrogat)',
                    '1',
                    0.5,
                    1.4,
                    binds='combined_slip.lateral_peak_fraction',
                    nullable=True,
                    used=('torquevector.physics.combined_slip',),
                ),
                'longitudinal_peak_fraction': num(
                    'Längskraft-Ellipsenhalbachse relativ zu mu*Fz (nur Surrogat)',
                    '1',
                    0.5,
                    1.4,
                    binds='combined_slip.longitudinal_peak_fraction',
                    nullable=True,
                    used=('torquevector.physics.combined_slip',),
                ),
            },
            'Combined-Slip-Modell; Surrogat-Parameter nur bei elliptic_damping_surrogate',
        ),
        'operating_limits': Section(
            {
                'max_abs_longitudinal_slip': num(
                    'Max. |kappa| im kalibrierten Bereich; außerhalb Fehler statt Extrapolation',
                    '1',
                    0,
                    1,
                    xlo=True,
                    binds='wheel.max_abs_slip',
                    used=('torquevector.physics.wheel_rotation',),
                ),
            },
            'Gültigkeitsgrenzen (Schräglaufgrenze, Geschwindigkeitsbereich, Druck, Temperatur nicht modelliert)',
        ),
    },
    'Reifenparameter (TyreParameters, CombinedSlipShape)',
)

WHEEL = Section(
    {
        **header(),
        'effective_radius_m': num(
            'Effektiver Rollradius',
            'm',
            0.15,
            0.6,
            binds='wheel.radius_m',
            used=('torquevector.physics.wheel_rotation', 'torquevector.control.allocation', 'torquevector.simulation'),
        ),
        'rotational_inertia_kgm2': num(
            'Radträgheitsmoment um die Drehachse',
            'kg*m^2',
            0,
            xlo=True,
            binds='wheel.inertia_kgm2',
            used=('torquevector.physics.wheel_rotation',),
        ),
        'min_contact_speed_ms': num(
            'Minimale Kontaktgeschwindigkeit des Vorwärtsmodells',
            'm/s',
            2,
            binds='wheel.min_contact_speed_ms',
            used=('torquevector.physics.wheel_rotation',),
        ),
        'initial_condition': enum(
            'Anfangsdrehzahl',
            ('free_rolling',),
            fixed='free_rolling',
            reason='omega0 = v0/R; andere Anfangsbedingungen ' + NOT_IMPLEMENTED,
        ),
    },
    'Radparameter (WheelParameters)',
)

POWERTRAIN = Section(
    {
        **header(),
        'layout': enum(
            'Antriebsstrangtopologie',
            ('four_wheel_independent',),
            fixed='four_wheel_independent',
            reason='Vier unabhängige Radmomente; Wirkungsgrad/Motorkennfeld ' + NOT_IMPLEMENTED,
        ),
        'wheel_torque_limit_nm': num(
            'Maximales Rad-Antriebs- und Bremsmoment je Rad (gemeinsame Grenze)',
            'N*m',
            0,
            xlo=True,
            binds='plant.initial_controls.torque_limit_nm',
            used=('torquevector.domain.controls', 'torquevector.control.allocation'),
        ),
        'drive_distribution': wheels(
            'Verteilung des Fahrer-Antriebsmoments auf die Räder (Summe 1)',
            '1',
            0,
            1,
            binds='plant.drive_distribution',
            used=('torquevector.simulation.policies',),
        ),
        'initial_state': Section(
            {
                'drive_torque_nm': wheels(
                    'Antriebsmomente vor dem ersten Takt (Startwert der Slew-Begrenzung)',
                    'N*m',
                    0,
                    binds='plant.initial_controls.drive_nm',
                    used=('torquevector.simulation.sampled',),
                ),
                'brake_torque_nm': wheels(
                    'Bremsmomente vor dem ersten Takt',
                    'N*m',
                    0,
                    binds='plant.initial_controls.brake_nm',
                    used=('torquevector.simulation.sampled',),
                ),
                'steer_command_deg': num(
                    'Lenkkommando vor dem ersten Takt',
                    'deg',
                    -25,
                    25,
                    binds='plant.initial_controls.steer_deg',
                    used=('torquevector.simulation.sampled',),
                ),
            },
            'Aktuator-Anfangszustände (keine physikalischen Eigenschaften)',
        ),
        'steering_actuator': Section(
            {
                'time_constant_s': num(
                    'Zeitkonstante PT1-Lenkaktuator',
                    's',
                    0,
                    xlo=True,
                    binds='plant.actuator.time_constant_s',
                    used=('torquevector.physics.steering',),
                ),
                'rate_limit_deg_s': num(
                    'Ratenbegrenzung Lenkaktuator',
                    'deg/s',
                    0,
                    xlo=True,
                    binds='plant.actuator.rate_limit_deg_s',
                    used=('torquevector.physics.steering',),
                ),
                'max_angle_deg': num(
                    'Lenkwinkelgrenze am Rad',
                    'deg',
                    0,
                    25,
                    xlo=True,
                    binds='plant.actuator.max_angle_deg',
                    used=('torquevector.physics.steering',),
                ),
                'dead_time_s': num('Aktuatortotzeit', 's', fixed=0.0, reason='Totzeit ' + NOT_IMPLEMENTED),
            },
            'Lenkaktuator (SteeringActuator)',
        ),
        'slip_control': Section(
            {
                'abs_enabled': flag(
                    'ABS: Bremsmoment je Rad bei Überschreiten des Ziel-Bremsschlupfs reduzieren',
                    binds='plant.slip_control.abs_enabled',
                    used=('torquevector.control.slip_control',),
                ),
                'asr_enabled': flag(
                    'ASR: Antriebsmoment je Rad bei Überschreiten des Ziel-Antriebsschlupfs reduzieren',
                    binds='plant.slip_control.asr_enabled',
                    used=('torquevector.control.slip_control',),
                ),
                'period_s': num(
                    'Abtastzeit des inneren Schlupfreglers (<= Regeltakt)',
                    's',
                    0,
                    0.04,
                    xlo=True,
                    binds='plant.slip_control.period_s',
                    used=('torquevector.simulation.sampled',),
                ),
                'abs_target_slip': num(
                    'Ziel-Bremsschlupf |kappa|',
                    '1',
                    0,
                    0.5,
                    xlo=True,
                    xhi=True,
                    binds='plant.slip_control.abs_target_slip',
                    used=('torquevector.control.slip_control',),
                ),
                'asr_target_slip': num(
                    'Ziel-Antriebsschlupf kappa',
                    '1',
                    0,
                    0.5,
                    xlo=True,
                    xhi=True,
                    binds='plant.slip_control.asr_target_slip',
                    used=('torquevector.control.slip_control',),
                ),
                'kp': num(
                    'Proportionalanteil auf den Reduktionsanteil je Schlupffehler',
                    '1',
                    0,
                    binds='plant.slip_control.kp',
                    used=('torquevector.control.slip_control',),
                ),
                'ki_per_s': num(
                    'Integralanteil auf den Reduktionsanteil je Schlupffehler und Sekunde',
                    '1/s',
                    0,
                    binds='plant.slip_control.ki_per_s',
                    used=('torquevector.control.slip_control',),
                ),
                'torque_rate_nm_s': num(
                    'Max. Änderungsrate des angelegten Radmoments im inneren Takt',
                    'N*m/s',
                    0,
                    xlo=True,
                    binds='plant.slip_control.torque_rate_nm_s',
                    used=('torquevector.control.slip_control',),
                ),
            },
            'Radindividuelle Schlupfregelung ABS/ASR (erste Version, Ersatzmodell ohne Druckmodulation)',
        ),
    },
    'Antriebsstrang, Bremse und Lenkaktuator',
)


def controller_header(kind):
    return {'type': enum('Reglertyp', (kind,), fixed=kind, reason='Dateityp'), **header()}


def yaw_reference(prefix):
    return Section(
        {
            'mu_source': enum(
                'Reibwert der Gierraten-Sollwertbegrenzung',
                ('tyre', 'explicit'),
                binds=f'controllers.{prefix}_reference_mu_source',
            ),
            'mu': num(
                'Expliziter Reibwert (nur bei mu_source=explicit)',
                '1',
                0.05,
                2,
                nullable=True,
                binds=f'controllers.{prefix}_reference_mu_explicit',
            ),
        },
        'Begrenzung der Sollgierrate auf friction_fraction*mu*g/v (driver.yaml)',
    )


P_CONTROLLER = Section(
    {
        **controller_header('p'),
        'gain_nm_per_rads': num(
            'Proportionalverstärkung Gierratenfehler -> Zusatzmoment',
            'N*m/(rad/s)',
            0,
            xlo=True,
            binds='controllers.proportional_gain',
            used=('torquevector.control.controllers.p',),
        ),
        'output': Section(
            {
                'yaw_moment_limit_nm': num(
                    'Ausgangsbegrenzung Zusatzgiermoment',
                    'N*m',
                    0,
                    xlo=True,
                    binds='controllers.p_yaw_limit_nm',
                    used=('torquevector.control.pipeline',),
                )
            },
            'Ausgang',
        ),
        'yaw_reference': yaw_reference('p'),
    },
    'P-Regler',
)

LQR_CONTROLLER = Section(
    {
        **controller_header('lqr'),
        'weights': Section(
            {
                'sideslip_scale_rad': num(
                    'Q = diag(1/s_beta^2, ...): Schwimmwinkel-Skala',
                    'rad',
                    0.005,
                    0.5,
                    binds='controllers.lqr.sideslip_scale_rad',
                    used=('torquevector.control.controllers.lqr',),
                ),
                'yaw_rate_scale_rads': num(
                    'Q: Gierraten-Skala',
                    'rad/s',
                    0.005,
                    1.0,
                    binds='controllers.lqr.yaw_rate_scale_rads',
                    used=('torquevector.control.controllers.lqr',),
                ),
                'moment_scale_nm': num(
                    'R = (1000/M_scale)^2: Momentenskala',
                    'N*m',
                    50,
                    10000,
                    binds='controllers.lqr.moment_scale_nm',
                    used=('torquevector.control.controllers.lqr',),
                ),
            },
            'Gewichtung über Skalen (Bryson); Q/R werden daraus berechnet',
        ),
        'scheduling': Section(
            {
                'min_speed_ms': num(
                    'Untere Geschwindigkeit des Gain-Schedulings',
                    'm/s',
                    3,
                    80,
                    binds='controllers.lqr.min_speed_ms',
                    used=('torquevector.control.controllers.lqr',),
                ),
                'max_speed_ms': num(
                    'Obere Geschwindigkeit des Gain-Schedulings',
                    'm/s',
                    3,
                    80,
                    binds='controllers.lqr.max_speed_ms',
                    used=('torquevector.control.controllers.lqr',),
                ),
                'speed_grid_ms': num(
                    'Rasterweite der Linearisierungsgeschwindigkeit',
                    'm/s',
                    0.1,
                    5,
                    binds='controllers.lqr.speed_grid_ms',
                    used=('torquevector.control.controllers.lqr',),
                ),
                'riccati': enum('Riccati-Gleichung', ('continuous_are',), fixed='continuous_are', reason=FROZEN),
            },
            'Linearisierung/Scheduling',
        ),
        'feedforward_weight': num(
            'Gewicht der modellbasierten Vorsteuerung',
            '1',
            0,
            1,
            binds='controllers.lqr.feedforward_weight',
            used=('torquevector.control.controllers.lqr',),
        ),
        'output': Section(
            {
                'yaw_moment_limit_nm': num(
                    'Ausgangsbegrenzung Zusatzgiermoment',
                    'N*m',
                    0,
                    xlo=True,
                    binds='controllers.lqr_yaw_limit_nm',
                    used=('torquevector.control.pipeline',),
                )
            },
            'Ausgang',
        ),
        'yaw_reference': yaw_reference('lqr'),
    },
    'LQR-Regler (LQRConfig)',
)


def predictive(prefix, kind, horizon_max):
    owner = f'torquevector.control.controllers.{"ltv_mpc" if kind == "ltv" else "nmpc"}'
    b = f'controllers.{kind}.'
    return {
        **controller_header(prefix),
        'sample_s': num(
            'Regelabtastzeit = Prädiktionsschritt (muss sampling.control_period_s entsprechen)',
            's',
            0.02,
            0.2,
            binds=b + 'sample_s',
            used=(owner,),
        ),
        'horizon_steps': integer(
            'Prädiktionshorizont in Takten', 2, horizon_max, binds=b + 'horizon_steps', used=(owner,)
        ),
        'weights': Section(
            {
                'beta_scale_rad': num(
                    'Zustandsgewicht Schwimmwinkel (1/s^2)',
                    'rad',
                    0.005,
                    0.5,
                    binds=b + 'beta_scale_rad',
                    used=(owner,),
                ),
                'yaw_scale_rads': num(
                    'Zustandsgewicht Gierrate (1/s^2)', 'rad/s', 0.005, 0.6, binds=b + 'yaw_scale_rads', used=(owner,)
                ),
                'moment_scale_nm': num(
                    'Normierung der Stellgröße', 'N*m', 100, 10000, binds=b + 'moment_scale_nm', used=(owner,)
                ),
                'input_weight': num(
                    'Stellgrößengewicht R', '1', 0, 100, xlo=True, binds=b + 'input_weight', used=(owner,)
                ),
                'slew_weight': num(
                    'Gewicht der Stellgrößenänderung', '1', 0, 100, binds=b + 'slew_weight', used=(owner,)
                ),
                'terminal_weight': num(
                    'Terminalgewicht (Faktor)', '1', 0.1, 10, binds=b + 'terminal_weight', used=(owner,)
                ),
                'state_soft_weight': num(
                    'Gewicht weicher Zustandsgrenzen', '1', 0, 10000, binds=b + 'state_soft_weight', used=(owner,)
                ),
            },
            'Gewichtungen (Q über Skalen, R, Terminal, weiche Grenzen)',
        ),
        'limits': Section(
            {
                'moment_limit_nm': num(
                    'Harte Momentengrenze', 'N*m', 100, 3000, binds=b + 'moment_limit_nm', used=(owner,)
                ),
                'moment_slew_nm_s': num(
                    'Harte Stellratengrenze', 'N*m/s', 100, 50000, binds=b + 'moment_slew_nm_s', used=(owner,)
                ),
                'beta_limit_rad': num(
                    'Weiche Schwimmwinkelgrenze', 'rad', 0.02, 0.5, binds=b + 'beta_limit_rad', used=(owner,)
                ),
                'yaw_limit_fraction': num(
                    'Weiche Gierratengrenze als Anteil von mu*g/v',
                    '1',
                    0.2,
                    1.5,
                    binds=b + 'yaw_limit_fraction',
                    used=(owner,),
                ),
                'mu_reference': num(
                    'Reibwert im Prädiktionsmodell und in Grenzen',
                    '1',
                    0.1,
                    1.5,
                    binds=b + 'mu_reference',
                    used=(owner,),
                ),
            },
            'Harte Stell- und weiche Zustandsgrenzen',
        ),
        'model_validity': Section(
            {
                'min_speed_ms': num('Untere Modellgültigkeit', 'm/s', 3, 80, binds=b + 'min_speed_ms', used=(owner,)),
                'max_speed_ms': num('Obere Modellgültigkeit', 'm/s', 3, 80, binds=b + 'max_speed_ms', used=(owner,)),
            },
            'Gültigkeitsbereich des Prädiktionsmodells',
        ),
    }


def friction_knowledge(kind):
    return Section(
        {
            'mode': enum(
                'Reibwertwissen: unknown = nur mu_reference; map_preview = Kartenreibwert entlang der aus dem '
                'aktuellen Zustand vorhergesagten Trajektorie',
                ('unknown', 'map_preview'),
                binds=f'controllers.{kind}_friction_preview',
                used=('torquevector.simulation.preview_adapter',),
            ),
        },
        'Kenntnis des Reglers über den Fahrbahnreibwert (die simulierte Fahrbahn ist in beiden Modi gleich)',
    )


LTV_CONTROLLER = Section(
    {
        **predictive('ltv_mpc', 'ltv', 70),
        'friction_knowledge': friction_knowledge('ltv'),
        'prediction_model': enum(
            'Prädiktionsmodell', ('linear_bicycle_ltv',), fixed='linear_bicycle_ltv', reason=FROZEN
        ),
        'preview': enum(
            'Sollwertvorschau',
            ('manoeuvre_preview',),
            fixed='manoeuvre_preview',
            reason='Stufenreferenz k+1 aus simulation.preview_adapter; Zeitkonstanten in driver.yaml',
        ),
        'solver': Section(
            {
                'method': enum('QP-Verfahren', ('SLSQP',), fixed='SLSQP', reason=FROZEN),
                'ftol': num('Abbruchtoleranz SLSQP', '1', fixed=1e-8, reason=FROZEN),
                'max_iterations': integer(
                    'Maximale Iterationszahl',
                    10,
                    500,
                    binds='controllers.ltv.max_iterations',
                    used=('torquevector.control.controllers.ltv_mpc',),
                ),
                'warm_start': enum('Warmstart', ('previous_command',), fixed='previous_command', reason=FROZEN),
                'failure_policy': enum(
                    'Fehlerbehandlung',
                    ('fail_closed',),
                    fixed='fail_closed',
                    reason='Solverfehler werden als FAIL_CLOSED gemeldet, kein stiller Fallback',
                ),
            },
            'Solver',
        ),
    },
    'LTV-MPC (LTVConfig)',
)

NMPC_CONTROLLER = Section(
    {
        **predictive('nmpc', 'nmpc', 40),
        'friction_knowledge': friction_knowledge('nmpc'),
        'prediction_model': Section(
            {
                'predictor': enum(
                    'Nichtlinearer Prädiktor', ('reduced_order_bicycle',), fixed='reduced_order_bicycle', reason=FROZEN
                ),
                'integrator': enum('Integrationsverfahren', ('rk4',), fixed='rk4', reason=FROZEN),
                'integration_step': enum(
                    'Integrationsschritt', ('equal_to_sample',), fixed='equal_to_sample', reason=FROZEN
                ),
                'tyre_model': enum(
                    'Reifenmodell im Prädiktor',
                    ('tanh_axle_saturation',),
                    fixed='tanh_axle_saturation',
                    reason=FROZEN + '; Steifigkeit aus Reifen-YAML, Grenze aus limits.mu_reference',
                ),
            },
            'Prädiktionsmodell',
        ),
        'solver': Section(
            {
                'backend': enum('NLP-Backend', ('casadi_ipopt',), fixed='casadi_ipopt', reason='einziger Adapter'),
                'max_iterations': integer(
                    'IPOPT max_iter',
                    5,
                    250,
                    binds='controllers.nmpc.max_iterations',
                    used=('torquevector.control.controllers.nmpc',),
                ),
                'tolerance': num(
                    'IPOPT tol',
                    '1',
                    1e-9,
                    1e-2,
                    binds='controllers.nmpc.tolerance',
                    used=('torquevector.control.controllers.nmpc',),
                ),
                'acceptable_tolerance_floor': num(
                    'acceptable_tol = max(floor, factor*tol): Untergrenze',
                    '1',
                    0,
                    1,
                    xlo=True,
                    binds='controllers.nmpc_solver.acceptable_tol_floor',
                    used=('torquevector.control.controllers.nmpc',),
                ),
                'acceptable_tolerance_factor': num(
                    'acceptable_tol = max(floor, factor*tol): Faktor',
                    '1',
                    1,
                    1e6,
                    binds='controllers.nmpc_solver.acceptable_tol_factor',
                    used=('torquevector.control.controllers.nmpc',),
                ),
                'print_level': integer(
                    'IPOPT print_level',
                    0,
                    12,
                    binds='controllers.nmpc_solver.print_level',
                    used=('torquevector.control.controllers.nmpc',),
                ),
                'print_time': flag(
                    'CasADi print_time',
                    binds='controllers.nmpc_solver.print_time',
                    used=('torquevector.control.controllers.nmpc',),
                ),
                'suppress_banner': flag(
                    'IPOPT-Banner unterdrücken (ipopt.sb)',
                    binds='controllers.nmpc_solver.suppress_banner',
                    used=('torquevector.control.controllers.nmpc',),
                ),
                'max_cpu_time_s': num(
                    'IPOPT max_cpu_time je Solve (null = kein Budget)',
                    's',
                    0,
                    xlo=True,
                    nullable=True,
                    binds='controllers.nmpc_solver.max_cpu_time_s',
                    used=('torquevector.control.controllers.nmpc',),
                ),
                'warm_start': enum(
                    'Warmstart',
                    ('shifted_previous_solution',),
                    fixed='shifted_previous_solution',
                    reason='verschobene Vorlösung, auf Stellgrenzen projiziert',
                ),
                'failure_policy': enum(
                    'Fehlerbehandlung/Fallback',
                    ('fail_closed',),
                    fixed='fail_closed',
                    reason='kein stiller Rückfall auf LQR/Null-Moment',
                ),
            },
            'CasADi/IPOPT',
        ),
    },
    'Reduced-Order-NMPC (NMPCConfig, NMPCSolverSettings)',
)

ALLOCATION = Section(
    {
        **header(),
        'method': enum(
            'Allokationsverfahren',
            ('bounded_linear_least_squares',),
            fixed='bounded_linear_least_squares',
            reason=FROZEN,
        ),
        'weights': Section(
            {
                'yaw_weight': num(
                    'Gewicht Giermomentfolge',
                    '1',
                    0,
                    xlo=True,
                    binds='controllers.allocation.yaw_weight',
                    used=('torquevector.control.allocation',),
                ),
                'total_weight': num(
                    'Gewicht Summenmoment-Erhalt',
                    '1',
                    0,
                    xlo=True,
                    binds='controllers.allocation.total_weight',
                    used=('torquevector.control.allocation',),
                ),
                'intervention_penalty': num(
                    'Regularisierung der Radmomentänderung',
                    '1',
                    0,
                    xlo=True,
                    binds='controllers.allocation.intervention_penalty',
                    used=('torquevector.control.allocation',),
                ),
            },
            'Gewichte',
        ),
        'max_delta_nm': num(
            'Max. Momentänderung je Rad durch Torque Vectoring',
            'N*m',
            0,
            xlo=True,
            binds='controllers.allocation.max_delta_nm',
            used=('torquevector.control.allocation',),
        ),
        'friction_limit': Section(
            {
                'mode': enum(
                    'Kraftschlussgrenze je Rad: off = nur Aktuatorgrenzen (eingefrorener allocate_yaw); '
                    'friction_aware = zusätzlich T_i <= R*reserve*mu*Fz_i',
                    ('off', 'friction_aware'),
                    binds='controllers.allocation_friction_mode',
                    used=('torquevector.control.allocation_friction',),
                ),
                'reserve_fraction': num(
                    'Anteil von mu*Fz für Längskraft (Rest als Seitenkraftreserve)',
                    '1',
                    0,
                    1,
                    xlo=True,
                    binds='controllers.allocation_reserve_fraction',
                    used=('torquevector.control.allocation_friction',),
                ),
                'mu_source': enum(
                    'Reibwertannahme der Grenze: tyre = Reifen-mu (Fahrbahn unbekannt), explicit, road_map = '
                    'Kartenreibwert am aktuellen Aufstandspunkt je Rad',
                    ('tyre', 'explicit', 'road_map'),
                    binds='controllers.allocation_mu_source',
                ),
                'mu': num(
                    'Expliziter Reibwert (nur mu_source=explicit)',
                    '1',
                    0.05,
                    2,
                    nullable=True,
                    binds='controllers.allocation_mu_explicit',
                ),
            },
            'Reibwert- und radlastbewusste Momentgrenze (Q11.2)',
        ),
        'legacy_four_wheel_controller': Section(
            {
                'mu_reference': num(
                    'Reibwert (nur historischer FourWheelYawController)',
                    '1',
                    0,
                    xlo=True,
                    binds='controllers.allocation.mu_reference',
                    used=('torquevector.control.controllers.legacy_fourwheel',),
                ),
                'yaw_kp_nm_per_rads': num(
                    'Verstärkung (nur historischer FourWheelYawController)',
                    'N*m/(rad/s)',
                    0,
                    xlo=True,
                    binds='controllers.allocation.yaw_kp_nm_per_rads',
                    used=('torquevector.control.controllers.legacy_fourwheel',),
                ),
            },
            'Felder von AllocationOptions, die nur der historische Regler liest',
        ),
        'solver': Section(
            {
                'method': enum('lsq_linear-Methode', ('bvls',), fixed='bvls', reason=FROZEN),
                'tolerance': num('lsq_linear tol', '1', fixed=1e-10, reason=FROZEN),
                'max_iterations': integer('lsq_linear max_iter', fixed=100, reason=FROZEN),
            },
            'Solver',
        ),
    },
    'Torque Allocation (AllocationOptions)',
)

INTEGRATOR = Section(
    {
        **header(),
        'method': enum(
            'ODE-Verfahren je ZOH-Intervall',
            ('DOP853', 'RK45', 'Radau'),
            binds='plant.sampled.method',
            used=('torquevector.simulation.sampled',),
        ),
        'rtol': num(
            'Relative Toleranz', '1', 1e-10, 1e-3, binds='plant.sampled.rtol', used=('torquevector.simulation.sampled',)
        ),
        'atol': num(
            'Absolute Toleranz', '1', 1e-14, 1e-3, binds='plant.sampled.atol', used=('torquevector.simulation.sampled',)
        ),
        'max_step_s': num(
            'Maximale Integrationsschrittweite (<= Regelperiode)',
            's',
            0,
            xlo=True,
            binds='plant.sampled.max_step_s',
            used=('torquevector.simulation.sampled',),
        ),
        'max_rhs_calls_per_period': integer(
            'Abbruchgrenze RHS-Aufrufe je Regelperiode',
            30,
            None,
            binds='plant.sampled.max_rhs_per_period',
            used=('torquevector.simulation.sampled',),
        ),
        'load_transfer_solver': enum(
            'Löser der quasi-statischen Last-/Beschleunigungskopplung',
            ('root', 'fixed_point'),
            binds='plant.load_solver',
            used=('torquevector.physics.wheel_rotation',),
        ),
    },
    'Numerische Integration der Pflanze',
)

SAMPLING = Section(
    {
        **header(),
        'control_period_s': num(
            'Regelabtastzeit der ZOH-Schleife (alle Regler)',
            's',
            0.025,
            0.5,
            binds='plant.sampled.period_s',
            used=('torquevector.simulation.sampled',),
        ),
        'hold': enum(
            'Halteglied',
            ('zero_order_hold',),
            fixed='zero_order_hold',
            reason='Online ausschließlich getaktete ZOH-Simulation',
        ),
        'telemetry_alignment': enum(
            'Telemetriezeitpunkt',
            ('start_of_hold_interval',),
            fixed='start_of_hold_interval',
            reason='Ausgabefrequenz = Regelfrequenz; Abtastung zu Beginn jedes Halteintervalls',
        ),
    },
    'Abtastung und Telemetrietakt',
)

LIMITS = Section(
    {
        **header(),
        'command_slew': Section(
            {
                'wheel_torque_rate_nm_s': num(
                    'Harte Änderungsrate je Radmoment (Antrieb/Bremse) in der ZOH-Schleife',
                    'N*m/s',
                    0,
                    xlo=True,
                    binds='plant.sampled.max_torque_rate_nm_s',
                    used=('torquevector.simulation.sampled',),
                ),
                'steer_command_rate_deg_s': num(
                    'Harte Änderungsrate des Lenkkommandos',
                    'deg/s',
                    0,
                    xlo=True,
                    binds='plant.sampled.max_steer_rate_command_deg_s',
                    used=('torquevector.simulation.sampled',),
                ),
            },
            'Stellgrößen-Slew-Limits',
        ),
    },
    'Stellgrößengrenzen der Simulationsschleife',
)

DRIVER = Section(
    {
        **header(),
        'base_policy': Section(
            {
                'max_base_wheel_torque_nm': num(
                    'Begrenzung des Fahrer-Grundmoments je Hinterrad vor der Verteilung',
                    'N*m',
                    0,
                    xlo=True,
                    binds='driver.max_drive_torque_nm',
                    used=('torquevector.simulation.driving',),
                ),
                'step_steer_ramp_s': num(
                    'Übergangszeit des Lenksprungs (Smoothstep)',
                    's',
                    0,
                    xlo=True,
                    binds='driver.step_steer_ramp_s',
                    used=('torquevector.simulation.driving',),
                ),
            },
            'Reglerunabhängiger Fahrer (Bahnfolge, Geschwindigkeit)',
        ),
        'yaw_reference': Section(
            {
                'source': enum(
                    'Quelle der Sollgierrate: path_curvature = v*kappa der Sollbahn (bis V12); driver_steering = '
                    'kinematische Fahrerabsicht v*tan(delta)/L (unterstützt Lenkkorrekturen statt sie zu bekämpfen)',
                    ('path_curvature', 'driver_steering'),
                    binds='driver.reference_source',
                    used=('torquevector.simulation.configured', 'torquevector.simulation.preview_adapter'),
                ),
                'engage_ramp_s': num(
                    'Einblendzeit der Sollgierrate (Sollwertfilter)',
                    's',
                    0,
                    xlo=True,
                    binds='driver.reference_engage_ramp_s',
                    used=('torquevector.simulation.policies', 'torquevector.simulation.preview_adapter'),
                ),
                'friction_fraction': num(
                    'Anteil von mu*g/v als Sollgierratengrenze',
                    '1',
                    0,
                    1.5,
                    xlo=True,
                    binds='driver.reference_friction_fraction',
                    used=('torquevector.control.pipeline', 'torquevector.simulation.preview_adapter'),
                ),
            },
            'Gemeinsame Sollwertbildung für alle Regler',
        ),
        'preview': Section(
            {
                'speed_time_constant_s': num(
                    'Zeitkonstante der Geschwindigkeitsvorschau',
                    's',
                    0,
                    xlo=True,
                    binds='driver.preview_speed_time_constant_s',
                    used=('torquevector.simulation.preview_adapter',),
                ),
            },
            'Vorschau für prädiktive Regler',
        ),
    },
    'Fahrer- und Sollwertmodell',
)

INITIAL_STATE = Section(
    {
        'x_m': num('Startposition x', 'm', binds='scenario.initial_state.x_m'),
        'y_m': num('Startposition y', 'm', binds='scenario.initial_state.y_m'),
        'yaw_rad': num('Start-Gierwinkel', 'rad', -3.2, 3.2, binds='scenario.initial_state.yaw_rad'),
        'vy_ms': num('Start-Quergeschwindigkeit', 'm/s', -5, 5, binds='scenario.initial_state.vy_ms'),
        'yaw_rate_rads': num('Start-Gierrate', 'rad/s', -2, 2, binds='scenario.initial_state.yaw_rate_rads'),
    },
    'Anfangszustand; vx = manoeuvre.speed_ms, Räder frei rollend',
)

SCENARIO = Section(
    {
        **header(),
        'manoeuvre': Section(
            {
                'kind': enum(
                    'Manövertyp',
                    (
                        'circle_open',
                        'circle_track',
                        'double_lane_change',
                        'step_steer',
                        'sine_steer',
                        'straight',
                        'ramp_steer',
                    ),
                    binds='scenario.spec.kind',
                    used=('torquevector.domain.scenarios', 'torquevector.simulation.driving'),
                ),
                'speed_ms': num(
                    'Sollgeschwindigkeit (und Anfangsgeschwindigkeit, falls initial_speed_ms null)',
                    'm/s',
                    5,
                    40,
                    binds='scenario.spec.speed_ms',
                ),
                'initial_speed_ms': num(
                    'Anfangsgeschwindigkeit; null = speed_ms',
                    'm/s',
                    3.5,
                    40,
                    nullable=True,
                    required=False,
                    binds='scenario.initial_speed_ms',
                ),
                'duration_s': num(
                    'Definitionsdauer des Manövers (Standard-Simulationsdauer)',
                    's',
                    0,
                    30,
                    xlo=True,
                    binds='scenario.spec.duration_s',
                ),
                'path': Section(
                    {
                        'radius_m': num('Kreisradius (vorzeichenbehaftet)', 'm', binds='scenario.spec.radius_m'),
                        'offset_m': num('Spurversatz Doppelspurwechsel', 'm', binds='scenario.spec.offset_m'),
                        'entry_m': num('Einfahrstrecke', 'm', 0, binds='scenario.spec.entry_m'),
                        'transition_m': num('Übergangslänge', 'm', 10, binds='scenario.spec.transition_m'),
                        'plateau_m': num('Plateaulänge', 'm', 0, binds='scenario.spec.plateau_m'),
                    },
                    'Bahngeometrie',
                ),
                'steering': Section(
                    {
                        'amplitude_deg': num(
                            'Lenkamplitude (Sprung/Sinus) bzw. Endlenkwinkel der Rampe (ramp_steer)',
                            'deg',
                            binds='scenario.spec.steer_amplitude_deg',
                        ),
                        'frequency_hz': num(
                            'Sinusfrequenz', 'Hz', 0, 3, xlo=True, binds='scenario.spec.steer_frequency_hz'
                        ),
                        'excitation_start_s': num(
                            'Beginn der Lenkanregung', 's', 0, binds='scenario.spec.excitation_start_s'
                        ),
                        'max_steer_deg': num(
                            'Lenkwinkelbegrenzung des Fahrers',
                            'deg',
                            0,
                            25,
                            xlo=True,
                            binds='scenario.spec.max_steer_deg',
                        ),
                    },
                    'Lenkprofil',
                ),
                'path_following': Section(
                    {
                        'cross_gain': num('Querfehlerverstärkung', '1/m', 0, 10, binds='scenario.spec.cross_gain'),
                        'heading_gain': num('Kursfehlerverstärkung', '1', 0, 5, binds='scenario.spec.heading_gain'),
                    },
                    'Bahnfolgeregler des Fahrers',
                ),
                'speed_control': Section(
                    {
                        'kp_nm_per_ms': num(
                            'Geschwindigkeitsregler-Verstärkung',
                            'N*m/(m/s)',
                            0,
                            250,
                            binds='scenario.spec.speed_kp_nm_per_ms',
                        ),
                    },
                    'Längsführung',
                ),
            },
            'Manöver (ManoeuvreSpec)',
        ),
        'simulation_duration_s': num(
            'Simulationsdauer; null = manoeuvre.duration_s (kürzere Smoke-Läufe zulässig)',
            's',
            0,
            30,
            xlo=True,
            nullable=True,
            binds='scenario.duration_s',
        ),
        'initial_state': INITIAL_STATE,
        'road': Section(
            {
                'model': enum(
                    'Fahrbahnmodell: räumliche Reibwertkarte (road-Komponente) oder historische zeit-/seitenbasierte '
                    'FrictionMap',
                    ('road_surface', 'friction_map'),
                    binds='scenario.road_model',
                    used=('torquevector.physics.road_friction', 'torquevector.physics.road'),
                ),
                'surface': Field(
                    'path',
                    'Fahrbahn-YAML dieses Szenarios (relativ); ersetzt includes.road, außer CLI --road',
                    required=False,
                    nullable=True,
                ),
                'left_mu': num(
                    'Reibwert linke Fahrzeugseite (ersetzt Reifen-mu)',
                    '1',
                    0.05,
                    2,
                    nullable=True,
                    binds='scenario.road.left_mu',
                ),
                'right_mu': num(
                    'Reibwert rechte Fahrzeugseite (ersetzt Reifen-mu)',
                    '1',
                    0.05,
                    2,
                    nullable=True,
                    binds='scenario.road.right_mu',
                ),
                'step_time_s': num(
                    'Zeitpunkt des Reibwertsprungs (null = kein Sprung)',
                    's',
                    0,
                    nullable=True,
                    binds='scenario.road.after_s',
                ),
                'step_mu': num(
                    'Reibwert nach dem Sprung (beide Seiten)',
                    '1',
                    0.05,
                    2,
                    nullable=True,
                    binds='scenario.road.after_mu',
                ),
            },
            'Fahrbahn: räumliche Reibwertkarte oder historische FrictionMap (Felder left_mu…step_mu nur dort)',
        ),
        'brake_phase': Section(
            {
                'start_s': num('Beginn der Bremsphase', 's', 0, binds='scenario.brake_phase.start_s'),
                'ramp_s': num('Smoothstep-Übergang', 's', 0, xlo=True, binds='scenario.brake_phase.ramp_s'),
                'brake_torque_nm': wheels(
                    'Bremsmoment je Rad nach dem Übergang', 'N*m', 0, binds='scenario.brake_phase.brake_nm'
                ),
                'cut_drive': flag(
                    'Antriebsmoment während des Übergangs ausblenden', binds='scenario.brake_phase.cut_drive'
                ),
            },
            'Optionale Bremsphase (Bremsen in der Kurve)',
            required=False,
            nullable=True,
        ),
        'drive_phase': Section(
            {
                'start_s': num('Beginn der Antriebsphase', 's', 0, binds='scenario.drive_phase.start_s'),
                'ramp_s': num(
                    'Smoothstep-Überblendung vom Fahrerwunsch', 's', 0, xlo=True, binds='scenario.drive_phase.ramp_s'
                ),
                'drive_torque_nm': wheels(
                    'Antriebsmoment je Rad nach dem Übergang', 'N*m', 0, binds='scenario.drive_phase.drive_nm'
                ),
            },
            'Optionale Antriebsphase mit festen Radmomenten (z. B. Split-mu-Beschleunigung)',
            required=False,
            nullable=True,
        ),
        'envelope': Section(
            {
                'check_final_speed': flag(
                    'Endgeschwindigkeit gegen Sollgeschwindigkeit prüfen (bei Bremsmanövern false)',
                    binds='scenario.check_final_speed',
                ),
            },
            'Szenariospezifische Gültigkeitsprüfung',
        ),
    },
    'Fahrmanöver (reglerunabhängig)',
)

REGION_FRICTION = Section(
    {
        'material': text('Oberflächenmaterial (Kennung)'),
        'mu': num('Fahrbahnreibwert', '1', 0.05, 2),
    },
    'Material und Reibwert',
    required=False,
    nullable=True,
)

ROAD_REGION = Section(
    {
        'id': text('Eindeutige Regions-ID'),
        'type': enum('Regionstyp', ('rectangle', 'longitudinal_segment', 'split_mu')),
        'material': text('Material (rectangle, longitudinal_segment)', required=False, nullable=True),
        'mu': num('Reibwert (rectangle, longitudinal_segment)', '1', 0.05, 2, nullable=True, required=False),
        'bounds': Section(
            {
                'x_min_m': num('untere x-Grenze (inklusiv; null = unbegrenzt)', 'm', nullable=True, required=False),
                'x_max_m': num('obere x-Grenze (exklusiv; null = unbegrenzt)', 'm', nullable=True, required=False),
                'y_min_m': num('untere y-Grenze (inklusiv; null = unbegrenzt)', 'm', nullable=True, required=False),
                'y_max_m': num('obere y-Grenze (exklusiv; null = unbegrenzt)', 'm', nullable=True, required=False),
            },
            'Grenzen im globalen Koordinatensystem',
        ),
        'split_y_m': num('Trennlinie split_mu (links: y >= Wert)', 'm', nullable=True, required=False),
        'left': REGION_FRICTION,
        'right': REGION_FRICTION,
    },
    'Fahrbahnregion; spätere Regionen haben Vorrang',
)

ROAD = Section(
    {
        **header(),
        'id': Field('str', 'Eindeutige Kennung der Fahrbahn', binds=('scenario.surface.surface.id',)),
        'coordinate_system': Section(
            {
                'x_axis': enum('x-Achse', ('forward',), fixed='forward', reason='globales System: x in Startrichtung'),
                'y_axis': enum('y-Achse', ('left',), fixed='left', reason='rechtshändig, y nach links'),
                'units': enum('Längeneinheit', ('meters',), fixed='meters', reason='SI'),
            },
            'Koordinatensystem (Fahrzeugstart im Ursprung mit Gierwinkel des Szenarios)',
        ),
        'default_surface': Section(
            {
                'material': Field(
                    'str', 'Material außerhalb aller Regionen', binds=('scenario.surface.surface.default_material',)
                ),
                'mu': num(
                    'Reibwert außerhalb aller Regionen', '1', 0.05, 2, binds='scenario.surface.surface.default_mu'
                ),
            },
            'Grundbelag',
        ),
        'regions': Field(
            'list',
            'Regionen (Reihenfolge = Priorität, spätere gewinnen)',
            item=ROAD_REGION,
            binds=('scenario.surface.surface.regions',),
        ),
        'transition': Section(
            {
                'mode': enum(
                    'Übergang an Regionsgrenzen', ('sharp', 'smooth'), binds='scenario.surface.surface.transition'
                ),
                'smoothing_distance_m': num(
                    'Breite des Smoothstep-Übergangs (0 bei sharp)',
                    'm',
                    0,
                    binds='scenario.surface.surface.smoothing_distance_m',
                ),
            },
            'Reibwertübergänge',
        ),
        'friction_model': Section(
            {
                'mode': enum(
                    'Kopplung Fahrbahn -> Reifen',
                    ('scale_tyre_mu', 'replace_tyre_mu'),
                    binds='scenario.surface.surface.friction_model',
                    used=('torquevector.physics.road_friction',),
                ),
                'reference_mu': num(
                    'Fahrbahnreibwert, auf dem die Reifenparameter gelten (scale_tyre_mu)',
                    '1',
                    0.05,
                    2,
                    binds='scenario.surface.surface.reference_mu',
                ),
            },
            'mu_Reifen,eff = mu_Reifen * mu_Fahrbahn / reference_mu bzw. = mu_Fahrbahn',
        ),
    },
    'Fahrbahnoberfläche mu(x, y)',
)

LIVE_PLOTS = Section(
    {
        **header(),
        'enabled': flag(
            'Interaktives Fenster öffnen (false = nur Headless-Export)', binds='visualization.live.enabled'
        ),
        'figure_size_in': Field(
            'float_list',
            'Fenstergröße',
            unit='in',
            length=2,
            minimum=1,
            maximum=60,
            binds=('visualization.live.figure_size_in',),
        ),
        'integration_step_s': num(
            'Schrittweite der Schlupf-Simulation im Live-Dashboard',
            's',
            0,
            0.05,
            xlo=True,
            binds='visualization.live.dt_s',
            used=('torquevector.cli.dashboard',),
        ),
        'show_four_wheel_forces': flag('Kammsche Kreise der vier Räder', fixed=True, reason='festes Dashboard-Layout'),
    },
    'Live-Dashboard (Slider-Ansicht)',
)

PANELS = ('trajectory', 'yaw_rate', 'sideslip', 'tyre_utilization', 'torque_allocation')
REPORT_SECTIONS = (
    'overview',
    'kpi',
    'tracking',
    'allocation',
    'wheels',
    'friction_circle',
    'handling',
    'solver',
    'tyre',
    'road_map',
    'road_mu',
)
STATIC_PLOTS = Section(
    {
        **header(),
        'panels': Field(
            'str_list',
            'Diagrammauswahl und Reihenfolge (2x2-Raster bzw. Zeilen)',
            choices=PANELS,
            binds=('visualization.chart.panels',),
        ),
        'figure_size_in': Field(
            'float_list',
            'Diagrammgröße',
            unit='in',
            length=2,
            minimum=1,
            maximum=60,
            binds=('visualization.chart.figure_size_in',),
        ),
        'dpi': integer('Auflösung', 50, 600, binds='visualization.chart.dpi'),
        'formats': Field('str_list', 'Exportformate', choices=('png', 'svg'), binds=('visualization.chart.formats',)),
        'file_stem': Field(
            'str', 'Dateiname ohne Endung (unter results/<run_id>/plots/)', binds=('visualization.chart_file_stem',)
        ),
        'legend': flag('Legenden anzeigen', binds='visualization.chart.legend'),
        'grid_alpha': num('Gitterdeckkraft', '1', 0, 1, binds='visualization.chart.grid_alpha'),
        'colors': Field(
            'str_list',
            'Farbfolge je Regler (null = Matplotlib-Zyklus)',
            nullable=True,
            binds=('visualization.chart.colors',),
        ),
        'utilization_ylim_max': num(
            'Obere Achsgrenze Reifenausnutzung', '1', 0, 5, xlo=True, binds='visualization.chart.utilization_ylim_max'
        ),
        'auto_report': flag(
            'Nach jedem simulate/compare automatisch Bericht (PNG + report.pdf) erzeugen',
            binds='visualization.auto_report',
        ),
        'report_sections': Field(
            'str_list',
            'Abschnitte des Analyseberichts (Reihenfolge)',
            choices=REPORT_SECTIONS,
            binds=('visualization.report_sections',),
        ),
        'provenance_label': enum(
            'Pflichtkennzeichnung im Titel',
            ('SYNTHETIC | NOT VEHICLE VALIDATED',),
            fixed='SYNTHETIC | NOT VEHICLE VALIDATED',
            reason='Kennzeichnung darf nicht entfernt werden',
        ),
    },
    'Statische Vergleichsdiagramme aus gespeicherter Telemetrie',
)

GIF_EXPORT = Section(
    {
        **header(),
        'fps': integer('Bildrate', 4, 30, binds='visualization.animation.fps'),
        'max_frames': integer(
            'Maximale Framezahl (Ausdünnung, kein Kürzen)', 10, 400, binds='visualization.animation.max_frames'
        ),
        'tail_s': num('Haltezeit des letzten Frames', 's', 0, 5, binds='visualization.animation.tail_seconds'),
        'dpi': integer('Auflösung', 60, 200, binds='visualization.animation.dpi'),
        'figure_size_in': Field(
            'float_list',
            'Diagrammgröße',
            unit='in',
            length=2,
            minimum=1,
            maximum=60,
            binds=('visualization.animation.figure_size_in',),
        ),
        'colors': Field(
            'str_list',
            'Farbfolge je Regler (null = Matplotlib-Zyklus)',
            nullable=True,
            binds=('visualization.animation.colors',),
        ),
        'file_name': Field(
            'str', 'Dateiname unter results/<run_id>/animations/', binds=('visualization.animation_file_name',)
        ),
        'panels': Field(
            'str_list',
            'Feste Panels',
            choices=('trajectory', 'yaw_rate', 'sideslip', 'torque_allocation_and_utilization'),
            fixed=['trajectory', 'yaw_rate', 'sideslip', 'torque_allocation_and_utilization'],
            fixed_reason='GIF-Layout fest; Auswahl ' + NOT_IMPLEMENTED,
        ),
    },
    'GIF-Export aus gespeicherter Telemetrie (AnimationConfig)',
)

QUALITY_GATES = Section(
    {
        **header(),
        'manifest_resource': path('Autoritatives Gate-Manifest (Paketressource)'),
        'gates': Field('str_list', 'Gate-IDs; muss exakt dem Manifest entsprechen'),
        'min_points_per_gate': integer(
            'Mindestpunkte je Gate', fixed=8, reason='Q01-Q13 unverändert (docs/quality/QUALITY_GATES.md)'
        ),
        'critical_check_requires_points': integer(
            'Punkte je kritischer Prüfung', fixed=2, reason='Q01-Q13 unverändert'
        ),
        'status': Section(
            {
                'Q10': enum(
                    'Q10-Status', ('REQUIRED_REAL_DATA',), fixed='REQUIRED_REAL_DATA', reason='keine Realdaten'
                ),
                'Q11': enum('Q11-Status', ('BLOCKED',), fixed='BLOCKED', reason='Bahnfehler-/Robustheitsdefizite'),
            },
            'Unveränderliche Gate-Zustände',
        ),
    },
    'Read-only-Spiegel der Quality-Gates; Abweichungen zum Manifest sind Fehler',
)

THRESHOLDS = Section(
    {
        **header(),
        'envelope': Section(
            {
                'max_abs_slip': num(
                    'Max. |kappa| für PASS',
                    '1',
                    0,
                    1,
                    xlo=True,
                    binds='validation.envelope.max_abs_slip',
                    used=('torquevector.evaluation.envelope',),
                ),
                'max_tyre_utilization': num(
                    'Max. Reifenausnutzung für PASS',
                    '1',
                    0,
                    1.5,
                    xlo=True,
                    binds='validation.envelope.max_tyre_utilization',
                    used=('torquevector.evaluation.envelope',),
                ),
                'max_abs_beta_deg': num(
                    'Max. |Schwimmwinkel| für PASS',
                    'deg',
                    0,
                    90,
                    xlo=True,
                    binds='validation.envelope.max_abs_beta_deg',
                    used=('torquevector.evaluation.envelope',),
                ),
                'max_final_speed_deviation_ms': num(
                    'Max. Abweichung der Endgeschwindigkeit',
                    'm/s',
                    0,
                    xlo=True,
                    binds='validation.envelope.max_final_speed_deviation_ms',
                    used=('torquevector.evaluation.envelope',),
                ),
            },
            'Modell-Gültigkeitsprüfung je Lauf (INVALID_MODEL_ENVELOPE)',
        ),
        'qualification': Section(
            {
                'min_yaw_improvement_pct': num(
                    'Mindestverbesserung Gierraten-RMSE ggü. OFF',
                    '%',
                    binds='validation.qualification.min_yaw_improvement_pct',
                    used=('torquevector.optimization.configured_tuning',),
                ),
                'min_path_improvement_pct': num(
                    'Mindestverbesserung Bahn-RMSE ggü. OFF (negativ = zulässige Verschlechterung)',
                    '%',
                    binds='validation.qualification.min_path_improvement_pct',
                    used=('torquevector.optimization.configured_tuning',),
                ),
            },
            'Projektkriterium Q11 (keine Freigabe)',
        ),
        'handling': Section(
            {
                'speeds_ms': Field(
                    'float_list',
                    'Geschwindigkeiten der Rampenlenkung (ISO 4138)',
                    unit='m/s',
                    minimum=5,
                    maximum=40,
                    binds=('validation.handling.speeds_ms',),
                    used_by=('torquevector.evaluation.handling',),
                ),
                'controllers': Field(
                    'str_list',
                    'Regler der Handling-Analyse',
                    choices=CONTROLLER_KINDS,
                    binds=('validation.handling.controllers',),
                ),
                'lateral_limit_estimate_factor': num(
                    'Endlenkwinkel der Rampe: Faktor auf die Reibgrenze mu*g (Ackermann-Anteil)',
                    '1',
                    0.5,
                    3,
                    binds='validation.handling.lateral_limit_estimate_factor',
                    used=('torquevector.evaluation.handling',),
                ),
                'ramp_margin_deg': num(
                    'Zusätzlicher Endlenkwinkel der Rampe für Eigenlenkanteil',
                    'deg',
                    0,
                    20,
                    binds='validation.handling.ramp_margin_deg',
                    used=('torquevector.evaluation.handling',),
                ),
                'driver_max_wheel_torque_nm': num(
                    'Fahrer-Grundmoment je Rad während der Rampe (Geschwindigkeit halten, ISO 4138)',
                    'N*m',
                    0,
                    xlo=True,
                    binds='validation.handling.driver_max_wheel_torque_nm',
                    used=('torquevector.cli.composition',),
                ),
                'speed_kp_nm_per_ms': num(
                    'Geschwindigkeitsregler-Verstärkung während der Rampe',
                    'N*m/(m/s)',
                    0,
                    250,
                    binds='validation.handling.speed_kp_nm_per_ms',
                    used=('torquevector.cli.composition',),
                ),
                'linear_limit_g': num(
                    'Obergrenze des linearen Bereichs für den Eigenlenkgradienten',
                    'g',
                    0,
                    1.5,
                    xlo=True,
                    binds='validation.handling.linear_limit_g',
                    used=('torquevector.evaluation.handling',),
                ),
                'gradient_bands_g': Field(
                    'float_list',
                    'Grenzen der a_y-Bänder für den bandweisen Eigenlenkgradienten',
                    unit='g',
                    minimum=0,
                    maximum=2,
                    binds=('validation.handling.gradient_bands_g',),
                    used_by=('torquevector.evaluation.handling',),
                ),
                'neutral_tolerance_deg_per_g': num(
                    'Toleranzband neutral (|K| darunter)',
                    'deg/g',
                    0,
                    5,
                    binds='validation.handling.neutral_tolerance_deg_per_g',
                    used=('torquevector.evaluation.handling',),
                ),
                'saturation_utilisation': num(
                    'Achsausnutzung, ab der eine Achse als gesättigt gilt (Grenzverhalten)',
                    '1',
                    0,
                    1,
                    xlo=True,
                    binds='validation.handling.saturation_utilisation',
                    used=('torquevector.evaluation.handling',),
                ),
                'stop_sideslip_deg': num(
                    'Abbruch der Rampe bei |Schwimmwinkel| (Stabilitätsverlust)',
                    'deg',
                    1,
                    45,
                    binds='validation.handling.stop_sideslip_deg',
                    used=('torquevector.evaluation.handling',),
                ),
                'gg_directions': integer(
                    'Richtungen der idealen g-g-Hüllkurve', 8, 720, binds='validation.handling.gg_directions'
                ),
                'gg_brake_in_turn_speed_ms': num(
                    'g-g-Läufe: Geschwindigkeit beim Bremsen in der Kurve',
                    'm/s',
                    5,
                    40,
                    binds='validation.handling.gg_brake_in_turn_speed_ms',
                ),
                'gg_brake_in_turn_nm': Field(
                    'float_list',
                    'g-g-Läufe: Bremsmomentstufen vorne je Rad',
                    unit='N*m',
                    minimum=0,
                    binds=('validation.handling.gg_brake_in_turn_nm',),
                ),
                'gg_rear_brake_fraction': num(
                    'g-g-Läufe: Bremsmoment hinten als Anteil von vorne',
                    '1',
                    0,
                    1,
                    binds='validation.handling.gg_rear_brake_fraction',
                ),
                'gg_straight_drive_nm': Field(
                    'float_list',
                    'g-g-Läufe: Antriebsmomentstufen je Rad (Geradeaus, trocken)',
                    unit='N*m',
                    minimum=0,
                    binds=('validation.handling.gg_straight_drive_nm',),
                ),
                'phase_plane': Section(
                    {
                        'speed_ms': num(
                            'Geschwindigkeit', 'm/s', 5, 40, binds='validation.handling.phase_plane.speed_ms'
                        ),
                        'steer_deg': num(
                            'Konstanter Radlenkwinkel',
                            'deg',
                            -25,
                            25,
                            binds='validation.handling.phase_plane.steer_deg',
                        ),
                        'sideslip_range_deg': num(
                            'Startschwimmwinkel ±',
                            'deg',
                            0,
                            30,
                            xlo=True,
                            binds='validation.handling.phase_plane.sideslip_range_deg',
                        ),
                        'yaw_rate_range_rads': num(
                            'Startgierrate ±',
                            'rad/s',
                            0,
                            3,
                            xlo=True,
                            binds='validation.handling.phase_plane.yaw_rate_range_rads',
                        ),
                        'grid_sideslip': integer(
                            'Gitterpunkte Schwimmwinkel', 2, 25, binds='validation.handling.phase_plane.grid_sideslip'
                        ),
                        'grid_yaw_rate': integer(
                            'Gitterpunkte Gierrate', 2, 25, binds='validation.handling.phase_plane.grid_yaw_rate'
                        ),
                        'duration_s': num(
                            'Dauer je Trajektorie',
                            's',
                            0,
                            10,
                            xlo=True,
                            binds='validation.handling.phase_plane.duration_s',
                        ),
                        'settled_yaw_acceleration_rads2': num(
                            'Eingeschwungen, wenn |dr/dt| am Ende darunter',
                            'rad/s^2',
                            0,
                            xlo=True,
                            binds='validation.handling.phase_plane.settled_yaw_acceleration_rads2',
                        ),
                    },
                    'Phasenebene Schwimmwinkel/Gierrate (Stabilitätsgrenze)',
                ),
            },
            'Eigenlenkverhalten und Fahrgrenzen (torquevector handling)',
        ),
    },
    'Validierungsschwellen',
)

MATRIX_CASE = Section(
    {
        'id': text('Fall-ID'),
        'scenario': path('Szenariodatei relativ zu dieser Datei'),
        'speed_ms': num('Geschwindigkeitsüberschreibung', 'm/s', 5, 40, nullable=True, required=False),
        'duration_s': num(
            'Simulationsdauer dieses Falls (scenario.simulation_duration_s)',
            's',
            0,
            30,
            xlo=True,
            nullable=True,
            required=False,
        ),
        'overrides': Field(
            'mapping',
            'Punktpfad-Overrides des aufgelösten Baums (z. B. vehicle.mass_kg)',
            required=False,
            nullable=True,
        ),
    },
    'Matrixfall',
)

SCENARIO_MATRIX = Section(
    {
        **header(),
        'sets': Field('mapping', 'Benannte Fallmengen: Name -> Liste von Fällen'),
    },
    'Szenariomatrix (Regression, Training, Hold-out, Robustheit)',
)

PSO = Section(
    {
        **header(),
        'algorithm': enum(
            'Verfahren', ('global_best_pso_normalized_box',), fixed='global_best_pso_normalized_box', reason=FROZEN
        ),
        'particles': integer(
            'Anzahl Partikel', 3, 1000, binds='pso.particles', used=('torquevector.optimization.pso',)
        ),
        'iterations': integer(
            'Anzahl Iterationen', 1, 10000, binds='pso.iterations', used=('torquevector.optimization.pso',)
        ),
        'seed': integer('Zufallsseed', 0, None, binds='pso.seed', used=('torquevector.optimization.pso',)),
        'cognitive': num(
            'Cognitive Coefficient', '1', 0, 3, binds='pso.cognitive', used=('torquevector.optimization.pso',)
        ),
        'social': num('Social Coefficient', '1', 0, 3, binds='pso.social', used=('torquevector.optimization.pso',)),
        'inertia_start': num(
            'Inertia Weight zu Beginn', '1', 0, 3, binds='pso.inertia_start', used=('torquevector.optimization.pso',)
        ),
        'inertia_end': num(
            'Inertia Weight am Ende', '1', 0, 3, binds='pso.inertia_end', used=('torquevector.optimization.pso',)
        ),
        'velocity_clamp': num('Geschwindigkeitsbegrenzung im normierten Raum', '1', fixed=0.3, reason=FROZEN),
        'boundary_handling': enum(
            'Grenzverletzungsbehandlung', ('clip_and_zero_velocity',), fixed='clip_and_zero_velocity', reason=FROZEN
        ),
        'parallel_workers': integer('Parallelisierung', fixed=1, reason='minimize_swarm ist sequenziell; ' + FROZEN),
        'default_study': path('Standard-Studie für optimize --config pso.yaml'),
    },
    'Offline-PSO (PSOSettings)',
)

SEARCH_DIMENSION = Section(
    {
        'parameter': enum(
            'Zu optimierender Reglerparameter',
            (
                'weights.beta_scale_rad',
                'weights.yaw_scale_rads',
                'weights.input_weight',
                'weights.slew_weight',
                'weights.terminal_weight',
                'weights.state_soft_weight',
                'weights.moment_scale_nm',
            ),
        ),
        'lower': num('Untere Grenze (Einheit des Zielparameters)', 'param'),
        'upper': num('Obere Grenze (Einheit des Zielparameters)', 'param'),
    },
    'Suchraum-Dimension',
)

TUNING = Section(
    {
        **header(),
        'controller': enum('Zu tunender Regler', ('ltv', 'nmpc')),
        'application': path('Basiskonfiguration (relativ)'),
        'algorithm': path('PSO-Einstellungen (relativ)'),
        'search_space': Field('list', 'Suchraum', item=SEARCH_DIMENSION),
        'initial_position': enum(
            'Startpartikel',
            ('current_controller_config',),
            fixed='current_controller_config',
            reason='Partikel 0 = aktuelle Regler-YAML',
        ),
        'objective': Section(
            {
                'formula': enum(
                    'Kostenfunktion',
                    ('weighted_squared_ratios',),
                    fixed='weighted_squared_ratios',
                    reason='sum w_i*(Kennzahl_i/Normierung_i)^2; Normierung yaw/path = OFF-Baseline',
                ),
                'weights': Section(
                    {
                        'yaw_rmse': num('Gewicht Gierraten-RMSE/OFF', '1', 0, binds='tuning.weights.yaw_rmse'),
                        'cross_rmse': num('Gewicht Bahn-RMSE/OFF', '1', 0, binds='tuning.weights.cross_rmse'),
                        'max_beta': num('Gewicht max. Schwimmwinkel', '1', 0, binds='tuning.weights.max_beta'),
                        'soft_state': num(
                            'Gewicht prädiktive Zustandsüberschreitung', '1', 0, binds='tuning.weights.soft_state'
                        ),
                        'allocation_residual': num(
                            'Gewicht Allokationsresiduum', '1', 0, binds='tuning.weights.allocation_residual'
                        ),
                    },
                    'Gewichte',
                ),
                'normalization': Section(
                    {
                        'max_beta_deg': num(
                            'Normierung Schwimmwinkel', 'deg', 0, xlo=True, binds='tuning.weights.beta_norm_deg'
                        ),
                        'soft_state_excess': num(
                            'Normierung Zustandsüberschreitung',
                            '1',
                            0,
                            xlo=True,
                            binds='tuning.weights.soft_state_norm',
                        ),
                        'allocation_residual_nm': num(
                            'Normierung Allokationsresiduum',
                            'N*m',
                            0,
                            xlo=True,
                            binds='tuning.weights.allocation_norm_nm',
                        ),
                    },
                    'Normierungen',
                ),
                'invalid_candidate_cost': num(
                    'Kosten ungültiger Kandidaten (harte Invalidierung)', '1', 1, binds='tuning.weights.invalid_cost'
                ),
            },
            'Gewichtete Kostenfunktion',
        ),
        'cases': Section(
            {
                'matrix': path('Szenariomatrix (relativ)'),
                'train': text('Trainingsmenge'),
                'holdout': text('Hold-out-Menge (nie im Optimierer)'),
                'robustness': text('Robustheitsmenge (nach Optimierung)', nullable=True),
            },
            'Manöver',
        ),
        'baseline_controller': enum('Referenzregler', ('off',), fixed='off', reason='Normierung gegen OFF'),
        'checkpoint': Section(
            {
                'enabled': flag('Kandidatenbewertungen fortlaufend speichern und bei --resume wiederverwenden'),
                'file_name': text('Checkpoint-Datei im Ergebnisordner'),
            },
            'Checkpointing (deterministische Wiederaufnahme über Seed + Bewertungscache)',
        ),
        'output': Section(
            {
                'root': path(
                    'Ergebniswurzel (relativ zum Arbeitsverzeichnis)', structural_default='results/optimization'
                ),
            },
            'Ausgabe',
        ),
    },
    'Offline-Abstimmungsstudie',
)

INCLUDES = Section(
    {
        'vehicle': path('Fahrzeug-YAML'),
        'tyre': path('Reifen-YAML'),
        'wheel': path('Rad-YAML'),
        'powertrain': path('Antriebsstrang-YAML'),
        'road': path('Fahrbahn-YAML (Standard; Szenario kann über road.surface ersetzen)'),
        'controllers': Section({k: path(f'{k}-Regler-YAML') for k in ('p', 'lqr', 'ltv_mpc', 'nmpc')}, 'Regler'),
        'allocation': path('Allocation-YAML'),
        'simulation': Section(
            {k: path(f'{k}-YAML') for k in ('integrator', 'sampling', 'limits', 'driver')}, 'Simulation'
        ),
        'scenario': path('Szenario-YAML'),
        'visualization': Section(
            {k: path(f'{k}-YAML') for k in ('live_plots', 'static_plots', 'gif_export')}, 'Visualisierung'
        ),
        'validation': Section(
            {k: path(f'{k}-YAML') for k in ('quality_gates', 'thresholds', 'scenario_matrix')}, 'Validierung'
        ),
    },
    'Referenzierte Komponenten (Pfade relativ zur Datei)',
)

RUN = Section(
    {
        'controllers': Field('str_list', 'Auszuführende Regler', choices=CONTROLLER_KINDS, binds=('run.controllers',)),
        'matrix_set': Field(
            'str', 'Name einer Matrixmenge (null = Einzelszenario)', nullable=True, binds=('run.matrix_set',)
        ),
        'results_dir': Field(
            'path',
            'Ergebniswurzel der Einzelläufe; CLI --results-dir und TORQUEVECTOR_RESULTS_DIR haben Vorrang',
            structural_default='results/runs',
            binds=('run.results_dir',),
        ),
        'telemetry': Section(
            {
                'format': enum(
                    'Telemetrieformat', ('json_lossless',), fixed='json_lossless', reason='einziger Adapter'
                ),
                'store_controller_diagnostics': flag(
                    'Prädiktive Solverdiagnostik je Takt speichern', binds='run.store_controller_diagnostics'
                ),
            },
            'Telemetrie',
        ),
    },
    'Laufsteuerung',
)

APPLICATION = Section(
    {
        'id': text('Konfigurations-ID'),
        'description': text('Beschreibung', required=False, nullable=True),
        'includes': INCLUDES,
        'run': RUN,
    },
    'Anwendungskonfiguration (Composition Root)',
)

PROFILE = Section(
    {
        'id': text('Profil-ID'),
        'description': text('Beschreibung', required=False, nullable=True),
        'base': path(
            'Basis-Anwendungskonfiguration (relativ), falls als --config verwendet', required=False, nullable=True
        ),
        'includes': Field(
            'mapping', 'Ersetzte Komponenten (Punktpfad -> Datei relativ zum Profil)', required=False, nullable=True
        ),
        'overrides': Field('mapping', 'Punktpfad -> Wert im aufgelösten Baum', required=False, nullable=True),
    },
    'Profil-Overlay',
)

# Abschnittsschlüssel -> (Schema, Pfad im aufgelösten Baum). Reglerdateien über ``controller.type``.
COMPONENTS = {
    'vehicle': (VEHICLE, ('vehicle',)),
    'tyre': (TYRE, ('tyre',)),
    'road': (ROAD, ('road',)),
    'wheel': (WHEEL, ('wheel',)),
    'powertrain': (POWERTRAIN, ('powertrain',)),
    'allocation': (ALLOCATION, ('allocation',)),
    'integrator': (INTEGRATOR, ('simulation', 'integrator')),
    'sampling': (SAMPLING, ('simulation', 'sampling')),
    'limits': (LIMITS, ('simulation', 'limits')),
    'driver': (DRIVER, ('simulation', 'driver')),
    'scenario': (SCENARIO, ('scenario',)),
    'live_plots': (LIVE_PLOTS, ('visualization', 'live_plots')),
    'static_plots': (STATIC_PLOTS, ('visualization', 'static_plots')),
    'gif_export': (GIF_EXPORT, ('visualization', 'gif_export')),
    'quality_gates': (QUALITY_GATES, ('validation', 'quality_gates')),
    'thresholds': (THRESHOLDS, ('validation', 'thresholds')),
    'scenario_matrix': (SCENARIO_MATRIX, ('validation', 'scenario_matrix')),
    'pso': (PSO, ('pso',)),
    'tuning': (TUNING, ('tuning',)),
}
CONTROLLERS = {
    'p': (P_CONTROLLER, ('controllers', 'p')),
    'lqr': (LQR_CONTROLLER, ('controllers', 'lqr')),
    'ltv_mpc': (LTV_CONTROLLER, ('controllers', 'ltv_mpc')),
    'nmpc': (NMPC_CONTROLLER, ('controllers', 'nmpc')),
}
RUNTIME_KINDS = tuple(k for k in COMPONENTS if k not in ('pso', 'tuning')) + ('controller',)


def tree_schema() -> Section:
    """Schema des vollständig aufgelösten Anwendungsbaums."""
    root = {
        'application': Section(
            {'id': text('Konfigurations-ID'), 'description': text('Beschreibung', required=False, nullable=True)}
        ),
        'run': RUN,
    }
    for kind, (schema, location) in list(COMPONENTS.items()) + list(CONTROLLERS.items()):
        if kind in ('pso', 'tuning'):
            continue
        node = root
        for key in location[:-1]:
            node = node.setdefault(key, {})
        node[location[-1]] = schema

    def wrap(mapping):
        return Section({k: (wrap(v) if isinstance(v, dict) else v) for k, v in mapping.items()})

    return wrap(root)
