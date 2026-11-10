"""Feldübergreifende Konsistenzprüfungen nach der Schema-Validierung.

Diese Regeln erkennen Konfigurationen, die einzeln gültige Werte enthalten,
zusammen aber widersprüchlich oder nicht ausführbar wären.
"""

from __future__ import annotations

import json
from pathlib import Path

from torquevector.infrastructure.config.schema import iter_fields, validate_section
from torquevector.infrastructure.config.schemas import CONTROLLERS, MATRIX_CASE
from torquevector.infrastructure.configuration import resource_path

SURROGATE_KEYS = ('alpha_scale_rad', 'kappa_scale', 'lateral_peak_fraction', 'longitudinal_peak_fraction')
VALIDATION_KEYS = ('validated_against_real_vehicle', 'experimentally_validated')


def _close(a, b, tol=1e-12):
    return abs(a - b) <= tol * max(1.0, abs(a), abs(b))


def check_provenance(section, where, lookup):
    issues = []
    prov = (section or {}).get('provenance') or {}
    source = prov.get('source_type', '')
    for key in VALIDATION_KEYS:
        if prov.get(key) is True and source.startswith('synthetic'):
            issues.append(
                f'{where}.provenance.{key}{lookup(where + ".provenance")}: synthetische Quelle darf nicht '
                f'als real validiert gekennzeichnet werden'
            )
    return issues


def check_vehicle(v, where, lookup):
    issues = check_provenance(v, where, lookup)
    g = v['geometry']
    if g['cg_to_front_axle_m'] >= g['wheelbase_m']:
        issues.append(
            f'{where}.geometry.cg_to_front_axle_m{lookup(where + ".geometry.cg_to_front_axle_m")}: '
            f'muss kleiner als wheelbase_m ({g["wheelbase_m"]}) sein'
        )
    return issues


def check_tyre(t, where, lookup):
    issues = check_provenance(t, where, lookup)
    cs = t['combined_slip']
    values = [cs.get(k) for k in SURROGATE_KEYS]
    if cs['model'] == 'elliptic_damping_surrogate' and any(v is None for v in values):
        issues.append(
            f'{where}.combined_slip{lookup(where + ".combined_slip")}: elliptic_damping_surrogate benötigt '
            f'{", ".join(SURROGATE_KEYS)}'
        )
    if cs['model'] == 'kamm_circle' and any(v is not None for v in values):
        issues.append(
            f'{where}.combined_slip{lookup(where + ".combined_slip")}: kamm_circle verwendet keine '
            f'Surrogat-Parameter; Werte auf null setzen (keine stillen Defaults)'
        )
    return issues


def check_powertrain(p, where, lookup, driver=None):
    issues = check_provenance(p, where, lookup)
    dist = p['drive_distribution']
    if not _close(sum(dist.values()), 1.0, 1e-9):
        issues.append(
            f'{where}.drive_distribution{lookup(where + ".drive_distribution")}: Summe muss 1 sein '
            f'(ist {sum(dist.values())!r})'
        )
    limit = p['wheel_torque_limit_nm']
    for key in ('drive_torque_nm', 'brake_torque_nm'):
        if max(p['initial_state'][key].values()) > limit:
            issues.append(f'{where}.initial_state.{key}: überschreitet wheel_torque_limit_nm {limit}')
    if driver is not None:
        peak = 2 * driver['base_policy']['max_base_wheel_torque_nm'] * max(dist.values())
        if peak > limit:
            issues.append(
                f'simulation.driver.base_policy.max_base_wheel_torque_nm: verteiltes Fahrer-Moment bis '
                f'{peak:g} N*m überschreitet wheel_torque_limit_nm {limit:g}'
            )
    return issues


def check_controller(kind, c, where, lookup):
    issues = check_provenance(c, where, lookup)
    if kind in ('p', 'lqr'):
        ref = c['yaw_reference']
        if (ref['mu_source'] == 'explicit') != (ref['mu'] is not None):
            issues.append(
                f'{where}.yaw_reference{lookup(where + ".yaw_reference")}: mu genau dann angeben, '
                f'wenn mu_source=explicit'
            )
    speeds = c.get('scheduling') or c.get('model_validity')
    if speeds and speeds['min_speed_ms'] >= speeds['max_speed_ms']:
        issues.append(f'{where}: min_speed_ms muss kleiner als max_speed_ms sein')
    return issues


def check_scenario(s, where, lookup, torque_limit=None):
    issues = check_provenance(s, where, lookup)
    road = s['road']
    friction = (road['left_mu'], road['right_mu'])
    step = (road['step_time_s'], road['step_mu'])
    if road['model'] == 'friction_map':
        if None in friction:
            issues.append(f'{where}.road{lookup(where + ".road")}: friction_map benötigt left_mu und right_mu')
        if (step[0] is None) != (step[1] is None):
            issues.append(f'{where}.road{lookup(where + ".road")}: step_time_s und step_mu nur gemeinsam angeben')
    elif any(v is not None for v in friction + step):
        issues.append(
            f'{where}.road{lookup(where + ".road")}: road_surface verwendet left_mu…step_mu nicht; '
            f'Werte null setzen (Fahrbahn über roads/*.yaml)'
        )
    if road['model'] == 'friction_map' and road.get('surface') is not None:
        issues.append(f'{where}.road.surface: nur mit road_surface zulässig')
    m = s['manoeuvre']
    if m['steering']['excitation_start_s'] >= m['duration_s']:
        issues.append(f'{where}.manoeuvre.steering.excitation_start_s: muss kleiner als duration_s sein')
    brake = s.get('brake_phase')
    if brake and torque_limit is not None and max(brake['brake_torque_nm'].values()) > torque_limit:
        issues.append(f'{where}.brake_phase.brake_torque_nm: überschreitet wheel_torque_limit_nm {torque_limit}')
    return issues


def check_road(r, where, lookup):
    issues = check_provenance(r, where, lookup)
    ids = set()
    for index, region in enumerate(r.get('regions') or []):
        loc = f'{where}.regions[{index}]'
        if region['id'] in ids:
            issues.append(f'{loc}{lookup(loc)}: doppelte Regions-ID {region["id"]!r}')
        ids.add(region['id'])
        b = region['bounds']
        for lo, hi, axis in (('x_min_m', 'x_max_m', 'x'), ('y_min_m', 'y_max_m', 'y')):
            if b.get(lo) is not None and b.get(hi) is not None and not b[lo] < b[hi]:
                issues.append(f'{loc}.bounds{lookup(loc)}: {lo} muss kleiner als {hi} sein')
        kind = region['type']
        simple = region.get('material') is not None and region.get('mu') is not None
        split = (
            region.get('left') is not None and region.get('right') is not None and region.get('split_y_m') is not None
        )
        if kind == 'split_mu':
            if not split or region.get('material') is not None or region.get('mu') is not None:
                issues.append(f'{loc}{lookup(loc)}: split_mu benötigt left, right, split_y_m (ohne material/mu)')
            elif b.get('y_min_m') is not None or b.get('y_max_m') is not None:
                issues.append(f'{loc}{lookup(loc)}: split_mu definiert y über split_y_m; y-Grenzen null setzen')
        else:
            if (
                not simple
                or region.get('left') is not None
                or region.get('right') is not None
                or region.get('split_y_m') is not None
            ):
                issues.append(f'{loc}{lookup(loc)}: {kind} benötigt material und mu (ohne left/right/split_y_m)')
            if kind == 'longitudinal_segment' and (b.get('y_min_m') is not None or b.get('y_max_m') is not None):
                issues.append(
                    f'{loc}{lookup(loc)}: longitudinal_segment gilt über die volle Breite; y-Grenzen null setzen'
                )
    t = r['transition']
    if (t['mode'] == 'smooth') != (t['smoothing_distance_m'] > 0):
        issues.append(
            f'{where}.transition{lookup(where + ".transition")}: smooth erfordert smoothing_distance_m > 0, '
            f'sharp erfordert 0'
        )
    return issues


def road_is_neutral(r) -> bool:
    return (
        not r.get('regions')
        and r['friction_model']['mode'] == 'scale_tyre_mu'
        and r['default_surface']['mu'] == r['friction_model']['reference_mu']
    )


def check_quality_gates(q, where, lookup):
    issues = check_provenance(q, where, lookup)
    try:
        manifest = json.loads(resource_path(q['manifest_resource']).read_text(encoding='utf-8'))
    except (OSError, ValueError) as exc:
        return issues + [f'{where}.manifest_resource: Manifest nicht lesbar ({exc})']
    ids = [c['id'] for c in manifest['categories']]
    if list(q['gates']) != ids:
        issues.append(
            f'{where}.gates{lookup(where + ".gates")}: weicht vom Manifest ab ({ids}); '
            f'Q01-Q13 dürfen nicht verändert werden'
        )
    if manifest['min_points_per_category'] != q['min_points_per_gate']:
        issues.append(f'{where}.min_points_per_gate: Manifest verlangt {manifest["min_points_per_category"]}')
    if manifest['critical_check_requires'] != q['critical_check_requires_points']:
        issues.append(
            f'{where}.critical_check_requires_points: Manifest verlangt {manifest["critical_check_requires"]}'
        )
    return issues


def check_visual(section, where, lookup):
    issues = check_provenance(section, where, lookup)
    if 'panels' in section and not section['panels']:
        issues.append(f'{where}.panels: mindestens ein Panel erforderlich')
    if 'panels' in section and len(section['panels']) > 6 and where.endswith('static_plots'):
        issues.append(f'{where}.panels: höchstens 6 Panels')
    if section.get('colors') is not None and not section['colors']:
        issues.append(f'{where}.colors: leere Liste unzulässig (null = Standardfarben)')
    return issues


def check_component(kind, section, lookup):
    where = kind
    if kind == 'vehicle':
        return check_vehicle(section, where, lookup)
    if kind == 'tyre':
        return check_tyre(section, where, lookup)
    if kind == 'road':
        return check_road(section, where, lookup)
    if kind == 'powertrain':
        return check_powertrain(section, where, lookup)
    if kind == 'controller':
        return check_controller(section['type'], section, 'controller', lookup)
    if kind == 'scenario':
        return check_scenario(section, where, lookup)
    if kind == 'quality_gates':
        return check_quality_gates(section, where, lookup)
    if kind in ('static_plots', 'gif_export', 'live_plots'):
        return check_visual(section, where, lookup)
    return check_provenance(section, where, lookup)


def check_tree(tree, lookup, warnings):
    issues = []
    issues += check_vehicle(tree['vehicle'], 'vehicle', lookup)
    issues += check_tyre(tree['tyre'], 'tyre', lookup)
    issues += check_provenance(tree['wheel'], 'wheel', lookup)
    issues += check_powertrain(tree['powertrain'], 'powertrain', lookup, tree['simulation']['driver'])
    for kind in CONTROLLERS:
        issues += check_controller(kind, tree['controllers'][kind], f'controllers.{kind}', lookup)
    period = tree['simulation']['sampling']['control_period_s']
    for kind in ('ltv_mpc', 'nmpc'):
        sample = tree['controllers'][kind]['sample_s']
        if not _close(sample, period):
            issues.append(
                f'controllers.{kind}.sample_s{lookup(f"controllers.{kind}.sample_s")}: {sample} widerspricht '
                f'simulation.sampling.control_period_s {period}{lookup("simulation.sampling.control_period_s")}'
            )
    if tree['simulation']['integrator']['max_step_s'] > period:
        issues.append(
            f'simulation.integrator.max_step_s{lookup("simulation.integrator.max_step_s")}: muss <= '
            f'control_period_s {period} sein'
        )
    for section in ('integrator', 'sampling', 'limits', 'driver'):
        issues += check_provenance(tree['simulation'][section], f'simulation.{section}', lookup)
    issues += check_provenance(tree['allocation'], 'allocation', lookup)
    fl = tree['allocation']['friction_limit']
    if (fl['mu_source'] == 'explicit') != (fl['mu'] is not None):
        issues.append(
            f'allocation.friction_limit{lookup("allocation.friction_limit")}: mu genau dann angeben, '
            f'wenn mu_source=explicit'
        )
    issues += check_scenario(tree['scenario'], 'scenario', lookup, tree['powertrain']['wheel_torque_limit_nm'])
    issues += check_road(tree['road'], 'road', lookup)
    if tree['scenario']['road']['model'] == 'friction_map':
        if not road_is_neutral(tree['road']):
            issues.append(
                f'scenario.road.model{lookup("scenario.road.model")}: friction_map und eine nicht neutrale '
                f'Fahrbahnkarte (road) würden den Reibwert doppelt festlegen'
            )
        for kind in ('ltv_mpc', 'nmpc'):
            if tree['controllers'][kind]['friction_knowledge']['mode'] == 'map_preview':
                issues.append(f'controllers.{kind}.friction_knowledge.mode: map_preview erfordert road_surface')
    if (
        tree['scenario'].get('drive_phase')
        and max(tree['scenario']['drive_phase']['drive_torque_nm'].values())
        > tree['powertrain']['wheel_torque_limit_nm']
    ):
        issues.append('scenario.drive_phase.drive_torque_nm: überschreitet wheel_torque_limit_nm')
    for section in ('live_plots', 'static_plots', 'gif_export'):
        issues += check_visual(tree['visualization'][section], f'visualization.{section}', lookup)
    issues += check_quality_gates(tree['validation']['quality_gates'], 'validation.quality_gates', lookup)
    issues += check_provenance(tree['validation']['thresholds'], 'validation.thresholds', lookup)
    speed = tree['scenario']['manoeuvre']['speed_ms']
    names = {'ltv': 'ltv_mpc', 'nmpc': 'nmpc'}
    for kind in tree['run']['controllers']:
        if kind in names:
            validity = tree['controllers'][names[kind]]['model_validity']
            if not validity['min_speed_ms'] <= speed <= validity['max_speed_ms']:
                warnings.append(
                    f'Szenariogeschwindigkeit {speed} m/s außerhalb der Modellgültigkeit von {kind}; '
                    f'Lauf wird voraussichtlich FAIL_CLOSED'
                )
    if tree['scenario'].get('brake_phase') and tree['scenario']['envelope']['check_final_speed']:
        warnings.append(
            'Bremsphase aktiv, aber Endgeschwindigkeitsprüfung eingeschaltet (voraussichtlich '
            'INVALID_MODEL_ENVELOPE)'
        )
    return issues


def check_matrix(matrix, matrix_file, selected, lookup):
    issues = []
    sets = matrix['sets']
    for name, cases in sets.items():
        where = f'validation.scenario_matrix.sets.{name}'
        if not isinstance(cases, list) or not cases:
            issues.append(f'{where}{lookup(where)}: nichtleere Liste von Fällen erwartet')
            continue
        ids = set()
        for index, case in enumerate(cases):
            validated = validate_section(MATRIX_CASE, case, f'{where}[{index}]', issues, lookup)
            if not validated:
                continue
            if validated['id'] in ids:
                issues.append(f'{where}[{index}]: doppelte Fall-ID {validated["id"]!r}')
            ids.add(validated['id'])
            target = Path(matrix_file).parent / validated['scenario']
            if not target.is_file():
                issues.append(f'{where}[{index}].scenario: Datei {target} existiert nicht')
            for key in validated.get('overrides') or {}:
                if not isinstance(key, str) or '.' not in key:
                    issues.append(f'{where}[{index}].overrides: ungültiger Punktpfad {key!r}')
    if selected is not None and selected not in sets:
        issues.append(f'run.matrix_set{lookup("run.matrix_set")}: Menge {selected!r} nicht in der Szenariomatrix')
    return issues


def _target_field(parameter, controller):
    schema = CONTROLLERS['ltv_mpc' if controller == 'ltv' else 'nmpc'][0]
    return dict(iter_fields(schema)).get(parameter)


def check_study(study, matrix, matrix_path, lookup):
    issues = []
    sets = matrix['sets']
    seen = set()
    for index, dim in enumerate(study['search_space']):
        where = f'search_space[{index}]'
        if dim['parameter'] in seen:
            issues.append(f'{where}: Parameter {dim["parameter"]} doppelt')
        seen.add(dim['parameter'])
        if dim['lower'] >= dim['upper']:
            issues.append(f'{where}: lower muss kleiner als upper sein')
        target = _target_field(dim['parameter'], study['controller'])
        for bound in ('lower', 'upper'):
            value = dim[bound]
            if target.minimum is not None and value < target.minimum:
                issues.append(f'{where}.{bound}: {value} unter zulässigem Reglerbereich ({target.range_text()})')
            if target.maximum is not None and value > target.maximum:
                issues.append(f'{where}.{bound}: {value} über zulässigem Reglerbereich ({target.range_text()})')
            if target.exclusive_minimum and value == target.minimum:
                issues.append(f'{where}.{bound}: {value} am offenen Rand ({target.range_text()})')
    if not study['search_space']:
        issues.append('search_space: mindestens eine Dimension erforderlich')
    names = [study['cases']['train'], study['cases']['holdout']] + (
        [study['cases']['robustness']] if study['cases'].get('robustness') else []
    )
    for name in names:
        if name not in sets:
            issues.append(f'cases: Matrixmenge {name!r} fehlt in {matrix_path}')
    if issues:
        return issues

    def keys(name):
        return {(case['scenario'], case.get('speed_ms')) for case in sets[name]}

    if keys(study['cases']['train']) & keys(study['cases']['holdout']):
        issues.append('cases: Training und Hold-out überschneiden sich (Szenario/Geschwindigkeit) - Hold-out-Leckage')
    for case in sets[study['cases']['train']] + sets[study['cases']['holdout']]:
        duration = case.get('duration_s')
        if duration is not None and (duration < 2 or ('double_lane_change' in case['scenario'] and duration < 4)):
            issues.append(f'cases: Fall {case["id"]} zu kurz für Offline-Abstimmung (>=2 s, DLC >=4 s)')
    return issues
