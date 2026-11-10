"""Auflösung der Konfigurationshierarchie zu einem eingefrorenen, validierten Baum.

Priorität (aufsteigend): strukturelle Schema-Defaults < Anwendungsdatei und ihre
Komponenten (inkl. ``extends``) < Profile (in Angabereihenfolge) < explizite
CLI-Komponentendateien und ``--set`` < Matrixfall. Widersprüche werden abgewiesen:
zwei Profile mit unterschiedlichen Werten für denselben Pfad sowie ein CLI-Wert,
den ein Matrixfall anders setzt.
"""

from __future__ import annotations

import copy
import hashlib
import json
from dataclasses import dataclass, field
from pathlib import Path
from types import MappingProxyType

import torquevector.infrastructure.config.consistency as consistency
from torquevector.infrastructure.config.errors import ConfigurationError
from torquevector.infrastructure.config.schema import Field, Section, iter_fields, validate_section
from torquevector.infrastructure.config.schemas import (
    APPLICATION,
    COMPONENTS,
    CONTROLLERS,
    MATRIX_CASE,
    PROFILE,
    PSO,
    SCHEMA_VERSION,
    TUNING,
    tree_schema,
)
from torquevector.infrastructure.config.yaml_io import dump, load_with_origins

FILE_KEYS = {'schema_version', 'extends', 'template'}


def _freeze(value):
    if isinstance(value, dict):
        return MappingProxyType({k: _freeze(v) for k, v in value.items()})
    if isinstance(value, list):
        return tuple(_freeze(v) for v in value)
    return value


def thaw(value):
    if isinstance(value, (dict, MappingProxyType)):
        return {k: thaw(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [thaw(v) for v in value]
    return value


def fingerprint_of(tree) -> str:
    canonical = json.dumps(
        thaw(tree), sort_keys=True, separators=(',', ':'), allow_nan=False, default=repr, ensure_ascii=True
    )
    return hashlib.sha256(canonical.encode()).hexdigest()


def _sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _dotted(parts) -> str:
    return '.'.join(str(p) for p in parts)


def _deep_merge(base, child):
    if isinstance(base, dict) and isinstance(child, dict):
        merged = dict(base)
        for key, value in child.items():
            merged[key] = _deep_merge(base[key], value) if key in base else value
        return merged
    return copy.deepcopy(child)


@dataclass(frozen=True)
class Component:
    kind: str
    controller_type: str | None
    section: dict
    origins: dict
    files: tuple
    template: bool

    @property
    def slot(self) -> tuple:
        if self.kind == 'controller':
            return CONTROLLERS[self.controller_type][1]
        return COMPONENTS[self.kind][1]

    @property
    def schema(self) -> Section:
        if self.kind == 'controller':
            return CONTROLLERS[self.controller_type][0]
        return COMPONENTS[self.kind][0]


def _version(data, path, issues):
    version = data.get('schema_version')
    if version is None:
        issues.append(f'{path}: schema_version fehlt')
    elif version != SCHEMA_VERSION or isinstance(version, bool):
        issues.append(f'{path}: schema_version {version!r} nicht unterstützt (erwartet {SCHEMA_VERSION})')


def load_component(path, *, _chain=()) -> Component:
    """Lädt eine Komponentendatei inklusive ``extends``-Kette (gleicher Typ)."""
    path = Path(path).resolve()
    if path in _chain:
        raise ConfigurationError(f'{path}: zyklische extends-Kette ({" -> ".join(map(str, _chain + (path,)))})')
    data, raw_origins = load_with_origins(path)
    issues = []
    _version(data, path, issues)
    sections = [k for k in data if k not in FILE_KEYS]
    known = set(COMPONENTS) | {'controller'}
    for key in sections:
        if key not in known:
            issues.append(f'{raw_origins.get((key,), path)}: unbekannter Abschnitt {key!r}')
    sections = [k for k in sections if k in known]
    if len(sections) != 1:
        issues.append(f'{path}: genau ein Komponentenabschnitt erwartet, gefunden {sections or "keinen"}')
    template = data.get('template', False)
    if not isinstance(template, bool):
        issues.append(f'{path}: template muss bool sein')
    if issues:
        raise ConfigurationError(issues)
    kind = sections[0]
    section = data[kind]
    if not isinstance(section, dict):
        raise ConfigurationError(f'{path}: Abschnitt {kind} muss ein Mapping sein')
    controller_type = None
    if kind == 'controller':
        controller_type = section.get('type')
        if controller_type not in CONTROLLERS:
            raise ConfigurationError(
                f'{raw_origins.get((kind,), path)}: controller.type muss einer von '
                f'{", ".join(CONTROLLERS)} sein, erhalten {controller_type!r}'
            )
    origins = {_dotted(key[1:]): where for key, where in raw_origins.items() if key[:1] == (kind,) and len(key) > 1}
    files = (path,)
    if 'extends' in data:
        parent_path = (path.parent / str(data['extends'])).resolve()
        parent = load_component(parent_path, _chain=_chain + (path,))
        if parent.kind != kind or parent.controller_type != controller_type:
            raise ConfigurationError(
                f'{path}: extends {parent_path} hat abweichenden Typ ' f'{parent.kind}/{parent.controller_type}'
            )
        section = _deep_merge(parent.section, section)
        origins = {**parent.origins, **origins}
        files = parent.files + files
    return Component(kind, controller_type, section, origins, files, bool(template))


def _origin_lookup(origins):
    def lookup(where):
        key = where
        while key:
            if key in origins:
                return f' ({origins[key]})'
            key = key.rpartition('.')[0]
        return ''

    return lookup


def validate_component(component: Component, *, template_ok=False):
    issues = []
    lookup = _origin_lookup(component.origins)
    result = validate_section(component.schema, component.section, '', issues, lookup)
    if component.template:
        if not template_ok:
            return None, [
                f'{component.files[-1]}: Vorlage (template: true) ist nicht als Laufzeitkonfiguration zulässig'
            ]
        structural = [i for i in issues if 'unbekannter' in i or 'erwartet Mapping' in i]
        return result, structural
    return result, issues


@dataclass(frozen=True)
class ResolvedConfiguration:
    tree: MappingProxyType
    origins: MappingProxyType
    sources: tuple
    include_files: MappingProxyType
    layers: tuple
    fingerprint: str
    application_path: Path
    warnings: tuple = ()
    inputs: dict = field(default_factory=dict, compare=False, repr=False)

    def get(self, dotted: str):
        node = self.tree
        for key in dotted.split('.'):
            node = node[key]
        return node

    def plain(self) -> dict:
        return thaw(self.tree)

    def export_yaml(self) -> str:
        lines = [
            f'Aufgelöste torquevector-Konfiguration (schema_version {SCHEMA_VERSION})',
            f'fingerprint_sha256: {self.fingerprint}',
            f'application: {self.application_path}',
        ]
        for role, source, digest in self.sources:
            lines.append(f'source {role}: {source} sha256={digest[:16]}')
        for layer in self.layers:
            lines.append(f'layer {layer}')
        return dump({'schema_version': SCHEMA_VERSION, **self.plain()}, header='\n'.join(lines))

    def origins_json(self) -> str:
        return json.dumps(dict(self.origins), indent=2, ensure_ascii=False) + '\n'

    def sources_json(self) -> str:
        return json.dumps([{'role': r, 'path': str(p), 'sha256': d} for r, p, d in self.sources], indent=2) + '\n'


def _flatten_includes(includes, prefix=()):
    for key, value in includes.items():
        if isinstance(value, dict):
            yield from _flatten_includes(value, prefix + (key,))
        else:
            yield _dotted(prefix + (key,)), value


def _schema_leaf(schema: Section, parts):
    node = schema
    for part in parts:
        if not isinstance(node, Section) or part not in node.fields:
            return None
        node = node.fields[part]
    return node


def _set_path(tree, parts, value):
    node = tree
    for part in parts[:-1]:
        child = node.get(part)
        if not isinstance(child, dict):
            child = {}
            node[part] = child
        node = child
    node[parts[-1]] = copy.deepcopy(value)


class _Layers:
    """Verfolgt, welche Ebene einen Pfad gesetzt hat, und weist Widersprüche ab."""

    def __init__(self):
        self.set_by = {}
        self.issues = []

    def claim(self, key, value, layer_kind, label):
        previous = self.set_by.get(key)
        if previous is not None and previous[1] != value:
            kinds = {previous[0], layer_kind}
            if kinds == {'profile'} or kinds == {'cli', 'case'}:
                self.issues.append(
                    f'Widersprüchliche Werte für {key}: {previous[2]} setzt {previous[1]!r}, '
                    f'{label} setzt {value!r}'
                )
        self.set_by[key] = (layer_kind, value, label)


def _load_profile(path: Path):
    data, raw = load_with_origins(path)
    issues = []
    _version(data, path, issues)
    extra = set(data) - {'schema_version', 'profile'}
    if extra or 'profile' not in data:
        issues.append(
            f'{path}: Profil erwartet genau die Schlüssel schema_version und profile (gefunden {sorted(data)})'
        )
    if issues:
        raise ConfigurationError(issues)
    origins = {_dotted(k[1:]): v for k, v in raw.items() if k[:1] == ('profile',) and len(k) > 1}
    profile = validate_section(
        PROFILE, data['profile'], 'profile', issues, _origin_lookup({'profile.' + k: v for k, v in origins.items()})
    )
    if issues:
        raise ConfigurationError(issues)
    return profile, origins


def resolve(config_path, *, profiles=(), replacements=None, overrides=None, case=None) -> ResolvedConfiguration:
    """Löst eine Anwendungs- oder Profildatei samt Overlays auf.

    ``replacements``: CLI-Komponentendateien ``{include_schlüssel: pfad}``.
    ``overrides``: CLI ``--set`` als Liste ``[(punktpfad, wert)]``.
    ``case``: Matrixfall ``{'replacements': {...}, 'overrides': [...], 'label': str}``.
    """
    inputs = {
        'config_path': str(config_path),
        'profiles': [str(p) for p in profiles],
        'replacements': dict(replacements or {}),
        'overrides': list(overrides or []),
        'case': case,
    }
    config_path = Path(config_path).resolve()
    data, raw = load_with_origins(config_path)
    profile_paths = [Path(p).resolve() for p in profiles]
    if 'profile' in data and 'application' not in data:
        profile, _ = _load_profile(config_path)
        if not profile.get('base'):
            raise ConfigurationError(f'{config_path}: Profil als --config benötigt profile.base')
        profile_paths.insert(0, config_path)
        application_path = (config_path.parent / profile['base']).resolve()
        data, raw = load_with_origins(application_path)
    else:
        application_path = config_path
    issues = []
    _version(data, application_path, issues)
    extra = set(data) - {'schema_version', 'application'}
    if extra or 'application' not in data:
        issues.append(
            f'{application_path}: Anwendungskonfiguration erwartet schema_version und application '
            f'(gefunden {sorted(data)})'
        )
    if issues:
        raise ConfigurationError(issues)
    app_origins = {
        'application.' + _dotted(k[1:]): v for k, v in raw.items() if k[:1] == ('application',) and len(k) > 1
    }
    app = validate_section(APPLICATION, data['application'], 'application', issues, _origin_lookup(app_origins))
    if issues:
        raise ConfigurationError(issues)

    includes = {key: (application_path.parent / value).resolve() for key, value in _flatten_includes(app['includes'])}
    include_origin = {key: app_origins.get(f'application.includes.{key}', str(application_path)) for key in includes}
    layers = _Layers()
    layer_labels = []
    override_list = []  # (pfad, wert, kind, label)

    for profile_path in profile_paths:
        profile, porigins = _load_profile(profile_path)
        label = f'profile {profile["id"]} ({profile_path})'
        layer_labels.append(label)
        for key, value in (profile.get('includes') or {}).items():
            if key not in includes:
                issues.append(f'{profile_path}: unbekannter include-Schlüssel {key!r}')
                continue
            target = (profile_path.parent / str(value)).resolve()
            layers.claim('includes.' + key, target, 'profile', label)
            includes[key] = target
            include_origin[key] = porigins.get(f'includes.{key}', str(profile_path))
        for key, value in (profile.get('overrides') or {}).items():
            layers.claim(key, value, 'profile', label)
            override_list.append((key, value, 'profile', f'{label}:{porigins.get("overrides." + key, "")}'))

    for kind, mapping, olist, label in (
        ('cli', replacements or {}, overrides or [], 'CLI'),
        (
            'case',
            (case or {}).get('replacements', {}),
            (case or {}).get('overrides', []),
            f'Matrixfall {(case or {}).get("label", "")}',
        ),
    ):
        if mapping or olist:
            layer_labels.append(label)
        for key, value in mapping.items():
            if key not in includes:
                issues.append(f'{label}: unbekannter Komponenten-Schlüssel {key!r}')
                continue
            target = Path(value).resolve()
            layers.claim('includes.' + key, target, kind, label)
            includes[key] = target
            include_origin[key] = label
        seen = {}
        for key, value in olist:
            if key in seen and seen[key] != value:
                issues.append(f'{label}: Pfad {key} mehrfach mit unterschiedlichen Werten gesetzt')
            seen[key] = value
            layers.claim(key, value, kind, label)
            override_list.append((key, value, kind, label))
    issues.extend(layers.issues)
    if issues:
        raise ConfigurationError(issues)

    # Szenario-gebundene Fahrbahn: scenario.road.surface ersetzt includes.road (außer CLI/Matrixfall).
    scenario_file = includes['scenario']
    if include_origin.get('road') not in ('CLI', f'Matrixfall {(case or {}).get("label", "")}'):
        scenario_component = load_component(scenario_file)
        linked = (scenario_component.section.get('road') or {}).get('surface')
        if isinstance(linked, str):
            includes['road'] = (scenario_file.parent / linked).resolve()
            include_origin['road'] = f'{scenario_file} (scenario.road.surface)'
    tree = {'application': {'id': app['id'], 'description': app.get('description')}, 'run': app['run']}
    origins = {k: v for k, v in app_origins.items()}
    origins.update(
        {'run.' + k[len('application.run.') :]: v for k, v in app_origins.items() if k.startswith('application.run.')}
    )
    sources = [('application', application_path, _sha256(application_path))]
    sources += [('profile', p, _sha256(p)) for p in profile_paths]
    for key, file_path in includes.items():
        component = load_component(file_path)
        if component.template:
            issues.append(
                f'{file_path}: Vorlage (template: true) darf nicht als {key} verwendet werden '
                f'(referenziert in {include_origin[key]})'
            )
            continue
        if _dotted(component.slot) != key:
            issues.append(
                f'{file_path}: Komponente vom Typ {_dotted(component.slot)} passt nicht in {key} '
                f'(referenziert in {include_origin[key]})'
            )
            continue
        _set_path(tree, list(component.slot), component.section)
        origins.update({f'{key}.{k}': v for k, v in component.origins.items()})
        origins[key] = f'{component.files[-1]} (eingebunden über {include_origin[key]})'
        sources += [(key, f, _sha256(f)) for f in component.files]
    if issues:
        raise ConfigurationError(issues)

    schema = tree_schema()
    for key, value, kind, label in override_list:
        parts = key.split('.')
        leaf = _schema_leaf(schema, parts)
        if leaf is None or isinstance(leaf, Section):
            issues.append(f'{label}: Override-Pfad {key!r} ist kein Blattparameter des Schemas')
            continue
        _set_path(tree, parts, value)
        origins[key] = f'override ({label})'
    if issues:
        raise ConfigurationError(issues)

    lookup = _origin_lookup(origins)
    normalized = validate_section(schema, tree, '', issues, lookup)
    if issues:
        raise ConfigurationError(issues)
    warnings = []
    issues.extend(consistency.check_tree(normalized, lookup, warnings))
    matrix_file = includes['validation.scenario_matrix']
    issues.extend(
        consistency.check_matrix(
            normalized['validation']['scenario_matrix'], matrix_file, normalized['run'].get('matrix_set'), lookup
        )
    )
    if issues:
        raise ConfigurationError(issues)
    frozen = _freeze(normalized)
    return ResolvedConfiguration(
        frozen,
        MappingProxyType(dict(origins)),
        tuple(sources),
        MappingProxyType({k: str(v) for k, v in includes.items()}),
        tuple(layer_labels),
        fingerprint_of(frozen),
        application_path,
        tuple(warnings),
        inputs,
    )


def matrix_cases(resolved: ResolvedConfiguration, set_name: str) -> list[dict]:
    """Matrixfälle einer Menge als Override-Ebenen (Szenario relativ zur Matrixdatei)."""
    matrix_file = Path(resolved.include_files['validation.scenario_matrix'])
    sets = resolved.tree['validation']['scenario_matrix']['sets']
    if set_name not in sets:
        raise ConfigurationError(f'Matrixmenge {set_name!r} existiert nicht ({matrix_file})')
    cases = []
    for raw in sets[set_name]:
        case = thaw(raw)
        overrides = []  # run.matrix_set des Falls wird ignoriert (nur Einzelfall-Laufzeit)
        if case.get('speed_ms') is not None:
            overrides.append(('scenario.manoeuvre.speed_ms', case['speed_ms']))
        if case.get('duration_s') is not None:
            overrides.append(('scenario.simulation_duration_s', case['duration_s']))
        overrides += list((case.get('overrides') or {}).items())
        cases.append(
            {
                'label': f'{set_name}/{case["id"]}',
                'id': case['id'],
                'replacements': {'scenario': str((matrix_file.parent / case['scenario']).resolve())},
                'overrides': overrides,
            }
        )
    return cases


def resolve_case(resolved: ResolvedConfiguration, case: dict) -> ResolvedConfiguration:
    inputs = resolved.inputs
    return resolve(
        inputs['config_path'],
        profiles=inputs['profiles'],
        replacements=inputs['replacements'],
        overrides=inputs['overrides'],
        case=case,
    )


def with_overrides(resolved: ResolvedConfiguration, overrides) -> ResolvedConfiguration:
    """Neue validierte Konfiguration mit zusätzlichen Overrides (z. B. PSO-Kandidat)."""
    inputs = resolved.inputs
    case = inputs.get('case') or {'label': 'candidate', 'replacements': {}, 'overrides': []}
    merged = dict(case)
    combined = dict(case.get('overrides', []))
    combined.update(dict(overrides))  # abgeleitete Konfiguration: neuer Wert ersetzt den Fallwert
    merged['overrides'] = list(combined.items())
    return resolve(
        inputs['config_path'],
        profiles=inputs['profiles'],
        replacements=inputs['replacements'],
        overrides=inputs['overrides'],
        case=merged,
    )


def resolve_component(path, *, template_ok=True):
    """Einzeldatei validieren (``config validate`` für Komponenten, PSO, Studien)."""
    component = load_component(path)
    result, issues = validate_component(component, template_ok=template_ok)
    if not component.template and not issues:
        lookup = _origin_lookup(component.origins)
        issues = consistency.check_component(component.kind, result, lookup)
    if issues:
        raise ConfigurationError(issues)
    return component, result


@dataclass(frozen=True)
class StudyDefinition:
    study: dict
    pso: dict
    study_path: Path
    pso_path: Path
    application_path: Path
    matrix_path: Path
    sources: tuple
    fingerprint: str


def resolve_study(path, *, overrides=None) -> StudyDefinition:
    """Löst eine Studie (tuning) bzw. eine pso.yaml mit default_study auf."""
    path = Path(path).resolve()
    component = load_component(path)
    if component.kind == 'pso':
        _, pso = resolve_component(path, template_ok=False)
        study_path = (path.parent / pso['default_study']).resolve()
        study_component = load_component(study_path)
    elif component.kind == 'tuning':
        study_path, study_component = path, component
    else:
        raise ConfigurationError(f'{path}: optimize erwartet eine pso- oder tuning-Datei, erhalten {component.kind}')
    if study_component.kind != 'tuning':
        raise ConfigurationError(f'{study_path}: default_study muss eine tuning-Datei sein')
    section = copy.deepcopy(study_component.section)
    issues = []
    for key, value in overrides or []:
        parts = key.split('.')
        if parts[0] != 'tuning' or _schema_leaf(TUNING, parts[1:]) is None:
            issues.append(
                f'Override {key!r}: Studien-Overrides müssen mit tuning. beginnen und ein Schemablatt treffen'
            )
            continue
        _set_path(section, parts[1:], value)
    if issues:
        raise ConfigurationError(issues)
    lookup = _origin_lookup(study_component.origins)
    study = validate_section(TUNING, section, '', issues, lookup)
    if issues:
        raise ConfigurationError(issues)
    pso_path = (study_path.parent / study['algorithm']).resolve()
    _, pso = resolve_component(pso_path, template_ok=False)
    if component.kind == 'pso' and pso_path != path:
        raise ConfigurationError(f'{study_path}: algorithm verweist auf {pso_path}, aufgerufen wurde {path}')
    matrix_path = (study_path.parent / study['cases']['matrix']).resolve()
    application_path = (study_path.parent / study['application']).resolve()
    _, matrix = resolve_component(matrix_path, template_ok=False)
    issues.extend(consistency.check_matrix(matrix, matrix_path, None, lambda where: ''))
    issues.extend(consistency.check_study(study, matrix, matrix_path, lookup))
    if issues:
        raise ConfigurationError(issues)
    sources = tuple(
        (role, p, _sha256(p))
        for role, p in (
            ('study', study_path),
            ('pso', pso_path),
            ('matrix', matrix_path),
            ('application', application_path),
        )
    )
    return StudyDefinition(
        study,
        pso,
        study_path,
        pso_path,
        application_path,
        matrix_path,
        sources,
        fingerprint_of({'study': study, 'pso': pso, 'sources': [d for _, _, d in sources]}),
    )


def load_snapshot(path) -> dict:
    """Gespeicherten Konfigurations-Snapshot eines Laufs laden und erneut gegen das Schema prüfen."""
    data, raw = load_with_origins(Path(path))
    issues = []
    _version(data, path, issues)
    body = {k: v for k, v in data.items() if k != 'schema_version'}
    origins = {_dotted(k): v for k, v in raw.items()}
    tree = validate_section(tree_schema(), body, '', issues, _origin_lookup(origins))
    if issues:
        raise ConfigurationError(issues)
    return tree


def all_leaf_paths() -> dict[str, Field]:
    """Alle Blattpfade des Anwendungsbaums (für Dokumentation und Tests)."""
    return dict(iter_fields(tree_schema()))
