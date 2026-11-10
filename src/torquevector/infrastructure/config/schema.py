"""Deklarative, strikte Schema-Beschreibung für YAML-Konfigurationen.

Ein ``Field`` beschreibt genau einen Blattwert: Datentyp, SI-Einheit, zulässigen
Bereich, Bedeutung, verwendende Module und die Bindung an das typisierte
Laufzeitobjekt (``binds``). Unbekannte Schlüssel werden ausnahmslos abgewiesen.

Einheitenprüfung ohne Umrechnung: Ein Zahlenwert darf alternativ als
``{value: <zahl>, unit: <einheit>}`` angegeben werden. Die Einheit muss exakt der
deklarierten Einheit entsprechen; es gibt bewusst keine automatische Konversion.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from math import isfinite

_UNSET = object()
WHEELS = ('FL', 'FR', 'RL', 'RR')
TYPES = ('float', 'int', 'bool', 'str', 'enum', 'wheel_map', 'float_list', 'str_list', 'path', 'mapping', 'list')


@dataclass(frozen=True)
class Field:
    type: str
    description: str
    unit: str | None = None
    minimum: float | None = None
    maximum: float | None = None
    exclusive_minimum: bool = False
    exclusive_maximum: bool = False
    choices: tuple | None = None
    required: bool = True
    nullable: bool = False
    fixed: object = _UNSET
    fixed_reason: str = ''
    binds: tuple[str, ...] = ()
    used_by: tuple[str, ...] = ()
    length: int | None = None
    structural_default: object = _UNSET
    item: 'Section | None' = None

    def __post_init__(self):
        if self.type not in TYPES:
            raise TypeError(f'Unbekannter Schema-Typ {self.type}')
        if self.type in ('float', 'int', 'wheel_map', 'float_list') and self.unit is None:
            raise TypeError(f'Numerisches Feld ohne Einheit: {self.description}')
        if self.type == 'enum' and not self.choices:
            raise TypeError('enum benötigt choices')
        if self.structural_default is not _UNSET and self.type not in ('str', 'path', 'enum', 'bool'):
            # Physikalische/numerische Größen erhalten keinen versteckten Fallback.
            raise TypeError(f'Modellparameter darf keinen strukturellen Default haben: {self.description}')

    @property
    def is_fixed(self) -> bool:
        return self.fixed is not _UNSET

    @property
    def has_structural_default(self) -> bool:
        return self.structural_default is not _UNSET

    def range_text(self) -> str:
        if self.is_fixed:
            return f'fest = {self.fixed!r}'
        if self.choices:
            return ', '.join(map(str, self.choices))
        lo = '' if self.minimum is None else f"{'>' if self.exclusive_minimum else '>='} {self.minimum:g}"
        hi = '' if self.maximum is None else f"{'<' if self.exclusive_maximum else '<='} {self.maximum:g}"
        text = ' und '.join(part for part in (lo, hi) if part)
        if self.length:
            text = (text + f'; Länge {self.length}').strip('; ')
        return text or '–'


@dataclass(frozen=True)
class Section:
    fields: dict
    description: str = ''
    required: bool = True
    nullable: bool = False
    open_mapping: bool = False  # nur für reine Metadaten (Provenienz), nie für Modellwerte


def _type_name(value) -> str:
    return type(value).__name__


def _number(value, kind):
    if isinstance(value, bool):
        return None
    if kind == 'int':
        return value if isinstance(value, int) else None
    if isinstance(value, (int, float)):
        return float(value)
    return None


def _check_range(spec: Field, number, where, issues):
    if not isfinite(number):
        issues.append(f'{where}: Wert muss endlich sein')
        return
    if spec.minimum is not None and (number < spec.minimum or (spec.exclusive_minimum and number == spec.minimum)):
        issues.append(f'{where}: {number!r} unterschreitet zulässigen Bereich ({spec.range_text()})')
    if spec.maximum is not None and (number > spec.maximum or (spec.exclusive_maximum and number == spec.maximum)):
        issues.append(f'{where}: {number!r} überschreitet zulässigen Bereich ({spec.range_text()})')


def _unwrap_unit(spec: Field, value, where, issues):
    if isinstance(value, dict) and set(value) == {'value', 'unit'}:
        if value['unit'] != spec.unit:
            issues.append(
                f"{where}: Einheit {value['unit']!r} entspricht nicht der geforderten Einheit "
                f"{spec.unit!r} (keine automatische Umrechnung)"
            )
            return _UNSET
        return value['value']
    if isinstance(value, dict) and 'unit' in value:
        issues.append(f'{where}: Einheitenform erfordert genau die Schlüssel value und unit')
        return _UNSET
    return value


def validate_value(spec: Field, value, where: str, issues: list, origin=lambda w: ''):
    """Prüft und normalisiert einen Blattwert. Liefert den normalisierten Wert."""
    loc = f'{where}{origin(where)}'
    if value is None:
        if spec.nullable:
            return None
        issues.append(f'{loc}: Pflichtwert ist null')
        return None
    if spec.type in ('float', 'int'):
        value = _unwrap_unit(spec, value, loc, issues)
        if value is _UNSET:
            return None
        number = _number(value, spec.type)
        if number is None:
            issues.append(f'{loc}: erwartet {spec.type} [{spec.unit}], erhalten {_type_name(value)} {value!r}')
            return None
        _check_range(spec, number, loc, issues)
        value = number
    elif spec.type == 'bool':
        if not isinstance(value, bool):
            issues.append(f'{loc}: erwartet bool (true/false), erhalten {_type_name(value)} {value!r}')
            return None
    elif spec.type in ('str', 'path'):
        if not isinstance(value, str) or not value.strip():
            issues.append(f'{loc}: erwartet nichtleeren String, erhalten {_type_name(value)} {value!r}')
            return None
    elif spec.type == 'enum':
        if value not in spec.choices:
            issues.append(f'{loc}: {value!r} nicht zulässig; erlaubt: {", ".join(map(str, spec.choices))}')
            return None
    elif spec.type == 'wheel_map':
        if not isinstance(value, dict) or set(value) != set(WHEELS):
            issues.append(f'{loc}: erwartet Mapping mit genau den Rädern {", ".join(WHEELS)}')
            return None
        out = {}
        for wheel in WHEELS:
            item = _unwrap_unit(spec, value[wheel], f'{loc}.{wheel}', issues)
            number = None if item is _UNSET else _number(item, 'float')
            if number is None:
                if item is not _UNSET:
                    issues.append(f'{loc}.{wheel}: erwartet float [{spec.unit}], erhalten {item!r}')
                continue
            _check_range(spec, number, f'{loc}.{wheel}', issues)
            out[wheel] = number
        value = out
    elif spec.type in ('float_list', 'str_list'):
        if not isinstance(value, list):
            issues.append(f'{loc}: erwartet Liste, erhalten {_type_name(value)}')
            return None
        if spec.length is not None and len(value) != spec.length:
            issues.append(f'{loc}: erwartet genau {spec.length} Einträge, erhalten {len(value)}')
        out = []
        for index, item in enumerate(value):
            if spec.type == 'float_list':
                number = _number(item, 'float')
                if number is None:
                    issues.append(f'{loc}[{index}]: erwartet float [{spec.unit}], erhalten {item!r}')
                    continue
                _check_range(spec, number, f'{loc}[{index}]', issues)
                out.append(number)
            else:
                if spec.choices is not None and item not in spec.choices:
                    issues.append(f'{loc}[{index}]: {item!r} nicht zulässig; erlaubt: {", ".join(spec.choices)}')
                    continue
                if not isinstance(item, str):
                    issues.append(f'{loc}[{index}]: erwartet String, erhalten {item!r}')
                    continue
                out.append(item)
        if spec.type == 'str_list' and len(set(out)) != len(out):
            issues.append(f'{loc}: Einträge müssen eindeutig sein')
        value = out
    elif spec.type == 'list':
        if not isinstance(value, list):
            issues.append(f'{loc}: erwartet Liste von Mappings, erhalten {_type_name(value)}')
            return None
        value = [
            validate_section(spec.item, item, f'{where}[{index}]', issues, origin) for index, item in enumerate(value)
        ]
    elif spec.type == 'mapping':
        if not isinstance(value, dict):
            issues.append(f'{loc}: erwartet Mapping, erhalten {_type_name(value)}')
            return None
    if spec.is_fixed and value != spec.fixed:
        issues.append(
            f'{loc}: Wert {value!r} ist im Code fest = {spec.fixed!r} '
            f'({spec.fixed_reason or "keine Konfigurationsfreiheit"})'
        )
    return value


def validate_section(section: Section, data, where: str, issues: list, origin=lambda w: ''):
    loc = f'{where}{origin(where)}'
    if data is None:
        if section.nullable:
            return None
        issues.append(f'{loc}: Abschnitt fehlt oder ist null')
        return None
    if not isinstance(data, dict):
        issues.append(f'{loc}: erwartet Mapping, erhalten {_type_name(data)}')
        return None
    if section.open_mapping:
        return dict(data)
    unknown = sorted(set(data) - set(section.fields))
    for key in unknown:
        issues.append(f'{where}.{key}{origin(where + "." + key)}: unbekannter Parameter (nicht im Schema)')
    result = {}
    for key, spec in section.fields.items():
        child = f'{where}.{key}' if where else key
        if key not in data:
            if isinstance(spec, Field) and spec.has_structural_default:
                result[key] = spec.structural_default
                continue
            required = spec.required
            if required:
                issues.append(f'{child}: Pflichtparameter fehlt{origin(where)}')
            elif isinstance(spec, Field) and spec.nullable:
                result[key] = None
            continue
        if isinstance(spec, Section):
            result[key] = validate_section(spec, data[key], child, issues, origin)
        else:
            result[key] = validate_value(spec, data[key], child, issues, origin)
    return result


def iter_fields(section: Section, prefix: str = ''):
    """Alle Blattfelder (Pfad, Field) in Deklarationsreihenfolge."""
    for key, spec in section.fields.items():
        path = f'{prefix}.{key}' if prefix else key
        if isinstance(spec, Section):
            if not spec.open_mapping:
                yield from iter_fields(spec, path)
        else:
            yield path, spec
            if spec.type == 'list' and spec.item is not None:
                yield from iter_fields(spec.item, path + '[]')
