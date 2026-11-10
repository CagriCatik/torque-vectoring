"""Striktes YAML-Lesen mit Zeilenherkunft und deterministischem Schreiben.

Abweichungen vom PyYAML-Standard (bewusst, YAML-1.2-nah):

- Nur ``true``/``false`` sind Booleans. ``off``/``on``/``yes``/``no`` bleiben Strings,
  sonst würde der Reglername ``off`` stillschweigend zu ``False``.
- Wissenschaftliche Notation ohne Dezimalpunkt (``1e-5``) ist eine Zahl.
- Doppelte Schlüssel sind ein Fehler statt "letzter gewinnt".
"""

from __future__ import annotations

import math
import re
from pathlib import Path

import yaml

from torquevector.infrastructure.config.errors import ConfigurationError

_BOOL = re.compile(r'^(?:true|True|TRUE|false|False|FALSE)$')
_FLOAT = re.compile(
    r'''^(?:[-+]?(?:[0-9][0-9_]*)\.[0-9_]*(?:[eE][-+]?[0-9]+)?
                    |[-+]?(?:[0-9][0-9_]*)(?:[eE][-+]?[0-9]+)
                    |\.[0-9_]+(?:[eE][-+][0-9]+)?
                    |[-+]?\.(?:inf|Inf|INF)
                    |\.(?:nan|NaN|NAN))$''',
    re.X,
)


class StrictLoader(yaml.SafeLoader):
    """SafeLoader mit YAML-1.2-Booleans und vollständiger Float-Erkennung."""


StrictLoader.yaml_implicit_resolvers = {
    key: [
        (tag, regexp) for tag, regexp in resolvers if tag not in ('tag:yaml.org,2002:bool', 'tag:yaml.org,2002:float')
    ]
    for key, resolvers in yaml.SafeLoader.yaml_implicit_resolvers.items()
}
StrictLoader.add_implicit_resolver('tag:yaml.org,2002:bool', _BOOL, list('tTfF'))
StrictLoader.add_implicit_resolver('tag:yaml.org,2002:float', _FLOAT, list('-+0123456789.'))


def _origin(path: Path, node) -> str:
    return f'{path}:{node.start_mark.line + 1}'


def load_with_origins(path: Path) -> tuple[dict, dict[tuple, str]]:
    """Lädt eine YAML-Datei; liefert Daten und Herkunft ``Pfadtupel -> Datei:Zeile``."""
    path = Path(path)
    try:
        text = path.read_text(encoding='utf-8')
    except OSError as exc:
        raise ConfigurationError(f'{path}: Datei nicht lesbar ({exc.strerror or exc})') from exc
    loader = StrictLoader(text)
    try:
        node = loader.get_single_node()
    except yaml.YAMLError as exc:
        raise ConfigurationError(f'{path}: ungültiges YAML ({exc})') from exc
    finally:
        loader.dispose()
    if node is None:
        raise ConfigurationError(f'{path}: leere Konfigurationsdatei')
    origins: dict[tuple, str] = {}
    issues: list[str] = []
    construct = StrictLoader('')

    def walk(current, key_path):
        origins[key_path] = _origin(path, current)
        if isinstance(current, yaml.MappingNode):
            result = {}
            for key_node, value_node in current.value:
                key = construct.construct_object(key_node, deep=True)
                if not isinstance(key, str):
                    issues.append(f'{_origin(path, key_node)}: Schlüssel muss ein String sein ({key!r})')
                    continue
                if key in result:
                    issues.append(f'{_origin(path, key_node)}: doppelter Schlüssel {key!r}')
                    continue
                result[key] = walk(value_node, key_path + (key,))
            return result
        if isinstance(current, yaml.SequenceNode):
            return [walk(item, key_path + (index,)) for index, item in enumerate(current.value)]
        if current.tag.startswith('tag:yaml.org,2002:') and current.tag.rsplit(':', 1)[1] in (
            'str',
            'int',
            'float',
            'bool',
            'null',
        ):
            return construct.construct_object(current, deep=True)
        issues.append(f'{_origin(path, current)}: nicht unterstützter YAML-Tag {current.tag}')
        return None

    data = walk(node, ())
    if issues:
        raise ConfigurationError(issues)
    if not isinstance(data, dict):
        raise ConfigurationError(f'{path}: oberste Ebene muss ein Mapping sein')
    return data, origins


class _Dumper(yaml.SafeDumper):
    pass


def _represent_float(dumper, value):
    if math.isnan(value) or math.isinf(value):
        raise ConfigurationError('Nicht endliche Zahl kann nicht exportiert werden')
    text = repr(float(value))
    if 'e' in text and '.' not in text.split('e')[0]:
        mantissa, exponent = text.split('e')
        text = f'{mantissa}.0e{exponent}'
    return dumper.represent_scalar('tag:yaml.org,2002:float', text)


def _represent_str(dumper, value):
    style = (
        '"'
        if (_BOOL.match(value) or value.lower() in ('off', 'on', 'yes', 'no', 'null', '~', '') or _FLOAT.match(value))
        else None
    )
    return dumper.represent_scalar('tag:yaml.org,2002:str', value, style=style)


_Dumper.add_representer(float, _represent_float)
_Dumper.add_representer(str, _represent_str)
_Dumper.add_representer(tuple, lambda dumper, value: dumper.represent_list(list(value)))


def dump(data, header: str | None = None) -> str:
    """Deterministischer YAML-Export; Floats verlustfrei per ``repr``."""
    body = yaml.dump(data, Dumper=_Dumper, sort_keys=False, allow_unicode=True, default_flow_style=False, width=100)
    if header:
        prefix = ''.join(f'# {line}\n' if line else '#\n' for line in header.splitlines())
        return prefix + body
    return body


def parse_scalar(text: str):
    """CLI-Override-Wert mit denselben Regeln wie Dateien interpretieren."""
    try:
        value = yaml.load(text, Loader=StrictLoader)
    except yaml.YAMLError as exc:
        raise ConfigurationError(f'Override-Wert nicht als YAML lesbar: {text!r} ({exc})') from exc
    return value
