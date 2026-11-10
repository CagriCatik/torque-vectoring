"""Erzeugt docs/configuration/PARAMETER_REFERENCE.md aus Schema + Referenzkonfiguration.

Die Referenz ist generiert, damit Datentyp, Einheit, Bereich, Bindung und Referenzwert
nie vom Code abweichen; ``tests/unit/test_parameter_reference_current.py`` prüft Aktualität.

    python scripts/configuration/generate_parameter_reference.py --check
"""

from __future__ import annotations

import argparse
from pathlib import Path

from torquevector.infrastructure.config.resolver import resolve, resolve_study
from torquevector.infrastructure.config.schema import iter_fields
from torquevector.infrastructure.config.schemas import COMPONENTS, CONTROLLERS, PSO, TUNING

ROOT = Path(__file__).resolve().parents[2]
TARGET = ROOT / 'docs/configuration/PARAMETER_REFERENCE.md'
SKIP_PREFIX = ('id', 'description', 'provenance')


def _value(tree, dotted):
    node = tree
    for key in dotted.split('.'):
        if not isinstance(node, dict) or key not in node:
            return '–'
        node = node[key]
    if isinstance(node, dict) and set(node) == {'FL', 'FR', 'RL', 'RR'}:
        return 'FL {FL:g}, FR {FR:g}, RL {RL:g}, RR {RR:g}'.format(**node)
    if isinstance(node, (list, tuple)):
        return '[' + ', '.join(map(str, node)) + ']'
    return '`null`' if node is None else f'`{node!r}`' if isinstance(node, str) else repr(node)


def _cell(text):
    return str(text).replace('|', '\\|').replace('\n', ' ')


def _rows(section, tree, files, provenance, prefix):
    for path, spec in iter_fields(section):
        if path.split('.')[0] in SKIP_PREFIX or '[]' in path:
            continue
        full = f'{prefix}.{path}' if prefix else path
        yield (
            '| `{}` | {} | {} | {} | {} | {} | {} | {} | {} | {} |'.format(
                full,
                _cell(files),
                spec.type,
                spec.unit or '–',
                _cell(spec.range_text()),
                _cell(_value(tree, full)),
                _cell(spec.description + (f' (fest: {spec.fixed_reason})' if spec.is_fixed else '')),
                _cell(', '.join(f'`{b}`' for b in spec.binds) or '–'),
                _cell(', '.join(spec.used_by) or '–'),
                _cell(provenance),
            )
        )


def generate() -> str:
    resolved = resolve(ROOT / 'configs/default.yaml')
    tree = resolved.plain()
    lines = [
        '# Parameterreferenz',
        '',
        '**Generiert** aus `src/torquevector/infrastructure/config/schemas.py` und `configs/default.yaml` '
        'mit `python scripts/configuration/generate_parameter_reference.py`. Nicht manuell bearbeiten.',
        '',
        'Referenzwert = Wert der aufgelösten Referenzkonfiguration `configs/default.yaml` '
        f'(Fingerprint `{resolved.fingerprint[:16]}…`). Bindung = Attributpfad im typisierten '
        '`RuntimeConfiguration`-Objekt (`torquevector.cli.composition`). „fest“ = im Code verankert '
        '(AST-eingefroren oder Alternative nicht implementiert); der Wert wird angezeigt und geprüft, ist '
        'aber nicht konfigurierbar. Jede Komponente trägt zusätzlich `id`, `description` und `provenance` '
        '(`source_type`, Validierungsflag, `origin`, `notes`, optionale `calibration`-Metadaten).',
        '',
        'Alle Werte sind synthetisch bzw. Engineering-Annahmen; keine Größe ist gegen reale Fahrzeug- oder '
        'Reifenmessungen validiert (Q10 REQUIRED_REAL_DATA).',
        '',
    ]
    header = (
        '| YAML-Pfad | Datei (Referenz) | Typ | Einheit | Zulässiger Bereich | Referenzwert | Bedeutung | '
        'Bindung | Verwendende Module | Provenienz |'
    )
    groups = [
        (kind, schema, location) for kind, (schema, location) in COMPONENTS.items() if kind not in ('pso', 'tuning')
    ]
    groups += [(f'controller ({kind})', schema, location) for kind, (schema, location) in CONTROLLERS.items()]
    for title, schema, location in groups:
        prefix = '.'.join(location)
        node = tree
        for key in location:
            node = node[key]
        files = Path(resolved.include_files[prefix]).relative_to(ROOT).as_posix()
        provenance = node.get('provenance', {}).get('origin', '–')
        lines += [f'## {title} – `{prefix}`', '', schema.description, '', header, '|' + '---|' * 10]
        lines += list(_rows(schema, tree, files, provenance, prefix))
        lines.append('')
    lines += ['## Laufsteuerung – `run`', '', header, '|' + '---|' * 10]
    from torquevector.infrastructure.config.schemas import RUN

    lines += list(_rows(RUN, tree, 'configs/default.yaml', 'Anwendungskonfiguration', 'run'))
    lines.append('')
    study = resolve_study(ROOT / 'configs/optimization/pso.yaml')
    for title, schema, data, file in (
        ('Offline-PSO – `pso`', PSO, study.pso, 'configs/optimization/pso.yaml'),
        ('Offline-Studie – `tuning`', TUNING, study.study, 'configs/optimization/nmpc_tuning.yaml'),
    ):
        lines += [
            f'## {title}',
            '',
            schema.description + ' (eigener Wurzelbaum, nicht Teil von default.yaml).',
            '',
            header,
            '|' + '---|' * 10,
        ]
        lines += list(_rows(schema, data, file, data.get('provenance', {}).get('origin', '–'), ''))
        lines.append('')
    lines += [
        'Suchraum (`tuning.search_space[]`): `parameter` ∈ Gewichtsfelder des Reglers, `lower`/`upper` in der Einheit '
        'des Zielparameters; Grenzen müssen im zulässigen Reglerbereich liegen (Konsistenzprüfung).',
        '',
        'Matrixfälle (`validation.scenario_matrix.sets.<name>[]`): `id`, `scenario` (Pfad relativ zur Matrixdatei), '
        'optional `speed_ms` [m/s], `duration_s` [s] (= `scenario.simulation_duration_s`), `overrides` (Punktpfade).',
        '',
    ]
    return '\n'.join(lines)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--check', action='store_true', help='nur prüfen, ob die Datei aktuell ist')
    args = parser.parse_args(argv)
    text = generate()
    if args.check:
        current = TARGET.read_text(encoding='utf-8') if TARGET.exists() else ''
        print('CURRENT' if current == text else 'OUTDATED')
        return 0 if current == text else 1
    TARGET.parent.mkdir(parents=True, exist_ok=True)
    TARGET.write_text(text, encoding='utf-8')
    print(TARGET)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
