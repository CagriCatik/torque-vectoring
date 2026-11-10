"""AST-Inventar fest verdrahteter numerischer Parameter im Paket ``torquevector``.

Das Skript ist ein Wartungswerkzeug (kein Laufzeitcode). Es klassifiziert jede
numerische Konstante nach ihrem syntaktischen Ort, damit die manuelle
Bewertung in ``docs/configuration/HARDCODED_PARAMETER_AUDIT.md`` vollständig
und reproduzierbar bleibt:

- ``dataclass_default``: Feld-Default einer ``@dataclass``
- ``parameter_default``: Default eines Funktionsparameters
- ``call_keyword``: Literal als Schlüsselwortargument eines Aufrufs (z. B. ``rtol=2e-5``)
- ``module_constant``: Modulweite Zuweisung
- ``inline``: sonstiges Literal in Ausdrücken

Triviale mathematische Literale (0, 1, -1, 2, 0.5) und Array-Indizes werden
gezählt, aber nicht einzeln gelistet.

Aufruf::

    python scripts/configuration/audit_hardcoded.py --output results/hardcoded_inventory.json
"""

from __future__ import annotations

import argparse
import ast
import json
from collections import Counter
from pathlib import Path

TRIVIAL = {0, 1, -1, 2, 0.5, 0.0, 1.0, 2.0, -1.0}
CORE_LAYERS = ('domain', 'physics', 'prediction', 'control', 'simulation')


def _number(node: ast.AST):
    if isinstance(node, ast.Constant) and type(node.value) in (int, float):
        return node.value
    if (
        isinstance(node, ast.UnaryOp)
        and isinstance(node.op, ast.USub)
        and isinstance(node.operand, ast.Constant)
        and type(node.operand.value) in (int, float)
    ):
        return -node.operand.value
    return None


def _is_dataclass(node: ast.ClassDef) -> bool:
    for decorator in node.decorator_list:
        target = decorator.func if isinstance(decorator, ast.Call) else decorator
        if ast.unparse(target).split('.')[-1] == 'dataclass':
            return True
    return False


class Collector(ast.NodeVisitor):
    def __init__(self, module: str, path: str):
        self.module = module
        self.path = path
        self.scope: list[str] = []
        self.rows: list[dict] = []
        self.handled: set[int] = set()
        self.trivial = 0
        self.indices = 0

    def _emit(self, node, kind, name, value):
        self.handled.add(id(node))
        self.rows.append(
            {
                'file': self.path,
                'line': node.lineno,
                'module': self.module,
                'symbol': '.'.join(self.scope) or '<module>',
                'kind': kind,
                'name': name,
                'value': value,
            }
        )

    def visit_ClassDef(self, node):
        self.scope.append(node.name)
        if _is_dataclass(node):
            for item in node.body:
                if isinstance(item, ast.AnnAssign) and item.value is not None:
                    value = _number(item.value)
                    if value is not None:
                        self._emit(item.value, 'dataclass_default', ast.unparse(item.target), value)
                    elif isinstance(item.value, (ast.Tuple, ast.Call)):
                        for sub in ast.walk(item.value):
                            v = _number(sub)
                            if v is not None and id(sub) not in self.handled:
                                self._emit(sub, 'dataclass_default', ast.unparse(item.target), v)
        self.generic_visit(node)
        self.scope.pop()

    def _visit_function(self, node):
        self.scope.append(node.name)
        args = node.args
        positional = args.posonlyargs + args.args
        for arg, default in zip(positional[len(positional) - len(args.defaults) :], args.defaults):
            for sub in ast.walk(default):
                v = _number(sub)
                if v is not None and id(sub) not in self.handled:
                    self._emit(sub, 'parameter_default', arg.arg, v)
        for arg, default in zip(args.kwonlyargs, args.kw_defaults):
            if default is None:
                continue
            for sub in ast.walk(default):
                v = _number(sub)
                if v is not None and id(sub) not in self.handled:
                    self._emit(sub, 'parameter_default', arg.arg, v)
        self.generic_visit(node)
        self.scope.pop()

    visit_FunctionDef = _visit_function
    visit_AsyncFunctionDef = _visit_function

    def visit_Assign(self, node):
        if not self.scope:
            for sub in ast.walk(node.value):
                v = _number(sub)
                if v is not None and id(sub) not in self.handled:
                    self._emit(sub, 'module_constant', ast.unparse(node.targets[0]), v)
        self.generic_visit(node)

    def visit_Call(self, node):
        for keyword in node.keywords:
            v = _number(keyword.value)
            if v is not None and id(keyword.value) not in self.handled:
                self._emit(keyword.value, 'call_keyword', f'{ast.unparse(node.func)}({keyword.arg}=)', v)
            if isinstance(keyword.value, ast.Dict):
                for key, value in zip(keyword.value.keys, keyword.value.values):
                    v = _number(value)
                    if v is not None and id(value) not in self.handled:
                        label = ast.unparse(key) if key is not None else '**'
                        self._emit(value, 'call_keyword', f'{ast.unparse(node.func)}({keyword.arg}[{label}])', v)
        self.generic_visit(node)

    def visit_Subscript(self, node):
        for sub in ast.walk(node.slice):
            if _number(sub) is not None:
                self.handled.add(id(sub))
                self.indices += 1
        self.generic_visit(node)

    def visit_UnaryOp(self, node):
        v = _number(node)
        if v is not None:
            self._literal(node, v)
            self.handled.add(id(node.operand))
            return
        self.generic_visit(node)

    def visit_Constant(self, node):
        v = _number(node)
        if v is not None:
            self._literal(node, v)

    def _literal(self, node, value):
        if id(node) in self.handled:
            return
        if value in TRIVIAL:
            self.trivial += 1
            self.handled.add(id(node))
            return
        self._emit(node, 'inline', None, value)


def scan(package_root: Path) -> dict:
    rows: list[dict] = []
    trivial = indices = 0
    for path in sorted(package_root.rglob('*.py')):
        rel = path.relative_to(package_root.parent)
        parts = list(path.relative_to(package_root).with_suffix('').parts)
        if parts[-1] == '__init__':
            parts.pop()
        module = '.'.join(['torquevector'] + parts)
        collector = Collector(module, str(Path('src') / rel))
        collector.visit(ast.parse(path.read_text(encoding='utf-8'), filename=str(path)))
        rows.extend(collector.rows)
        trivial += collector.trivial
        indices += collector.indices
    for row in rows:
        parts = row['module'].split('.')
        row['layer'] = parts[1] if len(parts) > 1 else None
        row['legacy'] = ('legacy' in row['module']) or row['module'].count('.') == 1
    return {
        'rows': rows,
        'summary': {
            'listed': len(rows),
            'trivial_math_literals': trivial,
            'subscript_indices': indices,
            'by_kind': dict(Counter(r['kind'] for r in rows)),
            'by_layer': dict(Counter(str(r['layer']) for r in rows)),
        },
    }


def core_defaults(report: dict) -> list[dict]:
    """Numerische Dataclass-/Parameter-Defaults der Kernschichten (Architektur-Baseline)."""
    keep = ('dataclass_default', 'parameter_default')
    return sorted(
        (
            {'module': r['module'], 'symbol': r['symbol'], 'name': r['name'], 'value': r['value']}
            for r in report['rows']
            if r['kind'] in keep and r['layer'] in CORE_LAYERS
        ),
        key=lambda r: (r['module'], r['symbol'], r['name'], str(r['value'])),
    )


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--package', type=Path, default=Path(__file__).resolve().parents[2] / 'src/torquevector')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--core-defaults-output', type=Path)
    args = parser.parse_args(argv)
    report = scan(args.package)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=1) + '\n', encoding='utf-8')
    if args.core_defaults_output:
        args.core_defaults_output.write_text(json.dumps(core_defaults(report), indent=1) + '\n', encoding='utf-8')
    print(json.dumps(report['summary'], indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
