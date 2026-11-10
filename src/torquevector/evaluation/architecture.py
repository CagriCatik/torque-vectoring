"""AST architecture audit, including imports nested in functions/conditionals."""

import ast, importlib.util
from pathlib import Path

LAYERS = {
    'domain',
    'physics',
    'prediction',
    'control',
    'simulation',
    'evaluation',
    'optimization',
    'calibration',
    'infrastructure',
    'visualization',
    'cli',
}
ALLOWED = {
    'domain': {'domain'},
    'physics': {'domain', 'physics'},
    'prediction': {'domain', 'physics', 'prediction'},
    'control': {'domain', 'physics', 'prediction', 'control'},
    'simulation': {'domain', 'physics', 'prediction', 'control', 'simulation', 'evaluation', 'infrastructure'},
    'visualization': {'domain', 'physics', 'visualization'},
    'evaluation': {'domain', 'physics', 'prediction', 'control', 'simulation', 'evaluation', 'infrastructure'},
    'calibration': {'domain', 'physics', 'calibration'},
    'infrastructure': {'domain', 'infrastructure', 'calibration'},
    'optimization': LAYERS - {'visualization', 'cli'},
    'cli': LAYERS,
}

# Fach- und Simulationslogik erhält typisierte Objekte; nur Loader und Composition Root kennen YAML.
YAML_LAYERS = {'infrastructure', 'cli'}


def layer(name):
    parts = name.split('.')
    return parts[1] if len(parts) > 1 and parts[1] in LAYERS else None


def scan(package_root):
    root = Path(package_root)
    modules = {}
    for path in root.rglob('*.py'):
        rel = path.relative_to(root).with_suffix('')
        parts = list(rel.parts)
        if parts[-1] == '__init__':
            parts.pop()
        name = '.'.join(['torquevector'] + parts)
        modules[name] = path
    graph = {name: set() for name in modules}
    violations = []
    for name, path in modules.items():
        tree = ast.parse(path.read_text(), filename=str(path))
        source_layer = layer(name)
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                targets = [a.name for a in node.names]
            elif isinstance(node, ast.ImportFrom):
                base = node.module or ''
                if node.level:
                    package = name if path.name == '__init__.py' else name.rpartition('.')[0]
                    base = importlib.util.resolve_name('.' * node.level + base, package)
                targets = [base]
                if any(a.name == '*' for a in node.names):
                    violations.append(f'{name}:{node.lineno}: wildcard import')
            else:
                targets = []
            for target in targets:
                if target in graph:
                    graph[name].add(target)
                if not source_layer:
                    continue  # one-way compatibility facades have no implementation ownership
                if target.startswith('torquevector.'):
                    dest = layer(target)
                    if dest not in ALLOWED[source_layer]:
                        violations.append(f'{name}:{node.lineno}: forbidden {source_layer} -> {target}')
                if source_layer in ('domain', 'physics', 'prediction', 'control') and target.split('.')[0] in (
                    'matplotlib',
                    'PIL',
                    'pathlib',
                    'csv',
                    'json',
                    'os',
                ):
                    violations.append(f'{name}:{node.lineno}: core imports {target}')
                if target.split('.')[0] == 'yaml' and source_layer not in YAML_LAYERS:
                    violations.append(f'{name}:{node.lineno}: YAML outside configuration infrastructure/CLI ({target})')
                if (
                    name == 'torquevector.control.controllers.nmpc'
                    and target == 'torquevector.control.controllers.ltv_mpc'
                ):
                    violations.append(f'{name}: concrete NMPC -> LTV')
            if isinstance(node, ast.Call):
                called = ast.unparse(node.func)
                if called in ('sys.path.insert', 'sys.path.append'):
                    violations.append(f'{name}:{node.lineno}: sys.path mutation')
    cycles = []
    active = []
    finished = set()

    def visit(name):
        if name in active:
            cycles.append(active[active.index(name) :] + [name])
            return
        if name in finished:
            return
        active.append(name)
        for target in sorted(graph[name]):
            visit(target)
        active.pop()
        finished.add(name)

    for name in sorted(graph):
        visit(name)
    return {
        'modules': len(modules),
        'edges': sum(map(len, graph.values())),
        'violations': violations,
        'cycles': cycles,
        'graph': {k: sorted(v) for k, v in sorted(graph.items())},
    }
