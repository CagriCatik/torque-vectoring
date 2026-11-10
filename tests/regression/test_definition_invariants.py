"""Frozen pre-migration mathematics/defaults; imports and docstrings excluded."""

import ast, hashlib, importlib.util, json, unittest
from pathlib import Path


class Normalize(ast.NodeTransformer):
    def visit_Import(self, node):
        return None

    def visit_ImportFrom(self, node):
        return None

    def visit_Expr(self, node):
        if isinstance(node.value, ast.Constant) and isinstance(node.value.value, str):
            return None
        return self.generic_visit(node)


class DefinitionInvariants(unittest.TestCase):
    def test_original_physics_solver_parameters_and_pso_ast(self):
        rows = json.loads((Path(__file__).parents[1] / 'fixtures/structure_definition_hashes.json').read_text())
        for row in rows:
            with self.subTest(source=row['source'], symbol=row['symbol']):
                spec = importlib.util.find_spec(row['module'])
                tree = ast.parse(Path(spec.origin).read_text())
                node = next(n for n in tree.body if getattr(n, 'name', None) == row['symbol'])
                digest = hashlib.sha256(
                    ast.dump(Normalize().visit(node), include_attributes=False).encode()
                ).hexdigest()
                self.assertEqual(digest, row['sha256_ast'])
