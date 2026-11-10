import json, tempfile, unittest
from pathlib import Path
from torquevector.evaluation.architecture import scan
import torquevector


class ArchitectureTests(unittest.TestCase):
    def test_all_package_layers_and_cycles(self):
        report = scan(Path(torquevector.__file__).parent)
        self.assertEqual(report['violations'], [])
        self.assertEqual(report['cycles'], [])

    def test_audit_detects_boundary_overload_and_nested_cycles(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            for layer in ('physics', 'control'):
                (root / layer).mkdir()
            (root / 'physics/a.py').write_text(
                'def f():\n    import torquevector.control.b\n    import matplotlib.pyplot\n'
            )
            (root / 'control/b.py').write_text('import torquevector.physics.a\n')
            report = scan(root)
            self.assertTrue(report['violations'])
            self.assertTrue(report['cycles'])

    def test_audit_detects_path_and_wildcard_hacks(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            (root / 'domain').mkdir()
            (root / 'domain/a.py').write_text('from math import *\nimport sys\nsys.path.insert(0,"bad")\n')
            report = scan(root)
            self.assertGreaterEqual(len(report['violations']), 2)
