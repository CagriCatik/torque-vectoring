"""Die generierte Parameterreferenz entspricht exakt Schema und Referenzkonfiguration."""

import importlib.util, unittest
from pathlib import Path

ROOT = Path(__file__).parents[2]


class ParameterReferenceIsCurrent(unittest.TestCase):
    def test_generated_reference_matches(self):
        spec = importlib.util.spec_from_file_location(
            'generate_parameter_reference', ROOT / 'scripts/configuration/generate_parameter_reference.py'
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self.assertEqual(
            (ROOT / 'docs/configuration/PARAMETER_REFERENCE.md').read_text(encoding='utf-8'),
            module.generate(),
            'docs/configuration/PARAMETER_REFERENCE.md veraltet: Generator ausführen',
        )


if __name__ == '__main__':
    unittest.main()
