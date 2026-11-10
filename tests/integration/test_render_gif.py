import tempfile
from pathlib import Path
import unittest

from torquevector.visualization.animation import AnimationConfig, render_controller_comparison_gif
from torquevector.simulation.runner import simulate_controller
from torquevector.domain.scenarios import presets


class GifRenderingTests(unittest.TestCase):
    def test_gif_export_smoke(self):
        spec = presets()['circle_track']
        seqs = {}
        for kind in ('off', 'ltv'):
            _, samples = simulate_controller(kind, spec, 0.8)
            seqs[kind] = samples
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'comparison.gif'
            render_controller_comparison_gif(
                seqs,
                path,
                title='test',
                config=AnimationConfig(fps=6, max_frames=25, tail_seconds=0.0, dpi=80),
            )
            self.assertTrue(path.is_file())
            self.assertGreater(path.stat().st_size, 1000)


if __name__ == '__main__':
    unittest.main()
