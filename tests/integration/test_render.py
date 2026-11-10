"""Optional adapter-level rendering smoke tests, including slider callbacks."""

import tempfile
from pathlib import Path
import unittest
import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
from torquevector.cli.dashboard import dashboard
from torquevector.visualization.plots import curves_figure, history_figure
from torquevector.domain.vehicle import VehicleParameters
from torquevector.domain.controls import DriverControls
from torquevector.domain.tyre import TyreParameters


class RenderingTests(unittest.TestCase):
    def setUp(self):
        self.car = VehicleParameters()
        self.tyre = TyreParameters()
        self.controls = DriverControls()

    def test_headless_png_and_curves(self):
        fig, samples = dashboard(self.car, self.tyre, self.controls, speed_ms=18, duration_s=0.6, interactive=False)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'dashboard.png'
            fig.savefig(path)
            self.assertGreater(path.stat().st_size, 1000)
        plt.close(fig)
        for create in (lambda: curves_figure(self.tyre), lambda: history_figure(samples)):
            fig = create()
            self.assertGreater(len(fig.axes), 0)
            plt.close(fig)

    def test_slider_refresh_stays_functional(self):
        fig, _ = dashboard(self.car, self.tyre, self.controls, speed_ms=18, duration_s=0.3, interactive=True)
        sliders = fig._widgets[:8]
        self.assertEqual(len(sliders), 8)
        sliders[0].set_val(-4)
        sliders[2].set_val(0.6)
        sliders[6].set_val(0.08)
        self.assertGreater(len(fig.axes), 10)
        plt.close(fig)


if __name__ == '__main__':
    unittest.main()
