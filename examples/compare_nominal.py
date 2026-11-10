"""Run the canonical controller comparison from Python (equivalent to the CLI call in the README).

    python examples/compare_nominal.py
"""

from torquevector.cli.main import main

if __name__ == '__main__':
    raise SystemExit(
        main(['compare', '--scenario', 'double_lane_change', '--controllers', 'off', 'lqr', 'nmpc', '--no-report'])
    )
