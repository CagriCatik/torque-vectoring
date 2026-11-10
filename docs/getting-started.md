# Getting Started and Reproducible Setup

This guide covers environment installation, dependency locking, quick-start simulation commands, and verification of a clean local setup.

---

## 1 System Requirements and Installation

`torquevector` requires **Python $\ge 3.10$** (verified on macOS arm64 with Python 3.13.5).

```bash
# 1. Create and activate a virtual environment
python3 -m venv .venv
source .venv/bin/activate

# 2. Install the package with all optional dependencies (NMPC, visualization, and developer tools)
python -m pip install -e '.[dev,nmpc,viz]'

# 3. For bit-reproducible numerical environments on macOS arm64 (Python 3.13), constrain to the lockfile:
python -m pip install -e '.[dev,nmpc,viz]' -c requirements-lock-macos-arm64-py313.txt
```

### Dependency Groups (`pyproject.toml`)
- **Core (`numpy>=2.0`, `scipy>=1.11`, `pyyaml>=6.0`)**: Nonlinear 4-wheel plant, RK45 integration, Powell hybrid load-transfer solver, BVLS allocation, and `OFF`, `P`, `LQR`, `LTV-MPC` controllers.
- **`nmpc` (`casadi>=3.6,<4`)**: Symbolic automatic differentiation and IPOPT nonlinear programming solver for `NMPC`.
- **`viz` (`matplotlib>=3.7`, `pillow>=9`)**: Multi-page PDF reports, static PNG/SVG plots, and animated GIF exports.
- **`dev` (`coverage>=7`, `black>=24`, `build>=1`)**: Branch coverage analysis, code formatting, and wheel packaging.

---

## 2 First Simulation and Verification Commands

```bash
# 1. Validate the default YAML configuration tree
torquevector config validate --config configs/default.yaml

# 2. Simulate all five controllers on the Double Lane Change and generate a PDF report
torquevector compare --scenario double_lane_change --controllers off p lqr ltv nmpc

# 3. Render static PNG plots and an animated GIF from the stored telemetry (without re-simulating)
torquevector render --run-id latest --format png
torquevector render --run-id latest --format gif

# 4. Run the smoke test suite
torquevector validate --suite smoke
```

---

## 3 Output Directory Structure (`results/`)

Every simulation command creates a self-contained, timestamped run directory under `results/runs/<timestamp>_<label>/`:

```text
results/runs/<run_id>/
├── manifest.json                       # Run status, git revision, version, validity envelope
├── report.pdf                          # Multi-page engineering diagnostic report
├── config/
│   ├── resolved_config.yaml            # Full merged configuration snapshot
│   ├── resolved_config.origins.json    # Parameter file & line provenance
│   ├── resolved_config.sources.json    # SHA-256 hashes of all input YAML files
│   └── resolved_config.fingerprint     # Canonical configuration SHA-256 digest
├── telemetry/
│   ├── off.csv                         # Full 11-state + 4-wheel force/slip/torque time series
│   ├── p.csv
│   ├── lqr.csv
│   ├── ltv.csv
│   └── nmpc.csv
├── metrics/
│   └── comparison.json                 # Tracking RMSE, sideslip, tyre utilization, solver timings
├── plots/                              # Generated via `torquevector render --format png`
└── animations/                         # Generated via `torquevector render --format gif`
```
