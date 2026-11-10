# Calibration Data and Provenance (`data/`)

This directory stores calibration datasets, provenance metadata, and synthetic reference truth files used for tyre parameter identification and quality-gate verification (Gates Q02 and Q03).

---

## 1. Directory Structure

### `data/synthetic/` — Primary Synthetic Tyre Calibration Dataset
Contains the canonical brush-model reference dataset and the resulting Pacejka Magic Formula + combined-slip parameter identification report:
- [`synthetic_brush_measurements.csv`](synthetic/synthetic_brush_measurements.csv) — $2{,}520$ operating points generated from a two-dimensional isotropic brush-model truth generator with explicit Gaussian measurement noise ($\sigma_F = \max(12.0\text{ N},\; 0.008\,F_z)$) and run-grouped splits (`R00`–`R03`: training, `R04`–`R05`: hold-out validation, `R06`: thermal/pressure stress test at $T = 27\,^\circ\text{C}$, $p = 212\text{ kPa}$).
- [`metadata.json`](synthetic/metadata.json) — Explicit provenance descriptor (`dataset_kind: "synthetic_independent_surrogate"`, `seed: 20260417`, SI unit contracts, and run-split definitions).
- [`fit_report.json`](synthetic/fit_report.json) — Multi-start bounded nonlinear least-squares (`scipy.optimize.least_squares`) identification results, training/hold-out/stress RMSE and NRMSE metrics, parameter covariance condition number, and residual quantile statistics.
- [`synthetic_calibration_diagnostics.png`](synthetic/synthetic_calibration_diagnostics.png) — Multi-panel diagnostic plot of pure longitudinal $F_x(\kappa)$, pure lateral $F_y(\alpha)$, combined-slip polar envelopes, and residual distributions across vertical loads $F_z \in \{2.5, 3.6, 4.8, 5.6\}\text{ kN}$.

### `data/calibration_examples/` — Secondary Synthetic Demo Dataset
Contains a lightweight synthetic Pacejka demonstration dataset (`dataset_kind: "synthetic_demo"`, seed `721`) used for pipeline smoke tests (`src/torquevector/calibration/tyre_fit.py`):
- [`synthetic_tyre_data.csv`](calibration_examples/synthetic_tyre_data.csv) — Synthetic operating points with `train` and `holdout` split labels.
- [`synthetic_tyre_metadata.json`](calibration_examples/synthetic_tyre_metadata.json) — Provenance and unit contract metadata.
- [`example_fit_report.json`](calibration_examples/example_fit_report.json) — Example identification output.

---

## 2. Physical Units & Sign Conventions

All calibration files enforce strict SI unit contracts verified at load time (`read_measurements` in `src/torquevector/calibration/tyre_fit.py` and `load_dataset` in `src/torquevector/calibration/surrogate.py`):

| Column / Field | Symbol | Unit | Description |
| :--- | :---: | :---: | :--- |
| `run_id` | — | — | Measurement sweep identifier (`R00`–`R06`) used for leakage-free group splitting |
| `split` | — | — | Partition assignment (`train`, `holdout`, `stress`) |
| `kappa` | $\kappa$ | $[-]$ | Longitudinal slip ratio ($\kappa > 0$ traction, $\kappa < 0$ braking) |
| `alpha_rad` | $\alpha$ | $\text{rad}$ | Lateral slip angle |
| `fz_n` | $F_z$ | $\text{N}$ | Vertical normal wheel load ($F_z > 0$) |
| `vx_ms` | $v_x$ | $\text{m/s}$ | Longitudinal wheel-carrier speed |
| `pressure_kpa` | $p$ | $\text{kPa}$ | Tyre inflation pressure |
| `temp_c` | $T$ | $^\circ\text{C}$ | Tyre bulk temperature |
| `fx_n` | $F_x$ | $\text{N}$ | Measured/synthetic longitudinal tyre force |
| `fy_n` | $F_y$ | $\text{N}$ | Measured/synthetic lateral tyre force |

---

## 3. Epistemic Status & Packaging Synchronization

- **No Physical Tyre Bench Data (`Blocker B1`)**: Both `data/synthetic/` and `data/calibration_examples/` are explicitly marked as synthetic in their JSON metadata (`synthetic_independent_surrogate` and `synthetic_demo`). Consequently, quality gates Q02 (`7/10`) and Q03 (`9/10`, critical sub-check `Q03.2 = 1/2`) remain capped until physical flat-trac or trailer measurements (`dataset_kind: "measured"`) are supplied.
- **Packaged Wheel Resources**: `src/torquevector/resources/synthetic/` mirrors `data/synthetic/` byte-for-byte so that installed wheel distributions execute identically outside the source checkout. This synchronization is enforced by `tests/architecture/test_preservation.py`.
