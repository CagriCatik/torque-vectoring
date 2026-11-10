# Changelog

All notable architectural, physical, control-theoretic, and verification changes to `torquevector` are documented in this file.

---

## [12.1.0] — 2026-10-31

### Added
- **Spatial Road-Friction Field (`RoadFrictionModel`)**:
  - Added position-dependent surface friction $\mu_{\text{road}}(x, y)$ supporting uniform surfaces, longitudinal $\mu$-steps (`mu_step`), lateral split-$\mu$ (`split_mu`), rectangular patches (`patch`), and multi-zone polygonal/rectangular layouts (`zones`).
  - Implemented $C^2$-continuous quintic smoothstep blending $S(\xi) = 6\xi^5 - 15\xi^4 + 10\xi^3$ across transition widths and exact ODE integration sub-step splitting via `scipy.integrate.solve_ivp` root-finding events at sharp surface boundaries (`transition_width_m = 0`).
  - Added configurable estimator perception modes (`ground_truth`, `constant_nominal`, `delayed_filtered`, `corrupted`, and `map_preview`) to decouple physical plant road friction from controller/allocator knowledge.
- **Friction-Aware Torque Allocation (`mode: friction_aware`)**:
  - Extended the Bounded Variable Least Squares (BVLS) yaw-moment allocator (`src/torquevector/control/allocation_friction.py`) with quasi-static normal-load estimation ($a_x \approx 0, a_y \approx v_x r$) and per-wheel friction-circle torque bounds $\tilde{T}_{\max,i} = R_{\text{eff}}\,\rho\,\mu_{\text{est},i}\,F_{z,i}$.
- **Inner-Loop Wheel Slip Control (`WheelSlipController`, ABS/ASR)**:
  - Implemented a 5 ms (`period_s = 0.005`) discrete-time velocity-form PI wheel-slip limiter operating inside the 40 ms outer yaw-moment sample-and-hold loop, reducing drive (ASR) or braking (ABS) torque magnitude via $r_i \in [0, 1]$ when $|\kappa_i|$ exceeds target thresholds ($\kappa_{\text{ref,drive}} = 0.12$, $\kappa_{\text{ref,brake}} = 0.10$).
- **Self-Steering & Handling Analysis (`torquevector handling`)**:
  - Added analytical linear single-track self-steering gradient $K = \frac{m}{L}\left(\frac{b}{C_{\alpha f}} - \frac{a}{C_{\alpha r}}\right)$, characteristic velocity $v_{\text{char}} = \sqrt{L/K}$, critical velocity $v_{\text{crit}} = \sqrt{-L/K}$, ISO 4138 quasi-steady ramp-steer evaluation, $(\beta, r)$ phase-plane stability portraits, and quasi-static friction-ellipse $g$-$g$ envelope computation.
- **Automated 13-Gate Quality Audit (`torquevector quality-audit` & `torquevector q11`)**:
  - Added complete verification suite across 13 quality categories (Q01–Q13, 65 sub-checks) with analytical reference benchmarks, runtime physical invariant checks, and reproducible scorecard generation.

### Changed
- **Yaw-Rate Reference Generation (`driver_steering` Default)**:
  - Changed the default yaw-rate reference source in `configs/simulation/driver.yaml` from `path_curvature` ($r_{\text{ref}} = v_x \kappa_{\text{path}}$) to `driver_steering` ($r_{\text{ref,raw}} = \frac{v_x \tan\delta_{\text{act}}}{L}$, bounded by $|r_{\text{ref}}| \le \frac{\mu_{\text{est}} g}{\max(v_x, v_{\min})}$ and filtered with $\tau_{\text{ref}} = 0.09\text{ s}$). This eliminates the structural conflict between the outer path-tracking steering PD controller and the inner yaw-moment controller during transient lane-change and constant-radius manoeuvres.
- **Documentation & Scientific Rigor**:
  - Overhauled all technical and scientific documentation under `docs/` and `README.md` with complete state-space derivations, tyre mechanics, optimal control formulations, empirical benchmark tables, and critical discussions of model and controller limitations.

---

## [12.0.0] — 2026-04-15

### Added
- **Hexagonal / Layered Package Architecture (`src/torquevector/`)**:
  - Consolidated legacy versioned scripts (`V1`–`V11`) into a strictly layered Python package (`domain` $\leftarrow$ `physics` $\leftarrow$ `prediction` $\leftarrow$ `control` $\leftarrow$ `simulation` $\leftarrow$ `evaluation` $\leftarrow$ `visualization` $\leftarrow$ `cli`, with `infrastructure` providing configuration and persistence).
  - Enforced zero cyclic dependencies and strict layer boundaries via AST architecture tests (`tests/architecture/`).
- **Schema-Validated YAML Configuration System (`configs/`)**:
  - Extracted all 175+ physical, tyre, actuator, controller, allocation, solver, scenario, and visualization parameters into modular YAML files with provenance tracking (`origin`), unit validation, and deterministic SHA-256 run snapshotting.
- **Frozen AST Physics & Control Invariants**:
  - Locked 44 core mathematical functions and dataclasses via normalized AST SHA-256 hashes (`tests/fixtures/structure_definition_hashes.json`) and numerical regression baselines (`tests/system/`).

---

## [11.0.0] — Historical Baseline (Pre-Migration)

- Introduced 11-state planar four-wheel vehicle dynamics model with coupled quasi-static normal load transfer, dynamic wheel spin states $\omega_i$, first-order steering actuator $\delta_{\text{act}}$, synthetic Pacejka Magic Formula + elliptic combined-slip tyre model, and five yaw-moment control modes (`OFF`, `P`, `LQR`, `LTV-MPC`, and reduced-order `NMPC` via CasADi/IPOPT).

