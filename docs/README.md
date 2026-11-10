# `torquevector` Technical and Scientific Documentation Index

Welcome to the technical and scientific documentation of `torquevector` — a modular four-wheel planar vehicle dynamics, heterogeneous road-friction, wheel-slip control (ABS/ASR), and optimal torque-vectoring simulation framework (`OFF`, `P`, `LQR`, `LTV-MPC`, `NMPC`).

> [!IMPORTANT]
> **Epistemic Status**: All vehicle, wheel, and tyre parameters in this repository are **synthetic engineering assumptions** or calibrated against a synthetic two-dimensional brush-model truth generator (`SYNTHETIC / NOT VEHICLE VALIDATED`). Experimental vehicle validation (`Gate Q10`) is explicitly marked **`REQUIRED_REAL_DATA`**.

---

## 1 Mathematical Modelling & Vehicle Physics (`docs/model/`)

- **[Vehicle and Tyre Model Specification](model/specification.md)**: Complete 11-state differential-algebraic system, ISO 8855 coordinate frames, wheel-hub velocity transformations, coupled quasi-static normal load transfer root solver, Pacejka Magic Formula pure slip, and opposing-slip elliptic combined-slip mechanics.
- **[Self-Steering Characteristic and Handling Limits](model/handling.md)**: Analytical linear single-track derivation of the understeer gradient $K$, characteristic speed $v_{\text{char}}$, critical speed $v_{\text{crit}}$, ISO 4138 quasi-steady ramp-steer evaluation, $(\beta, r)$ phase-plane stability analysis, and $g$-$g$ acceleration envelopes.
- **[Spatial Road-Friction Field (`RoadFrictionModel`)](model/road-friction.md)**: Position-dependent surface friction $\mu_{\text{road}}(X, Y)$, $C^2$-continuous quintic smoothstep transitions, exact RK45 boundary-crossing event splitting, Pacejka similarity scaling, and controller friction perception modes.

---

## 2 Control Theory, Allocation, and Slip Control (`docs/control/`)

- **[Control, Observation, and Torque Allocation Specification](control/specification.md)**: Causal yaw-rate reference generation (`driver_steering` vs. `path_curvature`), friction-envelope saturation, gain-scheduled continuous-time `LQR`, condensed matrix-exponential `LTV-MPC` (SLSQP), reduced-order CasADi/IPOPT `NMPC`, and friction-aware Bounded Variable Least Squares (`BVLS`) torque allocation.
- **[Inner-Loop Wheel Slip Control (ABS / ASR)](control/slip-control.md)**: Multi-rate $200\text{ Hz}$ inner / $25\text{ Hz}$ outer zero-order-hold architecture, velocity-form PI torque reduction law, and split-$\mu$ braking/traction verification.

---

## 3 Verification, Quality Gates, and Critical Review (`docs/validation/` & `docs/quality/`)

- **[Torque-Vectoring Assessment (Gate Q11)](validation/torque-vectoring.md)**: Quantitative comparison across nominal, robustness, and degraded manoeuvre matrices, including a critical control-theoretic analysis of `LQR` vs. `LTV-MPC`/`NMPC`.
- **[Automated Quality Audit Pipeline (`torquevector quality-audit`)](validation/quality-audit.md)**: Five-stage verification pipeline, analytical reference benchmarks, runtime physical invariants, and the audited 13-gate scorecard ($10/13$ gates passing at $10/10$; `Q02`, `Q03`, `Q10` honestly capped by synthetic data blockers).
- **[Quality Gates Framework (Q01–Q13)](validation/quality-gates.md)** & **[Full Gate Specification (`QUALITY_GATES.md`)](quality/QUALITY_GATES.md)**: Grading rules, critical sub-checks, and release blockers (`B1`–`B5`).
- **[Scientific Review of Model Limitations](limitations.md)**: Critical discussion of unmodelled pitch/roll/heave dynamics, fixed roll-stiffness split, Lorentzian combined-slip surrogate, steady-state MPC droop, and one-way allocator/ASR coupling.

---

## 4 Software Architecture, Configuration, and Usage

- **[Getting Started & Installation](getting-started.md)**: Environment setup, dependency lockfiles, and quick-start workflow.
- **[CLI User Guide](user-guide.md)**: Complete command reference for `simulate`, `compare`, `multi-mu`, `handling`, `q11`, `quality-audit`, `render`, `plot-tyre`, and `optimize`.
- **[YAML Configuration System](configuration.md)** & **[Auto-Generated Parameter Reference](configuration/PARAMETER_REFERENCE.md)**: Schema validation, provenance tracking, and all $175+$ configurable parameters.
- **[Telemetry & Diagnostic Plots Reference](plots.md)**: CSV column definitions, report sections, and animation formats.
- **[Software Architecture](architecture.md)** & **[Development & Testing Guide](development.md)**: Layered package graph, AST invariants, coverage configuration, and zero-skip testing policy.
