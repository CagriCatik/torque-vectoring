# Automated Quality Audit Pipeline (`torquevector quality-audit`)

This document describes the architecture, conservative scoring logic, artifact outputs, and empirical results of the automated 13-gate quality audit implemented in [`src/torquevector/evaluation/quality_audit.py`](../../src/torquevector/evaluation/quality_audit.py) and [`src/torquevector/cli/main.py`](../../src/torquevector/cli/main.py).

---

## 1 Purpose and Non-Destructive Audit Principle

The `torquevector quality-audit` command executes the complete verification and validation evidence pipeline in a single reproducible run and generates an evidence-backed **proposed** scorecard (`proposed_scorecard.csv`) inside a timestamped results directory `results/quality_audit/<timestamp>/`.

> [!IMPORTANT]
> **Immutable Official Template**: The repository file [`docs/quality/quality_scorecard.csv`](../quality/quality_scorecard.csv) is the unpopulated official release template ( verified by `tests/unit/test_quality_audit.py` and `tests/quality/test_q13_software.py`). `torquevector quality-audit` **never** overwrites `docs/quality/quality_scorecard.csv`; instead, it writes `proposed_scorecard.csv` with `reviewer = PENDING_REVIEW` so that every score can be inspected against its generated evidence artifact before sign-off.

---

## 2 Five-Stage Execution Pipeline

When invoked via `torquevector quality-audit`, the auditor executes five sequential stages:

1. **Quality Gate Test Suite (`tests/quality/test_q01_*.py` – `test_q13_*.py`)**:
   Runs all $61$ automated quality tests mapping $1\text{:}1$ to the $65$ sub-checks (`Q01.1` through `Q13.5`, with `Q10.1`–`Q10.5` consolidated into the epistemic honesty check `test_Q10_status_required_real_data_declared`). Results are serialized to `quality_tests.json`.
2. **Core Branch & Line Coverage Analysis (`coverage.py`)**:
   Executes the full test suite (`unit`, `integration`, `regression`, `architecture`, `system`, `quality`) under [`coverage.py`](../../.coveragerc) across the six core mathematical layers (`domain`, `physics`, `prediction`, `control`, `simulation`, `evaluation`). Achieving $\ge 90.0\,\%$ core coverage is required for full credit ($2/2$) on sub-check `Q13.3`.
3. **Analytical Reference Benchmarks (`reference_benchmarks.py`)**:
   Computes exact analytical solutions (`reference_benchmarks.csv` and `reference_benchmarks.json`) for:
   - Stationary wheel normal loads $F_{z,i}(a_x=0, a_y=0)$ and quasi-static load transfer $\Delta F_{z,x}, \Delta F_{z,y}$,
   - Pure-slip Magic Formula peak forces and initial stiffness slopes $\left.\partial F/\partial s\right|_{s=0} = B C D$,
   - Linear single-track steady-state yaw rate $r_{\text{ss}} = \frac{v_x \delta}{L + K v_x^2}$ vs. nonlinear four-wheel simulation at small steer angles ($\delta = 0.8^\circ$, relative discrepancy $< 1.2\,\%$),
   - Unpowered coasting mechanical energy balance $\Delta E_{\text{kin}} = W_{\text{aero}} + W_{\text{roll}} + W_{\text{tyre,diss}}$.
4. **Runtime Physical Invariant Auditing (`invariants.py`)**:
   Simulates five canonical verification cases (`straight`, `coast`, `circle_left`, `circle_right`, `dlc_lqr`) and verifies 15 continuous-time physical invariants (`invariants/*.json` and `invariants_summary.json`), including vertical load conservation $\sum_i F_{z,i} = m g$, positive wheel normal loads $F_{z,i} > 0$, combined-slip Kamm-circle compliance $\sqrt{F_{x,i}^2 + F_{y,i}^2} \le D_i(1 + 10^{-6})$, left/right mirror antisymmetry $r(- \delta) = -r(+\delta)$, and non-negative dissipative power in coasting.
5. **Conservative Point Proposal (`propose_points`) and Gate Grading (`assess`)**:
   Applies the conservative scoring rules defined in `src/torquevector/evaluation/quality_audit.py`:
   - Any failing test receives `0` points (`status = failed`).
   - Passing software/mathematical verification checks receive `2` points (`status = verified`).
   - Checks in `DATA_LIMITED = {'Q02.1', 'Q02.2', 'Q02.3', 'Q03.2'}` are capped at **`1` point** (`status = partial`) despite passing all numerical NRMSE thresholds, because the underlying calibration dataset `data/synthetic/synthetic_brush_measurements.csv` is synthetic (`Blocker B1`).
   - All five experimental validation checks `Q10.1`–`Q10.5` receive **`0` points** (`status = not_applicable_without_real_data`, `Blocker B5`).

---

## 3 Audited Gate Scores (`proposed_scorecard.csv`)

| Gate | Category Title | Proposed Score | Critical Checks Open | Gate Verdict | Reason for Cap / Block |
|---|---|---:|:---:|:---:|---|
| **`Q01`** | Parameters and Validity Domain | **$10/10$** | None | **PASS** | All parameters, units, origins, and envelopes verified |
| **`Q02`** | Pure-Slip Tyre Model (`Fx`, `Fy`) | **$7/10$** | None | **FAIL** ($< 8/10$) | `Q02.1`–`Q02.3` capped at $1/2$ each (`DATA_LIMITED`, Blocker B1: synthetic tyre dataset) |
| **`Q03`** | Combined Slip & Kamm Circle | **$9/10$** | **`Q03.2`** ($1/2$) | **FAIL** (critical $< 2$) | `Q03.2` capped at $1/2$ (`DATA_LIMITED`, Blocker B1: synthetic combined-slip dataset) |
| **`Q04`** | Wheel Kinematics and Slip | **$10/10$** | None | **PASS** | Independent matrix transformation & fail-closed low-speed checks verified |
| **`Q05`** | Vehicle Dynamics & Load Transfer | **$10/10$** | None | **PASS** | Newton–Euler balance, static/dynamic loads, and wheel-lift boundary verified |
| **`Q06`** | Steering, Yaw & Handling | **$10/10$** | None | **PASS** | Linear single-track benchmark, mirror symmetry, and $K$ gradient verified |
| **`Q07`** | Numerics and Time Integration | **$10/10$** | None | **PASS** | Step-halving convergence ($\Delta t$ vs. $\Delta t/2$), algebraic loop, and ZOH ticks verified |
| **`Q08`** | Physical Plausibility & Invariants | **$10/10$** | None | **PASS** | Energy dissipation, friction scaling, and runtime invariants verified |
| **`Q09`** | Calibration & Overfitting Prevention | **$10/10$** | None | **PASS** | Run-grouped train/hold-out/stress splits, multi-start stability, and covariance verified |
| **`Q10`** | Experimental Validation | **$0/10$** | **`Q10.1`, `Q10.2`** | **FAIL** (Blocker B5) | Requires physical test-vehicle telemetry (`REQUIRED_REAL_DATA`) |
| **`Q11`** | Torque Vectoring and Control | **$10/10$** | None | **PASS** | Reference, BVLS friction-aware allocator, $\ge 20\,\%$ LQR/P yaw improvement, and causality verified |
| **`Q12`** | 2D Visualization and Telemetry | **$10/10$** | None | **PASS** | Wheel force vectors, Kamm circles, CSV telemetry exactness, and scale consistency verified |
| **`Q13`** | Software Architecture & Reproducibility | **$10/10$** | None | **PASS** | Acyclic layers, $>90\,\%$ core coverage, zero skips, deterministic CLI, and fresh wheel install verified |

**Summary**: **$10/13$ gates pass with $10/10$ points** ($116/130$ total points across the 13 categories). Only the three categories that by definition require physical laboratory or test-track measurements (`Q02`, `Q03`, `Q10`) remain blocked (`overall: BLOCKED`), demonstrating complete software and mathematical verification alongside strict epistemic honesty.
