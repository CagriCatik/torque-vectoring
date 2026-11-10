# Torque-Vectoring Assessment and Empirical Evaluation (Gate Q11)

This document presents the empirical verification results, quantitative performance matrices, and control-theoretic analysis of the torque-vectoring controllers (`OFF`, `P`, `LQR`, `LTV-MPC`, `NMPC`) evaluated under Quality Gate Q11 (`torquevector q11`, implemented in [`src/torquevector/evaluation/q11_assessment.py`](../../src/torquevector/evaluation/q11_assessment.py)).

---

## 1 Gate Q11 Requirements and Verification Status

Quality Gate Q11 (*Torque Vectoring and Control*, [`docs/quality/QUALITY_GATES.md`](../quality/QUALITY_GATES.md)) specifies five sub-checks (two critical, minimum passing score $8/10$):

| Check | Critical | Requirement | Status | Points |
|---|:---:|---|:---:|---:|
| **`Q11.1`** | **Yes** | Explicit specification of yaw-rate reference $r_{\text{ref}}$, state observation contract, control laws, and passive `OFF` baseline (`docs/control/specification.md`) | **PASS** | $2/2$ |
| **`Q11.2`** | **Yes** | Torque allocator produces four admissible wheel torques satisfying actuator ($0 \le T_i \le T_{\text{act,max}}$), differential ($|\Delta T_i| \le \Delta T_{\max}$), and normal-load/friction-circle ($T_i \le \max(T_{\text{base},i}, \tilde{T}_{\max,i})$) limits | **PASS** | $2/2$ |
| **`Q11.3`** | No | At least one active torque-vectoring controller achieves $\ge 20\,\%$ yaw-rate tracking RMSE improvement over `OFF` across every feasible nominal manoeuvre without degrading path tracking or violating sideslip bounds | **PASS** | $2/2$ |
| **`Q11.4`** | No | Defined, numerically bounded degradation under physically unattainable yaw demands, low road friction ($\mu = 0.35$), and tight actuator torque saturation ($T_{\max} = 220\text{ Nm}$) | **PASS** | $2/2$ |
| **`Q11.5`** | No | Strict causal input-output separation: controllers output only yaw-moment commands `MomentCommand` and have no access to plant state derivatives, tyre forces, or wheel slip ratios $\kappa_i$ | **PASS** | $2/2$ |
| **Total** | — | **Gate Q11 Overall Status** | **PASS** | **$10/10$** |

---

## 2 Nominal Manoeuvre Matrix Results (`Q11.3_nominal`)

Evaluated across the four canonical nominal manoeuvres (`run.matrix_set: q11_nominal`: circle track at $15$ and $19\text{ m/s}$, double lane change at $17$ and $21\text{ m/s}$) with `driver.yaw_reference.source: driver_steering` and `allocation.friction_limit.mode: friction_aware`. Qualification requires $\ge 20.0\,\%$ yaw-rate RMSE improvement and $\ge -10.0\,\%$ path-tracking RMSE improvement relative to `OFF` in every single manoeuvre:

| Manoeuvre Case | Speed $v_{x,0}$ | Duration | Metric | `OFF` (Baseline) | `P` | `LQR` | `LTV-MPC` | `NMPC` |
|---|---:|---:|---|---:|---:|---:|---:|---:|
| **`circle_track_15`** ($R = 90\text{ m}$) | $15.0\text{ m/s}$ | $8.0\text{ s}$ | $\operatorname{RMSE}_r$ $[\text{rad/s}]$<br>Yaw impr. vs. `OFF`<br>Path impr. vs. `OFF` | $0.04316$<br>—<br>— | $0.03650$<br>$+15.42\,\%$<br>$+0.02\,\%$ | $0.03139$<br>**$+27.26\,\%$**<br>$-3.14\,\%$ | $0.03472$<br>$+19.56\,\%$<br>$-3.49\,\%$ | $0.03501$<br>$+18.87\,\%$<br>$-3.33\,\%$ |
| **`circle_track_19`** ($R = 90\text{ m}$) | $19.0\text{ m/s}$ | $8.0\text{ s}$ | $\operatorname{RMSE}_r$ $[\text{rad/s}]$<br>Yaw impr. vs. `OFF`<br>Path impr. vs. `OFF` | $0.06725$<br>—<br>— | $0.04954$<br>**$+26.34\,\%$**<br>$-0.28\,\%$ | $0.04099$<br>**$+39.04\,\%$**<br>$-3.57\,\%$ | $0.04002$<br>**$+40.49\,\%$**<br>$-3.67\,\%$ | $0.04064$<br>**$+39.56\,\%$**<br>$-3.45\,\%$ |
| **`double_lane_change_17`** | $17.0\text{ m/s}$ | $9.0\text{ s}$ | $\operatorname{RMSE}_r$ $[\text{rad/s}]$<br>Yaw impr. vs. `OFF`<br>Path impr. vs. `OFF` | $0.12232$<br>—<br>— | $0.09049$<br>**$+26.02\,\%$**<br>$+4.33\,\%$ | $0.07476$<br>**$+38.88\,\%$**<br>$-0.91\,\%$ | $0.08223$<br>**$+32.77\,\%$**<br>$+0.15\,\%$ | $0.08275$<br>**$+32.35\,\%$**<br>$+0.31\,\%$ |
| **`double_lane_change_21`** | $21.0\text{ m/s}$ | $9.0\text{ s}$ | $\operatorname{RMSE}_r$ $[\text{rad/s}]$<br>Yaw impr. vs. `OFF`<br>Path impr. vs. `OFF` | $0.24159$<br>—<br>— | $0.16949$<br>**$+29.84\,\%$**<br>$-0.64\,\%$ | $0.14927$<br>**$+38.21\,\%$**<br>$-4.52\,\%$ | $0.15485$<br>**$+35.91\,\%$**<br>$-4.86\,\%$ | $0.15603$<br>**$+35.41\,\%$**<br>$-3.29\,\%$ |

- **Qualified Nominal Controller ($\ge 20\,\%$ improvement across all 4 cases)**: **`LQR`** ($+27.26\,\%$ to $+39.04\,\%$).
- **Why `LTV-MPC` ($+40.49\,\%$) and `NMPC` ($+39.56\,\%$) Lead on `circle_track_19` but Reach $+19.56\,\%$ / $+18.87\,\%$ on `circle_track_15`**:
  At $19\text{ m/s}$ on the $90\text{ m}$ circle ($a_y \approx 4.0\text{ m/s}^2$), nonlinear tyre load sensitivity and sideslip effects become prominent, and `LTV-MPC` ($+40.49\,\%$) and `NMPC` ($+39.56\,\%$) achieve the highest yaw-rate RMSE reduction of all controllers. Conversely, at the lower speed of $15\text{ m/s}$ ($a_y \approx 2.5\text{ m/s}^2$), `LTV-MPC` ($+19.56\,\%$) and `NMPC` ($+18.87\,\%$) fall $0.44\text{--}1.13$ percentage points short of the $20.0\,\%$ threshold due to:
  1. Their configured yaw-moment authority ceiling $M_{z,\max} = 900\text{ Nm}$ (vs. $2{,}000\text{ Nm}$ for `LQR` and `P`).
  2. Finite-horizon input magnitude regularization $w_u (M_z / M_{\text{scale}})^2$ ($w_u = 0.12$) without an integral state or steady-state understeer feedforward $M_{z,\text{ff}}$, producing a slight steady-state proportional droop at moderate lateral acceleration.
  3. Simultaneous stage-cost penalization of sideslip $(\beta - 0)^2 / \beta_{\text{scale}}^2$ on a constant-radius turn where kinematic steady-state sideslip $\beta_{\text{ss}} \neq 0$.

---

## 3 Parameter Robustness Matrix (`q11_robust`)

Evaluated across five off-nominal physical and surface perturbations (`run.matrix_set: q11_robust`):

| Robustness Case | Perturbation | `OFF` $\operatorname{RMSE}_r$ | `P` Yaw Impr. | `LQR` Yaw Impr. | `LTV-MPC` Yaw Impr. | `NMPC` Yaw Impr. |
|---|---|---:|---:|---:|---:|---:|
| **`mass_plus_10pct_circle`** | Vehicle mass $+10\,\%$ ($1760\text{ kg}$), Circle @ $15\text{ m/s}$ | $0.0445\text{ rad/s}$ | $+14.11\,\%$ | **$+25.23\,\%$** | $+18.89\,\%$ | $+18.42\,\%$ |
| **`mass_plus_10pct_dlc`** | Vehicle mass $+10\,\%$ ($1760\text{ kg}$), DLC @ $17\text{ m/s}$ | $0.1088\text{ rad/s}$ | **$+21.93\,\%$** | **$+35.80\,\%$** | **$+30.05\,\%$** | **$+29.72\,\%$** |
| **`mu_minus_10pct_circle`** | Tyre friction $-10\,\%$ ($\mu = 0.904$), Circle @ $15\text{ m/s}$ | $0.0458\text{ rad/s}$ | $+17.03\,\%$ | **$+29.59\,\%$** | **$+21.77\,\%$** | **$+21.09\,\%$** |
| **`mu_minus_10pct_dlc`** | Tyre friction $-10\,\%$ ($\mu = 0.904$), DLC @ $17\text{ m/s}$ | $0.1446\text{ rad/s}$ | **$+30.94\,\%$** | **$+43.63\,\%$** | **$+36.81\,\%$** | **$+36.55\,\%$** |
| **`mixed_mu_lane_change`** | Heterogeneous dry/wet patches ($\mu \in [0.40, 0.95]$), DLC @ $17\text{ m/s}$ | $0.2137\text{ rad/s}$ | **$+36.07\,\%$** | **$+47.87\,\%$** | **$+33.12\,\%$** | **$+34.08\,\%$** |

**`LQR`** qualifies across $100\,\%$ of robustness cases ($+25.23\,\%$ to $+47.87\,\%$ improvement), while **`LTV-MPC`** and **`NMPC`** qualify on $4/5$ robustness cases (exceeding $+33\,\%$ on the mixed-$\mu$ lane change and $+21\,\%$ on the reduced-friction circle).

---

## 4 Degraded & Limit Operating Regime (`Q11.4_degraded`)

Gate `Q11.4` verifies that under extreme, wet, or physically unattainable manoeuvre demands (`run.matrix_set: q11_degraded`), every controller terminates in a well-defined state (`PASS`, `INVALID_MODEL_ENVELOPE`, or `FAIL_CLOSED`) with finite metrics and actuator commands strictly inside bounds:

| Degraded Case | Operating Condition | `OFF` | `P` | `LQR` | `LTV-MPC` | `NMPC` |
|---|---|---|---|---|---|---|
| **`wet_dlc_17`** | Wet asphalt ($\mu = 0.40$) DLC at $17\text{ m/s}$ | `FAIL_CLOSED` ($t=6.84\text{ s}$) | `INVALID_MODEL_ENVELOPE` | **`PASS`** ($\operatorname{RMSE}_r = 0.150$) | `INVALID_MODEL_ENVELOPE` | **`PASS`** ($\operatorname{RMSE}_r = 0.189$) |
| **`wet_circle_19`** | Wet asphalt ($\mu = 0.40$) Circle at $19\text{ m/s}$ | `FAIL_CLOSED` ($t=3.84\text{ s}$) | `INVALID_MODEL_ENVELOPE` | `INVALID_MODEL_ENVELOPE` | `INVALID_MODEL_ENVELOPE` | `INVALID_MODEL_ENVELOPE` |
| **`saturated_dlc_21`** | Tight actuator torque cap ($T_{\max}$ saturated) DLC at $21\text{ m/s}$ | **`PASS`** ($0.269$) | **`PASS`** ($0.229$) | **`PASS`** ($0.226$) | **`PASS`** ($0.218$) | **`PASS`** ($0.218$) |
| **`impossible_dlc_30`** | Physically unattainable $30\text{ m/s}$ severe DLC | `FAIL_CLOSED` | `FAIL_CLOSED` | `FAIL_CLOSED` | `FAIL_CLOSED` | `FAIL_CLOSED` |
| **`impossible_wet_dlc_24`** | Physically unattainable $24\text{ m/s}$ wet DLC ($\mu = 0.40$) | `FAIL_CLOSED` | `INVALID_MODEL_ENVELOPE` | `FAIL_CLOSED` | `INVALID_MODEL_ENVELOPE` | `INVALID_MODEL_ENVELOPE` |

Notably, on **`wet_dlc_17`**, the passive `OFF` vehicle loses control and exceeds the calibrated tyre slip envelope at $t = 6.84\text{ s}$ (`FAIL_CLOSED`), whereas **`LQR`** and **`NMPC`** successfully stabilize the vehicle through the entire $9.0\text{ s}$ wet lane change (**`PASS`**). On **`saturated_dlc_21`**, **`NMPC`** ($0.2177\text{ rad/s}$) and **`LTV-MPC`** ($0.2182\text{ rad/s}$) achieve the lowest tracking error because their constrained optimization explicitly accounts for the actuator saturation limit over the prediction horizon.
