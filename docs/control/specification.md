# Control, Observation, and Torque Allocation Specification

This document specifies the mathematical control architecture, reference-generation laws, state observation contract, discrete-time sample-and-hold execution, bounded least-squares torque allocation, and inner-loop wheel slip control implemented in [`src/torquevector/control/`](../../src/torquevector/control/).

---

## 1 Baseline OFF (`src/torquevector/control/controllers/off.py`)

The passive reference configuration (`mode: off`, [`configs/controllers/off.yaml`](../../configs/controllers/off.yaml)) commands an identically zero corrective yaw moment at every outer-loop control tick:

$$M_{z,\text{cmd}}[k] = 0\text{ N}\cdot\text{m} \quad \forall k \in \mathbb{N}_0.$$

Under identical driver steering commands $\delta_{\text{cmd}}(t)$, longitudinal speed-tracking total drive force $F_{x,\text{tot}}(t)$, steering actuator dynamics ($\tau_\delta = 0.045\text{ s}$), and symmetrical baseline drive torque distribution $T_{\text{base},i} = \tfrac{1}{4} R_{\text{eff}}\max(F_{x,\text{tot}}, 0)$, the `OFF` controller provides a strict counterfactual baseline for quantifying active torque-vectoring yaw-rate tracking and lateral path-following improvements (`Q11.1`, `Q11.3`).

---

## 2 Yaw reference (`src/torquevector/control/references.py`)

Active yaw-moment controllers regulate the vehicle yaw rate $r(t)$ and sideslip angle $\beta(t) = \operatorname{atan2}(v_y, v_x)$ against a causal, friction-bounded reference trajectory $(r_{\text{ref}}(t), \beta_{\text{ref}}(t))$ with $\beta_{\text{ref}}(t) \equiv 0\text{ rad}$.

### 2.1 Reference Source Modes (`driver.yaw_reference.source`)

Two causal reference generation modes are supported in [`YawControlPipeline.desired_yaw_rate()`](../../src/torquevector/control/pipeline.py) and [`make_maneuver_preview()`](../../src/torquevector/simulation/preview_adapter.py):

1. **`driver_steering` (Canonical Default since V12.1, `configs/simulation/driver.yaml`)**:
   Derives the raw kinematic yaw-rate demand directly from the realized front steering angle $\delta_{\text{act}}$ and longitudinal speed $v_x = \max(u, v_{\min})$ ($v_{\min} = 2.0\text{ m/s}$):
   $$r_{\text{raw}}(t) = s(t)\,\frac{v_x(t)\,\tan\delta_{\text{act}}(t)}{L}, \qquad s(t) = \begin{cases} 3\tau^2 - 2\tau^3, & \tau = \operatorname{clip}(t / T_{\text{ramp}}, 0, 1)\text{ (open-loop)}, \\ 1, & \text{(path-following)}, \end{cases}$$
   where $T_{\text{ramp}} = 0.55\text{ s}$ is a smoothstep engagement ramp for open-loop steering manoeuvres.

2. **`path_curvature` (Historical V11/V12.0 Legacy Mode, `configs/profiles/regression.yaml`)**:
   Derives the raw demand from the geometric road centreline curvature $\kappa_{\text{path}}(s)$:
   $$r_{\text{raw}}(t) = s(t)\,v_x(t)\,\kappa_{\text{path}}(X, Y).$$
   *Structural Limitation*: When paired with an outer-loop PD path-tracking steering driver ($\delta_{\text{cmd}} = \arctan(L\kappa_{\text{path}}) + k_y e_y + k_\psi e_\psi + k_r(r_{\text{path}} - r)$), `path_curvature` omits the corrective yaw rate demanded by the driver's lateral-error feedback $k_y e_y + k_\psi e_\psi$. Consequently, a yaw controller tracking $v_x \kappa_{\text{path}}$ actively opposes the driver's corrective steering during lane-change recovery. Selecting `driver_steering` aligns the inner yaw-moment reference with the driver's instantaneous steering intent.

### 2.2 Friction-Envelope Saturation and First-Order Reference Filtering

To prevent demanding physically unattainable yaw rates on low-friction surfaces, $r_{\text{raw}}$ is bounded by the quasi-steady lateral acceleration ceiling (`friction_bounded_yaw_rate`):

$$r_{\max}(v_x, \mu_{\text{est}}) = \frac{\mu_{\text{est}}\,g}{\max(v_x, v_{\min})}, \qquad r_{\text{sat}}(t) = \operatorname{clip}\!\big(r_{\text{raw}}(t),\; -r_{\max}(v_x, \mu_{\text{est}}),\; +r_{\max}(v_x, \mu_{\text{est}})\big),$$

where $\mu_{\text{est}} = \frac{1}{4}\sum_{i=1}^4 \mu_{\text{est},i}$ is the mean wheel friction coefficient provided by the configured `FrictionPerception` estimator (never the hidden ground-truth road map unless `estimator: ground_truth` is explicitly selected). The saturated demand is smoothed by a discrete-time first-order low-pass filter (`FirstOrderYawFilter`, time constant $\tau_{\text{ref}} = 0.09\text{ s}$, sampling period $T_s = 0.04\text{ s}$):

$$r_{\text{ref}}[k] = (1 - \alpha_{\text{ref}})\,r_{\text{ref}}[k-1] + \alpha_{\text{ref}}\,r_{\text{sat}}[k], \qquad \alpha_{\text{ref}} = \frac{T_s}{\tau_{\text{ref}} + T_s}.$$

---

## 3 State observation (`src/torquevector/control/estimation.py`)

The controller observation contract is defined by the [`StateEstimator`](../../src/torquevector/control/interfaces.py) protocol, which maps the plant `VehicleState` to an `EstimatedState` tuple containing:
- longitudinal velocity $v_x$ $[\text{m/s}]$,
- lateral velocity $v_y$ $[\text{m/s}]$ and sideslip angle $\beta = \operatorname{atan2}(v_y, \max(|v_x|, 0.5))$ $[\text{rad}]$,
- yaw rate $r$ $[\text{rad/s}]$,
- estimated road friction $\mu_{\text{est}}$ $[-]$.

In nominal simulation (`DirectStateEstimator`), $(v_x, v_y, r)$ are observed directly at each outer sampling instant $t_k = k T_s$, while $\mu_{\text{est}}$ is supplied by [`FrictionPerception`](../../src/torquevector/simulation/friction_perception.py) (`ground_truth`, `constant_nominal`, `delayed_filtered` with lag $\tau_\mu$ and bias, `corrupted`, or `map_preview`). Controllers have **no access** to plant state derivatives $(\dot{v}_x, \dot{v}_y, \dot{r})$, actual tyre forces $(F_{x,i}, F_{y,i}, F_{z,i})$, or wheel slip ratios $\kappa_i$ (`Q11.5`).

---

## 4 Control laws (`src/torquevector/control/controllers/`)

All outer-loop yaw-moment controllers execute in discrete time at $T_s = 0.04\text{ s}$ ($25\text{ Hz}$) with zero-order hold (ZOH) between sampling instants. Let $e_r[k] = r_{\text{ref}}[k] - r[k]$ denote the yaw-rate tracking error and $\mathbf{x}_{\text{lat}} = [\beta,\; r]^\top \in \mathbb{R}^2$ the reduced lateral state vector.

| Controller | Implementation | Horizon $N$ | Yaw Moment Limit $M_{z,\max}$ | Primary Tuning Parameters (`configs/controllers/*.yaml`) |
|---|---|:---:|---:|---|
| **`OFF`** | [`off.py`](../../src/torquevector/control/controllers/off.py) | — | $0\text{ Nm}$ | Passive baseline ($M_z \equiv 0$) |
| **`P`** | [`pid.py`](../../src/torquevector/control/controllers/pid.py) | $0$ | $2{,}000\text{ Nm}$ | $K_p = 4{,}400\text{ Nm/(rad/s)}$, $K_\beta = 1{,}450\text{ Nm/rad}$ |
| **`LQR`** | [`lqr.py`](../../src/torquevector/control/controllers/lqr.py) | $\infty$ | $2{,}000\text{ Nm}$ | Bryson scales: $\beta_{\text{scale}} = 0.05\text{ rad}$, $r_{\text{scale}} = 0.08\text{ rad/s}$, $M_{\text{scale}} = 1{,}500\text{ Nm}$; speed grid $8\text{--}34\text{ m/s}$ |
| **`LTV-MPC`** | [`ltv_mpc.py`](../../src/torquevector/control/controllers/ltv_mpc.py) | $6$ ($0.24\text{ s}$) | $900\text{ Nm}$ | $\beta_{\text{scale}} = 0.06\text{ rad}$, $r_{\text{scale}} = 0.065\text{ rad/s}$, $w_u = 0.12$, $w_{\Delta u} = 0.06$, $\Delta M_{z,\max} = 450\text{ Nm}$, $w_{\text{slack}} = 60$ |
| **`NMPC`** | [`nmpc.py`](../../src/torquevector/control/controllers/nmpc.py) | $5$ ($0.20\text{ s}$) | $900\text{ Nm}$ | $\beta_{\text{scale}} = 0.06\text{ rad}$, $r_{\text{scale}} = 0.065\text{ rad/s}$, $w_u = 0.12$, $w_{\Delta u} = 0.06$, $\Delta M_{z,\max} = 450\text{ Nm}$, $w_{\text{slack}} = 60$, IPOPT |

### 4.1 Proportional Yaw-Rate + Sideslip Damping (`P`)
Commands a memoryless linear feedback moment with soft sideslip damping activation above $|\beta| > \beta_{\text{db}} = 0.02\text{ rad}$:
$$M_{z,\text{raw}}[k] = K_p\,(r_{\text{ref}}[k] - r[k]) - s_\beta(\beta[k])\,K_\beta\,\beta[k], \qquad M_{z,\text{cmd}}[k] = \operatorname{clip}\!\big(M_{z,\text{raw}}[k],\; -M_{z,\max},\; +M_{z,\max}\big).$$

### 4.2 Gain-Scheduled Continuous-Time LQR (`LQR`)
Linearizing the single-track bicycle model at speed $v_x$ with cornering stiffnesses $C_{\alpha f}, C_{\alpha r}$ (`linear_bicycle_matrices` in [`src/torquevector/prediction/bicycle_linear.py`](../../src/torquevector/prediction/bicycle_linear.py)) yields the continuous-time state-space system for the error state $\tilde{\mathbf{x}} = [\beta,\; r - r_{\text{ref}}]^\top$:

$$\dot{\tilde{\mathbf{x}}} = \mathbf{A}(v_x)\,\tilde{\mathbf{x}} + \mathbf{B}\,M_z, \qquad \mathbf{A}(v_x) = \begin{bmatrix} -\dfrac{C_{\alpha f} + C_{\alpha r}}{m v_x} & \dfrac{b C_{\alpha r} - a C_{\alpha f}}{m v_x^2} - 1 \\[10pt] \dfrac{b C_{\alpha r} - a C_{\alpha f}}{I_z} & -\dfrac{a^2 C_{\alpha f} + b^2 C_{\alpha r}}{I_z v_x} \end{bmatrix}, \quad \mathbf{B} = \begin{bmatrix} 0 \\ \dfrac{1}{I_z} \end{bmatrix}.$$

With Bryson diagonal weighting matrices $\mathbf{Q} = \operatorname{diag}(\beta_{\text{scale}}^{-2},\; r_{\text{scale}}^{-2})$ and $R = M_{\text{scale}}^{-2}$, the continuous-time Algebraic Riccati Equation (CARE)
$$\mathbf{A}(v_x)^\top \mathbf{P}(v_x) + \mathbf{P}(v_x)\mathbf{A}(v_x) - \mathbf{P}(v_x)\mathbf{B}R^{-1}\mathbf{B}^\top\mathbf{P}(v_x) + \mathbf{Q} = \mathbf{0}$$
is solved offline across a 14-point speed grid $v_x \in [8.0, 34.0]\text{ m/s}$ via `scipy.linalg.solve_continuous_are`. Online, the feedback gain vector $\mathbf{K}(v_x) = R^{-1}\mathbf{B}^\top\mathbf{P}(v_x) = [k_\beta(v_x),\; k_r(v_x)]$ is linearly interpolated in $v_x$:
$$M_{z,\text{cmd}}[k] = \operatorname{clip}\!\Big(-k_\beta(v_x)\,\beta[k] - k_r(v_x)\,\big(r[k] - r_{\text{ref}}[k]\big),\; -M_{z,\max},\; +M_{z,\max}\Big).$$

### 4.3 Condensed Linear Time-Varying MPC (`LTV-MPC`)
At each sampling instant, `ltv_step_matrix()` in [`src/torquevector/prediction/ltv.py`](../../src/torquevector/prediction/ltv.py) computes the local operating-point axle cornering stiffnesses $(C_{\alpha f,k}, C_{\alpha r,k})$ via central finite differences of the nonlinear combined-slip lateral tyre force around the current slip angles $(\alpha_{f,0}, \alpha_{r,0})$ and normal loads, and performs exact zero-order-hold discretization over $T_s$ via the augmented matrix exponential (`scipy.linalg.expm`):

$$\exp\!\left(\begin{bmatrix} \mathbf{A}_k & \mathbf{B}_u & \mathbf{B}_\delta \\ \mathbf{0} & 0 & 0 \end{bmatrix} T_s\right) = \begin{bmatrix} \mathbf{A}_{d,k} & \mathbf{B}_{d,k} & \mathbf{E}_{d,k} \\ \mathbf{0} & 1 & 0 \end{bmatrix}, \qquad \mathbf{x}_{j+1} = \mathbf{A}_{d,k}\mathbf{x}_j + \mathbf{B}_{d,k} u_j + \mathbf{E}_{d,k}\delta_j + \mathbf{d}_k,$$

where $u_j = M_{z,j} / 1000$ $[\text{kNm}]$ is the scaled control input and $\mathbf{d}_k$ is the affine linearization offset capturing the exact nonlinear force operating point. By eliminating the predicted state sequence $\mathbf{X} = \mathbf{F}\mathbf{x}_0 + \boldsymbol{\Phi}\mathbf{U} + \mathbf{c}$ over the $N = 6$ step horizon, the finite-horizon optimal control problem is solved via Sequential Least Squares Programming (`scipy.optimize.minimize(..., method='SLSQP')`):

$$\min_{\mathbf{U} \in \mathbb{R}^N}\; \sum_{j=0}^{N-1} \left( \|\mathbf{x}_{j+1} - \mathbf{x}_{\text{ref},j+1}\|_{\mathbf{Q}_j}^2 + w_u\left(\frac{M_{z,j}}{M_{\text{scale}}}\right)^2 + w_{\Delta u}\left(\frac{M_{z,j} - M_{z,j-1}}{\Delta M_{\text{scale}}}\right)^2 + w_{\text{slack}}\,\|\boldsymbol{\xi}_{j+1}\|_2^2 \right),$$

where the terminal weight $\mathbf{Q}_{N-1} = \mathbf{P}_\infty$ is the solution of the Discrete Algebraic Riccati Equation (`scipy.linalg.solve_discrete_are`), $\boldsymbol{\xi}_{j+1} = \max\big(\mathbf{0},\; [|\beta_{j+1}| - \beta_{\max},\; |r_{j+1}| - r_{\max}]^\top\big)$ is an exact soft state constraint penalty ($\beta_{\max} = 0.14\text{ rad}$, $r_{\max} = 0.85\text{ rad/s}$), and hard input/slew bounds $|M_{z,j}| \le 900\text{ Nm}$ are enforced.

### 4.4 Reduced-Order Nonlinear Single-Track MPC (`NMPC`)
Formulates a direct single-shooting Nonlinear Program (NLP) in CasADi (`casadi.Opti`, solved with IPOPT) over $N = 5$ steps ($T_s = 0.04\text{ s}$). The prediction model (`casadi_rhs` in [`src/torquevector/prediction/nonlinear_bicycle.py`](../../src/torquevector/prediction/nonlinear_bicycle.py)) integrates the continuous-time nonlinear single-track equations via explicit 4th-order Runge–Kutta (`RK4`) with smooth hyperbolic-tangent axle lateral force saturation:

$$F_{y,f}(\alpha_f) = -D_f(\mu_{\text{est}})\tanh\!\left(\frac{C_{\alpha f}\,\alpha_f}{D_f(\mu_{\text{est}})}\right), \qquad F_{y,r}(\alpha_r) = -D_r(\mu_{\text{est}})\tanh\!\left(\frac{C_{\alpha r}\,\alpha_r}{D_r(\mu_{\text{est}})}\right),$$

subject to hard box constraints on input magnitude $|M_{z,j}| \le 900\text{ Nm}$, slew rate $|M_{z,j} - M_{z,j-1}| \le 450\text{ Nm}$, and non-negative slack variables $\boldsymbol{\epsilon}_j \ge \mathbf{0}$ for state bounds $|\beta_j| \le \beta_{\max} + \epsilon_{\beta,j}$, $|r_j| \le r_{\max} + \epsilon_{r,j}$.

> [!NOTE]
> **Critical Control-Theoretic Analysis — Why LQR Outperforms LTV-MPC & NMPC on Steady-State Circle Tracking (`Q11.3`)**:
> Across the four nominal Q11 manoeuvres, LQR achieves $+27.3\,\%$ to $+39.0\,\%$ yaw-rate RMSE reduction over `OFF` (qualifying across all nominal and robustness cases), whereas LTV-MPC ($+19.6\,\%$) and NMPC ($+18.9\,\%$) fall $0.4\text{--}1.1$ percentage points short of the $20\,\%$ threshold on the $15\text{ m/s}$ constant-radius circle (while exceeding $+21.6\,\%$ to $+37.3\,\%$ on the remaining three manoeuvres). Three structural design factors explain this behaviour:
> 1. **Tighter Torque Authority Ceiling**: LTV-MPC and NMPC are configured with $M_{z,\max} = 900\text{ Nm}$ (`configs/controllers/{ltv_mpc,nmpc}.yaml`), less than half of the $2{,}000\text{ Nm}$ limit granted to LQR and P.
> 2. **Input Magnitude Penalty without Steady-State Feedforward**: LTV-MPC and NMPC penalize absolute input magnitude $w_u (M_z / M_{\text{scale}})^2$ ($w_u = 0.12$) rather than deviation from the required steady-state understeer compensation moment $M_{z,\text{ff}}$, and include no integral state. In steady-state cornering, a non-zero $M_z$ is required indefinitely to counteract natural vehicle understeer; penalizing $M_z^2$ directly induces a proportional steady-state tracking droop.
> 3. **Sideslip Regularization against $\beta_{\text{ref}} = 0$ on a Constant-Radius Turn**: On a $42\text{ m}$ circle at $15\text{ m/s}$, kinematic geometry dictates a non-zero steady-state sideslip angle $\beta_{\text{ss}} \approx -b/R + \alpha_r \neq 0$. Because the MPC stage cost penalizes $(\beta - 0)^2 / \beta_{\text{scale}}^2$ alongside $(r - r_{\text{ref}})^2 / r_{\text{scale}}^2$, the optimizer trades off yaw-rate tracking against sideslip suppression.

---

## 5 Torque allocation (`src/torquevector/control/allocation.py`, `allocation_friction.py`)

The torque allocator maps the baseline longitudinal wheel drive torques $\mathbf{T}_{\text{base}} \in \mathbb{R}^4$ and the outer-loop yaw moment command $M_{z,\text{cmd}}$ into four admissible individual wheel drive torques $\mathbf{T} = \mathbf{T}_{\text{base}} + \Delta\mathbf{T} \in \mathbb{R}^4$ (`WHEEL_ORDER = ('FL', 'FR', 'RL', 'RR')`).

### 5.1 Bounded Variable Least Squares (BVLS) Formulation
Using the planar longitudinal moment arms $y_i \in \{+t/2, -t/2, +t/2, -t/2\}$ and effective rolling radius $R_{\text{eff}}$, the yaw moment generated by differential drive torques $\Delta\mathbf{T}$ is approximated by
$$M_z(\Delta\mathbf{T}) = \sum_{i=1}^4 \ell_i\,\Delta T_i, \qquad \ell_i = -\frac{y_i}{R_{\text{eff}}} \implies \boldsymbol{\ell} = \frac{t}{2 R_{\text{eff}}}\begin{bmatrix} -1 & +1 & -1 & +1 \end{bmatrix}^\top.$$
To preserve total longitudinal propulsion ($\sum_{i=1}^4 \Delta T_i = 0$) while realizing $M_{z,\text{cmd}}$ with minimum actuator effort, `allocate_yaw()` solves the weighted linear least-squares problem via `scipy.optimize.lsq_linear` (BVLS):

$$\min_{\Delta\mathbf{T} \in \mathbb{R}^4}\; \left\|\begin{bmatrix} w_M\,\boldsymbol{\ell}^\top \\ w_F\,\mathbf{1}^\top \\ \mathbf{W}_{\text{reg}}^{1/2} \end{bmatrix} \Delta\mathbf{T} - \begin{bmatrix} w_M\,M_{z,\text{cmd}} \\ 0 \\ \mathbf{0} \end{bmatrix}\right\|_2^2 \qquad \text{subject to} \quad \Delta\mathbf{T}_{\min} \le \Delta\mathbf{T} \le \Delta\mathbf{T}_{\max},$$

with priority weights $w_M = 3.0$, $w_F = 2.0$, and axle regularization $\mathbf{W}_{\text{reg}}^{1/2} = \operatorname{diag}(w_{\text{front}}, w_{\text{front}}, w_{\text{rear}}, w_{\text{rear}}) = \operatorname{diag}(0.025, 0.025, 0.018, 0.018)$ (biasing torque vectoring slightly toward the higher-loaded rear axle).

### 5.2 Actuator, Load, and Friction Bounds (`mode: friction_aware`)
In the default `friction_aware` mode (`src/torquevector/control/allocation_friction.py`), the upper torque bound for each wheel $i$ combines the physical drivetrain actuator limit $T_{\text{act,max}} = 800\text{ Nm}$, the maximum differential authority $\Delta T_{\max} = 420\text{ Nm}$, and the quasi-static friction-circle capacity $\tilde{T}_{\max,i}$:

$$\widehat{F}_{z,i}(v_x, r) = F_{z,i}(a_x = 0,\; a_y = v_x r), \qquad \tilde{T}_{\max,i} = R_{\text{eff}}\,\rho_{\text{res}}\,\mu_{\text{est},i}\,\widehat{F}_{z,i}(v_x, r),$$

where $\rho_{\text{res}} = 0.85$ (`friction_reserve_fraction`) reserves lateral grip margin on the Kamm circle. The box constraints passed to BVLS are:
$$\Delta T_{\min,i} = \max\!\big(-T_{\text{base},i},\; -\Delta T_{\max}\big), \qquad \Delta T_{\max,i} = \min\!\big(\max(T_{\text{base},i},\; \min(T_{\text{act,max}},\, \tilde{T}_{\max,i})) - T_{\text{base},i},\; +\Delta T_{\max}\big),$$
guaranteeing $0 \le T_i \le T_{\text{act,max}}$, $T_i \le \max(T_{\text{base},i}, \tilde{T}_{\max,i})$, and $|T_i - T_{\text{base},i}| \le \Delta T_{\max}$ across all operating states (`Q11.2`).

---

## 6 Wheel slip control (`src/torquevector/control/slip_control.py`)

When enabled (`slip_control.enabled: true`), an inner-loop discrete-time velocity-form PI wheel-slip controller (`WheelSlipController`) executes every $T_{\text{inner}} = 0.005\text{ s}$ ($200\text{ Hz}$, 8 sub-steps per outer $40\text{ ms}$ yaw-control tick) to prevent wheel spin (ASR, $\kappa_i > \kappa_{\text{ref,drive}} = 0.12$) and wheel lock (ABS, $-\kappa_i > \kappa_{\text{ref,brake}} = 0.10$) by scaling the outer-loop wheel torque via $T_{\text{eff},i} = (1 - r_i)\,T_{\text{outer},i}$ with reduction factor $r_i \in [0, r_{\max}]$. Full mathematical details and multi-rate interaction analysis are provided in [`docs/control/slip-control.md`](slip-control.md).
