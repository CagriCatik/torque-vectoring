# Vehicle and Tyre Model Specification

This document specifies the mathematical equations, coordinate frames, parameter provenance, operating envelope, and structural modelling boundaries of the planar four-wheel vehicle dynamics and tyre simulation model implemented in [`src/torquevector/physics/`](../../src/torquevector/physics/).

Canonical configuration defaults are defined in [`configs/default.yaml`](../../configs/default.yaml); the complete schema-generated parameter catalog is maintained in [`docs/configuration/PARAMETER_REFERENCE.md`](../configuration/PARAMETER_REFERENCE.md).

> [!IMPORTANT]
> **Epistemic Status — Synthetic Parameterization**: All vehicle, wheel, and tyre parameters in this repository are **synthetic engineering assumptions** or nonlinear least-squares fits to a synthetic two-dimensional brush-model dataset (`SYNTHETIC / NOT VEHICLE VALIDATED`). No physical flat-trac tyre bench or instrumented test-track measurements are included (`REQUIRED_REAL_DATA`).

---

## 1 Vehicle (`configs/vehicles/generic_passenger_car.yaml`)

The vehicle body is modelled as a rigid planar chassis of mass $m$ and yaw moment of inertia $I_z$ about the centre of gravity ($\text{CG}$), with wheelbase $L = a + b$, front CG distance $a$, rear CG distance $b = L - a$, equal front/rear track width $t$, and CG height $h_{\text{CG}}$.

| Parameter | Value | Unit | Source | Uncertainty |
|---|---:|---|---|---|
| Vehicle mass `vehicle.mass_kg` | 1600.0 | kg | `configs/vehicles/generic_passenger_car.yaml` (synthetic reference vehicle, C/D-segment EV) | ±5 % (engineering assumptions) |
| Yaw moment of inertia `vehicle.inertia.yaw_kgm2` | 2600.0 | kg·m² | `configs/vehicles/generic_passenger_car.yaml` (synthetic reference vehicle) | ±10 % (engineering assumptions) |
| Wheelbase `vehicle.geometry.wheelbase_m` | 2.76 | m | `configs/vehicles/generic_passenger_car.yaml` (synthetic reference vehicle) | ±0.01 m (nominal geometry) |
| Track width front/rear `vehicle.geometry.track_m` | 1.62 | m | `configs/vehicles/generic_passenger_car.yaml` (synthetic reference vehicle, equal front/rear) | ±0.01 m (nominal geometry) |
| CG to front axle `vehicle.geometry.cg_to_front_axle_m` | 1.18 | m | `configs/vehicles/generic_passenger_car.yaml` ($b = L - a = 1.58\text{ m}$) | ±0.03 m (loading-dependent) |
| CG height `vehicle.geometry.cg_height_m` | 0.52 | m | `configs/vehicles/generic_passenger_car.yaml` (synthetic reference vehicle) | ±0.03 m (engineering assumptions) |
| Aerodynamic drag area `vehicle.aerodynamics.drag_area_m2` | 0.65 | m² | `configs/vehicles/generic_passenger_car.yaml` ($C_d A$) | ±10 % (engineering assumptions) |
| Air density `vehicle.aerodynamics.air_density_kgm3` | 1.225 | kg/m³ | ISA sea-level standard atmosphere | ±3 % (ambient-dependent) |
| Rolling resistance coefficient `vehicle.rolling_resistance.coefficient` | 0.012 | 1 | `configs/vehicles/generic_passenger_car.yaml` | ±20 % (engineering assumptions) |
| Gravitational acceleration `vehicle.environment.gravity_ms2` | 9.81 | m/s² | Standard gravity | ±0.01 m/s² |

---

## 2 Tyre and wheel (`configs/tyres/pacejka_fitted_synthetic.yaml`, `configs/wheels/standard_wheel.yaml`)

The default profile loads [`configs/tyres/pacejka_fitted_synthetic.yaml`](../../configs/tyres/pacejka_fitted_synthetic.yaml), whose parameters are identified via multi-start bounded nonlinear least squares (`scipy.optimize.least_squares`, TRF `soft_l1`) from the synthetic two-dimensional brush-model dataset [`data/synthetic/synthetic_brush_measurements.csv`](../../data/synthetic/synthetic_brush_measurements.csv) ($N = 2{,}520$ operating points across training runs `R00`–`R03`, hold-out runs `R04`–`R05`, and stress run `R06`; see [`data/synthetic/fit_report.json`](../../data/synthetic/fit_report.json)). An uncalibrated baseline parameter set is preserved in [`configs/tyres/pacejka_baseline.yaml`](../../configs/tyres/pacejka_baseline.yaml).

| Parameter | Value | Unit | Source / Provenance | Uncertainty |
|---|---:|---|---|---|
| Tyre size (nominal reference) | 225/45 R17 | — | Synthetic passenger-car tyre assumption (geometry for effective radius) | synthetic |
| Inflation pressure (synthetic dataset) | 240.0 | kPa | `data/synthetic/metadata.json` (`pressure_kpa`, stress run `R06` at 212 kPa); not an active model input | ±5 kPa in synthetic generator |
| Effective rolling radius `wheel.effective_radius_m` | 0.315 | m | `configs/wheels/standard_wheel.yaml` (engineering assumptions) | ±0.005 m |
| Wheel polar inertia `wheel.rotational_inertia_kgm2` | 1.45 | kg·m² | `configs/wheels/standard_wheel.yaml` (wheel + tyre + hub assembly) | ±10 % |
| Nominal friction coefficient `tyre.friction.mu` | 1.0049 | 1 | Least-squares fit to `synthetic_brush_measurements.csv` (baseline: `1.00`) | synthetic fit (hold-out joint NRMSE 3.41 %) |
| Reference load `tyre.friction.reference_load_n` | 4000.0 | N | `configs/tyres/pacejka_fitted_synthetic.yaml` | fixed normalization reference |
| Load sensitivity exponent `tyre.friction.load_sensitivity_exponent` | -0.0716 | 1 | Least-squares fit to `synthetic_brush_measurements.csv` (baseline: `-0.08`) | synthetic fit |
| Longitudinal Magic Formula $(B_x, C_x, E_x)$ | 6.0729 / 1.6393 / 0.3853 | 1 | Least-squares fit to `synthetic_brush_measurements.csv` | synthetic fit |
| Lateral Magic Formula $(B_y, C_y, E_y)$ | 3.6825 / 1.9549 / 0.2959 | 1 | Least-squares fit to `synthetic_brush_measurements.csv` | synthetic fit |
| Combined-slip saturation scales $(\kappa_s, \alpha_s)$ | 0.1395 / 0.2357 | 1 / rad | Least-squares fit to `synthetic_brush_measurements.csv` (baseline: `0.15` / `0.22`) | synthetic fit |
| Friction-ellipse weights $(c_x, c_y)$ | 1.0 / 1.0 | 1 | `configs/tyres/pacejka_fitted_synthetic.yaml` (isotropic Kamm circle in normalized force coordinates) | model definition |

---

## 3 Operating envelope and enforced validity domain

The plant equations enforce a strict fail-closed operating envelope in [`src/torquevector/physics/wheel_rotation.py`](../../src/torquevector/physics/wheel_rotation.py) and [`src/torquevector/physics/tyre_magic_formula.py`](../../src/torquevector/physics/tyre_magic_formula.py). Any excursion outside the verified physical envelope raises an `OutsideEnvelopeError` immediately (`FAIL_CLOSED`) rather than clipping silently or propagating `NaN`/`Inf`:

| Quantity | Validated Envelope | Out-of-Bounds Behaviour |
|---|---|---|
| Vehicle speed and local wheel-carrier longitudinal speed $u_{w,i}$ | $v_x > 0$ and $u_{w,i}$ ≥ 3.0 m/s (`wheel.min_contact_speed_ms`) | Raises `OutsideEnvelopeError` (`FAIL_CLOSED`); standstill ($v_x = 0$), creeping ($u_{w,i} < 3.0\text{ m/s}$), and reverse motion ($v_x < 0$) are outside the valid domain |
| Wheel spin rate $\omega_i$ | $\omega_i \ge 0\text{ rad/s}$ (finite) | Negative or non-finite $\omega_i$ raises `OutsideEnvelopeError` |
| Longitudinal slip κ | \|κ\| ≤ 0.5 (`tyre.operating_limits.max_abs_longitudinal_slip`, calibration coverage $[-0.28, 0.28]$) | $\|\kappa\| > 0.5$ raises `OutsideEnvelopeError` (`FAIL_CLOSED`) |
| Slip angle α | $\|\alpha\| \le 0.45\text{ rad}$ ($\approx 25.8^\circ$; calibration coverage $[-0.25, 0.25]\text{ rad}$) | Evaluated via $\alpha_i = \operatorname{atan2}(v_{w,i}, u_{w,i})$; for $u_{w,i} \ge 3.0\text{ m/s}$, $\alpha_i \in (-\pi/2, \pi/2)$ is smooth and bounded |
| Wheel load Fz | $F_{z,i} > 0\text{ N}$ (no wheel lift; calibration coverage $2{,}500\text{--}5{,}600\text{ N}$) | $F_{z,i} \le 0\text{ N}$ raises `OutsideEnvelopeError("Quasi-static load transfer indicates wheel lift...")` (`FAIL_CLOSED`) |
| Road friction coefficient $\mu$ | $0 < \mu \le 2.5$ (`tyre.friction.mu`, spatial field $\mu_{\text{road}}(x,y) > 0$) | Verified during schema resolution and `TyreParameters` initialization; $\mu_{\text{road}}$ scales peak grip linearly via $\lambda_{\mu,i} = \mu_{\text{road},i}/\mu_{\text{ref}}$ |
| Temperature | **not modelled** (`temperature: not_modelled` in run manifest) | Tyre bulk/surface temperature and pressure effects are omitted from the dynamical plant; stress run `R06` in `data/synthetic/` serves solely as a calibration robustness check |

---

## 4 Coordinate systems and sign conventions

The simulator adheres to the right-handed **ISO 8855** coordinate convention across three frames (implemented in [`src/torquevector/physics/four_wheel.py`](../../src/torquevector/physics/four_wheel.py) and [`src/torquevector/physics/wheel_rotation.py`](../../src/torquevector/physics/wheel_rotation.py)):

1. **Inertial / World Frame $(X, Y, Z)$**:
   - $X$: longitudinal reference axis in the horizontal road plane $[\text{m}]$.
   - $Y$: lateral axis to the left of $X$ $[\text{m}]$.
   - $\psi$: vehicle yaw (heading) angle $[\text{rad}]$, positive counter-clockwise (CCW) from $X$ toward $Y$.
   - Global CG kinematics:
     $$\dot{X} = v_x \cos\psi - v_y \sin\psi, \qquad \dot{Y} = v_x \sin\psi + v_y \cos\psi, \qquad \dot{\psi} = r.$$

2. **Vehicle Body Frame $(x, y, z)$ at the Centre of Gravity ($\text{CG}$)**:
   - $x$: forward along the vehicle longitudinal plane of symmetry ($v_x > 0$ forward).
   - $y$: lateral to the left ($v_y > 0$ to the left; sideslip angle $\beta = \operatorname{atan2}(v_y, v_x)$).
   - $z$: upward normal to the road plane; yaw rate $r = \dot{\psi} > 0$ counter-clockwise (left turn).
   - Wheel-hub coordinates $\mathbf{p}_i = (x_i, y_i)$ in `WHEEL_ORDER = ('FL', 'FR', 'RL', 'RR')`:
     $$\mathbf{p}_{\text{FL}} = \left(+a, +\frac{t}{2}\right), \quad \mathbf{p}_{\text{FR}} = \left(+a, -\frac{t}{2}\right), \quad \mathbf{p}_{\text{RL}} = \left(-b, +\frac{t}{2}\right), \quad \mathbf{p}_{\text{RR}} = \left(-b, -\frac{t}{2}\right), \qquad b = L - a.$$

3. **Wheel Frame $(x_{w,i}, y_{w,i})$ and Steering Geometry**:
   - The front axle employs **parallel steering** (no Ackermann toe geometry, zero static toe, zero camber $\gamma = 0$, and zero pneumatic trail):
     $$\delta_{\text{FL}} = \delta_{\text{FR}} = \delta_{\text{act}}, \qquad \delta_{\text{RL}} = \delta_{\text{RR}} = 0,$$
     where $\delta_{\text{act}} > 0$ denotes a left steer angle.
   - The translational velocity of wheel hub $i$ expressed in the vehicle body frame is $\mathbf{v}_{\text{body},i} = \mathbf{v}_{\text{CG}} + \boldsymbol{\omega} \times \mathbf{p}_i$:
     $$v_{x,\text{body},i} = v_x - r\,y_i, \qquad v_{y,\text{body},i} = v_y + r\,x_i.$$
   - Rotating into the local wheel frame by $-\delta_i$ via the orthogonal rotation matrix $\mathbf{R}(-\delta_i)$ yields the local longitudinal and lateral contact-patch velocities $(u_{w,i}, v_{w,i})$:
     $$\begin{bmatrix} u_{w,i} \\ v_{w,i} \end{bmatrix} = \begin{bmatrix} \cos\delta_i & \sin\delta_i \\ -\sin\delta_i & \cos\delta_i \end{bmatrix} \begin{bmatrix} v_x - r\,y_i \\ v_y + r\,x_i \end{bmatrix}.$$
   - The wheel-frame tyre forces $(F_{x,w,i}, F_{y,w,i})$ are transformed back into the vehicle body frame via $\mathbf{R}(+\delta_i)$ (`rotate_wheel_to_body`):
     $$\begin{bmatrix} F_{x,\text{body},i} \\ F_{y,\text{body},i} \end{bmatrix} = \begin{bmatrix} \cos\delta_i & -\sin\delta_i \\ \sin\delta_i & \cos\delta_i \end{bmatrix} \begin{bmatrix} F_{x,w,i} \\ F_{y,w,i} \end{bmatrix}, \qquad \|\mathbf{F}_{\text{body},i}\|_2 = \|\mathbf{F}_{w,i}\|_2.$$

4. **Slip and Force Sign Conventions**:
   - **Lateral slip angle $\alpha_i$**:
     $$\alpha_i = \operatorname{atan2}(v_{w,i},\; u_{w,i}) \in (-\pi, \pi].$$
     For straight-ahead driving ($v_y = 0, r = 0$) with a left steer input ($\delta_{\text{act}} > 0$), $v_{w,\text{F}} = -v_x \sin\delta_{\text{act}} < 0$, yielding $\alpha_{\text{F}} = -\delta_{\text{act}} < 0$. The lateral tyre law produces a restoring force $F_{y,w,i} = -\operatorname{MF}_y(\alpha_i) > 0$ (pointing left), which generates a positive (CCW) yaw moment $M_z > 0$.
   - **Longitudinal slip ratio $\kappa_i$**:
     $$\kappa_i = \frac{R_{\text{eff}}\,\omega_i - u_{w,i}}{u_{w,i}}.$$
     Free rolling corresponds to $R_{\text{eff}}\,\omega_i = u_{w,i} \implies \kappa_i = 0$ and $F_{x,w,i} = 0$. Driving torque ($\omega_i R_{\text{eff}} > u_{w,i}$) yields $\kappa_i > 0$ and $F_{x,w,i} > 0$; braking ($\omega_i R_{\text{eff}} < u_{w,i}$) yields $\kappa_i < 0$ and $F_{x,w,i} < 0$.

---

## 5 Slip and kinematics special cases

- **Free-Rolling Equilibrium**: When $\omega_i = u_{w,i} / R_{\text{eff}}$, $\kappa_i = 0$ identically and longitudinal tyre force vanishes ($F_{x,w,i} = 0$).
- **Low-Speed / Standstill Singularity Prevention**: Unlike empirical denominator regularizations $\max(u_{w,i}, v_{\epsilon})$ that silently alter effective tyre slip stiffness at low speeds, `wheel_slips()` in [`src/torquevector/physics/wheel_rotation.py`](../../src/torquevector/physics/wheel_rotation.py) enforces a strict domain guard: if $v_x \le 0$ or $\min_i u_{w,i} < u_{\min} = 3.0\text{ m/s}$, an `OutsideEnvelopeError` is raised immediately without division by zero or unflagged model switching.
- **Locked-Wheel / High-Slip Limit**: A fully locked wheel ($\omega_i = 0$ at $u_{w,i} \ge 3.0\text{ m/s}$) yields $\kappa_i = -1.0$, which exceeds the validated slip envelope $|\kappa_i| \le 0.5$ and triggers `OutsideEnvelopeError`. Under active inner-loop slip control ([`docs/control/slip-control.md`](../control/slip-control.md)), ABS limits braking torque to maintain $|\kappa_i|$ near $\kappa_{\text{ref,brake}} = 0.10$.

---

## 6 Wheel loads and body: model limits

### 6.1 Quasi-Static Normal Load Distribution
Vertical wheel normal forces $F_{z,i}$ are computed from static axle weight split combined with quasi-static longitudinal and lateral load transfer (`normal_loads` in [`src/torquevector/physics/four_wheel.py`](../../src/torquevector/physics/four_wheel.py)):

$$F_{z,\text{FL}} = \frac{m g\,b}{2 L} - \Delta F_{z,x} - \Delta F_{z,y,\text{F}}, \qquad F_{z,\text{FR}} = \frac{m g\,b}{2 L} - \Delta F_{z,x} + \Delta F_{z,y,\text{F}},$$
$$F_{z,\text{RL}} = \frac{m g\,a}{2 L} + \Delta F_{z,x} - \Delta F_{z,y,\text{R}}, \qquad F_{z,\text{RR}} = \frac{m g\,a}{2 L} + \Delta F_{z,x} + \Delta F_{z,y,\text{R}},$$

with longitudinal and lateral load transfers:
$$\Delta F_{z,x} = \frac{m\,a_x\,h_{\text{CG}}}{2 L}, \qquad \Delta F_{z,y,\text{F}} = \frac{m\,a_y\,h_{\text{CG}}}{t}\cdot\frac{b}{L}, \qquad \Delta F_{z,y,\text{R}} = \frac{m\,a_y\,h_{\text{CG}}}{t}\cdot\frac{a}{L}.$$

Notice that the invariant $\sum_{i \in \{\text{FL,FR,RL,RR}\}} F_{z,i} = m g$ holds identically for all $(a_x, a_y) \in \mathbb{R}^2$.

### 6.2 Implicit Algebraic Loop Resolution (`solve_acceleration_Loads`)
Because the tyre forces $\mathbf{F}_{w,i}(F_{z,i})$ depend nonlinearly on $F_{z,i}$, which in turn depend on the body translational accelerations $(a_x, a_y) = (F_{x,\text{net}}/m,\; F_{y,\text{net}}/m)$, evaluating $F_{z,i}$ from the previous time step would introduce a non-physical numerical delay. Instead, at every Runge–Kutta stage, the simulator solves the coupled two-dimensional algebraic root problem $\mathbf{g}(a_x, a_y) = \mathbf{0}$:
$$\mathbf{g}(a_x, a_y) = \begin{bmatrix} a_x - \frac{1}{m}\,F_{x,\text{net}}\!\left(\mathbf{F}_z(a_x, a_y)\right) \\ a_y - \frac{1}{m}\,F_{y,\text{net}}\!\left(\mathbf{F}_z(a_x, a_y)\right) \end{bmatrix} = \mathbf{0},$$
using Powell's hybrid method (`scipy.optimize.root(..., method='hybr', tol=1e-11)`) with a damped fixed-point fallback (`max_iter = 60`, `tol = 1e-9`).

### 6.3 Structural Chassis Approximations and Model Limits
- **Roll stiffness distribution**: Separate front/rear suspension spring and anti-roll bar stiffnesses ($k_{\phi,\text{F}}, k_{\phi,\text{R}}$) and roll-centre heights are **not modelled**; lateral load transfer is partitioned purely in proportion to the static longitudinal weight distribution ($b/L$ front, $a/L$ rear).
- **Chassis pitch, roll, and heave dynamics**: Transient suspension heave, **pitch**, and **roll** degrees of freedom, damper rates, and unsprung-mass hop dynamics are **not modelled**.
- **Wheel lift fail-fast boundary**: Unilateral contact separation ($F_{z,i} = 0$ with three-wheel support) is **not modelled**. **Wheel lift** is treated as a strict **fail-fast** validity boundary (`validate_no_lift` raises `OutsideEnvelopeError` whenever $\min_i F_{z,i} \le 0$). For pure cornering ($a_x = 0$), wheel lift occurs at the critical lateral acceleration:
  $$|a_{y,\text{lift}}| = \frac{g\,t}{2\,h_{\text{CG}}} = \frac{9.81 \times 1.62}{2 \times 0.52} \approx 15.28\text{ m/s}^2 \quad (\approx 1.56\,g).$$

---

## 7 Combined slip and friction limit

The tyre force model (`combined_magic_formula` in [`src/torquevector/physics/combined_slip.py`](../../src/torquevector/physics/combined_slip.py) and [`src/torquevector/physics/tyre_magic_formula.py`](../../src/torquevector/physics/tyre_magic_formula.py)) couples pure-slip Pacejka Magic Formula curves with an opposing-slip damping envelope and radial Kamm-circle projection:

1. **Load-Sensitive Peak Grip $D_i(F_{z,i}, \mu_{\text{road},i})$**:
   $$\mu_{\text{eff}}(F_{z,i}) = \mu_{\text{road},i}\left(\frac{\max(F_{z,i}, 1\text{ N})}{F_{z,0}}\right)^{k_{\text{load}}}, \qquad D_i = \mu_{\text{eff}}(F_{z,i})\,F_{z,i},$$
   where $k_{\text{load}} = -0.0716 < 0$ captures degressive tyre load sensitivity ($\partial \mu_{\text{eff}}/\partial F_z < 0$).

2. **Pure-Slip Pacejka Magic Formula**:
   $$\operatorname{MF}(s; D_i, B, C, E) = D_i \sin\!\Big(C \arctan\!\big(B s - E\,(B s - \arctan(B s))\big)\Big),$$
   $$F_{x,0}(\kappa_i, F_{z,i}) = \operatorname{MF}(\kappa_i; D_i, B_x, C_x, E_x), \qquad F_{y,0}(\alpha_i, F_{z,i}) = -\operatorname{MF}(\alpha_i; D_i, B_y, C_y, E_y).$$

3. **Opposing-Slip Interaction Damping**:
   $$\tilde{F}_{x,i} = \frac{F_{x,0}(\kappa_i, F_{z,i})}{\sqrt{1 + (\alpha_i / \alpha_s)^2}}, \qquad \tilde{F}_{y,i} = \frac{F_{y,0}(\alpha_i, F_{z,i})}{\sqrt{1 + (\kappa_i / \kappa_s)^2}}.$$
   When $\alpha_i = 0$, $\tilde{F}_{x,i} = F_{x,0}$ and $\tilde{F}_{y,i} = 0$; when $\kappa_i = 0$, $\tilde{F}_{y,i} = F_{y,0}$ and $\tilde{F}_{x,i} = 0$, reproducing the exact pure-slip curves on both axes (`Q03.3`).

4. **Friction-Ellipse / Kamm-Circle Saturation and Utilization**:
   Defining the normalized utilization radius
   $$u_{\text{raw},i} = \sqrt{\left(\frac{c_x\,\tilde{F}_{x,i}}{D_i}\right)^2 + \left(\frac{c_y\,\tilde{F}_{y,i}}{D_i}\right)^2}, \qquad \eta_i = \min(u_{\text{raw},i},\; 1.0),$$
   if $u_{\text{raw},i} > 1$, the force vector is radially projected onto the friction boundary:
   $$\begin{bmatrix} F_{x,w,i} \\ F_{y,w,i} \end{bmatrix} = \begin{cases} \begin{bmatrix} \tilde{F}_{x,i} \\ \tilde{F}_{y,i} \end{bmatrix}, & u_{\text{raw},i} \le 1, \\[8pt] \dfrac{1}{u_{\text{raw},i}}\begin{bmatrix} \tilde{F}_{x,i} \\ \tilde{F}_{y,i} \end{bmatrix}, & u_{\text{raw},i} > 1. \end{cases}$$
   For the default isotropic weights $c_x = c_y = 1$, this guarantees $\sqrt{F_{x,w,i}^2 + F_{y,w,i}^2} \le D_i = \mu_{\text{eff}}(F_{z,i})\,F_{z,i}$ across the entire domain (`Q03.4`).

> [!NOTE]
> **Critical Comparison with Standard Pacejka MF 5.2 / 6.12**: Standard empirical tyre models employ cosine-based weighting functions $G_{x\alpha}(\alpha, \kappa, F_z) = \cos\!\big(C_{x\alpha}\arctan(B_{x\alpha}\alpha - E_{x\alpha}(B_{x\alpha}\alpha - \arctan(B_{x\alpha}\alpha)))\big)$ with load-dependent shift terms $S_{Vx\alpha}, S_{Vy\kappa}$ and transient relaxation lengths $\sigma_\kappa, \sigma_\alpha$. The opposing-slip Lorentzian damping $\big(1 + (s/s_0)^2\big)^{-1/2}$ implemented here is a smooth, symmetric, 2-parameter surrogate that avoids non-physical sign reversals (`Q03.5`), but omits ply-steer/conicity asymmetries and carcass relaxation lag.

---

## 8 Complete 11-state differential-algebraic system

In full sampled-data closed-loop simulation (`simulate_sampled` in [`src/torquevector/simulation/sampled.py`](../../src/torquevector/simulation/sampled.py)), the continuous-time plant state vector $\mathbf{x}(t) \in \mathbb{R}^{11}$ is

$$\mathbf{x}(t) = \begin{bmatrix} X & Y & \psi & v_x & v_y & r & \omega_{\text{FL}} & \omega_{\text{FR}} & \omega_{\text{RL}} & \omega_{\text{RR}} & \delta_{\text{act}} \end{bmatrix}^\top,$$

governed by the coupled nonlinear ODE-algebraic system:

$$\begin{aligned}
\dot{X} &= v_x \cos\psi - v_y \sin\psi, \\
\dot{Y} &= v_x \sin\psi + v_y \cos\psi, \\
\dot{\psi} &= r, \\
m(\dot{v}_x - r\,v_y) &= \sum_{i \in \mathcal{W}} \left(F_{x,w,i}\cos\delta_i - F_{y,w,i}\sin\delta_i\right) - \frac{1}{2}\rho_{\text{air}}(C_d A)\,v_x|v_x| - C_{\text{rr}}\,m g\,\tanh(3 v_x), \\
m(\dot{v}_y + r\,v_x) &= \sum_{i \in \mathcal{W}} \left(F_{x,w,i}\sin\delta_i + F_{y,w,i}\cos\delta_i\right), \\
I_z\,\dot{r} &= \sum_{i \in \mathcal{W}} \left(x_i F_{y,\text{body},i} - y_i F_{x,\text{body},i}\right), \\
J_w\,\dot{\omega}_i &= T_{\text{drive},i} - T_{\text{brake},i}\,\tanh(2\,\omega_i) - R_{\text{eff}}\,F_{x,w,i}, \qquad i \in \mathcal{W} = \{\text{FL}, \text{FR}, \text{RL}, \text{RR}\}, \\
\dot{\delta}_{\text{act}} &= \operatorname{clip}\!\left(\frac{\operatorname{clip}(\delta_{\text{cmd}}, -\delta_{\max}, \delta_{\max}) - \delta_{\text{act}}}{\tau_\delta},\; -\dot{\delta}_{\max},\; +\dot{\delta}_{\max}\right),
\end{aligned}$$

where $J_w = 1.45\text{ kg}\cdot\text{m}^2$, $R_{\text{eff}} = 0.315\text{ m}$, $\tau_\delta = 0.045\text{ s}$, $\delta_{\max} = 28^\circ$, and $\dot{\delta}_{\max} = 260^\circ/\text{s}$. The system is integrated between discrete controller/slip-control ticks using the explicit Runge–Kutta Dormand–Prince 5(4) method (`RK45`, `rtol = 1e-6`, `atol = 1e-8`, `max_step = 0.01` s) with exact zero-crossing event splitting at spatial road-friction boundaries.
