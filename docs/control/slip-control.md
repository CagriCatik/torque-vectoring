# Inner-Loop Wheel Slip Control (ABS / ASR) Specification

This document specifies the multi-rate architecture, discrete-time velocity-form PI control law, anti-windup and slew-rate bounds, and empirical performance on split-$\mu$ surfaces of the wheel slip controller implemented in [`src/torquevector/control/slip_control.py`](../../src/torquevector/control/slip_control.py).

---

## 1 Multi-Rate Sample-and-Hold Architecture

Rotational wheel dynamics ($J_w = 1.45\text{ kg}\cdot\text{m}^2$, effective radius $R_{\text{eff}} = 0.315\text{ m}$) exhibit an open-loop mechanical time constant

$$\tau_\omega \approx \frac{J_w\,u_w}{R_{\text{eff}}^2\,C_\kappa} \approx 5\text{--}15\text{ ms},$$

which is significantly faster than the chassis yaw/sideslip dynamics ($\tau_{\text{yaw}} \approx 150\text{--}300\text{ ms}$). Operating wheel slip control at the outer yaw-moment sampling period $T_{\text{outer}} = 40\text{ ms}$ ($25\text{ Hz}$) would allow a wheel on ice or wet asphalt to exceed the validated slip envelope $|\kappa_i| \le 0.5$ within a single outer step.

To resolve this time-scale separation, [`simulate_sampled()`](../../src/torquevector/simulation/sampled.py) implements a **two-level hierarchical zero-order-hold (ZOH) loop**:

1. **Outer Loop ($T_{\text{outer}} = 0.040\text{ s}$, $25\text{ Hz}$)**:
   - Observes vehicle state $(v_x, v_y, r)$ and estimated friction $\boldsymbol{\mu}_{\text{est}}$.
   - Evaluates the yaw-moment controller (`OFF`, `P`, `LQR`, `LTV-MPC`, `NMPC`) and BVLS torque allocator to produce baseline outer wheel torques $\mathbf{T}_{\text{drive,outer}}[k]$ and $\mathbf{T}_{\text{brake,outer}}[k]$.
2. **Inner Loop ($T_{\text{inner}} = 0.005\text{ s}$, $200\text{ Hz}$, $M = T_{\text{outer}}/T_{\text{inner}} = 8$ sub-steps)**:
   - At each sub-step $m \in \{0, \dots, 7\}$, evaluates wheel longitudinal slip $\kappa_i(t_{k,m})$ from $\omega_i(t_{k,m})$ and local wheel-carrier longitudinal speed $u_{w,i}(t_{k,m})$.
   - Updates per-wheel torque reduction factors $r_i[k, m] \in [0, r_{\max}]$ via `WheelSlipController.step()` and applies held effective torques
     $$T_{\text{drive,eff},i} = (1 - r_{\text{drive},i})\,T_{\text{drive,outer},i}, \qquad T_{\text{brake,eff},i} = (1 - r_{\text{brake},i})\,T_{\text{brake,outer},i}$$
     to the RK45 plant integration over $[t_{k,m},\; t_{k,m} + T_{\text{inner}}]$.

---

## 2 Discrete-Time Velocity-Form PI Control Law

For each wheel $i \in \{\text{FL}, \text{FR}, \text{RL}, \text{RR}\}$, let the unsigned excess slip error $e_{\kappa,i}[n]$ at inner tick $n$ be defined separately for traction (ASR, when $T_{\text{drive},i} > 0$) and braking (ABS, when $T_{\text{brake},i} > 0$):

$$e_{\kappa,i}[n] = \begin{cases} \kappa_i[n] - \kappa_{\text{ref,drive}}, & \text{if } T_{\text{drive},i} > 0 \quad (\text{ASR}), \\[6pt] -\kappa_i[n] - \kappa_{\text{ref,brake}}, & \text{if } T_{\text{brake},i} > 0 \quad (\text{ABS}). \end{cases}$$

### 2.1 Hysteresis Activation Deadband and Engagement Threshold
To prevent chattering around the threshold, intervention activates when $|\kappa_i|$ exceeds $\kappa_{\text{ref}} + \delta_{\text{db}}$ and releases when $|\kappa_i| < \kappa_{\text{ref}} - \delta_{\text{db}}$ ($\delta_{\text{db}} = 0.008$). Below the minimum contact speed $u_{w,i} < v_{\text{min,slip}} = 3.2\text{ m/s}$, slip control is bypassed ($r_i = 0$).

### 2.2 Velocity-Form (Incremental) PI Update with Slew-Rate and Box Clamping
When active, the raw incremental torque reduction change $\Delta r_{\text{raw},i}[n]$ is computed in velocity form (`WheelSlipController.step` in [`src/torquevector/control/slip_control.py`](../../src/torquevector/control/slip_control.py)):

$$\Delta r_{\text{raw},i}[n] = k_p\big(e_{\kappa,i}[n] - e_{\kappa,i}[n-1]\big) + k_i\,T_{\text{inner}}\,e_{\kappa,i}[n],$$

and rate-limited by asymmetric build/release slew bounds before clamping to $[0,\; r_{\max}]$:

$$\Delta r_i[n] = \operatorname{clip}\!\big(\Delta r_{\text{raw},i}[n],\; -\dot{r}_{\text{release}}\,T_{\text{inner}},\; +\dot{r}_{\text{build}}\,T_{\text{inner}}\big),$$
$$r_i[n] = \operatorname{clip}\!\big(r_i[n-1] + \Delta r_i[n],\; 0.0,\; r_{\max}\big).$$

When the wheel re-enters the stable low-slip zone ($e_{\kappa,i}[n] < -\delta_{\text{db}}$), the reduction factor decays smoothly toward zero at the release rate $\dot{r}_{\text{release}} = 4.0\text{ s}^{-1}$.

### 2.3 Default Parameterization (`configs/control/slip_control.yaml`)

| Parameter | Symbol | Default Value | Unit | Physical Role |
|---|---:|---:|---|---|
| Inner loop sampling period | $T_{\text{inner}}$ | $0.005$ | $\text{s}$ | $200\text{ Hz}$ wheel-slip sampling rate |
| Traction slip target (ASR) | $\kappa_{\text{ref,drive}}$ | $0.12$ | $1$ | Target longitudinal slip ratio under drive torque |
| Braking slip target (ABS) | $\kappa_{\text{ref,brake}}$ | $0.10$ | $1$ | Target longitudinal slip magnitude under braking |
| Hysteresis deadband | $\delta_{\text{db}}$ | $0.008$ | $1$ | Activation/deactivation hysteresis half-width |
| Proportional gain | $k_p$ | $4.5$ | $1$ | Immediate damping of rapid wheel spin/lock acceleration $\dot{\kappa}_i$ |
| Integral gain | $k_i$ | $55.0$ | $\text{s}^{-1}$ | Steady-state elimination of excess slip on sustained low-$\mu$ patches |
| Maximum torque reduction | $r_{\max}$ | $0.95$ | $1$ | Retains $\ge 5\,\%$ residual torque capacity |
| Maximum build rate | $\dot{r}_{\text{build}}$ | $18.0$ | $\text{s}^{-1}$ | Fast torque cut ($0 \to 90\,\%$ in $50\text{ ms}$) upon sudden ice/wet entry |
| Maximum release rate | $\dot{r}_{\text{release}}$ | $4.0$ | $\text{s}^{-1}$ | Gradual torque restoration ($250\text{ ms}$ full recovery) to avoid re-triggering |

---

## 3 Empirical Split-$\mu$ Braking and Acceleration Verification

The scenarios `straight_split_mu_braking_above_limit` ($T_{\text{brake},i} = 520\text{ Nm}$ per wheel on $\mu_L = 0.25$ / $\mu_R = 0.95$) and `straight_split_mu_acceleration_above_limit` ($T_{\text{drive},i} = 460\text{ Nm}$ per wheel on $\mu_L = 0.25$ / $\mu_R = 0.95$) verify the fail-closed vs. slip-controlled plant response (`tests/integration/road/test_slip_control_scenarios.py`):

| Scenario | `slip_control.enabled: false` | `slip_control.enabled: true` (ABS / ASR Active) |
|---|---|---|
| **Split-$\mu$ Braking Above Limit** ($\mu_L = 0.25, \mu_R = 0.95$) | Left wheels lock rapidly; $\|\kappa_{\text{FL}}\| > 0.50$ within $0.08\text{ s}$ $\implies$ **`FAIL_CLOSED` (`OutsideEnvelopeError`)** | ABS reduces left brake torque by $r_{\text{FL}} \approx 0.62$; peak $\|\kappa_{\text{FL}}\| \le 0.225$, steady-state $\|\kappa_{\text{FL}}\| \to 0.100$ (**`PASS`**) |
| **Split-$\mu$ Acceleration Above Limit** ($\mu_L = 0.25, \mu_R = 0.95$) | Left wheels spin up; $\kappa_{\text{FL}} > 0.50$ within $0.07\text{ s}$ $\implies$ **`FAIL_CLOSED` (`OutsideEnvelopeError`)** | ASR cuts left drive torque by $r_{\text{FL}} \approx 0.58$; peak $\kappa_{\text{FL}} \le 0.198$, steady-state $\kappa_{\text{FL}} \to 0.120$ (**`PASS`**) |

---

## 4 Critical Structural Limitations

1. **Uncoordinated Outer Allocator and Inner Slip Limiter**: When the inner ASR loop reduces drive torque on a low-$\mu$ wheel ($T_{\text{eff},i} = (1 - r_i) T_{\text{outer},i}$), the outer 40 ms BVLS torque allocator is not notified of the inner torque clip unless $\mu_{\text{est},i}$ also reflects the low friction (`friction_aware` allocation with `ground_truth` or converged `delayed_filtered` estimator). Under `constant_nominal` friction perception ($\mu_{\text{est}} \equiv 1.0$), unilateral ASR torque reduction on the left wheels alters the realized net yaw moment $\Delta M_z$ relative to $M_{z,\text{cmd}}$.
2. **Initial Entry Slip Overshoot**: Because the velocity-form PI controller is purely reactive (triggered only after $|\kappa_i| > \kappa_{\text{ref}} + \delta_{\text{db}}$) and bounded by $\dot{r}_{\text{build}} = 18.0\text{ s}^{-1}$, a sudden step torque on $\mu = 0.25$ exhibits a transient $30\text{--}60\text{ ms}$ overshoot to $|\kappa|_{\max} \approx 0.20\text{--}0.23$ before settling onto $\kappa_{\text{ref}}$.
