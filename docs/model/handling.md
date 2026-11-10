# Self-Steering Characteristic, Linear Single-Track Theory, and Handling Limits

This document presents the analytical derivation and numerical evaluation methodology for steady-state and quasi-steady lateral vehicle dynamics, self-steering classification (understeer, neutral steer, oversteer), $(\beta, r)$ phase-plane stability analysis, and combined-slip $g$-$g$ acceleration envelopes implemented in [`src/torquevector/evaluation/handling.py`](../../src/torquevector/evaluation/handling.py) and [`src/torquevector/physics/axle_tyre.py`](../../src/torquevector/physics/axle_tyre.py).

---

## 1 Linear Single-Track (Bicycle) Theory and Understeer Gradient

In the linear operating regime ($|a_y| \lesssim 0.3\,g$, small slip angles $|\alpha_{f,r}| \ll 1$, constant longitudinal speed $v_x$), the four-wheel vehicle reduces to the classical two-degree-of-freedom single-track model with static axle normal loads

$$F_{z,f0} = m g\,\frac{b}{L}, \qquad F_{z,r0} = m g\,\frac{a}{L}, \qquad L = a + b.$$

### 1.1 Axle Cornering Stiffnesses from Pacejka Magic Formula Parameters
At zero longitudinal slip ($\kappa = 0$), the pure lateral force of a single tyre at static wheel load $F_{z,i0} = \frac{1}{2}F_{z,\text{axle},0}$ is $F_{y,0}(\alpha) = -D_i \sin\!\big(C_y \arctan(B_y \alpha - E_y(B_y \alpha - \arctan(B_y \alpha)))\big)$. Differentiating with respect to $\alpha$ at $\alpha = 0$ yields the exact analytical single-tyre cornering stiffness $c_{\alpha,i} = -\left.\frac{\partial F_{y,0}}{\partial \alpha}\right|_{\alpha=0} = B_y C_y D_i$. Summing across the two wheels of each axle (`axle_cornering_stiffnesses` in [`src/torquevector/evaluation/handling.py`](../../src/torquevector/evaluation/handling.py)) with optional axle stiffness scaling $\sigma_{\alpha,\text{axle}}$ (`tyre.axle_scaling.{front,rear}.lateral_stiffness_scale`) gives:

$$C_{\alpha f} = 2\,\sigma_{\alpha,f}\,B_y\,C_y\,D\!\left(\frac{F_{z,f0}}{2}\right), \qquad C_{\alpha r} = 2\,\sigma_{\alpha,r}\,B_y\,C_y\,D\!\left(\frac{F_{z,r0}}{2}\right) \qquad \left[\text{N/rad}\right].$$

### 1.2 Analytical Self-Steering (Understeer) Gradient $K$
In steady-state cornering ($\dot{v}_y = 0$, $\dot{r} = 0$, lateral acceleration $a_y = v_x r$) without active yaw moment ($M_z = 0$), longitudinal moment balance about the rear and front axles requires

$$F_{y,f} = m\,a_y\,\frac{b}{L} = C_{\alpha f}(-\alpha_f), \qquad F_{y,r} = m\,a_y\,\frac{a}{L} = C_{\alpha r}(-\alpha_r).$$

Combining the kinematic relation $\delta - \frac{L}{R} = -\alpha_f + \alpha_r$ (where $1/R = r/v_x = a_y / v_x^2$ is the path curvature and $\delta_{\text{ack}} = L/R = L a_y / v_x^2$ is the kinematic Ackermann steer angle) yields the fundamental steady-state handling equation:

$$\delta_{\text{req}}(a_y, v_x) = \frac{L}{v_x^2}\,a_y + K\,a_y, \qquad K = \frac{m}{L}\left(\frac{b}{C_{\alpha f}} - \frac{a}{C_{\alpha r}}\right) \quad \left[\frac{\text{rad}}{\text{m/s}^2}\right].$$

Expressed in engineering units of degrees of front road-wheel steer per $g$ of lateral acceleration (`understeer_gradient_deg_per_g`):

$$K_{\text{deg}/g} = \frac{180}{\pi}\,g\,K = \frac{180}{\pi}\,m g\left(\frac{b}{L\,C_{\alpha f}} - \frac{a}{L\,C_{\alpha r}}\right) \quad \left[\text{deg}/g\right].$$

### 1.3 Characteristic Speed, Critical Speed, and Steady-State Yaw Gain
Substituting $a_y = v_x r$ into $\delta = \frac{L}{v_x} r + K v_x r$ gives the linear steady-state yaw-rate transfer gain (`yaw_gain_per_s`):

$$\left.\frac{r}{\delta}\right|_{\text{ss}}(v_x) = \frac{v_x}{L + K\,v_x^2} \quad \left[\text{s}^{-1}\right].$$

Three canonical regimes emerge according to the sign of $K$:

| Classification | Condition | Characteristic / Critical Speed | Physical Meaning |
|---|---|---|---|
| **Understeer** | $K > +0.05\text{ deg}/g$ | $v_{\text{char}} = \sqrt{\dfrac{L}{K}}$ | Unconditionally yaw-stable; at $v_x = v_{\text{char}}$, $\delta_{\text{req}} = 2\,\delta_{\text{ack}}$ and yaw gain $r/\delta$ reaches its global maximum $\frac{1}{2\sqrt{L K}}$ |
| **Neutral Steer** | $\|K\| \le 0.05\text{ deg}/g$ | $v_{\text{char}} \to \infty$ | Required steer angle equals the kinematic Ackermann angle $\delta_{\text{ack}} = L/R$ at all speeds; $r/\delta = v_x/L$ grows linearly with $v_x$ |
| **Oversteer** | $K < -0.05\text{ deg}/g$ | $v_{\text{crit}} = \sqrt{\dfrac{L}{-K}}$ | Required steer angle decreases with $a_y$; at $v_x \ge v_{\text{crit}}$, the denominator $L + K v_x^2 \le 0$ and the linear open-loop plant becomes exponentially unstable (eigenvalue crosses into $\operatorname{Re}(\lambda) > 0$) |

---

## 2 Configured Vehicle Handling Variants (`configs/profiles/handling_*.yaml`)

Because the nominal vehicle has a front-biased mass distribution ($a = 1.18\text{ m} < b = 1.58\text{ m}$, so $b/L = 0.5725 > a/L = 0.4275$) and degressive tyre load sensitivity ($k_{\text{load}} = -0.0716 < 0$), identical front and rear tyres naturally produce an understeering baseline. To study neutral/strong-understeer and unstable oversteer regimes without mutating the AST-frozen vehicle geometry, axle-specific cornering stiffness and peak friction multipliers (`AxleTyreScaling` in [`src/torquevector/physics/axle_tyre.py`](../../src/torquevector/physics/axle_tyre.py)) are applied:

$$B_{y,\text{axle}} = \frac{\sigma_{\alpha,\text{axle}}}{\sigma_{\mu,\text{axle}}}\,B_y, \qquad \mu_{\text{axle}} = \sigma_{\mu,\text{axle}}\,\mu,$$

which scales initial cornering stiffness $B_{y,\text{axle}} C_y D_{\text{axle}} = \sigma_{\alpha,\text{axle}}\,B_y C_y D$ and peak lateral force $D_{\text{axle}} = \sigma_{\mu,\text{axle}}\,D$ independently.

| Variant Profile | Front Scaling $(\sigma_{\alpha,f}, \sigma_{\mu,f})$ | Rear Scaling $(\sigma_{\alpha,r}, \sigma_{\mu,r})$ | Linear $K$ $[\text{deg}/g]$ | $v_{\text{char}}$ / $v_{\text{crit}}$ $[\text{m/s}]$ | Classification |
|---|:---:|:---:|---:|---:|---|
| **`reference`** ([`default.yaml`](../../configs/default.yaml)) | $(1.00,\; 1.00)$ | $(1.00,\; 1.00)$ | $+0.165\text{ deg}/g$ | $v_{\text{char}} = 96.9\text{ m/s}$ | Neutral / Mild Understeer |
| **`understeer`** ([`vehicle_understeer.yaml`](../../configs/profiles/vehicle_understeer.yaml)) | $(0.80,\; 0.95)$ | $(1.00,\; 1.00)$ | $+2.687\text{ deg}/g$ | $v_{\text{char}} = 24.0\text{ m/s}$ | Understeer |
| **`oversteer`** ([`vehicle_oversteer.yaml`](../../configs/profiles/vehicle_oversteer.yaml)) | $(1.00,\; 1.00)$ | $(0.85,\; 0.95)$ | $-1.699\text{ deg}/g$ | $v_{\text{crit}} = 30.2\text{ m/s}$ | Oversteer |

---

## 3 Quasi-Steady Ramp-Steer Evaluation (ISO 4138 Method)

To characterize nonlinear handling across the full lateral acceleration range up to tyre saturation, `run_ramp_steer()` simulates a slow quasi-steady steering ramp (`configs/scenarios/ramp_steer.yaml`, steer rate $\dot{\delta} \approx 0.67^\circ/\text{s}$ from $0^\circ$ to $12^\circ$ over $18\text{ s}$) at constant forward speeds $v_x \in \{15, 20, 25\}\text{ m/s}$:

1. **Handling Curve Extraction**: At each sample, the dynamic Ackermann angle $\delta_{\text{ack}}(t) = \frac{L\,r(t)}{v_x(t)}$ and excess self-steering angle $\Delta\delta(t) = \delta_{\text{act}}(t) - \delta_{\text{ack}}(t)$ are recorded against normalized lateral acceleration $a_y / g = v_x(t) r(t) / g$.
2. **Empirical Gradient Identification**: Linear regression of $\Delta\delta$ versus $a_y/g$ over the quasi-linear window $0.05\,g \le |a_y| \le 0.40\,g$ yields the simulated nonlinear understeer gradient $K_{\text{sim}}$ (reflecting longitudinal propulsion slip, parallel steering kinematics, and load-transfer nonlinearity).
3. **Limit-Behaviour Classification**: Near peak lateral acceleration $a_{y,\max}$, the trajectory is classified according to whether the front axle saturates first (`limit understeer (front axle saturates first)`) or rear-axle breakaway triggers sideslip divergence (`loss of stability (side slip exceeded)`).

| Variant | Controller @ Speed | Linear $K$ $[\text{deg}/g]$ | Simulated $K_{\text{sim}}$ ($0.05\text{--}0.4\,g$) | Peak $a_{y,\max}$ $[g]$ | Limit Behaviour | Phase-Plane Stability ($20\text{ m/s}$) |
|---|---|---:|---:|---:|---|:---:|
| **`reference`** | `OFF` @ $15 / 20 / 25\text{ m/s}$ | $+0.17$ | $+0.24 / +0.25 / +0.26\text{ deg}/g$ | $0.94 / 0.93 / 0.93\,g$ | `limit understeer` | $33/35$ stable |
| **`reference`** | `LQR` @ $15 / 20 / 25\text{ m/s}$ | $+0.17$ | $+0.19 / +0.23 / +0.25\text{ deg}/g$ | $0.89 / 0.89 / 0.88\,g$ | `limit understeer` | — |
| **`understeer`** | `OFF` @ $15 / 20 / 25\text{ m/s}$ | $+2.69$ | $+2.96 / +2.97 / +2.96\text{ deg}/g$ | $0.88 / 0.88 / 0.88\,g$ | `limit understeer` | $33/35$ stable |
| **`understeer`** | `LQR` @ $15 / 20 / 25\text{ m/s}$ | $+2.69$ | $+1.98 / +1.76 / +1.52\text{ deg}/g$ | $0.85 / 0.84 / 0.84\,g$ | `limit understeer` | — |
| **`oversteer`** | `OFF` @ $15 / 20 / 25\text{ m/s}$ | $-1.70$ | $-1.69 / -1.50 / -1.25\text{ deg}/g$ | $0.92 / 0.89 / 0.87\,g$ | **`loss of stability (side slip exceeded)`** | $22/35$ stable |
| **`oversteer`** | `LQR` @ $15 / 20 / 25\text{ m/s}$ | $-1.70$ | $-0.84 / -0.57 / -0.40\text{ deg}/g$ | $0.88 / 0.88 / 0.87\,g$ | **`limit understeer` (stabilized)** | — |

> [!TIP]
> **Physical Interpretation of Torque Vectoring on the Handling Diagram**: Because the yaw-rate reference in `driver_steering` mode is $r_{\text{ref}} = \frac{v_x \tan\delta_{\text{act}}}{L} \approx \frac{v_x}{L}\delta_{\text{act}}$ (which is the **neutral-steer** yaw rate corresponding to $K = 0$), an active yaw-moment controller (`LQR`) generates an outward yaw moment on the `understeer` vehicle (reducing $K_{\text{sim}}$ from $+2.96$ to $+1.52\text{ deg}/g$ at $25\text{ m/s}$) and a stabilizing counter-yaw moment on the `oversteer` vehicle (shifting $K_{\text{sim}}$ from $-1.69$ to $-0.84\text{ deg}/g$ at $15\text{ m/s}$ and $-0.40\text{ deg}/g$ at $25\text{ m/s}$, eliminating rear-axle breakaway and converting the limit response from `loss of stability` into safe `limit understeer`).

---

## 4 $(\beta, r)$ Phase-Plane Stability Analysis

`phase_plane_analysis()` evaluates autonomous lateral stability at $v_x = 20.0\text{ m/s}$ and fixed steering $\delta_{\text{act}} = 0^\circ$ across a $7 \times 5 = 35$ grid of initial perturbations $(\beta_0, r_0)$:
- Trajectories are classified as **`stable`** if they converge to the straight-running equilibrium and **`unstable`** if they diverge toward a spin or exceed the validated slip envelope (`OutsideEnvelopeError`).
- While the `reference` and `understeer` variants stabilize $33/35$ initial states, the `oversteer` variant exhibits saddle-point separatrix divergence across larger combined perturbations, leaving $22/35$ trajectories stable.

---

## 5 Quasi-Static $g$-$g$ Acceleration Envelope

`gg_diagram()` constructs the quasi-static combined longitudinal-lateral acceleration boundary $(a_x(\theta), a_y(\theta))$ across polar slip directions $\theta \in [-\pi, \pi]$ by solving for the maximum sustainable total tyre force vector under coupled quasi-static normal load transfer $\mathbf{F}_z(a_x, a_y)$ and the combined-slip Kamm circle at each wheel. The asymmetry between peak braking ($a_{x,\min}$) and peak all-wheel-drive traction ($a_{x,\max}$) reflects longitudinal load transfer onto the front axle under braking and degressive tyre load sensitivity ($k_{\text{load}} < 0$).
