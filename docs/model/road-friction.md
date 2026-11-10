# Spatial Road-Friction Field (`RoadFrictionModel`) and Multi-$\mu$ Integration

This document specifies the mathematical formulation of position-dependent road friction $\mu_{\text{road}}(X, Y)$, $C^2$-continuous transition blending, ODE discontinuity event handling, and controller friction perception implemented in [`src/torquevector/physics/road_friction.py`](../../src/torquevector/physics/road_friction.py) and [`src/torquevector/simulation/friction_perception.py`](../../src/torquevector/simulation/friction_perception.py).

---

## 1 Spatial Friction Field $\mu_{\text{road}}(X, Y)$

Rather than applying a uniform scalar friction coefficient at the vehicle centre of gravity, the simulator evaluates the local road friction $\mu_{\text{road},i} = \mu_{\text{road}}(X_i, Y_i)$ independently at the global coordinates $(X_i, Y_i)$ of each wheel contact patch $i \in \{\text{FL}, \text{FR}, \text{RL}, \text{RR}\}$:

$$\begin{bmatrix} X_i \\ Y_i \end{bmatrix} = \begin{bmatrix} X \\ Y \end{bmatrix} + \begin{bmatrix} \cos\psi & -\sin\psi \\ \sin\psi & \cos\psi \end{bmatrix} \begin{bmatrix} x_i \\ y_i \end{bmatrix}.$$

This contact-patch resolution ensures that during a longitudinal $\mu$-step (`mu_step`), the front axle enters the low-friction zone $\Delta t = L / v_x$ seconds before the rear axle, and during a lateral split-$\mu$ (`split_mu`), the left wheels $(\text{FL}, \text{RL})$ and right wheels $(\text{FR}, \text{RR})$ simultaneously experience distinct friction ceilings.

### 1.1 Supported Spatial Primitives (`RoadFrictionConfig`)

Five geometric field primitives are supported in [`RoadFrictionModel.mu_at_xy()`](../../src/torquevector/physics/road_friction.py):

1. **`uniform`**: Constant scalar field $\mu_{\text{road}}(X, Y) \equiv \mu_0$. When `scaling_mode: scale_tyre_mu` with $\mu_0 = 1.0$, the evaluation is bit-identical to the unmapped baseline model.
2. **`mu_step`**: Longitudinal step transition along $X$ at $X = X_{\text{step}}$ from $\mu_{\text{before}}$ to $\mu_{\text{after}}$ with transition width $w_t \ge 0$:
   $$\mu_{\text{road}}(X, Y) = \mu_{\text{before}} + (\mu_{\text{after}} - \mu_{\text{before}})\,S\!\left(\frac{X - X_{\text{step}}}{w_t}\right).$$
3. **`split_mu`**: Lateral split transition along $Y$ at $Y = Y_{\text{split}}$ between $\mu_{\text{right}}$ ($Y < Y_{\text{split}}$) and $\mu_{\text{left}}$ ($Y \ge Y_{\text{split}}$):
   $$\mu_{\text{road}}(X, Y) = \mu_{\text{right}} + (\mu_{\text{left}} - \mu_{\text{right}})\,S\!\left(\frac{Y - Y_{\text{split}}}{w_t}\right).$$
4. **`patch`**: Rectangular region $[X_{\min}, X_{\max}] \times [Y_{\min}, Y_{\max}]$ with interior friction $\mu_{\text{patch}}$ blended into background $\mu_0$.
5. **`zones`**: Ordered sequence of rectangular/half-plane zones where each matching zone overrides the underlying field according to its local $C^2$ boundary blending weight.

### 1.2 $C^2$-Continuous Quintic Smoothstep Blending
When a positive transition width $w_t > 0$ (`transition_width_m`) is specified, boundary transitions are smoothed using the quintic Hermite polynomial (`_quintic` in [`src/torquevector/physics/road_friction.py`](../../src/torquevector/physics/road_friction.py)):

$$S(\xi) = \begin{cases} 0, & \xi \le 0, \\ 6\xi^5 - 15\xi^4 + 10\xi^3, & 0 < \xi < 1, \\ 1, & \xi \ge 1, \end{cases} \qquad \xi = \frac{d + w_t/2}{w_t},$$

which satisfies $S'(0) = S'(1) = S''(0) = S''(1) = 0$, guaranteeing a $C^2$-continuous vector field for adaptive Runge–Kutta integration.

### 1.3 Sharp Discontinuities ($w_t = 0$) and ODE Sub-Step Splitting
When $w_t = 0$, $\mu_{\text{road}}(X, Y)$ is discontinuous across zone boundaries $X = X_b$ or $Y = Y_b$. To prevent adaptive RK45 steps from straddling a discontinuity and corrupting local truncation error estimates, `boundary_events()` in [`src/torquevector/physics/road_friction.py`](../../src/torquevector/physics/road_friction.py) registers terminal zero-crossing event functions

$$g_{i,b}(\mathbf{x}) = X_i(\mathbf{x}) - X_b = 0 \quad \text{or} \quad Y_i(\mathbf{x}) - Y_b = 0$$

with `scipy.integrate.solve_ivp`. Whenever any wheel contact patch crosses a sharp boundary inside a sample-and-hold interval $[t_k, t_{k+1}]$, the ODE integrator terminates precisely at the crossing time $t^* \in (t_k, t_{k+1})$ and restarts from $t^* + \varepsilon$ on the new surface.

---

## 2 Coupling to the Pacejka Magic Formula (`tyre_for_wheel`)

For each wheel $i$, the local surface friction $\mu_{\text{road},i}$ maps to an effective wheel tyre parameter set (`tyre_for_wheel` in [`src/torquevector/physics/road_friction.py`](../../src/torquevector/physics/road_friction.py)) via Pacejka similarity scaling:

$$\lambda_{\mu,i} = \begin{cases} \dfrac{\mu_{\text{road},i}}{\mu_{\text{ref}}}, & \text{if }\texttt{scaling\_mode = replace\_tyre\_mu}, \\[8pt] \mu_{\text{road},i}, & \text{if }\texttt{scaling\_mode = scale\_tyre\_mu}. \end{cases}$$

Peak grip scales linearly with $\lambda_{\mu,i}$ ($D_i \propto \lambda_{\mu,i}$), while the slip stiffness factor $B_i$ is scaled inversely ($B_i \leftarrow B_i / \lambda_{\mu,i}$ when `keep_initial_stiffness: true`) so that the low-slip cornering and longitudinal slip stiffness $K_{\text{slip}} = B_i C_i D_i$ remains invariant to surface friction (reflecting physical rubber carcass elasticity at vanishing slip, whereas peak force $D_i$ is governed by adhesive/hysteretic road friction).

---

## 3 Friction Perception Modes (`FrictionPerception`)

To maintain strict causal separation between the physical plant and the control system (`Q11.5`), controllers and torque allocators never query `RoadFrictionModel` directly unless configured to do so. Instead, [`FrictionPerception`](../../src/torquevector/simulation/friction_perception.py) provides an estimated wheel friction vector $\boldsymbol{\mu}_{\text{est}}[k] = (\mu_{\text{est,FL}}, \mu_{\text{est,FR}}, \mu_{\text{est,RL}}, \mu_{\text{est,RR}})$ according to `road.perception.estimator`:

| Estimator Mode | Mathematical Law | Engineering Purpose |
|---|---|---|
| **`ground_truth`** | $\mu_{\text{est},i}(t) = \mu_{\text{road}}(X_i(t), Y_i(t))$ | Idealized upper-bound benchmark with instantaneous per-wheel $\mu$ knowledge |
| **`constant_nominal`** | $\mu_{\text{est},i}(t) \equiv \mu_{\text{nom}}$ (default $1.0$) | Evaluates controller/allocator degradation when entering wet/icy patches without friction adaptation |
| **`delayed_filtered`** | $\tau_\mu \dot{\mu}_{\text{est},i} + \mu_{\text{est},i} = \operatorname{clip}(\mu_{\text{road},i}(t - T_d) + b_\mu,\; \mu_{\min},\; \mu_{\max})$ | Emulates realistic onboard $\mu$-estimation dynamics with transport delay $T_d$, filter lag $\tau_\mu$, and bias $b_\mu$ |
| **`corrupted`** | $\mu_{\text{est},i}(t) = \operatorname{clip}(\gamma_\mu\,\mu_{\text{road},i}(t) + b_\mu,\; \mu_{\min},\; \mu_{\max})$ | Sensitivity analysis under systematic multiplicative ($\gamma_\mu$) and additive ($b_\mu$) estimation errors |
| **`map_preview`** | $\mu_{\text{est},i}(t) = \mu_{\text{road},i}(t)$ + spatial horizon preview $\mu_{\text{road}}\big(X(t+\tau), Y(t+\tau)\big)$ | Supplies future road-friction trajectory along the MPC prediction horizon $j = 0, \dots, N-1$ |

---

## 4 Canonical Multi-$\mu$ Benchmark Scenarios (`torquevector multi-mu`)

Four canonical heterogeneous-friction scenarios are defined under [`configs/scenarios/`](../../configs/scenarios/) and executed via `torquevector multi-mu`:

| Scenario ID | Surface Layout | Speed $v_{x,0}$ | Primary Physical Phenomenon |
|---|---|---:|---|
| **`straight_multi_mu`** | Longitudinal zones: dry ($\mu=0.95$) $\to$ wet ($\mu=0.40$) $\to$ ice ($\mu=0.18$) $\to$ dry ($\mu=0.95$) | $18.0\text{ m/s}$ | Symmetric axle transition across sharp friction steps without spurious yaw |
| **`straight_split_mu_acceleration`** | Left $\mu_L = 0.30$ (wet/snow), Right $\mu_R = 0.95$ (dry) under drive demand | $14.0\text{ m/s}$ | Disturbance yaw moment $M_{z,\text{dist}} < 0$ toward low-$\mu$ side; friction-aware allocation + ASR |
| **`straight_split_mu_braking`** | Left $\mu_L = 0.30$, Right $\mu_R = 0.95$ under braking demand | $20.0\text{ m/s}$ | Asymmetric braking yaw disturbance toward high-$\mu$ side; counter-steering & ABS stability |
| **`mixed_mu_lane_change`** | Double lane change across alternating dry ($\mu=0.95$) and low-$\mu$ ($\mu=0.35$) patches | $16.0\text{ m/s}$ | Combined lateral-longitudinal saturation; demonstrates benefit of `map_preview` for LTV-MPC/NMPC |
