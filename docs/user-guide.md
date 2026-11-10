# CLI User Guide and Reproducible Workflows

Both `torquevector` and `tvsim` invoke the unified command-line interface defined in [`src/torquevector/cli/main.py`](../src/torquevector/cli/main.py). All commands support `--quiet`, `--log-level {DEBUG,INFO,WARNING,ERROR}`, and `--log-dir <path>`.

---

## 1 Scenario Simulation and Controller Comparison (`simulate`, `compare`)

```bash
# Simulate a single controller on a chosen scenario
torquevector simulate --scenario circle_track --controller lqr

# Compare all five controllers on the Double Lane Change
torquevector compare --scenario double_lane_change --controllers off p lqr ltv nmpc

# Execute the 4-manoeuvre nominal benchmark matrix (circle, DLC, step steer, sine steer)
torquevector simulate --profile configs/profiles/nominal.yaml

# Override individual physical or controller parameters from the CLI
torquevector simulate --scenario step_steer --controller lqr \
    --set vehicle.mass_kg=1750.0 \
    --set controllers.lqr.limits.yaw_moment_limit_nm=1800.0
```

---

## 2 Heterogeneous Road Friction and ABS/ASR Evaluation (`multi-mu`)

Executes the four canonical spatial road-friction scenarios (`straight_multi_mu`, `straight_split_mu_acceleration`, `straight_split_mu_braking`, `mixed_mu_lane_change`) and exports per-scenario telemetry, PDF reports, road-map animations, and a consolidated `SUMMARY.md`:

```bash
# Run all four multi-mu scenarios with default controllers
torquevector multi-mu --output results/multi_mu

# Compare mixed-mu lane change with horizon friction preview enabled for LTV-MPC and NMPC
torquevector compare --scenario mixed_mu_lane_change \
    --set road.perception.estimator=map_preview \
    --set controllers.ltv_mpc.friction_knowledge.mode=map_preview \
    --set controllers.nmpc.friction_knowledge.mode=map_preview
```

---

## 3 Self-Steering & Handling Limit Analysis (`handling`)

Computes analytical linear single-track properties ($K$, $v_{\text{char}}$, $v_{\text{crit}}$), quasi-steady ISO 4138 ramp-steer handling diagrams, $(\beta, r)$ phase-plane stability portraits, and combined-slip $g$-$g$ acceleration envelopes across the `reference`, `understeer`, and `oversteer` vehicle variants:

```bash
torquevector handling --output results/handling
```

---

## 4 Quality Gate Q11 Assessment and Full Quality Audit (`q11`, `quality-audit`)

```bash
# Run the complete Gate Q11 torque-vectoring assessment (nominal, robustness, and degraded matrices)
torquevector q11 --output results/q11

# Execute the full 13-gate automated quality audit (Q01-Q13 tests, coverage, benchmarks, invariants)
torquevector quality-audit --output results/quality_audit
```

---

## 5 Offline Telemetry Rendering and Tyre Curve Plotting (`render`, `plot-tyre`)

`render` (alias `visualize`) generates figures and animations strictly from persisted CSV telemetry in `results/` without re-running the ODE solver:

```bash
# Render static PNG or vector SVG plots for every report section
torquevector render --run-id latest --format png
torquevector render --run-id latest --format svg --sections kpi,allocation,friction_circle

# Render multi-controller top-down trajectory & wheel-force GIF animation
torquevector render --run-id latest --format gif --fps 20 --max-frames 140

# Render spatial road-friction map animation for a single controller
torquevector render --run-id latest --format road-gif --controller lqr

# Plot pure and combined-slip tyre characteristics directly from a tyre YAML
torquevector plot-tyre --tyre configs/tyres/pacejka_fitted_synthetic.yaml --format png
```

---

## 6 Offline Particle Swarm Optimization (`optimize`)

```bash
# Run a reproducible offline PSO controller tuning study
torquevector optimize --config configs/optimization/nmpc_tuning.yaml
```
