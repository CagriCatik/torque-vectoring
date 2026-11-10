# YAML Configuration System, Schema Validation, and Provenance

This document describes the modular YAML configuration architecture, schema validation rules, override precedence, and run snapshotting implemented in [`src/torquevector/infrastructure/config/`](../src/torquevector/infrastructure/config/).

For the complete auto-generated table of all $175+$ configuration parameters, types, units, ranges, and target dataclass bindings, see [`docs/configuration/PARAMETER_REFERENCE.md`](configuration/PARAMETER_REFERENCE.md).

---

## 1 Modular Configuration Layout (`configs/`)

The entry point [`configs/default.yaml`](../configs/default.yaml) assembles a complete simulation setup from modular, single-responsibility component files:

```text
configs/
├── default.yaml                        # Root application manifest (schema_version: 1)
├── vehicles/
│   └── generic_passenger_car.yaml      # Mass, yaw inertia, geometry, aerodynamics, load transfer
├── tyres/
│   ├── pacejka_fitted_synthetic.yaml   # Calibrated MF B/C/E + elliptic combined-slip parameters
│   └── pacejka_baseline.yaml           # Historical uncalibrated V11 baseline parameters
├── wheels/
│   └── standard_wheel.yaml             # Effective rolling radius, rotational inertia, min speed
├── roads/
│   ├── uniform_dry.yaml                # Constant nominal surface (mu = 1.0)
│   ├── dry_asphalt.yaml                # Uniform dry asphalt (mu = 0.90)
│   ├── wet_asphalt.yaml                # Uniform wet asphalt (mu = 0.40)
│   └── mixed_surface.yaml              # Multi-zone heterogeneous surface layout
├── controllers/
│   ├── off.yaml                        # Passive baseline (Mz = 0)
│   ├── p.yaml                          # Proportional yaw-rate + sideslip damping controller
│   ├── lqr.yaml                        # Gain-scheduled continuous-time Algebraic Riccati LQR
│   ├── ltv_mpc.yaml                    # Condensed matrix-exponential SLSQP LTV-MPC
│   └── nmpc.yaml                       # CasADi/IPOPT RK4 single-track nonlinear MPC
├── allocation/
│   └── torque_allocator.yaml           # BVLS weights, actuator bounds, friction-aware mode
├── control/
│   └── slip_control.yaml               # 200 Hz inner-loop velocity-form PI ABS/ASR settings
├── simulation/
│   ├── solver.yaml                     # RK45 tolerances, ZOH sample periods, load-transfer solver
│   ├── driver.yaml                     # Steering PD gains, speed PI, yaw_reference.source
│   └── validation.yaml                 # Envelope thresholds, Q11 qualification criteria, handling grid
├── scenarios/
│   ├── circle_track.yaml               # Constant-radius circle (R = 42 m, 15 m/s)
│   ├── double_lane_change.yaml         # ISO-inspired double lane change (17 m/s)
│   ├── step_steer.yaml                 # Open-loop 4.5 deg step steer (17 m/s)
│   ├── sine_steer.yaml                 # Open-loop 4.0 deg, 0.45 Hz sine steer (17 m/s)
│   ├── ramp_steer.yaml                 # Quasi-steady 0 -> 12 deg ramp steer (handling analysis)
│   └── straight_split_mu_*.yaml        # Multi-mu and split-mu braking/acceleration scenarios
├── profiles/
│   ├── nominal.yaml                    # 4-manoeuvre nominal comparison matrix
│   ├── regression.yaml                 # Frozen V11 legacy compatibility profile (path_curvature)
│   ├── vehicle_understeer.yaml         # Strong understeer axle-scaling variant (K = +2.71 deg/g)
│   └── vehicle_oversteer.yaml          # Oversteer axle-scaling variant (K = -1.72 deg/g)
└── optimization/
    ├── pso.yaml                        # Particle Swarm Optimization hyperparameters
    └── *_tuning.yaml                   # Controller-specific offline tuning studies
```

---

## 2 Precedence Order and Origin Tracking

When resolving a configuration (`resolve_from_args`), layers are merged in strict deterministic order from lowest to highest priority:

1. **Base Application Manifest** (`--config`, default `configs/default.yaml`) and its `include:` component files.
2. **Profile Overrides** (`--profile configs/profiles/<name>.yaml`).
3. **Component Replacement Flags** (`--vehicle`, `--tyre`, `--wheel`, `--road`, `--controller-config`, `--allocation`, `--solver`, `--driver`, `--scenario`).
4. **Explicit CLI Key-Value Overrides** (`--set path.to.key=value`, repeatable).

Every resolved parameter retains its exact provenance chain (`file:line` or `override (CLI)`), inspectable via:

```bash
# Validate syntax, schema types, physical ranges, and unit contracts
torquevector config validate --config configs/default.yaml

# Export fully resolved configuration with source file/line origins
torquevector config show --config configs/default.yaml --origins
```

---

## 3 Reproducible Run Snapshots (`results/<run_id>/config/`)

Every simulation or comparison run writes four immutable configuration audit files into `results/<run_id>/config/`:
- `resolved_config.yaml` — Self-contained, fully merged YAML configuration capable of reproducing the run with a single `--config` flag.
- `resolved_config.origins.json` — Exact file path and line number where each leaf parameter originated.
- `resolved_config.sources.json` — List of all participating YAML files with their individual 64-character SHA-256 hashes.
- `resolved_config.fingerprint` — Master SHA-256 digest of the canonicalized configuration tree.
