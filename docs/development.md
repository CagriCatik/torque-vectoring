# Development, Testing, and Verification Guide

This guide documents the test suite organization, coverage measurement, code formatting rules, and schema generation tools for developers working on `torquevector`.

---

## 1 Test Suite Organization (`tests/`)

The test suite comprises six specialized test categories executed via Python's `unittest` runner (`tests/quality/test_q13_software.py` enforces zero skipped tests across all categories):

```bash
# Run the entire test suite (unit, integration, regression, architecture, system, quality)
.venv/bin/python -m unittest discover -s tests -v
```

| Directory | Scope and Verification Target |Command |
|---|---|---|
| **`tests/unit/`** | Pure unit tests for tyre Magic Formula, combined slip, kinematics, linear/LTV/NMPC controllers, BVLS allocation, ABS/ASR slip control, handling metrics, and parameter reference | `.venv/bin/python -m unittest discover -s tests/unit -t tests -v` |
| **`tests/integration/`** | Coupled multi-component tests: algebraic load-transfer root solver, multi-$\mu$ road boundary event splitting, friction perception modes, CLI storage, and ABS/ASR split-$\mu$ scenarios | `.venv/bin/python -m unittest discover -s tests/integration -t tests -v` |
| **`tests/regression/`** | Normalized AST SHA-256 hash verification of 44 frozen core mathematical functions and dataclasses (`structure_definition_hashes.json`) | `.venv/bin/python -m unittest discover -s tests/regression -t tests -v` |
| **`tests/architecture/`** | Acyclic package layer dependencies, zero hidden numeric defaults in core layers, $100\,\%$ YAML schema-to-dataclass binding completeness, and packaged resource parity | `.venv/bin/python -m unittest discover -s tests/architecture -t tests -v` |
| **`tests/system/`** | End-to-end numerical regression baselines across frozen manoeuvres (`structure_nominal_reference.json`) and YAML profile equivalence | `.venv/bin/python -m unittest discover -s tests/system -t tests -v` |
| **`tests/quality/`** | Automated verification of all 13 quality gates (`test_q01_parameters.py` through `test_q13_software.py`, 61 test methods covering 65 gate checks) | `.venv/bin/python -m unittest discover -s tests/quality -t tests -v` |

---

## 2 Core Branch and Line Coverage (`.coveragerc`)

Quality Gate `Q13.3` requires $\ge 90.0\,\%$ line coverage across the core mathematical packages (`domain`, `physics`, `prediction`, `control`, `simulation`, `evaluation`) configured in [`.coveragerc`](../.coveragerc):

```bash
.venv/bin/python -m coverage run -m unittest discover -s tests
.venv/bin/python -m coverage report
```

---

## 3 Code Formatting and Parameter Reference Synchronization

```bash
# Format check (Black, line-length 120, single quotes preserved via skip-string-normalization)
.venv/bin/python -m black --check src tests scripts examples

# Regenerate docs/configuration/PARAMETER_REFERENCE.md after editing schema.py or default.yaml
.venv/bin/python scripts/configuration/generate_parameter_reference.py
.venv/bin/python scripts/configuration/generate_parameter_reference.py --check
```
