# Quality Gates Framework (Q01–Q13) and Grading Logic

This document explains the structure, grading rules, and CLI verification tools for the 13-gate quality framework defined in [`docs/quality/QUALITY_GATES.md`](../quality/QUALITY_GATES.md), [`docs/quality/quality_gate_manifest.json`](../quality/quality_gate_manifest.json), and [`src/torquevector/evaluation/quality_gates.py`](../../src/torquevector/evaluation/quality_gates.py).

---

## 1 Structure of the 13 Quality Gates

The verification and validation framework partitions simulator quality into **13 categories (`Q01`–`Q13`)**, each comprising exactly **5 sub-checks** (`Qxx.1`–`Qxx.5`, $65$ sub-checks in total):

1. ** Scoring Scale ($0\text{--}2$ points per sub-check, $0\text{--}10$ points per gate)**:
   - **`0` points**: Not implemented, failing verification test, or missing required evidence artifact.
   - **`1` point**: Partially fulfilled (e.g., numerically verified against a synthetic surrogate dataset rather than physical bench measurements, or missing secondary robustness coverage).
   - **`2` points**: Fully verified with automated test coverage, reproducible artifact path (`evidence_path`), and reviewer sign-off.
2. **Gate Passing Criteria (`quality_gate_manifest.json`)**:
   A quality gate `Qxx` passes (`passed: true`) if and only if:
   - All sub-checks in `Qxx` have been evaluated (no blank entries),
   - Every **critical** sub-check (`critical: true`, 25 across the 13 gates) achieves the full **`2/2` points** with a valid non-empty `evidence_path`, and
   - The category sum satisfies $\sum_{j=1}^5 \text{points}(\text{Qxx.}j) \ge 8/10$.
3. **Overall Release Criterion**:
   The overall release status is `PASS` only when all 13 categories pass simultaneously and none of the five release blockers (`B1`–`B5`) are active.

---

## 2 Evaluating a Scorecard via CLI

To grade any scorecard CSV against the manifest rules:

```bash
# Grade the repository template (exits with code 1: BLOCKED, 0/13 evaluated)
.venv/bin/python -m torquevector.evaluation.quality_gates \
    --scorecard docs/quality/quality_scorecard.csv \
    --manifest docs/quality/quality_gate_manifest.json \
    --json results/gates_template.json

# Run the full automated audit and grade the proposed scorecard (10/13 PASS, 3 data-blocked)
.venv/bin/torquevector quality-audit --output results/quality_audit
```

---

## 3 Active Release Blockers (`B1`–`B5`)

| Blocker ID | Definition | Current Status |
|---|---|:---:|
| **`B1`** | Tyre parameters calibrated only on synthetic data (`Q02.1`–`Q02.3`, `Q03.2` capped at $1/2$) | **ACTIVE** (synthetic brush dataset) |
| **`B2`** | Missing combined-slip friction-circle saturation ($\sqrt{F_x^2 + F_y^2} > \mu_{\text{eff}} F_z$) | **RESOLVED** (`Q03.4` = $2/2$) |
| **`B3`** | Core mathematical/physical defects (frame sign error, $\sum F_{z,i} \neq mg$, $\kappa$ not derived from $\omega_i$) | **RESOLVED** (`Q04`, `Q05` = $10/10$) |
| **`B4`** | Algebraic loop in normal load transfer or numerical step-size divergence | **RESOLVED** (`Q07` = $10/10$) |
| **`B5`** | No experimental vehicle validation against physical telemetry (`Q10` = $0/10$) | **ACTIVE** (`REQUIRED_REAL_DATA`) |
