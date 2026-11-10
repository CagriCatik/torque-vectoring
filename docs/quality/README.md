# Normative quality documents

These files define the quality gates. They are unchanged originals: hash-protected and partly in German.

| File | Content |
|---|---|
| `QUALITY_GATES.md` | the 13 gates with five checks each, scoring rules, release blockers (German) |
| `SCENARIO_MATRIX.md` | mandatory reference and stress scenarios S01–S14 (German) |
| `quality_gate_manifest.json` | machine-readable gate definition (check IDs, critical checks, minimum score) |
| `quality_scorecard.csv` | the **official** scorecard; points are entered only after review |

An English summary is in [../validation/quality-gates.md](../validation/quality-gates.md). The latest automated assessment is in [../validation/quality-audit.md](../validation/quality-audit.md).

```bash
python -m torquevector.evaluation.quality_gates --scorecard docs/quality/quality_scorecard.csv \
    --manifest docs/quality/quality_gate_manifest.json --json results/quality_report.json
```
