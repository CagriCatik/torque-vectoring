"""Bewertungsregel des Qualitäts-Audits: Testergebnisse je Check → vorgeschlagene Punkte (Q01–Q13).

Die Punkte sind ein **Vorschlag zur Review**, keine Freigabe. ``docs/quality/quality_scorecard.csv`` wird nie
verändert; der Audit schreibt eine eigene Scorecard unter ``results/quality_audit/<zeit>/``.

Regel (QUALITY_GATES.md, Bewertungssystem):
- 2 Punkte: automatisierter Test des Checks bestanden, Akzeptanzmaß vollständig nachgewiesen.
- 1 Punkt: Test bestanden, aber nur gegen synthetische statt reale/belastbare Referenzdaten (``DATA_LIMITED``,
  Blocker B1) oder Teilnachweis (z. B. Coverage zwischen 80 und 90 %).
- 0 Punkte: Test fehlgeschlagen, kein Test vorhanden oder reale Messdaten zwingend erforderlich (Q10).
"""

from __future__ import annotations

import csv
import re
from pathlib import Path

# Checks, deren Akzeptanzmaß unabhängige Mess- oder belastbare Benchmarkdaten verlangt (Blocker B1/B5).
DATA_LIMITED = {
    'Q02.1': 'Identifikation nur gegen synthetischen Brush-Datensatz (kein realer Reifenprüfstand)',
    'Q02.2': 'Hold-out-NRMSE nur gegen synthetische Daten',
    'Q02.3': 'Peak/Steifigkeit nur gegen synthetisches Wahrheitsmodell',
    'Q03.2': 'Combined-Slip-Hold-out nur gegen synthetische Daten',
}
REQUIRES_REAL_DATA = {f'Q10.{i}': 'REQUIRED_REAL_DATA: keine Fahrzeugmessdaten vorhanden' for i in range(1, 6)}
COVERAGE_CHECK = 'Q13.3'
COVERAGE_FULL, COVERAGE_PARTIAL = 90.0, 80.0

_TEST_NAME = re.compile(r'test_(Q\d\d)_(\d)_')


def check_id(test_name: str) -> str | None:
    match = _TEST_NAME.search(test_name)
    return f'{match.group(1)}.{match.group(2)}' if match else None


def propose_points(outcomes: dict[str, str], coverage_percent: float | None) -> dict[str, dict]:
    """``outcomes``: Check-ID → 'passed' | 'failed'. Ergebnis je Check: points, basis."""
    result = {}
    for cid in sorted(set(outcomes) | set(REQUIRES_REAL_DATA)):
        if cid in REQUIRES_REAL_DATA:
            result[cid] = {'points': 0, 'basis': REQUIRES_REAL_DATA[cid]}
            continue
        status = outcomes.get(cid)
        if status != 'passed':
            result[cid] = {'points': 0, 'basis': f'Test {status or "fehlt"}'}
        elif cid in DATA_LIMITED:
            result[cid] = {'points': 1, 'basis': f'Test bestanden, DATA_LIMITED: {DATA_LIMITED[cid]}'}
        elif cid == COVERAGE_CHECK:
            if coverage_percent is None:
                result[cid] = {'points': 1, 'basis': 'Skips/Branch-Tests bestanden, Coverage nicht gemessen'}
            else:
                points = 2 if coverage_percent >= COVERAGE_FULL else 1 if coverage_percent >= COVERAGE_PARTIAL else 0
                result[cid] = {
                    'points': points,
                    'basis': f'Core-Line-Coverage {coverage_percent:.1f} % ' f'(Ziel >= {COVERAGE_FULL:g} %)',
                }
        else:
            result[cid] = {'points': 2, 'basis': 'automatisierter Test bestanden'}
    return result


def write_scorecard(template: Path, target: Path, proposal: dict[str, dict], evidence: dict[str, str]) -> None:
    """Vorgeschlagene Scorecard im Originalformat; Reviewer-Felder bleiben ausdrücklich offen."""
    with template.open(encoding='utf-8') as fh:
        reader = csv.DictReader(fh)
        fields, rows = reader.fieldnames, list(reader)
    for row in rows:
        entry = proposal.get(row['check_id'])
        if entry is None:
            continue
        row['points'] = str(entry['points'])
        row['evidence_path'] = evidence.get(row['check_id'], '') if entry['points'] else ''
        row['reviewer'] = 'PENDING_REVIEW'
        row['review_date'] = ''
        row['note'] = entry['basis']
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open('w', encoding='utf-8', newline='') as fh:
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
