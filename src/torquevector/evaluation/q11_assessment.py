"""Maschinenlesbare Q11-Bewertung aus gespeicherten Matrixergebnissen (keine Simulation).

Kriterien (QUALITY_GATES.md Q11) werden hier nur ausgewertet,
nicht verändert:
- Q11.3: TV-Regler verbessert den Gierraten-RMSE ggü. OFF um >= 20 % in **jedem** Fall der vorab
  festgelegten Menge ``q11_nominal`` (strenger als ein Mittelwert), Bahnfehler höchstens 10 % schlechter,
  Status PASS (Modellgrenzen eingehalten).
- Q11.4: In ``q11_degraded`` endet jeder Lauf definiert (PASS, INVALID_MODEL_ENVELOPE oder FAIL_CLOSED
  mit Grund), Kennzahlen sind endlich und Reglerkommandos bleiben in ihren Grenzen.
- Robustheit (``q11_robust``): Bericht derselben Kennzahlen ohne Nachoptimierung.
"""

from __future__ import annotations

from math import isfinite
from typing import Mapping

DEFINED = ('PASS', 'INVALID_MODEL_ENVELOPE', 'FAIL_CLOSED')
TV = ('p', 'lqr', 'ltv', 'nmpc')


def _relative(entry: dict, off: dict):
    if entry.get('status') != 'PASS' or off.get('status') != 'PASS':
        return None
    m, b = entry['metrics'], off['metrics']
    return {
        'yaw_improvement_pct': 100 * (1 - m['yaw_rmse_rads'] / b['yaw_rmse_rads']),
        'path_improvement_pct': 100 * (1 - m['cross_rmse_m'] / b['cross_rmse_m']),
    }


def assess_nominal(matrix: Mapping[str, Mapping[str, dict]], *, min_yaw_pct: float, min_path_pct: float) -> dict:
    controllers = {}
    for kind in TV:
        cases = {}
        for case, records in matrix.items():
            if kind not in records:
                continue
            rel = _relative(records[kind], records.get('off', {}))
            cases[case] = {
                'status': records[kind]['status'],
                **(rel or {}),
                'qualifies': bool(
                    rel and rel['yaw_improvement_pct'] >= min_yaw_pct and rel['path_improvement_pct'] >= min_path_pct
                ),
            }
        if cases:
            controllers[kind] = {
                'cases': cases,
                'qualifies_all_cases': all(c['qualifies'] for c in cases.values()),
                'min_yaw_improvement_pct': min((c.get('yaw_improvement_pct', float('-inf')) for c in cases.values())),
                'min_path_improvement_pct': min((c.get('path_improvement_pct', float('-inf')) for c in cases.values())),
            }
    qualified = [k for k, v in controllers.items() if v['qualifies_all_cases']]
    return {
        'criterion': f'Gier >= {min_yaw_pct:g} % und Bahn >= {min_path_pct:g} % ggü. OFF in jedem Fall, Status PASS',
        'controllers': controllers,
        'qualified_controllers': qualified,
        'passed': bool(qualified),
    }


def assess_degraded(
    matrix: Mapping[str, Mapping[str, dict]],
    diagnostics: Mapping[str, Mapping[str, list]],
    moment_limits: Mapping[str, float],
) -> dict:
    findings, passed = {}, True
    for case, records in matrix.items():
        for kind, entry in records.items():
            problems = []
            if entry.get('status') not in DEFINED:
                problems.append(f"undefinierter Status {entry.get('status')}")
            if entry.get('status') == 'FAIL_CLOSED' and not entry.get('reason'):
                problems.append('FAIL_CLOSED ohne Grund')
            for key, value in (entry.get('metrics') or {}).items():
                if isinstance(value, float) and not isfinite(value):
                    problems.append(f'nicht endliche Kennzahl {key}')
            limit = moment_limits.get(kind)
            ticks = diagnostics.get(case, {}).get(kind, [])
            if limit is not None and ticks and max(abs(t['requested_nm']) for t in ticks) > limit + 1e-6:
                problems.append('Reglerkommando über Momentgrenze')
            findings.setdefault(case, {})[kind] = {
                'status': entry.get('status'),
                'reason': entry.get('reason'),
                'problems': problems,
            }
            passed = passed and not problems
    return {
        'criterion': 'jeder Lauf endet definiert, endliche Kennzahlen, Kommandos in Grenzen',
        'cases': findings,
        'passed': passed,
    }


def assess_robust(matrix: Mapping[str, Mapping[str, dict]], *, min_yaw_pct: float, min_path_pct: float) -> dict:
    return assess_nominal(matrix, min_yaw_pct=min_yaw_pct, min_path_pct=min_path_pct)
