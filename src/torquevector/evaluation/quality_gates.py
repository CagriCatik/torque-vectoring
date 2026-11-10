"""Strict, traceable 0-10 scoring of fahrdynamik quality gates."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from torquevector.infrastructure.configuration import resource_path


def assess(csv_path: Path, manifest_path: Path | None = None) -> tuple[dict, bool]:
    manifest_path = manifest_path or resource_path('quality/quality_gate_manifest.json')
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    with csv_path.open(encoding='utf-8', newline='') as stream:
        rows = list(csv.DictReader(stream))
    expected_categories = {c['id']: c for c in manifest['categories']}
    actual_categories = {r['category'] for r in rows}
    if actual_categories != set(expected_categories):
        raise ValueError(
            f'Category mismatch: missing {set(expected_categories) - actual_categories}, '
            f'extra {actual_categories - set(expected_categories)}'
        )

    groups: dict[str, dict] = {}
    for category, spec in expected_categories.items():
        entries = [r for r in rows if r['category'] == category]
        check_ids = [r['check_id'] for r in entries]
        if len(check_ids) != 5 or len(set(check_ids)) != 5 or set(check_ids) != set(spec['check_ids']):
            raise ValueError(f'{category}: missing, duplicated or unexpected check ids')
        critical = {r['check_id'] for r in entries if r['critical'] == 'true'}
        if critical != set(spec['critical_checks']):
            raise ValueError(f'{category}: critical check designations have been changed')
        group = {'title': spec['name'], 'points_credited': 0, 'score': None, 'not_evaluated': [], 'failed_critical': []}
        for row in entries:
            raw = row['points'].strip()
            if raw not in ('0', '1', '2'):
                group['not_evaluated'].append(row['check_id'])
                points = 0
            else:
                points = int(raw)
                if points > 0 and not row['evidence_path'].strip():
                    group['not_evaluated'].append(row['check_id'] + ':missing_evidence')
                    points = 0
            group['points_credited'] += points
            if row['check_id'] in critical and points != 2:
                group['failed_critical'].append(row['check_id'])
        if not group['not_evaluated']:
            group['score'] = group['points_credited']
        group['passed'] = (
            group['score'] is not None
            and group['score'] >= manifest['min_points_per_category']
            and not group['failed_critical']
        )
        groups[category] = group

    passed = all(g['passed'] for g in groups.values())
    return {
        'pass': passed,
        'min_points_per_category': manifest['min_points_per_category'],
        'category_count': len(groups),
        'categories': groups,
    }, passed


def main() -> int:
    parser = argparse.ArgumentParser(description='Evaluate 13 evidence-based quality gates')
    parser.add_argument('--scorecard', type=Path, default=resource_path('quality/quality_scorecard.csv'))
    parser.add_argument('--manifest', type=Path, default=resource_path('quality/quality_gate_manifest.json'))
    parser.add_argument('--json', type=Path, help='Write machine-readable report')
    args = parser.parse_args()
    report, passed = assess(args.scorecard, args.manifest)
    for key, data in report['categories'].items():
        status = 'PASS' if data['passed'] else ('NOT_EVALUATED' if data['score'] is None else 'FAIL')
        value = '--' if data['score'] is None else f'{data["score"]:02d}'
        print(
            f'{key} {value}/10 {status} | {data["title"]} '
            f'| unevaluated={len(data["not_evaluated"])} critical_failed={len(data["failed_critical"])}'
        )
    print('OVERALL:', 'PASS' if passed else 'BLOCKED')
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
