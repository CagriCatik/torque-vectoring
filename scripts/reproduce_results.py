"""Reproduce uniform scenario- and specification-based results in ``results/`` and curate ``assets/``."""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / 'results'
ASSETS = ROOT / 'assets'
PYTHON = sys.executable

SCENARIOS: tuple[dict[str, object], ...] = (
    {'name': 'circle_track', 'scenario': 'circle_track', 'road_gif': False},
    {'name': 'double_lane_change', 'scenario': 'double_lane_change', 'road_gif': False},
    {'name': 'step_steer', 'scenario': 'step_steer', 'road_gif': False},
    {'name': 'sine_steer', 'scenario': 'sine_steer', 'road_gif': False},
    {'name': 'mixed_mu_lane_change', 'scenario': 'mixed_mu_lane_change', 'road_gif': True},
    {'name': 'straight_multi_mu', 'scenario': 'straight_multi_mu', 'road_gif': True},
    {'name': 'straight_split_mu_braking', 'scenario': 'straight_split_mu_braking', 'road_gif': True},
    {
        'name': 'straight_split_mu_braking_abs',
        'scenario': 'straight_split_mu_braking_above_limit',
        'profiles': ('configs/profiles/slip_control.yaml',),
        'road_gif': True,
    },
    {'name': 'straight_split_mu_acceleration', 'scenario': 'straight_split_mu_acceleration', 'road_gif': True},
    {
        'name': 'double_lane_change_wet_mu04',
        'scenario': 'double_lane_change',
        'sets': ('road.default_surface.mu=0.4',),
        'road_gif': False,
    },
)

ASSET_EXPORTS: tuple[tuple[Path, str], ...] = (
    (RESULTS / 'double_lane_change/animations/comparison.gif', 'double_lane_change_comparison.gif'),
    (RESULTS / 'circle_track/animations/comparison.gif', 'circle_track_comparison.gif'),
    (RESULTS / 'mixed_mu_lane_change/animations/road_lqr.gif', 'mixed_mu_lane_change.gif'),
    (RESULTS / 'straight_split_mu_braking_abs/animations/road_lqr.gif', 'split_mu_braking_abs.gif'),
    (RESULTS / 'double_lane_change/plots/kpi.png', 'double_lane_change_kpi.png'),
    (RESULTS / 'double_lane_change/plots/allocation.png', 'double_lane_change_allocation.png'),
    (RESULTS / 'double_lane_change/plots/friction_circle.png', 'double_lane_change_friction_circle.png'),
    (RESULTS / 'mixed_mu_lane_change/plots/road_map.png', 'mixed_mu_lane_change_road_map.png'),
    (RESULTS / 'handling_iso4138/plots/handling_diagram_off.png', 'handling_diagram_off.png'),
    (RESULTS / 'handling_iso4138/plots/handling_diagram_lqr.png', 'handling_diagram_lqr.png'),
    (RESULTS / 'handling_iso4138/plots/phase_plane.png', 'phase_plane.png'),
    (RESULTS / 'handling_iso4138/plots/gg_diagram.png', 'gg_diagram.png'),
    (
        RESULTS / 'tyre_pacejka_fitted_synthetic/pacejka_fitted_synthetic_characteristics.png',
        'tyre_characteristics.png',
    ),
)


def _cli(*args: str) -> None:
    env = dict(os.environ, MPLBACKEND='Agg')
    cmd = [PYTHON, '-m', 'torquevector.cli.main', '--quiet', *args]
    print('->', ' '.join(cmd[3:]), flush=True)
    subprocess.run(cmd, cwd=ROOT, env=env, check=True)


def run_scenarios() -> list[dict[str, object]]:
    summaries = []
    for spec in SCENARIOS:
        name = str(spec['name'])
        target = RESULTS / name
        if target.exists():
            shutil.rmtree(target)
        args = [
            'compare',
            '--scenario',
            str(spec['scenario']),
            '--results-dir',
            str(RESULTS),
            '--run-name',
            name,
            '--report',
        ]
        for prof in spec.get('profiles', ()):
            args += ['--profile', str(prof)]
        for override in spec.get('sets', ()):
            args += ['--set', str(override)]
        try:
            _cli(*args)
        except subprocess.CalledProcessError:
            # Degraded scenarios (e.g. OFF on wet DLC) exit non-zero cleanly while persisting PASS TV controllers.
            if not (target / 'metrics/comparison.json').is_file():
                raise
        _cli('render', '--result', str(target), '--format', 'gif', '--fps', '10', '--max-frames', '80', '--dpi', '100')
        if spec.get('road_gif'):
            _cli('render', '--result', str(target), '--format', 'road-gif', '--controller', 'lqr')
        comp = json.loads((target / 'metrics/comparison.json').read_text(encoding='utf-8'))
        summaries.append({'name': name, 'controllers': comp})
    return summaries


def run_specifications() -> None:
    handling_dir = RESULTS / 'handling_iso4138'
    if handling_dir.exists():
        shutil.rmtree(handling_dir)
    _cli('handling', '--output', str(handling_dir), '--flat')

    tyre_dir = RESULTS / 'tyre_pacejka_fitted_synthetic'
    if tyre_dir.exists():
        shutil.rmtree(tyre_dir)
    _cli('plot-tyre', '--output', str(tyre_dir), '--dpi', '130')

    matrix_dir = RESULTS / 'scenario_matrix_benchmark'
    if matrix_dir.exists():
        shutil.rmtree(matrix_dir)
    matrix_dir.mkdir(parents=True, exist_ok=True)
    from torquevector.cli.composition import q11_assessment
    from torquevector.infrastructure.config.resolver import resolve

    base_cfg = ROOT / 'configs/default.yaml'
    report = q11_assessment(
        lambda set_name: resolve(base_cfg, overrides=[('run.matrix_set', set_name)]),
        matrix_dir,
    )
    (matrix_dir / 'assessment.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n', encoding='utf-8')


def write_results_index(summaries: list[dict[str, object]]) -> None:
    lines = [
        '# Reproducible Simulation & Specification Results',
        '',
        'All directories under `results/` use uniform scenario or specification identifiers generated by `scripts/reproduce_results.py`.',
        '',
        '## Scenario Runs (`OFF`, `P`, `LQR`, `LTV-MPC`, `NMPC`)',
        '',
        '| Scenario Folder | Status (`OFF` / `P` / `LQR` / `LTV` / `NMPC`) | Best Yaw RMSE Improvement vs. `OFF` | Report & 2D Animations |',
        '|---|---|---|---|',
    ]
    for entry in summaries:
        name = entry['name']
        comp = entry['controllers']
        statuses = ' / '.join(f'{k.upper()}: {v.get("status", "?")}' for k, v in comp.items())
        best_impr = max(
            (v['vs_off']['yaw_improvement_pct'] for v in comp.values() if isinstance(v, dict) and v.get('vs_off')),
            default=None,
        )
        impr_str = '–' if best_impr is None else f'+{best_impr:.1f} %'
        anim = f'[comparison.gif]({name}/animations/comparison.gif)'
        if (RESULTS / name / 'animations/road_lqr.gif').is_file():
            anim += f' · [road_lqr.gif]({name}/animations/road_lqr.gif)'
        lines.append(
            f'| [`{name}`]({name}/) | {statuses} | {impr_str} | [report.pdf]({name}/plots/report.pdf) · {anim} |'
        )
    lines += [
        '',
        '## Specification Studies',
        '',
        '| Specification Folder | Scope | Key Outputs |',
        '|---|---|---|',
        '| [`handling_iso4138`](handling_iso4138/) | ISO 4138 self-steering, yaw gain, $g$-$g$ envelope, $(\\beta, r)$ phase plane | [SUMMARY.md](handling_iso4138/SUMMARY.md), [handling.json](handling_iso4138/handling.json), `plots/*.png` |',
        '| [`tyre_pacejka_fitted_synthetic`](tyre_pacejka_fitted_synthetic/) | Combined-slip Pacejka characteristics & load sensitivity | [pacejka_fitted_synthetic_characteristics.png](tyre_pacejka_fitted_synthetic/pacejka_fitted_synthetic_characteristics.png) |',
        '| [`scenario_matrix_benchmark`](scenario_matrix_benchmark/) | Nominal, robustness ($m +10\\%$, $\\mu -10\\%$, mixed-$\\mu$), and degraded benchmark suites | [assessment.json](scenario_matrix_benchmark/assessment.json), `q11_nominal/`, `q11_robust/`, `q11_degraded/` |',
    ]
    (RESULTS / 'README.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')


def export_assets() -> None:
    ASSETS.mkdir(parents=True, exist_ok=True)
    for src, dst_name in ASSET_EXPORTS:
        if src.is_file():
            shutil.copy2(src, ASSETS / dst_name)
        else:
            print(f'WARNING: missing asset source {src}', file=sys.stderr)


def main() -> int:
    for legacy in ('runs', 'multi_mu', 'q11', 'handling', 'tyre_plots', 'quality_audit'):
        path = RESULTS / legacy
        if path.exists():
            shutil.rmtree(path)
    RESULTS.mkdir(parents=True, exist_ok=True)
    summaries = run_scenarios()
    run_specifications()
    write_results_index(summaries)
    export_assets()
    print('Reproduced all scenarios, specifications, and curated assets/ successfully.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
