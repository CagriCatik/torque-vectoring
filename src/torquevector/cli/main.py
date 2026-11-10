"""Einziger Einstiegspunkt ``torquevector`` (Kurzform ``tvsim``).

Alle Befehle arbeiten mit YAML-Konfigurationen (Standard: ``configs/default.yaml``).
Ergebnisse landen unter ``results/<run_id>/``, Logdateien unter ``logs/``.
Numerische Algorithmen liegen in den Fachpaketen; dieses Modul verbindet nur Anwendungsfälle.
"""

import argparse
import json
import logging
import os
import sys
from dataclasses import replace
from pathlib import Path

from torquevector.control.factory import CONTROLLERS

LOG = logging.getLogger('torquevector.cli')
YAML_SUFFIXES = ('.yaml', '.yml')
COMPONENT_FLAGS = ('vehicle', 'tyre', 'wheel', 'powertrain', 'allocation', 'road')
REPOSITORY_DEFAULT = Path(__file__).resolve().parents[3] / 'configs' / 'default.yaml'


def results_base() -> Path:
    """Top-level output folder (``results/``); collections such as multi-mu and q11 live below it."""
    return (Path.cwd() / 'results').resolve()


def results_root(value=None):
    """Folder of single simulate/compare runs: --results-dir > TORQUEVECTOR_RESULTS_DIR > results/runs."""
    return Path(value or os.environ.get('TORQUEVECTOR_RESULTS_DIR') or results_base() / 'runs').expanduser().resolve()


def _is_yaml(value):
    return isinstance(value, str) and value.lower().endswith(YAML_SUFFIXES)


def _scenario_config(args):
    """``--config <szenario.yaml>`` ist eine Kurzform für ``--scenario`` mit Standardkonfiguration."""
    value = getattr(args, 'config', None)
    if value and _is_yaml(value) and Path(value).is_file() and _file_kind(Path(value)) == 'component':
        from torquevector.infrastructure.config.resolver import load_component

        if load_component(value).kind == 'scenario':
            if getattr(args, 'scenario', None):
                raise ValueError('--config verweist auf ein Szenario; nicht zusätzlich --scenario angeben')
            args.scenario, args.config = value, None
    return args


def base_config(args):
    """--config > TORQUEVECTOR_CONFIG > ./configs/default.yaml > configs/ des Checkouts."""
    if getattr(args, 'config', None):
        return Path(args.config)
    for candidate in (os.environ.get('TORQUEVECTOR_CONFIG'), Path.cwd() / 'configs/default.yaml', REPOSITORY_DEFAULT):
        if candidate and Path(candidate).is_file():
            return Path(candidate)
    raise ValueError('Keine Konfiguration gefunden: --config angeben oder TORQUEVECTOR_CONFIG setzen')


def parse_sets(values):
    from torquevector.infrastructure.config.yaml_io import parse_scalar

    result = []
    for item in values or []:
        key, sep, raw = item.partition('=')
        if not sep or not key.strip():
            raise ValueError(f'--set erwartet pfad=wert, erhalten {item!r}')
        result.append((key.strip(), parse_scalar(raw)))
    return result


def resolve_from_args(args):
    """CLI-Optionen auf Konfigurationsebenen abbilden (Priorität: CLI > Profile > Anwendungsdatei)."""
    from torquevector.cli.composition import CONTROLLER_SLOTS
    from torquevector.infrastructure.config.resolver import load_component, resolve

    _scenario_config(args)
    config = base_config(args)
    replacements = {name: getattr(args, name) for name in COMPONENT_FLAGS if getattr(args, name, None)}
    overrides = []
    scenario = getattr(args, 'scenario', None)
    if scenario:
        if _is_yaml(scenario):
            replacements['scenario'] = scenario
        else:
            folders = (config.resolve().parent, config.resolve().parent.parent)
            found = [
                d / 'scenarios' / f'{scenario}.yaml'
                for d in folders
                if (d / 'scenarios' / f'{scenario}.yaml').is_file()
            ]
            if not found:
                raise ValueError(f'Szenario {scenario!r}: keine Datei scenarios/{scenario}.yaml neben {config}')
            replacements['scenario'] = str(found[0])
    controller = getattr(args, 'controller', None)
    if controller:
        if _is_yaml(controller):
            component = load_component(controller)
            if component.kind != 'controller':
                raise ValueError(f'{controller}: keine Regler-YAML')
            kind = {v: k for k, v in CONTROLLER_SLOTS.items()}[component.controller_type]
            replacements[f'controllers.{component.controller_type}'] = controller
        else:
            if controller not in CONTROLLERS:
                raise ValueError(f'Unbekannter Regler {controller!r}; erlaubt: {", ".join(CONTROLLERS)}')
            kind = controller
        overrides.append(('run.controllers', [kind]))
    if getattr(args, 'controllers', None):
        overrides.append(('run.controllers', list(args.controllers)))
    if getattr(args, 'speed_ms', None) is not None:
        overrides.append(('scenario.manoeuvre.speed_ms', args.speed_ms))
    if getattr(args, 'duration', None) is not None:
        overrides.append(('scenario.simulation_duration_s', args.duration))
    overrides += parse_sets(getattr(args, 'set', None))
    resolved = resolve(
        config, profiles=getattr(args, 'profile', None) or (), replacements=replacements, overrides=overrides
    )
    LOG.debug(
        'Konfiguration aufgelöst: %s, %d Dateien, Fingerprint %s', config, len(resolved.sources), resolved.fingerprint
    )
    return resolved


def run_simulation(args):
    from torquevector.cli.composition import run_configured

    report = False if args.no_report else (True if args.report else None)
    return run_configured(
        resolve_from_args(args),
        results_dir=args.results_dir,
        report=report,
        run_name=getattr(args, 'run_name', None),
    )


def locate_run(args):
    """--result <ordner> oder --run-id <id|latest> unter der Ergebniswurzel."""
    from torquevector.infrastructure.experiment_store import resolve_run

    if getattr(args, 'result', None) and args.result != 'latest':
        run = Path(args.result).expanduser().resolve()
        if not (run / 'manifest.json').is_file():
            raise FileNotFoundError(f'Kein Lauf unter {run}')
        return run
    root = results_root(args.results_dir)
    run_id = getattr(args, 'run_id', None) or 'latest'
    if run_id == 'latest':
        runs = sorted((p for p in root.iterdir() if (p / 'manifest.json').is_file()), key=lambda p: p.name)
        if not runs:
            raise FileNotFoundError(f'Keine Läufe unter {root}')
        return runs[-1]
    return resolve_run(root, run_id)


def render_run(args):
    from torquevector.cli.composition import load_run, render_report, render_road_animation, visualization_from_tree

    run = locate_run(args)
    if args.format == 'road-gif':
        result = render_road_animation(run, case=args.case, controller=args.controller)
        LOG.info('Fahrbahnanimation: %s', result)
        print(result)
        return 0
    if args.format == 'report':
        sections = args.sections.split(',') if args.sections else None
        written = render_report(run, case=args.case, sections=sections)
        for path in written:
            print(path)
        return 0
    data = load_run(run, args.case)
    if not data['seqs']:
        raise ValueError('Keine erfolgreiche Telemetrie zum Rendern')
    viz = visualization_from_tree(data['tree']) if data['tree'] is not None else None
    suffix = f'_{args.case}' if args.case else ''
    spec = data['spec']
    if args.format == 'gif':
        from torquevector.cli.composition import build_vehicle
        from torquevector.visualization.animation import AnimationConfig, render_controller_comparison_gif

        base = viz.animation if viz else AnimationConfig(fps=8, max_frames=100, dpi=80)
        explicit = {
            k: v for k, v in (('fps', args.fps), ('max_frames', args.max_frames), ('dpi', args.dpi)) if v is not None
        }
        name = Path(viz.animation_file_name if viz else 'comparison.gif').stem
        car = build_vehicle(data['tree']['vehicle']) if data['tree'] is not None else None
        result = render_controller_comparison_gif(
            data['seqs'],
            run / 'animations' / f'{name}{suffix}.gif',
            title=f'{spec.kind} | SYNTHETIC / NOT VEHICLE VALIDATED',
            config=replace(base, **explicit),
            car=car,
            primary=args.controller,
        )
    else:
        from torquevector.visualization.comparison import ChartStyle, chart

        style = viz.chart if viz else ChartStyle()
        if args.dpi is not None:
            style = replace(style, dpi=args.dpi)
        result = run / 'plots' / f'{viz.chart_file_stem if viz else "comparison"}{suffix}.{args.format}'
        chart(data['seqs'], spec, result, style)
    LOG.info('Gerendert: %s', result)
    print(result)
    return 0


MULTI_MU = (
    ('straight_multi_mu', 'straight_multi_mu'),
    ('straight_split_mu_braking', 'split_mu_braking'),
    ('straight_split_mu_acceleration', 'split_mu_acceleration'),
    ('mixed_mu_lane_change', 'mixed_mu_lane_change'),
)


def multi_mu(args):
    """Run the four multi-mu scenarios into one collection folder: one complete run folder per scenario."""
    from torquevector.cli.composition import render_road_animation, run_configured
    from torquevector.infrastructure.experiment_store import timestamp, unique_directory

    output = unique_directory(Path(args.output).expanduser().resolve(), timestamp())
    summary = {}
    for scenario, name in MULTI_MU:
        args.scenario, args.controller = scenario, None
        lines = []
        run_configured(resolve_from_args(args), results_dir=output, report=True, echo=lines.append, run_name=name)
        record = json.loads(lines[-1])
        run = Path(record['path'])
        gif = render_road_animation(run, controller=args.gif_controller)
        metrics = json.loads((run / 'metrics' / 'comparison.json').read_text())
        statuses = {k: v['status'] for k, v in metrics.get('controllers', metrics).items() if isinstance(v, dict)}
        summary[name] = {'status': record['status'], 'controllers': statuses, 'gif': str(gif.relative_to(output))}
        LOG.info('%s: %s (%s)', name, record['status'], run)
    write_collection_summary(output, 'Multi-µ road friction scenarios', summary)
    print(json.dumps({'output': str(output), 'scenarios': {k: v['status'] for k, v in summary.items()}}))
    return 0 if all(v['status'] == 'PASS' for v in summary.values()) else 1


def write_collection_summary(output: Path, title: str, summary: dict) -> None:
    """summary.json plus a SUMMARY.md with status per run and links to report, GIF and metrics."""
    (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    lines = [
        f'# {title}',
        '',
        f'Folder: `{output.name}`. Every subfolder is a complete run (see docs/user-guide.md).',
        '',
        '| Run | Status | Controllers | Report | Animation | Metrics |',
        '|---|---|---|---|---|---|',
    ]
    for name, entry in summary.items():
        controllers = ', '.join(f'{k}: {v}' for k, v in entry.get('controllers', {}).items())
        gif = f"[GIF]({entry['gif']})" if entry.get('gif') else '–'
        lines.append(
            f"| `{name}` | {entry['status']} | {controllers} | [report.pdf]({name}/plots/report.pdf) | {gif} "
            f"| [comparison.json]({name}/metrics/comparison.json) |"
        )
    (output / 'SUMMARY.md').write_text('\n'.join(lines) + '\n')


def q11(args):
    """Q11-Nachweis: Mengen q11_nominal, q11_robust, q11_degraded rechnen und maschinenlesbar bewerten."""
    from torquevector.infrastructure.experiment_store import timestamp, unique_directory
    from torquevector.cli.composition import q11_assessment

    output = unique_directory(Path(args.output).expanduser().resolve(), timestamp())
    base_sets = list(args.set or [])

    def resolve_set(name):
        args.set = base_sets + [f'run.matrix_set={name}']
        return resolve_from_args(args)

    report = q11_assessment(resolve_set, output)
    args.set = base_sets
    (output / 'q11_assessment.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    print(
        json.dumps(
            {
                'output': str(output),
                'Q11.3': report['Q11.3_nominal']['passed'],
                'qualified_controllers': report['Q11.3_nominal']['qualified_controllers'],
                'robust_qualified': report['robustness']['qualified_controllers'],
                'Q11.4': report['Q11.4_degraded']['passed'],
            }
        )
    )
    return 0 if report['Q11.3_nominal']['passed'] and report['Q11.4_degraded']['passed'] else 1


AUDIT_SCENARIOS = (
    ('double_lane_change', 'lqr'),
    ('circle_track', 'nmpc'),
    ('mixed_mu_lane_change', 'ltv'),
    ('straight_split_mu_braking', 'p'),
    ('straight_multi_mu', 'off'),
)


def quality_audit(args):
    """Gate-Suite tests/quality ausführen, Belege erzeugen und eine VORGESCHLAGENE Scorecard bewerten."""
    import csv
    import subprocess
    import unittest
    from datetime import datetime
    from torquevector.infrastructure.experiment_store import timestamp, unique_directory
    from torquevector.cli.composition import build_runtime, evaluate_case
    from torquevector.evaluation.invariants import invariant_report
    from torquevector.evaluation.quality_audit import check_id, propose_points, write_scorecard
    from torquevector.evaluation.quality_gates import assess
    from torquevector.evaluation.reference_benchmarks import run_reference_benchmarks
    from torquevector.infrastructure.config.resolver import resolve

    root = Path(args.repository).expanduser().resolve()
    output = unique_directory(Path(args.output).expanduser().resolve(), timestamp())
    (output / 'invariants').mkdir()

    # 1 Gate-Suite: ein Test je Check (Namensschema test_Qxx_y_*)
    suite = unittest.defaultTestLoader.discover(str(root / 'tests/quality'), top_level_dir=str(root / 'tests'))
    test_ids = [t.id() for t in _iter_tests(suite)]  # vor dem Lauf: unittest leert die Suite danach
    result = unittest.TestResult()
    started = datetime.now()
    suite.run(result)
    failed = {getattr(t, 'test_case', t).id(): 'failed' for t, _ in result.failures + result.errors}
    tests, outcomes, evidence = [], {}, {}
    for test_id in test_ids:
        status = failed.get(test_id, 'passed')
        cid = check_id(test_id)
        tests.append({'test': test_id, 'check_id': cid, 'status': status})
        if cid:
            outcomes[cid] = 'failed' if status == 'failed' or outcomes.get(cid) == 'failed' else 'passed'
            evidence[cid] = f'tests/quality/{test_id.split(".")[1]}.py::{test_id.split(".")[-1]}'
    details = {t.id(): tb for t, tb in result.failures + result.errors}
    (output / 'quality_tests.json').write_text(
        json.dumps(
            {'started': started.isoformat(), 'tests': tests, 'tracebacks': details}, indent=2, ensure_ascii=False
        )
        + '\n'
    )

    # 2 Belege: Referenzbenchmarks (Q09) und Invariantenberichte je Szenario (Q08)
    runtime = build_runtime(resolve(root / 'configs/default.yaml'))
    rows = run_reference_benchmarks(runtime.vehicle, runtime.tyre, runtime.wheel, runtime.tyre_model)
    with (output / 'reference_benchmarks.csv').open('w', newline='', encoding='utf-8') as fh:
        writer = csv.DictWriter(fh, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    for scenario, kind in AUDIT_SCENARIOS:
        case = build_runtime(
            resolve(
                root / 'configs/default.yaml',
                replacements={'scenario': str(root / f'configs/scenarios/{scenario}.yaml')},
            )
        )
        entry, seq, _ = evaluate_case(case, kind)
        report = invariant_report(seq, case.vehicle, case.wheel) if seq else {'status': entry['status']}
        (output / 'invariants' / f'{scenario}_{kind}.json').write_text(json.dumps(report, indent=2) + '\n')
    rel = output.relative_to(root) if output.is_relative_to(root) else output
    for cid in ('Q09.2', 'Q09.3', 'Q09.4', 'Q09.5'):
        evidence[cid] = f'{evidence.get(cid, "")}; {rel}/reference_benchmarks.csv'
    for cid in ('Q08.1', 'Q08.5'):
        evidence[cid] = f'{evidence.get(cid, "")}; {rel}/invariants/'

    # 3 Core-Line-Coverage (Q13.3)
    coverage = None
    if args.coverage == 'run':
        env = dict(os.environ, MPLBACKEND='Agg')
        rc = str(root / '.coveragerc')
        subprocess.run(
            [sys.executable, '-m', 'coverage', 'run', f'--rcfile={rc}', '-m', 'unittest', 'discover', '-s', 'tests'],
            cwd=root,
            env=env,
            capture_output=True,
            text=True,
        )
        done = subprocess.run(
            [sys.executable, '-m', 'coverage', 'report', f'--rcfile={rc}'], cwd=root, capture_output=True, text=True
        )
        (output / 'coverage.txt').write_text(done.stdout)
        coverage = _coverage_total(done.stdout)
    elif args.coverage_report:
        text = Path(args.coverage_report).read_text()
        (output / 'coverage.txt').write_text(text)
        coverage = _coverage_total(text)
    if coverage is not None:
        evidence['Q13.3'] = f'{evidence.get("Q13.3", "")}; {rel}/coverage.txt'

    # 4 Vorschlag und Gate-Report (Original-Scorecard bleibt unverändert)
    proposal = propose_points(outcomes, coverage)
    write_scorecard(root / 'docs/quality/quality_scorecard.csv', output / 'proposed_scorecard.csv', proposal, evidence)
    report, passed = assess(output / 'proposed_scorecard.csv', root / 'docs/quality/quality_gate_manifest.json')
    report.update(
        proposal_only=True,
        reviewer='PENDING_REVIEW',
        source_kind='SYNTHETIC_SIMULATION_ONLY',
        core_line_coverage_percent=coverage,
        tests_run=result.testsRun,
        tests_failed=len(result.failures) + len(result.errors),
    )
    (output / 'gate_report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
    lines = ['| Gate | Punkte | Status | kritisch offen |', '|---|---:|---|---|']
    for key, data in report['categories'].items():
        status = 'PASS' if data['passed'] else 'FAIL'
        lines.append(
            f"| {key} {data['title']} | {data['points_credited']}/10 | {status} | "
            f"{', '.join(data['failed_critical']) or '–'} |"
        )
    (output / 'SUMMARY.md').write_text('# Qualitäts-Audit (Vorschlag, Review ausstehend)\n\n' + '\n'.join(lines) + '\n')
    print(
        json.dumps(
            {
                'output': str(output),
                'overall': 'PASS' if passed else 'BLOCKED',
                'scores': {k: v['points_credited'] for k, v in report['categories'].items()},
                'tests_failed': report['tests_failed'],
                'coverage': coverage,
            },
            ensure_ascii=False,
        )
    )
    return 0 if passed else 1


def _iter_tests(suite):
    import unittest

    for item in suite:
        if isinstance(item, unittest.TestSuite):
            yield from _iter_tests(item)
        else:
            yield item


def _coverage_total(text: str) -> float | None:
    for line in text.splitlines():
        if line.startswith('TOTAL'):
            return float(line.split()[-1].rstrip('%'))
    return None


DEFAULT_VARIANTS = (
    ('reference', 'configs/default.yaml'),
    ('understeer', 'configs/profiles/vehicle_understeer.yaml'),
    ('oversteer', 'configs/profiles/vehicle_oversteer.yaml'),
)


def handling(args):
    """Self-steering characteristic and handling limits of one or more vehicle variants."""
    import numpy as np

    from torquevector.cli.composition import build_runtime, handling_analysis
    from torquevector.infrastructure.config.resolver import resolve
    from torquevector.infrastructure.experiment_store import timestamp, unique_directory
    from torquevector.visualization.handling_plot import (
        gg_figure,
        handling_diagram_figure,
        phase_plane_figure,
        yaw_gain_figure,
    )

    pairs = [v.split('=', 1) for v in args.variants] if args.variants else DEFAULT_VARIANTS
    root = Path(__file__).resolve().parents[3]
    variants = {name: (Path(path) if Path(path).is_absolute() else root / path) for name, path in pairs}
    base_out = Path(args.output).expanduser().resolve()
    if getattr(args, 'flat', False):
        output = base_out
        output.mkdir(parents=True, exist_ok=True)
    else:
        output = unique_directory(base_out, timestamp())
    report = handling_analysis(variants, output, controllers=args.controllers)
    figures = report.pop('_figures')
    plots = output / 'plots'
    plots.mkdir(exist_ok=True)
    speed = args.diagram_speed
    for controller in sorted({k[1].split('@')[0] for k in figures['curves']}):
        curves = {v: c for (v, key), c in figures['curves'].items() if key == f'{controller}@{speed:g}'}
        if curves:
            linear = {v: report['variants'][v]['linear']['understeer_gradient_deg_per_g'] for v in curves}
            fig = handling_diagram_figure(
                curves, linear, title=f'Handling diagram, {controller.upper()}, {speed:g} m/s'
            )
            fig.savefig(plots / f'handling_diagram_{controller}.png', dpi=110)
    measured = {}
    for (variant, key), curve in figures['curves'].items():
        if key.startswith('off@'):
            low = (abs(curve['ay_g']) > 0.05) & (abs(curve['ay_g']) < 0.2)
            if low.sum() > 5:
                measured.setdefault(variant, []).append(
                    (float(key.split('@')[1]), float(np.nanmedian(curve['yaw_gain_per_s'][low])))
                )
    linear_gain = {
        v: [(g['speed_ms'], g['linear_per_s']) for g in e['yaw_gain']] for v, e in report['variants'].items()
    }
    yaw_gain_figure(linear_gain, measured).savefig(plots / 'yaw_gain.png', dpi=110)
    gg_figure(figures['gg']).savefig(plots / 'gg_diagram.png', dpi=110)
    pp = build_runtime(resolve(next(iter(variants.values())))).validation.handling.phase_plane
    phase_plane_figure(figures['phase'], speed_ms=pp.speed_ms, steer_deg=pp.steer_deg).savefig(
        plots / 'phase_plane.png', dpi=110
    )
    (output / 'handling.json').write_text(json.dumps(report, indent=2, allow_nan=False, default=float) + '\n')
    write_handling_summary(output, report)
    print(
        json.dumps(
            {
                'output': str(output),
                'variants': {
                    v: {
                        'K_linear_deg_per_g': round(e['linear']['understeer_gradient_deg_per_g'], 3),
                        'classification': e['linear']['classification'],
                    }
                    for v, e in report['variants'].items()
                },
            }
        )
    )
    return 0


def write_handling_summary(output: Path, report: dict) -> None:
    lines = [
        '# Self-steering characteristic and handling limits',
        '',
        'SYNTHETIC SIMULATION – not a vehicle measurement. Definitions: docs/model/handling.md.',
        '',
        '| Variant | K linear model [deg/g] | v_char / v_crit [m/s] | Controller @ speed | K simulated, 0.05–0.4 g '
        '[deg/g] | a_y,max [g] | Limit behaviour |',
        '|---|---:|---|---|---:|---:|---|',
    ]
    for name, e in report['variants'].items():
        lin = e['linear']
        speed = (
            f"v_char {lin['characteristic_speed_ms']:.1f}"
            if lin['characteristic_speed_ms']
            else f"v_crit {lin['critical_speed_ms']:.1f}" if lin['critical_speed_ms'] else '–'
        )
        for key, r in e['ramp_steer'].items():
            if r.get('status') != 'PASS':
                lines.append(
                    f"| {name} | {lin['understeer_gradient_deg_per_g']:+.2f} | {speed} | {key} | – | – | "
                    f"FAIL_CLOSED: {r.get('reason', '')[:60]} |"
                )
                continue
            k = r['linear_gradient_deg_per_g']
            lines.append(
                f"| {name} | {lin['understeer_gradient_deg_per_g']:+.2f} | {speed} | {key} m/s | "
                f"{'–' if k is None else f'{k:+.2f}'} | {r['max_lateral_acceleration_g']:.2f} | "
                f"{r['limit_behaviour']} |"
            )
        stable = sum(p['outcome'] == 'stable' for p in e['phase_plane'])
        lines.append(f"| {name} | | | phase plane | | | {stable}/{len(e['phase_plane'])} start states stable |")
    lines += [
        '',
        'Plots: `plots/handling_diagram_<controller>.png`, `plots/yaw_gain.png`, `plots/gg_diagram.png`, '
        '`plots/phase_plane.png`. Telemetry CSV per run: `<variant>/ramp_steer_<controller>_<speed>ms.csv`.',
    ]
    (output / 'SUMMARY.md').write_text('\n'.join(lines) + '\n')


def plot_tyre(args):
    from torquevector.cli.composition import build_tyre
    from torquevector.infrastructure.config.resolver import resolve_component
    from torquevector.visualization.report import tyre_figure

    path = Path(args.tyre) if args.tyre else Path(resolve_from_args(args).include_files['tyre'])
    _, section = resolve_component(path, template_ok=False)
    tyre, model_name, _, model = build_tyre(section)
    output = Path(args.output).expanduser().resolve() if args.output else results_base() / 'tyre_plots'
    output.mkdir(parents=True, exist_ok=True)
    target = output / f"{section['id']}_characteristics.{args.format}"
    figure = tyre_figure(tyre, model, f"{section['id']} ({model_name})")
    figure.savefig(target, dpi=args.dpi)
    import matplotlib.pyplot as plt

    plt.close(figure)
    LOG.info('Reifenkennlinien: %s', target)
    print(target)
    return 0


def optimize(args):
    from torquevector.cli.composition import run_optimization

    return run_optimization(
        args.config, study_overrides=parse_sets(args.set), results_dir=args.results_dir, resume=args.resume
    )


def _file_kind(path):
    from torquevector.infrastructure.config.yaml_io import load_with_origins

    data, _ = load_with_origins(path)
    for key in ('application', 'profile', 'pso', 'tuning'):
        if key in data:
            return key
    return 'component'


def config_command(args):
    from torquevector.infrastructure.config.resolver import resolve_component, resolve_study

    path = base_config(args)
    kind = _file_kind(path)
    if kind in ('application', 'profile'):
        resolved = resolve_from_args(args)
        if args.action == 'validate':
            from torquevector.cli.composition import build_runtime

            build_runtime(resolved)
            for warning in resolved.warnings:
                LOG.warning(warning)
            print(
                json.dumps(
                    {
                        'status': 'VALID',
                        'kind': kind,
                        'fingerprint': resolved.fingerprint,
                        'files': len(resolved.sources),
                    }
                )
            )
            return 0
        text = (
            resolved.export_yaml()
            if args.format == 'yaml'
            else json.dumps({'fingerprint': resolved.fingerprint, 'tree': resolved.plain()}, indent=2)
        )
        if args.origins:
            text += '\n# Herkunft je Parameter\n' + ''.join(
                f'# {k}: {v}\n' for k, v in sorted(resolved.origins.items())
            )
        if args.output:
            Path(args.output).write_text(text, encoding='utf-8')
            print(args.output)
        else:
            print(text, end='')
        return 0
    if kind in ('pso', 'tuning'):
        definition = resolve_study(path, overrides=parse_sets(args.set))
        print(
            json.dumps(
                {
                    'status': 'VALID',
                    'kind': kind,
                    'study': definition.study['id'],
                    'fingerprint': definition.fingerprint,
                }
            )
        )
        return 0
    component, result = resolve_component(path)
    if args.action == 'show':
        from torquevector.infrastructure.config.yaml_io import dump

        print(dump({'schema_version': 1, component.kind: result}), end='')
    else:
        print(
            json.dumps(
                {
                    'status': 'VALID_TEMPLATE_STRUCTURE' if component.template else 'VALID',
                    'kind': component.kind,
                    'files': [str(f) for f in component.files],
                }
            )
        )
    return 0


def validate(args):
    from torquevector.evaluation.test_runner import run_tests

    return run_tests(args.suite, Path(args.results_dir).resolve() if args.results_dir else results_base() / 'tests')


def dashboard(args):
    import matplotlib.pyplot as plt
    from math import atan, degrees
    from torquevector.cli.composition import build_runtime
    from torquevector.cli.dashboard import dashboard as create
    from torquevector.domain.controls import DriverControls

    runtime = build_runtime(resolve_from_args(args))
    spec, car, tyre = runtime.scenario.spec, runtime.vehicle, runtime.tyre
    live = runtime.visualization.live
    interactive = live.enabled and not args.no_gui
    steer = degrees(atan(car.wheelbase_m / spec.radius_m)) if spec.kind.startswith('circle') else 0.0
    fig, _ = create(
        car,
        tyre,
        DriverControls(steer_deg=steer),
        spec.speed_ms,
        runtime.scenario.duration_s,
        interactive=interactive,
        dt=live.dt_s,
        figure_size_in=live.figure_size_in,
    )
    if args.output:
        fig.savefig(args.output)
        LOG.info('Dashboard gespeichert: %s', args.output)
    if interactive:
        plt.show()
    else:
        plt.close(fig)
    return 0


def _config_options(p, *, components=True):
    p.add_argument('--config', help='application or profile YAML (default: configs/default.yaml)')
    p.add_argument('--profile', action='append', help='profile overlay; repeatable, applied in order')
    p.add_argument(
        '--set', action='append', metavar='PATH=VALUE', help='override a parameter, e.g. vehicle.mass_kg=1700'
    )
    if components:
        for name in COMPONENT_FLAGS:
            p.add_argument(f'--{name}', metavar='YAML', help=f'replace the {name} file')


def _run_options(p):
    p.add_argument('--scenario', help='scenario YAML or name in configs/scenarios/ (e.g. double_lane_change)')
    p.add_argument('--duration', type=float, help='simulated time [s] (the manoeuvre definition is unchanged)')
    p.add_argument('--speed-ms', type=float, help='target/initial speed [m/s]')
    p.add_argument('--results-dir', help='results root (default: results/)')
    p.add_argument('--run-name', help='exact subfolder name under results-dir (default: <timestamp>_<scenario>)')
    group = p.add_mutually_exclusive_group()
    group.add_argument('--report', action='store_true', help='always write the PDF report')
    group.add_argument('--no-report', action='store_true', help='do not write the automatic report')
    _config_options(p)


def _render_options(p):
    p.add_argument('--run-id', help='run ID or latest (default: latest)')
    p.add_argument('--result', help='run folder (alternative to --run-id)')
    p.add_argument('--format', choices=('report', 'png', 'svg', 'gif', 'road-gif'), default='report')
    p.add_argument('--controller', help='controller for single-controller views (road-gif)')
    p.add_argument('--case', help='case ID of a matrix run')
    p.add_argument('--sections', help='report sections, comma-separated (e.g. kpi,allocation,road_map)')
    p.add_argument('--results-dir')
    p.add_argument('--fps', type=int)
    p.add_argument('--max-frames', type=int)
    p.add_argument('--dpi', type=int)
    p.set_defaults(func=render_run)


def build_parser():
    parser = argparse.ArgumentParser(
        prog='torquevector',
        description='Four-wheel vehicle dynamics and torque vectoring (P/LQR/LTV-MPC/NMPC). SYNTHETIC / NOT VEHICLE VALIDATED.',
        epilog='Beispiel: torquevector simulate --scenario double_lane_change --controllers off lqr nmpc',
    )
    parser.add_argument(
        '--log-level',
        default='INFO',
        choices=('DEBUG', 'INFO', 'WARNING', 'ERROR'),
        help='console log level (the log file is always DEBUG)',
    )
    parser.add_argument('--log-dir', help='log directory (default: ./logs or TORQUEVECTOR_LOG_DIR)')
    parser.add_argument('--quiet', action='store_true', help='no log output on the console')
    sub = parser.add_subparsers(dest='command', required=True, metavar='COMMAND')

    p = sub.add_parser('simulate', help='simulate a scenario (controllers from the configuration or --controller)')
    _run_options(p)
    p.add_argument('--controller', help='off/p/lqr/ltv/nmpc or a controller YAML')
    p.set_defaults(func=run_simulation, controllers=None)

    p = sub.add_parser('compare', help='compare several controllers on the same scenario')
    _run_options(p)
    p.add_argument(
        '--controllers', choices=CONTROLLERS, nargs='+', help='default: all controllers of the configuration'
    )
    p.set_defaults(func=run_simulation, controller=None)

    for name in ('render', 'visualize'):
        p = sub.add_parser(name, help='plots/GIF/report from stored telemetry (no simulation)')
        _render_options(p)

    p = sub.add_parser(
        'multi-mu', help='run the four multi-mu road scenarios into one collection folder (report + GIF per scenario)'
    )
    p.add_argument('--output', default='results/multi_mu')
    p.add_argument(
        '--controllers', choices=CONTROLLERS, nargs='+', help='default: all controllers of the configuration'
    )
    p.add_argument('--gif-controller', help='controller shown in the GIF (default: first active controller)')
    _config_options(p)
    p.set_defaults(func=multi_mu, controller=None, scenario=None)

    p = sub.add_parser('q11', help='torque-vectoring assessment (Q11): nominal, robustness and degradation sets')
    p.add_argument('--output', default='results/q11')
    p.add_argument(
        '--controllers', choices=CONTROLLERS, nargs='+', help='default: all controllers of the configuration'
    )
    _config_options(p)
    p.set_defaults(func=q11, controller=None, scenario=None)

    p = sub.add_parser('plot-tyre', help='plot the curves of a tyre YAML (no simulation)')
    p.add_argument('--tyre', help='tyre YAML (default: tyre of the configuration)')
    p.add_argument('--output', help='output folder (default: results/tyre_plots)')
    p.add_argument('--format', choices=('png', 'svg', 'pdf'), default='png')
    p.add_argument('--dpi', type=int, default=130)
    p.add_argument('--results-dir')
    _config_options(p, components=False)
    p.set_defaults(func=plot_tyre, scenario=None, controller=None)

    p = sub.add_parser('optimize', help='offline PSO tuning (pso.yaml or *_tuning.yaml)')
    p.add_argument('--config', required=True)
    p.add_argument('--set', action='append', metavar='tuning.PATH=VALUE')
    p.add_argument('--results-dir')
    p.add_argument('--resume', metavar='RESULT_FOLDER')
    p.set_defaults(func=optimize)

    p = sub.add_parser('config', help='validate a configuration or show/export it resolved')
    actions = p.add_subparsers(dest='action', required=True)
    for action in ('validate', 'show'):
        q = actions.add_parser(action)
        _config_options(q)
        q.set_defaults(func=config_command, scenario=None, controller=None)
        if action == 'show':
            q.add_argument('--format', choices=('yaml', 'json'), default='yaml')
            q.add_argument('--origins', action='store_true')
            q.add_argument('--output')
        else:
            q.set_defaults(origins=False, output=None, format='yaml')

    p = sub.add_parser('validate', help='run the test suite (development checkout)')
    p.add_argument('--suite', choices=('smoke', 'full'), default='smoke')
    p.add_argument('--results-dir')
    p.set_defaults(func=validate)

    p = sub.add_parser('handling', help='self-steering characteristic (understeer/oversteer) and handling limits')
    p.add_argument('--output', default='results/handling')
    p.add_argument(
        '--variants',
        nargs='+',
        metavar='NAME=YAML',
        help='vehicle variants (default: reference, understeer and oversteer profiles)',
    )
    p.add_argument('--controllers', choices=CONTROLLERS, nargs='+', help='default: validation.handling.controllers')
    p.add_argument('--diagram-speed', type=float, default=20.0, help='speed of the handling-diagram plots [m/s]')
    p.add_argument('--flat', action='store_true', help='write directly to --output without a timestamp subfolder')
    p.set_defaults(func=handling)

    p = sub.add_parser(
        'quality-audit', help='quality gates Q01-Q13: gate tests, evidence and a PROPOSED scorecard (needs review)'
    )
    p.add_argument('--output', default='results/quality_audit')
    p.add_argument('--repository', default='.', help='development checkout containing tests/quality and docs/quality')
    p.add_argument('--coverage', choices=('run', 'skip'), default='run', help='measure core coverage (slow)')
    p.add_argument('--coverage-report', help='use an existing coverage report text (with --coverage skip)')
    p.set_defaults(func=quality_audit)

    p = sub.add_parser('dashboard', help='interactive dashboard (sliders, friction circles)')
    p.add_argument('--scenario')
    p.add_argument('--duration', type=float)
    p.add_argument('--no-gui', action='store_true')
    p.add_argument('--output')
    _config_options(p)
    p.set_defaults(func=dashboard, controller=None)
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    from torquevector.infrastructure.logging_setup import configure_logging

    log_file = configure_logging(args.command, level=args.log_level, log_dir=args.log_dir, console=not args.quiet)
    LOG.debug('Aufruf: torquevector %s', ' '.join(sys.argv[1:] if argv is None else argv))
    try:
        code = args.func(args)
    except (ValueError, RuntimeError, FileNotFoundError) as exc:
        LOG.error('%s: %s', type(exc).__name__, exc)
        LOG.info('Details in %s', log_file)
        if args.quiet:
            print(f'{type(exc).__name__}: {exc}', file=sys.stderr)
        return 1
    LOG.debug('Exit-Code %s', code)
    return code


if __name__ == '__main__':
    raise SystemExit(main())
