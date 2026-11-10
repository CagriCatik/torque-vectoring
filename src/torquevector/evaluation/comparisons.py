"""Controller comparison matrix over preset manoeuvres with fail-closed status handling."""

from __future__ import annotations
from torquevector.domain.errors import OutsideEnvelopeError
from torquevector.simulation.runner import simulate_controller
from dataclasses import replace
from torquevector.domain.scenarios import presets
from torquevector.control.controllers.nmpc import NMPCConfig
from torquevector.control.factory import CONTROLLERS
from torquevector.evaluation.envelope import LEGACY_THRESHOLDS
from torquevector.evaluation.envelope import envelope_status

FAILURES = (RuntimeError, ValueError, ArithmeticError, OverflowError, OutsideEnvelopeError)


def assess(kind, spec, duration, config):
    try:
        m, seq = simulate_controller(kind, spec, duration, nmpc_config=config)
        return {'status': envelope_status(m, spec.speed_ms, LEGACY_THRESHOLDS), 'metrics': m}, seq
    except FAILURES as exc:
        return {'status': 'FAIL_CLOSED', 'reason': str(exc)}, None


def assess_run(run, target_speed_ms, thresholds, check_final_speed=True):
    """Gemeinsame Statuslogik für konfigurierte Läufe; ``run`` liefert (Kennzahlen, Telemetrie, Diagnostik)."""
    try:
        m, seq, diagnostics = run()
    except FAILURES as exc:
        return {'status': 'FAIL_CLOSED', 'reason': str(exc)}, None, []
    return (
        {'status': envelope_status(m, target_speed_ms, thresholds, check_final_speed), 'metrics': m},
        seq,
        diagnostics,
    )


def compare(cases, controllers=CONTROLLERS, config=NMPCConfig(), *, collect_samples=False):
    records = []
    evidence = {}
    for scenario, speed, duration in cases:
        spec = replace(presets()[scenario], speed_ms=speed)
        row = {'scenario': scenario, 'speed_ms': speed, 'duration_s': duration, 'controllers': {}}
        for kind in controllers:
            entry, seq = assess(kind, spec, duration, config)
            row['controllers'][kind] = entry
            if collect_samples and seq is not None:
                evidence.setdefault(scenario, {})[kind] = seq
        annotate_relative(row['controllers'])
        records.append(row)
    return records, evidence


def annotate_relative(entries):
    """Shared comparison bookkeeping, independent of simulation ownership."""
    baseline = entries.get('off', {})
    if baseline.get('status') == 'PASS':
        b = baseline['metrics']
        for kind, entry in entries.items():
            if kind == 'off' or entry['status'] != 'PASS':
                continue
            m = entry['metrics']
            entry['vs_off'] = {
                'yaw_improvement_pct': 100 * (1 - m['yaw_rmse_rads'] / max(b['yaw_rmse_rads'], 1e-9)),
                'path_improvement_pct': 100 * (1 - m['cross_rmse_m'] / max(b['cross_rmse_m'], 1e-9)),
            }
