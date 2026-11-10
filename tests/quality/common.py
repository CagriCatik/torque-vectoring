"""Gemeinsame Bausteine der Qualitäts-Gate-Suite (eine Testmethode je Check, Namensschema test_Qxx_y_*).

Jede Methode trägt im Docstring die Check-ID und das Akzeptanzmaß aus docs/quality/QUALITY_GATES.md.
Checks, die ohne reale Mess-/Benchmarkdaten nicht voll erfüllbar sind (Blocker B1/B5), führt
``torquevector.evaluation.quality_audit.DATA_LIMITED``; diese erhalten höchstens 1 Punkt.
"""

from functools import lru_cache
from pathlib import Path

ROOT = Path(__file__).parents[2]
CONFIGS = ROOT / 'configs'


@lru_cache(maxsize=None)
def runtime(scenario=None, overrides=()):
    from torquevector.cli.composition import build_runtime
    from torquevector.infrastructure.config.resolver import resolve

    reps = {'scenario': str(CONFIGS / f'scenarios/{scenario}.yaml')} if scenario else None
    return build_runtime(resolve(CONFIGS / 'default.yaml', replacements=reps, overrides=list(overrides)))


def run_case(scenario, kind='off', overrides=()):
    from torquevector.cli.composition import evaluate_case

    rt = runtime(scenario, tuple(overrides))
    entry, seq, ticks = evaluate_case(rt, kind)
    return rt, entry, seq, ticks


def nrmse(sim, ref, floor):
    import numpy as np

    sim, ref = np.asarray(sim, float), np.asarray(ref, float)
    scale = max(floor, float(np.quantile(np.abs(ref), 0.95)))
    return float(np.sqrt(np.mean((sim - ref) ** 2)) / scale)
