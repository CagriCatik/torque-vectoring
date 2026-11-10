"""Gemeinsame Hilfen der Road-Integrationstests."""

from pathlib import Path
from torquevector.cli.composition import build_runtime, evaluate_case, road_contact_rows, road_summary
from torquevector.infrastructure.config.resolver import resolve

ROOT = Path(__file__).parents[3]
CONFIGS = ROOT / 'configs'


def runtime(scenario, overrides=(), replacements=None):
    reps = {'scenario': str(CONFIGS / f'scenarios/{scenario}.yaml'), **(replacements or {})}
    return build_runtime(resolve(CONFIGS / 'default.yaml', replacements=reps, overrides=list(overrides)))


def run(scenario, kind='off', overrides=(), replacements=None):
    rt = runtime(scenario, overrides, replacements)
    entry, seq, ticks = evaluate_case(rt, kind)
    contacts = road_contact_rows(rt, seq) if seq else []
    if seq:
        entry['metrics']['road'] = road_summary(rt, seq, contacts, ticks)  # wie run_configured
    return rt, entry, seq, contacts, ticks
