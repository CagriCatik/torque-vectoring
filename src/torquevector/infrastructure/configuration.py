"""Versioned packaged defaults; no dependence on the caller's cwd."""

import json
from importlib.resources import files
from pathlib import Path
from torquevector.calibration.surrogate import fitted_tyre_from_report


def resource_path(relative):
    """Filesystem resources in installed/editable distributions."""
    return Path(str(files('torquevector').joinpath('resources', relative)))


def load_model():
    return fitted_tyre_from_report(json.loads(resource_path('synthetic/fit_report.json').read_text()))
