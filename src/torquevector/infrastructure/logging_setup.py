"""Zentrales Logging: Konsole (stderr) plus Logdatei je Aufruf unter ``logs/``.

Fachmodule verwenden nur ``logging.getLogger(__name__)``; Handler und Dateien
konfiguriert ausschließlich die CLI über :func:`configure_logging`. Zusätzlich
schreibt :func:`run_log` während eines Laufs eine Kopie nach ``results/<run_id>/logs/run.log``.
"""

from __future__ import annotations

import logging
import os
import sys
from contextlib import contextmanager
from datetime import datetime
from pathlib import Path

ROOT_LOGGER = 'torquevector'
FILE_FORMAT = '%(asctime)s.%(msecs)03d %(levelname)-7s %(name)s: %(message)s'
CONSOLE_FORMAT = '%(levelname)-7s %(message)s'
DATE_FORMAT = '%Y-%m-%d %H:%M:%S'


def log_directory(value: str | None = None) -> Path:
    """Priorität: --log-dir > TORQUEVECTOR_LOG_DIR > ./logs."""
    return Path(value or os.environ.get('TORQUEVECTOR_LOG_DIR') or Path.cwd() / 'logs').expanduser().resolve()


def configure_logging(command: str, *, level: str = 'INFO', log_dir: str | None = None, console: bool = True) -> Path:
    """Richtet Konsole und Logdatei ein (idempotent) und liefert den Pfad der Logdatei."""
    directory = log_directory(log_dir)
    directory.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    path = directory / f'{stamp}_{command}.log'
    logger = logging.getLogger(ROOT_LOGGER)
    logger.setLevel(logging.DEBUG)
    logger.propagate = False
    for handler in list(logger.handlers):
        logger.removeHandler(handler)
        handler.close()
    file_handler = logging.FileHandler(path, encoding='utf-8')
    file_handler.setLevel(logging.DEBUG)
    file_handler.setFormatter(logging.Formatter(FILE_FORMAT, DATE_FORMAT))
    logger.addHandler(file_handler)
    if console:
        stream = logging.StreamHandler(sys.stderr)
        stream.setLevel(getattr(logging, level.upper()))
        stream.setFormatter(logging.Formatter(CONSOLE_FORMAT))
        logger.addHandler(stream)
    logging.captureWarnings(True)
    warnings_logger = logging.getLogger('py.warnings')
    for handler in list(warnings_logger.handlers):
        warnings_logger.removeHandler(handler)
    warnings_logger.addHandler(file_handler)
    warnings_logger.propagate = False
    logger.debug('Logging gestartet: Befehl=%s Datei=%s Python=%s', command, path, sys.version.split()[0])
    return path


@contextmanager
def run_log(directory: Path):
    """Während eines Laufs zusätzlich nach ``<run>/logs/run.log`` protokollieren."""
    directory.mkdir(parents=True, exist_ok=True)
    handler = logging.FileHandler(directory / 'run.log', encoding='utf-8')
    handler.setLevel(logging.DEBUG)
    handler.setFormatter(logging.Formatter(FILE_FORMAT, DATE_FORMAT))
    logger = logging.getLogger(ROOT_LOGGER)
    logger.addHandler(handler)
    try:
        yield directory / 'run.log'
    finally:
        logger.removeHandler(handler)
        handler.close()
