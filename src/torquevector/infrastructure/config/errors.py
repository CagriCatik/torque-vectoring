"""Gesammelte, auf YAML-Herkunft zurückführbare Konfigurationsfehler."""

from __future__ import annotations


class ConfigurationError(ValueError):
    """Eine oder mehrere Konfigurationsverletzungen; ``issues`` bleibt maschinenlesbar."""

    def __init__(self, issues):
        self.issues = tuple(issues) if not isinstance(issues, str) else (issues,)
        if not self.issues:
            raise ValueError('ConfigurationError requires at least one issue')
        lines = '\n'.join(f'  - {issue}' for issue in self.issues)
        super().__init__(f'{len(self.issues)} Konfigurationsfehler:\n{lines}')
