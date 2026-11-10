"""YAML-Konfigurationssystem: Laden, Auflösen, Validieren, Einfrieren, Exportieren.

Nur diese Infrastruktur und die CLI-Composition-Root kennen YAML und Dateipfade.
Fachschichten erhalten ausschließlich typisierte, unveränderliche Objekte.
"""

from torquevector.infrastructure.config.errors import ConfigurationError
from torquevector.infrastructure.config.resolver import ResolvedConfiguration, resolve, resolve_component

__all__ = ['ConfigurationError', 'ResolvedConfiguration', 'resolve', 'resolve_component']
