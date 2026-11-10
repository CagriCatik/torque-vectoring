"""Domain exceptions; ``OutsideEnvelopeError`` marks states outside the validated model envelope (fail closed)."""

from __future__ import annotations


class OutsideEnvelopeError(ValueError):
    """State/command is outside the modeled domain, not a converged solution."""
