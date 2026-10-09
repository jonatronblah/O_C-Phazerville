"""ocapp-agent: clone an audio effect from a dry/wet pair into a Phazerville applet.

The shared schema contract lives in :mod:`ocapp_agent.models`; it is the interface
between the analysis, verify, and codegen lanes.
"""
from ocapp_agent.models import (
    AnalysisReport,
    ChainModel,
    CloneResult,
    FitReport,
)

__all__ = [
    "AnalysisReport",
    "ChainModel",
    "CloneResult",
    "FitReport",
]
