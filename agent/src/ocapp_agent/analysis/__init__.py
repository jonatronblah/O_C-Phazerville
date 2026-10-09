"""Deterministic (no-LLM) analysis of a dry/wet audio pair.

Public surface — sibling lanes code against exactly these names:

- :data:`TARGET_SR` (48000)
- :func:`load_audio`
- :func:`align_pair`
- :func:`match_loudness`
- :func:`detect_parallel_mix`
- :func:`measure_eq`
- :func:`measure_delay`
- :func:`measure_reverb`
- :func:`measure_dynamics`
- :func:`measure_waveshaper`
- :func:`measure_bitcrush`
- :func:`analyze_pair`

All functions are pure/deterministic given the same inputs. Estimation is
best-effort: each ``measure_*`` returns ``None`` (or an empty list for EQ)
when there is no evidence for that stage.
"""
from __future__ import annotations

from .alignment import align_pair, detect_parallel_mix, parallel_mix_score
from .audio import load_audio, match_loudness
from .common import TARGET_SR, apply_biquads, design_biquad
from .digital import measure_bitcrush
from .dynamics import measure_dynamics
from .ir import estimate_ir, measure_delay, measure_reverb
from .nonlinear import measure_waveshaper
from .pipeline import analyze_pair
from .spectral import measure_eq, stages_response_db, transfer_function_db

__all__ = [
    "TARGET_SR",
    "load_audio",
    "align_pair",
    "match_loudness",
    "detect_parallel_mix",
    "parallel_mix_score",
    "measure_eq",
    "measure_delay",
    "measure_reverb",
    "measure_dynamics",
    "measure_waveshaper",
    "measure_bitcrush",
    "analyze_pair",
    "estimate_ir",
    "transfer_function_db",
    "stages_response_db",
    "design_biquad",
    "apply_biquads",
]
