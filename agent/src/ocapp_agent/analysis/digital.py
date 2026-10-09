"""Bitcrush detection: quantization combs and sample-rate reduction (holds)."""
from __future__ import annotations

import numpy as np

from .common import TARGET_SR, validate_audio
from ..models import BitcrushModel

#: Above this many distinct levels, assume full-precision audio.
MAX_LEVELS = 65536


def _estimate_bit_depth(wet: np.ndarray) -> int | None:
    """Bit depth from the occupied quantization levels of ``wet``.

    Quantized values sit on a uniform grid; the minimum gap between distinct
    levels is the step size (full scale assumed +/-1, span 2). Returns None
    when the values do not form a uniform grid.
    """
    u = np.unique(np.round(wet, 6))
    if u.size < 3 or u.size > MAX_LEVELS:
        return None
    gaps = np.diff(u)
    step = float(gaps.min())
    if step <= 1e-5:
        return None
    # All gaps must be integer multiples of the step (levels may be skipped).
    mult = gaps / step
    if float(np.max(np.abs(mult - np.round(mult)))) > 0.05:
        return None
    depth = int(np.clip(round(np.log2(2.0 / step)), 1, 16))
    return depth


def _estimate_hold_factor(wet: np.ndarray) -> int:
    """Zero-order-hold factor from runs of adjacent-equal samples.

    Quantization alone produces short runs of equal values (a geometric run
    distribution with high variance), while S&H downsampling produces runs of
    a consistent length k. Detect via run-length structure; 1 = no reduction.
    """
    scale = float(np.max(np.abs(wet))) or 1.0
    tol = max(1e-9, 1e-5 * scale)
    changes = np.diff(wet)
    # Run lengths of (near-)equal samples.
    bounds = np.flatnonzero(np.abs(changes) >= tol)
    runs = np.diff(np.concatenate(([0], bounds + 1, [wet.size])))
    runs = runs[runs > 0]
    if runs.size < 8:
        return 1
    mean_run = float(runs.mean())
    if mean_run < 1.8:
        return 1
    cov = float(runs.std()) / mean_run
    if cov > 0.6:  # geometric-ish: just quantization, not a hold
        return 1
    factor = int(round(float(np.median(runs))))
    return int(np.clip(factor, 2, 64))


def measure_bitcrush(
    dry: np.ndarray, wet: np.ndarray, sr: int = TARGET_SR
) -> BitcrushModel | None:
    """Detect quantization and/or sample-rate reduction in wet.

    ``bit_depth`` comes from the histogram/level comb (uniform quantization
    grid); ``downsample`` comes from zero-order-hold runs (equivalently: the
    aliasing structure that S&H downsampling creates). Returns None when
    there is no evidence of either.
    """
    w = validate_audio(wet, "wet")
    validate_audio(dry, "dry")  # pair consistency; dry itself is unused here

    depth = _estimate_bit_depth(w)
    hold = _estimate_hold_factor(w)

    if depth is None and hold == 1:
        return None
    return BitcrushModel(
        bit_depth=depth if depth is not None else 16,
        downsample=hold,
    )
