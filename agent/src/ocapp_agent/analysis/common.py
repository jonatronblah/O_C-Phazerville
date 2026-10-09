"""Shared helpers for the analysis package: validation, dB math, RBJ biquads.

All estimators work internally on 1-D float64 arrays for numerical stability;
public loaders return float32 (the firmware's interchange format).
"""
from __future__ import annotations

import numpy as np

#: The firmware's sample rate. All analysis happens at this rate.
TARGET_SR = 48000

#: Signals quieter than this (rms, ~ -120 dBFS) are treated as silent.
MIN_RMS = 1e-6

_EPS = 1e-30


def rms(x: np.ndarray) -> float:
    """Root-mean-square level of ``x`` (float64 safe)."""
    x = np.asarray(x, dtype=np.float64)
    return float(np.sqrt(np.mean(x * x))) if x.size else 0.0


def rms_db(x: np.ndarray) -> float:
    """RMS level in dBFS-ish units (dB re. full scale = 1.0)."""
    return float(20.0 * np.log10(rms(x) + _EPS))


def to_db(x: np.ndarray | float) -> np.ndarray | float:
    return 20.0 * np.log10(np.maximum(x, _EPS))


def validate_audio(x: np.ndarray, name: str = "signal", *, min_rms: float = MIN_RMS) -> np.ndarray:
    """Return ``x`` as a clean 1-D float64 array.

    Accepts 1-D or 2-D input (2-D is averaged down to mono along the
    channel axis). Raises :class:`ValueError` for empty input, non-finite
    samples (NaN/inf), or near-silent content.
    """
    arr = np.asarray(x)
    if arr.size == 0:
        raise ValueError(f"{name} is empty")
    if arr.ndim > 2:
        raise ValueError(f"{name} must be 1-D or 2-D, got shape {arr.shape}")
    if arr.ndim == 2:
        # Assume the shorter axis holds channels: (n, ch) or (ch, n).
        axis = 1 if arr.shape[1] <= arr.shape[0] else 0
        arr = arr.mean(axis=axis)
    try:
        arr = np.asarray(arr, dtype=np.float64).ravel()
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be numeric") from exc
    if not np.all(np.isfinite(arr)):
        raise ValueError(f"{name} contains NaN or infinite samples")
    level = rms(arr)
    if not np.isfinite(level):
        raise ValueError(f"{name} has non-finite energy")
    if level < min_rms:
        raise ValueError(f"{name} is silent (rms={level:.3g}); cannot estimate effects")
    return arr


def biquad_response_db(b: np.ndarray, a: np.ndarray, freqs: np.ndarray, sr: int) -> np.ndarray:
    """Magnitude response in dB of one biquad at ``freqs`` (vectorized)."""
    w = 2.0 * np.pi * np.asarray(freqs, dtype=np.float64) / sr
    zj = np.exp(-1j * w)
    num = b[0] + b[1] * zj + b[2] * zj * zj
    den = a[0] + a[1] * zj + a[2] * zj * zj
    return to_db(np.abs(num / den))


def design_biquad(
    kind: str,
    freq_hz: float,
    q: float = 0.707,
    gain_db: float = 0.0,
    sr: int = TARGET_SR,
) -> tuple[np.ndarray, np.ndarray]:
    """Design a biquad (RBJ Audio-EQ-Cookbook formulas).

    Returns ``(b, a)`` with ``a[0] == 1``. Supported kinds match
    :class:`ocapp_agent.models.BiquadStage`.
    """
    if not np.isfinite(freq_hz) or not (0.0 < freq_hz < sr / 2.0):
        raise ValueError(f"freq_hz must be in (0, {sr / 2}), got {freq_hz}")
    if not np.isfinite(q) or q <= 0.0:
        raise ValueError(f"q must be > 0, got {q}")
    w0 = 2.0 * np.pi * freq_hz / sr
    cos_w0 = np.cos(w0)
    sin_w0 = np.sin(w0)
    alpha = sin_w0 / (2.0 * q)
    a_lin = 10.0 ** (gain_db / 40.0)

    if kind == "peaking":
        b = np.array([1.0 + alpha * a_lin, -2.0 * cos_w0, 1.0 - alpha * a_lin])
        a = np.array([1.0 + alpha / a_lin, -2.0 * cos_w0, 1.0 - alpha / a_lin])
    elif kind == "lowshelf":
        sa = 2.0 * np.sqrt(a_lin) * alpha
        b = np.array([
            a_lin * ((a_lin + 1.0) + (a_lin - 1.0) * cos_w0 + sa),
            -2.0 * a_lin * ((a_lin - 1.0) + (a_lin + 1.0) * cos_w0),
            a_lin * ((a_lin + 1.0) + (a_lin - 1.0) * cos_w0 - sa),
        ])
        a = np.array([
            (a_lin + 1.0) - (a_lin - 1.0) * cos_w0 + sa,
            2.0 * ((a_lin - 1.0) - (a_lin + 1.0) * cos_w0),
            (a_lin + 1.0) - (a_lin - 1.0) * cos_w0 - sa,
        ])
    elif kind == "highshelf":
        sa = 2.0 * np.sqrt(a_lin) * alpha
        b = np.array([
            a_lin * ((a_lin + 1.0) - (a_lin - 1.0) * cos_w0 + sa),
            2.0 * a_lin * ((a_lin - 1.0) - (a_lin + 1.0) * cos_w0),
            a_lin * ((a_lin + 1.0) - (a_lin - 1.0) * cos_w0 - sa),
        ])
        a = np.array([
            (a_lin + 1.0) + (a_lin - 1.0) * cos_w0 + sa,
            -2.0 * ((a_lin - 1.0) + (a_lin + 1.0) * cos_w0),
            (a_lin + 1.0) + (a_lin - 1.0) * cos_w0 - sa,
        ])
    elif kind == "lowpass":
        b = np.array([(1.0 - cos_w0) / 2.0, 1.0 - cos_w0, (1.0 - cos_w0) / 2.0])
        a = np.array([1.0 + alpha, -2.0 * cos_w0, 1.0 - alpha])
    elif kind == "highpass":
        b = np.array([(1.0 + cos_w0) / 2.0, -(1.0 + cos_w0), (1.0 + cos_w0) / 2.0])
        a = np.array([1.0 + alpha, -2.0 * cos_w0, 1.0 - alpha])
    elif kind == "bandpass":
        b = np.array([alpha, 0.0, -alpha])
        a = np.array([1.0 + alpha, -2.0 * cos_w0, 1.0 - alpha])
    elif kind == "notch":
        b = np.array([1.0, -2.0 * cos_w0, 1.0])
        a = np.array([1.0 + alpha, -2.0 * cos_w0, 1.0 - alpha])
    else:
        raise ValueError(f"unknown biquad kind: {kind!r}")
    return b / a[0], a / a[0]


def apply_biquads(x: np.ndarray, stages, sr: int = TARGET_SR) -> np.ndarray:
    """Run ``x`` through a series of :class:`BiquadStage`-like objects."""
    from scipy import signal

    y = np.asarray(x, dtype=np.float64)
    for stage in stages:
        b, a = design_biquad(
            stage.kind, stage.freq_hz, stage.q, stage.gain_db, sr
        )
        y = signal.lfilter(b, a, y)
    return y
