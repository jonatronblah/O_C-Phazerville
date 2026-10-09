"""Cross-correlation alignment and parallel-mix detection."""
from __future__ import annotations

import numpy as np
from scipy import signal

from .common import TARGET_SR, validate_audio

#: Residual correlation above this flags an unprocessed dry blend.
PARALLEL_MIX_THRESHOLD = 0.5


def align_pair(dry: np.ndarray, wet: np.ndarray) -> tuple[np.ndarray, np.ndarray, int]:
    """Cross-correlation alignment; returns (dry_aligned, wet_aligned, lag_samples).

    The correlation is computed spectrally as a regularized deconvolution
    (equivalent to a whitened cross-correlation) so tonal material cannot lock
    onto period sidelobes of a plain cross-correlation. The lag is taken at the
    *direct path*: the significant IR peak (>= 20% of the peak) closest to zero
    lag — with mix < 1 the first echo can be louder than the direct tap, and
    anchoring on the largest peak would report the echo position as latency.
    A fully-wet delay (no direct path) is absorbed into the lag.

    ``lag_samples`` positive means wet is delayed relative to dry and that many
    samples were trimmed from the start of ``wet`` (negative: from the start of
    ``dry``). Both outputs are trimmed to a common length.
    """
    from .ir import estimate_ir, _significant_peaks

    d = validate_audio(dry, "dry")
    w = validate_audio(wet, "wet")

    ir = np.abs(estimate_ir(d, w, TARGET_SR))
    n = ir.size
    pmax = float(ir.max()) if n else 0.0

    def signed_lag(i: int) -> int:
        return i if i <= n // 2 else i - n

    lag = 0
    if pmax > 0.0:
        # Direct path: a peak within -14 dB of the dominant one, nearest zero.
        peaks = _significant_peaks(ir, TARGET_SR, pmax, rel=0.2)
        if not peaks:
            peaks = _significant_peaks(ir, TARGET_SR, pmax)
        if peaks:
            lag = signed_lag(min(peaks, key=lambda i: (abs(signed_lag(i)), i)))
        else:
            lag = signed_lag(int(np.argmax(ir)))

    if lag >= 0:
        w_a = w[lag:]
        d_a = d[: w_a.size]
    else:
        d_a = d[-lag:]
        w_a = w[: d_a.size]
    n_out = min(d_a.size, w_a.size)
    return d_a[:n_out], w_a[:n_out], int(lag)


def parallel_mix_score(dry: np.ndarray, wet: np.ndarray, sr: int = TARGET_SR) -> float:
    """How much dry (direct or delayed copies) survives in wet, in [0, 1]-ish.

    Removes the best short linear prediction of wet from dry (a 65-tap FIR —
    absorbs gain and EQ), then takes the peak normalized cross-correlation of
    the residual against dry over +/-0.5 s. Delayed/unaltered dry copies score
    near 1; gain-, EQ- or nonlinear-only pairs score near 0.
    """
    from scipy.linalg import solve_toeplitz

    d = validate_audio(dry, "dry")
    w = validate_audio(wet, "wet")
    n = min(d.size, w.size)
    d = d[:n]
    w = w[:n]

    # Best L-tap linear predictor of wet from dry (normal equations).
    l_taps = int(min(65, max(8, n // 8)))
    corr_dd = signal.correlate(d, d, mode="full", method="fft")[n - 1: n - 1 + l_taps]
    corr_wd = signal.correlate(w, d, mode="full", method="fft")[n - 1: n - 1 + l_taps]
    if corr_dd[0] <= 0.0:
        raise ValueError("dry has zero energy")
    corr_dd = corr_dd.copy()
    corr_dd[0] *= 1.0 + 1e-9  # regularize
    taps = solve_toeplitz((corr_dd, corr_dd), corr_wd)
    resid = w - signal.lfilter(taps, [1.0], d)

    rr = float(np.dot(resid, resid))
    ww = float(np.dot(w, w))
    if rr < 1e-8 * max(ww, 1e-30):
        return 0.0  # wet is fully explained by a linear filter of dry

    corr = signal.correlate(resid, d, mode="full", method="fft")
    lags = signal.correlation_lags(n, n, mode="full")
    lag_max = int(min(0.5 * sr, n // 4))
    mask = np.abs(lags) <= lag_max
    denom = np.sqrt(rr * float(np.dot(d, d)))
    return float(np.max(np.abs(corr[mask])) / (denom + 1e-30))


def detect_parallel_mix(dry: np.ndarray, wet: np.ndarray, sr: int = TARGET_SR) -> bool:
    """True when wet appears to contain an unprocessed dry blend.

    High residual correlation after best gain match (see
    :func:`parallel_mix_score`). Note: a delay/reverb whose output is largely
    delayed copies of dry also scores high; the chain model represents those
    with the ``mix`` parameter instead.
    """
    return parallel_mix_score(dry, wet, sr) > PARALLEL_MIX_THRESHOLD
