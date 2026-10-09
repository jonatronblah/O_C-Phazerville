"""Linear EQ estimation: spectral-ratio transfer function + greedy biquad fit."""
from __future__ import annotations

import numpy as np
from scipy import optimize, signal

from .common import TARGET_SR, biquad_response_db, design_biquad, to_db, validate_audio
from ..models import BiquadStage

#: Residual below this (dB) stops the greedy stage search.
FIT_STOP_DB = 1.0

#: A stage must improve the fit by at least this (dB rms) to be kept. This is
#: the parsimony rule: a periodic comb ripple left over from a delay (or plain
#: estimator wobble) can be approximated by peaking biquads, but a delay
#: explains the ripple with one stage, so the EQ fit only accepts stages that
#: capture real (smooth, broadband) EQ shape.
MIN_STAGE_IMPROVEMENT_DB = 0.35

#: Log-spaced grid used for fitting (resolution ~1/25 octave).
_N_GRID = 512
_F_MIN = 20.0
_F_MAX = 20000.0


def _welch_transfer(
    dry: np.ndarray, wet: np.ndarray, sr: int, smooth_hz: float | None = None
) -> tuple[np.ndarray, np.ndarray]:
    """H1 transfer-function estimate H(f) = W/D via Welch cross/auto spectra.

    Returns ``(freqs, mag_db)`` on a log-spaced grid, median-smoothed and
    normalized to zero weighted-mean gain (a flat offset is not an EQ shape;
    overall level belongs to ``ChainModel.output_gain_db``).

    ``smooth_hz`` sets the median-smoothing width. When the pair contains a
    delay, the transfer has a comb ripple of period 1/T Hz (plus sharp pockets
    of exact, un-averaged transfer values around any tonal partials of the
    source); smoothing wider than the ripple period removes both so the EQ fit
    only sees the smooth remainder. Narrow EQ features below that width are
    deliberately absorbed by the delay explanation (parsimony).
    """
    nperseg = int(min(8192, max(256, 2 ** int(np.floor(np.log2(max(dry.size // 8, 256)))))))
    f, pxy = signal.csd(dry, wet, fs=sr, nperseg=nperseg)
    _, pxx = signal.welch(dry, fs=sr, nperseg=nperseg)

    pmax = float(pxx.max()) if pxx.size else 0.0
    if pmax <= 0.0:
        raise ValueError("dry has zero power")
    h = pxy / np.maximum(pxx, 1e-6 * pmax)
    mag_db = 20.0 * np.log10(np.maximum(np.abs(h), 1e-6))

    f_hi = min(_F_MAX, 0.45 * sr)
    band = (f >= _F_MIN) & (f <= f_hi) & (pxx >= 1e-5 * pmax)
    if band.sum() < 8:
        raise ValueError("spectrum too narrow to estimate EQ")
    fb = f[band]
    mb = mag_db[band]

    # Median smoothing kills comb ripple (delayed dry) and spectral outliers.
    df = float(fb[1] - fb[0]) if fb.size > 1 else 1.0
    if smooth_hz is not None and smooth_hz > 0.0 and df > 0.0:
        kernel = int(round(smooth_hz / df))
    else:
        kernel = 7
    kernel = max(3, min(kernel, mb.size if mb.size % 2 else mb.size - 1))
    if kernel % 2 == 0:
        kernel -= 1
    mb = signal.medfilt(mb, kernel_size=max(3, kernel))

    grid = np.geomspace(max(fb[0], _F_MIN), min(fb[-1], f_hi), _N_GRID)
    target = np.interp(grid, fb, mb)
    target -= np.mean(target)  # remove flat gain; not representable as EQ shape
    return grid, target


def transfer_function_db(
    dry: np.ndarray, wet: np.ndarray, sr: int = TARGET_SR, smooth_hz: float | None = None
) -> tuple[np.ndarray, np.ndarray]:
    """Public helper: normalized log-magnitude transfer estimate of wet/dry."""
    d = validate_audio(dry, "dry")
    w = validate_audio(wet, "wet")
    n = min(d.size, w.size)
    return _welch_transfer(d[:n], w[:n], sr, smooth_hz)


def stages_response_db(stages, freqs: np.ndarray, sr: int) -> np.ndarray:
    """Combined magnitude response (dB) of a sequence of biquad stages."""
    out = np.zeros_like(freqs, dtype=np.float64)
    for st in stages:
        b, a = design_biquad(st.kind, st.freq_hz, st.q, st.gain_db, sr)
        out += biquad_response_db(b, a, freqs, sr)
    return out


def _stage_response_db(kind: str, f0: float, q: float, gain: float,
                       freqs: np.ndarray, sr: int) -> np.ndarray:
    try:
        b, a = design_biquad(kind, f0, q, gain, sr)
    except ValueError:
        return np.full_like(freqs, np.nan)
    return biquad_response_db(b, a, freqs, sr)


def _pick_kind(freqs: np.ndarray, resid: np.ndarray) -> list[str]:
    """Candidate filter shapes for the current residual, best guess first."""
    k = int(np.argmax(np.abs(resid)))
    f_edge = 2.0  # within a factor of ~1.4 of the band edge
    if k <= f_edge or k >= resid.size - 1 - f_edge:
        return ["lowshelf", "highshelf", "peaking"]
    return ["peaking", "lowshelf", "highshelf"]


def _fit_one_stage(
    freqs: np.ndarray, target: np.ndarray, model: np.ndarray, sr: int
) -> tuple[BiquadStage | None, float]:
    """Greedy fit of one biquad: coarse init then least-squares refine."""
    resid = target - model
    k = int(np.argmax(np.abs(resid)))
    if abs(resid[k]) < FIT_STOP_DB:
        return None, 0.0

    f0_init = float(freqs[k])
    gain_init = float(np.clip(resid[k], -18.0, 18.0))
    best: tuple[float, str, float, float, float] | None = None
    for kind in _pick_kind(freqs, resid):
        for q0 in (0.7, 1.4, 3.0):
            r = _stage_response_db(kind, f0_init, q0, gain_init, freqs, sr)
            err = float(np.sqrt(np.mean((model + r - target) ** 2)))
            if np.isfinite(err) and (best is None or err < best[0]):
                best = (err, kind, f0_init, q0, gain_init)
    if best is None:
        return None, 0.0
    _, kind, f0, q0, g0 = best

    def residual(params: np.ndarray) -> np.ndarray:
        log_f, log_q, gain = params
        r = _stage_response_db(kind, 10.0 ** log_f, 10.0 ** log_q, gain, freqs, sr)
        if not np.all(np.isfinite(r)):
            return np.full(freqs.size, 1e3)
        return model + r - target

    lo_f, hi_f = float(np.log10(freqs[0])), float(np.log10(freqs[-1]))
    x0 = np.array([np.clip(np.log10(f0), lo_f, hi_f), np.log10(q0), g0])
    lower = np.array([lo_f, np.log10(0.15), -24.0])
    upper = np.array([hi_f, np.log10(18.0), 24.0])
    x0 = np.clip(x0, lower, upper)
    try:
        sol = optimize.least_squares(residual, x0, bounds=(lower, upper), method="trf")
        log_f, log_q, gain = sol.x
    except Exception:
        log_f, log_q, gain = x0

    stage = BiquadStage(
        kind=kind,
        freq_hz=float(np.clip(10.0 ** log_f, freqs[0], freqs[-1])),
        q=float(np.clip(10.0 ** log_q, 0.15, 18.0)),
        gain_db=float(np.clip(gain, -24.0, 24.0)),
    )
    r = _stage_response_db(stage.kind, stage.freq_hz, stage.q, stage.gain_db, freqs, sr)
    new_model = model + r
    improvement = float(np.sqrt(np.mean(resid ** 2)) - np.sqrt(np.mean((target - new_model) ** 2)))
    return stage, improvement


def measure_eq(
    dry: np.ndarray,
    wet: np.ndarray,
    sr: int = TARGET_SR,
    max_stages: int = 4,
    smooth_hz: float | None = None,
) -> list[BiquadStage]:
    """Estimate linear EQ: transfer H(f) = W/D, fit up to ``max_stages`` biquads.

    Returns the fitted stages (possibly empty when the transfer is flat).
    Stages are ordered by the greedy fit; shapes used are peaking / lowshelf /
    highshelf. ``smooth_hz`` (see :func:`_welch_transfer`) lets callers who
    already detected a delay suppress the comb ripple before fitting.
    """
    if max_stages < 1:
        raise ValueError("max_stages must be >= 1")
    d = validate_audio(dry, "dry")
    w = validate_audio(wet, "wet")
    n = min(d.size, w.size)
    freqs, target = _welch_transfer(d[:n], w[:n], sr, smooth_hz)

    stages: list[BiquadStage] = []
    model = np.zeros_like(target)
    for _ in range(max_stages):
        if float(np.max(np.abs(target - model))) < FIT_STOP_DB:
            break
        stage, improvement = _fit_one_stage(freqs, target, model, sr)
        if stage is None or improvement < MIN_STAGE_IMPROVEMENT_DB:
            break
        stages.append(stage)
        model += _stage_response_db(
            stage.kind, stage.freq_hz, stage.q, stage.gain_db, freqs, sr
        )
    return stages
