"""Impulse-response deconvolution, delay and reverb estimation."""
from __future__ import annotations

import numpy as np
from scipy import fft, signal

from .common import TARGET_SR, validate_audio
from ..models import DelayModel, ReverbModel

#: IR tails shorter than this are not reverb.
MIN_REVERB_TAIL_S = 0.15

#: Delay peaks below this (relative to the main peak) are ignored. -30 dB:
#: weak echoes still matter for the feedback decay estimate, and the quiet-gap
#: plus periodicity checks keep noise peaks out.
MIN_ECHO_REL = 0.03

#: Min spacing between IR peaks when searching for echoes.
MIN_PEAK_SEP_S = 0.002


def estimate_ir(
    dry: np.ndarray, wet: np.ndarray, sr: int = TARGET_SR, reg: float = 1e-3
) -> np.ndarray:
    """Regularized frequency-domain deconvolution: W ~ H*D  ->  H = W*D/(|D|^2+lam).

    Returns the real-valued IR of length ``next_fast_len(2*N)`` (zero-padded so
    the estimate is linear, not circular). ``reg`` is the Tikhonov
    regularization as a fraction of the *mean* dry power spectrum. Scaling by
    the mean (not the max) matters for tonal dry signals: a sine's bin is many
    orders of magnitude above the noise bins, and a max-scaled lambda would
    suppress every non-tonal bin and flatten the IR by tens of dB.
    """
    d = validate_audio(dry, "dry")
    w = validate_audio(wet, "wet")
    n = min(d.size, w.size)
    d = d[:n]
    w = w[:n]
    n_fft = fft.next_fast_len(2 * n)
    dry_f = fft.rfft(d, n_fft)
    wet_f = fft.rfft(w, n_fft)
    power = np.abs(dry_f) ** 2
    lam = float(reg * max(float(power.mean()), 1e-300))
    h = (np.conj(dry_f) * wet_f) / (power + lam)
    return fft.irfft(h, n_fft)


def _significant_peaks(
    absir: np.ndarray, sr: int, pmax: float, rel: float = MIN_ECHO_REL
) -> list[int]:
    """Significant IR peaks (index space) at or above ``rel`` of the peak."""
    idx, _ = signal.find_peaks(
        absir, height=rel * pmax, distance=max(1, int(MIN_PEAK_SEP_S * sr))
    )
    peaks = [int(i) for i in idx]
    # find_peaks never reports endpoints; the direct tap is often at index 0.
    if absir.size and absir[0] >= rel * pmax and (
        absir.size == 1 or absir[0] >= absir[1]
    ):
        peaks.insert(0, 0)
    return peaks


def _gap_quiet(absir: np.ndarray, i: int, j: int, pmax: float, sr: int) -> bool:
    """True when the IR floor between peaks i and j is quiet (a comb, not a tail)."""
    shoulder = int(0.001 * sr)
    lo, hi = i + shoulder, j - shoulder
    if hi - lo < 8:
        return False
    floor = float(np.percentile(absir[lo:hi], 20))
    return floor < max(0.10 * min(absir[i], absir[j]), 0.03 * pmax)


def _analyze_comb(absir: np.ndarray, sr: int) -> tuple[int, list[int]] | None:
    """Find the comb spacing T and peak indices at p_direct + k*T, or None.

    The reference peak is the *first arrival* (significant peak closest to
    zero lag — the direct path), not the largest peak: with mix < 1 the first
    echo can be louder than the direct tap, and anchoring on the argmax would
    report the echo-to-echo gap as the period while dropping the direct peak
    from the amplitude series. Requires quiet gaps between consecutive matched
    peaks (a sparse echo train, not a diffuse tail). Echoes are searched
    forward from the direct path only.
    """
    n = absir.size
    pmax = float(absir.max()) if n else 0.0
    if pmax <= 0.0:
        return None
    peaks = _significant_peaks(absir, sr, pmax)
    if len(peaks) < 2:
        return None

    def signed_lag(i: int) -> int:
        return i if i <= n // 2 else i - n

    p0 = min(peaks, key=lambda i: (abs(signed_lag(i)), i))

    min_t = int(MIN_PEAK_SEP_S * sr)
    tol = 0.05  # peak must sit within 5% of the expected multiple
    scored: list[tuple[int, list[int], float]] = []
    pair_gaps = sorted(
        {abs(b - a) for a in peaks for b in peaks if abs(b - a) >= min_t}
    )
    for t in pair_gaps:
        matched = [p0]
        for k in range(1, (n - p0) // t + 1):
            expected = p0 + k * t
            near = [c for c in peaks if abs(c - expected) <= tol * t]
            if not near:
                break
            matched.append(min(near, key=lambda c: abs(c - expected)))
        if len(matched) >= 2:
            score = float(sum(absir[m] for m in matched[1:]))
            scored.append((int(t), matched, score))
    if not scored:
        return None
    best_score = max(s for _, _, s in scored)
    # Among near-best scorers prefer the smallest spacing (true delay line).
    viable = [s for s in scored if s[2] >= 0.9 * best_score]
    t, matched, _ = min(viable, key=lambda s: s[0])

    for i in range(len(matched) - 1):
        if not _gap_quiet(absir, matched[i], matched[i + 1], pmax, sr):
            return None
    return t, matched


def _window_around(ir: np.ndarray, center: int, half: int) -> np.ndarray:
    """Fixed-size Hann-windowed slice around ``center``, zero-padded at edges."""
    size = 2 * half + 1
    out = np.zeros(size, dtype=np.float64)
    lo = max(center - half, 0)
    hi = min(center + half + 1, ir.size)
    out[lo - (center - half): size - (center + half + 1 - hi)] = ir[lo:hi]
    return out * np.hanning(size)


def _echo_tone_hz(ir: np.ndarray, p0: int, echo: int, sr: int) -> float | None:
    """Feedback-loop tone: -6 dB rolloff of the echo spectrum vs the main peak."""
    half = int(0.004 * sr)
    if ir.size < 4 * half:
        return None
    e_win = _window_around(ir, echo, half)
    d_win = _window_around(ir, p0, half)
    e_spec = np.abs(np.fft.rfft(e_win))
    d_spec = np.abs(np.fft.rfft(d_win))
    freqs = np.fft.rfftfreq(e_win.size, 1.0 / sr)
    ratio = e_spec / (d_spec + 1e-12 * float(d_spec.max()) + 1e-30)
    low = (freqs >= 50.0) & (freqs <= 400.0)
    if not low.any():
        return None
    ref = float(np.median(ratio[low]))
    if ref <= 0.0:
        return None
    above = np.where(ratio >= 0.5 * ref)[0]  # -6 dB
    if above.size == 0:
        return None
    f_edge = float(freqs[above[-1]])
    if f_edge >= 0.4 * sr:  # no meaningful rolloff: no tone filter
        return None
    return float(np.clip(f_edge, 20.0, sr / 2.0))


def measure_delay(
    dry: np.ndarray, wet: np.ndarray, sr: int = TARGET_SR
) -> DelayModel | None:
    """Detect echo/delay from the deconvolved IR.

    Peak-picks delayed copies of the direct path; the comb spacing gives
    ``time_ms``, the geometric decay of matched echo amplitudes gives
    ``feedback`` (echo-to-echo ratios only), ``mix`` maps the direct/first-echo
    amplitude ratio onto the ``(1-mix)*x + mix*delayed`` model, and the echo
    spectral tilt gives ``tone_hz``. Returns None when no significant isolated
    delayed peak exists (or when the IR is a diffuse tail rather than a comb).
    """
    ir = estimate_ir(dry, wet, sr)
    absir = np.abs(ir)
    if absir.max() <= 0.0:
        return None
    comb = _analyze_comb(absir, sr)
    if comb is None:
        return None
    t_samples, matched = comb
    amps = [float(absir[m]) for m in matched]

    # Feedback from echo-to-echo amplitude ratios (skip the direct-to-echo ratio:
    # the first peak may be the dry path, whose ratio to echo 1 is not feedback).
    if len(amps) >= 3:
        ratios = [amps[k + 1] / max(amps[k], 1e-30) for k in range(1, len(amps) - 1)]
        feedback = float(np.clip(np.median(ratios), 0.0, 0.95))
    else:
        feedback = 0.0

    # mix: model taps are (1-mix) at the direct path and mix at the first echo.
    mix = amps[1] / max(amps[0] + amps[1], 1e-30) if len(amps) >= 2 else 1.0

    p0 = matched[0]
    tone = _echo_tone_hz(ir, p0, matched[1], sr) if len(matched) >= 2 else None

    return DelayModel(
        time_ms=float(t_samples) * 1000.0 / sr,
        feedback=feedback,
        mix=float(np.clip(mix, 0.0, 1.0)),
        tone_hz=tone,
    )


def _rt60_from_energy(energy: np.ndarray, sr: int) -> float | None:
    """Schroeder backward integration, decay slope extrapolated to -60 dB.

    The deconvolution residual floor is estimated from late IR segments and
    its linear pedestal is subtracted from the *integral* (per-sample clipping
    retains a biased remainder and flattens the curve). The slope is fitted on
    the contiguous decay between -5 and -35 dB.
    """
    e = np.asarray(energy, dtype=np.float64)
    if e.size < 256 or float(np.sum(e)) <= 0.0:
        return None
    seg = max(256, e.size // 8)
    floor_est = 0.0
    for k in range(4):
        s0 = e.size - (k + 1) * seg
        if s0 < 0:
            break
        floor_est = max(floor_est, float(np.mean(e[s0: s0 + seg])))

    counts = np.arange(e.size, 0, -1, dtype=np.float64)
    edc = np.cumsum(e[::-1])[::-1] - counts * floor_est
    if edc[0] <= 0.0:
        return None
    curve = 10.0 * np.log10(np.maximum(edc / edc[0], 1e-9))
    # Edge-safe light smoothing to tame Schroeder wobble.
    w = max(9, int(0.0005 * sr) | 1)
    padded = np.pad(curve, w, mode="edge")
    sm = np.convolve(padded, np.ones(w) / w, mode="same")[w:-w]

    start = int(0.002 * sr)
    t_all = np.arange(curve.size) / float(sr)
    best: tuple[float, float] | None = None  # (r2, slope)
    for lo, hi in ((-5.0, -35.0), (-3.0, -25.0), (-2.0, -12.0)):
        below_lo = np.flatnonzero((sm <= lo) & (np.arange(curve.size) >= start))
        if below_lo.size == 0:
            continue
        i0 = int(below_lo[0])
        below_hi = np.flatnonzero(sm[i0:] <= hi)
        i1 = i0 + int(below_hi[0]) if below_hi.size else curve.size
        seg_idx = np.arange(i0, i1)
        if seg_idx.size < 32:
            continue
        yy = sm[seg_idx]
        slope = float(np.polyfit(t_all[seg_idx], yy, 1)[0])
        if slope >= -1e-6:
            continue
        # A genuine exponential decay gives a linear Schroeder curve; a
        # flattening (floor-dominated) window does not and must be rejected.
        ss_tot = float(np.sum((yy - yy.mean()) ** 2))
        ss_res = float(np.sum((yy - (slope * t_all[seg_idx] + (yy.mean() - slope * t_all[seg_idx].mean()))) ** 2))
        r2 = 1.0 - ss_res / max(ss_tot, 1e-30)
        if r2 >= 0.97:
            return float(np.clip(-60.0 / slope, 0.05, 20.0))
        if best is None or r2 > best[0]:
            best = (r2, slope)
    if best is not None and best[0] >= 0.90:
        return float(np.clip(-60.0 / best[1], 0.05, 20.0))
    return None


def measure_reverb(
    dry: np.ndarray, wet: np.ndarray, sr: int = TARGET_SR
) -> ReverbModel | None:
    """Estimate reverb from the deconvolved IR.

    Requires a dense diffuse tail (>= ~0.15 s); sparse comb tails are delay and
    return None. Estimates rt60 (Schroeder T30), predelay (tail onset), damping
    (HF vs LF decay ratio), room_size (decay time + diffuseness) and mix (tail
    vs direct energy).
    """
    ir = estimate_ir(dry, wet, sr)
    absir = np.abs(ir)
    p0 = int(np.argmax(absir))
    pmax = float(absir[p0])
    if pmax <= 0.0:
        return None
    tail = absir[p0:]
    above = np.where(tail > 0.01 * pmax)[0]
    if above.size == 0:
        return None
    tail_s = float(above[-1]) / sr
    if tail_s < MIN_REVERB_TAIL_S:
        return None

    # Diffuseness on the tail after the direct peak: a sparse echo train puts
    # nearly all its energy into a few sharp peaks; a diffuse tail does not.
    half = int(0.002 * sr)
    tail_body = absir[p0 + half + 1:]
    if tail_body.size < 256:
        return None
    tail_max = float(tail_body.max())
    if tail_max <= 0.0:
        return None
    pk_idx, _ = signal.find_peaks(
        tail_body, height=0.05 * tail_max, distance=max(1, int(0.002 * sr))
    )
    win = max(2, int(0.00025 * sr))
    captured = np.zeros(tail_body.size, dtype=bool)
    for i in pk_idx:
        captured[max(int(i) - win, 0): int(i) + win + 1] = True
    comb_share = float(
        np.sum(tail_body[captured] ** 2) / max(float(np.sum(tail_body ** 2)), 1e-30)
    )
    if comb_share > 0.3:
        return None  # sparse echo train: that is a delay, not a reverb
    diffuseness = 1.0 - comb_share

    rt60 = _rt60_from_energy(ir[p0:] ** 2, sr)
    if rt60 is None:
        return None

    # Predelay: onset of significant tail energy after the main peak.
    onset = np.where(tail > 0.02 * pmax)[0]
    predelay_ms = float(onset[0]) * 1000.0 / sr if onset.size else 0.0

    # Damping from HF vs LF decay rates.
    damping = 0.5
    try:
        sos_lo = signal.butter(2, 400.0, "lowpass", fs=sr, output="sos")
        sos_hi = signal.butter(2, 2000.0, "highpass", fs=sr, output="sos")
        seg = ir[p0:]
        lo = signal.sosfiltfilt(sos_lo, seg)
        hi = signal.sosfiltfilt(sos_hi, seg)
        rt_lo = _rt60_from_energy(lo ** 2, sr)
        rt_hi = _rt60_from_energy(hi ** 2, sr)
        if rt_lo and rt_hi and rt_lo > 0:
            damping = float(np.clip(1.0 - rt_hi / rt_lo, 0.0, 1.0))
    except Exception:
        damping = 0.5

    half = int(0.002 * sr)
    lo_i, hi_i = max(p0 - half, 0), min(p0 + half + 1, absir.size)
    e_direct = float(np.sum(absir[lo_i:hi_i] ** 2))
    e_tail = float(np.sum(absir[p0 + half + 1:] ** 2))
    mix = float(np.clip(e_tail / max(e_direct + e_tail, 1e-30), 0.0, 1.0))

    room_size = float(
        np.clip(0.5 * np.clip(rt60 / 2.0, 0.0, 1.0) + 0.5 * diffuseness, 0.0, 1.0)
    )

    return ReverbModel(
        rt60_s=rt60,
        predelay_ms=predelay_ms,
        damping=damping,
        room_size=room_size,
        mix=mix,
    )
