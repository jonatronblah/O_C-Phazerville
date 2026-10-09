"""Offline digital-twin renderer and scorer for proposed effect chains.

This module is the verification loop for cloning audio effects into Teensy 4.1
firmware applets: given a proposed :class:`~ocapp_agent.models.ChainModel` and
the original dry signal, :func:`render_chain` renders a processed approximation
of what the firmware DSP would produce, :func:`score_render` compares it against
the actual wet signal, and :func:`verify_chain` does both (aligning rendered vs
wet by cross-correlation first, since effect chains may introduce latency).
The agent refines parameters until the score is good, then emits C++ code.
The firmware runs at 48 kHz with Teensy Audio building blocks; this offline
render approximates the same signal chain.

DSP design per stage (all deterministic, pure numpy/scipy):

* EQ        -- cascade of RBJ Audio-EQ-Cookbook biquads (``_rbj_biquad``),
               applied with ``scipy.signal.lfilter``.  Coefficients follow the
               cookbook for peaking / lowshelf / highshelf / lowpass /
               highpass / bandpass / notch, normalized by ``a0``.
* Delay     -- fractional-delay line with linear interpolation, feedback loop
               (``|feedback| < 1`` enforced), optional one-pole LPF at
               ``tone_hz`` in the feedback path (``a = 1 - exp(-2*pi*f/sr)``),
               wet/dry mix (``mix=1.0`` is fully wet).
* Bitcrush  -- sample-and-hold by ``downsample``, then a uniform quantizer with
               exactly ``2**bit_depth`` reconstruction levels spanning [-1, 1]
               (``round((x+1)*(L-1)/2)`` clamped to ``[0, L-1]``, then mapped
               back), so the output never exceeds ``2**bit_depth`` levels.
* Waveshaper-- memoryless curve on a driven copy of the input (``tanh``,
               ``hard_clip``, ``soft_clip`` = ``x/(1+|x|)``, ``asymmetric`` =
               tanh with different slopes for + / - halves, ``table`` =
               ``np.interp`` over uniformly spaced normalized -1..1 points),
               then wet/dry mix against the undriven input.
* Compressor-- feed-forward design.  Detector law: peak detector on ``|x|``
               with a branching one-pole smoother; coefficient
               ``exp(-1/(tau_s*sr))`` (≈63% approach per time constant), attack
               coefficient when ``|x|`` exceeds the envelope (rising), release
               coefficient otherwise (falling).  Gain computer: quadratic soft
               knee on the dB level with static slope ``(1/ratio - 1)`` dB out
               per dB in above the threshold; there is no second smoothing
               stage -- the envelope follower alone defines attack/release.
               Makeup gain applied at the output.
* Reverb    -- Freeverb-style Schroeder reverb: 4 parallel damped combs into 2
               series allpasses, integer predelay, HF-damping one-pole in the
               comb feedback (``store = out*(1-damp) + lp*damp``).  Comb
               feedback is calibrated from the effective RT60:
               ``fb = 10 ** (-3 * t_comb / rt60_eff)`` so amplitude drops
               60 dB per ``rt60_eff`` seconds;
               ``rt60_eff = rt60_s * (0.75 + 0.5 * room_size)`` (the default
               ``room_size=0.5`` makes RT60 exactly ``rt60_s``; room_size acts
               as a ±25% room trim so RT60 always approximates ``rt60_s``).

Stages are applied in list order, then ``chain.output_gain_db``.  Renders are
computed in float64 and returned as mono float32 with the input length.

Score semantics (what "convincing" means):

* ``si_sdr_db`` -- scale-invariant SDR (Le Roux et al.), zero-mean, clamped to
  ±100 dB.  >= 20 dB is a convincing clone; >= 25-30 dB is essentially
  indistinguishable in a blind A/B for monophonic chains; <= 5 dB means the
  structure is wrong.  Note it is scale-*invariant*: a global level error can
  coexist with a high si-SDR, which is why loudness_diff_lufs also exists.
* ``log_spectral_distance`` -- per-frame RMS over bins of the log-magnitude
  STFT ratio (2048-pt Hann, 512 hop), averaged over frames, in dB.  <= 1 dB is
  convincing, <= 0.3 dB is near-identical, 0 exactly for identical signals.
* ``loudness_diff_lufs`` -- integrated BS.1770 loudness (pyloudnorm) of
  rendered minus wet, in LU.  |diff| <= 1 LU is fine.

``FitReport.limitations`` carries limitations observed DURING verification
(verify-stage caveats); the agent lane merges them into
``CloneResult.limitations``, which is the authoritative final list.
"""
from __future__ import annotations

import math

import numpy as np
import pyloudnorm as pyln
from scipy import signal as sps

from ocapp_agent.models import (
    BitcrushModel,
    ChainModel,
    CompressorModel,
    DelayModel,
    EqModel,
    FitReport,
    ReverbModel,
    WaveshaperModel,
)

TARGET_SR = 48000

_EPS = 1e-12
_MAG_FLOOR = 1e-8
_SI_SDR_CAP_DB = 100.0
_N_FFT = 2048
_HOP = 512

# Freeverb-derived comb / allpass tunings in samples at 44.1 kHz (scaled to sr).
_COMB_TUNING_44K = (1116, 1188, 1277, 1356)
_ALLPASS_TUNING_44K = (556, 441)


# ---------------------------------------------------------------- validation

def _check_sr(sr: int) -> int:
    if isinstance(sr, bool) or not isinstance(sr, (int, np.integer)):
        raise ValueError(f"sr must be a positive integer, got {sr!r}")
    if int(sr) <= 0:
        raise ValueError(f"sr must be positive, got {sr!r}")
    return int(sr)


def _validate_mono(x: np.ndarray, name: str) -> np.ndarray:
    """Return x as a 1-D float64 array; raise ValueError on empty/NaN/inf."""
    arr = np.asarray(x)
    if arr.ndim != 1:
        raise ValueError(f"{name} must be a 1-D mono array, got shape {arr.shape}")
    if arr.size == 0:
        raise ValueError(f"{name} is empty")
    if not np.issubdtype(arr.dtype, np.number):
        raise ValueError(f"{name} must be numeric, got dtype {arr.dtype}")
    out = arr.astype(np.float64, copy=False)
    if not np.all(np.isfinite(out)):
        raise ValueError(f"{name} contains NaN or infinite samples")
    return out


def _gain_db_to_linear(db: float) -> float:
    return 10.0 ** (db / 20.0)


# ------------------------------------------------------------------- EQ stage

def _rbj_biquad(
    kind: str, freq_hz: float, q: float, gain_db: float, sr: int
) -> tuple[np.ndarray, np.ndarray]:
    """RBJ Audio-EQ-Cookbook biquad coefficients (b, a), normalized by a0.

    A = 10**(gain_db/40) for peaking/shelving; w0 = 2*pi*f0/sr;
    alpha = sin(w0)/(2*Q).  Frequencies at or above Nyquist are clamped to
    0.99*Nyquist so the filter stays well-defined.
    """
    nyquist = 0.5 * sr
    f0 = min(float(freq_hz), 0.99 * nyquist)
    w0 = 2.0 * math.pi * f0 / sr
    cw = math.cos(w0)
    sw = math.sin(w0)
    alpha = sw / (2.0 * float(q))
    a_lin = 10.0 ** (float(gain_db) / 40.0)
    sqrt_a = math.sqrt(a_lin)

    if kind == "peaking":
        b = [1.0 + alpha * a_lin, -2.0 * cw, 1.0 - alpha * a_lin]
        a = [1.0 + alpha / a_lin, -2.0 * cw, 1.0 - alpha / a_lin]
    elif kind == "lowshelf":
        b = [
            a_lin * ((a_lin + 1.0) - (a_lin - 1.0) * cw + 2.0 * sqrt_a * alpha),
            2.0 * a_lin * ((a_lin - 1.0) - (a_lin + 1.0) * cw),
            a_lin * ((a_lin + 1.0) - (a_lin - 1.0) * cw - 2.0 * sqrt_a * alpha),
        ]
        a = [
            (a_lin + 1.0) + (a_lin - 1.0) * cw + 2.0 * sqrt_a * alpha,
            -2.0 * ((a_lin - 1.0) + (a_lin + 1.0) * cw),
            (a_lin + 1.0) + (a_lin - 1.0) * cw - 2.0 * sqrt_a * alpha,
        ]
    elif kind == "highshelf":
        b = [
            a_lin * ((a_lin + 1.0) + (a_lin - 1.0) * cw + 2.0 * sqrt_a * alpha),
            -2.0 * a_lin * ((a_lin - 1.0) + (a_lin + 1.0) * cw),
            a_lin * ((a_lin + 1.0) + (a_lin - 1.0) * cw - 2.0 * sqrt_a * alpha),
        ]
        a = [
            (a_lin + 1.0) - (a_lin - 1.0) * cw + 2.0 * sqrt_a * alpha,
            2.0 * ((a_lin - 1.0) - (a_lin + 1.0) * cw),
            (a_lin + 1.0) - (a_lin - 1.0) * cw - 2.0 * sqrt_a * alpha,
        ]
    elif kind == "lowpass":
        b = [(1.0 - cw) / 2.0, 1.0 - cw, (1.0 - cw) / 2.0]
        a = [1.0 + alpha, -2.0 * cw, 1.0 - alpha]
    elif kind == "highpass":
        b = [(1.0 + cw) / 2.0, -(1.0 + cw), (1.0 + cw) / 2.0]
        a = [1.0 + alpha, -2.0 * cw, 1.0 - alpha]
    elif kind == "bandpass":  # constant 0 dB peak gain
        b = [alpha, 0.0, -alpha]
        a = [1.0 + alpha, -2.0 * cw, 1.0 - alpha]
    elif kind == "notch":
        b = [1.0, -2.0 * cw, 1.0]
        a = [1.0 + alpha, -2.0 * cw, 1.0 - alpha]
    else:  # pragma: no cover - guarded by the model schema
        raise ValueError(f"unknown biquad kind: {kind!r}")

    a0 = a[0]
    return np.array(b) / a0, np.array(a) / a0


def _render_eq(x: np.ndarray, stage: EqModel, sr: int) -> np.ndarray:
    y = x
    for sub in stage.stages:
        b, a = _rbj_biquad(sub.kind, sub.freq_hz, sub.q, sub.gain_db, sr)
        y = sps.lfilter(b, a, y)
    return y


# ---------------------------------------------------------------- delay stage

def _render_delay(x: np.ndarray, stage: DelayModel, sr: int) -> np.ndarray:
    n = x.size
    delay = float(stage.time_ms) * sr / 1000.0
    fb = float(stage.feedback)
    if abs(fb) >= 1.0:
        raise ValueError(f"delay feedback must satisfy |feedback| < 1, got {fb}")

    if delay <= 0.0:
        wet = x.copy()
    else:
        buf_len = int(math.ceil(delay)) + 2
        buf = np.zeros(buf_len)
        wet = np.empty(n)
        w = 0
        use_lp = stage.tone_hz is not None
        if use_lp:
            lp_a = 1.0 - math.exp(-2.0 * math.pi * float(stage.tone_hz) / sr)
            lp_state = 0.0
        for i in range(n):
            rp = w - delay
            base = math.floor(rp)
            frac = rp - base
            i0 = int(base) % buf_len
            i1 = (i0 + 1) % buf_len
            delayed = buf[i0] * (1.0 - frac) + buf[i1] * frac  # linear interp
            wet[i] = delayed
            fb_sig = delayed
            if use_lp:  # one-pole LPF at tone_hz inside the feedback path
                lp_state += lp_a * (fb_sig - lp_state)
                fb_sig = lp_state
            buf[w] = x[i] + fb * fb_sig
            w += 1
            if w >= buf_len:
                w = 0

    mix = float(stage.mix)
    return (1.0 - mix) * x + mix * wet


# --------------------------------------------------------------- bitcrush

def _render_bitcrush(x: np.ndarray, stage: BitcrushModel) -> np.ndarray:
    y = x
    k = int(stage.downsample)
    if k > 1:  # sample-and-hold: take every k-th sample and hold it
        idx = (np.arange(x.size) // k) * k
        y = x[idx]
    levels = 2 ** int(stage.bit_depth)
    idx_q = np.round((y + 1.0) * (levels - 1) / 2.0)
    idx_q = np.clip(idx_q, 0.0, float(levels - 1))
    return idx_q * (2.0 / (levels - 1)) - 1.0


# -------------------------------------------------------------- waveshaper

def _render_waveshaper(x: np.ndarray, stage: WaveshaperModel) -> np.ndarray:
    drive = _gain_db_to_linear(float(stage.drive_db))
    xd = x * drive
    curve = stage.curve
    if curve == "tanh":
        shaped = np.tanh(xd)
    elif curve == "hard_clip":
        shaped = np.clip(xd, -1.0, 1.0)
    elif curve == "soft_clip":  # x/(1+|x|): unity slope at 0, asymptotes ±1
        shaped = xd / (1.0 + np.abs(xd))
    elif curve == "asymmetric":  # different tanh slopes for + / - halves
        shaped = np.where(xd >= 0.0, np.tanh(2.0 * xd), np.tanh(1.0 * xd))
    elif curve == "table":
        tbl = np.asarray(stage.table, dtype=np.float64) if stage.table is not None else np.empty(0)
        if tbl.size < 2:
            raise ValueError("waveshaper 'table' curve needs at least 2 points")
        if not np.all(np.isfinite(tbl)):
            raise ValueError("waveshaper 'table' contains NaN or infinite values")
        xp = np.linspace(-1.0, 1.0, tbl.size)  # uniformly spaced -1..1 points
        shaped = np.interp(xd, xp, tbl)  # flat endpoint hold outside -1..1
    else:  # pragma: no cover - guarded by the model schema
        raise ValueError(f"unknown waveshaper curve: {curve!r}")
    mix = float(stage.mix)
    return (1.0 - mix) * x + mix * shaped


# -------------------------------------------------------------- compressor

def _render_compressor(x: np.ndarray, stage: CompressorModel, sr: int) -> np.ndarray:
    """Feed-forward compressor; detector law documented in the module docstring."""
    atk = math.exp(-1.0 / (float(stage.attack_ms) * 0.001 * sr))
    rel = math.exp(-1.0 / (float(stage.release_ms) * 0.001 * sr))
    thr = float(stage.threshold_db)
    knee = float(stage.knee_db)
    slope = 1.0 / float(stage.ratio) - 1.0  # negative: dB out per dB in above thr
    makeup = _gain_db_to_linear(float(stage.makeup_db))
    n = x.size
    y = np.empty(n)
    env = 0.0
    for i in range(n):
        a = abs(x[i])
        if a > env:
            env = atk * env + (1.0 - atk) * a
        else:
            env = rel * env + (1.0 - rel) * a
        level_db = 20.0 * math.log10(env + _EPS)
        over = level_db - thr
        if knee > 0.0 and over < knee / 2.0 and over > -knee / 2.0:
            gain_db = slope * (over + knee / 2.0) ** 2 / (2.0 * knee)  # soft knee
        elif over > knee / 2.0 or (knee <= 0.0 and over > 0.0):
            gain_db = slope * over
        else:
            gain_db = 0.0
        y[i] = x[i] * (10.0 ** (gain_db / 20.0)) * makeup
    return y


# ------------------------------------------------------------------ reverb

def _comb_damped(x: np.ndarray, delay: int, fb: float, damp: float) -> np.ndarray:
    """Freeverb-style damped comb: one-pole LPF (damp) inside the feedback path."""
    n = x.size
    buf = np.zeros(delay)
    out = np.empty(n)
    w = 0
    lp = 0.0
    damp2 = 1.0 - damp
    for i in range(n):
        o = buf[w]
        out[i] = o
        lp = o * damp2 + lp * damp
        buf[w] = x[i] + lp * fb
        w += 1
        if w >= delay:
            w = 0
    return out


def _allpass(x: np.ndarray, delay: int) -> np.ndarray:
    """Freeverb-style allpass (feedback 0.5): out = -x + buf, buf = x + buf*0.5."""
    n = x.size
    buf = np.zeros(delay)
    out = np.empty(n)
    w = 0
    for i in range(n):
        b = buf[w]
        out[i] = b - x[i]
        buf[w] = x[i] + b * 0.5
        w += 1
        if w >= delay:
            w = 0
    return out


def _render_reverb(x: np.ndarray, stage: ReverbModel, sr: int) -> np.ndarray:
    n = x.size
    # predelay (integer samples), shifted in; tail beyond the window is truncated
    pre = int(round(float(stage.predelay_ms) * sr / 1000.0))
    if pre > 0:
        xin = np.zeros(n)
        pre = min(pre, n - 1)
        xin[pre:] = x[: n - pre]
    else:
        xin = x

    # room_size trims effective decay: default 0.5 -> rt60_eff == rt60_s
    rt60_eff = float(stage.rt60_s) * (0.75 + 0.5 * float(stage.room_size))
    rt60_eff = max(rt60_eff, 0.01)
    scale = sr / 44100.0
    damp = float(stage.damping)

    acc = np.zeros(n)
    for tuning in _COMB_TUNING_44K:
        delay = max(1, int(round(tuning * scale)))
        t_comb = delay / sr
        fb = 10.0 ** (-3.0 * t_comb / rt60_eff)  # amplitude -60 dB per rt60_eff
        fb = min(fb, 0.999)
        acc += _comb_damped(xin, delay, fb, damp)
    wet_sig = acc / len(_COMB_TUNING_44K)
    for tuning in _ALLPASS_TUNING_44K:
        delay = max(1, int(round(tuning * scale)))
        wet_sig = _allpass(wet_sig, delay)

    mix = float(stage.mix)
    return (1.0 - mix) * x + mix * wet_sig


# ------------------------------------------------------------------- render

def _render_stage(x: np.ndarray, stage, sr: int) -> np.ndarray:
    if isinstance(stage, EqModel):
        return _render_eq(x, stage, sr)
    if isinstance(stage, DelayModel):
        return _render_delay(x, stage, sr)
    if isinstance(stage, BitcrushModel):
        return _render_bitcrush(x, stage)
    if isinstance(stage, WaveshaperModel):
        return _render_waveshaper(x, stage)
    if isinstance(stage, CompressorModel):
        return _render_compressor(x, stage, sr)
    if isinstance(stage, ReverbModel):
        return _render_reverb(x, stage, sr)
    raise TypeError(f"unsupported stage type: {type(stage).__name__}")


def render_chain(dry: np.ndarray, sr: int, chain: ChainModel) -> np.ndarray:
    """Apply the chain to dry and return the rendered signal (mono float32, same length as input)."""
    sr = _check_sr(sr)
    if not isinstance(chain, ChainModel):
        raise TypeError(f"chain must be a ChainModel, got {type(chain).__name__}")
    x = _validate_mono(dry, "dry")
    y = x
    for stage in chain.stages:
        y = _render_stage(y, stage, sr)
    if chain.output_gain_db != 0.0:
        y = y * _gain_db_to_linear(float(chain.output_gain_db))
    return y.astype(np.float32)


# -------------------------------------------------------------------- score

def _si_sdr_db(est: np.ndarray, ref: np.ndarray) -> float | None:
    """Scale-invariant SDR (Le Roux et al.), zero-mean, clamped to ±100 dB.

    Returns None only when the reference is (near-)silent and the metric is
    therefore undefined.
    """
    est = est - est.mean()
    ref = ref - ref.mean()
    ref_e = float(ref @ ref)
    if ref_e <= _EPS:
        return None
    alpha = float(est @ ref) / ref_e
    target = alpha * ref
    noise = est - target
    t_e = float(target @ target)
    n_e = float(noise @ noise)
    if n_e <= _EPS:
        return _SI_SDR_CAP_DB  # perfect (up to scale): clamp, never return inf
    if t_e <= _EPS:
        return -_SI_SDR_CAP_DB  # nothing in common
    si = 10.0 * math.log10(t_e / n_e)
    return float(max(-_SI_SDR_CAP_DB, min(_SI_SDR_CAP_DB, si)))


def _stft_magnitudes(x: np.ndarray, n_fft: int = _N_FFT, hop: int = _HOP) -> np.ndarray:
    """Periodic-Hann STFT magnitudes, shape (n_frames, n_fft//2 + 1)."""
    if x.size < n_fft:
        x = np.pad(x, (0, n_fft - x.size))
    n_frames = 1 + (x.size - n_fft) // hop
    window = np.hanning(n_fft + 1)[:-1]
    out = np.empty((n_frames, n_fft // 2 + 1), dtype=np.float64)
    for i in range(n_frames):
        frame = x[i * hop : i * hop + n_fft] * window
        out[i] = np.abs(np.fft.rfft(frame))
    return out


def _log_spectral_distance(est_mag: np.ndarray, ref_mag: np.ndarray) -> float:
    """Per-frame RMS over bins of the 20*log10(mag ratio), averaged over frames."""
    ratio_db = 20.0 * np.log10((est_mag + _MAG_FLOOR) / (ref_mag + _MAG_FLOOR))
    per_frame = np.sqrt(np.mean(ratio_db**2, axis=1))
    return float(np.mean(per_frame))


def _band_notes(
    est_mag: np.ndarray, ref_mag: np.ndarray, freqs: np.ndarray
) -> list[str]:
    """Freeform per-band spectral observations (e.g. 'spectral excess above 8 kHz')."""
    est_psd = np.mean(est_mag**2, axis=0)
    ref_psd = np.mean(ref_mag**2, axis=0)
    bands = (
        (20.0, 250.0, "in the 20-250 Hz band"),
        (250.0, 2000.0, "in the 250 Hz-2 kHz band"),
        (2000.0, 8000.0, "in the 2-8 kHz band"),
        (8000.0, float("inf"), "above 8 kHz"),
    )
    notes: list[str] = []
    for lo, hi, label in bands:
        sel = (freqs >= lo) & (freqs < hi)
        if not np.any(sel):
            continue
        e = float(np.sum(est_psd[sel]))
        r = float(np.sum(ref_psd[sel]))
        if e <= 0.0 and r <= 0.0:
            continue
        diff = 10.0 * math.log10((e + _EPS) / (r + _EPS))
        if diff >= 3.0:
            notes.append(f"spectral excess of {diff:.1f} dB {label}")
        elif diff <= -3.0:
            notes.append(f"spectral deficit of {abs(diff):.1f} dB {label}")
    return notes


def _loudness_lufs(x: np.ndarray, sr: int) -> tuple[float | None, bool]:
    """Integrated BS.1770 loudness via pyloudnorm; (value, was_padded).

    Returns (None, _) when loudness is unavailable (e.g. silent signal).
    Short signals are zero-padded to the 400 ms gating block; the padding bias
    cancels when comparing equal-length signals.
    """
    padded = False
    min_len = int(0.4 * sr) + 1
    if x.size < min_len:
        x = np.concatenate([x, np.zeros(min_len - x.size)])
        padded = True
    try:
        meter = pyln.Meter(sr)
        value = float(meter.integrated_loudness(x))
    except Exception:
        return None, padded
    if not math.isfinite(value):
        return None, padded
    return value, padded


def _tail_active(wet: np.ndarray, sr: int) -> bool:
    """True when the wet signal is still loud in its last 50 ms block."""
    block = max(1, int(0.05 * sr))
    n_blocks = wet.size // block
    if n_blocks < 2:
        return False
    blocks = wet[: n_blocks * block].reshape(n_blocks, block)
    brms = np.sqrt(np.mean(blocks**2, axis=1))
    peak = float(np.max(brms))
    return peak > 1e-6 and float(brms[-1]) > 0.1 * peak


def score_render(rendered: np.ndarray, wet: np.ndarray, sr: int) -> FitReport:
    """Compare rendered vs wet: si_sdr_db, log_spectral_distance, loudness_diff_lufs (pyloudnorm), stage_notes (freeform observations, e.g. 'spectral excess above 8 kHz'), limitations (verify-stage caveats)."""
    sr = _check_sr(sr)
    r = _validate_mono(rendered, "rendered")
    w = _validate_mono(wet, "wet")

    limitations: list[str] = []
    notes: list[str] = []

    if r.size != w.size:
        n = min(r.size, w.size)
        limitations.append(
            f"length mismatch (rendered {r.size} vs wet {w.size} samples); "
            f"compared the first {n}"
        )
        r = r[:n]
        w = w[:n]

    si_sdr = _si_sdr_db(r, w)
    if si_sdr is None:
        limitations.append("si-SDR undefined: wet reference is (near-)silent")

    est_mag = _stft_magnitudes(r)
    ref_mag = _stft_magnitudes(w)
    lsd = _log_spectral_distance(est_mag, ref_mag)
    freqs = np.fft.rfftfreq(_N_FFT, d=1.0 / sr)
    notes.extend(_band_notes(est_mag, ref_mag, freqs))

    loud_r, padded_r = _loudness_lufs(r, sr)
    loud_w, padded_w = _loudness_lufs(w, sr)
    if loud_r is not None and loud_w is not None:
        loud_diff: float | None = float(loud_r - loud_w)
        if abs(loud_diff) >= 1.0:
            direction = "louder" if loud_diff > 0.0 else "quieter"
            notes.append(
                f"rendered is {abs(loud_diff):.1f} LU {direction} than wet"
            )
    else:
        loud_diff = None
        limitations.append(
            "integrated loudness unavailable (signal silent or gating failed)"
        )
    if padded_r or padded_w:
        notes.append(
            "signal shorter than the 400 ms loudness block; zero-padded for measurement"
        )

    if _tail_active(w, sr):
        limitations.append(
            "significant energy at the end of the render window; "
            "any effect tail beyond the window is not scored"
        )

    limitations.append(
        "offline render approximates the firmware DSP; 128-sample block "
        "processing and fixed-point rounding are not modeled"
    )

    return FitReport(
        si_sdr_db=si_sdr,
        log_spectral_distance=lsd,
        loudness_diff_lufs=loud_diff,
        stage_notes=notes,
        limitations=limitations,
    )


# -------------------------------------------------------------------- verify

def _shift_to_length(x: np.ndarray, lag: int, out_len: int) -> np.ndarray:
    """y[n] = x[n - lag] with zero fill, length out_len (wet[n] ≈ rendered[n - lag])."""
    y = np.zeros(out_len)
    idx = np.arange(out_len) - lag
    valid = (idx >= 0) & (idx < x.size)
    y[valid] = x[idx[valid]]
    return y


def _align_lag(rendered: np.ndarray, wet: np.ndarray, sr: int) -> tuple[int, bool]:
    """Cross-correlation lag L with wet[n] ≈ rendered[n - L]; (lag, hit_bound)."""
    max_lag = int(min(0.5 * sr, max(1, min(rendered.size, wet.size) // 2)))
    corr = sps.correlate(wet, rendered, mode="full", method="fft")
    lags = sps.correlation_lags(wet.size, rendered.size, mode="full")
    window = np.abs(lags) <= max_lag
    sub_lags = lags[window]
    sub_corr = corr[window]
    lag = int(sub_lags[int(np.argmax(np.abs(sub_corr)))])
    hit_bound = abs(lag) >= max_lag
    return lag, hit_bound


def _chain_limitations(chain: ChainModel, sr: int, n_samples: int, fit: FitReport) -> list[str]:
    """Verify-stage caveats derived from the proposed chain vs the render window."""
    lims: list[str] = []
    dur = n_samples / sr
    for i, stage in enumerate(chain.stages):
        if isinstance(stage, ReverbModel) and stage.rt60_s > 0.25 * dur:
            lims.append(
                f"stage {i} (reverb): tail rt60_s={stage.rt60_s:.2f} s may "
                f"extend beyond the render window ({dur:.2f} s)"
            )
        elif isinstance(stage, DelayModel) and stage.time_ms > 0.0:
            span = (stage.time_ms * 0.001) / max(1.0 - stage.feedback, 0.05)
            if span > dur:
                lims.append(
                    f"stage {i} (delay): repeats span up to {span:.2f} s, "
                    f"which exceeds the render window ({dur:.2f} s)"
                )
        elif isinstance(stage, BitcrushModel):
            lims.append(
                f"stage {i} (bitcrush): rendered in float64; firmware "
                "fixed-point rounding may add a different noise floor"
            )
    if (fit.si_sdr_db is not None and fit.si_sdr_db < 5.0) or (
        fit.log_spectral_distance is not None and fit.log_spectral_distance > 3.0
    ):
        lims.append(
            "whole-chain residual is large; stage-level attribution of the "
            "error was not computed"
        )
    return lims[:6]


def verify_chain(dry: np.ndarray, wet: np.ndarray, sr: int, chain: ChainModel) -> FitReport:
    """render_chain + score_render in one call (align rendered vs wet by cross-correlation before scoring, since effect chains may introduce latency)."""
    sr = _check_sr(sr)
    d = _validate_mono(dry, "dry")
    w = _validate_mono(wet, "wet")
    if not isinstance(chain, ChainModel):
        raise TypeError(f"chain must be a ChainModel, got {type(chain).__name__}")

    rendered = render_chain(d, sr, chain).astype(np.float64)
    lag, hit_bound = _align_lag(rendered, w, sr)
    aligned = _shift_to_length(rendered, lag, w.size)

    report = score_render(aligned, w, sr)
    if lag != 0:
        report.stage_notes.insert(
            0,
            f"cross-correlation alignment: rendered shifted by {lag} samples "
            f"({lag * 1000.0 / sr:.1f} ms) before scoring",
        )
    if hit_bound:
        report.limitations.append(
            "latency estimate hit the alignment search bound; "
            "rendered-vs-wet alignment may be wrong"
        )
    report.limitations.extend(_chain_limitations(chain, sr, w.size, report))
    return report
