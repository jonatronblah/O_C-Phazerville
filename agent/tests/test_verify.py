"""Tests for ocapp_agent.verify: offline digital-twin render + scoring.

Tolerances are pragmatic: we assert directionally meaningful behavior (good fit
good, bad fit bad) over exact values, since the render is an approximation of
firmware DSP.
"""
import numpy as np
import pytest
from scipy.signal import welch

from ocapp_agent.models import (
    BiquadStage,
    BitcrushModel,
    ChainModel,
    CompressorModel,
    DelayModel,
    EqModel,
    FitReport,
    ReverbModel,
    WaveshaperModel,
)
from ocapp_agent.verify import render_chain, score_render, verify_chain

SR = 48000


def _rms(x: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(x))))


# ------------------------------------------------------------------- 1. identity

def test_identity_empty_chain_matches_dry() -> None:
    rng = np.random.default_rng(1)
    dry = (0.25 * rng.standard_normal(int(0.3 * SR))).astype(np.float32)
    rendered = render_chain(dry, SR, ChainModel())
    assert rendered.dtype == np.float32
    assert rendered.shape == dry.shape
    assert np.allclose(rendered, dry, atol=1e-6)
    # deterministic: same input -> identical output
    again = render_chain(dry, SR, ChainModel())
    assert np.array_equal(rendered, again)


def test_output_gain_db_applied() -> None:
    rng = np.random.default_rng(2)
    dry = 0.25 * rng.standard_normal(int(0.1 * SR))
    rendered = render_chain(dry, SR, ChainModel(output_gain_db=-6.0))
    expected_gain = 10.0 ** (-6.0 / 20.0)  # -6 dB ≈ 0.5012
    assert np.allclose(rendered.astype(np.float64), dry * expected_gain, atol=1e-5)


# ------------------------------------------------------------------------- 2. EQ

def test_eq_peaking_bump_visible_in_spectrum_and_self_fit_is_good() -> None:
    rng = np.random.default_rng(3)
    dry = 0.3 * rng.standard_normal(int(0.5 * SR))
    chain = ChainModel(
        stages=[
            EqModel(
                stages=[
                    BiquadStage(kind="peaking", freq_hz=1000.0, q=2.0, gain_db=8.0)
                ]
            )
        ]
    )
    rendered = render_chain(dry, SR, chain).astype(np.float64)

    f, p_d = welch(dry, fs=SR, nperseg=4096)
    _, p_r = welch(rendered, fs=SR, nperseg=4096)
    bump = (f >= 900.0) & (f <= 1100.0)
    ctrl = (f >= 4000.0) & (f <= 6000.0)
    gain_bump_db = 10.0 * np.log10(np.mean(p_r[bump]) / np.mean(p_d[bump]))
    gain_ctrl_db = 10.0 * np.log10(np.mean(p_r[ctrl]) / np.mean(p_d[ctrl]))
    assert 6.0 <= gain_bump_db <= 10.0, f"bump gain {gain_bump_db:.2f} dB"
    assert -1.5 <= gain_ctrl_db <= 1.5, f"control gain {gain_ctrl_db:.2f} dB"

    # scoring a known render against that same wet must be an excellent fit
    rep = score_render(rendered, rendered.copy(), SR)
    assert isinstance(rep, FitReport)
    assert rep.si_sdr_db is not None and rep.si_sdr_db > 20


def test_eq_all_biquad_kinds_render_finite() -> None:
    dry = 0.3 * np.sin(2 * np.pi * 440.0 * np.arange(int(0.05 * SR)) / SR)
    for kind in ("peaking", "lowshelf", "highshelf", "lowpass", "highpass", "bandpass", "notch"):
        chain = ChainModel(
            stages=[EqModel(stages=[BiquadStage(kind=kind, freq_hz=1000.0, q=0.9, gain_db=6.0)])]
        )
        out = render_chain(dry, SR, chain)
        assert out.shape == dry.shape
        assert np.all(np.isfinite(out))


# ----------------------------------------------------------------------- 3. delay

def test_delay_autocorrelation_peak_at_100ms() -> None:
    rng = np.random.default_rng(4)
    dry = 0.3 * rng.standard_normal(int(0.6 * SR))
    chain = ChainModel(
        stages=[DelayModel(time_ms=100.0, feedback=0.3, mix=1.0)]
    )
    y = render_chain(dry, SR, chain).astype(np.float64)

    n = y.size
    spec = np.fft.rfft(y, 2 * n)
    ac = np.fft.irfft(spec * np.conj(spec))[:n]
    ac /= ac[0]
    lo, hi = int(0.02 * SR), int(0.25 * SR)  # search 20 ms .. 250 ms
    peak = (lo + int(np.argmax(ac[lo:hi]))) / SR * 1000.0
    assert abs(peak - 100.0) <= 1.0, f"autocorrelation peak at {peak:.2f} ms"


def test_delay_mix_one_is_fully_wet_with_feedback_and_tone() -> None:
    imp = np.zeros(int(0.5 * SR))
    imp[0] = 1.0
    chain = ChainModel(stages=[DelayModel(time_ms=100.0, feedback=0.3, mix=1.0)])
    y = render_chain(imp, SR, chain).astype(np.float64)
    # fully wet: silence before the first delay tap
    assert float(np.max(np.abs(y[:4700]))) < 1e-6
    assert y[4800] == pytest.approx(1.0, abs=1e-5)  # 100 ms = 4800 samples
    assert y[9600] == pytest.approx(0.3, abs=1e-4)  # first feedback repeat

    # mix=0.0 is pure dry
    dry_chain = ChainModel(stages=[DelayModel(time_ms=100.0, feedback=0.3, mix=0.0)])
    y_dry = render_chain(imp, SR, dry_chain)
    assert np.allclose(y_dry, imp.astype(np.float32), atol=1e-7)

    # tone_hz LPF in the feedback path darkens the repeats
    lp_chain = ChainModel(stages=[DelayModel(time_ms=100.0, feedback=0.3, mix=1.0, tone_hz=500.0)])
    y_lp = render_chain(imp, SR, lp_chain).astype(np.float64)
    assert 0.0 < y_lp[9600] < 0.3


# ------------------------------------------------------------------ 4. waveshaper

def test_waveshaper_table_reproduced_within_interp_tolerance() -> None:
    rng = np.random.default_rng(5)
    table = np.tanh(np.linspace(-2.0, 2.0, 33))  # monotone -1..1 transfer curve
    dry = rng.uniform(-1.0, 1.0, 500)
    chain = ChainModel(
        stages=[
            WaveshaperModel(curve="table", table=table.tolist(), drive_db=0.0, mix=1.0)
        ]
    )
    rendered = render_chain(dry, SR, chain).astype(np.float64)
    expected = np.interp(dry, np.linspace(-1.0, 1.0, table.size), table)
    assert np.allclose(rendered, expected, atol=1e-5)


def test_waveshaper_curves_behave_as_documented() -> None:
    x = np.array([0.5, -0.5, 1.5, -1.5])

    hard = render_chain(x, SR, ChainModel(stages=[WaveshaperModel(curve="hard_clip", mix=1.0)]))
    assert np.allclose(hard, np.array([0.5, -0.5, 1.0, -1.0]), atol=1e-6)

    soft = render_chain(x, SR, ChainModel(stages=[WaveshaperModel(curve="soft_clip", mix=1.0)]))
    assert np.allclose(soft.astype(np.float64)[2], 1.5 / 2.5, atol=1e-6)

    asym = render_chain(
        np.array([0.5, -0.5]), SR, ChainModel(stages=[WaveshaperModel(curve="asymmetric", mix=1.0)])
    ).astype(np.float64)
    assert abs(asym[0]) > 0.7 and abs(asym[1]) < 0.5  # + and - halves differ
    assert abs(asym[0]) != abs(asym[1])


# ------------------------------------------------------------------ 5. compressor

def test_compressor_loud_attenuated_per_ratio_quiet_untouched() -> None:
    n = int(0.5 * SR)
    t = np.arange(n) / SR
    loud = 0.9 * np.sin(2 * np.pi * 200.0 * t)
    quiet = 0.05 * np.sin(2 * np.pi * 200.0 * t)
    chain = ChainModel(
        stages=[
            CompressorModel(
                threshold_db=-18.0, ratio=4.0, attack_ms=5.0, release_ms=50.0, knee_db=6.0
            )
        ]
    )
    y_loud = render_chain(loud, SR, chain).astype(np.float64)
    y_quiet = render_chain(quiet, SR, chain).astype(np.float64)

    steady = slice(n // 2, None)  # skip the attack transient
    red_loud = 20.0 * np.log10(_rms(loud[steady]) / _rms(y_loud[steady]))
    red_quiet = 20.0 * np.log10(_rms(quiet[steady]) / _rms(y_quiet[steady]))

    # expected static reduction: (1 - 1/ratio) * (level - threshold) above knee
    expected = (1.0 - 1.0 / 4.0) * (20.0 * np.log10(0.9) + 18.0)  # ~12.8 dB
    assert abs(red_loud - expected) < 4.0, f"loud reduction {red_loud:.2f} dB"
    assert abs(red_quiet) < 1.5, f"quiet reduction {red_quiet:.2f} dB"
    assert red_loud > red_quiet + 6.0


# ------------------------------------------------------------------- 6. bitcrush

def test_bitcrush_4bit_has_at_most_16_levels() -> None:
    ramp = np.linspace(-1.0, 1.0, 1001)
    chain = ChainModel(stages=[BitcrushModel(bit_depth=4, downsample=1)])
    out = render_chain(ramp, SR, chain)
    assert np.unique(out).size <= 16


def test_bitcrush_downsample_is_sample_and_hold() -> None:
    rng = np.random.default_rng(6)
    x = rng.uniform(-1.0, 1.0, 256)
    chain = ChainModel(stages=[BitcrushModel(bit_depth=8, downsample=4)])
    out = render_chain(x, SR, chain)
    assert np.all(out[:4] == out[0])
    assert np.all(out[4:8] == out[4])


# --------------------------------------------------------------------- 7. reverb

def test_reverb_noise_burst_decays() -> None:
    rng = np.random.default_rng(7)
    n = int(0.4 * SR)
    x = np.zeros(n)
    burst_len = int(0.03 * SR)
    x[:burst_len] = 0.5 * rng.standard_normal(burst_len)
    chain = ChainModel(
        stages=[
            ReverbModel(rt60_s=0.25, predelay_ms=0.0, damping=0.5, room_size=0.5, mix=1.0)
        ]
    )
    y = render_chain(x, SR, chain).astype(np.float64)

    def win_rms(t0: float, t1: float) -> float:
        return _rms(y[int(t0 * SR) : int(t1 * SR)])

    e_on = win_rms(0.0, 0.08)
    e_mid = win_rms(0.12, 0.20)
    e_late = win_rms(0.20, 0.28)
    assert e_on > 0.0
    assert 20.0 * np.log10(e_late / e_on) < -15.0  # far below onset at +0.2 s
    assert e_late < e_mid  # still decaying


# ----------------------------------------------------------------- 8. verify_chain

def test_verify_chain_self_render_is_strong_fit_unrelated_is_poor() -> None:
    rng = np.random.default_rng(11)
    dry = 0.3 * rng.standard_normal(int(0.5 * SR))
    chain = ChainModel(
        stages=[
            EqModel(
                stages=[BiquadStage(kind="peaking", freq_hz=1000.0, q=1.5, gain_db=6.0)]
            ),
            DelayModel(time_ms=50.0, feedback=0.2, mix=0.4),
        ]
    )
    wet = render_chain(dry, SR, chain).astype(np.float64)

    good = verify_chain(dry, wet, SR, chain)
    assert isinstance(good, FitReport)
    assert good.si_sdr_db is not None and good.si_sdr_db > 25
    assert good.log_spectral_distance is not None and good.log_spectral_distance < 0.5

    unrelated = 0.3 * rng.standard_normal(dry.size)
    bad = verify_chain(dry, unrelated, SR, chain)
    assert bad.si_sdr_db is None or bad.si_sdr_db < 5
    assert bad.log_spectral_distance is not None and bad.log_spectral_distance > 3
    assert good.log_spectral_distance < bad.log_spectral_distance


def test_verify_chain_aligns_latency() -> None:
    rng = np.random.default_rng(12)
    dry = 0.3 * rng.standard_normal(int(0.5 * SR))
    chain = ChainModel(stages=[EqModel(stages=[])])
    wet = render_chain(dry, SR, chain).astype(np.float64)
    # wet arrives 3 ms late relative to the render (effect-chain latency)
    delayed_wet = np.concatenate([np.zeros(int(0.003 * SR)), wet])[: wet.size]
    rep = verify_chain(dry, delayed_wet, SR, chain)
    assert rep.si_sdr_db is not None and rep.si_sdr_db > 25


# --------------------------------------------------------------- 9. self-distance

def test_score_render_self_distance_is_zero() -> None:
    rng = np.random.default_rng(13)
    dry = 0.3 * rng.standard_normal(SR)  # 1 s: comfortably past the loudness block
    chain = ChainModel(
        stages=[WaveshaperModel(curve="tanh", drive_db=4.0, mix=0.8)]
    )
    wet = render_chain(dry, SR, chain).astype(np.float64)

    rep = score_render(wet.copy(), wet, SR)
    # si_sdr is a similarity: self-score is at the perfection end (clamped cap)
    assert rep.si_sdr_db is not None and rep.si_sdr_db >= 40
    assert rep.log_spectral_distance is not None
    assert rep.log_spectral_distance == pytest.approx(0.0, abs=1e-6)
    assert rep.loudness_diff_lufs is not None
    assert rep.loudness_diff_lufs == pytest.approx(0.0, abs=0.1)
    assert rep.limitations  # verify-stage caveats always documented


# ------------------------------------------------------------- contract behaviour

def test_render_and_score_reject_empty_and_nan_inputs() -> None:
    chain = ChainModel()
    with pytest.raises(ValueError):
        render_chain(np.array([]), SR, chain)
    nan_sig = np.zeros(100)
    nan_sig[5] = np.nan
    with pytest.raises(ValueError):
        render_chain(nan_sig, SR, chain)
    with pytest.raises(ValueError):
        score_render(np.zeros(10), np.array([]), SR)
    inf_sig = np.zeros(10)
    inf_sig[2] = np.inf
    with pytest.raises(ValueError):
        score_render(inf_sig, np.zeros(10), SR)
    with pytest.raises(ValueError):
        verify_chain(nan_sig, np.zeros(100), SR, chain)


def test_delay_rejects_unstable_feedback() -> None:
    x = np.zeros(100)
    x[0] = 1.0
    stage = DelayModel(time_ms=10.0, feedback=0.5, mix=1.0)
    bad = stage.model_copy(update={"feedback": 1.5})  # bypass schema guard
    with pytest.raises(ValueError):
        render_chain(x, SR, ChainModel(stages=[bad]))
