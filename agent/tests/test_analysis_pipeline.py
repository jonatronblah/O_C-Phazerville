"""analyze_pair integration tests: combined chain + report population."""
from __future__ import annotations

import numpy as np

from ocapp_agent.analysis import (
    TARGET_SR,
    analyze_pair,
    apply_biquads,
    detect_parallel_mix,
    load_audio,
    match_loudness,
    align_pair,
)
from ocapp_agent.models import AnalysisReport, ChainModel

from test_analysis_support import echo_train, make_noise, peaking_stage


def _chain_pair() -> tuple[np.ndarray, np.ndarray, int]:
    """dry -> peaking EQ -> parallel delay comb, with a known input lag."""
    sr = TARGET_SR
    dry = make_noise(4 * sr, seed=6)
    x = apply_biquads(dry, [peaking_stage(1000.0, 2.0, 8.0)], sr)
    train = echo_train(x, time_s=0.150, feedback=0.35, n_echoes=6, sr=sr)
    wet = 0.5 * x + 0.5 * train
    shift = 237
    wet = np.concatenate([np.zeros(shift), wet])[: wet.size]
    return dry, wet, shift


def test_analyze_pair_eq_plus_delay() -> None:
    sr = TARGET_SR
    dry, wet, shift = _chain_pair()

    chain, report = analyze_pair(dry, wet, sr)

    assert isinstance(chain, ChainModel)
    assert isinstance(report, AnalysisReport)
    kinds = [s.kind for s in chain.stages]
    assert "eq" in kinds, kinds
    assert "delay" in kinds, kinds

    eq = next(s for s in chain.stages if s.kind == "eq")
    assert any(
        abs(st.freq_hz - 1000.0) / 1000.0 <= 0.12 and abs(st.gain_db - 8.0) <= 1.5
        for st in eq.stages
    ), [(st.kind, st.freq_hz, st.gain_db) for st in eq.stages]

    delay = next(s for s in chain.stages if s.kind == "delay")
    assert abs(delay.time_ms - 150.0) <= 8.0, delay.time_ms
    assert abs(delay.feedback - 0.35) <= 0.2, delay.feedback

    # Report fields.
    assert abs(report.lag_samples - shift) <= 2, report.lag_samples
    assert report.is_parallel_mix is True
    assert np.isfinite(report.gain_db_applied)
    assert np.isfinite(report.dry_rms_db) and np.isfinite(report.wet_rms_db)
    for key in ("lag_samples", "gain_db_applied", "eq_fit_rms_db", "ir_peak_db"):
        assert key in report.measurements, sorted(report.measurements)

    # No bogus stages for the constructed chain.
    for kind in ("waveshaper", "bitcrush", "reverb"):
        assert kind not in kinds, kinds


def test_analyze_pair_plain_gain() -> None:
    sr = TARGET_SR
    dry = make_noise(2 * sr, seed=15)
    chain, report = analyze_pair(dry, 0.5 * dry, sr)
    assert chain.stages == []
    assert report.is_parallel_mix is False
    assert abs(report.lag_samples) <= 1


def test_align_pair_recovers_shift() -> None:
    sr = TARGET_SR
    dry = make_noise(sr, seed=16)
    shift = 512
    wet = np.concatenate([np.zeros(shift), dry])[: dry.size]
    d_a, w_a, lag = align_pair(dry, wet)
    assert lag == shift
    assert d_a.size == w_a.size == dry.size - shift


def test_match_loudness_matches_levels() -> None:
    sr = TARGET_SR
    dry = make_noise(2 * sr, seed=17)
    wet = 4.0 * dry
    scaled, gain_db = match_loudness(dry, wet, sr)
    assert abs(gain_db + 20.0 * np.log10(4.0)) <= 1.5
    from ocapp_agent.analysis.common import rms

    assert abs(rms(scaled) - rms(dry)) / rms(dry) <= 0.1


def test_detect_parallel_mix_flags_blend() -> None:
    sr = TARGET_SR
    dry = make_noise(2 * sr, seed=18)
    delayed = np.concatenate([np.zeros(int(0.05 * sr)), dry])[: dry.size]
    assert detect_parallel_mix(dry, 0.5 * dry + 0.5 * delayed, sr) is True
    assert detect_parallel_mix(dry, 0.9 * dry, sr) is False


def _e2e_repro_pair() -> tuple[np.ndarray, np.ndarray]:
    """The exact cross-lane smoke scenario: tonal dry, +8dB peaking @ 1kHz
    Q=2, then 120ms delay with wet-feedback 0.35 and mix 0.5.

    Built with the same recurrence as tests/e2e_smoke.py:
    wet[n] = 0.5*eqd[n] + 0.5*(eqd[n-T] + 0.35*wet[n-T]) for n >= T
    (steering the regression at the reported bug: delay swallowed by the EQ).
    """
    from scipy.signal import lfilter

    rng = np.random.default_rng(7)
    sr = TARGET_SR
    t = np.arange(sr * 2) / sr
    dry = (
        0.3 * np.sin(2 * np.pi * 220 * t)
        + 0.15 * np.sin(2 * np.pi * 550 * t)
        + 0.05 * rng.standard_normal(len(t))
    )
    a_gain = 10 ** (8 / 40)
    w0 = 2 * np.pi * 1000 / sr
    alpha = np.sin(w0) / (2 * 2.0)
    b = [1 + alpha * a_gain, -2 * np.cos(w0), 1 - alpha * a_gain]
    a = [1 + alpha / a_gain, -2 * np.cos(w0), 1 - alpha / a_gain]
    eqd = lfilter(b, a, dry)
    d_samp = int(0.120 * sr)
    wet = np.zeros_like(eqd)
    for n in range(len(eqd)):
        fb = wet[n - d_samp] * 0.35 if n >= d_samp else 0.0
        wet[n] = 0.5 * eqd[n] + 0.5 * (eqd[n - d_samp] + fb if n >= d_samp else eqd[n])
    return dry, wet


def test_analyze_pair_eq_plus_delay_mix05_regression() -> None:
    """Regression: delay after EQ at mix=0.5 must not be swallowed by the EQ."""
    sr = TARGET_SR
    dry, wet = _e2e_repro_pair()

    chain, report = analyze_pair(dry, wet, sr)

    kinds = [s.kind for s in chain.stages]
    assert "eq" in kinds, kinds
    assert "delay" in kinds, kinds

    # Parsimonious EQ: ~1 peaking stage near 1 kHz +8 dB (not comb ripple).
    eq = next(s for s in chain.stages if s.kind == "eq")
    assert 1 <= len(eq.stages) <= 2, [(s.kind, s.freq_hz, s.gain_db) for s in eq.stages]
    assert any(
        abs(st.freq_hz - 1000.0) / 1000.0 <= 0.15 and abs(st.gain_db - 8.0) <= 1.0
        for st in eq.stages
    ), [(st.kind, st.freq_hz, st.gain_db) for st in eq.stages]

    delay = next(s for s in chain.stages if s.kind == "delay")
    assert abs(delay.time_ms - 120.0) <= 8.0, delay.time_ms
    assert abs(delay.feedback - 0.35) <= 0.2, delay.feedback
    assert 0.3 <= delay.mix <= 0.7, delay.mix

    # Alignment must anchor on the direct path (the repro has no global shift).
    assert abs(report.lag_samples) <= 2, report.lag_samples
    # IR metrics must be meaningful again (was ~-57 dB / full window).
    assert report.measurements["ir_peak_db"] > -12.0, report.measurements
    assert report.measurements["ir_tail_s"] < 1.5, report.measurements


def test_analyze_pair_delay_weaker_echo_mix035() -> None:
    """Weaker echo (mix=0.35): delay must still be detected with sane params.

    Detection floor (documented): an isolated echo at or above -26 dB relative
    to the direct peak with quiet gaps between peaks; 0.35/0.65 amplitude is
    well above that.
    """
    sr = TARGET_SR
    dry = make_noise(4 * sr, seed=21)
    x = apply_biquads(dry, [peaking_stage(1000.0, 2.0, 8.0)], sr)
    train = echo_train(x, time_s=0.120, feedback=0.35, n_echoes=8, sr=sr)
    wet = 0.65 * x + 0.35 * train

    chain, report = analyze_pair(dry, wet, sr)

    delay = next((s for s in chain.stages if s.kind == "delay"), None)
    assert delay is not None, [s.kind for s in chain.stages]
    assert abs(delay.time_ms - 120.0) <= 8.0, delay.time_ms
    assert abs(delay.feedback - 0.35) <= 0.2, delay.feedback
    assert abs(delay.mix - 0.35) <= 0.15, delay.mix
