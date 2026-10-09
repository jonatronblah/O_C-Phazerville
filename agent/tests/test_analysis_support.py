"""Shared synthetic-signal helpers for the analysis test suite.

Every generator is seeded, so all tests are deterministic.
"""
from __future__ import annotations

import numpy as np
from scipy import signal

from ocapp_agent.analysis import TARGET_SR, apply_biquads, design_biquad
from ocapp_agent.models import BiquadStage


def make_noise(n: int, seed: int, scale: float = 1.0) -> np.ndarray:
    rng = np.random.default_rng(seed)
    return (rng.standard_normal(n) * scale).astype(np.float64)


def peaking_stage(freq_hz: float = 1000.0, q: float = 2.0, gain_db: float = 8.0) -> BiquadStage:
    return BiquadStage(kind="peaking", freq_hz=freq_hz, q=q, gain_db=gain_db)


def echo_train(
    x: np.ndarray, time_s: float, feedback: float, n_echoes: int = 8, sr: int = TARGET_SR
) -> np.ndarray:
    """Sum of delayed copies: taps feedback**k at (k+1)*time_s (no dry path)."""
    out = np.zeros_like(x)
    for k in range(1, n_echoes + 1):
        shift = int(round(k * time_s * sr))
        if shift >= x.size:
            break
        out[shift:] += (feedback ** (k - 1)) * x[: x.size - shift]
    return out


def quantize(x: np.ndarray, bits: int) -> np.ndarray:
    """Uniform mid-tread quantizer with 2**(bits-1) levels per sign."""
    levels = 2 ** (bits - 1)
    return np.round(x * levels) / levels


def zoh_downsample(x: np.ndarray, factor: int) -> np.ndarray:
    """Zero-order-hold downsample by ``factor``."""
    m = x.size // factor
    return np.repeat(x[::factor][:m], factor)[: x.size]


class TestInputValidation:
    def test_nan_raises(self) -> None:
        x = make_noise(4800, seed=0)
        x[10] = np.nan
        import pytest

        from ocapp_agent.analysis import analyze_pair

        with pytest.raises(ValueError, match="NaN"):
            analyze_pair(x, make_noise(4800, seed=1))

    def test_silence_raises(self) -> None:
        import pytest

        from ocapp_agent.analysis import measure_eq

        with pytest.raises(ValueError, match="silent"):
            measure_eq(np.zeros(4800), make_noise(4800, seed=1))

    def test_empty_raises(self) -> None:
        import pytest

        from ocapp_agent.analysis import align_pair

        with pytest.raises(ValueError):
            align_pair(np.zeros(0), make_noise(4800, seed=1))


class TestLoadAudio:
    def test_load_resamples_and_downmixes(self, tmp_path) -> None:
        import soundfile as sf

        from ocapp_agent.analysis import load_audio

        sr_in = 44100
        n = sr_in  # 1 s
        t = np.arange(n) / sr_in
        stereo = np.stack([np.sin(2 * np.pi * 440 * t), np.sin(2 * np.pi * 440 * t)], axis=1)
        path = tmp_path / "tone.wav"
        sf.write(str(path), stereo.astype(np.float32), sr_in)

        y, sr = load_audio(path)
        assert sr == TARGET_SR
        assert y.dtype == np.float32
        assert y.ndim == 1
        assert abs(y.size - TARGET_SR) <= 8
        # Still a 440 Hz tone after resampling: dominant peak near 440.
        spec = np.abs(np.fft.rfft(y * np.hanning(y.size)))
        freqs = np.fft.rfftfreq(y.size, 1.0 / TARGET_SR)
        assert abs(freqs[np.argmax(spec)] - 440.0) < 5.0

    def test_missing_file_raises(self, tmp_path) -> None:
        import pytest

        from ocapp_agent.analysis import load_audio

        with pytest.raises(FileNotFoundError):
            load_audio(tmp_path / "nope.wav")
