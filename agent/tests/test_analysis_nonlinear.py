"""Waveshaper tests: tanh recovery and hysteresis rejection."""
from __future__ import annotations

import numpy as np
from scipy import signal

from ocapp_agent.analysis import TARGET_SR, measure_waveshaper

from test_analysis_support import make_noise


def test_tanh_waveshaper_recovered() -> None:
    sr = TARGET_SR
    dry = make_noise(3 * sr, seed=4) * 0.5
    wet = np.tanh(2.0 * dry)

    m = measure_waveshaper(dry, wet, sr)

    assert m is not None, "waveshaper not detected"
    assert m.curve == "tanh"
    assert m.drive_db > 0.0
    # True drive is 20*log10(2) = 6.02 dB.
    assert abs(m.drive_db - 6.02) <= 2.0


def test_soft_clip_detected_as_nonlinear() -> None:
    sr = TARGET_SR
    dry = make_noise(3 * sr, seed=5) * 0.5
    g = 2.5
    wet = np.sign(dry) * (1.0 - np.exp(-g * np.abs(dry)))

    m = measure_waveshaper(dry, wet, sr)

    assert m is not None
    assert m.curve in ("soft_clip", "tanh", "asymmetric", "table")
    if m.curve != "table":
        assert abs(m.drive_db - 20.0 * np.log10(g)) <= 3.0


def test_hysteresis_loop_returns_none_or_table() -> None:
    sr = TARGET_SR
    dry = make_noise(2 * sr, seed=3) * 0.5
    # 90-degree phase-shifted (Hilbert) copy: memory, not a memoryless curve.
    wet = dry + 0.3 * np.imag(signal.hilbert(dry))

    m = measure_waveshaper(dry, wet, sr)

    assert m is None or m.curve == "table"


def test_linear_gain_returns_none() -> None:
    sr = TARGET_SR
    dry = make_noise(2 * sr, seed=6) * 0.5
    assert measure_waveshaper(dry, 1.7 * dry, sr) is None
