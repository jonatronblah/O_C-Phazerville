"""Dynamics tests: envelope-regression recovery of a known soft compressor."""
from __future__ import annotations

import numpy as np

from ocapp_agent.analysis import TARGET_SR, measure_dynamics

from test_analysis_support import make_noise


def _soft_knee_gain_db(over_db: np.ndarray, ratio: float, knee_db: float) -> np.ndarray:
    half = knee_db / 2.0
    gain = np.zeros_like(over_db)
    mid = np.abs(over_db) <= half
    high = over_db > half
    gain[mid] = (1.0 / ratio - 1.0) * (over_db[mid] + half) ** 2 / (2.0 * knee_db)
    gain[high] = (1.0 / ratio - 1.0) * over_db[high]
    return gain


def test_measure_dynamics_recovers_ratio() -> None:
    sr = TARGET_SR
    n = 6 * sr
    t = np.arange(n) / sr
    env_db = -30.0 + 18.0 * np.sin(2.0 * np.pi * 0.3 * t)  # spans -48 .. -12 dB
    env = 10.0 ** (env_db / 20.0)
    dry = make_noise(n, seed=9) * env

    theta, ratio, knee = -20.0, 4.0, 6.0
    gain_db = _soft_knee_gain_db(env_db - theta, ratio, knee)
    wet = dry * 10.0 ** (gain_db / 20.0)

    m = measure_dynamics(dry, wet, sr)

    assert m is not None, "compression not detected"
    assert abs(m.ratio - ratio) <= 1.5, m.ratio
    assert abs(m.threshold_db - theta) <= 8.0, m.threshold_db
    assert m.ratio > 1.0
    assert m.attack_ms > 0.0 and m.release_ms > 0.0
    assert 0.0 <= m.knee_db <= 18.0


def test_measure_dynamics_none_for_linear_gain() -> None:
    sr = TARGET_SR
    n = 3 * sr
    t = np.arange(n) / sr
    env = 10.0 ** ((-30.0 + 15.0 * np.sin(2.0 * np.pi * 0.5 * t)) / 20.0)
    dry = make_noise(n, seed=10) * env

    assert measure_dynamics(dry, 0.5 * dry, sr) is None
