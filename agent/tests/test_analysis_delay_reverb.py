"""Delay and reverb estimation tests on synthetic comb / decaying-tail IRs."""
from __future__ import annotations

import numpy as np
from scipy import signal

from ocapp_agent.analysis import TARGET_SR, measure_delay, measure_reverb

from test_analysis_support import echo_train, make_noise


def test_measure_delay_recovers_comb() -> None:
    sr = TARGET_SR
    dry = make_noise(5 * sr, seed=2)
    wet = echo_train(dry, time_s=0.120, feedback=0.4, n_echoes=8, sr=sr)

    m = measure_delay(dry, wet, sr)

    assert m is not None, "delay not detected"
    assert abs(m.time_ms - 120.0) <= 5.0
    assert abs(m.feedback - 0.4) <= 0.15
    assert 0.0 <= m.mix <= 1.0


def test_measure_delay_slapback_has_zero_feedback() -> None:
    sr = TARGET_SR
    dry = make_noise(4 * sr, seed=7)
    wet = np.zeros_like(dry)
    shift = int(0.200 * sr)
    wet[shift:] = dry[: dry.size - shift]
    wet[2 * shift:] += 0.5 * dry[: dry.size - 2 * shift]

    m = measure_delay(dry, wet, sr)
    assert m is not None
    assert abs(m.time_ms - 200.0) <= 6.0
    assert m.feedback <= 0.15  # echo-to-echo: only one echo -> 0


def test_measure_delay_none_for_plain_gain() -> None:
    sr = TARGET_SR
    dry = make_noise(3 * sr, seed=8)
    assert measure_delay(dry, 0.5 * dry, sr) is None


def test_measure_reverb_dense_tail() -> None:
    sr = TARGET_SR
    rng = np.random.default_rng(11)
    n = 6 * sr
    dry = make_noise(n, seed=11)

    # Synthetic exponential-noise tail designed for rt60 = 0.8 s.
    rt60 = 0.8
    tau = rt60 / (3.0 * np.log(10.0))
    tail_len = int(2.5 * sr)
    t = np.arange(tail_len) / sr
    tail = rng.standard_normal(tail_len) * np.exp(-t / tau)
    tail /= np.max(np.abs(tail))  # keep the direct peak dominant in the IR
    tail[: int(0.010 * sr)] = 0.0  # 10 ms predelay in the tail itself
    wet = 0.7 * dry + 0.3 * signal.fftconvolve(dry, tail, mode="full")[:n]

    m = measure_reverb(dry, wet, sr)

    assert m is not None, "reverb not detected"
    assert 0.4 <= m.rt60_s <= 1.6, m.rt60_s
    assert 0.0 <= m.damping <= 1.0
    assert 0.0 <= m.room_size <= 1.0
    assert 0.0 <= m.mix <= 1.0


def test_measure_reverb_none_for_delay_comb() -> None:
    sr = TARGET_SR
    dry = make_noise(5 * sr, seed=12)
    wet = echo_train(dry, time_s=0.120, feedback=0.4, n_echoes=8, sr=sr)
    assert measure_reverb(dry, wet, sr) is None
