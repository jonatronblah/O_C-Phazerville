"""EQ estimation tests: recovers a known peaking biquad from white noise."""
from __future__ import annotations

import numpy as np

from ocapp_agent.analysis import TARGET_SR, apply_biquads, measure_eq, stages_response_db
from ocapp_agent.models import BiquadStage

from test_analysis_support import make_noise, peaking_stage


def test_measure_eq_recovers_peaking_stage() -> None:
    sr = TARGET_SR
    dry = make_noise(4 * sr, seed=1)
    stages_in = [peaking_stage(freq_hz=1000.0, q=2.0, gain_db=8.0)]
    wet = apply_biquads(dry, stages_in, sr)

    out = measure_eq(dry, wet, sr, max_stages=4)

    assert out, "no EQ stages detected"
    assert len(out) <= 4
    # A stage near 1 kHz must be present within +/-10% frequency...
    near = [s for s in out if abs(s.freq_hz - 1000.0) / 1000.0 <= 0.10]
    assert near, [(s.kind, s.freq_hz, s.gain_db, s.q) for s in out]
    # ...with gain within +/-1 dB and a sane Q.
    best = min(near, key=lambda s: abs(s.gain_db - 8.0))
    assert abs(best.gain_db - 8.0) <= 1.0
    assert 0.8 <= best.q <= 5.0
    # Combined fitted response at 1 kHz must match the true boost closely.
    freqs = np.array([1000.0])
    total_db = float(stages_response_db(out, freqs, sr)[0])
    assert abs(total_db - 8.0) <= 1.5


def test_measure_eq_flat_returns_empty() -> None:
    sr = TARGET_SR
    dry = make_noise(2 * sr, seed=2)
    wet = 0.7 * dry  # pure gain is not an EQ shape
    assert measure_eq(dry, wet, sr) == []


def test_measure_eq_cut_filter() -> None:
    sr = TARGET_SR
    dry = make_noise(4 * sr, seed=3)
    stages_in = [peaking_stage(freq_hz=3000.0, q=1.5, gain_db=-6.0)]
    wet = apply_biquads(dry, stages_in, sr)

    out = measure_eq(dry, wet, sr, max_stages=4)
    near = [s for s in out if abs(s.freq_hz - 3000.0) / 3000.0 <= 0.15]
    assert near, [(s.kind, s.freq_hz, s.gain_db) for s in out]
    best = min(near, key=lambda s: abs(s.gain_db + 6.0))
    assert abs(best.gain_db + 6.0) <= 1.5
