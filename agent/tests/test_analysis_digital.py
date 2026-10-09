"""Bitcrush tests: quantization depth and sample-rate reduction."""
from __future__ import annotations

import numpy as np

from ocapp_agent.analysis import TARGET_SR, measure_bitcrush

from test_analysis_support import make_noise, quantize, zoh_downsample


def test_measure_bitcrush_recovers_4bit() -> None:
    sr = TARGET_SR
    dry = make_noise(2 * sr, seed=5)
    dry = dry / np.max(np.abs(dry))
    wet = quantize(dry, bits=4)

    m = measure_bitcrush(dry, wet, sr)

    assert m is not None, "quantization not detected"
    assert m.bit_depth <= 6
    assert m.downsample == 1


def test_measure_bitcrush_with_hold() -> None:
    sr = TARGET_SR
    dry = make_noise(2 * sr, seed=13)
    dry = dry / np.max(np.abs(dry))
    wet = zoh_downsample(quantize(dry, bits=6), factor=4)

    m = measure_bitcrush(dry, wet, sr)

    assert m is not None
    assert m.bit_depth <= 6
    assert m.downsample >= 2


def test_measure_bitcrush_none_for_plain_audio() -> None:
    sr = TARGET_SR
    dry = make_noise(2 * sr, seed=14)
    assert measure_bitcrush(dry, 0.8 * dry, sr) is None
