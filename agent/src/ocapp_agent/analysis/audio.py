"""Audio I/O and loudness matching."""
from __future__ import annotations

from pathlib import Path

import numpy as np

from .common import TARGET_SR, rms, validate_audio


def load_audio(path: str | Path) -> tuple[np.ndarray, int]:
    """Load audio as mono float32, resampled to TARGET_SR (48000).

    Channels are averaged to mono. Raises ``FileNotFoundError`` for a missing
    file and ``ValueError`` for unreadable or non-finite content.
    """
    import librosa
    import soundfile as sf

    p = Path(path)
    if not p.is_file():
        raise FileNotFoundError(f"audio file not found: {p}")
    try:
        data, sr = sf.read(str(p), always_2d=True, dtype="float32")
    except Exception as exc:  # libsndfile errors vary by platform
        raise ValueError(f"could not read audio file {p}: {exc}") from exc
    if data.size == 0:
        raise ValueError(f"audio file {p} is empty")

    mono = data.mean(axis=1).astype(np.float32)
    if not np.all(np.isfinite(mono)):
        raise ValueError(f"audio file {p} contains NaN or infinite samples")

    if sr != TARGET_SR:
        mono = librosa.resample(mono, orig_sr=sr, target_sr=TARGET_SR)
        mono = np.asarray(mono, dtype=np.float32)
    return np.ascontiguousarray(mono), TARGET_SR


def match_loudness(
    dry: np.ndarray, wet: np.ndarray, sr: int = TARGET_SR
) -> tuple[np.ndarray, float]:
    """Scale ``wet`` to ``dry``'s loudness (pyloudnorm LUFS).

    Returns ``(wet_scaled, gain_db_applied)`` where
    ``wet_scaled = wet * 10**(gain_db_applied / 20)``. Falls back to RMS
    matching for very short signals (< ~0.5 s) where LUFS gating is
    unreliable. Both inputs must be non-silent and finite.
    """
    import pyloudnorm

    d = validate_audio(dry, "dry")
    w = validate_audio(wet, "wet")

    gain_db = None
    if min(d.size, w.size) >= int(0.5 * sr):
        try:
            meter = pyloudnorm.Meter(sr)
            lufs_dry = float(meter.integrated_loudness(d))
            lufs_wet = float(meter.integrated_loudness(w))
            if np.isfinite(lufs_dry) and np.isfinite(lufs_wet):
                gain_db = lufs_dry - lufs_wet
        except Exception:
            gain_db = None  # fall back to RMS below

    if gain_db is None or not np.isfinite(gain_db):
        gain_db = float(20.0 * np.log10((rms(d) + 1e-12) / (rms(w) + 1e-12)))
    # Clamp to a sane range; extreme values indicate a broken pair.
    gain_db = float(np.clip(gain_db, -60.0, 60.0))

    wet_scaled = w * (10.0 ** (gain_db / 20.0))
    return wet_scaled, gain_db
