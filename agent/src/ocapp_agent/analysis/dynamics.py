"""Dynamics (compression) estimation from RMS envelope regression."""
from __future__ import annotations

import numpy as np
from scipy import optimize

from .common import TARGET_SR, validate_audio
from ..models import CompressorModel

#: Envelope frame rate (10 ms RMS windows, non-overlapping).
ENV_HOP_S = 0.010

#: Static-curve knee width used for the fit (matches the schema default).
KNEE_DB = 6.0

#: Slope deviation from 1.0 below which the pair is considered ~linear.
LINEAR_SLOPE_TOL = 0.05

#: Envelope-fit residual above this (dB) means the model does not explain it.
MAX_FIT_RESID_DB = 4.0


def _rms_envelope_db(x: np.ndarray, hop: int) -> np.ndarray:
    n_frames = x.size // hop
    if n_frames < 8:
        raise ValueError("signal too short for envelope analysis")
    frames = x[: n_frames * hop].reshape(n_frames, hop)
    env = np.sqrt(np.mean(frames * frames, axis=1) + 1e-24)
    return 20.0 * np.log10(env)


def static_curve_db(x_db: np.ndarray, theta: float, ratio: float, knee: float) -> np.ndarray:
    """Compressor static law in dB (soft knee); returns output level in dB."""
    over = x_db - theta
    half = knee / 2.0
    gain = np.zeros_like(x_db)
    mid = np.abs(over) <= half
    high = over > half
    gain[mid] = (1.0 / ratio - 1.0) * (over[mid] + half) ** 2 / (2.0 * knee) if knee > 0 else 0.0
    gain[high] = (1.0 / ratio - 1.0) * over[high]
    return x_db + gain


def _envelope_follow(
    target_db: np.ndarray, attack_s: float, release_s: float, hop_s: float
) -> np.ndarray:
    """One-pole envelope follower on a dB gain-reduction sequence (vector loop)."""
    out = np.empty_like(target_db)
    a_att = 1.0 - float(np.exp(-hop_s / max(attack_s, 1e-4)))
    a_rel = 1.0 - float(np.exp(-hop_s / max(release_s, 1e-4)))
    state = float(target_db[0])
    out[0] = state
    for i in range(1, target_db.size):
        t = float(target_db[i])
        alpha = a_att if t > state else a_rel
        state += (t - state) * alpha
        out[i] = state
    return out


def measure_dynamics(
    dry: np.ndarray, wet: np.ndarray, sr: int = TARGET_SR
) -> CompressorModel | None:
    """Estimate compression from 10 ms RMS envelopes.

    Regresses the wet envelope against the dry envelope in dB with a
    soft-knee piecewise curve (threshold / ratio / makeup), estimates
    attack/release by fitting a one-pole follower to the gain sequence.
    Returns None when the envelope relationship is ~linear, the data does not
    span the threshold, or the static curve does not explain the relationship.
    """
    d = validate_audio(dry, "dry")
    w = validate_audio(wet, "wet")
    n = min(d.size, w.size)
    hop = max(1, int(round(ENV_HOP_S * sr)))
    try:
        x_db = _rms_envelope_db(d[:n], hop)
        y_db = _rms_envelope_db(w[:n], hop)
    except ValueError:
        return None

    floor = float(x_db.max()) - 40.0
    active = x_db > floor
    if active.sum() < 60:
        return None  # too little level variation to judge
    xd = x_db[active]
    yw = y_db[active]
    if float(xd.max() - xd.min()) < 6.0:
        return None  # dry is effectively static: unidentifiable

    # Quick ~linear check on the robust global slope.
    slope, intercept = np.polyfit(xd, yw, 1)
    if abs(float(slope) - 1.0) < LINEAR_SLOPE_TOL:
        return None

    def resid(params: np.ndarray) -> np.ndarray:
        theta, log_ratio, makeup = params
        ratio = 1.0 + np.exp(log_ratio)
        return static_curve_db(xd, theta, ratio, KNEE_DB) + makeup - yw

    theta0 = float(np.clip(np.percentile(xd, 70), -60.0, 0.0))
    ratio0 = float(np.clip(1.0 / max(1.0 - (1.0 - float(slope)), 0.05), 1.05, 20.0))
    if not np.isfinite(ratio0):
        ratio0 = 4.0
    x0 = np.array([theta0, np.log(max(ratio0 - 1.0, 0.05)), float(intercept)])
    lower = np.array([-60.0, np.log(0.05), -36.0])
    upper = np.array([0.0, np.log(29.0), 36.0])
    x0 = np.clip(x0, lower, upper)
    try:
        sol = optimize.least_squares(resid, x0, bounds=(lower, upper), method="trf")
        theta, log_ratio, makeup = sol.x
        fit_resid = float(np.sqrt(np.mean(resid(sol.x) ** 2)))
    except Exception:
        return None

    ratio = 1.0 + float(np.exp(log_ratio))
    if ratio <= 1.05 or fit_resid > MAX_FIT_RESID_DB:
        return None
    if int(np.sum(xd > theta)) < 30:
        return None  # almost nothing is being compressed

    # Attack/release: fit a one-pole follower to the predicted gain sequence.
    gain_pred = static_curve_db(xd, float(theta), ratio, KNEE_DB) - xd
    hop_s = hop / float(sr)
    attack_grid = np.geomspace(0.001, 0.5, 9)
    release_grid = np.geomspace(0.01, 2.0, 9)
    best = (np.inf, 0.010, 0.100)
    for att in attack_grid:
        for rel in release_grid:
            sim = _envelope_follow(gain_pred, float(att), float(rel), hop_s)
            err = float(np.mean((sim - gain_pred) ** 2))
            if err < best[0]:
                best = (err, float(att), float(rel))

    return CompressorModel(
        threshold_db=float(np.clip(theta, -60.0, 0.0)),
        ratio=float(np.clip(ratio, 1.05, 20.0)),
        attack_ms=float(np.clip(best[1] * 1000.0, 0.1, 2000.0)),
        release_ms=float(np.clip(best[2] * 1000.0, 1.0, 5000.0)),
        makeup_db=float(np.clip(makeup, -36.0, 36.0)),
        knee_db=KNEE_DB,
    )


def apply_compressor(
    x: np.ndarray, model: CompressorModel, sr: int = TARGET_SR
) -> np.ndarray:
    """Approximate a compressor's action (static curve + envelope follower)."""
    hop = max(1, int(round(ENV_HOP_S * sr)))
    n_frames = x.size // hop
    if n_frames < 8:
        return np.asarray(x, dtype=np.float64).copy()
    frames = np.asarray(x, dtype=np.float64)[: n_frames * hop].reshape(n_frames, hop)
    env_db = 20.0 * np.log10(np.sqrt(np.mean(frames * frames, axis=1) + 1e-24))
    out_db = static_curve_db(env_db, model.threshold_db, model.ratio, model.knee_db)
    gain_db = out_db - env_db
    gain_db = _envelope_follow(
        gain_db, model.attack_ms / 1000.0, model.release_ms / 1000.0, hop / float(sr)
    )
    gain_db = gain_db + model.makeup_db
    per_frame = np.repeat(10.0 ** (gain_db / 20.0), hop)
    y = np.asarray(x, dtype=np.float64).copy()
    m = min(per_frame.size, y.size)
    y[:m] *= per_frame[:m]
    return y
