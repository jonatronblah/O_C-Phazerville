"""Memoryless nonlinearity (waveshaper) estimation from the scatter curve."""
from __future__ import annotations

import numpy as np
from scipy import optimize

from .common import TARGET_SR, validate_audio
from ..models import WaveshaperModel

#: Bin count for the empirical transfer curve / table size.
N_BINS = 65

#: Min samples per bin for a usable curve point.
MIN_BIN_COUNT = 30

#: A curve deviating from a straight line by less than this (in wet sigma) is
#: considered a linear system (gain, EQ, delay...), not a waveshaper.
LINEARITY_TOL = 0.025

#: Rising-vs-falling loop area above this (wet sigma) flags memory/hysteresis.
HYSTERESIS_TOL = 0.06

#: Parametric fits worse than this (wet sigma) fall back to a table curve.
MAX_PARAM_RESID = 0.04


def _scatter_curve(
    x: np.ndarray, y: np.ndarray, n_bins: int = N_BINS
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    """Median y per x-bin over the well-populated range of x.

    Returns (centers, medians, counts, sigma_y).
    """
    x_lo, x_hi = float(x.min()), float(x.max())
    sigma_y = float(np.std(y))
    if x_hi - x_lo <= 0.0 or sigma_y <= 0.0:
        raise ValueError("signal has no variation")
    edges = np.linspace(x_lo, x_hi, n_bins + 1)
    idx = np.clip(np.digitize(x, edges) - 1, 0, n_bins - 1)
    counts = np.bincount(idx, minlength=n_bins)
    sums = np.bincount(idx, weights=y, minlength=n_bins)
    centers = 0.5 * (edges[:-1] + edges[1:])
    medians = np.full(n_bins, np.nan)
    for i in range(n_bins):
        if counts[i] >= MIN_BIN_COUNT:
            medians[i] = float(np.median(y[idx == i]))
    ok = np.isfinite(medians)
    return centers[ok], medians[ok], counts[ok], sigma_y


def _curve_model(family: str, params: np.ndarray, x: np.ndarray) -> np.ndarray:
    if family == "tanh":
        (g,) = params
        return np.tanh(np.clip(g * x, -30.0, 30.0))
    if family == "soft_clip":
        (g,) = params
        return np.sign(x) * (1.0 - np.exp(-np.clip(abs(g) * np.abs(x), 0.0, 30.0)))
    if family == "hard_clip":
        (g,) = params
        return np.clip(g * x, -1.0, 1.0)
    if family == "asymmetric":
        g_p, g_n = params
        return np.where(
            x >= 0.0,
            np.tanh(np.clip(g_p * x, -30.0, 30.0)),
            np.tanh(np.clip(g_n * x, -30.0, 30.0)),
        )
    raise ValueError(family)


def _fit_family(
    family: str, xc: np.ndarray, yc: np.ndarray, weights: np.ndarray
) -> tuple[float, np.ndarray]:
    """Weighted least-squares fit of a parametric curve.

    Returns (rms residual in y units, params).
    """
    if family == "asymmetric":
        x0 = np.array([1.0, 1.0])
        lower = np.array([0.05, 0.05])
        upper = np.array([20.0, 20.0])
    else:
        x0 = np.array([1.0])
        lower = np.array([0.05])
        upper = np.array([20.0])

    def resid(params: np.ndarray) -> np.ndarray:
        return (_curve_model(family, params, xc) - yc) * weights

    try:
        sol = optimize.least_squares(resid, x0, bounds=(lower, upper), method="trf")
        fit = _curve_model(family, sol.x, xc)
        r = float(np.sqrt(np.mean((fit - yc) ** 2)))
        return r, sol.x
    except Exception:
        return np.inf, x0


def _curve_nonlinearity(
    xc: np.ndarray, yc: np.ndarray, counts: np.ndarray, sigma_y: float
) -> float:
    """Max |curve - best line| in units of sigma_y (weighted LS line)."""
    w = counts.astype(np.float64)
    a = np.vstack([xc, np.ones_like(xc)]).T
    aw = a * w[:, None]
    try:
        line_params, *_ = np.linalg.lstsq(aw, yc * w, rcond=None)
    except np.linalg.LinAlgError:
        return np.inf
    dev = yc - a @ line_params
    return float(np.max(np.abs(dev)) / (sigma_y + 1e-30))


def _loop_area(
    x: np.ndarray, y: np.ndarray, sigma_y: float
) -> float:
    """Directional hysteresis: rising vs falling conditional curves (sigma units)."""
    dx = np.diff(x, prepend=x[0])
    curves = []
    for mask in (dx > 0, dx < 0):
        try:
            xc, yc, _, _ = _scatter_curve(x[mask], y[mask], n_bins=33)
        except ValueError:
            return np.inf
        curves.append((xc, yc))
    (x1, y1), (x2, y2) = curves
    if x1.size < 8 or x2.size < 8:
        return np.inf
    grid = np.linspace(max(x1[0], x2[0]), min(x1[-1], x2[-1]), 33)
    if grid.size < 4:
        return np.inf
    d = np.interp(grid, x1, y1) - np.interp(grid, x2, y2)
    return float(np.max(np.abs(d)) / (sigma_y + 1e-30))


def measure_waveshaper(
    dry: np.ndarray, wet: np.ndarray, sr: int = TARGET_SR
) -> WaveshaperModel | None:
    """Estimate a memoryless nonlinearity from the instantaneous scatter wet vs dry.

    - No curve deviation from a line -> linear system -> None.
    - Large rising/falling loop (hysteresis / dynamic memory) -> None.
    - Fits tanh / soft_clip / hard_clip / asymmetric parametric curves; when
      none fits well, returns a 65-point normalized table curve.
    """
    d = validate_audio(dry, "dry")
    w = validate_audio(wet, "wet")
    n = min(d.size, w.size)
    x = d[:n]
    y = w[:n]
    if float(x.max() - x.min()) <= 0.0:
        return None

    try:
        xc, yc, counts, sigma_y = _scatter_curve(x, y)
    except ValueError:
        return None
    if xc.size < 8:
        return None

    # 1) Is there any nonlinearity at all? (Linear filters keep the curve a line.)
    if _curve_nonlinearity(xc, yc, counts, sigma_y) < LINEARITY_TOL:
        return None

    # 2) Memorylessness: a large directional loop means time-varying behavior.
    if _loop_area(x, y, sigma_y) > HYSTERESIS_TOL:
        return None

    # 3) Parametric fits on the empirical curve (bin-count weighted).
    weights = np.sqrt(counts.astype(np.float64) / float(counts.mean()))
    results: list[tuple[float, str, np.ndarray]] = []
    for family in ("tanh", "soft_clip", "hard_clip", "asymmetric"):
        r, params = _fit_family(family, xc, yc, weights)
        results.append((r, family, params))

    results.sort(key=lambda t: (t[0], t[1] != "tanh"))
    resid, family, params = results[0]

    if resid <= MAX_PARAM_RESID:
        drive_db = float(20.0 * np.log10(max(float(params[0]), 1e-3)))
        return WaveshaperModel(curve=family, drive_db=drive_db)

    # 4) Fallback: normalized table with 65 points over normalized x in [-1, 1].
    x_scale = max(abs(float(xc[0])), abs(float(xc[-1])), 1e-9)
    y_scale = float(np.max(np.abs(yc))) or 1.0
    grid = np.linspace(-1.0, 1.0, N_BINS)
    ys = np.interp(grid * x_scale, xc, yc)
    table = [float(np.clip(v / y_scale, -1.0, 1.0)) for v in ys]
    return WaveshaperModel(curve="table", drive_db=0.0, table=table)


def apply_waveshaper(
    x: np.ndarray, model: WaveshaperModel, x_scale: float | None = None
) -> np.ndarray:
    """Apply an estimated waveshaper curve to ``x`` (chain simulation)."""
    xa = np.asarray(x, dtype=np.float64)
    drive = 10.0 ** (model.drive_db / 20.0)
    if model.curve == "tanh":
        return np.tanh(np.clip(drive * xa, -30.0, 30.0))
    if model.curve == "soft_clip":
        g = max(drive, 1e-3)
        return np.sign(xa) * (1.0 - np.exp(-np.clip(g * np.abs(xa), 0.0, 30.0))) / max(
            1.0 - np.exp(-abs(g)), 1e-6
        )
    if model.curve == "hard_clip":
        return np.clip(drive * xa, -1.0, 1.0)
    if model.curve == "asymmetric":
        # Symmetric approximation: use drive for both polarities.
        return np.tanh(np.clip(drive * xa, -30.0, 30.0))
    # table: 65 uniformly spaced y-values over normalized x in [-1, 1]
    if not model.table:
        return xa.copy()
    scale = x_scale if x_scale else max(float(np.max(np.abs(xa))), 1e-9)
    xn = np.clip(xa / scale, -1.0, 1.0)
    grid = np.linspace(-1.0, 1.0, len(model.table))
    return np.interp(xn, grid, np.asarray(model.table, dtype=np.float64)) * scale
