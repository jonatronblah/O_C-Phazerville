"""analyze_pair orchestration: align -> loudness -> staged measurements."""
from __future__ import annotations

import numpy as np

from .alignment import align_pair, detect_parallel_mix, parallel_mix_score
from .audio import match_loudness
from .common import TARGET_SR, apply_biquads, rms_db, validate_audio
from .digital import measure_bitcrush
from .dynamics import apply_compressor, measure_dynamics
from .ir import estimate_ir, measure_delay, measure_reverb
from .nonlinear import apply_waveshaper, measure_waveshaper
from .spectral import measure_eq, transfer_function_db, stages_response_db
from ..models import AnalysisReport, ChainModel, EqModel, Stage


def analyze_pair(
    dry: np.ndarray, wet: np.ndarray, sr: int = TARGET_SR
) -> tuple[ChainModel, AnalysisReport]:
    """Estimate the effect chain that transformed dry into wet.

    Pipeline: cross-correlation alignment -> loudness match -> parallel-mix
    detection -> waveshaper / compressor / bitcrush (broadband nonlinear, on
    the aligned pair) -> delay detection from the IR (before EQ, so the comb
    ripple cannot be absorbed by biquads) -> EQ (transfer ratio after
    subtracting nonlinear predictions, smoothed past the comb period) ->
    reverb (on the residual after EQ). Delay is re-measured after the EQ
    compensation for cleaner tap amplitudes.

    Returns ``(chain, report)``. Chain order convention:
    ``[waveshaper?, compressor?, bitcrush?, eq?, delay?, reverb?]``.
    ``report.gain_db_applied`` is the gain applied to wet during the loudness
    match; ``chain.output_gain_db`` is its negation (the static gain the
    chain must add to dry to reach wet's level).
    """
    d = validate_audio(dry, "dry")
    w = validate_audio(wet, "wet")

    dry_a, wet_a, lag = align_pair(d, w)
    dry_rms = rms_db(dry_a)
    wet_rms = rms_db(wet_a)

    wet_m, gain_db = match_loudness(dry_a, wet_a, sr)
    parallel = detect_parallel_mix(dry_a, wet_m, sr)
    score = parallel_mix_score(dry_a, wet_m, sr)

    measurements: dict[str, float] = {
        "lag_samples": float(lag),
        "gain_db_applied": float(gain_db),
        "dry_rms_db": float(dry_rms),
        "wet_rms_db": float(wet_rms),
        "parallel_score": float(score),
        "wet_clip_fraction": float(np.mean(np.abs(wet_a) >= 0.999)),
    }

    stages: list[Stage] = []

    # Nonlinear stages first, on the aligned pair. Each measurement runs on
    # the prediction of everything before it ("subtracting nonlinear
    # predictions") so later stages are not mis-attributed.
    pred = dry_a
    ws = measure_waveshaper(pred, wet_m, sr)
    if ws is not None:
        pred = apply_waveshaper(pred, ws)
        stages.append(ws)
        measurements["ws_drive_db"] = float(ws.drive_db)

    comp = measure_dynamics(pred, wet_m, sr)
    if comp is not None:
        pred = apply_compressor(pred, comp, sr)
        stages.append(comp)
        measurements["dynamics_threshold_db"] = float(comp.threshold_db)
        measurements["dynamics_ratio"] = float(comp.ratio)
        measurements["dynamics_attack_ms"] = float(comp.attack_ms)
        measurements["dynamics_release_ms"] = float(comp.release_ms)
        measurements["dynamics_makeup_db"] = float(comp.makeup_db)

    crush = measure_bitcrush(pred, wet_m, sr)
    if crush is not None:
        stages.append(crush)
        measurements["bitcrush_bit_depth"] = float(crush.bit_depth)
        measurements["bitcrush_downsample"] = float(crush.downsample)

    # Delay is detected BEFORE the EQ fit. Decision rule (parsimony): the
    # periodic ripple a comb delay puts in the transfer function is time-domain
    # echo structure -- one delay stage explains it, while peaking biquads
    # would burn one stage per ripple bump. So the delay wins the ripple and
    # the EQ fit only sees the smooth remainder: we widen the transfer
    # smoothing past the comb period (~2/T Hz, at least 120 Hz) before fitting.
    # EQ features narrower than that are deliberately absorbed by the delay
    # explanation.
    delay = measure_delay(pred, wet_m, sr)
    smooth_hz = None
    if delay is not None and delay.time_ms > 0.0:
        smooth_hz = max(120.0, 2.0 / (delay.time_ms * 1e-3))

    eq_stages = measure_eq(pred, wet_m, sr, max_stages=4, smooth_hz=smooth_hz)
    freqs, target_db = transfer_function_db(pred, wet_m, sr, smooth_hz)
    model_db = stages_response_db(eq_stages, freqs, sr)
    measurements["eq_stage_count"] = float(len(eq_stages))
    measurements["eq_fit_rms_db"] = float(np.sqrt(np.mean((target_db - model_db) ** 2)))
    if eq_stages:
        pred = apply_biquads(pred, eq_stages, sr)
        stages.append(EqModel(stages=eq_stages))
        # Re-measure the delay on the EQ-compensated residual: the tap
        # amplitudes are cleaner once the EQ coloring is removed.
        refined = measure_delay(pred, wet_m, sr)
        if refined is not None:
            delay = refined

    ir = estimate_ir(pred, wet_m, sr)
    absir = np.abs(ir)
    pmax_ir = float(absir.max()) if absir.size else 0.0
    measurements["ir_peak_db"] = float(20.0 * np.log10(max(pmax_ir, 1e-12)))
    # Tail extent of *significant* structure: above both -26 dB and 3x the
    # deconvolution residual floor, ignoring the last 2% of the buffer (FFT
    # end-of-buffer artifact). A plain -40 dB threshold would report the whole
    # window whenever the residual floor sits near -40 dB (tonal sources).
    floor = float(np.median(absir[absir.size // 2:])) if absir.size else 0.0
    usable = absir[: max(1, int(0.98 * absir.size))]
    thr = max(0.05 * pmax_ir, 3.0 * floor)
    above = np.where(usable > thr)[0]
    measurements["ir_tail_s"] = float(above[-1]) / sr if above.size else 0.0

    if delay is not None:
        stages.append(delay)
        measurements["delay_time_ms"] = float(delay.time_ms)
        measurements["delay_feedback"] = float(delay.feedback)
        measurements["delay_mix"] = float(delay.mix)
        if delay.tone_hz is not None:
            measurements["delay_tone_hz"] = float(delay.tone_hz)

    rev = measure_reverb(pred, wet_m, sr)
    if rev is not None:
        stages.append(rev)
        measurements["rt60_s"] = float(rev.rt60_s)
        measurements["predelay_ms"] = float(rev.predelay_ms)
        measurements["reverb_damping"] = float(rev.damping)
        measurements["reverb_room_size"] = float(rev.room_size)
        measurements["reverb_mix"] = float(rev.mix)

    chain = ChainModel(stages=stages, output_gain_db=float(-gain_db))
    report = AnalysisReport(
        sample_rate=int(sr),
        lag_samples=int(lag),
        gain_db_applied=float(gain_db),
        is_parallel_mix=bool(parallel),
        dry_rms_db=float(dry_rms),
        wet_rms_db=float(wet_rms),
        measurements=measurements,
    )
    return chain, report
