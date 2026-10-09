"""Smoke tests for the shared schema contract in ocapp_agent.models."""
import pytest
from pydantic import ValidationError

from ocapp_agent.models import (
    AnalysisReport,
    BitcrushModel,
    ChainModel,
    CloneResult,
    CompressorModel,
    DelayModel,
    EqModel,
    FitReport,
    ReverbModel,
    WaveshaperModel,
)


def _full_chain() -> ChainModel:
    return ChainModel(
        stages=[
            EqModel(stages=[{"kind": "peaking", "freq_hz": 1000.0, "q": 1.0, "gain_db": -3.0}]),
            DelayModel(time_ms=250.0, feedback=0.4, mix=0.3, tone_hz=4000.0),
            BitcrushModel(bit_depth=8, downsample=2),
            WaveshaperModel(curve="tanh", drive_db=6.0, mix=0.5),
            CompressorModel(threshold_db=-18.0, ratio=4.0),
            ReverbModel(rt60_s=1.5, mix=0.3),
        ],
        output_gain_db=-1.5,
    )


def test_chain_with_each_stage_kind() -> None:
    chain = _full_chain()
    kinds = [stage.kind for stage in chain.stages]
    assert kinds == ["eq", "delay", "bitcrush", "waveshaper", "compressor", "reverb"]


def test_discriminator_selects_concrete_types() -> None:
    chain = _full_chain()
    assert isinstance(chain.stages[0], EqModel)
    assert isinstance(chain.stages[1], DelayModel)
    assert isinstance(chain.stages[2], BitcrushModel)
    assert isinstance(chain.stages[3], WaveshaperModel)
    assert isinstance(chain.stages[4], CompressorModel)
    assert isinstance(chain.stages[5], ReverbModel)


def test_discriminator_rejects_unknown_kind() -> None:
    with pytest.raises(ValidationError):
        ChainModel.model_validate({"stages": [{"kind": "chorus", "rate_hz": 1.0}]})


def test_clone_result_json_round_trip() -> None:
    result = CloneResult(
        chain=_full_chain(),
        analysis=AnalysisReport(
            sample_rate=48000,
            lag_samples=128,
            gain_db_applied=-2.0,
            is_parallel_mix=False,
            dry_rms_db=-20.0,
            wet_rms_db=-18.0,
            measurements={"rt60_estimate_s": 1.4},
        ),
        fit=FitReport(
            si_sdr_db=9.5,
            log_spectral_distance=1.2,
            loudness_diff_lufs=0.4,
            stage_notes=["reverb tail approximated"],
            limitations=["stereo width not modeled"],
        ),
        applet_name="ClonedReverb",
        applet_code="// generated",
        registration_snippet="HEMISPHERE_AUDIO_APPLET(...)",
        limitations=["stereo width not modeled"],
    )
    restored = CloneResult.model_validate_json(result.model_dump_json())
    assert restored == result
    assert restored.chain.stages[0].kind == "eq"
