"""Shared schema contract: analysis produces these, verify scores them, codegen consumes them."""
from typing import Annotated, Literal, Union
from pydantic import BaseModel, Field


class BiquadStage(BaseModel):
    kind: Literal['peaking', 'lowshelf', 'highshelf', 'lowpass', 'highpass', 'bandpass', 'notch']
    freq_hz: float = Field(gt=0)
    q: float = Field(default=0.707, gt=0)
    gain_db: float = 0.0


class EqModel(BaseModel):
    kind: Literal['eq'] = 'eq'
    stages: list[BiquadStage] = Field(default_factory=list)


class DelayModel(BaseModel):
    kind: Literal['delay'] = 'delay'
    time_ms: float = Field(ge=0)
    feedback: float = Field(default=0.0, ge=0.0, le=0.95)
    mix: float = Field(default=0.5, ge=0.0, le=1.0)
    tone_hz: float | None = Field(default=None, gt=0)  # LPF in feedback loop


class BitcrushModel(BaseModel):
    kind: Literal['bitcrush'] = 'bitcrush'
    bit_depth: int = Field(default=8, ge=1, le=16)
    downsample: int = Field(default=1, ge=1, le=64)


class WaveshaperModel(BaseModel):
    kind: Literal['waveshaper'] = 'waveshaper'
    curve: Literal['tanh', 'hard_clip', 'soft_clip', 'asymmetric', 'table'] = 'tanh'
    drive_db: float = 0.0
    mix: float = Field(default=1.0, ge=0.0, le=1.0)
    table: list[float] | None = None  # normalized -1..1 transfer points when curve == 'table'


class CompressorModel(BaseModel):
    kind: Literal['compressor'] = 'compressor'
    threshold_db: float = -18.0
    ratio: float = Field(default=4.0, gt=1.0)
    attack_ms: float = Field(default=10.0, gt=0)
    release_ms: float = Field(default=100.0, gt=0)
    makeup_db: float = 0.0
    knee_db: float = Field(default=6.0, ge=0)


class ReverbModel(BaseModel):
    kind: Literal['reverb'] = 'reverb'
    rt60_s: float = Field(gt=0)
    predelay_ms: float = Field(default=0.0, ge=0)
    damping: float = Field(default=0.5, ge=0.0, le=1.0)  # HF damping
    room_size: float = Field(default=0.5, ge=0.0, le=1.0)
    mix: float = Field(default=0.3, ge=0.0, le=1.0)


Stage = Annotated[
    Union[EqModel, DelayModel, BitcrushModel, WaveshaperModel, CompressorModel, ReverbModel],
    Field(discriminator='kind'),
]


class ChainModel(BaseModel):
    stages: list[Stage] = Field(default_factory=list)
    output_gain_db: float = 0.0


class AnalysisReport(BaseModel):
    sample_rate: int = 48000
    lag_samples: int = 0            # dry-vs-wet alignment shift applied
    gain_db_applied: float = 0.0    # loudness-match gain applied to wet
    is_parallel_mix: bool | None = None   # True = wet contains dry blended in
    dry_rms_db: float = 0.0
    wet_rms_db: float = 0.0
    measurements: dict[str, float] = Field(default_factory=dict)  # raw named measurements


class FitReport(BaseModel):
    si_sdr_db: float | None = None
    log_spectral_distance: float | None = None
    loudness_diff_lufs: float | None = None
    stage_notes: list[str] = Field(default_factory=list)
    limitations: list[str] = Field(default_factory=list)


class CloneResult(BaseModel):
    chain: ChainModel
    analysis: AnalysisReport
    fit: FitReport
    applet_name: str
    applet_code: str
    registration_snippet: str
    limitations: list[str] = Field(default_factory=list)
