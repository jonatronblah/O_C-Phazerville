"""Offline tests for the C++ code generator (no LLM, no sibling lanes)."""

from __future__ import annotations

import re

import pytest

from ocapp_agent.codegen import (
    cpp_strhash,
    generate_applet,
    validate_name,
    waveshaper_table,
)
from ocapp_agent.models import (
    BitcrushModel,
    BiquadStage,
    ChainModel,
    CompressorModel,
    DelayModel,
    EqModel,
    ReverbModel,
    WaveshaperModel,
)

REQUIRED_METHODS = [
    "InputStream(",
    "OutputStream(",
    "Start(",
    "Controller(",
    "View(",
    "OnDataRequest(",
    "OnDataReceive(",
    "SetHelp(",
    "OnEncoderMove(",
]


def assert_balanced(code: str) -> None:
    assert code.count("{") == code.count("}"), "unbalanced braces"
    assert code.count("(") == code.count(")"), "unbalanced parens"


def assert_structure(code: str, name: str) -> None:
    assert_balanced(code)
    assert f"class {name}Applet : public HemisphereAudioApplet" in code
    for m in REQUIRED_METHODS:
        assert m in code, f"missing {m}"
    assert "AudioPassthrough<MONO> input;" in code
    assert "AudioMixer4 out_mix;" in code
    assert "applet_id() override" in code
    assert "AllowRestart();" in code


# ---------------------------------------------------------------------------
# name validation + strhash
# ---------------------------------------------------------------------------

@pytest.mark.parametrize(
    "name",
    ["", "TOOLONGNAME", "9lead", "has space", "has-dash", "a" * 10, "ümlaut"],
)
def test_invalid_names_raise(name):
    with pytest.raises(ValueError):
        validate_name(name)


@pytest.mark.parametrize("name", ["a", "A", "MyClone", "PTCHDLY1", "a_b_c", "X" * 9])
def test_valid_names_pass(name):
    assert validate_name(name) == name


def test_strhash_matches_firmware_property():
    # HSUtils.h: for ascii strings of <= 9 chars the hash is just the ascii
    # bits concatenated together (7 bits per char, rotated left 7 each step).
    assert cpp_strhash("abc") == (97 << 14) | (98 << 7) | 99
    assert cpp_strhash("") == 0
    assert cpp_strhash("Reverb") != cpp_strhash("ReverbX")


def test_applet_id_uses_stable_hash_of_name():
    code, _ = generate_applet(ChainModel(), "MyName")
    expected = f"{cpp_strhash('MyName'):016x}"
    assert f"0x{expected}ULL" in code
    # renaming safety documented in the emitted comment
    assert "never orphans saved state" in code


# ---------------------------------------------------------------------------
# per-stage mapping
# ---------------------------------------------------------------------------

def test_empty_chain_is_passthrough():
    code, reg = generate_applet(ChainModel(), "Bare")
    assert_structure(code, "Bare")
    assert "PatchCable(input, 0, out_mix, 0);" in code
    assert "DeclareFancyApplet<BareApplet>" in reg


def test_eq_stage_uses_biquads():
    chain = ChainModel(
        stages=[
            EqModel(
                stages=[
                    BiquadStage(kind="lowpass", freq_hz=5000, q=0.7),
                    BiquadStage(kind="peaking", freq_hz=1000, q=1.0, gain_db=3),
                    BiquadStage(kind="lowshelf", freq_hz=200, q=0.7, gain_db=-2),
                ]
            )
        ]
    )
    code, _ = generate_applet(chain, "EqOnly")
    assert "AudioFilterBiquad eq0;" in code
    assert "setLowpass(0, 5000.0f" in code
    assert "setPeakingCoeffs(eq0, 1" in code  # RBJ helper used for peaking
    assert "setLowShelf(2" in code
    assert_structure(code, "EqOnly")


def test_eq_many_biquads_cascades_objects():
    stages = [
        BiquadStage(kind="peaking", freq_hz=200 * (i + 1), q=1.0, gain_db=1)
        for i in range(6)
    ]
    code, _ = generate_applet(ChainModel(stages=[EqModel(stages=stages)]), "WideEq")
    assert "AudioFilterBiquad eq0_0;" in code
    assert "AudioFilterBiquad eq0_1;" in code
    assert code.count("setPeakingCoeffs(") >= 6


def test_peaking_helper_not_emitted_without_peaking():
    chain = ChainModel(
        stages=[EqModel(stages=[BiquadStage(kind="highpass", freq_hz=80, q=0.7)])]
    )
    code, _ = generate_applet(chain, "NoPeak")
    assert "setPeakingCoeffs" not in code


def test_delay_stage_mapping():
    chain = ChainModel(
        stages=[DelayModel(time_ms=250, feedback=0.4, mix=0.5, tone_hz=3000)]
    )
    code, _ = generate_applet(chain, "Echo")
    assert "AudioEffectDelay dly0;" in code
    assert "AudioMixer4 dly0_fb;" in code
    assert "AudioMixer4 dly0_mix;" in code
    assert "AudioFilterBiquad dly0_tone;" in code
    assert "dly0.delay(0, 250.0f);" in code
    assert "setLowpass(0, 3000.0f" in code
    assert_structure(code, "Echo")


def test_delay_time_clamped_to_safe_max():
    chain = ChainModel(stages=[DelayModel(time_ms=5000)])
    code, _ = generate_applet(chain, "LongDly")
    assert "1400.0f" in code
    assert "clamped" in code  # limitation note in header comment


def test_bitcrush_stage_mapping():
    chain = ChainModel(stages=[BitcrushModel(bit_depth=6, downsample=4)])
    code, _ = generate_applet(chain, "Crush")
    assert "AudioEffectBitcrusher bc0;" in code
    assert "bc0.bits((uint8_t)6);" in code
    assert "bc0.sampleRate(12000.0f);" in code  # 48000 / 4
    assert_structure(code, "Crush")


@pytest.mark.parametrize("curve", ["tanh", "hard_clip", "soft_clip", "asymmetric", "table"])
def test_waveshaper_stage_mapping(curve):
    table = [float(x) / 4 for x in range(-4, 5)] if curve == "table" else None
    chain = ChainModel(
        stages=[WaveshaperModel(curve=curve, drive_db=6, mix=0.8, table=table)]
    )
    code, _ = generate_applet(chain, "Growl")
    assert "AudioEffectWaveshaper ws0;" in code
    assert "float ws0_table[129]" in code
    assert "ws0.shape(ws0_table, 129);" in code
    assert "AudioMixer4 ws0_mix;" in code
    assert_structure(code, "Growl")


def test_waveshaper_table_has_129_points():
    chain = ChainModel(stages=[WaveshaperModel(curve="table", table=[-1, 0, 1])])
    code, _ = generate_applet(chain, "Tbl")
    m = re.search(r"float ws0_table\[129\] = \{(.*?)\};", code, re.S)
    assert m, "table array not found"
    values = re.findall(r"[-0-9.eE+]+f", m.group(1))
    assert len(values) == 129


def test_waveshaper_table_python_helper():
    t = waveshaper_table(WaveshaperModel(curve="tanh", drive_db=0))
    assert len(t) == 129
    assert t[0] == pytest.approx(-1.0)
    assert t[-1] == pytest.approx(1.0)
    assert t[64] == pytest.approx(0.0, abs=1e-6)
    hard = waveshaper_table(WaveshaperModel(curve="hard_clip", drive_db=12))
    assert hard[-1] == pytest.approx(1.0)  # clamped to [-1, 1]


def test_compressor_uses_shared_factory():
    chain = ChainModel(
        stages=[
            CompressorModel(
                threshold_db=-18, ratio=4, attack_ms=10, release_ms=100, makeup_db=2
            )
        ]
    )
    code, _ = generate_applet(chain, "Squash")
    assert "AudioEffectDynamics* cmp0" in code
    assert "cmp0 = GetComp();" in code
    assert "ReleaseComp(cmp0);" in code
    assert "cmp0->compression(-18.0f" in code
    assert "cmp0->makeupGain(2.0f);" in code
    assert "Compressor mapping (approximate)" in code
    assert "AudioPassthrough<MONO> cmp0_out;" in code
    assert_structure(code, "Squash")


def test_reverb_uses_shared_factory():
    chain = ChainModel(
        stages=[
            ReverbModel(rt60_s=2.5, predelay_ms=20, damping=0.5, mix=0.3)
        ]
    )
    code, _ = generate_applet(chain, "Hall")
    assert "AudioEffectReverbSchroeder* rvb0" in code
    assert "rvb0 = GetBungverb();" in code
    assert "ReleaseBungverb(rvb0);" in code
    assert "rvb0->setDecayTime(2.5f);" in code
    assert "rvb0->setDamping(0.5f);" in code
    assert "AudioEffectDelay rvb0_pre;" in code  # predelay
    assert "AudioMixer4 rvb0_mix;" in code
    assert_structure(code, "Hall")


# ---------------------------------------------------------------------------
# combined chain + header/registration content
# ---------------------------------------------------------------------------

def _combined_chain() -> ChainModel:
    return ChainModel(
        stages=[
            EqModel(stages=[BiquadStage(kind="peaking", freq_hz=800, q=1.2, gain_db=3)]),
            DelayModel(time_ms=250, feedback=0.4, mix=0.5, tone_hz=3000),
            WaveshaperModel(curve="tanh", drive_db=6, mix=0.8),
            BitcrushModel(bit_depth=8, downsample=2),
            CompressorModel(threshold_db=-18, ratio=4),
            ReverbModel(rt60_s=2.0, mix=0.3),
        ],
        output_gain_db=2.0,
    )


def test_combined_chain_structure():
    code, reg = generate_applet(_combined_chain(), "Combo")
    assert_structure(code, "Combo")
    for token in [
        "AudioFilterBiquad",
        "AudioEffectDelay",
        "AudioEffectWaveshaper",
        "AudioEffectBitcrusher",
        "AudioEffectDynamics",
        "AudioEffectReverbSchroeder",
        "AudioMixer4",
    ]:
        assert token in code, f"missing {token}"
    assert "DeclareFancyApplet<ComboApplet>" in reg
    assert '#include "Combo_applet.h"' in reg
    assert "stereo" in reg.lower()


def test_header_comment_block():
    code, _ = generate_applet(_combined_chain(), "Hdr")
    assert '"stages"' in code  # ChainModel JSON dumped as comment
    assert "Known fidelity limitations" in code
    assert code.count("//") > 20


def test_output_gain_applied():
    code, _ = generate_applet(ChainModel(output_gain_db=6.0), "Gain6")
    assert "out_mix.gain(0, 1.99526f);" in code


def test_cv_param_ui_present():
    chain = ChainModel(stages=[DelayModel(time_ms=100, feedback=0.3)])
    code, _ = generate_applet(chain, "Ctrl")
    assert "CVInputMap" in code
    assert "CheckEditInputMapPress" in code
    assert "PackPackables(" in code
    assert "UnpackPackables(" in code
    assert "ApplyParams();" in code
    assert "NUM_PARAMS" in code


def test_fixed_chain_has_no_cv_ui():
    code, _ = generate_applet(ChainModel(output_gain_db=1.0), "Fixed")
    assert "CVInputMap" not in code
    assert "NUM_PARAMS = 0" in code


def test_cable_overflow_raises():
    chain = ChainModel(stages=[DelayModel(time_ms=100) for _ in range(7)])
    with pytest.raises(ValueError, match="patch cables"):
        generate_applet(chain, "TooMany")
