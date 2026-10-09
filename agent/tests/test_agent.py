"""Hermetic tests for the effect_cloner agent and its tools.

No network, no API keys and no imports of the sibling `ocapp_agent.analysis`
/ `ocapp_agent.verify` lanes: the lazy-import seams in ocapp_agent.tools are
monkeypatched instead. The agent runs under TestModel via
`agent.override(model=...)` per the building-pydantic-ai-agents skill —
never by assigning agent.model directly.
"""

from __future__ import annotations

import json
from types import SimpleNamespace

import numpy as np
import pytest
from pydantic_ai import ModelRetry
from pydantic_ai.models.test import TestModel

from ocapp_agent import tools
from ocapp_agent.agent import agent, clone_effect
from ocapp_agent.models import (
    AnalysisReport,
    BiquadStage,
    ChainModel,
    EqModel,
    FitReport,
)
from ocapp_agent.tools import CloneSession


@pytest.fixture
def pipeline(monkeypatch):
    """Stub every lazy-import seam; returns a call recorder + fixed models."""
    calls: list[str] = []
    chain = ChainModel(
        stages=[EqModel(stages=[BiquadStage(kind="peaking", freq_hz=1000.0, q=1.0, gain_db=3)])],
        output_gain_db=1.0,
    )
    analysis = AnalysisReport(dry_rms_db=-20.0, wet_rms_db=-18.0)
    fit = FitReport(si_sdr_db=12.5, limitations=["fit-side caveat"])

    def fake_load_and_analyze(dry_path, wet_path):
        calls.append("analyze_audio")
        return (
            np.zeros(64, dtype=np.float32),
            np.zeros(64, dtype=np.float32),
            48000,
            chain,
            analysis,
        )

    def fake_load_arrays(dry_path, wet_path):
        calls.append("load_arrays")
        return np.zeros(64, dtype=np.float32), np.zeros(64, dtype=np.float32), 48000

    def fake_verify(dry, wet, sr, c):
        calls.append("render_and_score")
        return fit

    def fake_measure(fn_name, dry, wet, sr):
        calls.append(f"measure:{fn_name}")
        return EqModel(stages=[BiquadStage(kind="lowpass", freq_hz=4000.0, q=0.7)])

    monkeypatch.setattr(tools, "_load_and_analyze", fake_load_and_analyze)
    monkeypatch.setattr(tools, "_load_arrays", fake_load_arrays)
    monkeypatch.setattr(tools, "_verify_chain", fake_verify)
    monkeypatch.setattr(tools, "_measure_stage", fake_measure)
    monkeypatch.setattr(tools, "parse_chain_json", lambda s: chain.model_copy(deep=True))
    return SimpleNamespace(calls=calls, chain=chain, analysis=analysis, fit=fit)


@pytest.fixture
def wav_pair(tmp_path):
    """Tiny synthetic wav files (no sibling lanes needed)."""
    import soundfile as sf

    dry = tmp_path / "dry.wav"
    wet = tmp_path / "wet.wav"
    sig = np.zeros(480, dtype=np.float32)
    sig[:10] = 0.5
    sf.write(str(dry), sig, 48000)
    sf.write(str(wet), sig * 0.8, 48000)
    return str(dry), str(wet)


@pytest.fixture(autouse=True)
def quiet_env(monkeypatch):
    monkeypatch.setenv("PYDANTIC_AI_NO_BANNER", "1")
    monkeypatch.delenv("LOGFIRE_TOKEN", raising=False)
    monkeypatch.delenv("OCA_LOGFIRE", raising=False)


def test_clone_effect_end_to_end_with_test_model(pipeline, wav_pair, tmp_path, monkeypatch):
    dry, wet = wav_pair
    out_dir = tmp_path / "out"

    m = TestModel(call_tools=["analyze_audio", "render_and_score", "emit_applet"])
    with agent.override(model=m):
        clone = clone_effect(dry, wet, str(out_dir))

    # tools ran (stubs recorded)
    assert "analyze_audio" in pipeline.calls
    assert "render_and_score" in pipeline.calls

    # structured output produced + reconciled with session-authoritative data
    assert clone.chain == pipeline.chain
    assert clone.analysis == pipeline.analysis
    assert clone.fit == pipeline.fit
    assert "class" in clone.applet_code
    assert "DeclareFancyApplet" in clone.registration_snippet

    # emitted artifacts + CloneResult JSON on disk
    assert (out_dir / "clone_result.json").is_file()
    applets = list(out_dir.glob("*_applet.h"))
    regs = list(out_dir.glob("*_registration.txt"))
    assert applets and regs

    saved = json.loads((out_dir / "clone_result.json").read_text(encoding="utf-8"))
    assert saved["fit"]["si_sdr_db"] == 12.5

    # limitations is the merged list (FitReport.limitations + agent's own)
    assert "fit-side caveat" in clone.limitations


def test_clone_effect_missing_file_raises(tmp_path):
    with pytest.raises(FileNotFoundError):
        clone_effect(str(tmp_path / "nope.wav"), str(tmp_path / "nope2.wav"), str(tmp_path))


def test_analyze_audio_tool(pipeline, wav_pair):
    session = CloneSession(dry_path="a", wet_path="b", out_dir=".")
    ctx = SimpleNamespace(deps=session)
    out = tools.analyze_audio(ctx, "a", "b")
    assert "Detected chain" in out
    assert session.chain == pipeline.chain
    assert session.analysis == pipeline.analysis
    assert session.dry is not None and session.wet is not None


def test_render_and_score_tool(pipeline):
    session = CloneSession(dry_path="a", wet_path="b", out_dir=".")
    ctx = SimpleNamespace(deps=session)
    out = tools.render_and_score(ctx, '{"stages": [], "output_gain_db": 0}')
    assert "si_sdr_db" in out
    assert len(session.fit_history) == 1
    assert session.chain is not None


def test_parse_chain_json_rejects_garbage():
    with pytest.raises(ModelRetry):
        tools.parse_chain_json("not json at all")
    with pytest.raises(ModelRetry):
        tools.parse_chain_json('{"stages": [{"kind": "nope"}]}')


def test_measure_stage_tool(pipeline):
    session = CloneSession(dry_path="a", wet_path="b", out_dir=".")
    ctx = SimpleNamespace(deps=session)
    out = tools.measure_stage(ctx, "eq")
    assert "measure_eq result" in out
    assert "lowpass" in out
    # alias: compressor kind maps to measure_dynamics
    out2 = tools.measure_stage(ctx, "compressor")
    assert "measure_dynamics result" in out2


def test_measure_stage_unknown_kind_retries():
    session = CloneSession(dry_path="a", wet_path="b", out_dir=".")
    ctx = SimpleNamespace(deps=session)
    with pytest.raises(ModelRetry):
        tools.measure_stage(ctx, "granular")


def test_emit_applet_tool_writes_files(pipeline, tmp_path):
    session = CloneSession(dry_path="a", wet_path="b", out_dir=str(tmp_path))
    session.chain = pipeline.chain
    ctx = SimpleNamespace(deps=session)
    out = tools.emit_applet(ctx, "MyClone")
    assert (tmp_path / "MyClone_applet.h").is_file()
    assert (tmp_path / "MyClone_registration.txt").is_file()
    assert "DeclareFancyApplet<MyCloneApplet>" in out
    assert session.applet_code and "class MyCloneApplet" in session.applet_code


def test_emit_applet_bad_name_retries(pipeline, tmp_path):
    session = CloneSession(dry_path="a", wet_path="b", out_dir=str(tmp_path))
    session.chain = pipeline.chain
    ctx = SimpleNamespace(deps=session)
    with pytest.raises(ModelRetry):
        tools.emit_applet(ctx, "TOOLONGNAME")


def test_emit_applet_without_chain_retries(tmp_path):
    session = CloneSession(dry_path="a", wet_path="b", out_dir=str(tmp_path))
    ctx = SimpleNamespace(deps=session)
    with pytest.raises(ModelRetry):
        tools.emit_applet(ctx, "X")


def test_session_arrays_cached(pipeline):
    session = CloneSession(dry_path="a", wet_path="b", out_dir=".")
    d1, w1, sr1 = session.arrays()
    d2, w2, sr2 = session.arrays()
    assert d1 is d2 and w1 is w2 and sr1 == sr2 == 48000
    assert pipeline.calls.count("load_arrays") == 1
