"""CLI tests for `ocapp-clone` (offline --chain path + basic UX).

The verify/analysis seams are stubbed so these tests never import the sibling
lanes; the wavs are synthesized in-test with soundfile.
"""

from __future__ import annotations

import json

import numpy as np
import pytest

from ocapp_agent import tools
from ocapp_agent.cli import main
from ocapp_agent.models import (
    BiquadStage,
    ChainModel,
    DelayModel,
    EqModel,
    FitReport,
)


@pytest.fixture(autouse=True)
def quiet_env(monkeypatch):
    monkeypatch.setenv("PYDANTIC_AI_NO_BANNER", "1")
    monkeypatch.delenv("LOGFIRE_TOKEN", raising=False)
    monkeypatch.delenv("OCA_LOGFIRE", raising=False)


@pytest.fixture
def wav_pair(tmp_path):
    import soundfile as sf

    dry = tmp_path / "dry.wav"
    wet = tmp_path / "wet.wav"
    sig = np.zeros(480, dtype=np.float32)
    sig[:10] = 0.5
    sf.write(str(dry), sig, 48000)
    sf.write(str(wet), sig * 0.7, 48000)
    return dry, wet


@pytest.fixture
def chain_file(tmp_path):
    chain = ChainModel(
        stages=[
            EqModel(stages=[BiquadStage(kind="peaking", freq_hz=1000.0, q=1.0, gain_db=3)]),
            DelayModel(time_ms=120, feedback=0.3, mix=0.4),
        ],
        output_gain_db=0.0,
    )
    path = tmp_path / "chain.json"
    path.write_text(chain.model_dump_json(indent=2), encoding="utf-8")
    return path


@pytest.fixture
def stub_verify(monkeypatch):
    calls: list[str] = []

    def fake_load_arrays(dry_path, wet_path):
        calls.append("load_arrays")
        return np.zeros(64, dtype=np.float32), np.zeros(64, dtype=np.float32), 48000

    def fake_verify(dry, wet, sr, chain):
        calls.append("verify_chain")
        return FitReport(si_sdr_db=17.25, log_spectral_distance=2.5)

    monkeypatch.setattr(tools, "_load_arrays", fake_load_arrays)
    monkeypatch.setattr(tools, "_verify_chain", fake_verify)
    return calls


def test_help_works(capsys):
    with pytest.raises(SystemExit) as exc:
        main(["--help"])
    assert exc.value.code == 0
    assert "ocapp-clone" in capsys.readouterr().out


def test_missing_dry_file_errors(tmp_path, capsys):
    with pytest.raises(SystemExit) as exc:
        main([str(tmp_path / "missing.wav"), str(tmp_path / "missing2.wav")])
    assert exc.value.code == 2
    assert "not found" in capsys.readouterr().err


def test_offline_chain_path_prints_fitreport(wav_pair, chain_file, stub_verify, tmp_path, capsys):
    dry, wet = wav_pair
    out_dir = tmp_path / "out"
    main([str(dry), str(wet), "--chain", str(chain_file), "-o", str(out_dir)])
    out = capsys.readouterr().out
    assert "FitReport" in out
    assert "si_sdr_db" in out
    assert "17.25" in out
    assert "verify_chain" in stub_verify
    # no LLM path: nothing but stdout, no artifacts
    assert not list(out_dir.glob("*_applet.h"))


def test_offline_emit_writes_applet(wav_pair, chain_file, stub_verify, tmp_path, capsys):
    dry, wet = wav_pair
    out_dir = tmp_path / "out"
    main(
        [str(dry), str(wet), "--chain", str(chain_file), "-o", str(out_dir), "--emit", "MyName"]
    )
    out = capsys.readouterr().out
    assert (out_dir / "MyName_applet.h").is_file()
    assert (out_dir / "MyName_registration.txt").is_file()
    assert "Wrote" in out
    code = (out_dir / "MyName_applet.h").read_text(encoding="utf-8")
    assert "class MyNameApplet" in code
    assert "AudioFilterBiquad" in code
    assert "AudioEffectDelay" in code


def test_offline_emit_flag_derives_name(wav_pair, chain_file, stub_verify, tmp_path, capsys):
    dry, wet = wav_pair
    out_dir = tmp_path / "out"
    main([str(dry), str(wet), "--chain", str(chain_file), "-o", str(out_dir), "--emit"])
    assert (out_dir / "chain_applet.h").is_file()  # stem of chain.json


def test_offline_emit_with_name_option(wav_pair, chain_file, stub_verify, tmp_path):
    dry, wet = wav_pair
    out_dir = tmp_path / "out"
    main(
        [
            str(dry),
            str(wet),
            "--chain",
            str(chain_file),
            "-o",
            str(out_dir),
            "--emit",
            "--name",
            "Named1",
        ]
    )
    assert (out_dir / "Named1_applet.h").is_file()


def test_bad_chain_json_errors(wav_pair, tmp_path, capsys):
    dry, wet = wav_pair
    bad = tmp_path / "bad.json"
    bad.write_text("{not json", encoding="utf-8")
    with pytest.raises(SystemExit) as exc:
        main([str(dry), str(wet), "--chain", str(bad)])
    assert exc.value.code == 2
    assert "invalid chain JSON" in capsys.readouterr().err


def test_bad_emit_name_errors(wav_pair, chain_file, stub_verify, tmp_path, capsys):
    dry, wet = wav_pair
    with pytest.raises(SystemExit) as exc:
        main(
            [str(dry), str(wet), "--chain", str(chain_file), "--emit", "TOOLONGNAME"]
        )
    assert exc.value.code == 2
    assert "invalid" in capsys.readouterr().err


def test_missing_chain_file_errors(wav_pair, tmp_path, capsys):
    dry, wet = wav_pair
    with pytest.raises(SystemExit) as exc:
        main([str(dry), str(wet), "--chain", str(tmp_path / "nope.json")])
    assert exc.value.code == 2
    assert "chain file not found" in capsys.readouterr().err


def test_offline_chain_json_roundtrip(wav_pair, chain_file, stub_verify, tmp_path):
    """The chain file we verify is exactly what codegen consumes."""
    dry, wet = wav_pair
    out_dir = tmp_path / "out"
    main(
        [str(dry), str(wet), "--chain", str(chain_file), "-o", str(out_dir), "--emit", "RT"]
    )
    saved = json.loads(chain_file.read_text(encoding="utf-8"))
    assert saved["stages"][0]["kind"] == "eq"
