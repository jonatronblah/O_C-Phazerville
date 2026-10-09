"""Cross-lane integration smoke: analysis -> verify -> codegen on synthetic audio.

Not part of the pytest suite (owned by orchestrator for reconciliation).
Usage: uv run python tests/e2e_smoke.py
"""
import sys
import numpy as np
import soundfile as sf
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from ocapp_agent.analysis import load_audio, analyze_pair, TARGET_SR
from ocapp_agent.verify import verify_chain
from ocapp_agent.codegen import generate_applet
from ocapp_agent.models import ChainModel, EqModel, DelayModel, BiquadStage

rng = np.random.default_rng(7)
sr = TARGET_SR
t = np.arange(sr * 2) / sr
dry = 0.3 * np.sin(2 * np.pi * 220 * t) + 0.15 * np.sin(2 * np.pi * 550 * t) + 0.05 * rng.standard_normal(len(t))
dry = dry.astype(np.float32)

# Known effect: +8dB peaking @ 1kHz Q=2 (RBJ), then 120ms delay, fb=0.35, mix=0.5
from scipy.signal import lfilter
A = 10 ** (8 / 40)
w0 = 2 * np.pi * 1000 / sr
alpha = np.sin(w0) / (2 * 2.0)
b = [1 + alpha * A, -2 * np.cos(w0), 1 - alpha * A]
a = [1 + alpha / A, -2 * np.cos(w0), 1 - alpha / A]
eqd = lfilter(b, a, dry).astype(np.float32)
d_samp = int(0.120 * sr)
wet = np.zeros_like(eqd)
for n in range(len(eqd)):
    fb = wet[n - d_samp] * 0.35 if n >= d_samp else 0.0
    wet[n] = 0.5 * eqd[n] + 0.5 * (eqd[n - d_samp] + fb if n >= d_samp else eqd[n])
wet = wet.astype(np.float32)

tmp = Path(r"C:\Users\jonat\AppData\Local\Temp\opencode\ocapp_e2e")
tmp.mkdir(parents=True, exist_ok=True)
dry_p, wet_p = tmp / "dry.wav", tmp / "wet.wav"
sf.write(dry_p, dry, sr)
sf.write(wet_p, wet, sr)

print("== load_audio ==")
d, s1 = load_audio(dry_p)
w, s2 = load_audio(wet_p)
assert s1 == s2 == TARGET_SR and d.ndim == 1, "load_audio contract broken"
print(f"ok: loaded {len(d)/sr:.2f}s mono @ {s1}")

print("== analyze_pair ==")
chain, report = analyze_pair(d, w)
print(f"stages: {[s.kind for s in chain.stages]}")
print(f"lag={report.lag_samples} gain_db={report.gain_db_applied:.2f} parallel={report.is_parallel_mix}")
print(f"measurements: {report.measurements}")
assert any(s.kind == "eq" for s in chain.stages), "EQ not detected"
assert any(s.kind == "delay" for s in chain.stages), "delay not detected"
for s in chain.stages:
    if s.kind == "eq":
        print(f"  eq stages: {[(b.kind, round(b.freq_hz), round(b.gain_db,1), round(b.q,2)) for b in s.stages]}")
    if s.kind == "delay":
        print(f"  delay: {s.time_ms:.1f}ms fb={s.feedback:.2f} mix={s.mix:.2f}")

print("== verify_chain (detected chain vs ground truth wet) ==")
fit = verify_chain(d, w, sr, chain)
print(f"si_sdr_db={fit.si_sdr_db:.2f} log_spec_dist={fit.log_spectral_distance:.3f} loud_diff={fit.loudness_diff_lufs}")
print(f"stage_notes: {fit.stage_notes}")
print(f"limitations: {fit.limitations}")
assert fit.si_sdr_db is not None and fit.si_sdr_db > 5, f"fit too poor: {fit.si_sdr_db}"

print("== generate_applet (detected chain) ==")
code, reg = generate_applet(chain, "E2EClone")
for token in ["class E2ECloneApplet", "InputStream", "OutputStream", "PatchCable",
              "OnDataRequest", "OnDataReceive", "DeclareFancyApplet"]:
    assert token in (code + reg), f"missing {token} in emitted code"
assert code.count("{") == code.count("}"), "unbalanced braces"
print(f"ok: {len(code)} bytes C++, {len(reg)} bytes registration")

print("== round-trip: ChainModel JSON through verify+codegen ==")
chain2 = ChainModel.model_validate_json(chain.model_dump_json())
fit2 = verify_chain(d, w, sr, chain2)
assert abs((fit2.si_sdr_db or 0) - (fit.si_sdr_db or 0)) < 1e-6, "JSON round-trip changed the score"
print("ok: deterministic round-trip")

print("\nE2E SMOKE PASSED")
