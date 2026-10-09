"""pydantic-ai tools over the deterministic clone pipeline.

The four tools (``analyze_audio``, ``render_and_score``, ``measure_stage``,
``emit_applet``) are registered onto the shared ``effect_cloner`` agent via
:func:`register_tools` — the ``Agent.tool`` decorator form from the
building-pydantic-ai-agents skill, with ``RunContext[CloneSession]`` as the
first parameter (required for ``@agent.tool``).

Sibling lanes (``ocapp_agent.analysis`` / ``ocapp_agent.verify``) are imported
LAZILY inside the private ``_*`` seam functions at the bottom of this module,
so importing this module never pulls them in — and tests can monkeypatch the
seams without touching those packages.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Any

from pydantic_ai import ModelRetry, RunContext

from ocapp_agent.codegen import generate_applet, validate_name
from ocapp_agent.models import AnalysisReport, ChainModel, FitReport

if TYPE_CHECKING:  # pragma: no cover
    import numpy as np

TARGET_SR = 48000

#: stage kinds accepted by measure_stage -> analysis.measure_* function names
MEASURE_FNS: dict[str, str] = {
    "eq": "measure_eq",
    "delay": "measure_delay",
    "reverb": "measure_reverb",
    "dynamics": "measure_dynamics",
    "compressor": "measure_dynamics",  # ChainModel kind alias
    "waveshaper": "measure_waveshaper",
    "bitcrush": "measure_bitcrush",
}


@dataclass
class CloneSession:
    """Working state threaded through every tool via ``ctx.deps``.

    Holds the dry/wet paths and (lazily loaded) arrays, the current chain and
    analysis report, the running fit history, and emitted artifact paths.
    """

    dry_path: str
    wet_path: str
    out_dir: str = "."
    name_hint: str | None = None
    dry: "np.ndarray | None" = None
    wet: "np.ndarray | None" = None
    sr: int = TARGET_SR
    chain: ChainModel | None = None
    analysis: AnalysisReport | None = None
    fit_history: list[FitReport] = field(default_factory=list)
    emitted_files: list[str] = field(default_factory=list)
    applet_code: str | None = None
    registration_snippet: str | None = None

    def arrays(self) -> tuple["np.ndarray", "np.ndarray", int]:
        """Return (dry, wet, sr), loading the audio files on first use."""
        if self.dry is None or self.wet is None:
            self.dry, self.wet, self.sr = _load_arrays(self.dry_path, self.wet_path)
        return self.dry, self.wet, self.sr


# ---------------------------------------------------------------------------
# tools (registered via register_tools; RunContext first param per skill)
# ---------------------------------------------------------------------------

def analyze_audio(ctx: RunContext[CloneSession], dry_path: str, wet_path: str) -> str:
    """Analyze a dry/wet audio file pair and detect the effect chain.

    Runs the deterministic analyzers (EQ / delay / reverb / dynamics /
    waveshaper / bitcrush measurements) and stores the detected ChainModel,
    AnalysisReport and audio buffers on the session. Call this FIRST; every
    other tool depends on its results. Returns a compact JSON summary of the
    detected chain.
    """
    session = ctx.deps
    try:
        dry, wet, sr, chain, report = _load_and_analyze(dry_path, wet_path)
    except Exception as exc:  # noqa: BLE001 - surfaced to the model
        raise ModelRetry(f"analysis failed for ({dry_path!r}, {wet_path!r}): {exc}") from exc
    session.dry_path, session.wet_path = dry_path, wet_path
    session.dry, session.wet, session.sr = dry, wet, sr
    session.chain, session.analysis = chain, report
    summary = {
        "stages": [s.model_dump() for s in chain.stages],
        "output_gain_db": chain.output_gain_db,
        "analysis": report.model_dump(),
    }
    return (
        "Detected chain (stored on session):\n"
        + json.dumps(summary, indent=2)
        + "\nNext: call render_and_score with this chain JSON to score the fit."
    )


def render_and_score(ctx: RunContext[CloneSession], chain_json: str) -> str:
    """Render the given chain offline against the dry signal and score the fit.

    `chain_json` is a full ChainModel JSON document (you may hand-edit the
    parameters between calls to iterate). The chain is verified offline
    (render + SI-SDR / spectral / loudness scoring) against the wet target,
    the FitReport is appended to the session history and the current chain is
    updated. Returns a compact score summary.
    """
    session = ctx.deps
    chain = parse_chain_json(chain_json)
    dry, wet, sr = session.arrays()
    try:
        fit = _verify_chain(dry, wet, sr, chain)
    except Exception as exc:  # noqa: BLE001
        raise ModelRetry(f"render/verify failed: {exc}") from exc
    session.chain = chain
    session.fit_history.append(fit)
    summary = {
        "score": fit.model_dump(),
        "scores_so_far": len(session.fit_history),
        "chain": [s.model_dump() for s in chain.stages],
    }
    return (
        "Fit report:\n"
        + json.dumps(summary, indent=2)
        + "\nGuidance: si_sdr_db > ~15 is convincing for linear-ish chains "
        "(eq/delay). Reverb and compressor clones are approximations - "
        "iterate a few times, then accept and note the residuals in "
        "limitations. Re-run measure_stage for stages that score poorly."
    )


def measure_stage(ctx: RunContext[CloneSession], stage_kind: str) -> str:
    """Re-measure one stage kind with fresh eyes on the session's audio.

    stage_kind is one of: eq, delay, reverb, dynamics (compressor), waveshaper,
    bitcrush. Returns the measured parameter model(s) as JSON - use them to
    hand-edit the chain_json you pass to render_and_score.
    """
    session = ctx.deps
    fn_name = MEASURE_FNS.get(stage_kind.strip().lower())
    if fn_name is None:
        raise ModelRetry(
            f"unknown stage_kind {stage_kind!r}; use one of: "
            + ", ".join(sorted(MEASURE_FNS))
        )
    dry, wet, sr = session.arrays()
    try:
        result = _measure_stage(fn_name, dry, wet, sr)
    except Exception as exc:  # noqa: BLE001
        raise ModelRetry(f"measure_{stage_kind} failed: {exc}") from exc
    return f"{fn_name} result:\n{_fmt_measurement(result)}"


def emit_applet(ctx: RunContext[CloneSession], name: str) -> str:
    """Generate the Phazerville C++ applet for the session's current chain.

    `name` is the applet display name: 1-9 chars of [A-Za-z0-9_] starting
    with a letter. Writes `<name>_applet.h` and `<name>_registration.txt`
    into the session out_dir. Returns the file paths and the registration
    snippet (code is truncated here; the full text is stored on the session
    and written to disk).
    """
    session = ctx.deps
    if session.chain is None:
        raise ModelRetry("no chain on session: call analyze_audio first")
    try:
        validate_name(name)
    except ValueError as exc:
        raise ModelRetry(str(exc)) from exc
    try:
        code, registration = generate_applet(session.chain, name)
    except Exception as exc:  # noqa: BLE001
        raise ModelRetry(f"code generation failed: {exc}") from exc

    out_dir = Path(session.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    applet_path = out_dir / f"{name}_applet.h"
    reg_path = out_dir / f"{name}_registration.txt"
    applet_path.write_text(code, encoding="utf-8")
    reg_path.write_text(registration, encoding="utf-8")
    session.applet_code = code
    session.registration_snippet = registration
    session.emitted_files.extend([str(applet_path), str(reg_path)])

    truncated = "\n".join(code.splitlines()[:60])
    return (
        f"Wrote {applet_path} and {reg_path}.\n\n"
        "Registration snippet:\n"
        + registration
        + "\nFirst 60 lines of applet code (full code is on disk and in "
        "CloneResult.applet_code):\n"
        + truncated
    )


def register_tools(agent: Any) -> Any:
    """Register the four tools onto `agent` (skill's @agent.tool pattern)."""
    agent.tool(analyze_audio)
    agent.tool(render_and_score)
    agent.tool(measure_stage)
    agent.tool(emit_applet)
    return agent


# ---------------------------------------------------------------------------
# parsing / formatting helpers
# ---------------------------------------------------------------------------

def parse_chain_json(chain_json: str) -> ChainModel:
    """Parse a ChainModel JSON document; raises ModelRetry when invalid."""
    try:
        return ChainModel.model_validate_json(chain_json)
    except Exception as exc:  # ValidationError etc.
        raise ModelRetry(
            f"chain_json is not a valid ChainModel: {exc}. It must be a JSON "
            'object like {"stages": [{"kind": "eq", ...}], "output_gain_db": 0}. '
            "Stage kinds: eq, delay, bitcrush, waveshaper, compressor, reverb."
        ) from exc


def _fmt_measurement(result: Any) -> str:
    """Format a measure_* result (pydantic model | list | None) as JSON."""
    if result is None:
        return "null (nothing measured)"
    if isinstance(result, list):
        return json.dumps(
            [r.model_dump() if hasattr(r, "model_dump") else r for r in result],
            indent=2,
        )
    if hasattr(result, "model_dump"):
        return json.dumps(result.model_dump(), indent=2)
    return json.dumps(result, indent=2, default=str)


# ---------------------------------------------------------------------------
# lazy-import seams over the sibling lanes (monkeypatch these in tests)
# ---------------------------------------------------------------------------

def _load_and_analyze(
    dry_path: str, wet_path: str
) -> tuple["np.ndarray", "np.ndarray", int, ChainModel, AnalysisReport]:
    """load_audio both files + analyze_pair (ocapp_agent.analysis)."""
    from ocapp_agent import analysis  # sibling lane A

    dry, sr = analysis.load_audio(dry_path)
    wet, _ = analysis.load_audio(wet_path)
    chain, report = analysis.analyze_pair(dry, wet, sr)
    return dry, wet, sr, chain, report


def _load_arrays(
    dry_path: str, wet_path: str
) -> tuple["np.ndarray", "np.ndarray", int]:
    """load_audio both files only (ocapp_agent.analysis)."""
    from ocapp_agent import analysis  # sibling lane A

    dry, sr = analysis.load_audio(dry_path)
    wet, _ = analysis.load_audio(wet_path)
    return dry, wet, sr


def _verify_chain(
    dry: "np.ndarray", wet: "np.ndarray", sr: int, chain: ChainModel
) -> FitReport:
    """verify_chain (ocapp_agent.verify): render the chain, score vs wet."""
    from ocapp_agent import verify  # sibling lane B

    return verify.verify_chain(dry, wet, sr, chain)


def _measure_stage(
    fn_name: str, dry: "np.ndarray", wet: "np.ndarray", sr: int
) -> Any:
    """Call analysis.measure_<fn_name>(dry, wet, sr)."""
    from ocapp_agent import analysis  # sibling lane A

    fn = getattr(analysis, fn_name)
    return fn(dry, wet, sr)
