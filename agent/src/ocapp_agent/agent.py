"""The `effect_cloner` agent: orchestrates analysis, iteration and codegen.

Workflow encoded in the instructions:

1. ``analyze_audio`` on the dry/wet pair -> detected ChainModel.
2. ``render_and_score`` the detected chain (offline render + scoring loop).
3. Refine: hand-edit ``chain_json`` between scores; ``measure_stage`` to
   re-estimate specific stages when the score is poor; iterate until the fit
   is acceptable (si_sdr_db > ~15 is convincing for linear-ish chains).
4. ``emit_applet`` with a short C++-safe name (<= 9 chars).
5. Return a CloneResult whose ``limitations`` merges FitReport.limitations
   with the agent's own honest caveats.

The model string comes from env ``OCA_MODEL`` (default ``openai:gpt-5.2``,
pydantic-ai ``provider:model`` format). Tests use
``agent.override(model=TestModel())`` - never assign ``agent.model`` directly.
"""

from __future__ import annotations

import os
from contextlib import nullcontext
from pathlib import Path
from typing import Any

from pydantic_ai import Agent
from pydantic_ai.exceptions import UserError
from pydantic_ai.models.test import TestModel

from ocapp_agent.models import CloneResult
from ocapp_agent.tools import CloneSession, register_tools

DEFAULT_MODEL = os.environ.get("OCA_MODEL", "openai:gpt-5.2")


class _MissingProviderModel(TestModel):
    """Stand-in model used when no provider is configured.

    Importing this module must work without any API key (tests use
    ``agent.override(model=TestModel())``). If a real run reaches the model
    without a provider, fail with a clear message instead of silently
    pretending.
    """

    _MSG = (
        "No LLM provider configured. Set OPENAI_API_KEY (or the key for your "
        "provider) or point OCA_MODEL at a configured provider:model, or "
        "pass model=... to clone_effect()."
    )

    async def request(self, *args: Any, **kwargs: Any):  # type: ignore[override]
        raise UserError(self._MSG)

    async def request_stream(self, *args: Any, **kwargs: Any):  # type: ignore[override]
        raise UserError(self._MSG)

INSTRUCTIONS = """\
You are an audio-effect cloner for the Phazerville firmware (Teensy 4.1, \
O.R.N.8 hardware: 8ch in/out, 48 kHz). Given a dry/wet recording pair of the \
same performance, you reproduce the effect as (a) a parametric effect chain \
and (b) a C++ HemisphereAudioApplet that runs it live.

Strict workflow - use the tools in this order:

1. analyze_audio(dry_path, wet_path) FIRST. It detects the chain
   (eq / delay / bitcrush / waveshaper / compressor / reverb stages) and
   stores the audio and analysis on the session.

2. render_and_score(chain_json) with the detected chain JSON. It renders the
   chain offline against the dry signal and scores the fit to the wet target.
   Read the score summary carefully.

3. Refine the chain, then score again. Iterate at least twice if the score is
   weak, but do not spin forever - 3 to 5 scoring rounds is plenty:
   - Hand-edit the chain_json parameters between render_and_score calls
     (it is a full ChainModel JSON document; adjust freqs, times, gains,
     mix values, etc.). Small numeric tweaks are the main lever.
   - Use measure_stage(stage_kind) to re-estimate a single stage with fresh
     eyes (stage_kind: eq, delay, reverb, dynamics, waveshaper, bitcrush)
     when that stage looks wrong, then fold its numbers into your chain_json.
   - Score guidance: si_sdr_db > ~15 is convincing for linear-ish chains
     (EQ, simple delay). Reverb and compressor clones are APPROXIMATIONS -
     even a good one rarely exceeds ~10 dB SI-SDR because the tail/envelope
     character differs. When the score is acceptably stable, stop.

4. emit_applet(name) with a short name: 1-9 characters of [A-Za-z0-9_]
   starting with a letter (it is the display name and the C++ class suffix).
   Prefer the user's requested name when the task provides one and it is
   valid; otherwise derive a short descriptive one. This writes
   <name>_applet.h and <name>_registration.txt.

5. Finish by returning the CloneResult (structured output):
   - chain: your final ChainModel (after refinements)
   - analysis: the AnalysisReport from analyze_audio
   - fit: the LAST FitReport from render_and_score
   - applet_name / applet_code / registration_snippet: from emit_applet
     (echo the tool's registration snippet; applet_code may be summarized
     from the truncated view - the authoritative file is on disk)
   - limitations: the AUTHORITATIVE merged list. Merge the last
     FitReport.limitations with your own honest caveats (every approximation
     you know of: Schroeder reverb character, compressor detector mismatch,
     delay feedback latency, baked waveshaper drive, block-based feedback,
     anything the offline score could not verify). Do not omit the
     FitReport.limitations entries.

Honesty rules:
- If the effect class is OUT OF SCOPE for this pipeline (pitch shifting,
  granular, spectral freeze, complex modulation like chorus/flanger/phaser
  with LFOs, anything time-varying), do NOT hallucinate a clone: emit the
  best partial chain that captures what you can (e.g. its static EQ or tone
  shape), score it once to document the residual, and say plainly in
  limitations which part could not be reproduced and why.
- Never claim a score you did not see. If render_and_score fails or the
  score is poor after a few rounds, say so in limitations.
- Approximate hardware mappings (compressor detector, reverb topology,
  delay feedback latency, table-quantized waveshaper) must appear in
  limitations when those stages are present.
"""


def _build_agent(model: str | None = None) -> Agent[CloneSession, CloneResult]:
    try:
        resolved: Any = model or DEFAULT_MODEL
        agent: Agent[CloneSession, CloneResult] = Agent(
            resolved,
            name="effect_cloner",
            deps_type=CloneSession,
            output_type=CloneResult,
            instructions=INSTRUCTIONS,
        )
    except UserError:
        # No provider configured (e.g. no API key in CI/tests): keep the agent
        # constructible so agent.override(model=...) still works; a real run
        # without a provider then fails with a clear message.
        agent = Agent(
            _MissingProviderModel(),
            name="effect_cloner",
            deps_type=CloneSession,
            output_type=CloneResult,
            instructions=INSTRUCTIONS,
        )
    register_tools(agent)
    return agent


#: the shared agent instance (tests replace its model via agent.override())
agent = _build_agent()


def _maybe_init_logfire() -> None:
    """Optional Logfire instrumentation; silently skipped unless enabled."""
    if not (os.environ.get("OCA_LOGFIRE") or os.environ.get("LOGFIRE_TOKEN")):
        return
    try:
        import logfire

        logfire.init()
        logfire.instrument_pydantic_ai()
    except Exception:  # noqa: BLE001 - observability must never break a run
        pass


def clone_effect(
    dry_path: str,
    wet_path: str,
    out_dir: str,
    model: str | None = None,
    name_hint: str | None = None,
) -> CloneResult:
    """Run the effect-cloner agent on a dry/wet pair and persist the result.

    Builds a CloneSession, runs the agent (model from ``OCA_MODEL`` unless
    `model` is given), and writes the resulting CloneResult JSON to
    `out_dir`/clone_result.json. Session-authoritative fields (chain,
    analysis, fit and the emitted applet code) are reconciled into the
    result so the JSON on disk always matches the artifacts on disk.

    `name_hint` (optional) suggests an applet display name to the agent.
    """
    _maybe_init_logfire()

    for p in (dry_path, wet_path):
        if not Path(p).is_file():
            raise FileNotFoundError(f"audio file not found: {p}")
    out_path = Path(out_dir)
    out_path.mkdir(parents=True, exist_ok=True)

    session = CloneSession(
        dry_path=dry_path,
        wet_path=wet_path,
        out_dir=out_dir,
        name_hint=name_hint,
    )
    prompt = (
        f"Clone the audio effect applied to the dry file {dry_path!r} to "
        f"produce the wet file {wet_path!r}. Follow the workflow exactly and "
        f"write artifacts to {out_dir!r}."
    )
    if name_hint:
        prompt += (
            f" Use the applet display name {name_hint!r} when emitting the "
            "applet, if it is valid (1-9 chars of [A-Za-z0-9_] starting "
            "with a letter)."
        )

    override_ctx = agent.override(model=model) if model else nullcontext()
    with override_ctx:
        result = agent.run_sync(prompt, deps=session)
    clone: CloneResult = result.output

    # Reconcile with session-authoritative data (the files on disk are the
    # source of truth; the model echoes summaries of them).
    updates: dict[str, Any] = {}
    if session.chain is not None:
        updates["chain"] = session.chain
    if session.analysis is not None:
        updates["analysis"] = session.analysis
    if session.fit_history:
        updates["fit"] = session.fit_history[-1]
    if session.applet_code is not None:
        updates["applet_code"] = session.applet_code
    if session.registration_snippet is not None:
        updates["registration_snippet"] = session.registration_snippet
    if updates:
        clone = clone.model_copy(update=updates)

    # Limitations is the AUTHORITATIVE merged list: FitReport.limitations +
    # the agent's own (deduped, order preserved).
    merged: list[str] = []
    for item in [*clone.fit.limitations, *clone.limitations]:
        if item not in merged:
            merged.append(item)
    clone = clone.model_copy(update={"limitations": merged})

    (out_path / "clone_result.json").write_text(
        clone.model_dump_json(indent=2), encoding="utf-8"
    )
    return clone
