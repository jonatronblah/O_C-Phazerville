# ocapp-agent

Clone an audio effect from a dry/wet file pair into a Phazerville T4.1
`HemisphereAudioApplet` — measure what the effect does, fit a chain of DSP
stages to it, score the fit offline, and emit C++ applet code.

## Layout

- `src/ocapp_agent/models.py` — the shared schema contract (pydantic models every
  other module builds against: `ChainModel`, `AnalysisReport`, `FitReport`,
  `CloneResult`)
- `analysis/` — deterministic measurement of the dry/wet pair (produces
  `AnalysisReport`)
- `verify.py` — offline render of a candidate chain + scoring (produces
  `FitReport`)
- `agent.py` + `codegen/` — pydantic-ai orchestration and C++ emission
  (consumes the models, produces `CloneResult`)

## Setup

Requires [uv](https://docs.astral.sh/uv/) and Python 3.12 (pinned via
`.python-version`; the range is `>=3.11,<3.13` for numba/llvmlite wheel
compatibility via librosa).

```sh
uv sync
uv run pytest
```

## CLI

```sh
uv run ocapp-clone DRY.wav WET.wav [-o OUTDIR] [--chain chain.json]
```

(Currently a stub — prints `not implemented yet`.)
