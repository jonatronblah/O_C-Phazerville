"""CLI entrypoint for `ocapp-clone`.

Offline/CI path (no LLM):
    ocapp-clone DRY.wav WET.wav --chain chain.json [-o OUTDIR] [--emit NAME]
loads a ChainModel JSON, runs the deterministic render+score (verify_chain)
and prints the FitReport; ``--emit NAME`` additionally writes the applet via
generate_applet. ``--emit`` may be used as a flag (name from ``--name`` or
derived from the chain filename) or given its own NAME.

LLM path:
    ocapp-clone DRY.wav WET.wav [-o OUTDIR] [--name NAME]
runs the effect_cloner agent (see ocapp_agent.agent.clone_effect), prints a
summary and the output paths.
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path

from ocapp_agent.models import ChainModel, FitReport


def _derive_name(chain_path: str) -> str:
    """Best-effort applet name from the chain file stem, C++-safe, <= 9 chars."""
    stem = Path(chain_path).stem
    name = re.sub(r"[^A-Za-z0-9_]", "", stem)
    if not name or name[0].isdigit():
        name = "C" + name
    name = name[:9]
    return name


def _check_name(parser: argparse.ArgumentParser, name: str) -> str:
    from ocapp_agent.codegen import validate_name

    try:
        return validate_name(name)
    except ValueError as exc:
        parser.error(str(exc))
        raise  # unreachable; keeps type checkers happy


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        prog="ocapp-clone",
        description=(
            "Clone an audio effect from a dry/wet file pair into a "
            "Phazerville T4.1 HemisphereAudioApplet."
        ),
    )
    parser.add_argument("dry", help="dry (unprocessed) WAV file")
    parser.add_argument("wet", help="wet (processed) WAV file")
    parser.add_argument(
        "-o", "--outdir", default=".", help="output directory (default: current directory)"
    )
    parser.add_argument(
        "--chain",
        default=None,
        help="chain JSON to verify offline instead of running the LLM agent",
    )
    parser.add_argument(
        "--name",
        default=None,
        metavar="NAME",
        help="applet display name: 1-9 chars of [A-Za-z0-9_] starting with a letter",
    )
    parser.add_argument(
        "--emit",
        nargs="?",
        const=True,
        default=None,
        metavar="NAME",
        help=(
            "with --chain: also write the applet via generate_applet; "
            "optionally give the NAME here (otherwise --name or the chain "
            "file stem is used)"
        ),
    )
    args = parser.parse_args(argv)

    out_dir = Path(args.outdir)

    for label, p in (("dry", args.dry), ("wet", args.wet)):
        if not Path(p).is_file():
            parser.error(f"{label} file not found: {p}")

    if args.chain:
        chain_path = Path(args.chain)
        if not chain_path.is_file():
            parser.error(f"chain file not found: {args.chain}")
        _run_offline(parser, args, chain_path, out_dir)
    else:
        _run_agent(parser, args, out_dir)


def _run_offline(
    parser: argparse.ArgumentParser,
    args: argparse.Namespace,
    chain_path: Path,
    out_dir: Path,
) -> None:
    """--chain path: load ChainModel, verify, print FitReport, maybe emit."""
    from ocapp_agent import tools  # seams over the sibling lanes (lazy)
    from ocapp_agent.codegen import generate_applet

    try:
        chain = ChainModel.model_validate_json(chain_path.read_text(encoding="utf-8"))
    except Exception as exc:  # noqa: BLE001 - user-facing error
        parser.error(f"invalid chain JSON {chain_path}: {exc}")
        return

    out_dir.mkdir(parents=True, exist_ok=True)
    dry, wet, sr = tools._load_arrays(args.dry, args.wet)
    fit: FitReport = tools._verify_chain(dry, wet, sr, chain)

    print(f"Chain ({len(chain.stages)} stage(s), output {chain.output_gain_db:+g} dB):")
    for i, stage in enumerate(chain.stages):
        print(f"  [{i}] {stage.kind}: {stage.model_dump_json()}")
    print("\nFitReport:")
    print(fit.model_dump_json(indent=2))

    if args.emit is not None:
        name = (
            args.emit
            if isinstance(args.emit, str)
            else (args.name or _derive_name(str(chain_path)))
        )
        name = _check_name(parser, name)
        code, registration = generate_applet(chain, name)
        applet_path = out_dir / f"{name}_applet.h"
        reg_path = out_dir / f"{name}_registration.txt"
        applet_path.write_text(code, encoding="utf-8")
        reg_path.write_text(registration, encoding="utf-8")
        print(f"\nWrote {applet_path}")
        print(f"Wrote {reg_path}")


def _run_agent(
    parser: argparse.ArgumentParser, args: argparse.Namespace, out_dir: Path
) -> None:
    """LLM path: run the effect_cloner agent."""
    if args.name is not None:
        args.name = _check_name(parser, args.name)
    from ocapp_agent.agent import clone_effect

    try:
        clone = clone_effect(
            args.dry,
            args.wet,
            str(out_dir),
            name_hint=args.name,
        )
    except FileNotFoundError as exc:
        parser.error(str(exc))
        return

    print(f"applet_name : {clone.applet_name}")
    if clone.fit.si_sdr_db is not None:
        print(f"si_sdr_db   : {clone.fit.si_sdr_db:.2f}")
    print(f"limitations :")
    for item in clone.limitations:
        print(f"  - {item}")
    print("\nOutput files:")
    print(f"  {out_dir / 'clone_result.json'}")
    for f in sorted(out_dir.glob("*_applet.h")):
        print(f"  {f}")
    for f in sorted(out_dir.glob("*_registration.txt")):
        print(f"  {f}")


if __name__ == "__main__":
    main()
