"""Pure C++ code generation for Phazerville audio applets.

`generate_applet(chain, name)` is a pure function — ChainModel in, C++ source
text out, no filesystem or network I/O — so it is fully testable offline.

The emitted code follows the conventions of the applets in
``software/src/audio_applets`` (see PassthruApplet.h for the minimal shape and
VCAApplet.h / FreeverbApplet.h for the full param + CV pattern):

* a ``HemisphereAudioApplet`` subclass with ``Start()`` wiring via ``PatchCable``
  (called from Start only, max 32 cables), ``Unload()`` releasing shared
  factory objects and calling ``AllowRestart()``;
* ``Controller()``/``View()``/``OnButtonPress()``/``OnEncoderMove()`` param UI
  with ``CVInputMap`` (the VCAApplet pattern) for up to 2 CV-mapped params;
* state save/load via ``OnDataRequest(std::array<uint64_t, CONFIG_SIZE>&)``
  with ``PackPackables``;
* ``InputStream()``/``OutputStream()`` pure virtuals returning AudioStream*.

Stage -> AudioStream mapping (mono chain, 48 kHz):

===========  =========================================================
ChainModel   AudioStream nodes
===========  =========================================================
eq           AudioFilterBiquad (4 biquads/object, cascaded chunks);
             lowpass/highpass/bandpass/notch via setLowpass() etc.,
             peaking via generated RBJ setPeakingCoeffs() helper,
             shelves via setLowShelf()/setHighShelf() (slope=1 approx)
delay        AudioEffectDelay tap 0 + AudioMixer4 feedback mixer
             (optional AudioFilterBiquad LPF in the feedback loop) +
             AudioMixer4 wet/dry mix
bitcrush     AudioEffectBitcrusher (bits(), sampleRate(48000/downsample))
waveshaper   AudioEffectWaveshaper with a 129-point float transfer table
             (linear interp); drive baked into the table; AudioMixer4
             wet/dry mix
compressor   AudioEffectDynamics via the shared GetComp()/ReleaseComp()
             factory; approximate param mapping (documented in comments)
reverb       AudioEffectReverbSchroeder via GetBungverb()/ReleaseBungverb()
             with setDecayTime(rt60_s)/setDamping(); optional
             AudioEffectDelay predelay; AudioMixer4 wet/dry mix
(output)     final AudioMixer4 gain = output_gain_db
===========  =========================================================
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from string import Template

from ocapp_agent.models import (
    BitcrushModel,
    ChainModel,
    CompressorModel,
    DelayModel,
    EqModel,
    ReverbModel,
    WaveshaperModel,
)

#: display names are <= 9 chars and must be C++-identifier-safe (they are
#: hashed into a stable applet_id() and used in the generated class name).
MAX_APPLET_NAME = 9
_NAME_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_]{0,8}$")

#: PatchCable() hard limit from HemisphereAudioApplet.h
MAX_CABLES = 32

#: transfer-table resolution for AudioEffectWaveshaper (2^7 + 1 points; the
#: class linearly interpolates between table points)
TABLE_POINTS = 129

#: Stock AudioEffectDelay keeps its queue in the shared audio-block pool
#: (RAM2). Long delays starve other objects; the firmware's own DelayApplet
#: switches to PSRAM-backed AudioDelayExt past this kind of length. We clamp
#: generated delay times and document it.
MAX_DELAY_MS = 1400.0

TARGET_SR = 48000


# ---------------------------------------------------------------------------
# name validation / firmware strhash
# ---------------------------------------------------------------------------

def validate_name(name: str) -> str:
    """Validate an applet display name.

    Must be a C++-identifier-safe string of 1..MAX_APPLET_NAME characters
    (letters, digits, underscore; must not start with a digit). Raises
    ValueError otherwise. The name is used for the display label, the
    generated class name (``<name>Applet``), the generated file name
    (``<name>_applet.h``) and the stable ``applet_id()`` hash.
    """
    if not isinstance(name, str) or not _NAME_RE.match(name):
        raise ValueError(
            f"applet name {name!r} is invalid: use 1-{MAX_APPLET_NAME} "
            "characters of [A-Za-z0-9_] starting with a letter"
        )
    return name


def cpp_strhash(s: str) -> int:
    """Replicate the firmware's constexpr strhash() from HSUtils.h.

    For each char: rotate-left-7 the 64-bit accumulator, then xor the char.
    """
    acc = 0
    for ch in s.encode("ascii", errors="strict"):
        acc = ((acc << 7) | (acc >> 57)) & 0xFFFFFFFFFFFFFFFF
        acc ^= ch
    return acc


# ---------------------------------------------------------------------------
# waveshaper transfer tables
# ---------------------------------------------------------------------------

def waveshaper_table(model: WaveshaperModel, n: int = TABLE_POINTS) -> list[float]:
    """Evaluate a WaveshaperModel into an n-point transfer table in [-1, 1].

    ``drive_db`` is baked into the table (input scaling before the curve) so
    no separate gain stage is needed; the trade-off is that drive is not
    live-adjustable on the device. For ``curve == 'table'`` the provided
    points are resampled onto the n-point grid (and the edges clamp, as
    np.interp does).
    """
    import numpy as np

    drive = 10.0 ** (model.drive_db / 20.0)
    drive = max(drive, 1e-3)
    xs = np.linspace(-1.0, 1.0, n)

    if model.curve == "table":
        pts = list(model.table) if model.table and len(model.table) >= 2 else [-1.0, 1.0]
        xp = np.linspace(-1.0, 1.0, len(pts))
        ys = np.interp(xs * drive, xp, np.asarray(pts, dtype=np.float64))
    elif model.curve == "hard_clip":
        ys = np.clip(xs * drive, -1.0, 1.0)
    elif model.curve == "soft_clip":
        # rational cubic soft clipper: unity slope at 0, f(3) = 1
        xd = np.clip(xs * drive * 3.0, -3.0, 3.0)
        ys = xd * (27.0 + xd * xd) / (27.0 + 9.0 * xd * xd)
    elif model.curve == "asymmetric":
        # steeper positive half than negative (approximation of asymmetric
        # saturation found in many pedals)
        t = math.tanh(drive)
        t2 = math.tanh(2.0 * drive)
        t = t if abs(t) > 1e-6 else 1.0
        t2 = t2 if abs(t2) > 1e-6 else 1.0
        ys = np.where(xs >= 0.0, np.tanh(2.0 * xs * drive) / t2, np.tanh(xs * drive) / t)
    else:  # 'tanh'
        t = math.tanh(drive)
        t = t if abs(t) > 1e-6 else 1.0
        ys = np.tanh(xs * drive) / t

    return [float(min(1.0, max(-1.0, float(y)))) for y in ys]


# ---------------------------------------------------------------------------
# live (encodermappable / CV-mappable) parameters
# ---------------------------------------------------------------------------

@dataclass
class LiveParam:
    """A user-facing parameter exposed on the applet UI + one CV input."""

    label: str            # short UI label (<= 5 chars keeps rows tidy)
    ctype: str            # C++ storage type: 'int8_t' or 'int16_t'
    vmin: int
    vmax: int
    step: int
    default: int
    unit_fmt: str         # graphics.printf format for the stored value
    apply: list[str]      # C++ lines; '%V%' is replaced by the effective
                          # float value expression

    @property
    def span(self) -> float:
        """CV (nominally -1..1) sweeps +/- span around the stored value."""
        return (self.vmax - self.vmin) / 2.0


def _fmt_float(x: float) -> str:
    """C++ float literal."""
    s = f"{x:.6g}"
    if "." not in s and "e" not in s:
        s += ".0"
    return s + "f"


class _StageEmit:
    """Accumulator for generated C++ fragments across the stage walk."""

    members: list[str]
    start: list[str]
    unload: list[str]
    apply: list[str]
    helpers: list[str]
    notes: list[str]
    params: list[tuple[str, LiveParam]]  # (owner stage tag, param)
    cables: int

    def __init__(self) -> None:
        self.members = []
        self.start = []
        self.unload = []
        self.apply = []
        self.helpers = []
        self.notes = []
        self.params = []
        self.cables = 0

    def cable(self, src: str, dst: str, dch: int = 0) -> None:
        """Emit PatchCable(src, 0, dst, dch); counted against MAX_CABLES."""
        self.cables += 1
        if self.cables > MAX_CABLES:
            raise ValueError(
                f"chain needs more than {MAX_CABLES} patch cables; "
                "simplify the chain"
            )
        self.start.append(f"    PatchCable({src}, 0, {dst}, {dch});")


# ---------------------------------------------------------------------------
# per-stage emitters
# ---------------------------------------------------------------------------

_SETTERS = {
    "lowpass": "setLowpass",
    "highpass": "setHighpass",
    "bandpass": "setBandpass",
    "notch": "setNotch",
}

_PEAKING_HELPER = """\
// RBJ peaking-EQ coefficients (AudioFilterBiquad has no setPeaking()).
static void setPeakingCoeffs(AudioFilterBiquad& f, uint32_t stage, float freq,
                             float q, float gain_db) {
  double a = pow(10.0, gain_db / 40.0);
  double w0 = freq * (2.0 * 3.14159265358979 / """ + str(TARGET_SR) + """.0);
  double sinw = sin(w0), cosw = cos(w0);
  double alpha = sinw / (2.0 * (double)q);
  double a0 = 1.0 + alpha / a;
  double coef[5];
  coef[0] = (1.0 + alpha * a) / a0;
  coef[1] = (-2.0 * cosw) / a0;
  coef[2] = (1.0 - alpha * a) / a0;
  coef[3] = (-2.0 * cosw) / a0;
  coef[4] = (1.0 - alpha / a) / a0;
  f.setCoefficients(stage, coef);
}
"""


def _emit_eq(e: _StageEmit, model: EqModel, k: int) -> str:
    """AudioFilterBiquad chunk(s); returns the output node name."""
    stages = [s for s in model.stages]
    if not stages:
        # An AudioFilterBiquad with no coefficients passes nothing, so an
        # empty EQ is simply omitted from the chain.
        e.notes.append("eq stage with 0 biquads was omitted (identity)")
        return "INPUT_NODE"

    node_in = "INPUT_NODE"
    for chunk_i in range(0, len(stages), 4):
        chunk = stages[chunk_i : chunk_i + 4]
        obj = f"eq{k}_{chunk_i // 4}" if len(stages) > 4 else f"eq{k}"
        e.members.append(f"  AudioFilterBiquad {obj};")
        setter_lines = []
        for i, s in enumerate(chunk):
            freq = _fmt_float(s.freq_hz)
            q = _fmt_float(s.q)
            if s.kind in _SETTERS:
                setter_lines.append(
                    f"    {obj}.{_SETTERS[s.kind]}({i}, {freq}, {q});"
                )
            elif s.kind in ("lowshelf", "highshelf"):
                meth = "setLowShelf" if s.kind == "lowshelf" else "setHighShelf"
                setter_lines.append(
                    f"    {obj}.{meth}({i}, {freq}, {_fmt_float(s.gain_db)}, 1.0f);"
                    " // Q mapped to shelf slope 1.0 (approx.)"
                )
            else:  # peaking
                if not any(h.startswith("// RBJ peaking") for h in e.helpers):
                    e.helpers.append(_PEAKING_HELPER)
                setter_lines.append(
                    f"    setPeakingCoeffs({obj}, {i}, {freq}, {q}, "
                    f"{_fmt_float(s.gain_db)});"
                )
        e.start.append(f"    // eq biquads {obj}")
        e.start.extend(setter_lines)
        e.cable(node_in, obj)
        node_in = obj

    # live param candidates: freq of the first builtin-setter biquad, or
    # gain of the first peaking/shelf biquad
    for i, s in enumerate(stages[:4]):
        obj = f"eq{k}" if len(stages) <= 4 else f"eq{k}_{i // 4}"
        if s.kind in _SETTERS:
            e.params.append((
                f"eq{k}",
                LiveParam(
                    label="Freq",
                    ctype="int16_t",
                    vmin=20,
                    vmax=18000,
                    step=10,
                    default=int(s.freq_hz),
                    unit_fmt="%5dHz",
                    apply=[
                        f"      {obj}.{_SETTERS[s.kind]}({i % 4}, %V%, {_fmt_float(s.q)});"
                    ],
                ),
            ))
            break
        if s.kind in ("peaking", "lowshelf", "highshelf"):
            if s.kind == "peaking":
                if not any(h.startswith("// RBJ peaking") for h in e.helpers):
                    e.helpers.append(_PEAKING_HELPER)
                apply = [
                    f"      setPeakingCoeffs({obj}, {i % 4}, {_fmt_float(s.freq_hz)}, "
                    f"{_fmt_float(s.q)}, %V%);"
                ]
            else:
                meth = "setLowShelf" if s.kind == "lowshelf" else "setHighShelf"
                apply = [
                    f"      {obj}.{meth}({i % 4}, {_fmt_float(s.freq_hz)}, %V%, 1.0f);"
                ]
            e.params.append((
                f"eq{k}",
                LiveParam(
                    label="Gain",
                    ctype="int8_t",
                    vmin=-24,
                    vmax=24,
                    step=1,
                    default=int(round(s.gain_db)),
                    unit_fmt="%4ddB",
                    apply=apply,
                ),
            ))
            break
    return node_in


def _emit_delay(e: _StageEmit, model: DelayModel, k: int) -> str:
    """AudioEffectDelay + feedback mixer + wet/dry mixer."""
    time_ms = min(float(model.time_ms), MAX_DELAY_MS)
    if model.time_ms > MAX_DELAY_MS:
        e.notes.append(
            f"delay time {model.time_ms:.0f}ms clamped to {MAX_DELAY_MS:.0f}ms "
            "(AudioEffectDelay keeps its queue in the shared RAM2 block pool; "
            "for longer delays port the chain to AudioDelayExt + PSRAM like "
            "DelayApplet)"
        )
    fb = min(max(float(model.feedback), 0.0), 0.95)
    mix = min(max(float(model.mix), 0.0), 1.0)

    fbm, delay, mixm = f"dly{k}_fb", f"dly{k}", f"dly{k}_mix"
    e.members.append(f"  AudioMixer4 {fbm};   // ch0 = input, ch1 = feedback")
    e.members.append(f"  AudioEffectDelay {delay};")
    e.start.append(f"    {delay}.delay(0, {_fmt_float(time_ms)});")
    e.start.append(f"    {fbm}.gain(0, 1.0f);")
    e.start.append(f"    {fbm}.gain(1, {_fmt_float(fb)});")
    e.start.append(f"    {fbm}.gain(2, 0.0f);")
    e.start.append(f"    {fbm}.gain(3, 0.0f);")

    e.cable("INPUT_NODE", fbm, 0)
    e.cable(fbm, delay, 0)

    if model.tone_hz is not None:
        tone = f"dly{k}_tone"
        e.members.append(f"  AudioFilterBiquad {tone};  // LPF in feedback loop")
        e.start.append(
            f"    {tone}.setLowpass(0, {_fmt_float(float(model.tone_hz))}, 0.7071f);"
        )
        e.cable(delay, tone, 0)
        e.cable(tone, fbm, 1)  # feedback return
    else:
        e.cable(delay, fbm, 1)  # feedback return

    e.members.append(f"  AudioMixer4 {mixm};  // ch0 = wet, ch1 = dry")
    e.start.append(f"    {mixm}.gain(0, {_fmt_float(mix)});")
    e.start.append(f"    {mixm}.gain(1, {_fmt_float(1.0 - mix)});")
    e.start.append(f"    {mixm}.gain(2, 0.0f);")
    e.start.append(f"    {mixm}.gain(3, 0.0f);")
    e.cable(delay, mixm, 0)       # wet
    e.cable("INPUT_NODE", mixm, 1)  # dry

    e.notes.append(
        "delay feedback loop passes through the audio graph, adding one "
        "audio-block (~2.7 ms) of latency per repeat compared to the offline "
        "model; very short slap-back settings will differ slightly"
    )

    e.params.append((
        f"dly{k}",
        LiveParam(
            label="Time",
            ctype="int16_t",
            vmin=1,
            vmax=int(MAX_DELAY_MS),
            step=1,
            default=int(round(time_ms)),
            unit_fmt="%5dms",
            apply=[f"      {delay}.delay(0, %V%);"],
        ),
    ))
    e.params.append((
        f"dly{k}",
        LiveParam(
            label="Fbk",
            ctype="int8_t",
            vmin=0,
            vmax=95,
            step=1,
            default=int(round(fb * 100)),
            unit_fmt="%4d%%",
            apply=[f"      {fbm}.gain(1, %V% * 0.01f);"],
        ),
    ))
    return mixm


def _emit_bitcrush(e: _StageEmit, model: BitcrushModel, k: int) -> str:
    obj = f"bc{k}"
    e.members.append(f"  AudioEffectBitcrusher {obj};")
    e.start.append(f"    {obj}.bits((uint8_t){int(model.bit_depth)});")
    e.start.append(
        f"    {obj}.sampleRate({_fmt_float(TARGET_SR / float(model.downsample))});"
        f" // downsample x{int(model.downsample)}"
    )
    e.cable("INPUT_NODE", obj)
    e.notes.append(
        "bitcrusher 'downsample' maps to AudioEffectBitcrusher::sampleRate() "
        "(sample doubling); its character differs from the offline model's "
        "decimation"
    )
    e.params.append((
        f"bc{k}",
        LiveParam(
            label="Bits",
            ctype="int8_t",
            vmin=1,
            vmax=16,
            step=1,
            default=int(model.bit_depth),
            unit_fmt="%4dbt",
            apply=[f"      {obj}.bits((uint8_t)(int)(%V% + 0.5f));"],
        ),
    ))
    e.params.append((
        f"bc{k}",
        LiveParam(
            label="Rate",
            ctype="int8_t",
            vmin=1,
            vmax=64,
            step=1,
            default=int(model.downsample),
            unit_fmt="%4dx",
            apply=[f"      {obj}.sampleRate(48000.0f / (float)(int)(%V% + 0.5f));"],
        ),
    ))
    return obj


def _emit_waveshaper(e: _StageEmit, model: WaveshaperModel, k: int) -> str:
    obj, mixm, tbl = f"ws{k}", f"ws{k}_mix", f"ws{k}_table"
    table = waveshaper_table(model, TABLE_POINTS)
    rows = []
    for r in range(0, len(table), 6):
        rows.append("    " + " ".join(f"{v: .6f}f," for v in table[r : r + 6]))
    e.members.append(
        f"  // transfer curve '{model.curve}' with drive {model.drive_db:g} dB\n"
        f"  // baked in; {TABLE_POINTS} points, linear interpolation in\n"
        f"  // AudioEffectWaveshaper::shape()\n"
        f"  float {tbl}[{TABLE_POINTS}] = {{\n" + "\n".join(rows) + "\n  };"
    )
    e.members.append(f"  AudioEffectWaveshaper {obj};")
    e.start.append(f"    {obj}.shape({tbl}, {TABLE_POINTS});")

    mix = min(max(float(model.mix), 0.0), 1.0)
    e.members.append(f"  AudioMixer4 {mixm};  // ch0 = wet, ch1 = dry")
    e.start.append(f"    {mixm}.gain(0, {_fmt_float(mix)});")
    e.start.append(f"    {mixm}.gain(1, {_fmt_float(1.0 - mix)});")
    e.start.append(f"    {mixm}.gain(2, 0.0f);")
    e.start.append(f"    {mixm}.gain(3, 0.0f);")
    e.cable("INPUT_NODE", obj, 0)
    e.cable(obj, mixm, 0)         # wet
    e.cable("INPUT_NODE", mixm, 1)  # dry

    e.notes.append(
        "waveshaper drive is baked into the lookup table (not live-"
        "adjustable); curve sampled to "
        f"{TABLE_POINTS} points with linear interpolation"
    )
    e.params.append((
        f"ws{k}",
        LiveParam(
            label="Mix",
            ctype="int8_t",
            vmin=0,
            vmax=100,
            step=1,
            default=int(round(mix * 100)),
            unit_fmt="%4d%%",
            apply=[
                f"      {mixm}.gain(0, %V% * 0.01f);",
                f"      {mixm}.gain(1, 1.0f - %V% * 0.01f);",
            ],
        ),
    ))
    return mixm


def _emit_compressor(e: _StageEmit, model: CompressorModel, k: int) -> str:
    ptr, out = f"cmp{k}", f"cmp{k}_out"
    e.members.append(f"  AudioEffectDynamics* {ptr} = nullptr;")
    e.members.append(f"  AudioPassthrough<MONO> {out};")
    atk = max(float(model.attack_ms), 0.001) / 1000.0
    rel = max(float(model.release_ms), 0.001) / 1000.0
    e.start.append(f"    {ptr} = GetComp();  // shared AudioEffectDynamics factory")
    e.start.append(f"    if ({ptr}) {{")
    e.start.append("      // Compressor mapping (approximate):")
    e.start.append("      //   threshold_db -> compression(threshold)")
    e.start.append("      //   attack_ms/1000 -> attack [s], release_ms/1000 -> release [s]")
    e.start.append("      //   ratio -> ratio, knee_db -> kneeWidth")
    e.start.append("      //   makeup_db -> makeupGain(). The detector is RMS-based;")
    e.start.append("      //   time constants and knee shape differ from the offline model.")
    e.start.append(
        f"      {ptr}->compression({_fmt_float(float(model.threshold_db))}, "
        f"{_fmt_float(atk)}, {_fmt_float(rel)}, "
        f"{_fmt_float(float(model.ratio))}, {_fmt_float(float(model.knee_db))});"
    )
    e.start.append(
        f"      {ptr}->makeupGain({_fmt_float(float(model.makeup_db))});"
    )
    e.start.append(f"      PatchCable(INPUT_NODE, 0, *{ptr}, 0);")
    e.start.append(f"      PatchCable(*{ptr}, 0, {out}, 0);")
    e.start.append("    } else {")
    e.start.append("      // factory exhausted: bypass the stage")
    e.start.append(f"      PatchCable(INPUT_NODE, 0, {out}, 0);")
    e.start.append("    }")
    e.cables += 2  # counted once for the if/else pair (one path is active)
    if e.cables > MAX_CABLES:
        raise ValueError(f"chain needs more than {MAX_CABLES} patch cables")
    e.unload.append(f"    if ({ptr}) {{ ReleaseComp({ptr}); {ptr} = nullptr; }}")
    e.notes.append(
        "compressor reproduced approximately with AudioEffectDynamics "
        "(shared factory); expect audible differences on transients"
    )
    e.params.append((
        f"cmp{k}",
        LiveParam(
            label="Thr",
            ctype="int8_t",
            vmin=-60,
            vmax=0,
            step=1,
            default=int(round(model.threshold_db)),
            unit_fmt="%4ddB",
            apply=[
                f"      if ({ptr}) {ptr}->compression(%V%, {_fmt_float(atk)}, "
                f"{_fmt_float(rel)}, {_fmt_float(float(model.ratio))}, "
                f"{_fmt_float(float(model.knee_db))});"
            ],
        ),
    ))
    return out


def _emit_reverb(e: _StageEmit, model: ReverbModel, k: int) -> str:
    ptr, mixm = f"rvb{k}", f"rvb{k}_mix"
    e.members.append(f"  AudioEffectReverbSchroeder* {ptr} = nullptr;")
    mix = min(max(float(model.mix), 0.0), 1.0)
    e.members.append(f"  AudioMixer4 {mixm};  // ch0 = wet, ch1 = dry")

    pre_node = "INPUT_NODE"
    if model.predelay_ms > 0:
        pre = f"rvb{k}_pre"
        e.members.append(f"  AudioEffectDelay {pre};")
        e.start.append(
            f"    {pre}.delay(0, {_fmt_float(min(float(model.predelay_ms), MAX_DELAY_MS))});"
        )
        e.cable("INPUT_NODE", pre)
        pre_node = pre

    e.start.append(f"    {ptr} = GetBungverb();  // shared Schroeder reverb factory")
    e.start.append(f"    if ({ptr}) {{")
    e.start.append(
        f"      {ptr}->setDecayTime({_fmt_float(float(model.rt60_s))});"
        f" // rt60_s"
    )
    e.start.append(
        f"      {ptr}->setDamping({_fmt_float(float(model.damping))});"
    )
    e.start.append(f"      PatchCable({pre_node}, 0, *{ptr}, 0);")
    e.start.append(f"      PatchCable(*{ptr}, 0, {mixm}, 0);  // wet")
    e.start.append("    }")
    e.cables += 2
    if e.cables > MAX_CABLES:
        raise ValueError(f"chain needs more than {MAX_CABLES} patch cables")
    e.cable("INPUT_NODE", mixm, 1)  # dry always passes
    e.start.append(
        f"    {mixm}.gain(0, {ptr} ? {_fmt_float(mix)} : 0.0f);"
    )
    e.start.append(
        f"    {mixm}.gain(1, {ptr} ? {_fmt_float(1.0 - mix)} : 1.0f);"
    )
    e.start.append(f"    {mixm}.gain(2, 0.0f);")
    e.start.append(f"    {mixm}.gain(3, 0.0f);")
    e.unload.append(f"    if ({ptr}) {{ ReleaseBungverb({ptr}); {ptr} = nullptr; }}")

    e.notes.append(
        "reverb is a Schroeder approximation (GetBungverb): rt60 maps to "
        "setDecayTime, damping to setDamping; room_size has no equivalent and "
        "is folded into decay time. Tail character will NOT match the target "
        "exactly"
    )
    e.params.append((
        f"rvb{k}",
        LiveParam(
            label="Mix",
            ctype="int8_t",
            vmin=0,
            vmax=100,
            step=1,
            default=int(round(mix * 100)),
            unit_fmt="%4d%%",
            apply=[
                f"      if ({ptr}) {{ {mixm}.gain(0, %V% * 0.01f); "
                f"{mixm}.gain(1, 1.0f - %V% * 0.01f); }}",
            ],
        ),
    ))
    e.params.append((
        f"rvb{k}",
        LiveParam(
            label="Dec",
            ctype="int16_t",
            vmin=1,
            vmax=200,
            step=1,
            default=int(round(float(model.rt60_s) * 10)),
            unit_fmt="%3d.%01ds",
            apply=[
                f"      if ({ptr}) {ptr}->setDecayTime(%V% * 0.1f);",
            ],
        ),
    ))
    return mixm


# ---------------------------------------------------------------------------
# class body assembly
# ---------------------------------------------------------------------------

_CLASS_TEMPLATE = Template("""\
class ${name}Applet : public HemisphereAudioApplet {
public:
  const char* applet_name() {
    return "${name}";
  }

  // Saved-state identity: strhash("${name}") computed at generation time
  // (0x${id_hex}ULL). It is intentionally NOT derived from applet_name(), so
  // renaming the display name later never orphans saved state / presets.
  const uint64_t applet_id() override {
    return 0x${id_hex}ULL;
  }

  void Start() override {
    // ---- signal graph wiring (PatchCable, max 32 cables) ----
${start}
  }

  void Unload() override {
    // release shared factory objects so other applets can use them
${unload}
    AllowRestart();
  }

  void Controller() override {
    ApplyParams();  // CV-modulated parameters
  }

  void View() override {
${view}
    gfxDisplayInputMapEditor();
  }

${button}
${encoder}

  void OnDataRequest(std::array<uint64_t, CONFIG_SIZE>& data) override {
    data[0] = ${pack0};
    data[1] = ${pack1};
    data[2] = 0;
    data[3] = 0;
  }

  void OnDataReceive(const std::array<uint64_t, CONFIG_SIZE>& data) override {
${unpack}
    ApplyParams();
  }

  AudioStream* InputStream() override {
    return &input;
  }
  AudioStream* OutputStream() override {
    return &out_mix;
  }

protected:
  void SetHelp() override {}

private:
  void ApplyParams() {
${apply}
  }

  // ---- DSP objects, in chain order ----
  AudioPassthrough<MONO> input;
${members}
  AudioMixer4 out_mix;  // ch0 = chain out * output gain

  // ---- parameters (packed into OnDataRequest) ----
${param_members}
  int8_t cursor = 0;
};
""")


def _build_view(param_specs: list[tuple[str, LiveParam]]) -> str:
    lines: list[str] = []
    for i, (var, p) in enumerate(param_specs):
        y = 15 + 10 * i
        lines.append(f'    gfxPrint(1, {y}, "{p.label}:");')
        lines.append("    gfxStartCursor();")
        if p.label == "Dec":  # rt60 stored in deciseconds: print as s.s
            lines.append(
                f"    graphics.printf(\"%3d.%01ds\", {var}_val / 10, "
                f"{var}_val % 10);"
            )
        else:
            lines.append(f'    graphics.printf("{p.unit_fmt}", {var}_val);')
        lines.append(f"    gfxEndCursor(cursor == {2 * i});")
        lines.append("    gfxStartCursor();")
        lines.append(f"    gfxPrint({var}_cv);")
        lines.append(
            f"    gfxEndCursor(cursor == {2 * i + 1}, false, {var}_cv.InputName());"
        )
    if not lines:
        lines.append(f'    gfxPrint(1, 15, "fixed chain");')
    return "\n".join(lines)


def _build_button(param_specs: list[tuple[str, LiveParam]]) -> str:
    if not param_specs:
        return ""
    indexed = ",\n        ".join(
        f"IndexedInput({2 * i + 1}, {var}_cv)" for i, (var, _) in enumerate(param_specs)
    )
    return f"""\
  void OnButtonPress() override {{
    if (CheckEditInputMapPress(cursor,
        {indexed}
    )) return;
    CursorToggle();
  }}"""


def _build_encoder(param_specs: list[tuple[str, LiveParam]]) -> str:
    if not param_specs:
        return "  void OnEncoderMove(int direction) override {}"
    cases = []
    for i, (var, p) in enumerate(param_specs):
        cases.append(f"""\
      case {2 * i}:
        {var}_val = constrain({var}_val + direction * {p.step}, {p.vmin}, {p.vmax});
        ApplyParams();
        break;
      case {2 * i + 1}:
        {var}_cv.ChangeSource(direction);
        break;""")
    switch = "\n".join(cases)
    return f"""\
  void OnEncoderMove(int direction) override {{
    if (!EditMode()) {{
      MoveCursor(cursor, direction, NUM_PARAMS - 1);
      return;
    }}
    if (EditSelectedInputMap(direction)) return;
    switch (cursor) {{
{switch}
    }}
  }}"""


def _build_apply(param_specs: list[tuple[str, LiveParam]]) -> str:
    blocks = []
    for var, p in param_specs:
        lines = [
            f"    // {p.label}: + CV (see {var}_cv)",
            "    {",
            f"      float v = constrain((float){var}_val + {var}_cv.InF() * "
            f"{_fmt_float(p.span)}, {_fmt_float(float(p.vmin))}, "
            f"{_fmt_float(float(p.vmax))});",
        ]
        for tpl in p.apply:
            lines.append("        " + tpl.replace("%V%", "v").strip())
        lines.append("    }")
        blocks.append("\n".join(lines))
    if not blocks:
        blocks.append("    // no live parameters (fixed chain)")
    return "\n".join(blocks)


# ---------------------------------------------------------------------------
# public API
# ---------------------------------------------------------------------------

def generate_applet(chain: ChainModel, name: str) -> tuple[str, str]:
    """Generate a Phazerville audio applet from a ChainModel.

    Returns ``(applet_code, registration_snippet)``: a complete C++ header
    implementing the chain as a ``HemisphereAudioApplet`` (mono, 48 kHz), and
    the ``DeclareFancyApplet<>`` registration lines to add to
    ``software/src/audio_applets/_config.h``.

    Raises ValueError for invalid names, chains needing more than 32 patch
    cables, or non-finite parameter values.
    """
    validate_name(name)
    chain = ChainModel.model_validate(chain)  # tolerate plain dicts too

    e = _StageEmit()
    node = "input"
    for k, stage in enumerate(chain.stages):
        emitter = {
            "eq": _emit_eq,
            "delay": _emit_delay,
            "bitcrush": _emit_bitcrush,
            "waveshaper": _emit_waveshaper,
            "compressor": _emit_compressor,
            "reverb": _emit_reverb,
        }[stage.kind]
        # INPUT_NODE is patched textually so stage emitters stay decoupled
        saved = e.start
        e.start = []
        out_node = emitter(e, stage, k)  # type: ignore[arg-type]
        block = [ln.replace("INPUT_NODE", node) for ln in e.start]
        e.start = saved
        e.start.extend(block)
        if out_node != "INPUT_NODE":
            node = out_node

    # ---- final output gain ----
    gain = 10.0 ** (float(chain.output_gain_db) / 20.0)
    e.start.append(f"    // output gain {chain.output_gain_db:+g} dB")
    e.start.append(f"    out_mix.gain(0, {_fmt_float(gain)});")
    e.start.append("    out_mix.gain(1, 0.0f);")
    e.start.append("    out_mix.gain(2, 0.0f);")
    e.start.append("    out_mix.gain(3, 0.0f);")
    e.cable(node, "out_mix")

    # ---- pick up to 2 live params (the CV-mapped controls) ----
    # Rule (documented in the emitted UI): the primary (first) param of the
    # first stage that has candidates, plus the primary param of the LAST
    # stage with candidates — or that same stage's secondary param when the
    # chain is a single stage.
    if e.params:
        tag_first = e.params[0][0]
        last_primary: tuple[str, LiveParam] | None = None
        seen: set[str] = set()
        for item in e.params:
            if item[0] not in seen:
                seen.add(item[0])
                last_primary = item
        assert last_primary is not None
        param_specs = [e.params[0]]
        if last_primary[0] != tag_first:
            param_specs.append(last_primary)
        elif len(e.params) > 1:
            param_specs.append(e.params[1])
    else:
        param_specs = []

    # ---- pack/unpack ----
    if not param_specs:
        pack0 = pack1 = "0"
        unpack = "    // no parameters to restore"
    else:
        vars_ = [var for var, _ in param_specs]
        pack0 = "PackPackables(" + ", ".join(f"{v}_val" for v in vars_) + ")"
        pack1 = "PackPackables(" + ", ".join(f"{v}_cv" for v in vars_) + ")"
        unpack = (
            "    UnpackPackables(data[0], "
            + ", ".join(f"{v}_val" for v in vars_)
            + ");\n"
            + "    UnpackPackables(data[1], "
            + ", ".join(f"{v}_cv" for v in vars_)
            + ");"
        )

    param_members = []
    for var, p in param_specs:
        param_members.append(f"  {p.ctype} {var}_val = {p.default};")
        param_members.append(f"  CVInputMap {var}_cv;")

    notes = e.notes or ["(none)"]
    json_dump = chain.model_dump_json(indent=2)
    header = "\n".join(
        [
            "#pragma once",
            "",
            "// " + "=" * 70,
            f"//  {name} - generated by ocapp-agent (effect chain cloner)",
            "//",
            f"//  Display name : \"{name}\" (<= {MAX_APPLET_NAME} chars)",
            "//  Hardware     : O.R.N.8 / Teensy 4.1, 8ch I/O, 48 kHz (mono chain)",
            f"//  Class        : {name}Applet",
            "//",
            "//  Source ChainModel (JSON):",
        ]
        + ["//  " + ln for ln in json_dump.splitlines()]
        + [
            "//",
            "//  Known fidelity limitations:",
        ]
        + [f"//   - {note}" for note in notes]
        + [
            "// " + "=" * 70,
            "",
            "#include <Audio.h>",
            '#include "../HemisphereAudioApplet.h"',
            '#include "../Audio/AudioPassthrough.h"',
            "",
            "// NOTE: include this header from software/src/audio_applets/_config.h",
            "// with the other applet includes (BEFORE `#undef applet_name`, so the",
            "// name macro rewrites the boilerplate below), then add the",
            "// DeclareFancyApplet<> line from the companion registration snippet.",
            "",
        ]
    )

    helpers = "\n".join(e.helpers)
    if helpers:
        helpers = "\n" + helpers + "\n"

    body = _CLASS_TEMPLATE.substitute(
        name=name,
        id_hex=f"{cpp_strhash(name):016x}",
        start="\n".join(e.start),
        unload="\n".join(e.unload) or "    // (no factory objects to release)",
        view=_build_view(param_specs),
        button=_build_button(param_specs),
        encoder=_build_encoder(param_specs),
        pack0=pack0,
        pack1=pack1,
        unpack=unpack,
        apply=_build_apply(param_specs),
        members="\n".join(e.members),
        param_members="\n".join(param_members) or "  // (fixed chain)",
    )
    num_params = len(param_specs) * 2
    num_params_decl = f"\n  static const int8_t NUM_PARAMS = {num_params};\n"
    # NUM_PARAMS must live in the private section; append before cursor decl
    body = body.replace(
        "  int8_t cursor = 0;",
        num_params_decl + "  int8_t cursor = 0;",
    )

    applet_code = header + helpers + body

    registration = "\n".join(
        [
            f"// ---- ocapp-agent registration snippet: {name} ----",
            "//",
            "// 1) In software/src/audio_applets/_config.h add the include with the",
            "//    other applet includes (BEFORE `#undef applet_name`):",
            "//",
            f'//        #include "{name}_applet.h"',
            "//",
            "// 2) Add the applet to the mono registry list:",
            "//",
            f"//        , DeclareFancyApplet<{name}Applet>",
            "//",
            "// Stereo note: this applet processes mono only (the analysis chain is",
            "// mono). For a stereo variant, template the class on AudioChannels and",
            "// run one chain per channel (see DelayApplet<STEREO> in DelayApplet.h),",
            "// then register with DeclareFancyApplet<...<STEREO>> in the stereo list.",
            "",
        ]
    )
    return applet_code, registration
