#pragma once

#include "../src/Audio/effect_looper.h"
#include "../Audio/AudioMixer.h"
#include "../Audio/AudioPassthrough.h"
#include "../dsputils.h"
#include <Audio.h>

// ClockLoop - clockable stereo audio looper with buffer scrubbing.
//
// Signal chain (the looper is a custom AudioStream; everything else is
// stock graph glue):
//
//   in L ─┬─ AudioLooper (stereo, interleaved PSRAM buffer) ─┐
//   in R ─┘                                                  ├─ (dry/wet) ─ out L/R
//   in L/R ──────────────────────── (dry leg, equal-power mix) ┘
//
// Transport (state machine lives in the applet; see effect_looper.h for the
// block-atomic transition contract - update() never performs transitions):
//   Stopped → Recording → Playing/Looping ⇄ Overdub → One-shot
//
// Clockable: the loop length is N clock cycles (Clock Div N ∈ 1,2,4,8,16).
// The clock source is a DigitalInputMap (default CL1 internal clock) whose
// period is measured in control ticks (16.6 kHz) - the same beat-period
// tracking pattern as GlitchApplet/DelayApplet.  Recording starts on a
// clock edge when armed with "Qnt" on (clock-quantized), or immediately
// otherwise, and auto-stops after exactly N measured cycles (manual Trig1
// stop is also possible; the loop then keeps the recorded length).
//
// EDGE CASE (documented decision): changing Clock Div while a loop plays
// restarts playback cleanly at the next clock edge with the new length
// (N x measured period) - no rescaling of the recorded material.  If no
// clock has been measured yet, Clock Div only sets the auto-stop target of
// the next recording.  Reverse / Half / Double keep the recorded loop
// bounds fixed; the wall-clock loop period scales with the speed (Half = 2x
// period, Double = 1/2x, Reverse = same period backwards).
//
// Overdub: Rec Mode {Mix, Replace} x Dub Level (0-100%) x Feedback (0-100%,
// Frippertronics-style decay of the previous content on each dub pass).
//   Mix:     buf = old*feedback + in*dub
//   Replace: buf = old*feedback*(1-dub) + in*dub
//
// Scrub Mode {Loop, Scrub, Rev, Half, Dbl}:  Scrub positions the playhead
// directly from CV (0..100% of the buffer, one-pole slew ~1.3 ms - that
// slew IS the click-prevention for scrubbing).  Scrub Range scales how much
// of the loop the bipolar CV covers around the midpoint (100% = full loop).
// Other discontinuities (loop wrap, mode jumps, retrigger) get a ~5 ms
// output-domain crossfade from the last emitted sample, plus a one-time
// tail/head splice crossfade when a recording is committed to a loop.
//
// Trig1 = arm record (clock-quantized when "Qnt" is on) / stop recording.
//   Rising edge only; the gate length is ignored.  While playing it re-arms
//   a fresh recording (the loop is overwritten once it starts).
// Trig2 = jump-to-start one-shot retrigger (plays from 0; with "1S" on the
//   loop plays once and stops).
//
// MEMORY LAYOUT NOTE (from the MidSider build): GCC drops section attrs on
// template/inline members, so ALL heavy code lives in the non-template
// ClockLoopCore below with strong FLASHMEM symbols; ClockLoop<Channels> is
// the thin registered wrapper.  UI strings use PSTR() so they land in flash
// (.progmem) instead of the DTCM .rodata pool - the RAM1/DTCM stack budget
// is only ~19 KB.  The loop buffer is PSRAM (~8 s) with a ~2 s RAM2 malloc
// fallback + "No PSRAM" note (GlitchApplet/WAVRecorder precedent).

// Name string forced into flash (.progmem) to protect the DTCM/.rodata
// budget; memory-mapped flash reads are fine for display + strhash().
static const char CLOCKLOOP_NAME[] PROGMEM = "ClockLoop";

class ClockLoopCore : public HemisphereAudioApplet {
public:
  // (applet_name is macro-rewritten by _config.h into a static accessor)
  const char* applet_name() {
    return CLOCKLOOP_NAME;
  }

  void Start() final;
  void Unload() final;
  void mainloop() final;
  void Controller() final;
  void View() final;
  void DrawFullScreen() override;
  void OnButtonPress() override;
  void OnEncoderMove(int direction) override;
  void OnDataRequest(std::array<uint64_t, CONFIG_SIZE>& data) override;
  void OnDataReceive(const std::array<uint64_t, CONFIG_SIZE>& data) override;

  AudioStream* InputStream() override {
    return &input;
  }
  AudioStream* OutputStream() override {
    return &output;
  }

protected:
  void SetHelp() override;

private:
  enum Cursor : int8_t {
    // Page 0: transport / loop
    T_MODE = 0, CLK_DIV, REC_MODE,
    DUB_LVL, DUB_CV,
    FBK_LVL, FBK_CV,
    // Page 1: output / scrub
    MIX_LVL, MIX_CV,
    SCRUB_MODE, SCRUB_CV, SCRUB_RANGE,
    ONESHOT, QUANT_ARM,
    CURSOR_LENGTH,
  };

  // transport requests from the UI (consumed in Controller())
  enum TransportReq : uint8_t {
    TREQ_STOP = 0, TREQ_REC, TREQ_DUB, TREQ_PLAY, TREQ_NONE = 0xFF,
  };

  // ---- params (state round-trip via OnDataRequest/Receive) ----------------
  uint8_t scrub_mode = AudioLooper::MODE_LOOP; // PlayMode
  uint8_t clock_div = 2;    // 0..4 -> loop = 1,2,4,8,16 clock cycles
  uint8_t rec_mode = 0;     // 0 = Mix, 1 = Replace
  uint8_t dub_level = 50;   // 0..100%
  uint8_t feedback = 100;   // 0..100%
  uint8_t mix = 50;         // 0..100% dry..wet
  uint8_t scrub_range = 100; // 25..200%
  bool one_shot = false;
  bool quant_arm = true;

  CVInputMap scrub_cv;
  CVInputMap mix_cv;
  CVInputMap dub_cv;
  CVInputMap fdb_cv;

  DigitalInputMap clock_source; // loop sync clock (period measurement)
  DigitalInputMap arm_input;    // Trig1: arm record / stop
  DigitalInputMap retrig_input; // Trig2: jump to start / one-shot

  // ---- control-tick bookkeeping ------------------------------------------
  int8_t cursor = T_MODE;
  uint8_t transport_sel = TREQ_PLAY; // UI selector for the Trn row
  uint8_t pending_transport = TREQ_NONE;
  uint32_t clock_count = 0;
  uint32_t period_ticks = 0; // control ticks between the last two clocks
  bool armed = false;        // waiting for a clock edge to start recording
  bool pending_clkdiv = false;
  bool alloc_ok = false;

  AudioPassthrough<2> input;
  AudioLooper looper;
  AudioMixer<2> mix_l;
  AudioMixer<2> mix_r;
  AudioPassthrough<2> output;

  uint32_t TargetFrames();
  void StartRecordingNow();
  void ApplyTransport(uint8_t req);
  void DrawStateIndicator(int x0);
};

// Thin template wrapper so the registry can use ClockLoop<STEREO> form.
// (All real work lives in the non-template core so FLASHMEM section
// placement survives this toolchain; Channels is intentionally unused and
// the wrapper overrides nothing, so it shares the core's vtable layout.)
template <AudioChannels Channels>
class ClockLoop : public ClockLoopCore {
};

// -- Out-of-line definitions (strong symbols, FLASHMEM honored) --------------

// Recording auto-stop target in frames: N clock cycles from the measured
// control-tick period (16.6 kHz ticks -> 48 kHz samples).
FLASHMEM uint32_t ClockLoopCore::TargetFrames() {
  if (period_ticks == 0) return 0; // no clock: manual recording length
  const uint32_t n = 1ul << clock_div;
  float frames = (float)period_ticks * (48000.0f / 16666.0f) * (float)n;
  const uint32_t maxf = looper.CapacityFrames();
  if (frames < 2.0f) return 0;
  if (frames > (float)maxf) return maxf;
  return (uint32_t)frames;
}

FLASHMEM void ClockLoopCore::StartRecordingNow() {
  armed = false;
  looper.startRecording(TargetFrames());
}

FLASHMEM void ClockLoopCore::ApplyTransport(uint8_t req) {
  switch (req) {
    case TREQ_STOP:
      armed = false;
      looper.stop();
      break;
    case TREQ_REC:
      if (looper.state() == AudioLooper::LOOPER_RECORDING) {
        looper.stopRecordingToPlay();
        looper.clearFlags();
      } else if (quant_arm && period_ticks > 0) {
        armed = true;
      } else {
        StartRecordingNow();
      }
      break;
    case TREQ_DUB:
      armed = false;
      looper.setOverdub(true);
      break;
    case TREQ_PLAY:
      armed = false;
      if (looper.state() == AudioLooper::LOOPER_OVERDUB) {
        looper.setOverdub(false);
      } else if (looper.state() == AudioLooper::LOOPER_STOPPED
                 && looper.loopLenFrames() >= 2) {
        looper.restartPlayback(one_shot);
      }
      break;
    default:
      break;
  }
}

FLASHMEM void ClockLoopCore::Start() {
  alloc_ok = looper.begin();
  if (!alloc_ok) return;

  // Sensible input-map defaults (preserved when a map was already assigned)
  if (!clock_source.enabled()) clock_source.SetClockSource(0); // CL1
  if (!arm_input.enabled()) arm_input.SetGateInput(0);         // Trig1
  if (!retrig_input.enabled()) retrig_input.SetGateInput(1);   // Trig2

  PatchCable(input, 0, looper, 0);
  PatchCable(input, 1, looper, 1);
  PatchCable(input, 0, mix_l, 0); // dry
  PatchCable(input, 1, mix_r, 0);
  PatchCable(looper, 0, mix_l, 1); // wet
  PatchCable(looper, 1, mix_r, 1);
  PatchCable(mix_l, 0, output, 0);
  PatchCable(mix_r, 0, output, 1);
  // 8 cables total (<= 32)
}

FLASHMEM void ClockLoopCore::Unload() {
  armed = false;
  period_ticks = 0;
  clock_count = 0;
  looper.end();
  alloc_ok = false;
  AllowRestart();
}

FLASHMEM void ClockLoopCore::mainloop() {
  if (!alloc_ok) return;
  // Tail/head splice for freshly committed recordings (loop context, so
  // AudioNoInterrupts() here is fully atomic against the audio ISR)
  looper.commitSplice();
}

// 16.6 kHz control tick: edge detection, the transport state machine and
// parameter pushes.  All looper transitions go through the block-atomic API
// documented in effect_looper.h (never update()).
FLASHMEM void ClockLoopCore::Controller() {
  if (!alloc_ok) return;

  // --- clock period measurement (control ticks between clock edges) -------
  clock_count++;
  const bool clk = clock_source.Clock();
  if (clk) {
    if (clock_count > 1) period_ticks = clock_count;
    clock_count = 0;
  }
  const bool arm_trig = arm_input.Clock();
  const bool retrig = retrig_input.Clock();

  // --- UI transport request (from the Trn row) ----------------------------
  if (pending_transport != TREQ_NONE) {
    ApplyTransport(pending_transport);
    pending_transport = TREQ_NONE;
  }

  // --- Trig1: arm record (quantized) / stop -------------------------------
  if (arm_trig) {
    if (armed) {
      armed = false; // disarm
    } else if (looper.state() == AudioLooper::LOOPER_RECORDING) {
      looper.stopRecordingToPlay();
      looper.clearFlags();
    } else if (quant_arm && period_ticks > 0) {
      armed = true;
    } else {
      StartRecordingNow();
    }
  }

  // clock-quantized record start
  if (armed && clk && looper.state() != AudioLooper::LOOPER_RECORDING) {
    StartRecordingNow();
  }

  // --- Trig2: jump-to-start one-shot retrigger ----------------------------
  if (retrig && looper.loopLenFrames() >= 2) {
    looper.restartPlayback(one_shot);
  }

  // --- flags from the audio ISR (data commits, transport flips here) ------
  if (looper.recComplete()) {
    looper.stopRecordingToPlay();
    looper.clearFlags();
  }
  if (looper.oneShotDone()) {
    looper.stop();
    looper.clearFlags();
  }

  // --- Clock Div change: clean restart at the next clock edge -------------
  if (pending_clkdiv && clk && period_ticks > 0) {
    pending_clkdiv = false;
    if (looper.loopLenFrames() >= 2) looper.restartWithLength(TargetFrames());
  }

  // --- parameter pushes (atomic single-word setters) ---------------------
  looper.setPlayMode(scrub_mode);
  looper.setOneShot(one_shot);
  looper.setRecReplace(rec_mode != 0);
  looper.setDubLevel(
    (uint8_t)constrain(dub_level + dub_cv.InF() * 50.0f, 0.0f, 100.0f));
  looper.setFeedback(
    (uint8_t)constrain(feedback + fdb_cv.InF() * 50.0f, 0.0f, 100.0f));

  // scrub CV: 0.5 + cv * (range/200), clamped -> Q13 playhead target
  const float loopf = (float)looper.loopLenFrames();
  if (loopf >= 2.0f) {
    float posn = 0.5f + scrub_cv.InF() * (scrub_range * 0.005f);
    posn = constrain(posn, 0.0f, 1.0f);
    looper.setScrubTarget((uint32_t)(posn * loopf * 8192.0f));
  }

  // equal-power dry/wet mix (block-rate gains, like GlitchApplet)
  float dry_gain, wet_gain;
  EqualPowerFade(dry_gain, wet_gain,
                 constrain(0.01f * (mix + mix_cv.InF() * 50.0f), 0.0f, 1.0f));
  mix_l.gain(0, dry_gain);
  mix_l.gain(1, wet_gain);
  mix_r.gain(0, dry_gain);
  mix_r.gain(1, wet_gain);

  looper.decayPeak();
}

// --- UI --------------------------------------------------------------------

FLASHMEM void ClockLoopCore::View() {
  if (!alloc_ok) {
    gfxPrint(2, 15, PSTR("Out of RAM!!"));
    return;
  }

  const int8_t page = (cursor >= MIX_LVL);
  gfxRect(page * 15, 10, 14, 3); // page tab

  if (page == 0) { // ---- transport / loop ----
    gfxPrint(1, 15, PSTR("Trn:"));
    gfxStartCursor();
    switch (looper.state()) {
      case AudioLooper::LOOPER_RECORDING: gfxPrint(PSTR("Rec")); break;
      case AudioLooper::LOOPER_OVERDUB: gfxPrint(PSTR("Ovr")); break;
      case AudioLooper::LOOPER_PLAYING:
        gfxPrint(one_shot ? PSTR("1Sht") : PSTR("Play"));
        break;
      default: gfxPrint(armed ? PSTR("Arm") : PSTR("Stop")); break;
    }
    gfxEndCursor(cursor == T_MODE);
    if (!looper.IsPSRAM()) {
      gfxPos(34, 15);
      gfxPrint(PSTR("2sRAM"));
    }

    gfxPrint(1, 25, PSTR("Clk:"));
    gfxStartCursor();
    graphics.printf(PSTR("x%u"), 1u << clock_div);
    gfxEndCursor(cursor == CLK_DIV);
    // loop length display (seconds)
    const uint32_t ms = looper.loopLenFrames() / 48; // ms
    if (ms > 0) {
      gfxPos(34, 25);
      graphics.printf(PSTR("%u.%02us"), (unsigned)(ms / 1000),
                      (unsigned)((ms % 1000) / 10));
    }

    gfxPrint(1, 35, PSTR("Rec:"));
    gfxStartCursor();
    gfxPrint(rec_mode ? PSTR("Repl") : PSTR("Mix"));
    gfxEndCursor(cursor == REC_MODE);

    gfxPrint(1, 45, PSTR("Dub:"));
    gfxStartCursor();
    graphics.printf(PSTR("%3d%%"), dub_level);
    gfxEndCursor(cursor == DUB_LVL);
    gfxStartCursor();
    gfxPrint(dub_cv);
    gfxEndCursor(cursor == DUB_CV, false, dub_cv.InputName());

    gfxPrint(1, 55, PSTR("Fbk:"));
    gfxStartCursor();
    graphics.printf(PSTR("%3d%%"), feedback);
    gfxEndCursor(cursor == FBK_LVL);
    gfxStartCursor();
    gfxPrint(fdb_cv);
    gfxEndCursor(cursor == FBK_CV, false, fdb_cv.InputName());
  } else {
    gfxPrint(1, 25, PSTR("Scr:"));
    gfxStartCursor();
    switch (scrub_mode) {
      case AudioLooper::MODE_SCRUB: gfxPrint(PSTR("Scrub")); break;
      case AudioLooper::MODE_REVERSE: gfxPrint(PSTR("Rev")); break;
      case AudioLooper::MODE_HALF: gfxPrint(PSTR("Half")); break;
      case AudioLooper::MODE_DOUBLE: gfxPrint(PSTR("Dbl")); break;
      default: gfxPrint(PSTR("Loop")); break;
    }
    gfxEndCursor(cursor == SCRUB_MODE);

    gfxPrint(1, 35, PSTR("Spos:"));
    gfxStartCursor();
    gfxPrint(scrub_cv);
    gfxEndCursor(cursor == SCRUB_CV, false, scrub_cv.InputName());

    gfxPrint(1, 45, PSTR("Rng:"));
    gfxStartCursor();
    graphics.printf(PSTR("%3d%%"), scrub_range);
    gfxEndCursor(cursor == SCRUB_RANGE);

    gfxPrint(1, 55, PSTR("1S:"));
    gfxStartCursor();
    gfxPrintIcon(one_shot ? CHECK_ON_ICON : CHECK_OFF_ICON);
    gfxEndCursor(cursor == ONESHOT);
    gfxPos(34, 55);
    gfxPrint(PSTR("Q:"));
    gfxStartCursor();
    gfxPrintIcon(quant_arm ? CHECK_ON_ICON : CHECK_OFF_ICON);
    gfxEndCursor(cursor == QUANT_ARM);
  }

  gfxDisplayInputMapEditor();
}

FLASHMEM void ClockLoopCore::DrawStateIndicator(int x0) {
  // state + playhead position bar on the "meter" half of the screen
  graphics.setPrintPos(x0 + 2, 12);
  switch (looper.state()) {
    case AudioLooper::LOOPER_RECORDING: graphics.print(PSTR("REC")); break;
    case AudioLooper::LOOPER_OVERDUB: graphics.print(PSTR("OVR")); break;
    case AudioLooper::LOOPER_PLAYING:
      graphics.print(one_shot ? PSTR("1SHT") : PSTR("PLAY"));
      break;
    default: graphics.print(armed ? PSTR("ARM") : PSTR("STOP")); break;
  }

  // playhead position bar (full width = one loop)
  const uint32_t len = looper.loopLenFrames();
  if (len >= 2) {
    graphics.drawFrame(x0, 17, 64, 5);
    const uint32_t ph = looper.playheadQ13() >> 13;
    int px = (int)((ph * 62u) / len);
    if (px > 62) px = 62;
    graphics.drawRect(x0 + 1 + px, 18, 2, 3);
  }
}

FLASHMEM void ClockLoopCore::DrawFullScreen() {
  if (!alloc_ok) return;
  const int x0 = 64 - gfx_offset;

  DrawStateIndicator(x0);

  // wet-output level meter (32k == 28px) + loop length readout
  const int h = Proportion((int)looper.peak(), 32767, 28);
  graphics.drawFrame(x0 + 15, 28, 10, 30);
  if (h > 0) graphics.drawRect(x0 + 16, 57 - h, 8, h);
  const uint32_t ms = looper.loopLenFrames() / 48;
  graphics.setPrintPos(x0 + 32, 48);
  if (ms > 0) {
    graphics.printf(PSTR("%u.%02us"), (unsigned)(ms / 1000),
                    (unsigned)((ms % 1000) / 10));
  } else {
    graphics.print(PSTR("--.--s"));
  }

  graphics.drawLine(x0, 26, 127 - gfx_offset, 26, 3);
  View();
}

FLASHMEM void ClockLoopCore::OnButtonPress() {
  if (CheckEditInputMapPress(
        cursor,
        IndexedInput(DUB_CV, dub_cv),
        IndexedInput(FBK_CV, fdb_cv),
        IndexedInput(MIX_CV, mix_cv),
        IndexedInput(SCRUB_CV, scrub_cv)
      ))
    return;
  CursorToggle();
}

FLASHMEM void ClockLoopCore::OnEncoderMove(int direction) {
  if (!EditMode()) {
    MoveCursor(cursor, direction, CURSOR_LENGTH - 1);
    return;
  }
  if (EditSelectedInputMap(direction)) return;

  switch (cursor) {
    case T_MODE:
      transport_sel = (uint8_t)constrain((int)transport_sel + direction,
                                         TREQ_STOP, TREQ_PLAY);
      pending_transport = transport_sel;
      break;
    case CLK_DIV: {
      const uint8_t nd = (uint8_t)constrain((int)clock_div + direction, 0, 4);
      if (nd != clock_div) {
        clock_div = nd;
        pending_clkdiv = true; // clean restart at the next clock edge
      }
      break;
    }
    case REC_MODE: rec_mode = constrain(rec_mode + direction, 0, 1); break;
    case DUB_LVL:
      dub_level = (uint8_t)constrain((int)dub_level + direction, 0, 100);
      break;
    case DUB_CV: dub_cv.ChangeSource(direction); break;
    case FBK_LVL:
      feedback = (uint8_t)constrain((int)feedback + direction, 0, 100);
      break;
    case FBK_CV: fdb_cv.ChangeSource(direction); break;

    case MIX_LVL:
      mix = (uint8_t)constrain((int)mix + direction, 0, 100);
      break;
    case MIX_CV: mix_cv.ChangeSource(direction); break;
    case SCRUB_MODE:
      scrub_mode = (uint8_t)constrain((int)scrub_mode + direction,
                                      0, AudioLooper::MODE_COUNT - 1);
      break;
    case SCRUB_CV: scrub_cv.ChangeSource(direction); break;
    case SCRUB_RANGE:
      scrub_range = (uint8_t)constrain((int)scrub_range + direction, 25, 200);
      break;
    case ONESHOT: one_shot = !one_shot; break;
    case QUANT_ARM: quant_arm = !quant_arm; break;

    default: return;
  }
}

// 256-bit state layout (4 x 64-bit words):
//   word 0 (38b): scrub_mode(3) clock_div(3) rec_mode(1) dub_level(7)
//                 feedback(7) mix(7) scrub_range(8) one_shot(1) quant_arm(1)
//   word 1 (64b): scrub_cv, mix_cv, dub_cv, fdb_cv (4 x 16)
//   word 2-3: unused (0).  Transport/record state is NOT saved (a loaded
//   preset always starts Stopped).
FLASHMEM void ClockLoopCore::OnDataRequest(std::array<uint64_t, CONFIG_SIZE>& data) {
  data[0] = PackPackables(
    pack<3>(scrub_mode),
    pack<3>(clock_div),
    pack<1>(rec_mode),
    pack<7>(dub_level),
    pack<7>(feedback),
    pack<7>(mix),
    pack<8>(scrub_range),
    pack<1>(one_shot),
    pack<1>(quant_arm)
  );
  data[1] = PackPackables(scrub_cv, mix_cv, dub_cv, fdb_cv);
  data[2] = 0;
  data[3] = 0;
}

FLASHMEM void ClockLoopCore::OnDataReceive(
  const std::array<uint64_t, CONFIG_SIZE>& data
) {
  UnpackPackables(
    data[0],
    pack<3>(scrub_mode),
    pack<3>(clock_div),
    pack<1>(rec_mode),
    pack<7>(dub_level),
    pack<7>(feedback),
    pack<7>(mix),
    pack<8>(scrub_range),
    pack<1>(one_shot),
    pack<1>(quant_arm)
  );
  UnpackPackables(data[1], scrub_cv, mix_cv, dub_cv, fdb_cv);

  // re-derive packed ranges defensively
  scrub_mode = constrain(scrub_mode, 0, AudioLooper::MODE_COUNT - 1);
  clock_div = constrain(clock_div, 0, 4);
  rec_mode = constrain(rec_mode, 0, 1);
  dub_level = constrain(dub_level, 0, 100);
  feedback = constrain(feedback, 0, 100);
  mix = constrain(mix, 0, 100);
  scrub_range = constrain(scrub_range, 25, 200);
}

FLASHMEM void ClockLoopCore::SetHelp() {
  help[HELP_DIGITAL1] = PSTR("ArmRec");
  help[HELP_DIGITAL2] = PSTR("Retrig");
  help[HELP_CV1]      = PSTR("Scrub");
  help[HELP_CV2]      = PSTR("Mix");
  help[HELP_OUT1]     = PSTR("Out L");
  help[HELP_OUT2]     = PSTR("Out R");
  help[HELP_EXTRA1]   = PSTR("Clockable looper");
  help[HELP_EXTRA2]   = PSTR("scrub + o.dub");
}
