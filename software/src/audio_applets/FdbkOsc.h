#pragma once

#include "../HemisphereAudioApplet.h"
#include "../Audio/AudioPassthrough.h"
#include "../dsputils.h"
#include <Audio.h>
#include <smalloc.h>

// FdbkOsc - infinite-feedback delay played as a VCO: the delay time IS the
// pitch control ("infinitely feed back, and then play the delay time as if
// it were a VCO"). The feedback ring is a custom AudioStream (below) with an
// interpolated (4-point Hermite) read, in-loop damping LPF and FREEZE.
//
// Signal chain:
//
//   in L ─┬─ FdbkOscRing ─┬─ out L
//   in R ─┘   (PSRAM      └─ out R
//              delay ring,
//              fb + damp + tanh)
//
//   per sample, per channel:
//     ring  = hermite(buf, write_head - delay)     // fractional read
//     loop  = one_pole_lpf(ring, damping)          // brightness in the loop
//     write = softclip(excite * drive + loop * fb)  // tanh bounds fb > 100%
//     out   = dry * in + wet * loop
//   FREEZE / Scan mode: skip the write and keep reading = the buffer loops
//   at 1/delay, so scanning the delay time repitches the frozen loop.
//
// V/OCT MATH (128 units/semitone, ONE_OCTAVE = 1536):
//   pitch  = tune_semi * 128 + fine + DetentedIn-style(pitch_cv.In())
//   freq   = C3 * 2^(pitch / 1536)          // C3 = 130.81 Hz at pitch 0
//   delay  = 1 / freq seconds, clamped to the 0.2..50 ms read window
//          = 20 Hz..5 kHz of tracking (~8 octaves); delay_samples = 48000/freq
//   The pitch CV gets a small zero deadzone (DetentedIn-style, +/-24 units
//   ~= +/-19 cents) so an unconnected/noisy source reads exactly 0. Delay
//   time moves through the fractional read position - pitch sweeps glide
//   without clicks; "Slw" adds 0..100 ms of exponential glide on top.
//
// MODES:
//   Sing  - self-oscillating drone: input excite muted, feedback floored at
//           100%; a -60 dB dither seed (like BBD thermal noise) kicks the
//           ring into oscillation from silence. Output: mix in vs ring.
//   Proc  - external audio excites the ring; pitch transposes the resulting
//           resonances ("delay screams into pitched drone").
//   Scan  - frozen loop while delay time is scrubbed = spectral glitch.
//   "Frz" freezes the buffer (stops the write head) in any mode; the Trig1
//   map toggles freeze on rising edges.
//
// MEMORY: the ring buffers prefer PSRAM (extmem_calloc + explicit memset,
// since extmem_calloc does not guarantee zero-fill) with a RAM2 malloc
// fallback + a "RAM2" UI note (ClockLoop/CueLoop precedent). 2 x 8192
// floats = 170 ms/channel stereo (>= 100 ms required), power-of-two masked.
//
// MEMORY LAYOUT NOTE (from the MidSider/ClockLoop builds): GCC drops section
// attributes on vague-linkage entities (inline/template members), so ALL
// code lives in non-template classes with out-of-line FLASHMEM definitions
// (update() included). FdbkOsc<Channels> is the thin registered wrapper.
// UI strings use PSTR()/PROGMEM so they land in flash, not DTCM .rodata.

// Name string forced into flash (.progmem) to protect the DTCM budget.
static const char FDBKOSC_NAME[] PROGMEM = "FdbkOsc";

extern "C" uint8_t external_psram_size;

// 2-in / 2-out feedback delay ring (stereo, one buffer per channel).
class FdbkOscRing : public AudioStream {
public:
  FdbkOscRing();

  // Loop-context lifecycle (Start()/Unload()); alloc-free everywhere else.
  bool begin();
  void end();
  bool IsPSRAM() const;

  // Control-context setters (atomic float/bool stores, ISR-read).
  void delaySamples(float d);   // target delay in samples (slewed in update)
  void feedbackGain(float fb);  // 0..1.5
  void dampingCoef(float a);    // one-pole LPF coefficient (0 dark..1 open)
  void driveGain(float g);      // excite level into the ring
  void mixGains(float dry, float wet);
  void slewCoef(float a);       // per-sample glide toward the delay target
  void freeze(bool f);
  void mode(uint8_t m);         // 0 Sing, 1 Proc, 2 Scan

  float peak() const;
  void decayPeak();

private:
  static constexpr int CAP = 8192; // per channel (170 ms @ 48 kHz)
  static constexpr int CAP_MASK = CAP - 1;

  audio_block_t* inputQueueArray[2];
  float* buf_[2] = {nullptr, nullptr};
  bool in_psram_ = false;

  // ISR-owned
  int wi_ = 0;          // write index (channels are lockstep)
  float dcur_ = 480.0f; // slewed delay in samples
  float lp_[2] = {0.0f, 0.0f};
  float dry_s_ = 0.7071f, wet_s_ = 0.7071f;
  float peak_ = 0.0f;
  uint32_t nz_ = 0x1a2b3c4d; // dither seed (Sing mode kick)

  // control-written
  float dtar_ = 480.0f;
  float slew_a_ = 1.0f;
  float damp_a_ = 1.0f;
  float fb_ = 1.0f;
  float drive_ = 0.0f;
  float dry_t_ = 0.7071f, wet_t_ = 0.7071f;
  bool frozen_ = false;
  uint8_t mode_ = 0;

  virtual void update(void);
};

class FdbkOscCore : public HemisphereAudioApplet {
public:
  void Start() final;
  void Unload() final;
  void Controller() final;
  void View() final;
  void DrawFullScreen() final;
  void OnButtonPress() override;
  void OnEncoderMove(int direction) override;
  void OnDataRequest(std::array<uint64_t, CONFIG_SIZE>& data) override;
  void OnDataReceive(const std::array<uint64_t, CONFIG_SIZE>& data) override;
  void SetParams();

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
    // Page 0: pitch / ring
    P_TUNE = 0, PITCH_CV, P_FINE,
    FEEDBACK, FB_CV,
    FREEZE, FREEZE_CV, FREEZE_TRIG,
    MODE_SEL,
    // Page 1: shape / output
    DRIVE, DAMP, DAMP_CV, MIX, MIX_CV, SLEW,
    CURSOR_LENGTH,
  };

  static constexpr int PITCH_DETENT = 24;    // units, ~19 cents deadzone
  static constexpr int FREEZE_GATE = 384;    // units, ~0.25 V opens the gate

  // ---- params (state round-trip via OnDataRequest/Receive) ----------------
  int8_t tune_semi = 0;  // coarse pitch, semitones, [-33, 63]
  int8_t fine = 0;       // fine pitch, 1/128 semitone units, [-64, 63]
  uint8_t feedback = 110; // 0..150%
  uint8_t drive = 50;     // excite 0..200%
  uint8_t damp = 70;      // loop brightness, 0..100
  uint8_t mix = 50;       // ring vs dry, 0..100
  uint8_t slew = 0;       // pitch glide, 0..100 ms
  uint8_t mode_sel = 1;   // 0 Sing, 1 Proc, 2 Scan
  bool freeze = false;

  CVInputMap pitch_cv;
  CVInputMap fb_cv;
  CVInputMap freeze_cv;
  CVInputMap damp_cv;
  CVInputMap mix_cv;
  DigitalInputMap freeze_trig; // rising edge toggles freeze

  int8_t cursor = P_TUNE;
  bool alloc_ok = false;

  float RingFreq() const;

  AudioPassthrough<2> input;
  FdbkOscRing ring;
  AudioPassthrough<2> output;
};

// Thin template wrapper so the registry can use the FdbkOsc<STEREO> form.
// (All real work lives in the non-template core so FLASHMEM section placement
// survives this toolchain; Channels is intentionally unused.)
template <AudioChannels Channels>
class FdbkOsc : public FdbkOscCore {
public:
  const char* applet_name() {
    return FDBKOSC_NAME;
  }
};

// -- Out-of-line definitions (strong symbols, FLASHMEM honored) --------------

FLASHMEM FdbkOscRing::FdbkOscRing() : AudioStream(2, inputQueueArray) {}

FLASHMEM bool FdbkOscRing::begin() {
  end();
  const size_t bytes = (size_t)CAP * sizeof(float);
  if (external_psram_size > 0) {
    buf_[0] = static_cast<float*>(extmem_calloc(CAP, sizeof(float)));
    buf_[1] = static_cast<float*>(extmem_calloc(CAP, sizeof(float)));
    if (buf_[0] && buf_[1]) {
      in_psram_ = true;
    } else {
      if (buf_[0]) extmem_free(buf_[0]);
      if (buf_[1]) extmem_free(buf_[1]);
      buf_[0] = buf_[1] = nullptr;
    }
  }
  if (!buf_[0]) {
    // RAM2 heap fallback (the ring is small, so full length still fits)
    in_psram_ = false;
    buf_[0] = static_cast<float*>(malloc(bytes));
    buf_[1] = static_cast<float*>(malloc(bytes));
    if (!buf_[0] || !buf_[1]) {
      if (buf_[0]) free(buf_[0]);
      if (buf_[1]) free(buf_[1]);
      buf_[0] = buf_[1] = nullptr;
      return false;
    }
  }
  // extmem_calloc does not guarantee zero-fill; the ring must start silent
  memset(buf_[0], 0, bytes);
  memset(buf_[1], 0, bytes);
  wi_ = 0;
  dcur_ = dtar_;
  lp_[0] = lp_[1] = 0.0f;
  peak_ = 0.0f;
  return true;
}

FLASHMEM void FdbkOscRing::end() {
  if (buf_[0]) {
    if (in_psram_) extmem_free(buf_[0]);
    else free(buf_[0]);
    buf_[0] = nullptr;
  }
  if (buf_[1]) {
    if (in_psram_) extmem_free(buf_[1]);
    else free(buf_[1]);
    buf_[1] = nullptr;
  }
  in_psram_ = false;
}

FLASHMEM bool FdbkOscRing::IsPSRAM() const {
  return in_psram_;
}

FLASHMEM void FdbkOscRing::delaySamples(float d) {
  if (d < 8.0f) d = 8.0f;
  if (d > 2400.0f) d = 2400.0f;
  dtar_ = d;
}

FLASHMEM void FdbkOscRing::feedbackGain(float fb) {
  if (fb < 0.0f) fb = 0.0f;
  if (fb > 1.5f) fb = 1.5f;
  fb_ = fb;
}

FLASHMEM void FdbkOscRing::dampingCoef(float a) {
  if (a < 0.0f) a = 0.0f;
  if (a > 1.0f) a = 1.0f;
  damp_a_ = a;
}

FLASHMEM void FdbkOscRing::driveGain(float g) {
  if (g < 0.0f) g = 0.0f;
  if (g > 2.0f) g = 2.0f;
  drive_ = g;
}

FLASHMEM void FdbkOscRing::mixGains(float dry, float wet) {
  dry_t_ = dry;
  wet_t_ = wet;
}

FLASHMEM void FdbkOscRing::slewCoef(float a) {
  if (a < 0.0001f) a = 0.0001f;
  if (a > 1.0f) a = 1.0f;
  slew_a_ = a;
}

FLASHMEM void FdbkOscRing::freeze(bool f) {
  frozen_ = f;
}

FLASHMEM void FdbkOscRing::mode(uint8_t m) {
  mode_ = (m > 2) ? 2 : m;
}

FLASHMEM float FdbkOscRing::peak() const {
  return peak_;
}

FLASHMEM void FdbkOscRing::decayPeak() {
  peak_ *= 0.85f;
}

FLASHMEM void FdbkOscRing::update(void) {
  if (!buf_[0] || !buf_[1]) return;

  audio_block_t* bl = receiveWritable(0);
  audio_block_t* br = receiveWritable(1);
  // The ring must keep circulating even with no input at all (Sing drone /
  // frozen loop), so substitute silent blocks like AudioDelayExt does.
  if (!bl && !br) {
    bl = allocate();
    br = allocate();
    if (!bl || !br) {
      if (bl) release(bl);
      if (br) release(br);
      return;
    }
    memset(bl->data, 0, sizeof(bl->data));
    memset(br->data, 0, sizeof(br->data));
  } else if (!bl) {
    bl = allocate();
    if (!bl) {
      release(br);
      return;
    }
    memset(bl->data, 0, sizeof(bl->data));
  } else if (!br) {
    br = allocate();
    if (!br) {
      release(bl);
      return;
    }
    memset(br->data, 0, sizeof(br->data));
  }

  const bool frz = frozen_ || mode_ == 2; // Scan implies freeze
  const bool sing = (mode_ == 0);
  // Sing floors the feedback at 100% so the ring self-oscillates
  const float fb = (sing && fb_ < 1.0f) ? 1.0f : fb_;
  const float dr = drive_;
  const float da = damp_a_;
  const float sa = slew_a_;
  const float dtar = dtar_;
  float dcur = dcur_;
  float dry = dry_s_, wet = wet_s_;
  float lp0 = lp_[0], lp1 = lp_[1];
  int wi = wi_;
  uint32_t nz = nz_;
  float pk = peak_;
  const float capf = (float)CAP;

  for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
    dcur += (dtar - dcur) * sa;

    const float inL = bl->data[i] * (1.0f / 32768.0f);
    const float inR = br->data[i] * (1.0f / 32768.0f);

    // fractional read position (write head - delay), wrapped into [0, CAP)
    float rp = (float)wi - dcur + capf;
    while (rp >= capf) rp -= capf;
    const int i1 = (int)rp;
    const float t = rp - (float)i1;
    const int m = CAP_MASK;

    const float xm0 = buf_[0][(i1 - 1) & m];
    const float x0 = buf_[0][i1 & m];
    const float x1 = buf_[0][(i1 + 1) & m];
    const float x2 = buf_[0][(i1 + 2) & m];
    const float r0 = InterpHermite(xm0, x0, x1, x2, t);

    const float ym0 = buf_[1][(i1 - 1) & m];
    const float y0 = buf_[1][i1 & m];
    const float y1 = buf_[1][(i1 + 1) & m];
    const float y2 = buf_[1][(i1 + 2) & m];
    const float r1 = InterpHermite(ym0, y0, y1, y2, t);

    // damping LPF in the loop (also colors the output)
    lp0 += (r0 - lp0) * da;
    lp1 += (r1 - lp1) * da;

    if (!frz) {
      // xorshift dither seed: lets Sing mode start from digital silence
      nz ^= nz << 13;
      nz ^= nz >> 17;
      nz ^= nz << 5;
      const float nse = (float)((int32_t)nz) * (1.0f / 2147483648.0f) * 1e-3f;
      float w0 = (sing ? 0.0f : inL * dr) + lp0 * fb + (sing ? nse : 0.0f);
      float w1 = (sing ? 0.0f : inR * dr) + lp1 * fb + (sing ? nse : 0.0f);
      // tanh soft saturation bounds the ring when feedback exceeds 100%
      w0 = 1.0f - 2.0f / (fastexp(2.0f * w0) + 1.0f);
      w1 = 1.0f - 2.0f / (fastexp(2.0f * w1) + 1.0f);
      buf_[0][wi] = w0;
      buf_[1][wi] = w1;
    }
    wi = (wi + 1) & m;

    // equal-power mix targets glide per sample (zipper-free)
    dry += (dry_t_ - dry) * 0.004f;
    wet += (wet_t_ - wet) * 0.004f;

    const float oL = dry * inL + wet * lp0;
    const float oR = dry * inR + wet * lp1;
    bl->data[i] = Clip16(oL * 32768.0f);
    br->data[i] = Clip16(oR * 32768.0f);

    float p = (oL < 0.0f) ? -oL : oL;
    const float pr = (oR < 0.0f) ? -oR : oR;
    if (pr > p) p = pr;
    if (p > pk) pk = p;
  }

  dcur_ = dcur;
  wi_ = wi;
  lp_[0] = lp0;
  lp_[1] = lp1;
  dry_s_ = dry;
  wet_s_ = wet;
  nz_ = nz;
  peak_ = pk;

  transmit(bl, 0);
  release(bl);
  transmit(br, 1);
  release(br);
}

// -- Applet core --------------------------------------------------------------

FLASHMEM float FdbkOscCore::RingFreq() const {
  int cv = pitch_cv.In();
  if (cv > -PITCH_DETENT && cv < PITCH_DETENT) cv = 0; // DetentedIn-style
  const float pitch_u = float(tune_semi) * 128.0f + float(fine) + float(cv);
  return C3 * SemitonesToRatio(pitch_u * (1.0f / 128.0f));
}

FLASHMEM void FdbkOscCore::Start() {
  if (!freeze_trig.enabled()) freeze_trig.SetGateInput(0); // Trig1
  alloc_ok = ring.begin();
  if (!alloc_ok) return;
  PatchCable(input, 0, ring, 0);
  PatchCable(input, 1, ring, 1);
  PatchCable(ring, 0, output, 0);
  PatchCable(ring, 1, output, 1);
  SetParams();
}

FLASHMEM void FdbkOscCore::Unload() {
  ring.end();
  alloc_ok = false;
  AllowRestart();
}

FLASHMEM void FdbkOscCore::SetParams() {
  if (!alloc_ok) return;
  // Controller() recomputes the effective values each tick
}

// 16.6 kHz control tick.
FLASHMEM void FdbkOscCore::Controller() {
  if (!alloc_ok) return;

  if (freeze_trig.Clock()) freeze = !freeze;

  const float freq = constrain(RingFreq(), 20.0f, 5000.0f);
  ring.delaySamples(AUDIO_SAMPLE_RATE_EXACT / freq);

  ring.feedbackGain(constrain(feedback + fb_cv.InF() * 75.0f, 0.0f, 150.0f) * 0.01f);
  ring.freeze(freeze || freeze_cv.In() > FREEZE_GATE);
  ring.mode(mode_sel);
  ring.driveGain(drive * 0.02f); // 0..200% -> 0..2

  const float dmp = constrain(damp + damp_cv.InF() * 50.0f, 0.0f, 100.0f);
  ring.dampingCoef(0.02f + 0.98f * dmp * 0.01f);

  float dry_g, wet_g;
  EqualPowerFade(
    dry_g, wet_g,
    constrain((mix + mix_cv.InF() * 50.0f) * 0.01f, 0.0f, 1.0f)
  );
  ring.mixGains(dry_g, wet_g);

  ring.slewCoef(slew == 0 ? 1.0f : 1.0f - fastexp(-1.0f / (48.0f * float(slew))));

  ring.decayPeak();
}

// --- UI ---------------------------------------------------------------------

FLASHMEM void FdbkOscCore::View() {
  if (!alloc_ok) {
    gfxPrint(2, 15, PSTR("Out of RAM!!"));
    return;
  }

  const int label_x = 1;
  const int page = (cursor > MODE_SEL);
  gfxRect(page * 15, 10, 14, 3); // page tab

  if (page == 0) { // ---- pitch / ring ----
    gfxPrint(label_x, 15, PSTR("P:"));
    gfxStartCursor();
    gfxPrintPitchHz(static_cast<int16_t>(int(tune_semi) * 128 + fine));
    gfxEndCursor(cursor == P_TUNE);
    gfxStartCursor();
    gfxPrint(pitch_cv);
    gfxEndCursor(cursor == PITCH_CV, false, pitch_cv.InputName());

    gfxPrint(label_x, 25, PSTR("Fin:"));
    gfxStartCursor();
    graphics.printf(PSTR("%+3dc"), int(fine) * 100 / 128);
    gfxEndCursor(cursor == P_FINE);

    gfxPrint(label_x, 35, PSTR("Fbk:"));
    gfxStartCursor();
    graphics.printf(PSTR("%3d%%"), feedback);
    gfxEndCursor(cursor == FEEDBACK);
    gfxStartCursor();
    gfxPrint(fb_cv);
    gfxEndCursor(cursor == FB_CV, false, fb_cv.InputName());

    gfxPrint(label_x, 45, PSTR("Frz:"));
    gfxStartCursor();
    gfxPrintIcon(freeze ? CHECK_ON_ICON : CHECK_OFF_ICON);
    gfxEndCursor(cursor == FREEZE);
    gfxStartCursor();
    gfxPrint(freeze_cv);
    gfxEndCursor(cursor == FREEZE_CV, false, freeze_cv.InputName());
    gfxStartCursor();
    gfxPrint(freeze_trig);
    gfxEndCursor(cursor == FREEZE_TRIG, false, freeze_trig.InputName());

    gfxPrint(label_x, 55, PSTR("Mod:"));
    gfxStartCursor();
    switch (mode_sel) {
      case 0: gfxPrint(PSTR("Sing")); break;
      case 2: gfxPrint(PSTR("Scan")); break;
      default: gfxPrint(PSTR("Proc")); break;
    }
    gfxEndCursor(cursor == MODE_SEL);
  } else { // ---- shape / output ----
    gfxPrint(label_x, 15, PSTR("Drv:"));
    gfxStartCursor();
    graphics.printf(PSTR("%3d%%"), drive);
    gfxEndCursor(cursor == DRIVE);

    gfxPrint(label_x, 25, PSTR("Dmp:"));
    gfxStartCursor();
    graphics.printf(PSTR("%3d%%"), damp);
    gfxEndCursor(cursor == DAMP);
    gfxStartCursor();
    gfxPrint(damp_cv);
    gfxEndCursor(cursor == DAMP_CV, false, damp_cv.InputName());

    gfxPrint(label_x, 35, PSTR("Mix:"));
    gfxStartCursor();
    graphics.printf(PSTR("%3d%%"), mix);
    gfxEndCursor(cursor == MIX);
    gfxStartCursor();
    gfxPrint(mix_cv);
    gfxEndCursor(cursor == MIX_CV, false, mix_cv.InputName());

    gfxPrint(label_x, 45, PSTR("Slw:"));
    gfxStartCursor();
    graphics.printf(PSTR("%3dms"), slew);
    gfxEndCursor(cursor == SLEW);

    // ring level meter (output peak, 28 px full scale)
    gfxPrint(label_x, 55, PSTR("Lvl:"));
    const int h = Proportion(static_cast<int>(ring.peak() * 32768.0f), 32767, 28);
    gfxInvert(20, 55, h > 28 ? 28 : h, 7);
    if (!ring.IsPSRAM()) {
      gfxPos(52, 55);
      gfxPrint(PSTR("RAM2"));
    }
  }

  gfxDisplayInputMapEditor();
}

FLASHMEM void FdbkOscCore::DrawFullScreen() {
  if (!alloc_ok) return;
  const int x0 = 64 - gfx_offset;
  graphics.drawLine(x0, 26, 127 - gfx_offset, 26, 3);

  // ring output level bar (32k == 30 px)
  const int h = Proportion(static_cast<int>(ring.peak() * 32768.0f), 32767, 30);
  graphics.drawFrame(x0 + 12, 26, 10, 30);
  if (h > 0) graphics.drawRect(x0 + 13, 55 - h, 8, h > 29 ? 29 : h);

  // delay time + ring frequency readout
  const float freq = constrain(RingFreq(), 20.0f, 5000.0f);
  graphics.setPrintPos(x0 + 28, 36);
  graphics.printf(PSTR("%4dms"), static_cast<int>(1000.0f / freq + 0.5f));
  graphics.setPrintPos(x0 + 28, 48);
  switch (mode_sel) {
    case 0: graphics.print(PSTR("SING")); break;
    case 2: graphics.print(PSTR("SCAN")); break;
    default: graphics.print(PSTR("PROC")); break;
  }
  if (freeze || mode_sel == 2) {
    graphics.setPrintPos(x0 + 48, 48);
    graphics.print(PSTR("FRZ"));
  }

  View();
}

FLASHMEM void FdbkOscCore::OnButtonPress() {
  if (CheckEditInputMapPress(
        cursor,
        IndexedInput(PITCH_CV, pitch_cv),
        IndexedInput(FB_CV, fb_cv),
        IndexedInput(FREEZE_CV, freeze_cv),
        IndexedInput(DAMP_CV, damp_cv),
        IndexedInput(MIX_CV, mix_cv)
      ))
    return;
  CursorToggle();
}

FLASHMEM void FdbkOscCore::OnEncoderMove(int direction) {
  if (!EditMode()) {
    MoveCursor(cursor, direction, CURSOR_LENGTH - 1);
    return;
  }
  if (EditSelectedInputMap(direction)) return;

  switch (cursor) {
    case P_TUNE: tune_semi = constrain(tune_semi + direction, -33, 63); break;
    case PITCH_CV: pitch_cv.ChangeSource(direction); break;
    case P_FINE: fine = constrain(fine + direction, -64, 63); break;

    case FEEDBACK: feedback = constrain(feedback + direction, 0, 150); break;
    case FB_CV: fb_cv.ChangeSource(direction); break;

    case FREEZE: freeze = !freeze; break;
    case FREEZE_CV: freeze_cv.ChangeSource(direction); break;
    case FREEZE_TRIG: freeze_trig.ChangeSource(direction); break;

    case MODE_SEL: mode_sel = constrain(mode_sel + direction, 0, 2); break;

    case DRIVE: drive = constrain(drive + direction, 0, 200); break;
    case DAMP: damp = constrain(damp + direction, 0, 100); break;
    case DAMP_CV: damp_cv.ChangeSource(direction); break;
    case MIX: mix = constrain(mix + direction, 0, 100); break;
    case MIX_CV: mix_cv.ChangeSource(direction); break;
    case SLEW: slew = constrain(slew + direction, 0, 100); break;

    default: return;
  }
  SetParams();
}

// 256-bit state layout (4 x 64-bit words), everything in-bounds:
//   word 0 (55b): tune_semi(8) fine(7) feedback(8) drive(8) damp(7) mix(7)
//                 slew(7) mode_sel(2) freeze(1)
//   word 1 (64b): pitch_cv, fb_cv, freeze_cv, damp_cv (4 x 16)
//   word 2 (48b): mix_cv (16), freeze_trig (32)
//   word 3: unused (0)
// 167 bits used, 89 spare. fine is signed 7-bit [-64, 63], tune_semi signed
// 8-bit. Ring/transport state is NOT saved (a loaded preset starts unfrozen).
FLASHMEM void FdbkOscCore::OnDataRequest(std::array<uint64_t, CONFIG_SIZE>& data) {
  data[0] = PackPackables(
    tune_semi,
    pack<7>(fine),
    pack<8>(feedback),
    pack<8>(drive),
    pack<7>(damp),
    pack<7>(mix),
    pack<7>(slew),
    pack<2>(mode_sel),
    pack<1>(freeze)
  );
  data[1] = PackPackables(pitch_cv, fb_cv, freeze_cv, damp_cv);
  data[2] = PackPackables(mix_cv, freeze_trig);
  data[3] = 0;
}

FLASHMEM void FdbkOscCore::OnDataReceive(
  const std::array<uint64_t, CONFIG_SIZE>& data
) {
  UnpackPackables(
    data[0],
    tune_semi,
    pack<7>(fine),
    pack<8>(feedback),
    pack<8>(drive),
    pack<7>(damp),
    pack<7>(mix),
    pack<7>(slew),
    pack<2>(mode_sel),
    pack<1>(freeze)
  );
  UnpackPackables(data[1], pitch_cv, fb_cv, freeze_cv, damp_cv);
  UnpackPackables(data[2], mix_cv, freeze_trig);

  // re-derive packed ranges defensively
  tune_semi = constrain(tune_semi, -33, 63);
  fine = constrain(fine, -64, 63);
  feedback = constrain(feedback, 0, 150);
  drive = constrain(drive, 0, 200);
  damp = constrain(damp, 0, 100);
  mix = constrain(mix, 0, 100);
  slew = constrain(slew, 0, 100);
  mode_sel = constrain(mode_sel, 0, 2);

  SetParams();
}

FLASHMEM void FdbkOscCore::SetHelp() {
  help[HELP_DIGITAL1] = PSTR("FrzTog");
  help[HELP_DIGITAL2] = PSTR(" - ");
  help[HELP_CV1]      = PSTR("V/Oct");
  help[HELP_CV2]      = PSTR("Mod");
  help[HELP_OUT1]     = PSTR("Out L");
  help[HELP_OUT2]     = PSTR("Out R");
  help[HELP_EXTRA1] = PSTR("delay-ring osc");
  help[HELP_EXTRA2] = PSTR("Sing/Proc/Scan");
}
