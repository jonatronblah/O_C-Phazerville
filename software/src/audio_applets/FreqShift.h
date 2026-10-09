#pragma once

#include "../HemisphereAudioApplet.h"
#include "../dsputils.h"
#include "../Audio/AudioMixer.h"
#include "../Audio/AudioPassthrough.h"
#include <Audio.h>

// FreqShift - Bode-style single-sideband frequency shifter inside a
// feedback ring (Echobode-in-a-module).
//
// Signal chain (all DSP lives in the custom FSRingShifter AudioStream;
// the graph objects are stock glue for dry/wet mixing):
//
//            ┌──────────────────────────────────────────────┐
//            │              FSRingShifter (per ch)          │
//   in L/R ──┤  u = x + gain*fb(128 ago) ─ Hilbert I/Q ─ SSB ─ y ─┤─ wet
//            │   fb ← soft-limit(y) into a 128-sample ring  │
//   in L/R ────────────────────────────────── (dry leg) ─────┴─ equal-power mix ─ out
//
// Quadrature network: two-path FIR Hilbert approximation.  The Q path is
// a 255-tap antisymmetric (odd-symmetric) windowed-ideal Hilbert FIR,
// h[m] = 2/(pi*m) * BlackmanHarris, odd m only (64 non-zero coefficients);
// the I path is the matched 127-sample pure delay.  Linear phase means the
// 90-degree relation is EXACT at every frequency; only the amplitude match
// degrades toward DC (windowed-sinc rolloff), which is the honest limit:
// image rejection is >45 dB above ~500 Hz, ~20 dB at 300 Hz, and fades
// below that.  Coefficients are baked at construction with the trig-free
// Taylor sine (libm sin/cos pull ~4 KB of kernel reduction into the ITCM
// bank; see effect_width.h).
//
// The shifter sits inside a feedback ring: the output is soft-limited
// (cubic knee at digital full scale) and re-injected 128 samples (~2.7 ms)
// later, so every round trip is shifted again.  Ring gain runs 0..150%:
// under 100% it is a shimmering resonator, at/above 100% it is a runaway
// self-oscillator - the UI shows a RUNAWAY! flag when ring gain >= 1.
//
// Carrier: phase-accumulator sine/cosine at the shift rate (signed: the
// sign of the shift selects the up/down sideband), with an optional LFO
// wobble of the carrier rate (sine / triangle / sample & hold).
//
// MEMORY LAYOUT NOTE (same as MidSider/ClockLoop): GCC drops section attrs
// on template/inline members, so ALL methods live out-of-line on the
// non-template classes below with FLASHMEM (strong symbols).  Nothing in
// this file allocates; the 4 KB of filter/feedback state are plain members
// of the applet instance (heap/PSRAM pool via the registry factory).

// Name string forced into flash (.progmem) to protect the DTCM/.rodata
// budget; memory-mapped flash reads are fine for display + strhash().
static const char FREQSHIFT_NAME[] PROGMEM = "FreqShft"; // <= 9 chars

// ---- trig-free helpers (distinct names; duplicated per applet header) ----

// sin(x) for |x| <= pi/2, 9th-order Taylor (max error ~1e-6)
FLASHMEM static float FSSinPoly(float x) {
  const float x2 = x * x;
  return x * (1.0f + x2 * (-1.0f / 6.0f
    + x2 * (1.0f / 120.0f
    + x2 * (-1.0f / 5040.0f
    + x2 * (1.0f / 362880.0f)))));
}

// sin(x) for any x in [-2pi, 2pi] via quadrant folding (no libm)
FLASHMEM static float FSSineApprox(float x) {
  const float kPI = 3.14159265f;
  const float kTWO_PI = 6.28318531f;
  while (x > kPI) x -= kTWO_PI;
  while (x < -kPI) x += kTWO_PI;
  const bool neg = x < 0.0f;
  if (neg) x = -x;
  if (x > 1.57079633f) x = kPI - x;
  return neg ? -FSSinPoly(x) : FSSinPoly(x);
}

// Cubic soft saturation at digital full scale: unity below half scale,
// soft knee to 1.0*FS, hard bound 1.0*FS (keeps the runaway ring finite)
FLASHMEM static float FSSoftLimit(float x) {
  const float kFS = 32767.0f;
  float v = x * (1.0f / kFS);
  if (v > 1.0f) v = 1.0f;
  else if (v < -1.0f) v = -1.0f;
  else v = 1.5f * v - 0.5f * v * v * v;
  return v * kFS;
}

// ---- the shifter AudioStream ---------------------------------------------

class FSRingShifter : public AudioStream {
public:
  FSRingShifter();

  // Control-context setters (atomic float/byte stores, read by update())
  void shiftHz(float hz); // signed shift target (smoothed per sample)
  void ringGain(float gain); // 0..1.5
  void lfoSet(float rate_hz, float depth, uint8_t shape); // depth 0..1

private:
  static constexpr int FS_HILB_MID = 127; // I-path delay / FIR center
  static constexpr int FS_HILB_C = 64; // non-zero coefs (odd m 1..127)
  static constexpr uint8_t FS_RING_LEN = 128; // feedback loop delay (samples)
  static constexpr float FS_LFO_HZ = 250.0f; // carrier wobble at depth 1.0

  audio_block_t* inputQueueArray[2];

  float fir_[FS_HILB_C]; // Q-path coefficients (baked in the ctor)
  float hbuf_[2][256]; // quadrature delay lines (power-of-two ring)
  float fbuf_[2][256]; // feedback ring (delay FS_RING_LEN)
  uint8_t hwr_[2] = {0, 0};
  uint8_t fwr_[2] = {0, 0};

  float shift_tgt_ = 0.0f, shift_cur_ = 0.0f;
  float ring_gain_ = 0.0f;
  float phi_ = 0.0f; // carrier phase, [0, 2pi)
  float lfo_inc_ = 0.0f, lfo_phase_ = 0.0f, lfo_depth_ = 0.0f;
  uint8_t lfo_shape_ = 0;
  float lfo_hold_ = 0.0f;
  uint32_t lfo_rng_ = 0x2545F491u;

  float lfoNext();
  virtual void update(void);
};

// -- Out-of-line definitions (strong symbols, FLASHMEM honored) --------------

FLASHMEM FSRingShifter::FSRingShifter() : AudioStream(2, inputQueueArray) {
  // Bake the windowed-ideal Hilbert coefficients:
  //   c_m = 2/(pi*m) * BH(127 + m), m = 1, 3, ..., 127
  // with cos() evaluated by the trig-free Taylor sine.  Blackman-Harris
  // over 254 taps gives exact unity gain from ~800 Hz up (verified offline
  // against the analytic response) - see the honest-limits note above.
  constexpr float kTwoPiOverN = 6.28318531f / 254.0f;
  for (int m = 1, i = 0; m <= FS_HILB_MID; m += 2, i++) {
    const float k = static_cast<float>(FS_HILB_MID + m);
    const float w = 0.35875f
      - 0.48829f * FSSineApprox(k * kTwoPiOverN + 1.57079633f)
      + 0.14128f * FSSineApprox(2.0f * k * kTwoPiOverN + 1.57079633f)
      - 0.01168f * FSSineApprox(3.0f * k * kTwoPiOverN + 1.57079633f);
    fir_[i] = (2.0f / (3.14159265f * m)) * w;
  }
  memset(hbuf_, 0, sizeof(hbuf_));
  memset(fbuf_, 0, sizeof(fbuf_));
}

FLASHMEM void FSRingShifter::shiftHz(float hz) {
  shift_tgt_ = hz;
}

FLASHMEM void FSRingShifter::ringGain(float gain) {
  ring_gain_ = gain;
}

FLASHMEM void FSRingShifter::lfoSet(float rate_hz, float depth, uint8_t shape) {
  lfo_inc_ = rate_hz / AUDIO_SAMPLE_RATE_EXACT;
  lfo_depth_ = depth;
  lfo_shape_ = shape;
}

FLASHMEM float FSRingShifter::lfoNext() {
  lfo_phase_ += lfo_inc_;
  if (lfo_phase_ >= 1.0f) {
    lfo_phase_ -= 1.0f;
    if (lfo_shape_ >= 2) {
      uint32_t r = lfo_rng_;
      r ^= r << 13;
      r ^= r >> 17;
      r ^= r << 5;
      lfo_rng_ = r;
      lfo_hold_ = static_cast<float>(r & 0xFFFF) * (2.0f / 65535.0f) - 1.0f;
    }
  }
  switch (lfo_shape_) {
    case 1: // triangle
      return lfo_phase_ < 0.5f ? (4.0f * lfo_phase_ - 1.0f)
                               : (3.0f - 4.0f * lfo_phase_);
    case 2: // sample & hold
      return lfo_hold_;
    default: // sine
      return FSSineApprox(lfo_phase_ * 6.28318531f);
  }
}

FLASHMEM void FSRingShifter::update(void) {
  audio_block_t* inl = receiveReadOnly(0);
  audio_block_t* inr = receiveReadOnly(1);
  audio_block_t* outl = allocate();
  audio_block_t* outr = allocate();
  if (!outl || !outr) {
    if (outl) release(outl);
    if (outr) release(outr);
    if (inl) release(inl);
    if (inr) release(inr);
    return;
  }

  for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
    // shared carrier + wobble LFO (one advance per sample)
    ONE_POLE(shift_cur_, shift_tgt_, 0.002f);
    const float wobble = lfo_depth_ * FS_LFO_HZ * lfoNext();
    const float inc = (shift_cur_ + wobble) * (6.28318531f / AUDIO_SAMPLE_RATE_EXACT);
    float phi = phi_ + inc;
    if (phi > 6.28318531f) phi -= 6.28318531f;
    else if (phi < 0.0f) phi += 6.28318531f;
    phi_ = phi;
    const float cs = FSSineApprox(phi + 1.57079633f);
    const float sn = FSSineApprox(phi);

    const float dryl = inl ? static_cast<float>(inl->data[n]) : 0.0f;
    const float dryr = inr ? static_cast<float>(inr->data[n]) : 0.0f;

    for (int ch = 0; ch < 2; ch++) {
      const float dry = ch ? dryr : dryl;
      const float fbv = fbuf_[ch][static_cast<uint8_t>(fwr_[ch] - FS_RING_LEN)];
      const float u = dry + ring_gain_ * fbv;
      hbuf_[ch][hwr_[ch]] = u;
      const uint8_t base = static_cast<uint8_t>(hwr_[ch] - FS_HILB_MID);
      const float I = hbuf_[ch][base];
      float Q = 0.0f;
      for (int m = 1, i = 0; m <= FS_HILB_MID; m += 2, i++) {
        Q += fir_[i] * (hbuf_[ch][static_cast<uint8_t>(base - m)]
                      - hbuf_[ch][static_cast<uint8_t>(base + m)]);
      }
      hwr_[ch]++;
      // sum-sideband modulation; a negative shift frequency selects the
      // difference sideband instead (sin(-x) = -sin(x))
      const float y = I * cs - Q * sn;
      fbuf_[ch][fwr_[ch]] = FSSoftLimit(y);
      fwr_[ch]++;
      (ch ? outr : outl)->data[n] = Clip16(y);
    }
  }

  if (inl) release(inl);
  if (inr) release(inr);
  transmit(outl, 0);
  release(outl);
  transmit(outr, 1);
  release(outr);
}

// ---- applet core ----------------------------------------------------------

class FreqShiftCore : public HemisphereAudioApplet {
public:
  // (applet_name is macro-rewritten by _config.h into a static accessor)
  const char* applet_name() {
    return FREQSHIFT_NAME;
  }

  void Start() final;
  void Unload() final;
  void Controller() final;
  void View() final;
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
    // Page 0: shift
    SHIFT = 0, SHIFT_CV, FINE, DIR,
    // Page 1: feedback ring / mix
    FBK, FBK_CV, MIX, MIX_CV,
    // Page 2: carrier LFO
    LFO_RATE, LFO_DEPTH, LFO_DEPTH_CV, LFO_SHAPE,
    CURSOR_LENGTH,
  };

  static float RateFromUnits(int units); // 0..63 -> 0.01..20 Hz, log

  int8_t cursor = SHIFT;

  // ---- params (state round-trip via OnDataRequest/Receive) ----------------
  int8_t shift_c = 20; // coarse, units of 10 Hz, -100..100 -> +/-1 kHz
  int8_t shift_f = 0; // fine, units of 0.1 Hz, -99..99
  uint8_t fbk = 40; // ring gain 0..150%
  uint8_t dir = 0; // 0 = up, 1 = down (inverts the shift)
  uint8_t lfo_rate = 20; // 0..63, log 0.01..20 Hz
  uint8_t lfo_depth = 0; // 0..100%
  uint8_t lfo_shape = 0; // 0 sine / 1 tri / 2 S&H
  uint8_t mix = 50; // 0..100% dry..wet

  CVInputMap shift_cv;
  CVInputMap fbk_cv;
  CVInputMap mix_cv;
  CVInputMap lfod_cv;

  bool runaway = false;

  AudioPassthrough<2> input;
  FSRingShifter shifter;
  AudioMixer<2> mix_l;
  AudioMixer<2> mix_r;
  AudioPassthrough<2> output;
};

// -- Out-of-line definitions (strong symbols, FLASHMEM honored) --------------

FLASHMEM float FreqShiftCore::RateFromUnits(int units) {
  return 0.01f * fastpow2(units * (10.96578f / 63.0f)); // 0.01..20 Hz
}

FLASHMEM void FreqShiftCore::Start() {
  PatchCable(input, 0, shifter, 0);
  PatchCable(input, 1, shifter, 1);
  PatchCable(input, 0, mix_l, 0); // dry leg
  PatchCable(input, 1, mix_r, 0);
  PatchCable(shifter, 0, mix_l, 1); // wet leg
  PatchCable(shifter, 1, mix_r, 1);
  PatchCable(mix_l, 0, output, 0);
  PatchCable(mix_r, 0, output, 1);
  // 8 cables total (<= 32)
}

FLASHMEM void FreqShiftCore::Unload() {
  AllowRestart();
}

// 16.6 kHz control tick: CV + parameter pushes (all atomic stores into the
// stream; the stream smooths per sample where it matters).
FLASHMEM void FreqShiftCore::Controller() {
  // effective shift in Hz (bipolar, direction flips the sideband)
  const float eff_tenths = constrain(
    100.0f * shift_c + shift_f + shift_cv.InF() * 10000.0f, -10000.0f, 10000.0f
  );
  const float eff_hz = (dir ? -0.1f : 0.1f) * eff_tenths;
  shifter.shiftHz(eff_hz);

  const float eff_fbk = constrain(fbk + fbk_cv.InF() * 150.0f, 0.0f, 150.0f);
  shifter.ringGain(eff_fbk * 0.01f);
  runaway = (eff_fbk >= 100.0f);

  shifter.lfoSet(
    RateFromUnits(lfo_rate),
    constrain(lfo_depth + lfod_cv.InF() * 50.0f, 0.0f, 100.0f) * 0.01f,
    lfo_shape
  );

  // equal-power dry/wet mix (block-rate gains, like ClockLoop)
  float dry_gain, wet_gain;
  EqualPowerFade(
    dry_gain, wet_gain, constrain(0.01f * (mix + mix_cv.InF() * 50.0f), 0.0f, 1.0f)
  );
  mix_l.gain(0, dry_gain);
  mix_l.gain(1, wet_gain);
  mix_r.gain(0, dry_gain);
  mix_r.gain(1, wet_gain);
}

// --- UI --------------------------------------------------------------------

FLASHMEM void FreqShiftCore::View() {
  const int8_t page = (cursor >= FBK) + (cursor >= LFO_RATE);
  gfxRect(page * 15, 10, 14, 3); // page tab

  if (page == 0) { // ---- shift ----
    const int32_t tenths = 100 * shift_c + shift_f;
    gfxPrint(1, 15, PSTR("Fr:"));
    gfxStartCursor();
    graphics.printf(PSTR("%+4d.%01d"), static_cast<int>(tenths / 10),
                    static_cast<int>(tenths < 0 ? -tenths % 10 : tenths % 10));
    gfxEndCursor(cursor == SHIFT);
    gfxStartCursor();
    gfxPrint(shift_cv);
    gfxEndCursor(cursor == SHIFT_CV, false, shift_cv.InputName());

    gfxPrint(1, 25, PSTR("Fn:"));
    gfxStartCursor();
    graphics.printf(PSTR("%+3d.%01d"), static_cast<int>(shift_f / 10),
                    static_cast<int>(shift_f < 0 ? -shift_f % 10 : shift_f % 10));
    gfxEndCursor(cursor == FINE);

    gfxPrint(1, 35, PSTR("Dir:"));
    gfxStartCursor();
    gfxPrint(dir ? PSTR("Down") : PSTR("Up  "));
    gfxEndCursor(cursor == DIR);

    // effective shift (with CV + LFO wobble) and ring state
    gfxPrint(1, 45, PSTR("Wob:"));
    graphics.printf(PSTR("%3d%%"), lfo_depth);
    gfxPrint(34, 45, dir ? PSTR("<<") : PSTR(">>"));
    gfxInvert(54, 45, Proportion(lfo_depth, 100, 20), 7);
  } else if (page == 1) { // ---- ring / mix ----
    gfxPrint(1, 15, PSTR("Fb:"));
    gfxStartCursor();
    graphics.printf(PSTR("%3d%%"), fbk);
    gfxEndCursor(cursor == FBK);
    gfxStartCursor();
    gfxPrint(fbk_cv);
    gfxEndCursor(cursor == FBK_CV, false, fbk_cv.InputName());

    gfxPrint(1, 25, PSTR("Mix:"));
    gfxStartCursor();
    graphics.printf(PSTR("%3d%%"), mix);
    gfxEndCursor(cursor == MIX);
    gfxStartCursor();
    gfxPrint(mix_cv);
    gfxEndCursor(cursor == MIX_CV, false, mix_cv.InputName());

    if (runaway) {
      gfxPrint(1, 35, PSTR("RUNAWAY!"));
      gfxInvert(0, 33, 56, 11);
    } else {
      gfxPrint(1, 35, PSTR("Ring <1: shimmer"));
    }

    // ring gain bar (100% == 30px, overflow shows runaway headroom)
    gfxPrint(1, 50, PSTR("G:"));
    gfxDottedLine(12, 54, 60, 54);
    gfxInvert(12, 51, Proportion(fbk, 150, 48), 7);
  } else { // ---- carrier LFO ----
    gfxPrint(1, 15, PSTR("Shp:"));
    gfxStartCursor();
    switch (lfo_shape) {
      case 1: gfxPrint(PSTR("Tri")); break;
      case 2: gfxPrint(PSTR("S&H")); break;
      default: gfxPrint(PSTR("Sin")); break;
    }
    gfxEndCursor(cursor == LFO_SHAPE);

    gfxPrint(1, 25, PSTR("Rt:"));
    gfxStartCursor();
    graphics.printf(PSTR("%5d"), static_cast<int>(RateFromUnits(lfo_rate) * 100.0f));
    gfxPrint(PSTR("Hz"));
    gfxEndCursor(cursor == LFO_RATE);

    gfxPrint(1, 35, PSTR("Dp:"));
    gfxStartCursor();
    graphics.printf(PSTR("%3d%%"), lfo_depth);
    gfxEndCursor(cursor == LFO_DEPTH);
    gfxStartCursor();
    gfxPrint(lfod_cv);
    gfxEndCursor(cursor == LFO_DEPTH_CV, false, lfod_cv.InputName());
  }

  gfxDisplayInputMapEditor();
}

FLASHMEM void FreqShiftCore::OnButtonPress() {
  if (CheckEditInputMapPress(
        cursor,
        IndexedInput(SHIFT_CV, shift_cv),
        IndexedInput(FBK_CV, fbk_cv),
        IndexedInput(MIX_CV, mix_cv),
        IndexedInput(LFO_DEPTH_CV, lfod_cv)
      ))
    return;
  CursorToggle();
}

FLASHMEM void FreqShiftCore::OnEncoderMove(int direction) {
  if (!EditMode()) {
    MoveCursor(cursor, direction, CURSOR_LENGTH - 1);
    return;
  }
  if (EditSelectedInputMap(direction)) return;

  switch (cursor) {
    case SHIFT: shift_c = constrain(shift_c + direction * 2, -100, 100); break;
    case SHIFT_CV: shift_cv.ChangeSource(direction); break;
    case FINE: shift_f = constrain(shift_f + direction, -99, 99); break;
    case DIR: dir = constrain(dir + direction, 0, 1); break;

    case FBK: fbk = constrain(fbk + direction, 0, 150); break;
    case FBK_CV: fbk_cv.ChangeSource(direction); break;
    case MIX: mix = constrain(mix + direction, 0, 100); break;
    case MIX_CV: mix_cv.ChangeSource(direction); break;

    case LFO_RATE: lfo_rate = constrain(lfo_rate + direction, 0, 63); break;
    case LFO_DEPTH: lfo_depth = constrain(lfo_depth + direction, 0, 100); break;
    case LFO_DEPTH_CV: lfod_cv.ChangeSource(direction); break;
    case LFO_SHAPE: lfo_shape = constrain(lfo_shape + direction, 0, 2); break;

    default: return;
  }
}

// 256-bit state layout (4 x 64-bit words):
//   word 0 (47b): shift_c(8) shift_f(8) fbk(8) dir(1) lfo_rate(6)
//                 lfo_depth(7) lfo_shape(2) mix(7)
//   word 1 (64b): shift_cv, fbk_cv, mix_cv, lfod_cv (4 x 16)
//   word 2-3: unused (0)
FLASHMEM void FreqShiftCore::OnDataRequest(std::array<uint64_t, CONFIG_SIZE>& data) {
  data[0] = PackPackables(
    pack<8>(shift_c),
    pack<8>(shift_f),
    pack<8>(fbk),
    pack<1>(dir),
    pack<6>(lfo_rate),
    pack<7>(lfo_depth),
    pack<2>(lfo_shape),
    pack<7>(mix)
  );
  data[1] = PackPackables(shift_cv, fbk_cv, mix_cv, lfod_cv);
  data[2] = 0;
  data[3] = 0;
}

FLASHMEM void FreqShiftCore::OnDataReceive(
  const std::array<uint64_t, CONFIG_SIZE>& data
) {
  UnpackPackables(
    data[0],
    pack<8>(shift_c),
    pack<8>(shift_f),
    pack<8>(fbk),
    pack<1>(dir),
    pack<6>(lfo_rate),
    pack<7>(lfo_depth),
    pack<2>(lfo_shape),
    pack<7>(mix)
  );
  UnpackPackables(data[1], shift_cv, fbk_cv, mix_cv, lfod_cv);

  // re-derive packed ranges defensively
  shift_c = constrain(shift_c, -100, 100);
  shift_f = constrain(shift_f, -99, 99);
  fbk = constrain(fbk, 0, 150);
  dir = constrain(dir, 0, 1);
  lfo_rate = constrain(lfo_rate, 0, 63);
  lfo_depth = constrain(lfo_depth, 0, 100);
  lfo_shape = constrain(lfo_shape, 0, 2);
  mix = constrain(mix, 0, 100);
}

FLASHMEM void FreqShiftCore::SetHelp() {
  help[HELP_DIGITAL1] = PSTR(" - ");
  help[HELP_DIGITAL2] = PSTR(" - ");
  help[HELP_CV1]      = PSTR("Shift");
  help[HELP_CV2]      = PSTR("Ring");
  help[HELP_OUT1]     = PSTR("Out L");
  help[HELP_OUT2]     = PSTR("Out R");
  help[HELP_EXTRA1]   = PSTR("Bode shift in ring");
  help[HELP_EXTRA2]   = PSTR("100%+ = runaway");
}

// Thin template wrapper so the registry can use the FreqShift<STEREO> form.
// (All real work lives in the non-template core so FLASHMEM section
// placement survives this toolchain; Channels is intentionally unused.)
template <AudioChannels Channels>
class FreqShift : public FreqShiftCore {};
