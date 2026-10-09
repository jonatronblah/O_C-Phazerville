#pragma once

#include "../HemisphereAudioApplet.h"
#include "../dsputils.h"
#include "../Audio/AudioMixer.h"
#include "../Audio/AudioPassthrough.h"
#include <Audio.h>
#include <smalloc.h>

// CombChorus - CV comb filter + deep-CV stereo ensemble chorus.
//
// Signal chain (custom CCCombChorusStream AudioStream does everything;
// stock graph objects only mix dry/wet):
//
//   in L/R ─┬─ CCCombChorusStream ─────────────┐
//           │   comb (0.1-20 ms):              │
//           │     FB: y = x + f*y[n-D]         ├─ wet ─ equal-power mix ─ out
//           │     FF: y = x - f*x[n-D]         │
//           │   3 modulated chorus taps of y   │
//           │   (5-40 ms + 3 ms LFO swing)     │
//   in L/R ────────────────────── (dry leg) ───┘
//
// Comb: interpolated (4-point Hermite) delay line with delay time from the
// Freq parameter (log 50 Hz..10 kHz -> 20..0.1 ms) under CV, either linear
// (+/-500 Hz) or V/Oct (x2^+/-5 octaves, pitched combs a la FdbkOsc).
// Feedback is -95..+95% (negative gives the phase-flipped comb family);
// Feedforward mode is the flanger pole y = x - f*x[n-D].  Separate input
// and output history lines keep both modes exact.
//
// Chorus/ensemble: three taps of the comb output with a complex LFO -
// sine / triangle / sample & hold / smooth noise (interpolated S&H), rate
// 0.01-20 Hz log, depth 0-100% (max +/-3 ms swing).  Taps carry fixed
// 0/120/240 degree phase offsets; Spread decorrelates L/R by up to half a
// cycle so the ensemble widens from mono-correlated to fully decorrelated.
//
// MEMORY: delay buffers are 2 x (1024 + 2048) int16 (12 KB) allocated in
// PSRAM (extmem_calloc + memset) with a RAM2 malloc fallback; the View
// notes "RAM2" when PSRAM is not in use (GlitchApplet/ClockLoop precedent).
//
// MEMORY LAYOUT NOTE (MidSider/ClockLoop): all methods live out-of-line on
// the non-template classes with FLASHMEM so the section attr survives this
// toolchain; UI strings use PSTR(); the name string lives in .progmem.

// Name string forced into flash (.progmem) to protect the DTCM/.rodata
// budget.  "CombChorus" is 10 chars, so the applet displays "CombChrs".
static const char COMBCHORUS_NAME[] PROGMEM = "CombChrs"; // <= 9 chars

extern "C" uint8_t external_psram_size;

// ---- trig-free helpers (distinct names; duplicated per applet header) ----

// sin(x) for |x| <= pi/2, 9th-order Taylor (max error ~1e-6)
FLASHMEM static float CCSinPoly(float x) {
  const float x2 = x * x;
  return x * (1.0f + x2 * (-1.0f / 6.0f
    + x2 * (1.0f / 120.0f
    + x2 * (-1.0f / 5040.0f
    + x2 * (1.0f / 362880.0f)))));
}

// sin(x) for any x in [-2pi, 2pi] via quadrant folding (no libm)
FLASHMEM static float CCSineApprox(float x) {
  const float kPI = 3.14159265f;
  const float kTWO_PI = 6.28318531f;
  while (x > kPI) x -= kTWO_PI;
  while (x < -kPI) x += kTWO_PI;
  const bool neg = x < 0.0f;
  if (neg) x = -x;
  if (x > 1.57079633f) x = kPI - x;
  return neg ? -CCSinPoly(x) : CCSinPoly(x);
}

// xorshift32 -> value in [-1, 1] (ISR-safe, no libm)
FLASHMEM static float CCNextRandom(uint32_t& state) {
  uint32_t r = state;
  r ^= r << 13;
  r ^= r >> 17;
  r ^= r << 5;
  state = r;
  return static_cast<float>(r & 0xFFFF) * (2.0f / 65535.0f) - 1.0f;
}

// ---- per-tap LFO (sine / tri / S&H / smooth noise) ------------------------

struct CCLFO {
  float phase = 0.0f;
  float prev = 0.0f, target = 0.0f;
  uint32_t rng = 1u;

  void reset(float p0, uint32_t seed);
  float advance(float inc, uint8_t shape);
};

FLASHMEM void CCLFO::reset(float p0, uint32_t seed) {
  phase = p0 - static_cast<float>(static_cast<int>(p0));
  if (phase < 0.0f) phase += 1.0f;
  rng = seed ? seed : 1u;
  prev = CCNextRandom(rng);
  target = CCNextRandom(rng);
}

FLASHMEM float CCLFO::advance(float inc, uint8_t shape) {
  phase += inc;
  if (phase >= 1.0f) {
    phase -= 1.0f;
    if (shape >= 2) {
      prev = target;
      target = CCNextRandom(rng);
    }
  }
  switch (shape) {
    case 1: // triangle
      return phase < 0.5f ? (4.0f * phase - 1.0f) : (3.0f - 4.0f * phase);
    case 2: // sample & hold
      return target;
    case 3: { // smooth noise: smoothstep between held randoms
      const float p = phase;
      return prev + (target - prev) * (p * p * (3.0f - 2.0f * p));
    }
    default: // sine
      return CCSineApprox(phase * 6.28318531f);
  }
}

// ---- interpolated int16 ring buffer ---------------------------------------

struct CCDelayLine {
  int16_t* buf = nullptr;
  uint16_t mask = 0;
  uint16_t wr = 0;

  CCDelayLine() = default;
  CCDelayLine(int16_t* b, uint16_t m, uint16_t w) : buf(b), mask(m), wr(w) {}

  void write(float v);
  float readHermite(float samples_back) const;
};

FLASHMEM void CCDelayLine::write(float v) {
  buf[wr] = Clip16(v);
  wr = static_cast<uint16_t>((wr + 1) & mask);
}

FLASHMEM float CCDelayLine::readHermite(float samples_back) const {
  int di = static_cast<int>(samples_back);
  if (di < 1) di = 1;
  const float t = samples_back - static_cast<float>(di);
  const uint16_t base = static_cast<uint16_t>(wr - 1 - di);
  return InterpHermite(
    buf[static_cast<uint16_t>(base + 1) & mask], // u[n-di+1]
    buf[base & mask],                            // u[n-di]
    buf[static_cast<uint16_t>(base - 1) & mask], // u[n-di-1]
    buf[static_cast<uint16_t>(base - 2) & mask], // u[n-di-2]
    t
  );
}

// ---- the comb + chorus AudioStream ----------------------------------------

class CCCombChorusStream : public AudioStream {
public:
  CCCombChorusStream();

  bool begin(); // allocate delay arena (PSRAM, RAM2 fallback)
  void end();
  bool ready() const {
    return arena_ != nullptr;
  }
  bool isPSRAM() const {
    return in_psram_;
  }

  // Control-context setters (atomic stores; update() smooths per sample)
  void setComb(float delay_ms, float feedback, bool feedforward);
  void setTaps(float d1_ms, float d2_ms, float d3_ms);
  void setLfo(float rate_hz, float depth, uint8_t shape, float spread);

private:
  // 2 channels x (comb input line 1024 + wet line 2048) int16 samples
  static constexpr int CC_ARENA_SAMPLES = 2 * (1024 + 2048);
  static constexpr float CC_TAP_JITTER = 144.0f; // +/-3 ms @ 48 kHz
  static constexpr float CC_TAP_GAIN = 0.57735027f; // EQUAL_POWER_EQUAL_MIX[3]

  audio_block_t* inputQueueArray[2];

  int16_t* arena_ = nullptr;
  bool in_psram_ = false;
  CCDelayLine xline_[2]; // comb input history (FF pole)
  CCDelayLine wline_[2]; // comb output history (FB + chorus taps)
  CCLFO lfo_[2][3];

  float comb_d_tgt_ = 120.0f, comb_d_ = 120.0f; // samples
  float comb_fb_ = 0.0f;
  bool comb_ff_ = false;
  float tap_d_tgt_[3] = {240.0f, 480.0f, 720.0f};
  float tap_d_[3] = {240.0f, 480.0f, 720.0f};
  float lfo_inc_ = 0.0f, lfo_depth_ = 0.0f, lfo_spread_ = 0.0f;
  uint8_t lfo_shape_ = 0;

  virtual void update(void);
};

// -- Out-of-line definitions (strong symbols, FLASHMEM honored) --------------

FLASHMEM CCCombChorusStream::CCCombChorusStream()
  : AudioStream(2, inputQueueArray) {}

FLASHMEM bool CCCombChorusStream::begin() {
  if (arena_) return true;
  if (external_psram_size > 0) {
    arena_ = static_cast<int16_t*>(
      extmem_calloc(CC_ARENA_SAMPLES, sizeof(int16_t))
    );
    in_psram_ = (arena_ != nullptr);
  }
  if (!arena_) {
    arena_ = static_cast<int16_t*>(malloc(CC_ARENA_SAMPLES * sizeof(int16_t)));
    in_psram_ = false;
  }
  if (!arena_) return false;
  // extmem_calloc does NOT guarantee zero-fill on this core
  memset(arena_, 0, CC_ARENA_SAMPLES * sizeof(int16_t));
  for (int ch = 0; ch < 2; ch++) {
    int16_t* base = arena_ + ch * 3072;
    xline_[ch] = {base, 1023, 0};
    wline_[ch] = {base + 1024, 2047, 0};
    for (int k = 0; k < 3; k++) {
      lfo_[ch][k].reset(
        k * (1.0f / 3.0f) + (ch ? lfo_spread_ * 0.5f : 0.0f),
        0x9E3779B9u + static_cast<uint32_t>(ch * 3 + k) * 0x85EBCA6Bu
      );
    }
  }
  return true;
}

FLASHMEM void CCCombChorusStream::end() {
  if (arena_) {
    if (in_psram_) extmem_free(arena_);
    else free(arena_);
  }
  arena_ = nullptr;
  in_psram_ = false;
  for (int ch = 0; ch < 2; ch++) {
    xline_[ch].buf = nullptr;
    wline_[ch].buf = nullptr;
  }
}

FLASHMEM void CCCombChorusStream::setComb(
  float delay_ms, float feedback, bool feedforward
) {
  float d = delay_ms * (AUDIO_SAMPLE_RATE_EXACT * 0.001f);
  if (d < 4.8f) d = 4.8f; // 0.1 ms
  if (d > 960.0f) d = 960.0f; // 20 ms
  comb_d_tgt_ = d;
  comb_fb_ = feedback;
  comb_ff_ = feedforward;
}

FLASHMEM void CCCombChorusStream::setTaps(float d1_ms, float d2_ms, float d3_ms) {
  const float s = AUDIO_SAMPLE_RATE_EXACT * 0.001f;
  tap_d_tgt_[0] = d1_ms * s;
  tap_d_tgt_[1] = d2_ms * s;
  tap_d_tgt_[2] = d3_ms * s;
}

FLASHMEM void CCCombChorusStream::setLfo(
  float rate_hz, float depth, uint8_t shape, float spread
) {
  lfo_inc_ = rate_hz / AUDIO_SAMPLE_RATE_EXACT;
  lfo_depth_ = depth;
  const float ds = spread - lfo_spread_;
  if (shape != lfo_shape_ || ds > 0.004f || ds < -0.004f) {
    // phase offsets live in the LFO state; re-seed when decorrelation changes
    lfo_shape_ = shape;
    lfo_spread_ = spread;
    for (int ch = 0; ch < 2; ch++) {
      for (int k = 0; k < 3; k++) {
        lfo_[ch][k].reset(
          k * (1.0f / 3.0f) + (ch ? spread * 0.5f : 0.0f),
          0x9E3779B9u + static_cast<uint32_t>(ch * 3 + k) * 0x85EBCA6Bu
        );
      }
    }
  }
}

FLASHMEM void CCCombChorusStream::update(void) {
  audio_block_t* inl = receiveReadOnly(0);
  audio_block_t* inr = receiveReadOnly(1);
  if (!arena_) {
    if (inl) release(inl);
    if (inr) release(inr);
    return;
  }
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
    ONE_POLE(comb_d_, comb_d_tgt_, 0.001f);
    for (int k = 0; k < 3; k++) ONE_POLE(tap_d_[k], tap_d_tgt_[k], 0.001f);

    const float dryl = inl ? static_cast<float>(inl->data[n]) : 0.0f;
    const float dryr = inr ? static_cast<float>(inr->data[n]) : 0.0f;

    for (int ch = 0; ch < 2; ch++) {
      CCDelayLine& xl = xline_[ch];
      CCDelayLine& wl = wline_[ch];
      const float x = ch ? dryr : dryl;

      // comb: feedback reads output history, feedforward reads input history
      float y;
      if (comb_ff_) {
        y = x - comb_fb_ * xl.readHermite(comb_d_);
      } else {
        y = x + comb_fb_ * wl.readHermite(comb_d_);
      }
      xl.write(x);
      wl.write(y);

      // 3 modulated chorus taps of the comb output
      float wet = 0.0f;
      for (int k = 0; k < 3; k++) {
        const float l = lfo_[ch][k].advance(lfo_inc_, lfo_shape_);
        float d = tap_d_[k] + lfo_depth_ * CC_TAP_JITTER * l;
        if (d < 2.5f) d = 2.5f;
        else if (d > 2045.0f) d = 2045.0f;
        wet += CC_TAP_GAIN * wl.readHermite(d);
      }
      (ch ? outr : outl)->data[n] = Clip16(wet);
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

class CombChorusCore : public HemisphereAudioApplet {
public:
  // (applet_name is macro-rewritten by _config.h into a static accessor)
  const char* applet_name() {
    return COMBCHORUS_NAME;
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
    // Page 0: comb
    COMB_FREQ = 0, COMB_FREQ_CV, COMB_FB, COMB_FB_CV, COMB_MODE, VOCT,
    // Page 1: chorus taps
    TAP1, TAP1_CV, TAP2, TAP3, SPREAD,
    // Page 2: LFO / mix
    LFO_SHAPE, LFO_RATE, LFO_RATE_CV, LFO_DEPTH, LFO_DEPTH_CV, MIX, MIX_CV,
    CURSOR_LENGTH,
  };

  // 0..2047 units -> 50 Hz..10 kHz, log (7.643856 = log2(200))
  static float FreqFromUnits(float units);
  // 0..127 units -> 5..40 ms
  static float TapFromUnits(int units);
  // 0..63 units -> 0.01..20 Hz, log
  static float RateFromUnits(int units);

  int8_t cursor = COMB_FREQ;

  // ---- params (state round-trip via OnDataRequest/Receive) ----------------
  uint16_t comb_freq = 1024; // 0..2047, log 50 Hz..10 kHz
  int8_t comb_fb = 30; // -95..95%
  uint8_t comb_mode = 0; // 0 = feedback, 1 = feedforward
  uint8_t voct = 0; // CV mode: 0 linear Hz, 1 V/Oct
  uint8_t tap1 = 30; // 0..127 -> 5..40 ms
  uint8_t tap2 = 55;
  uint8_t tap3 = 80;
  uint8_t spread = 50; // 0..100%
  uint8_t lfo_shape = 0; // 0 sine / 1 tri / 2 S&H / 3 smooth noise
  uint8_t lfo_rate = 30; // 0..63, log 0.01..20 Hz
  uint8_t lfo_depth = 40; // 0..100%
  uint8_t mix = 40; // 0..100% dry..wet

  CVInputMap freq_cv;
  CVInputMap rate_cv;
  CVInputMap depth_cv;
  CVInputMap fb_cv;
  CVInputMap tap1_cv;
  CVInputMap mix_cv;

  bool alloc_ok = false;

  AudioPassthrough<2> input;
  CCCombChorusStream chorus;
  AudioMixer<2> mix_l;
  AudioMixer<2> mix_r;
  AudioPassthrough<2> output;
};

// Thin template wrapper so the registry can use the CombChorus<STEREO> form.
// (All real work lives in the non-template core so FLASHMEM section
// placement survives this toolchain; Channels is intentionally unused.)
template <AudioChannels Channels>
class CombChorus : public CombChorusCore {};

// -- Out-of-line definitions (strong symbols, FLASHMEM honored) --------------

FLASHMEM float CombChorusCore::FreqFromUnits(float units) {
  return 50.0f * fastpow2(units * (7.643856f / 2047.0f));
}

FLASHMEM float CombChorusCore::TapFromUnits(int units) {
  return 5.0f + units * (35.0f / 127.0f);
}

FLASHMEM float CombChorusCore::RateFromUnits(int units) {
  return 0.01f * fastpow2(units * (10.96578f / 63.0f)); // 0.01..20 Hz
}

FLASHMEM void CombChorusCore::Start() {
  alloc_ok = chorus.begin();
  if (!alloc_ok) return;

  PatchCable(input, 0, chorus, 0);
  PatchCable(input, 1, chorus, 1);
  PatchCable(input, 0, mix_l, 0); // dry leg
  PatchCable(input, 1, mix_r, 0);
  PatchCable(chorus, 0, mix_l, 1); // wet leg
  PatchCable(chorus, 1, mix_r, 1);
  PatchCable(mix_l, 0, output, 0);
  PatchCable(mix_r, 0, output, 1);
  // 8 cables total (<= 32)
}

FLASHMEM void CombChorusCore::Unload() {
  chorus.end();
  alloc_ok = false;
  AllowRestart();
}

// 16.6 kHz control tick: CV + parameter pushes (atomic stores into the
// stream; it smooths the delay targets per sample).
FLASHMEM void CombChorusCore::Controller() {
  if (!alloc_ok) return;

  // comb frequency -> delay time (linear Hz or V/Oct on the freq parameter)
  float eff_hz = FreqFromUnits(comb_freq);
  const float cvf = freq_cv.InF();
  if (voct) eff_hz *= fastpow2(cvf * 5.0f); // +/-5 octaves
  else eff_hz += cvf * 500.0f; // +/-500 Hz
  if (eff_hz < 50.0f) eff_hz = 50.0f;
  else if (eff_hz > 10000.0f) eff_hz = 10000.0f;
  const float comb_ms = 1000.0f / eff_hz; // 0.1..20 ms

  const float eff_fb = constrain(comb_fb + fb_cv.InF() * 95.0f, -95.0f, 95.0f);
  chorus.setComb(comb_ms, eff_fb * 0.01f, comb_mode != 0);

  chorus.setTaps(
    TapFromUnits(constrain(tap1 + tap1_cv.InF() * 64.0f, 0.0f, 127.0f)),
    TapFromUnits(constrain(tap2 + tap1_cv.InF() * 32.0f, 0.0f, 127.0f)),
    TapFromUnits(constrain(tap3 + tap1_cv.InF() * 32.0f, 0.0f, 127.0f))
  );

  chorus.setLfo(
    RateFromUnits(constrain(lfo_rate + rate_cv.InF() * 32.0f, 0.0f, 63.0f)),
    constrain(lfo_depth + depth_cv.InF() * 50.0f, 0.0f, 100.0f) * 0.01f,
    lfo_shape,
    spread * 0.01f
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

FLASHMEM void CombChorusCore::View() {
  if (!alloc_ok) {
    gfxPrint(2, 15, PSTR("Out of RAM!!"));
    return;
  }

  const int8_t page = (cursor >= TAP1) + (cursor >= LFO_SHAPE);
  gfxRect(page * 15, 10, 14, 3); // page tab

  if (page == 0) { // ---- comb ----
    const float hz = FreqFromUnits(comb_freq);
    gfxPrint(1, 15, PSTR("CF:"));
    gfxStartCursor();
    graphics.printf(PSTR("%5d"), static_cast<int>(hz));
    gfxEndCursor(cursor == COMB_FREQ);
    gfxStartCursor();
    gfxPrint(freq_cv);
    gfxEndCursor(cursor == COMB_FREQ_CV, false, freq_cv.InputName());

    gfxPrint(1, 25, PSTR("Fb:"));
    gfxStartCursor();
    graphics.printf(PSTR("%+4d%%"), comb_fb);
    gfxEndCursor(cursor == COMB_FB);
    gfxStartCursor();
    gfxPrint(fb_cv);
    gfxEndCursor(cursor == COMB_FB_CV, false, fb_cv.InputName());

    gfxPrint(1, 35, PSTR("Mo:"));
    gfxStartCursor();
    gfxPrint(comb_mode ? PSTR("FF") : PSTR("FB"));
    gfxEndCursor(cursor == COMB_MODE);
    gfxPos(34, 35);
    gfxPrint(PSTR("V:"));
    gfxStartCursor();
    gfxPrintIcon(voct ? CHECK_ON_ICON : CHECK_OFF_ICON);
    gfxEndCursor(cursor == VOCT);

    gfxPrint(1, 45, PSTR("Dly:"));
    graphics.printf(PSTR("%2d.%01dms"), static_cast<int>(1000.0f / hz),
                    static_cast<int>(1000.0f / hz * 10.0f) % 10);

    if (!chorus.isPSRAM()) {
      gfxPrint(34, 55, PSTR("RAM2!"));
    }
  } else if (page == 1) { // ---- chorus taps ----
    gfxPrint(1, 15, PSTR("T1:"));
    gfxStartCursor();
    graphics.printf(PSTR("%2d.%01d"), static_cast<int>(TapFromUnits(tap1)),
                    static_cast<int>(TapFromUnits(tap1) * 10.0f) % 10);
    gfxEndCursor(cursor == TAP1);
    gfxStartCursor();
    gfxPrint(tap1_cv);
    gfxEndCursor(cursor == TAP1_CV, false, tap1_cv.InputName());

    gfxPrint(1, 25, PSTR("T2:"));
    gfxStartCursor();
    graphics.printf(PSTR("%2d.%01d"), static_cast<int>(TapFromUnits(tap2)),
                    static_cast<int>(TapFromUnits(tap2) * 10.0f) % 10);
    gfxEndCursor(cursor == TAP2);

    gfxPrint(1, 35, PSTR("T3:"));
    gfxStartCursor();
    graphics.printf(PSTR("%2d.%01d"), static_cast<int>(TapFromUnits(tap3)),
                    static_cast<int>(TapFromUnits(tap3) * 10.0f) % 10);
    gfxEndCursor(cursor == TAP3);

    gfxPrint(1, 45, PSTR("Spd:"));
    gfxStartCursor();
    graphics.printf(PSTR("%3d%%"), spread);
    gfxEndCursor(cursor == SPREAD);
    gfxInvert(12, 55, Proportion(spread, 100, 48), 7);
  } else { // ---- LFO / mix ----
    gfxPrint(1, 15, PSTR("Shp:"));
    gfxStartCursor();
    switch (lfo_shape) {
      case 1: gfxPrint(PSTR("Tri")); break;
      case 2: gfxPrint(PSTR("S&H")); break;
      case 3: gfxPrint(PSTR("Nz ")); break;
      default: gfxPrint(PSTR("Sin")); break;
    }
    gfxEndCursor(cursor == LFO_SHAPE);

    gfxPrint(1, 25, PSTR("Rt:"));
    gfxStartCursor();
    graphics.printf(PSTR("%5d"), static_cast<int>(RateFromUnits(lfo_rate) * 100.0f));
    gfxPrint(PSTR("Hz"));
    gfxEndCursor(cursor == LFO_RATE);
    gfxStartCursor();
    gfxPrint(rate_cv);
    gfxEndCursor(cursor == LFO_RATE_CV, false, rate_cv.InputName());

    gfxPrint(1, 35, PSTR("Dp:"));
    gfxStartCursor();
    graphics.printf(PSTR("%3d%%"), lfo_depth);
    gfxEndCursor(cursor == LFO_DEPTH);
    gfxStartCursor();
    gfxPrint(depth_cv);
    gfxEndCursor(cursor == LFO_DEPTH_CV, false, depth_cv.InputName());

    gfxPrint(1, 45, PSTR("Mix:"));
    gfxStartCursor();
    graphics.printf(PSTR("%3d%%"), mix);
    gfxEndCursor(cursor == MIX);
    gfxStartCursor();
    gfxPrint(mix_cv);
    gfxEndCursor(cursor == MIX_CV, false, mix_cv.InputName());
  }

  gfxDisplayInputMapEditor();
}

FLASHMEM void CombChorusCore::OnButtonPress() {
  if (CheckEditInputMapPress(
        cursor,
        IndexedInput(COMB_FREQ_CV, freq_cv),
        IndexedInput(COMB_FB_CV, fb_cv),
        IndexedInput(TAP1_CV, tap1_cv),
        IndexedInput(LFO_RATE_CV, rate_cv),
        IndexedInput(LFO_DEPTH_CV, depth_cv),
        IndexedInput(MIX_CV, mix_cv)
      ))
    return;
  CursorToggle();
}

FLASHMEM void CombChorusCore::OnEncoderMove(int direction) {
  if (!EditMode()) {
    MoveCursor(cursor, direction, CURSOR_LENGTH - 1);
    return;
  }
  if (EditSelectedInputMap(direction)) return;

  switch (cursor) {
    case COMB_FREQ:
      comb_freq = constrain(comb_freq + direction * 4, 0, 2047);
      break;
    case COMB_FREQ_CV: freq_cv.ChangeSource(direction); break;
    case COMB_FB: comb_fb = constrain(comb_fb + direction, -95, 95); break;
    case COMB_FB_CV: fb_cv.ChangeSource(direction); break;
    case COMB_MODE: comb_mode = constrain(comb_mode + direction, 0, 1); break;
    case VOCT: voct = constrain(voct + direction, 0, 1); break;

    case TAP1: tap1 = constrain(tap1 + direction, 0, 127); break;
    case TAP1_CV: tap1_cv.ChangeSource(direction); break;
    case TAP2: tap2 = constrain(tap2 + direction, 0, 127); break;
    case TAP3: tap3 = constrain(tap3 + direction, 0, 127); break;
    case SPREAD: spread = constrain(spread + direction, 0, 100); break;

    case LFO_SHAPE: lfo_shape = constrain(lfo_shape + direction, 0, 3); break;
    case LFO_RATE: lfo_rate = constrain(lfo_rate + direction, 0, 63); break;
    case LFO_RATE_CV: rate_cv.ChangeSource(direction); break;
    case LFO_DEPTH: lfo_depth = constrain(lfo_depth + direction, 0, 100); break;
    case LFO_DEPTH_CV: depth_cv.ChangeSource(direction); break;
    case MIX: mix = constrain(mix + direction, 0, 100); break;
    case MIX_CV: mix_cv.ChangeSource(direction); break;

    default: return;
  }
}

// 256-bit state layout (4 x 64-bit words):
//   word 0 (58b): comb_freq(11) comb_fb(8) comb_mode(2) voct(1) tap1(7)
//                 tap2(7) tap3(7) lfo_shape(2) lfo_rate(6) lfo_depth(7)
//   word 1 (46b): spread(7) mix(7) freq_cv(16) rate_cv(16)
//   word 2 (64b): depth_cv, fb_cv, tap1_cv, mix_cv (4 x 16)
//   word 3: unused (0)
FLASHMEM void CombChorusCore::OnDataRequest(std::array<uint64_t, CONFIG_SIZE>& data) {
  data[0] = PackPackables(
    pack<11>(comb_freq),
    pack<8>(comb_fb),
    pack<2>(comb_mode),
    pack<1>(voct),
    pack<7>(tap1),
    pack<7>(tap2),
    pack<7>(tap3),
    pack<2>(lfo_shape),
    pack<6>(lfo_rate),
    pack<7>(lfo_depth)
  );
  data[1] = PackPackables(pack<7>(spread), pack<7>(mix), freq_cv, rate_cv);
  data[2] = PackPackables(depth_cv, fb_cv, tap1_cv, mix_cv);
  data[3] = 0;
}

FLASHMEM void CombChorusCore::OnDataReceive(
  const std::array<uint64_t, CONFIG_SIZE>& data
) {
  UnpackPackables(
    data[0],
    pack<11>(comb_freq),
    pack<8>(comb_fb),
    pack<2>(comb_mode),
    pack<1>(voct),
    pack<7>(tap1),
    pack<7>(tap2),
    pack<7>(tap3),
    pack<2>(lfo_shape),
    pack<6>(lfo_rate),
    pack<7>(lfo_depth)
  );
  UnpackPackables(data[1], pack<7>(spread), pack<7>(mix), freq_cv, rate_cv);
  UnpackPackables(data[2], depth_cv, fb_cv, tap1_cv, mix_cv);

  // re-derive packed ranges defensively
  comb_freq = constrain(comb_freq, 0, 2047);
  comb_fb = constrain(comb_fb, -95, 95);
  comb_mode = constrain(comb_mode, 0, 1);
  voct = constrain(voct, 0, 1);
  tap1 = constrain(tap1, 0, 127);
  tap2 = constrain(tap2, 0, 127);
  tap3 = constrain(tap3, 0, 127);
  spread = constrain(spread, 0, 100);
  lfo_shape = constrain(lfo_shape, 0, 3);
  lfo_rate = constrain(lfo_rate, 0, 63);
  lfo_depth = constrain(lfo_depth, 0, 100);
  mix = constrain(mix, 0, 100);
}

FLASHMEM void CombChorusCore::SetHelp() {
  help[HELP_DIGITAL1] = PSTR(" - ");
  help[HELP_DIGITAL2] = PSTR(" - ");
  help[HELP_CV1]      = PSTR("CFreq");
  help[HELP_CV2]      = PSTR("Rate");
  help[HELP_OUT1]     = PSTR("Out L");
  help[HELP_OUT2]     = PSTR("Out R");
  help[HELP_EXTRA1]   = PSTR("CV comb + ensemble");
  help[HELP_EXTRA2]   = PSTR("complex LFO");
}
