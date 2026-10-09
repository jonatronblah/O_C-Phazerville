#pragma once

#include "../Audio/AudioPassthrough.h"
#include "../dsputils.h"
#include <Audio.h>

// SpectraClust - steerable animated resonant filter cluster.
//
// 8 parallel resonant bandpass bands (0 dB peak, RBJ) are fed from the mono
// sum of the stereo input; each band's output is scaled by its morphable
// scene gain and panned across L/R (spectral panner), then all bands are
// summed to the wet L/R outputs and crossfaded against the dry stereo input:
//
//              ┌───────────────┐   band 0 (gain, pan) ─┐
//   L ──┬─ 0.5 ┤               ├─ band 1 (gain, pan) ──┤
//   R ──┴─ 0.5 ┤ AudioSpectraClust  ...                ├─ (dry/wet) ─ L out
//              └───────────────┘   band 7 (gain, pan) ─┘         ─ R out
//
// Band centers are geometric: f_n = base x spread^n, then each center is
// animated: f_n x 2^(depth x sin(2*pi*(phase + n*0.618034))) - a per-band
// phase-offset sweep around the scene center (the "animated cluster").
// Scene A/B are per-band gain patterns (vowel/formant-like presets, or
// user-edited); Morph crossfades between them, with Morph CV 0..1.
//
// CPU strategy: 8 bands x direct-form-I float32 biquads on a single mono
// feed (~15k flops per 128-sample block, well under 2% of the T4.1 core),
// so no band-count compromise was needed.  Coefficients are recomputed ONCE
// PER AUDIO BLOCK (375 Hz) inside update() - not per control tick - which
// keeps the control path cheap and race-free (all filter state lives in the
// audio object; Controller() only writes plain float members).  Animation
// sweeps are therefore smooth up to roughly 30-50 Hz rates; beyond that the
// per-block coefficient refresh becomes audible as stepping - fine for the
// intended slow sweeps.  Gains/pan weights are written per control tick
// (atomic float stores, one-pole smoothed in Controller()).
//
// MEMORY LAYOUT NOTE (from the MidSider build): GCC silently drops
// section/FLASHMEM attrs on vague-linkage entities (inline/template member
// functions), and the slot0.ld script runs all default .text in ITCM (32 KB
// bank granular).  To keep FLASHMEM effective, ALL state + code lives in
// non-template classes below (AudioSpectraClust + SpectraClustCore) with
// out-of-line definitions (strong symbols) - including the audio ISR update()
// (the ClockLoop/effect_looper precedent: flash-resident update() is fine at
// 375 Hz block rate).  SpectraClust<Channels> is the thin registered wrapper.
// UI strings use PSTR() / PROGMEM so they land in flash (.progmem), not the
// DTCM .rodata pool.  No dynamic buffers are used (fixed in-object arrays),
// so there is no PSRAM/RAM2 allocation and no "Out of RAM" path.

// Name string forced into flash (.progmem) to protect the DTCM/.rodata
// budget; memory-mapped flash reads are fine for display + strhash().
static const char SPECTRACLUST_NAME[] PROGMEM = "SpecClust";

// Vowel/formant-like gain presets (per-band, 0..63), loaded into a scene.
static const uint8_t SC_PRESETS[8][8] PROGMEM = {
  //  b0  b1  b2  b3  b4  b5  b6  b7
  {   40, 40, 40, 40, 40, 40, 40, 40 }, // Flat
  {   56, 48, 32, 24, 32, 20, 12,  8 }, // Ahh
  {   24, 32, 48, 56, 40, 28, 20, 12 }, // Eee
  {   56, 44, 28, 16, 12, 10,  8,  6 }, // Ooo
  {   63, 52, 32, 20, 12,  8,  6,  4 }, // Mm
  {    8, 16, 32, 56, 56, 32, 16,  8 }, // Peak
  {    8, 12, 18, 26, 36, 46, 54, 63 }, // Rise
  {   63, 54, 46, 36, 26, 18, 12,  8 }, // Fall
};

static const char SC_PRESET_NAMES[8][5] PROGMEM = {
  "Flat", "Ahh ", "Eee ", "Ooo ", "Mm  ", "Peak", "Rise", "Fall",
};

// Custom AudioStream: 2-in / 2-out resonant band cluster with spectral pan.
// Per-sample math is float32 and allocation-free in update().
class AudioSpectraClust : public AudioStream {
public:
  static const int BANDS = 8;

  AudioSpectraClust(void);
  void reset();

  // Parameter pushes from Controller() - plain float stores (atomic on ARM),
  // no locking needed; a torn parameter *set* is at worst a one-block mix of
  // old/new values.
  void setBase(float hz);
  void setSpread(float ratio);
  void setQ(float q);
  void setSweep(float rate_hz, float depth_oct);
  void setGains(const float* gl, const float* gr);
  void setMix(float dry, float wet);

  virtual void update(void);

private:
  audio_block_t* inputQueueArray[2];

  // cluster parameters (written from Controller(), read per block)
  float base_ = 220.0f;   // Hz, band 0 center
  float spread_ = 1.5f;   // geometric ratio between band centers
  float q_ = 1.0f;        // per-band Q (global)
  float rate_ = 0.5f;     // animation rate, Hz
  float depth_ = 0.0f;    // animation depth, octaves
  float gl_[BANDS] = {0}; // per-band L weight (gain x pan)
  float gr_[BANDS] = {0}; // per-band R weight (gain x pan)
  float dry_ = 1.0f, wet_ = 0.0f;

  // animation phase, turns 0..1
  float phase_ = 0.0f;

  // per-band biquad coefficients (direct form I, a0-normalized)
  float b0_[BANDS] = {0}, b1_[BANDS] = {0}, b2_[BANDS] = {0};
  float a1_[BANDS] = {0}, a2_[BANDS] = {0};
  // per-band filter state
  float x1_[BANDS] = {0}, x2_[BANDS] = {0};
  float y1_[BANDS] = {0}, y2_[BANDS] = {0};
  // current animated band centers (for UI)
  float freq_[BANDS] = {0};

  // Trig-free sine (9th-order Taylor + quadrant folding).  libm's sin/cos
  // pull ~4 KB of __kernel_* reduction code into ITCM - never call them.
  static float Sine(float radians);

  // Advance the LFO phase one block and refresh all band coefficients.
  void updateCoeffs();
};

// Non-template core: all state + out-of-line FLASHMEM definitions live here
// so section placement survives this toolchain.
class SpectraClustCore : public HemisphereAudioApplet {
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
    // Page 0: cluster
    BASE = 0, BASE_CV,
    SPREAD, SPREAD_CV,
    QVAL, Q_CV,
    // Page 1: morph / animation
    MORPH, MORPH_CV,
    RATE, DEPTH, DEPTH_CV, PAN,
    // Page 2: scenes
    SCENE, PRESET,
    GAIN0, GAIN1, GAIN2, GAIN3,
    GAIN4, GAIN5, GAIN6, GAIN7,
    // Page 3: output
    MIX, MIX_CV,
    CURSOR_LENGTH,
  };

  static const int NUM_PRESETS = 8;

  // 0..2047 units -> 20 Hz..8 kHz, log (8.644 octaves)
  static float FreqFromUnits(float units);
  // 0..255 units -> 1.05..3.00
  static float SpreadFromUnits(int units);
  // 0..127 units -> Q 0.5..16
  static float QFromUnits(int units);
  // 0..63 units -> 0.02..20 Hz, log
  static float RateFromUnits(int units);

  void LoadPreset(int preset, int scene);
  void DrawBars(int x0, int y_bottom, int bar_w, int pitch, int max_h);

  int8_t cursor = BASE;

  // ---- params (state round-trip via OnDataRequest/Receive) ----------------
  uint16_t base_freq = 600; // 0..2047 units, log 20 Hz..8 kHz
  uint8_t spread = 96;      // 0..255 -> ratio 1.05..3.00
  uint8_t q_units = 40;     // 0..127 -> Q 0.5..16
  uint8_t morph = 0;        // 0..100% scene A..B
  uint8_t rate = 24;        // 0..63, log 0.02..20 Hz
  uint8_t depth = 0;        // 0..100% -> 0..1 octave sweep
  uint8_t pan = 0;          // 0..100% spectral spread
  uint8_t mix = 100;        // 0..100% dry..wet
  uint8_t scene_a[8] = {40, 40, 40, 40, 40, 40, 40, 40};
  uint8_t scene_b[8] = {16, 24, 32, 40, 40, 32, 24, 16};

  // transient UI state (not persisted)
  uint8_t scene_sel = 0; // 0 = edit scene A, 1 = edit scene B
  uint8_t preset_idx = 0;

  CVInputMap base_cv;
  CVInputMap spread_cv;
  CVInputMap q_cv;
  CVInputMap morph_cv;
  CVInputMap depth_cv;
  CVInputMap mix_cv;

  // smoothed effective values (see Controller())
  float sm_base = 600, sm_spread = 96, sm_q = 40;
  float sm_morph = 0, sm_depth = 0, sm_mix = 100;
  float sm_gain[8] = {0}; // per-band scene-morphed gains 0..1

  AudioPassthrough<2> input;
  AudioSpectraClust bank;
  AudioPassthrough<2> output;
};

// Thin template wrapper so the registry can use the SpectraClust<STEREO>
// form.  (All real work lives in the non-template classes so FLASHMEM
// placement survives this toolchain; Channels is intentionally unused.)
template <AudioChannels Channels>
class SpectraClust : public SpectraClustCore {
public:
  const char* applet_name() {
    return SPECTRACLUST_NAME;
  }
};

// -- Out-of-line definitions (strong symbols, FLASHMEM honored) --------------

FLASHMEM AudioSpectraClust::AudioSpectraClust(void)
  : AudioStream(2, inputQueueArray) {
  reset();
}

FLASHMEM void AudioSpectraClust::reset() {
  phase_ = 0.0f;
  for (int n = 0; n < BANDS; n++) {
    x1_[n] = x2_[n] = y1_[n] = y2_[n] = 0.0f;
    freq_[n] = base_;
  }
  updateCoeffs();
}

FLASHMEM void AudioSpectraClust::setBase(float hz) {
  if (!(hz > 0.0f)) hz = 220.0f;
  base_ = hz;
}

FLASHMEM void AudioSpectraClust::setSpread(float ratio) {
  if (ratio < 1.0f) ratio = 1.0f;
  if (ratio > 4.0f) ratio = 4.0f;
  spread_ = ratio;
}

FLASHMEM void AudioSpectraClust::setQ(float q) {
  if (q < 0.5f) q = 0.5f;
  if (q > 16.0f) q = 16.0f;
  q_ = q;
}

FLASHMEM void AudioSpectraClust::setSweep(float rate_hz, float depth_oct) {
  if (rate_hz < 0.0f) rate_hz = 0.0f;
  if (rate_hz > 50.0f) rate_hz = 50.0f;
  if (depth_oct < 0.0f) depth_oct = 0.0f;
  if (depth_oct > 2.0f) depth_oct = 2.0f;
  rate_ = rate_hz;
  depth_ = depth_oct;
}

FLASHMEM void AudioSpectraClust::setGains(const float* gl, const float* gr) {
  for (int n = 0; n < BANDS; n++) {
    gl_[n] = gl[n];
    gr_[n] = gr[n];
  }
}

FLASHMEM void AudioSpectraClust::setMix(float dry, float wet) {
  dry_ = dry;
  wet_ = wet;
}

FLASHMEM float AudioSpectraClust::Sine(float x) {
  // sin(x) for |x| <= pi/2, 9th-order Taylor (max error ~1e-6)
  // sin(x) for any x in [-2pi, 2pi] via quadrant folding (no range-reduction
  // libm calls; the while loops iterate at most a few times for our inputs)
  const float kPI = 3.14159265f;
  const float kTWO_PI = 6.28318531f;
  while (x > kPI) x -= kTWO_PI;
  while (x < -kPI) x += kTWO_PI;
  const bool neg = x < 0.0f;
  if (neg) x = -x;
  if (x > 1.57079633f) x = kPI - x; // fold [pi/2, pi]
  const float x2 = x * x;
  const float s = x * (1.0f + x2 * (-1.0f / 6.0f
    + x2 * (1.0f / 120.0f
    + x2 * (-1.0f / 5040.0f
    + x2 * (1.0f / 362880.0f)))));
  return neg ? -s : s;
}

FLASHMEM void AudioSpectraClust::updateCoeffs() {
  // Per-band center: base x spread^n x 2^(depth x sin(2pi*phase_n)),
  // with a golden-ratio phase offset per band so the cluster "swirls".
  float mult = 1.0f;
  for (int n = 0; n < BANDS; n++) {
    float t = phase_ + n * 0.61803399f;
    t -= static_cast<int>(t); // wrap to 0..1 turns (t >= 0 here)
    const float lfo = Sine(t * 6.28318531f);
    float f = base_ * mult * fastpow2(depth_ * lfo);
    mult *= spread_;
    if (f < 20.0f) f = 20.0f;
    if (f > 15000.0f) f = 15000.0f;
    freq_[n] = f;

    // RBJ bandpass (0 dB peak), a0-normalized, trig-free:
    //   coef = {b0, b1, b2, a1, a2} for y = b0*x + b1*x1 + b2*x2
    //                                       - a1*y1 - a2*y2
    const float w0 = f * (6.28318531f / AUDIO_SAMPLE_RATE_EXACT);
    const float sn = Sine(w0);
    const float cs = Sine(w0 + 1.57079633f); // cos(x) = sin(x + pi/2)
    const float alpha = sn / (2.0f * q_);
    const float a0 = 1.0f + alpha;
    b0_[n] = alpha / a0;
    b1_[n] = 0.0f;
    b2_[n] = -alpha / a0;
    a1_[n] = -2.0f * cs / a0;
    a2_[n] = (1.0f - alpha) / a0;
  }
}

FLASHMEM void AudioSpectraClust::update(void) {
  audio_block_t* bl = receiveWritable(0);
  audio_block_t* br = receiveWritable(1);
  if (!bl && !br) return;
  if (!bl) {
    bl = allocate();
    if (!bl) {
      if (br) {
        transmit(br, 1);
        release(br);
      }
      return;
    }
    memset(bl->data, 0, sizeof(bl->data));
  }
  if (!br) {
    br = allocate();
    if (!br) {
      transmit(bl, 0);
      release(bl);
      return;
    }
    memset(br->data, 0, sizeof(br->data));
  }

  // Advance the animation phase one block, then refresh coefficients.
  // (Once per 128-sample block = 375 Hz coefficient update rate.)
  phase_ += rate_ * (static_cast<float>(AUDIO_BLOCK_SAMPLES)
                     / AUDIO_SAMPLE_RATE_EXACT);
  if (phase_ >= 1.0f) phase_ -= 1.0f;
  updateCoeffs();

  const float dry = dry_;
  const float wet = wet_;
  for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
    const float in_l = static_cast<float>(bl->data[i]);
    const float in_r = static_cast<float>(br->data[i]);
    const float x = 0.5f * (in_l + in_r); // mono feed
    float wl = 0.0f;
    float wr = 0.0f;
    for (int n = 0; n < BANDS; n++) {
      const float y = b0_[n] * x + b1_[n] * x1_[n] + b2_[n] * x2_[n]
        - a1_[n] * y1_[n] - a2_[n] * y2_[n];
      x2_[n] = x1_[n];
      x1_[n] = x;
      y2_[n] = y1_[n];
      y1_[n] = y;
      wl += y * gl_[n];
      wr += y * gr_[n];
    }
    bl->data[i] = Clip16(dry * in_l + wet * wl);
    br->data[i] = Clip16(dry * in_r + wet * wr);
  }

  transmit(bl, 0);
  transmit(br, 1);
  release(bl);
  release(br);
}

FLASHMEM float SpectraClustCore::FreqFromUnits(float units) {
  // 20 Hz x 2^(units * log2(400) / 2047), 8000 / 20 = 400
  return 20.0f * fastpow2(units * (8.643856f / 2047.0f));
}

FLASHMEM float SpectraClustCore::SpreadFromUnits(int units) {
  return 1.05f + units * (1.95f / 255.0f);
}

FLASHMEM float SpectraClustCore::QFromUnits(int units) {
  return 0.5f + units * (15.5f / 127.0f);
}

FLASHMEM float SpectraClustCore::RateFromUnits(int units) {
  // 0.02 Hz x 2^(units * log2(1000) / 63), 20 / 0.02 = 1000
  return 0.02f * fastpow2(units * (9.965784f / 63.0f));
}

FLASHMEM void SpectraClustCore::LoadPreset(int preset, int scene) {
  if (preset < 0 || preset >= NUM_PRESETS) return;
  uint8_t* g = scene ? scene_b : scene_a;
  for (int n = 0; n < 8; n++) g[n] = pgm_read_byte(&SC_PRESETS[preset][n]);
}

FLASHMEM void SpectraClustCore::Start() {
  PatchCable(input, 0, bank, 0);
  PatchCable(input, 1, bank, 1);
  PatchCable(bank, 0, output, 0);
  PatchCable(bank, 1, output, 1);

  bank.reset();
  SetParams();
}

FLASHMEM void SpectraClustCore::Unload() {
  // No factory-owned objects to release; cables are disconnected by the host
  AllowRestart();
}

FLASHMEM void SpectraClustCore::SetParams() {
  // Snap smoothed values to the base params; Controller() adds CV on top
  sm_base = base_freq;
  sm_spread = spread;
  sm_q = q_units;
  sm_morph = morph;
  sm_depth = depth;
  sm_mix = mix;
  for (int n = 0; n < 8; n++) {
    sm_gain[n] = (scene_a[n] + (morph * 0.01f) * (scene_b[n] - scene_a[n]))
      * (1.0f / 63.0f);
  }
}

// 16.6 kHz control tick.  FLASHMEM is fine here (same as MidSider);
// only the audio object's update() does per-sample work.
FLASHMEM void SpectraClustCore::Controller() {
  // Effective values = base param + CV (bipolar, via attenuversion).
  // Locals first: the constrain() macro evaluates its argument repeatedly.
  const float eff_base
    = constrain(base_freq + base_cv.InF() * 400.0f, 0.0f, 2047.0f);
  const float eff_spread
    = constrain(spread + spread_cv.InF() * 64.0f, 0.0f, 255.0f);
  const float eff_q
    = constrain(q_units + q_cv.InF() * 32.0f, 0.0f, 127.0f);
  const float eff_morph
    = constrain(morph + morph_cv.InF() * 50.0f, 0.0f, 100.0f);
  const float eff_depth
    = constrain(depth + depth_cv.InF() * 50.0f, 0.0f, 100.0f);
  const float eff_mix
    = constrain(mix + mix_cv.InF() * 50.0f, 0.0f, 100.0f);

  ONE_POLE(sm_base, eff_base, 0.2f);
  ONE_POLE(sm_spread, eff_spread, 0.2f);
  ONE_POLE(sm_q, eff_q, 0.2f);
  ONE_POLE(sm_morph, eff_morph, 0.1f);
  ONE_POLE(sm_depth, eff_depth, 0.1f);
  ONE_POLE(sm_mix, eff_mix, 0.1f);

  bank.setBase(FreqFromUnits(sm_base));
  bank.setSpread(SpreadFromUnits(static_cast<int>(sm_spread + 0.5f)));
  bank.setQ(QFromUnits(static_cast<int>(sm_q + 0.5f)));
  // depth 0..100% -> 0..1 octave peak deviation (12 semitones)
  bank.setSweep(RateFromUnits(rate), sm_depth * 0.01f);

  // Per-band scene gains (morph crossfade) x equal-power spectral pan.
  // Pan law via sqrt (hardware vsqrt on the M7 - no libm): band n sits at
  // p = (2n/(N-1) - 1) x amount, gl = g*sqrt((1-p)/2), gr = g*sqrt((1+p)/2).
  const float m = sm_morph * 0.01f;
  const float amt = pan * 0.01f;
  float gl[8];
  float gr[8];
  for (int n = 0; n < 8; n++) {
    const float eff_g
      = (scene_a[n] + m * static_cast<float>(scene_b[n] - scene_a[n]))
        * (1.0f / 63.0f);
    ONE_POLE(sm_gain[n], eff_g, 0.3f);
    const float p = (2.0f * n / 7.0f - 1.0f) * amt;
    gl[n] = sm_gain[n] * sqrtf(0.5f * (1.0f - p));
    gr[n] = sm_gain[n] * sqrtf(0.5f * (1.0f + p));
  }
  bank.setGains(gl, gr);

  const float w = sm_mix * 0.01f;
  bank.setMix(1.0f - w, w);
}

FLASHMEM void SpectraClustCore::DrawBars(
  int x0, int y_bottom, int bar_w, int pitch, int max_h
) {
  for (int n = 0; n < 8; n++) {
    int h = 1 + static_cast<int>(sm_gain[n] * (max_h - 1) + 0.5f);
    if (h < 1) h = 1;
    if (h > max_h) h = max_h;
    gfxInvert(x0 + n * pitch, y_bottom - h, bar_w, h);
  }
}

FLASHMEM void SpectraClustCore::View() {
  const int label_x = 1;
  const int page = (cursor >= MORPH) + (cursor >= SCENE) + (cursor >= MIX);
  gfxRect(page * 15, 10, 14, 3); // page tab

  switch (page) {
    case 0: // ---- Cluster ----
      gfxPrint(label_x, 15, PSTR("Frq:"));
      gfxStartCursor();
      graphics.printf(PSTR("%5d"),
                      static_cast<int>(FreqFromUnits(base_freq)));
      gfxEndCursor(cursor == BASE);
      gfxStartCursor();
      gfxPrint(base_cv);
      gfxEndCursor(cursor == BASE_CV, false, base_cv.InputName());

      gfxPrint(label_x, 25, PSTR("Spr:"));
      gfxStartCursor();
      gfxPrint(SpreadFromUnits(spread), 2);
      gfxEndCursor(cursor == SPREAD);
      gfxStartCursor();
      gfxPrint(spread_cv);
      gfxEndCursor(cursor == SPREAD_CV, false, spread_cv.InputName());

      gfxPrint(label_x, 35, PSTR("Q:  "));
      gfxStartCursor();
      gfxPrint(QFromUnits(q_units), 1);
      gfxEndCursor(cursor == QVAL);
      gfxStartCursor();
      gfxPrint(q_cv);
      gfxEndCursor(cursor == Q_CV, false, q_cv.InputName());
      break;

    case 1: // ---- Morph / animation ----
      gfxPrint(label_x, 15, PSTR("Mrp:"));
      gfxStartCursor();
      graphics.printf(PSTR("%3d%%"), morph);
      gfxEndCursor(cursor == MORPH);
      gfxStartCursor();
      gfxPrint(morph_cv);
      gfxEndCursor(cursor == MORPH_CV, false, morph_cv.InputName());

      gfxPrint(label_x, 25, PSTR("Rte:"));
      gfxStartCursor();
      // integer-only formatting: x.xxHz from millihertz (no %f in this printf)
      {
        const int millihz = static_cast<int>(RateFromUnits(rate) * 100.0f + 0.5f);
        graphics.printf(PSTR("%u.%02uHz"),
                        static_cast<unsigned>(millihz / 100),
                        static_cast<unsigned>(millihz % 100));
      }
      gfxEndCursor(cursor == RATE);

      gfxPrint(label_x, 35, PSTR("Dpt:"));
      gfxStartCursor();
      graphics.printf(PSTR("%3dst"), depth * 12 / 100);
      gfxEndCursor(cursor == DEPTH);
      gfxStartCursor();
      gfxPrint(depth_cv);
      gfxEndCursor(cursor == DEPTH_CV, false, depth_cv.InputName());

      gfxPrint(label_x, 45, PSTR("Pan:"));
      gfxStartCursor();
      graphics.printf(PSTR("%3d%%"), pan);
      gfxEndCursor(cursor == PAN);
      break;

    case 2: // ---- Scenes ----
      gfxPrint(label_x, 15, PSTR("Scn:"));
      gfxStartCursor();
      gfxPrint(scene_sel ? PSTR(" B  ") : PSTR(" A  "));
      gfxEndCursor(cursor == SCENE);
      gfxPrint(44, 15, PSTR("Pre:"));
      gfxStartCursor();
      gfxPrint(SC_PRESET_NAMES[preset_idx]);
      gfxEndCursor(cursor == PRESET);

      // Band gains for the selected scene (4 per row)
      for (int n = 0; n < 8; n++) {
        const int x = label_x + (n & 3) * 32;
        const int y = (n < 4) ? 25 : 35;
        uint8_t* g = scene_sel ? scene_b : scene_a;
        gfxPos(x, y);
        graphics.printf(PSTR("%u:"), static_cast<unsigned>(n + 1));
        gfxStartCursor();
        graphics.printf(PSTR("%2d"), g[n]);
        gfxEndCursor(cursor == GAIN0 + n);
      }
      break;

    case 3: // ---- Output mix ----
      gfxPrint(label_x, 15, PSTR("Mix:"));
      gfxStartCursor();
      graphics.printf(PSTR("%3d%%"), mix);
      gfxEndCursor(cursor == MIX);
      gfxStartCursor();
      gfxPrint(mix_cv);
      gfxEndCursor(cursor == MIX_CV, false, mix_cv.InputName());
      break;
  }

  // Money strip: 8 tiny bars of current (morphed) band gains
  DrawBars(4, 63, 8, 15, 9);

  gfxDisplayInputMapEditor();
}

FLASHMEM void SpectraClustCore::DrawFullScreen() {
  const int x0 = 64 - gfx_offset;
  // Big per-band gain bars across the top half
  DrawBars(x0 + 2, 28, 6, 8, 24);
  graphics.drawLine(x0, 28, 127 - gfx_offset, 28, 3);

  View();
}

FLASHMEM void SpectraClustCore::OnButtonPress() {
  if (CheckEditInputMapPress(
        cursor,
        IndexedInput(BASE_CV, base_cv),
        IndexedInput(SPREAD_CV, spread_cv),
        IndexedInput(Q_CV, q_cv),
        IndexedInput(MORPH_CV, morph_cv),
        IndexedInput(DEPTH_CV, depth_cv),
        IndexedInput(MIX_CV, mix_cv)
      ))
    return;
  CursorToggle();
}

FLASHMEM void SpectraClustCore::OnEncoderMove(int direction) {
  if (!EditMode()) {
    MoveCursor(cursor, direction, CURSOR_LENGTH - 1);
    return;
  }
  if (EditSelectedInputMap(direction)) return;

  uint8_t* g = scene_sel ? scene_b : scene_a;
  switch (cursor) {
    case BASE:
      base_freq = constrain(base_freq + direction * 4, 0, 2047);
      break;
    case BASE_CV: base_cv.ChangeSource(direction); break;
    case SPREAD:
      spread = constrain(spread + direction, 0, 255);
      break;
    case SPREAD_CV: spread_cv.ChangeSource(direction); break;
    case QVAL: q_units = constrain(q_units + direction, 0, 127); break;
    case Q_CV: q_cv.ChangeSource(direction); break;

    case MORPH: morph = constrain(morph + direction, 0, 100); break;
    case MORPH_CV: morph_cv.ChangeSource(direction); break;
    case RATE: rate = constrain(rate + direction, 0, 63); break;
    case DEPTH: depth = constrain(depth + direction, 0, 100); break;
    case DEPTH_CV: depth_cv.ChangeSource(direction); break;
    case PAN: pan = constrain(pan + direction, 0, 100); break;

    case SCENE: scene_sel = constrain(scene_sel + direction, 0, 1); break;
    case PRESET:
      preset_idx = (preset_idx + direction + NUM_PRESETS) % NUM_PRESETS;
      LoadPreset(preset_idx, scene_sel);
      break;
    case GAIN0:
    case GAIN1:
    case GAIN2:
    case GAIN3:
    case GAIN4:
    case GAIN5:
    case GAIN6:
    case GAIN7:
      g[cursor - GAIN0] = constrain(g[cursor - GAIN0] + direction, 0, 63);
      break;

    case MIX: mix = constrain(mix + direction, 0, 100); break;
    case MIX_CV: mix_cv.ChangeSource(direction); break;

    default: return;
  }
  SetParams();
}

// 256-bit state layout (4 x 64-bit words), everything in-bounds:
//   word 0 (60b): base_freq(11) spread(8) q(7) morph(7)
//                 rate(6) depth(7) pan(7) mix(7)
//   word 1 (64b): scene_a[8] gains (8 x 6) + morph_cv(16)
//   word 2 (64b): scene_b[8] gains (8 x 6) + base_cv(16)
//   word 3 (64b): spread_cv, depth_cv, q_cv, mix_cv (4 x 16)
// 252 bits used, 4 spare.  CV maps (priority): morph, base freq, spread,
// animation depth, Q, dry/wet.  Transient UI state (scene_sel, preset_idx)
// is deliberately not persisted.
FLASHMEM void SpectraClustCore::OnDataRequest(
  std::array<uint64_t, CONFIG_SIZE>& data
) {
  data[0] = PackPackables(
    pack<11>(base_freq),
    pack<8>(spread),
    pack<7>(q_units),
    pack<7>(morph),
    pack<6>(rate),
    pack<7>(depth),
    pack<7>(pan),
    pack<7>(mix)
  );
  data[1] = PackPackables(
    pack<6>(scene_a[0]),
    pack<6>(scene_a[1]),
    pack<6>(scene_a[2]),
    pack<6>(scene_a[3]),
    pack<6>(scene_a[4]),
    pack<6>(scene_a[5]),
    pack<6>(scene_a[6]),
    pack<6>(scene_a[7]),
    morph_cv
  );
  data[2] = PackPackables(
    pack<6>(scene_b[0]),
    pack<6>(scene_b[1]),
    pack<6>(scene_b[2]),
    pack<6>(scene_b[3]),
    pack<6>(scene_b[4]),
    pack<6>(scene_b[5]),
    pack<6>(scene_b[6]),
    pack<6>(scene_b[7]),
    base_cv
  );
  data[3] = PackPackables(spread_cv, depth_cv, q_cv, mix_cv);
}

FLASHMEM void SpectraClustCore::OnDataReceive(
  const std::array<uint64_t, CONFIG_SIZE>& data
) {
  UnpackPackables(
    data[0],
    pack<11>(base_freq),
    pack<8>(spread),
    pack<7>(q_units),
    pack<7>(morph),
    pack<6>(rate),
    pack<7>(depth),
    pack<7>(pan),
    pack<7>(mix)
  );
  UnpackPackables(
    data[1],
    pack<6>(scene_a[0]),
    pack<6>(scene_a[1]),
    pack<6>(scene_a[2]),
    pack<6>(scene_a[3]),
    pack<6>(scene_a[4]),
    pack<6>(scene_a[5]),
    pack<6>(scene_a[6]),
    pack<6>(scene_a[7]),
    morph_cv
  );
  UnpackPackables(
    data[2],
    pack<6>(scene_b[0]),
    pack<6>(scene_b[1]),
    pack<6>(scene_b[2]),
    pack<6>(scene_b[3]),
    pack<6>(scene_b[4]),
    pack<6>(scene_b[5]),
    pack<6>(scene_b[6]),
    pack<6>(scene_b[7]),
    base_cv
  );
  UnpackPackables(data[3], spread_cv, depth_cv, q_cv, mix_cv);

  // re-derive packed ranges defensively
  q_units = constrain(q_units, 0, 127);
  morph = constrain(morph, 0, 100);
  rate = constrain(rate, 0, 63);
  depth = constrain(depth, 0, 100);
  pan = constrain(pan, 0, 100);
  mix = constrain(mix, 0, 100);
  for (int n = 0; n < 8; n++) {
    scene_a[n] = constrain(scene_a[n], 0, 63);
    scene_b[n] = constrain(scene_b[n], 0, 63);
  }

  SetParams();
}

FLASHMEM void SpectraClustCore::SetHelp() {
  help[HELP_DIGITAL1] = PSTR(" - ");
  help[HELP_DIGITAL2] = PSTR(" - ");
  help[HELP_CV1]      = PSTR("Morph");
  help[HELP_CV2]      = PSTR("Freq");
  help[HELP_OUT1]     = PSTR("Out L");
  help[HELP_OUT2]     = PSTR("Out R");
  help[HELP_EXTRA1]   = PSTR("8-band cluster");
  help[HELP_EXTRA2]   = PSTR("scenes + pan");
}
