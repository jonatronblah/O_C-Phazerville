#pragma once

#include "../HemisphereAudioApplet.h"
#include "../src/Audio/effect_width.h"
#include "../Audio/AudioMixer.h"
#include "../Audio/AudioPassthrough.h"
#include "../dsputils.h"
#include <Audio.h>

// CascDyn - 3-band OTT-style multiband dynamics: downward AND upward
// compression per band, one Macro depth knob (0-300%), optional cascading
// sidechains (band N's gain computer ducks band N+1).
//
// Signal chain (custom AudioStream nodes below; stereo, one split per side):
//
//   in L ─┬─ CascDynXover L ─┬─ lo ── CascDynBand 0 ──┐
//         │                  ├─ mid ─ CascDynBand 1 ──┼─ AudioMixer<3> ─ out L
//         │                  └─ hi ── CascDynBand 2 ──┘        │
//   in R ─┴─ CascDynXover R ─┬─ lo ── CascDynBand 0 ──┐        ├─ out R
//                            ├─ mid ─ CascDynBand 1 ──┤        │
//                            └─ hi ── CascDynBand 2 ──┴────────┘
//
// BAND SPLIT: serial LR4 crossovers, coefficients from the trig-free RBJ
// helper WidthDynamicsBiquadCoeffs() in effect_width.h (never setLowpass()
// etc. - those pull libm sin/cos into the 32KB ITCM bank):
//   low  = LP4(f1)(x)   (two cascaded Butterworth biquads, Q = 0.7071)
//   h1   = HP4(f1)(x)
//   mid  = LP4(f2)(h1)
//   high = HP4(f2)(h1)
// LR4 has the LP4+HP4 = allpass property, so the three bands recombine to a
// flat-magnitude ("Linkwitz-Riley-ish") sum. Splits f1 "SLo" / f2 "SHi" are
// 20 Hz..20 kHz log units (defaults 250 / 2500 Hz).
//
// PER BAND (stereo-linked peak detector + soft-knee gain computer, math
// cribbed from effect_dynamics.cpp - Marc Paquette / PJRC, MIT - rather than
// the AudioEffectDynamics factory pool, because those objects have neither
// upward compression nor a sidechain input):
//   env  = one-pole peak follower per channel (attack/release alphas)
//   down = soft-knee gain computer above the threshold (<= 0 dB), same
//          quadratic knee form as effect_dynamics (knee fixed 6 dB)
//   up   = the mirrored soft-knee boost below the threshold (>= 0 dB,
//          capped at +24 dB), scaled by the global "Up" amount
//   duck = cascade depth x (previous band's smoothed downward gain)
//   gain = down + up + duck, smoothed (attack on engage, release on relax);
//          "Lnk" on = both channels share the max detector (stereo-linked),
//          off = independent dual-mono detectors
//   out  = in + depth * (dbToScalar(gain) * in - in)
// where depth = "Mac" / 100 in [0..3]: 1 = fully compressed (OTT 100%),
// 2 = 100% "over" depth (wet x2 / dry x-1), 3 = wet x3 / dry x-2.
//
// CASCADING SIDECHAINS ("Cas" x "Cscp"): band N's smoothed downward gain
// reduction ducks band N+1 (low -> mid -> high). Because each band's gain
// already includes its own duck, the chain propagates: heavy lows pull the
// mids, which in turn pull the highs. Cross-band keys are plain float reads
// of the previous band object (single-TU integration); AudioStream update
// order is construction order, so a key is at most one block stale.
//
// MEMORY LAYOUT NOTE (from the MidSider/ClockLoop builds): GCC drops section
// attributes on vague-linkage entities (inline/template members), so ALL
// code lives in non-template classes with out-of-line FLASHMEM definitions
// (update() included - the ITCM bank budget is exhausted). CascDyn<Channels>
// is the thin registered wrapper. UI strings use PSTR()/PROGMEM so they land
// in flash instead of the DTCM .rodata pool.

// Name string forced into flash (.progmem) to protect the DTCM budget.
static const char CASC_DYN_NAME[] PROGMEM = "CascDyn";

// 1-in / 3-out LR4-ish band split for one audio channel.
class CascDynXover : public AudioStream {
public:
  CascDynXover();
  // Recompute the eight stage coefficients (control context; skips when the
  // frequencies have not moved).
  void crossover(float f1, float f2);
  void reset();

private:
  // Direct form I biquad; Process() is out-of-line FLASHMEM (no inlines!).
  struct Biq {
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;
    float Process(float x);
    void Zero();
  };

  audio_block_t* inputQueueArray[1];
  Biq lp1_, lp2_; // LR4 lowpass @ f1
  Biq hp1_, hp2_; // LR4 highpass @ f1
  Biq lp3_, lp4_; // LR4 lowpass @ f2 (on the HP4(f1) branch)
  Biq hp3_, hp4_; // LR4 highpass @ f2 (on the HP4(f1) branch)
  float ap_f1_ = -1.0f, ap_f2_ = -1.0f;

  virtual void update(void);
};

// 2-in / 2-out dynamics band: L/R audio in, macro-processed L/R out.
class CascDynBand : public AudioStream {
public:
  CascDynBand();
  // Control-context parameter push (atomic float stores, ISR-read).
  //   thr_db   [-64, 0], ratio [1, 60] (x:1), atk/rel in seconds
  //   up_amt   [0, 1] scales the upward-compression side
  //   depth    macro depth [0, 3]
  //   casc     cascade duck depth [0, 1] (0 = off)
  //   linked   true = stereo-linked detectors
  void params(
    float thr_db,
    float ratio,
    float atk_s,
    float rel_s,
    float up_amt,
    float depth,
    float casc,
    bool linked
  );
  // Previous band in the cascade chain (nullptr = none). Its smoothed
  // downward gain is read directly from update().
  void cascadeFrom(const CascDynBand* prev);
  // Smoothed total gain in dB (down + up), for the UI meters.
  float gainDb(int ch) const;
  void reset();

private:
  static constexpr float CD_MIN_DB = -90.0f;
  static constexpr float CD_KNEE = 6.0f;
  static constexpr float CD_UP_MAX_DB = 24.0f;

  audio_block_t* inputQueueArray[2];

  // ISR-owned state (2 channels)
  float env_[2];
  float compdb_[2]; // smoothed downward + duck gain (<= 0)
  float updb_[2];   // smoothed upward boost (>= 0)
  float depth_s_;   // smoothed macro depth

  // control-written params
  float thr_db_ = -20.0f;
  float comp_ratio_ = 0.25f;  // 1 / ratio
  float knee_ratio_ = -0.75f; // comp_ratio - 1
  float up_knee_ratio_ = 0.0f; // (1 - comp_ratio) * up_amt
  float up_slope_ = 0.0f;      // (1 - comp_ratio) * up_amt
  float two_knee_width_ = 1.0f / (CD_KNEE * 2.0f);
  float low_knee_ = -23.0f, high_knee_ = -17.0f;
  float a_att_ = 0.0f, a_rel_ = 0.0f;
  float depth_t_ = 1.0f;
  float casc_ = 0.0f;
  bool linked_ = true;
  const CascDynBand* prev_ = nullptr;

  virtual void update(void);
};

class CascDynCore : public HemisphereAudioApplet {
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
    // Page 0: Low band
    B1_THRESH = 0, B1_RATIO, B1_ATK, B1_REL, SPL_O,
    // Page 1: Mid band (threshold carries the CV row)
    B2_THRESH, B2_THRESH_CV, B2_RATIO, B2_ATK, B2_REL,
    // Page 2: High band
    B3_THRESH, B3_RATIO, B3_ATK, B3_REL, SPL_H,
    // Page 3: global
    UP_AMT, UP_CV, MACRO, MAC_CV,
    CAS_EN, CAS_DEPTH, CAS_CV,
    TSCALE, TSCALE_CV, LINK, OUT_GAIN,
    CURSOR_LENGTH,
  };

  // ---- params (state round-trip via OnDataRequest/Receive) ----------------
  int8_t thr[3] = {-18, -20, -24}; // dB, [-64, 0]
  uint8_t ratio[3] = {4, 4, 4};    // x:1, [1, 60]
  uint8_t atk[3] = {12, 10, 8};    // 0..31, log 3 ms..4 s
  uint8_t rel[3] = {20, 18, 16};   // 0..31
  uint16_t spl_o = 890;  // 0..2047 units, log 20 Hz..20 kHz (~250 Hz)
  uint16_t spl_h = 1283; // (~2500 Hz)
  uint8_t up = 40;     // upward amount, 0..100%
  uint16_t mac = 100;  // macro depth, 0..300%
  bool cas = false;    // cascading sidechains on/off
  uint8_t cas_d = 50;  // cascade depth, 0..100%
  uint8_t tsc = 16;    // atk/rel timescale, 0..31 (16 = x1)
  bool lnk = true;     // stereo link
  int8_t out_gain = 0; // dB, [-16, 15]

  CVInputMap mac_cv;
  CVInputMap thr2_cv;
  CVInputMap up_cv;
  CVInputMap cas_cv;
  CVInputMap tsc_cv;

  int8_t cursor = B1_THRESH;
  bool alloc_ok = false;

  // smoothed effective values (see Controller)
  float sm_mac = 100, sm_up = 40, sm_casd = 50, sm_tsc = 16;
  float sm_thr2 = -20, sm_out = 0;
  float sm_spl_o = 890, sm_spl_h = 1283;

  // 0..2047 units -> 20 Hz..20 kHz, log (same as MidSider)
  static float FreqFromUnits(float units);
  // 0..31 units -> 3 ms..4 s, log
  static float TimeFromUnits(int units);
  static int MillisFromUnits(int units);

  void DrawGrStrip(int x, int y);

  AudioPassthrough<2> input;
  CascDynXover xover[2]; // L, R
  CascDynBand band[3];   // low, mid, high
  AudioMixer<3> mix[2];  // L, R band sum
  AudioPassthrough<2> output;
};

// Thin template wrapper so the registry can use the CascDyn<STEREO> form.
// (All real work lives in the non-template core so FLASHMEM section placement
// survives this toolchain; Channels is intentionally unused.)
template <AudioChannels Channels>
class CascDyn : public CascDynCore {
public:
  const char* applet_name() {
    return CASC_DYN_NAME;
  }
};

// -- Out-of-line definitions (strong symbols, FLASHMEM honored) --------------

FLASHMEM float CascDynXover::Biq::Process(float x) {
  const float y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
  x2 = x1;
  x1 = x;
  y2 = y1;
  y1 = y;
  return y;
}

FLASHMEM void CascDynXover::Biq::Zero() {
  x1 = x2 = y1 = y2 = 0.0f;
}

FLASHMEM CascDynXover::CascDynXover() : AudioStream(1, inputQueueArray) {
  reset();
  // identity-ish defaults until crossover() runs (mid band passes via LP path)
  float coef[5];
  WidthDynamicsBiquadCoeffs(0, 250.0f, 0.7071f, coef);
  lp1_.b0 = coef[0]; lp1_.b1 = coef[1]; lp1_.b2 = coef[2];
  lp1_.a1 = coef[3]; lp1_.a2 = coef[4];
  lp2_ = lp1_;
  WidthDynamicsBiquadCoeffs(1, 250.0f, 0.7071f, coef);
  hp1_.b0 = coef[0]; hp1_.b1 = coef[1]; hp1_.b2 = coef[2];
  hp1_.a1 = coef[3]; hp1_.a2 = coef[4];
  hp2_ = hp1_;
  WidthDynamicsBiquadCoeffs(0, 2500.0f, 0.7071f, coef);
  lp3_.b0 = coef[0]; lp3_.b1 = coef[1]; lp3_.b2 = coef[2];
  lp3_.a1 = coef[3]; lp3_.a2 = coef[4];
  lp4_ = lp3_;
  WidthDynamicsBiquadCoeffs(1, 2500.0f, 0.7071f, coef);
  hp3_.b0 = coef[0]; hp3_.b1 = coef[1]; hp3_.b2 = coef[2];
  hp3_.a1 = coef[3]; hp3_.a2 = coef[4];
  hp4_ = hp3_;
}

FLASHMEM void CascDynXover::reset() {
  lp1_.Zero(); lp2_.Zero(); hp1_.Zero(); hp2_.Zero();
  lp3_.Zero(); lp4_.Zero(); hp3_.Zero(); hp4_.Zero();
}

FLASHMEM void CascDynXover::crossover(float f1, float f2) {
  if (fabsf(f1 - ap_f1_) < 0.5f && fabsf(f2 - ap_f2_) < 0.5f) return;
  ap_f1_ = f1;
  ap_f2_ = f2;
  float coef[5];
  WidthDynamicsBiquadCoeffs(0, f1, 0.7071f, coef);
  lp1_.b0 = coef[0]; lp1_.b1 = coef[1]; lp1_.b2 = coef[2];
  lp1_.a1 = coef[3]; lp1_.a2 = coef[4];
  lp2_ = lp1_;
  WidthDynamicsBiquadCoeffs(1, f1, 0.7071f, coef);
  hp1_.b0 = coef[0]; hp1_.b1 = coef[1]; hp1_.b2 = coef[2];
  hp1_.a1 = coef[3]; hp1_.a2 = coef[4];
  hp2_ = hp1_;
  WidthDynamicsBiquadCoeffs(0, f2, 0.7071f, coef);
  lp3_.b0 = coef[0]; lp3_.b1 = coef[1]; lp3_.b2 = coef[2];
  lp3_.a1 = coef[3]; lp3_.a2 = coef[4];
  lp4_ = lp3_;
  WidthDynamicsBiquadCoeffs(1, f2, 0.7071f, coef);
  hp3_.b0 = coef[0]; hp3_.b1 = coef[1]; hp3_.b2 = coef[2];
  hp3_.a1 = coef[3]; hp3_.a2 = coef[4];
  hp4_ = hp3_;
}

FLASHMEM void CascDynXover::update(void) {
  audio_block_t* block = receiveWritable(0);
  if (!block) return;
  audio_block_t* out_lo = allocate();
  audio_block_t* out_mid = allocate();
  audio_block_t* out_hi = allocate();
  if (!out_lo || !out_mid || !out_hi) {
    if (out_lo) release(out_lo);
    if (out_mid) release(out_mid);
    if (out_hi) release(out_hi);
    release(block);
    return;
  }

  for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
    const float x = block->data[i] * (1.0f / 32768.0f);
    // serial LR4 split (see header comment)
    float lo = lp1_.Process(x);
    lo = lp2_.Process(lo);
    float h1 = hp1_.Process(x);
    h1 = hp2_.Process(h1);
    float mid = lp3_.Process(h1);
    mid = lp4_.Process(mid);
    float hi = hp3_.Process(h1);
    hi = hp4_.Process(hi);
    out_lo->data[i] = Clip16(lo * 32768.0f);
    out_mid->data[i] = Clip16(mid * 32768.0f);
    out_hi->data[i] = Clip16(hi * 32768.0f);
  }

  transmit(out_lo, 0);
  release(out_lo);
  transmit(out_mid, 1);
  release(out_mid);
  transmit(out_hi, 2);
  release(out_hi);
  release(block);
}

FLASHMEM CascDynBand::CascDynBand() : AudioStream(2, inputQueueArray) {
  reset();
}

FLASHMEM void CascDynBand::reset() {
  env_[0] = env_[1] = 0.0f;
  compdb_[0] = compdb_[1] = 0.0f;
  updb_[0] = updb_[1] = 0.0f;
  depth_s_ = depth_t_;
}

FLASHMEM void CascDynBand::cascadeFrom(const CascDynBand* prev) {
  prev_ = prev;
}

FLASHMEM float CascDynBand::gainDb(int ch) const {
  return compdb_[ch] + updb_[ch];
}

FLASHMEM void CascDynBand::params(
  float thr_db,
  float ratio,
  float atk_s,
  float rel_s,
  float up_amt,
  float depth,
  float casc,
  bool linked
) {
  if (thr_db < -64.0f) thr_db = -64.0f;
  if (thr_db > 0.0f) thr_db = 0.0f;
  if (ratio < 1.0f) ratio = 1.0f;
  if (ratio > 60.0f) ratio = 60.0f;
  if (atk_s < 0.003f) atk_s = 0.003f;
  if (atk_s > 4.0f) atk_s = 4.0f;
  if (rel_s < 0.003f) rel_s = 0.003f;
  if (rel_s > 4.0f) rel_s = 4.0f;
  if (up_amt < 0.0f) up_amt = 0.0f;
  if (up_amt > 1.0f) up_amt = 1.0f;
  if (depth < 0.0f) depth = 0.0f;
  if (depth > 3.0f) depth = 3.0f;
  if (casc < 0.0f) casc = 0.0f;
  if (casc > 1.0f) casc = 1.0f;

  thr_db_ = thr_db;
  comp_ratio_ = 1.0f / ratio;
  knee_ratio_ = comp_ratio_ - 1.0f;
  up_knee_ratio_ = (1.0f - comp_ratio_) * up_amt;
  up_slope_ = (1.0f - comp_ratio_) * up_amt;
  two_knee_width_ = 1.0f / (CD_KNEE * 2.0f);
  low_knee_ = thr_db - CD_KNEE * 0.5f;
  high_knee_ = thr_db + CD_KNEE * 0.5f;
  a_att_ = fastexp(-0.9542f / (AUDIO_SAMPLE_RATE_EXACT * atk_s));
  a_rel_ = fastexp(-0.9542f / (AUDIO_SAMPLE_RATE_EXACT * rel_s));
  depth_t_ = depth;
  casc_ = casc;
  linked_ = linked;
}

FLASHMEM void CascDynBand::update(void) {
  audio_block_t* bl = receiveWritable(0);
  audio_block_t* br = receiveWritable(1);
  if (!bl && !br) return;

  // A missing channel is treated as silence (keeps the link detector honest)
  if (!bl || !br) {
    audio_block_t* zb = allocate();
    if (!zb) {
      if (bl) release(bl);
      if (br) release(br);
      return;
    }
    memset(zb->data, 0, sizeof(zb->data));
    if (!bl) {
      bl = zb;
    } else {
      br = zb;
    }
  }

  float env0 = env_[0], env1 = env_[1];
  float cd0 = compdb_[0], cd1 = compdb_[1];
  float ud0 = updb_[0], ud1 = updb_[1];
  float depth_s = depth_s_;

  // cascade key: previous band's smoothed downward gain (<= 0)
  float duckdb = 0.0f;
  if (prev_ && casc_ > 0.0f) {
    float p0 = prev_->compdb_[0];
    float p1 = prev_->compdb_[1];
    duckdb = casc_ * (p0 < p1 ? p0 : p1);
  }

  for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
    const float l = bl->data[i] * (1.0f / 32768.0f);
    const float r = br->data[i] * (1.0f / 32768.0f);

    // one-pole peak followers
    float a = (l < 0.0f) ? -l : l;
    env0 += (a - env0) * ((a > env0) ? a_att_ : a_rel_);
    a = (r < 0.0f) ? -r : r;
    env1 += (a - env1) * ((a > env1) ? a_att_ : a_rel_);

    float e0 = env0, e1 = env1;
    if (linked_) {
      const float em = (e0 > e1) ? e0 : e1;
      e0 = e1 = em;
    }

    // channel 0
    float inputdb = (e0 > 0.0f) ? scalarToDb(e0) : CD_MIN_DB;
    float attdb = 0.0f;
    if (inputdb >= low_knee_) {
      if (inputdb <= high_knee_) {
        const float knee = inputdb - low_knee_;
        attdb = knee_ratio_ * knee * knee * two_knee_width_;
      } else {
        attdb = thr_db_ + (inputdb - thr_db_) * comp_ratio_ - inputdb;
      }
    }
    float dtarget = attdb + duckdb;
    if (dtarget <= cd0) cd0 += (dtarget - cd0) * a_att_;
    else cd0 += (dtarget - cd0) * a_rel_;

    float udb = 0.0f;
    if (inputdb < high_knee_) {
      if (inputdb > low_knee_) {
        const float knee = high_knee_ - inputdb;
        udb = up_knee_ratio_ * knee * knee * two_knee_width_;
      } else {
        udb = (thr_db_ - inputdb) * up_slope_;
      }
    }
    if (udb > CD_UP_MAX_DB) udb = CD_UP_MAX_DB;
    if (udb >= ud0) ud0 += (udb - ud0) * a_att_;
    else ud0 += (udb - ud0) * a_rel_;

    // channel 1
    inputdb = (e1 > 0.0f) ? scalarToDb(e1) : CD_MIN_DB;
    attdb = 0.0f;
    if (inputdb >= low_knee_) {
      if (inputdb <= high_knee_) {
        const float knee = inputdb - low_knee_;
        attdb = knee_ratio_ * knee * knee * two_knee_width_;
      } else {
        attdb = thr_db_ + (inputdb - thr_db_) * comp_ratio_ - inputdb;
      }
    }
    dtarget = attdb + duckdb;
    if (dtarget <= cd1) cd1 += (dtarget - cd1) * a_att_;
    else cd1 += (dtarget - cd1) * a_rel_;

    udb = 0.0f;
    if (inputdb < high_knee_) {
      if (inputdb > low_knee_) {
        const float knee = high_knee_ - inputdb;
        udb = up_knee_ratio_ * knee * knee * two_knee_width_;
      } else {
        udb = (thr_db_ - inputdb) * up_slope_;
      }
    }
    if (udb > CD_UP_MAX_DB) udb = CD_UP_MAX_DB;
    if (udb >= ud1) ud1 += (udb - ud1) * a_att_;
    else ud1 += (udb - ud1) * a_rel_;

    // macro depth smoothing (zipper-free while sweeping the OTT depth knob)
    depth_s += (depth_t_ - depth_s) * 0.02f;

    // out = in + depth * (compressed - in)
    const float g0 = dbToScalar(cd0 + ud0);
    const float g1 = dbToScalar(cd1 + ud1);
    const float s0 = 1.0f + depth_s * (g0 - 1.0f);
    const float s1 = 1.0f + depth_s * (g1 - 1.0f);
    bl->data[i] = Clip16(l * s0 * 32768.0f);
    br->data[i] = Clip16(r * s1 * 32768.0f);
  }

  env_[0] = env0;
  env_[1] = env1;
  compdb_[0] = cd0;
  compdb_[1] = cd1;
  updb_[0] = ud0;
  updb_[1] = ud1;
  depth_s_ = depth_s;

  transmit(bl, 0);
  release(bl);
  transmit(br, 1);
  release(br);
}

// -- Applet core --------------------------------------------------------------

FLASHMEM float CascDynCore::FreqFromUnits(float units) {
  return 20.0f * fastpow2(units * (9.965784f / 2047.0f));
}

FLASHMEM float CascDynCore::TimeFromUnits(int units) {
  return 0.003f * fastpow2(units * (10.3803f / 31.0f));
}

FLASHMEM int CascDynCore::MillisFromUnits(int units) {
  return static_cast<int>(TimeFromUnits(units) * 1000.0f + 0.5f);
}

FLASHMEM void CascDynCore::Start() {
  // band cascade chain: low -> mid -> high
  band[0].cascadeFrom(nullptr);
  band[1].cascadeFrom(&band[0]);
  band[2].cascadeFrom(&band[1]);

  for (int ch = 0; ch < 2; ch++) {
    PatchCable(input, ch, xover[ch], 0);
    PatchCable(xover[ch], 0, band[0], ch); // low
    PatchCable(xover[ch], 1, band[1], ch); // mid
    PatchCable(xover[ch], 2, band[2], ch); // high
    for (int b = 0; b < 3; b++) {
      PatchCable(band[b], ch, mix[ch], b);
    }
    PatchCable(mix[ch], 0, output, ch);
  }
  // 16 cables total (<= 32)

  alloc_ok = true;
  SetParams();
}

FLASHMEM void CascDynCore::Unload() {
  // No factory-owned objects; cables are disconnected by the host
  alloc_ok = false;
  AllowRestart();
}

FLASHMEM void CascDynCore::SetParams() {
  if (!alloc_ok) return;
  sm_mac = mac;
  sm_up = up;
  sm_casd = cas_d;
  sm_tsc = tsc;
  sm_thr2 = thr[1];
  sm_out = out_gain;
  sm_spl_o = spl_o;
  sm_spl_h = spl_h;
}

// 16.6 kHz control tick. FLASHMEM is fine here; only the audio ISR's update()
// loops run from flash too (deliberately - see header note).
FLASHMEM void CascDynCore::Controller() {
  if (!alloc_ok) return;

  // effective values = base param + CV (bipolar, via attenuversion)
  const float eff_mac = constrain(mac + mac_cv.InF() * 150.0f, 0.0f, 300.0f);
  const float eff_up = constrain(up + up_cv.InF() * 50.0f, 0.0f, 100.0f);
  const float eff_casd = constrain(cas_d + cas_cv.InF() * 50.0f, 0.0f, 100.0f);
  const float eff_tsc = constrain(tsc + tsc_cv.InF() * 16.0f, 0.0f, 31.0f);
  const float eff_thr2 = constrain(
    float(thr[1]) + thr2_cv.InF() * 24.0f, -64.0f, 0.0f
  );
  const float eff_spl_o = float(spl_o);
  const float eff_spl_h = float(spl_h);

  ONE_POLE(sm_mac, eff_mac, 0.1f);
  ONE_POLE(sm_up, eff_up, 0.1f);
  ONE_POLE(sm_casd, eff_casd, 0.1f);
  ONE_POLE(sm_tsc, eff_tsc, 0.1f);
  ONE_POLE(sm_thr2, eff_thr2, 0.1f);
  ONE_POLE(sm_out, float(out_gain), 0.1f);
  ONE_POLE(sm_spl_o, eff_spl_o, 0.2f);
  ONE_POLE(sm_spl_h, eff_spl_h, 0.2f);

  for (int ch = 0; ch < 2; ch++) {
    xover[ch].crossover(FreqFromUnits(sm_spl_o), FreqFromUnits(sm_spl_h));
  }

  // atk/rel timescale: 0..31 units -> x0.25..x3.7 (16 = x1)
  const float tsc_scale = fastpow2((sm_tsc - 16.0f) * 0.125f);
  const float casc = cas ? (sm_casd * 0.01f) : 0.0f;
  const float depth = sm_mac * 0.01f;
  const float up_amt = sm_up * 0.01f;

  band[0].params(
    float(thr[0]), float(ratio[0]),
    TimeFromUnits(atk[0]) * tsc_scale, TimeFromUnits(rel[0]) * tsc_scale,
    up_amt, depth, casc, lnk
  );
  band[1].params(
    sm_thr2, float(ratio[1]),
    TimeFromUnits(atk[1]) * tsc_scale, TimeFromUnits(rel[1]) * tsc_scale,
    up_amt, depth, casc, lnk
  );
  band[2].params(
    float(thr[2]), float(ratio[2]),
    TimeFromUnits(atk[2]) * tsc_scale, TimeFromUnits(rel[2]) * tsc_scale,
    up_amt, depth, casc, lnk
  );

  const float og = dbToScalar(sm_out);
  mix[0].gain(0, og);
  mix[0].gain(1, og);
  mix[0].gain(2, og);
  mix[1].gain(0, og);
  mix[1].gain(1, og);
  mix[1].gain(2, og);
}

// --- UI ---------------------------------------------------------------------

FLASHMEM void CascDynCore::DrawGrStrip(int x, int y) {
  // 3 mini gain meters (low/mid/high): one horizontal bar per band, width =
  // |gain| (24 dB full scale == 14 px); the fullscreen view shows the sign.
  for (int b = 0; b < 3; b++) {
    int g = static_cast<int>(band[b].gainDb(0));
    int w = Proportion(g < 0 ? -g : g, 24, 14);
    if (w > 14) w = 14;
    gfxInvert(x + b * 16, y, w, 7);
  }
}

FLASHMEM void CascDynCore::View() {
  if (!alloc_ok) {
    gfxPrint(2, 15, PSTR("Out of RAM!!"));
    return;
  }

  const int label_x = 1;
  const int page = (cursor > SPL_O) + (cursor > B2_REL) + (cursor > SPL_H);
  gfxRect(page * 15, 10, 14, 3); // page tab

  switch (page) {
    case 0: // ---- Low band ----
      gfxPrint(label_x, 15, PSTR("L:"));
      gfxStartCursor();
      graphics.printf(PSTR("%3ddB"), thr[0]);
      gfxEndCursor(cursor == B1_THRESH);

      gfxPrint(label_x, 25, PSTR("Rat:"));
      gfxStartCursor();
      graphics.printf(PSTR("%2d:1"), ratio[0]);
      gfxEndCursor(cursor == B1_RATIO);

      gfxPrint(label_x, 35, PSTR("Atk:"));
      gfxStartCursor();
      graphics.printf(PSTR("%4dms"), MillisFromUnits(atk[0]));
      gfxEndCursor(cursor == B1_ATK);

      gfxPrint(label_x, 45, PSTR("Rel:"));
      gfxStartCursor();
      graphics.printf(PSTR("%4dms"), MillisFromUnits(rel[0]));
      gfxEndCursor(cursor == B1_REL);

      gfxPrint(label_x, 55, PSTR("SLo:"));
      gfxStartCursor();
      graphics.printf(PSTR("%5d"), static_cast<int>(FreqFromUnits(spl_o)));
      gfxEndCursor(cursor == SPL_O);
      break;

    case 1: // ---- Mid band ----
      gfxPrint(label_x, 15, PSTR("M:"));
      gfxStartCursor();
      graphics.printf(PSTR("%3ddB"), thr[1]);
      gfxEndCursor(cursor == B2_THRESH);
      gfxStartCursor();
      gfxPrint(thr2_cv);
      gfxEndCursor(cursor == B2_THRESH_CV, false, thr2_cv.InputName());

      gfxPrint(label_x, 25, PSTR("Rat:"));
      gfxStartCursor();
      graphics.printf(PSTR("%2d:1"), ratio[1]);
      gfxEndCursor(cursor == B2_RATIO);

      gfxPrint(label_x, 35, PSTR("Atk:"));
      gfxStartCursor();
      graphics.printf(PSTR("%4dms"), MillisFromUnits(atk[1]));
      gfxEndCursor(cursor == B2_ATK);

      gfxPrint(label_x, 45, PSTR("Rel:"));
      gfxStartCursor();
      graphics.printf(PSTR("%4dms"), MillisFromUnits(rel[1]));
      gfxEndCursor(cursor == B2_REL);

      // per-band GR meters (the money strip)
      gfxPrint(label_x, 55, PSTR("GR:"));
      DrawGrStrip(18, 55);
      break;

    case 2: // ---- High band ----
      gfxPrint(label_x, 15, PSTR("H:"));
      gfxStartCursor();
      graphics.printf(PSTR("%3ddB"), thr[2]);
      gfxEndCursor(cursor == B3_THRESH);

      gfxPrint(label_x, 25, PSTR("Rat:"));
      gfxStartCursor();
      graphics.printf(PSTR("%2d:1"), ratio[2]);
      gfxEndCursor(cursor == B3_RATIO);

      gfxPrint(label_x, 35, PSTR("Atk:"));
      gfxStartCursor();
      graphics.printf(PSTR("%4dms"), MillisFromUnits(atk[2]));
      gfxEndCursor(cursor == B3_ATK);

      gfxPrint(label_x, 45, PSTR("Rel:"));
      gfxStartCursor();
      graphics.printf(PSTR("%4dms"), MillisFromUnits(rel[2]));
      gfxEndCursor(cursor == B3_REL);

      gfxPrint(label_x, 55, PSTR("SHi:"));
      gfxStartCursor();
      graphics.printf(PSTR("%5d"), static_cast<int>(FreqFromUnits(spl_h)));
      gfxEndCursor(cursor == SPL_H);
      break;

    default: // ---- Global ----
      gfxPrint(label_x, 15, PSTR("Up:"));
      gfxStartCursor();
      graphics.printf(PSTR("%3d%%"), up);
      gfxEndCursor(cursor == UP_AMT);
      gfxStartCursor();
      gfxPrint(up_cv);
      gfxEndCursor(cursor == UP_CV, false, up_cv.InputName());

      gfxPrint(label_x, 25, PSTR("Mac:"));
      gfxStartCursor();
      graphics.printf(PSTR("%3d%%"), mac);
      gfxEndCursor(cursor == MACRO);
      gfxStartCursor();
      gfxPrint(mac_cv);
      gfxEndCursor(cursor == MAC_CV, false, mac_cv.InputName());

      gfxPrint(label_x, 35, PSTR("Cas:"));
      gfxStartCursor();
      gfxPrintIcon(cas ? CHECK_ON_ICON : CHECK_OFF_ICON);
      gfxEndCursor(cursor == CAS_EN);
      gfxStartCursor();
      graphics.printf(PSTR("%3d%%"), cas_d);
      gfxEndCursor(cursor == CAS_DEPTH);
      gfxStartCursor();
      gfxPrint(cas_cv);
      gfxEndCursor(cursor == CAS_CV, false, cas_cv.InputName());

      gfxPrint(label_x, 45, PSTR("TSc:"));
      gfxStartCursor();
      graphics.printf(PSTR("%3d%%"), static_cast<int>(100.0f * fastpow2((tsc - 16) * 0.125f) + 0.5f));
      gfxEndCursor(cursor == TSCALE);
      gfxStartCursor();
      gfxPrint(tsc_cv);
      gfxEndCursor(cursor == TSCALE_CV, false, tsc_cv.InputName());

      gfxPrint(label_x, 55, PSTR("Lnk:"));
      gfxStartCursor();
      gfxPrintIcon(lnk ? CHECK_ON_ICON : CHECK_OFF_ICON);
      gfxEndCursor(cursor == LINK);
      gfxPrint(PSTR(" Out:"));
      gfxStartCursor();
      graphics.printf(PSTR("%+3ddB"), out_gain);
      gfxEndCursor(cursor == OUT_GAIN);
      break;
  }

  gfxDisplayInputMapEditor();
}

FLASHMEM void CascDynCore::DrawFullScreen() {
  if (!alloc_ok) return;
  const int x0 = 64 - gfx_offset;
  graphics.drawLine(x0, 26, 127 - gfx_offset, 26, 3);

  // 3 per-band gain bars (60 dB == 30 px): down = below the line, up = above
  for (int b = 0; b < 3; b++) {
    const int x = x0 + 10 + b * 18;
    const int h = Proportion(static_cast<int>(band[b].gainDb(0)), 60, 30);
    if (h >= 0) graphics.drawRect(x, 26 - h, 10, h);
    else graphics.drawRect(x, 26, 10, -h);
  }
  graphics.setPrintPos(x0 + 11, 48);
  graphics.print(PSTR("L M H"));

  View();
}

FLASHMEM void CascDynCore::OnButtonPress() {
  if (CheckEditInputMapPress(
        cursor,
        IndexedInput(B2_THRESH_CV, thr2_cv),
        IndexedInput(UP_CV, up_cv),
        IndexedInput(MAC_CV, mac_cv),
        IndexedInput(CAS_CV, cas_cv),
        IndexedInput(TSCALE_CV, tsc_cv)
      ))
    return;
  CursorToggle();
}

FLASHMEM void CascDynCore::OnEncoderMove(int direction) {
  if (!EditMode()) {
    MoveCursor(cursor, direction, CURSOR_LENGTH - 1);
    return;
  }
  if (EditSelectedInputMap(direction)) return;

  switch (cursor) {
    case B1_THRESH: thr[0] = constrain(thr[0] + direction, -64, 0); break;
    case B1_RATIO: ratio[0] = constrain(ratio[0] + direction, 1, 60); break;
    case B1_ATK: atk[0] = constrain(atk[0] + direction, 0, 31); break;
    case B1_REL: rel[0] = constrain(rel[0] + direction, 0, 31); break;
    case SPL_O:
      spl_o = constrain(spl_o + direction * 8, 0, spl_h);
      break;

    case B2_THRESH: thr[1] = constrain(thr[1] + direction, -64, 0); break;
    case B2_THRESH_CV: thr2_cv.ChangeSource(direction); break;
    case B2_RATIO: ratio[1] = constrain(ratio[1] + direction, 1, 60); break;
    case B2_ATK: atk[1] = constrain(atk[1] + direction, 0, 31); break;
    case B2_REL: rel[1] = constrain(rel[1] + direction, 0, 31); break;

    case B3_THRESH: thr[2] = constrain(thr[2] + direction, -64, 0); break;
    case B3_RATIO: ratio[2] = constrain(ratio[2] + direction, 1, 60); break;
    case B3_ATK: atk[2] = constrain(atk[2] + direction, 0, 31); break;
    case B3_REL: rel[2] = constrain(rel[2] + direction, 0, 31); break;
    case SPL_H:
      spl_h = constrain(spl_h + direction * 8, spl_o, 2047);
      break;

    case UP_AMT: up = constrain(up + direction, 0, 100); break;
    case UP_CV: up_cv.ChangeSource(direction); break;
    case MACRO: mac = constrain(mac + direction, 0, 300); break;
    case MAC_CV: mac_cv.ChangeSource(direction); break;
    case CAS_EN: cas = !cas; break;
    case CAS_DEPTH: cas_d = constrain(cas_d + direction, 0, 100); break;
    case CAS_CV: cas_cv.ChangeSource(direction); break;
    case TSCALE: tsc = constrain(tsc + direction, 0, 31); break;
    case TSCALE_CV: tsc_cv.ChangeSource(direction); break;
    case LINK: lnk = !lnk; break;
    case OUT_GAIN: out_gain = constrain(out_gain + direction, -16, 15); break;

    default: return;
  }
  SetParams();
}

// 256-bit state layout (4 x 64-bit words), everything in-bounds:
//   word 0 (64b): thr[0](7) ratio[0](6) atk[0](5) rel[0](5)
//                 thr[1](7) ratio[1](6) atk[1](5) rel[1](5) spl_o(11) up(7)
//   word 1 (62b): thr[2](7) ratio[2](6) atk[2](5) rel[2](5) spl_h(11)
//                 mac(9) cas(1) cas_d(7) tsc(5) lnk(1) out_gain(5)
//   word 2 (64b): mac_cv, thr2_cv, up_cv, cas_cv (4 x 16)
//   word 3 (16b): tsc_cv (16); rest spare
// 210 bits used, 46 spare. thr is signed 7-bit [-64, 63], out_gain signed
// 5-bit [-16, 15].
FLASHMEM void CascDynCore::OnDataRequest(std::array<uint64_t, CONFIG_SIZE>& data) {
  data[0] = PackPackables(
    pack<7>(thr[0]),
    pack<6>(ratio[0]),
    pack<5>(atk[0]),
    pack<5>(rel[0]),
    pack<7>(thr[1]),
    pack<6>(ratio[1]),
    pack<5>(atk[1]),
    pack<5>(rel[1]),
    pack<11>(spl_o),
    pack<7>(up)
  );
  data[1] = PackPackables(
    pack<7>(thr[2]),
    pack<6>(ratio[2]),
    pack<5>(atk[2]),
    pack<5>(rel[2]),
    pack<11>(spl_h),
    pack<9>(mac),
    pack<1>(cas),
    pack<7>(cas_d),
    pack<5>(tsc),
    pack<1>(lnk),
    pack<5>(out_gain)
  );
  data[2] = PackPackables(mac_cv, thr2_cv, up_cv, cas_cv);
  data[3] = PackPackables(tsc_cv);
}

FLASHMEM void CascDynCore::OnDataReceive(
  const std::array<uint64_t, CONFIG_SIZE>& data
) {
  UnpackPackables(
    data[0],
    pack<7>(thr[0]),
    pack<6>(ratio[0]),
    pack<5>(atk[0]),
    pack<5>(rel[0]),
    pack<7>(thr[1]),
    pack<6>(ratio[1]),
    pack<5>(atk[1]),
    pack<5>(rel[1]),
    pack<11>(spl_o),
    pack<7>(up)
  );
  UnpackPackables(
    data[1],
    pack<7>(thr[2]),
    pack<6>(ratio[2]),
    pack<5>(atk[2]),
    pack<5>(rel[2]),
    pack<11>(spl_h),
    pack<9>(mac),
    pack<1>(cas),
    pack<7>(cas_d),
    pack<5>(tsc),
    pack<1>(lnk),
    pack<5>(out_gain)
  );
  UnpackPackables(data[2], mac_cv, thr2_cv, up_cv, cas_cv);
  UnpackPackables(data[3], tsc_cv);

  // re-derive packed ranges defensively
  for (int b = 0; b < 3; b++) {
    thr[b] = constrain(thr[b], -64, 0);
    ratio[b] = constrain(ratio[b], 1, 60);
    atk[b] = constrain(atk[b], 0, 31);
    rel[b] = constrain(rel[b], 0, 31);
  }
  spl_o = constrain(spl_o, 0, 2047);
  spl_h = constrain(spl_h, 0, 2047);
  if (spl_o > spl_h) spl_o = spl_h;
  up = constrain(up, 0, 100);
  mac = constrain(mac, 0, 300);
  cas_d = constrain(cas_d, 0, 100);
  tsc = constrain(tsc, 0, 31);
  out_gain = constrain(out_gain, -16, 15);

  SetParams();
}

FLASHMEM void CascDynCore::SetHelp() {
  help[HELP_DIGITAL1] = PSTR(" - ");
  help[HELP_DIGITAL2] = PSTR(" - ");
  help[HELP_CV1]      = PSTR("Macro");
  help[HELP_CV2]      = PSTR("M Thr");
  help[HELP_OUT1]     = PSTR("Out L");
  help[HELP_OUT2]     = PSTR("Out R");
  help[HELP_EXTRA1] = PSTR("OTT up+down");
  help[HELP_EXTRA2] = PSTR("casc. sidech.");
}
