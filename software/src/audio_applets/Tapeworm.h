#pragma once

#include "../Audio/AudioMixer.h"
#include "../Audio/AudioPassthrough.h"
#include "../dsputils.h"
#include <Audio.h>
#include <smalloc.h>

// Tapeworm - tape & vinyl transport character with CV'd wow, flutter, age,
// dropouts, crackle and rumble.
//
// Signal chain (the transport is one custom AudioStream; everything else is
// stock graph glue):
//
//   in L ─┬─ AudioTapeworm (stereo PSRAM delay line, varispeed read head) ─┐
//   in R ─┘                                                               ├─ (dry/wet) ─ out L/R
//   in L/R ─────────────────────────────── (dry leg, equal-power mix) ─────┘
//
// The tape transport is a fractional-delay line with a time-varying read
// rate.  The write head runs at exactly 1x (48 kHz).  Each channel owns a
// floating-point delay D(t) (frames) that integrates the transport rate:
//
//   D += (1 - rate)              rate = speed * (1 + wow + flutter + rumble)
//   D += (NOMINAL - D) * SERVO   slow capstan servo (tau ~ 8 s)
//   D  clamped to [500, cap-500] hard travel limit
//
// Read position = write - D with linear interpolation (ClockLoop/effect_
// looper Q13 playhead pattern, float flavor).  Sustained speed != 1 drifts D
// until the servo/travel limit returns the effective rate to 1x - see the
// honest limits below.
//
// Warble generators (all per-sample, allocation-free):
//   Wow      two-sine bank (f and 1.37*f, amps 0.62/0.38) -> rate +-2% max.
//            The R channel evaluates the same bank at phase offsets scaled
//            by Stereo Spread (L/R decorrelated wow).
//   Flutter  one-pole lowpassed white noise (cutoff 1.5x rate) per channel,
//            R blended toward L by (1 - spread) -> rate +-0.8% max.
//   Rumble   one-pole lowpassed white noise (1.5 Hz) - slow subsonic delay
//            wander -> rate +-1.5% max.  Shared L/R (turntable platter).
// All three modulate the READ RATE; the delay excursions are the integral,
// bounded by the transport servo (wow 0.1 Hz at full depth = +-32 ms swing).
//
// Vinyl/tape damage (post-read, per channel):
//   Age      macro 0-100%: low-shelf tilt (250 Hz warm boost), 2-pole HF loss
//            (16 kHz -> 3 kHz), tanh saturation, hiss floor, soft compression
//   Dropout  random amplitude dips (shared L/R envelope, fast attack)
//   Crackle  random impulses through a one-pole decay (independent L/R)
//   Drive    tanh record saturation (pre, into the buffer) + light playback
//            soft clip (post)
//
// MEMORY LAYOUT NOTE (from the MidSider/ClockLoop builds): GCC drops section
// attrs on template/inline members, so ALL heavy code lives in the
// non-template classes below (AudioTapeworm, TapewormCore) with strong
// out-of-line FLASHMEM symbols - update() included; Tapeworm<Channels> is
// the thin registered wrapper.  UI strings use PSTR() so they land in flash
// instead of the DTCM .rodata pool.  The delay buffer prefers PSRAM
// (extmem_calloc + explicit memset - extmem_calloc does NOT guarantee
// zero-fill on this core) with a shorter RAM2 malloc fallback and a UI note.

// Name string forced into flash (.progmem) to protect the DTCM/.rodata
// budget; memory-mapped flash reads are fine for display + strhash().
static const char TAPEWORM_NAME[] PROGMEM = "Tapeworm";

extern "C" uint8_t external_psram_size;

// ---------------------------------------------------------------------------
// AudioTapeworm - stereo varispeed fractional delay line + tape/vinyl damage
//
// Concurrency contract (same shape as AudioLooper):
//  - the "control-written" group is only ever written from Controller() /
//    Start() / Unload(); single 32-bit stores are atomic and update() only
//    ever reads them;
//  - the "ISR-owned" group (write index, delays, phases, filters, peak) is
//    only written by update();
//  - update() never allocates (the only allocator is begin(), called from
//    Start() in loop context).
// ---------------------------------------------------------------------------
class AudioTapeworm : public AudioStream {
public:
  // 1.5 s of stereo interleaved int16 @ 48 kHz in PSRAM (288 KB)
  static const size_t MAX_FRAMES = 72000;
  // No-PSRAM fallback starts at 0.375 s and halves until malloc() succeeds
  static const size_t FALLBACK_FRAMES = 18000;
  // Nominal record->playhead distance (frames); the capstan servo target
  static constexpr float NOMINAL_DELAY = 12000.0f; // 250 ms
  static constexpr float MIN_DELAY = 500.0f;
  // capstan servo: ~8 s time constant (per-sample coefficient)
  static constexpr float SERVO_K = 2.6e-6f;

  AudioTapeworm();

  // ---- buffer lifecycle (loop context only) --------------------------------
  bool begin();
  void end();
  bool IsReady() const;
  bool IsPSRAM() const;
  size_t CapacityFrames() const;

  // ---- control-context parameter pushes (atomic single-word stores) --------
  void setSpeed(float ratio);                 // 0.25..4 transport rate
  void setWow(float rate_hz, float depth);    // depth 0..1 -> rate +-2%
  void setFlutter(float rate_hz, float depth);// depth 0..1 -> rate +-0.8%
  void setRumble(float depth);                // depth 0..1 -> rate +-1.5%
  void setSpread(float spread);               // 0..1 L/R warble decorrelation
  void setAge(float age);                     // 0..1 damage macro
  void setDrive(float drive_db);              // 0..24 dB record saturation
  void setDropout(float rate, float depth);   // 0..1 each
  void setCrackle(float density);             // 0..1

  // Wet-output peak meter (UI reads; update() writes)
  uint16_t peak() const;
  void decayPeak();

private:
  // trig-free sine: 9th-order Taylor + quadrant folding (effect_width.h
  // pattern - libm sin/cos pull ~4 KB of reduction code into ITCM)
  static float Sine(float x);
  // tanh via fastexp (no waveshaper tables: shape() allocates)
  static float Saturate(float x);
  static float WhiteNoise(uint32_t& state);

  audio_block_t* inputQueueArray[2];

  // -- written by control context, read by update() -------------------------
  volatile float speed_ = 1.0f;       // transport rate ratio
  volatile float w1_ = 0.0f, w2_ = 0.0f;    // wow phase increments (rad/sample)
  volatile float wow_amp_ = 0.0f;           // total rate deviation amplitude
  volatile float sp1_ = 0.0f, sp2_ = 0.0f;  // R phase offsets (spread-scaled)
  volatile float flut_inc_ = 0.0f;    // flutter LP alpha
  volatile float flut_amp_ = 0.0f;
  volatile float rum_amp_ = 0.0f;
  volatile float spread_mix_ = 1.0f;  // R noise blend toward L: 1-spread
  volatile float age_ = 0.0f;
  volatile float age_hf_a_ = 1.0f;    // HF-loss one-pole alpha
  volatile float pre_gain_ = 1.0f;    // record saturation gain
  volatile float drop_rate_ = 0.0f;   // events probability scale
  volatile float drop_depth_ = 0.0f;
  volatile float crk_prob_ = 0.0f;

  // -- ISR-owned ------------------------------------------------------------
  int16_t* volatile buf_ = nullptr;
  volatile uint32_t cap_frames_ = 0;
  bool in_psram_ = false;
  uint32_t write_ix_ = 0;
  float delay_l_ = NOMINAL_DELAY, delay_r_ = NOMINAL_DELAY;
  float ph1_ = 0.0f, ph2_ = 0.0f;            // wow phases
  float fl_l_ = 0.0f, fl_r_ = 0.0f;          // flutter LP state
  float ru_l_ = 0.0f;                        // rumble LP state (shared)
  float warm_l_ = 0.0f, warm_r_ = 0.0f;      // age tilt state
  float hf_l_ = 0.0f, hf_r_ = 0.0f;          // age HF-loss pole 1
  float hf2_l_ = 0.0f, hf2_r_ = 0.0f;        // age HF-loss pole 2
  float comp_l_ = 0.0f, comp_r_ = 0.0f;      // age compressor envelope
  float drop_env_ = 1.0f;                    // dropout gain (shared)
  float crk_l_ = 0.0f, crk_r_ = 0.0f;        // crackle decay registers
  uint32_t rng_ = 0x12345678;
  volatile uint16_t peak_ = 0;

  virtual void update(void);
};

// Inline-free on purpose: even trivial accessors are out-of-line FLASHMEM so
// the ITCM bank budget never moves (house rule).

FLASHMEM AudioTapeworm::AudioTapeworm() : AudioStream(2, inputQueueArray) {
  inputQueueArray[0] = nullptr;
  inputQueueArray[1] = nullptr;
}

FLASHMEM bool AudioTapeworm::begin() {
  end();
  if (external_psram_size > 0) {
    buf_ = (int16_t*)extmem_calloc(MAX_FRAMES * 2, sizeof(int16_t));
    if (buf_) {
      in_psram_ = true;
      cap_frames_ = MAX_FRAMES;
    }
  }
  if (!buf_) {
    // No PSRAM: progressively smaller buffer from the fast RAM2 heap
    for (size_t frames = FALLBACK_FRAMES; frames >= FALLBACK_FRAMES / 8;
         frames /= 2) {
      buf_ = (int16_t*)malloc(frames * 2 * sizeof(int16_t));
      if (buf_) {
        in_psram_ = false;
        cap_frames_ = frames;
        break;
      }
    }
  }
  if (buf_) {
    // extmem_calloc does NOT guarantee zero-fill on this core; the delay
    // line must start silent or the first pass reads garbage "tape".
    memset(buf_, 0, cap_frames_ * 2 * sizeof(int16_t));
    AudioNoInterrupts();
    write_ix_ = 0;
    delay_l_ = NOMINAL_DELAY;
    delay_r_ = NOMINAL_DELAY;
    ph1_ = 0.0f;
    ph2_ = 0.0f;
    fl_l_ = fl_r_ = ru_l_ = 0.0f;
    warm_l_ = warm_r_ = hf_l_ = hf_r_ = 0.0f;
    hf2_l_ = hf2_r_ = 0.0f;
    comp_l_ = comp_r_ = 0.0f;
    drop_env_ = 1.0f;
    crk_l_ = crk_r_ = 0.0f;
    peak_ = 0;
    AudioInterrupts();
  }
  return buf_ != nullptr;
}

FLASHMEM void AudioTapeworm::end() {
  AudioNoInterrupts();
  int16_t* b = buf_;
  buf_ = nullptr;
  cap_frames_ = 0;
  AudioInterrupts();
  if (b) {
    if (in_psram_) extmem_free(b);
    else free(b);
  }
  in_psram_ = false;
}

FLASHMEM bool AudioTapeworm::IsReady() const {
  return buf_ != nullptr;
}

FLASHMEM bool AudioTapeworm::IsPSRAM() const {
  return in_psram_;
}

FLASHMEM size_t AudioTapeworm::CapacityFrames() const {
  return cap_frames_;
}

FLASHMEM uint16_t AudioTapeworm::peak() const {
  return peak_;
}

FLASHMEM void AudioTapeworm::decayPeak() {
  uint16_t p = peak_;
  if (p > 0) peak_ = p - (p >> 5) - 1;
}

// ---- control-context parameter pushes --------------------------------------

FLASHMEM void AudioTapeworm::setSpeed(float ratio) {
  if (!(ratio > 0.0f)) ratio = 1.0f;
  speed_ = constrain(ratio, 0.25f, 4.0f);
}

FLASHMEM void AudioTapeworm::setWow(float rate_hz, float depth) {
  rate_hz = constrain(rate_hz, 0.05f, 20.0f);
  depth = constrain(depth, 0.0f, 1.0f);
  const float k = 2.0f * 3.14159265f / AUDIO_SAMPLE_RATE_EXACT;
  w1_ = rate_hz * k;
  w2_ = rate_hz * 1.37f * k;
  wow_amp_ = depth * 0.02f; // +-2% rate deviation at full depth
}

FLASHMEM void AudioTapeworm::setFlutter(float rate_hz, float depth) {
  rate_hz = constrain(rate_hz, 1.0f, 60.0f);
  depth = constrain(depth, 0.0f, 1.0f);
  // one-pole LP cutoff tracks the flutter rate (smoothed noise warble)
  const float fc = rate_hz * 1.5f;
  flut_inc_ = 1.0f - fastexp(-6.2831853f * fc / AUDIO_SAMPLE_RATE_EXACT);
  flut_amp_ = depth * 0.008f; // +-0.8% rate deviation
}

FLASHMEM void AudioTapeworm::setRumble(float depth) {
  depth = constrain(depth, 0.0f, 1.0f);
  rum_amp_ = depth * 0.015f; // +-1.5% rate deviation (fixed 1.5 Hz wander)
}

FLASHMEM void AudioTapeworm::setSpread(float spread) {
  spread = constrain(spread, 0.0f, 1.0f);
  sp1_ = spread * 2.4f;
  sp2_ = spread * 1.3f;
  spread_mix_ = 1.0f - spread;
}

FLASHMEM void AudioTapeworm::setAge(float age) {
  age = constrain(age, 0.0f, 1.0f);
  age_ = age;
  // HF loss cutoff 16 kHz -> 3 kHz across the macro
  const float fc = 16000.0f - 13000.0f * age;
  age_hf_a_ = 1.0f - fastexp(-6.2831853f * fc / AUDIO_SAMPLE_RATE_EXACT);
}

FLASHMEM void AudioTapeworm::setDrive(float drive_db) {
  drive_db = constrain(drive_db, 0.0f, 24.0f);
  pre_gain_ = dbToScalar(drive_db);
}

FLASHMEM void AudioTapeworm::setDropout(float rate, float depth) {
  rate = constrain(rate, 0.0f, 1.0f);
  drop_rate_ = rate * rate * (25.0f / AUDIO_SAMPLE_RATE_EXACT);
  drop_depth_ = constrain(depth, 0.0f, 1.0f);
}

FLASHMEM void AudioTapeworm::setCrackle(float density) {
  density = constrain(density, 0.0f, 1.0f);
  crk_prob_ = density * density * (60.0f / AUDIO_SAMPLE_RATE_EXACT);
}

// ---- small DSP helpers -----------------------------------------------------

FLASHMEM float AudioTapeworm::Sine(float x) {
  // fold [-pi, pi] then 9th-order Taylor (exact to ~1e-6 for filter use)
  const float kPI = 3.14159265f;
  const float kTWO_PI = 6.28318531f;
  while (x > kPI) x -= kTWO_PI;
  while (x < -kPI) x += kTWO_PI;
  const bool neg = x < 0.0f;
  if (neg) x = -x;
  if (x > 1.57079633f) x = kPI - x;
  const float x2 = x * x;
  float s = x * (1.0f + x2 * (-1.0f / 6.0f
    + x2 * (1.0f / 120.0f
    + x2 * (-1.0f / 5040.0f
    + x2 * (1.0f / 362880.0f)))));
  return neg ? -s : s;
}

FLASHMEM float AudioTapeworm::Saturate(float x) {
  // tanh(x) = 1 - 2/(e^2x + 1) via fastexp (MidSider table math, no table)
  if (x > 4.0f) return 1.0f;
  if (x < -4.0f) return -1.0f;
  return 1.0f - 2.0f / (fastexp(2.0f * x) + 1.0f);
}

FLASHMEM float AudioTapeworm::WhiteNoise(uint32_t& state) {
  // xorshift32 -> uniform [-1, 1)
  uint32_t s = state;
  s ^= s << 13;
  s ^= s >> 17;
  s ^= s << 5;
  state = s;
  return (int32_t)s * (1.0f / 2147483648.0f);
}

// ---- audio ISR -------------------------------------------------------------

FLASHMEM void AudioTapeworm::update(void) {
  audio_block_t* inl = receiveReadOnly(0);
  audio_block_t* inr = receiveReadOnly(1);
  audio_block_t* outl = allocate();
  audio_block_t* outr = allocate();

  if (!outl || !outr) {
    if (inl) release(inl);
    if (inr) release(inr);
    if (outl) release(outl);
    if (outr) release(outr);
    return;
  }

  int16_t* b = buf_;
  const uint32_t cap = cap_frames_;

  if (!b || cap < 2) {
    // no buffer: wet path is silent (dry leg lives outside this object)
    memset(outl->data, 0, AUDIO_BLOCK_SAMPLES * sizeof(int16_t));
    memset(outr->data, 0, AUDIO_BLOCK_SAMPLES * sizeof(int16_t));
  } else {
    // snapshot the control-written group once per block
    const float speed = speed_;
    const float w1 = w1_, w2 = w2_;
    const float wow_amp = wow_amp_;
    const float sp1 = sp1_, sp2 = sp2_;
    const float flut_inc = flut_inc_, flut_amp = flut_amp_;
    const float rum_amp = rum_amp_;
    const float spread_mix = spread_mix_;
    const float age = age_, hf_a = age_hf_a_;
    const float pre_gain = pre_gain_;
    const float drop_prob = drop_rate_, drop_depth = drop_depth_;
    const float crk_prob = crk_prob_;

    const float max_delay = (float)cap - MIN_DELAY;
    uint32_t w = write_ix_;
    float dl = delay_l_, dr = delay_r_;
    float p1 = ph1_, p2 = ph2_;
    float fll = fl_l_, flr = fl_r_, ru = ru_l_;
    float warml = warm_l_, warmr = warm_r_;
    float hfl = hf_l_, hfr = hf_r_;
    float hf2l = hf2_l_, hf2r = hf2_r_;
    float compl_ = comp_l_, compr = comp_r_;
    float drop = drop_env_;
    float crl = crk_l_, crr = crk_r_;
    uint32_t rng = rng_;
    uint16_t pk = peak_;

    const float rum_alpha = 1.963e-4f; // one-pole at 1.5 Hz
    const float warm_alpha = 3.224e-2f; // one-pole at 250 Hz
    const float crk_decay = 0.993f;
    const float inv32768 = 1.0f / 32768.0f;

    for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
      const float xl = (inl ? inl->data[i] : 0) * inv32768;
      const float xr = (inr ? inr->data[i] : 0) * inv32768;

      // ---- warble generators -> rate modulation (per sample) --------------
      p1 += w1;
      p2 += w2;
      if (p1 > 6.28318531f) p1 -= 6.28318531f;
      if (p2 > 6.28318531f) p2 -= 6.28318531f;
      const float wow_l = 0.62f * Sine(p1) + 0.38f * Sine(p2);
      const float wow_r = 0.62f * Sine(p1 + sp1) + 0.38f * Sine(p2 + sp2);

      fll += flut_inc * (WhiteNoise(rng) - fll);
      flr += flut_inc * (WhiteNoise(rng) - flr);
      const float flut_r = spread_mix * fll + (1.0f - spread_mix) * flr;

      ru += rum_alpha * (WhiteNoise(rng) - ru);

      const float rate_l = speed * (1.0f + wow_amp * wow_l
        + flut_amp * fll + rum_amp * ru);
      const float rate_r = speed * (1.0f + wow_amp * wow_r
        + flut_amp * flut_r + rum_amp * ru);

      // ---- tape transport: delay integration + capstan servo --------------
      dl += (1.0f - rate_l) + (NOMINAL_DELAY - dl) * SERVO_K;
      dr += (1.0f - rate_r) + (NOMINAL_DELAY - dr) * SERVO_K;
      if (dl < MIN_DELAY) dl = MIN_DELAY;
      else if (dl > max_delay) dl = max_delay;
      if (dr < MIN_DELAY) dr = MIN_DELAY;
      else if (dr > max_delay) dr = max_delay;

      // ---- record head: tanh saturation into the tape --------------------
      const int16_t wl = Clip16(Saturate(xl * pre_gain) * 32767.0f);
      const int16_t wr = Clip16(Saturate(xr * pre_gain) * 32767.0f);
      b[2 * w] = wl;
      b[2 * w + 1] = wr;

      // ---- play head: fractional read, linear interpolation --------------
      float rpl = (float)w - dl;
      float rpr = (float)w - dr;
      while (rpl < 0.0f) rpl += (float)cap;
      while (rpr < 0.0f) rpr += (float)cap;
      const uint32_t i0l = (uint32_t)rpl, i0r = (uint32_t)rpr;
      uint32_t i1l = i0l + 1, i1r = i0r + 1;
      if (i1l >= cap) i1l = 0;
      if (i1r >= cap) i1r = 0;
      const float fl_ = rpl - (float)i0l;
      const float fr_ = rpr - (float)i0r;
      float yl = (b[2 * i0l] + fl_ * (float)(b[2 * i1l] - b[2 * i0l])) * inv32768;
      float yr = (b[2 * i0r] + fr_ * (float)(b[2 * i1r] - b[2 * i0r])) * inv32768;

      if (++w >= cap) w = 0;

      // ---- age: tilt, HF loss, saturation, hiss, soft compression --------
      warml += warm_alpha * (yl - warml);
      warmr += warm_alpha * (yr - warmr);
      yl += age * 0.45f * warml;
      yr += age * 0.45f * warmr;

      hfl += hf_a * (yl - hfl); // 2-pole cascade: pole 1 -> pole 2
      hfr += hf_a * (yr - hfr);
      hf2l += hf_a * (hfl - hf2l);
      hf2r += hf_a * (hfr - hf2r);
      yl = hf2l;
      yr = hf2r;

      yl = Saturate(yl * (1.0f + age * 1.2f));
      yr = Saturate(yr * (1.0f + age * 1.2f));

      const float ay = fabsf(yl);
      const float ar = fabsf(yr);
      compl_ += (ay > compl_ ? 0.001f : 0.0001f) * (ay - compl_);
      compr += (ar > compr ? 0.001f : 0.0001f) * (ar - compr);
      yl *= 1.0f / (1.0f + age * 0.8f * compl_);
      yr *= 1.0f / (1.0f + age * 0.8f * compr);

      if (age > 0.0f) {
        yl += age * age * 0.02f * WhiteNoise(rng);
        yr += age * age * 0.02f * WhiteNoise(rng);
      }

      // ---- dropout: random amplitude dips (shared L/R) -------------------
      float target = 1.0f;
      if (drop_prob > 0.0f && WhiteNoise(rng) < drop_prob * 2.0f - 1.0f) {
        target = 1.0f - drop_depth * (0.4f + 0.6f * (WhiteNoise(rng) + 1.0f) * 0.5f);
      }
      drop += (target - drop) * (target < drop ? 0.02f : 0.002f);
      yl *= drop;
      yr *= drop;

      // ---- crackle: random impulses through a one-pole decay -------------
      if (crk_prob > 0.0f) {
        if (WhiteNoise(rng) < crk_prob * 2.0f - 1.0f)
          crl += WhiteNoise(rng) * 0.8f;
        if (WhiteNoise(rng) < crk_prob * 2.0f - 1.0f)
          crr += WhiteNoise(rng) * 0.8f;
      }
      crl *= crk_decay;
      crr *= crk_decay;
      yl += crl;
      yr += crr;

      // ---- playback soft clip + output ----------------------------------
      yl = Saturate(yl);
      yr = Saturate(yr);

      const int16_t ol = Clip16(yl * 32767.0f);
      const int16_t or_ = Clip16(yr * 32767.0f);
      outl->data[i] = ol;
      outr->data[i] = or_;

      const uint16_t pol = (uint16_t)(ol < 0 ? -ol : ol);
      const uint16_t por = (uint16_t)(or_ < 0 ? -or_ : or_);
      if (pol > pk) pk = pol;
      if (por > pk) pk = por;
    }

    write_ix_ = w;
    delay_l_ = dl;
    delay_r_ = dr;
    ph1_ = p1;
    ph2_ = p2;
    fl_l_ = fll;
    fl_r_ = flr;
    ru_l_ = ru;
    warm_l_ = warml;
    warm_r_ = warmr;
    hf_l_ = hfl;
    hf_r_ = hfr;
    hf2_l_ = hf2l;
    hf2_r_ = hf2r;
    comp_l_ = compl_;
    comp_r_ = compr;
    drop_env_ = drop;
    crk_l_ = crl;
    crk_r_ = crr;
    rng_ = rng;
    peak_ = pk;
  }

  transmit(outl, 0);
  transmit(outr, 1);
  if (inl) release(inl);
  if (inr) release(inr);
  release(outl);
  release(outr);
}

// ---------------------------------------------------------------------------
// TapewormCore - applet UI / CV mapping / state (non-template, FLASHMEM)
// ---------------------------------------------------------------------------
class TapewormCore : public HemisphereAudioApplet {
public:
  // (applet_name is macro-rewritten by _config.h into a static accessor)
  const char* applet_name() {
    return TAPEWORM_NAME;
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

  AudioStream* InputStream() override;
  AudioStream* OutputStream() override;

protected:
  void SetHelp() override;

private:
  enum Cursor : int8_t {
    // Page 0: transport / warble
    SPEED = 0, SPEED_CV,
    WOW_RATE, WOW_DEPTH, WOW_CV,
    FLUT_RATE, FLUT_DEPTH, FLUT_CV,
    SPREAD,
    // Page 1: wear
    AGE, AGE_CV,
    DRIVE, DRIVE_CV,
    DROP_RATE, DROP_DEPTH, DROP_CV,
    CRACKLE, CRACKLE_CV,
    RUMBLE,
    // Page 2: output / momentary transport
    MIX, MIX_CV,
    DFAST, DSLOW,
    CURSOR_LENGTH,
  };

  // ---- params (state round-trip via OnDataRequest/Receive) ----------------
  uint8_t speed = 64;       // 0..127 -> 0.5x..2x, 64 == 1x
  uint8_t wow_rate = 52;    // 0..127 -> 0.1..5 Hz log
  uint8_t wow_depth = 20;   // 0..100%
  uint8_t flut_rate = 16;   // 0..31 -> 4..30 Hz log
  uint8_t flut_depth = 15;  // 0..100%
  uint8_t spread = 40;      // 0..100% L/R warble decorrelation
  uint8_t age = 30;         // 0..100%
  uint8_t drive = 6;        // 0..24 dB
  uint8_t drop_rate = 0;    // 0..100%
  uint8_t drop_depth = 60;  // 0..100%
  uint8_t crackle = 0;      // 0..100%
  uint8_t rumble = 20;      // 0..100%
  uint8_t mix = 50;         // 0..100% dry..wet

  CVInputMap spd_cv;
  CVInputMap wow_cv;
  CVInputMap flut_cv;
  CVInputMap age_cv;
  CVInputMap drive_cv;
  CVInputMap drop_cv;
  CVInputMap crackle_cv;
  CVInputMap mix_cv;

  DigitalInputMap fast_gate; // held high: transport x2 (momentary +)
  DigitalInputMap slow_gate; // held high: transport x0.5 (momentary -)

  int8_t cursor = SPEED;
  bool alloc_ok = false;

  AudioPassthrough<2> input;
  AudioTapeworm tape;
  AudioMixer<2> mix_l;
  AudioMixer<2> mix_r;
  AudioPassthrough<2> output;

  // 0..127 -> 0.5x..2.0x (exactly 1.0x at 64), log
  static float SpeedFromUnits(int units);
  // 0..127 -> 0.1..5 Hz, log
  static float WowRateFromUnits(int units);
  // 0..31 -> 4..30 Hz, log
  static float FlutRateFromUnits(int units);
  void ApplyTransport();
};

// Thin template wrapper so the registry can use Tapeworm<STEREO> form.
// (All real work lives in the non-template core so FLASHMEM section
// placement survives this toolchain; Channels is intentionally unused and
// the wrapper overrides nothing, so it shares the core's vtable layout.)
template <AudioChannels Channels>
class Tapeworm : public TapewormCore {
};

// -- Out-of-line definitions (strong symbols, FLASHMEM honored) --------------

FLASHMEM float TapewormCore::SpeedFromUnits(int units) {
  // two-segment log ramp: 0 -> 0.5x, 64 -> 1x, 127 -> ~2x
  const int u = constrain(units, 0, 127);
  const float seg = (u < 64) ? (1.0f / 64.0f) : (1.0f / 63.0f);
  return fastpow2((u - 64) * seg);
}

FLASHMEM float TapewormCore::WowRateFromUnits(int units) {
  return 0.1f * fastpow2(constrain(units, 0, 127) * (5.643856f / 127.0f));
}

FLASHMEM float TapewormCore::FlutRateFromUnits(int units) {
  return 4.0f * fastpow2(constrain(units, 0, 31) * (2.906891f / 31.0f));
}

FLASHMEM void TapewormCore::Start() {
  alloc_ok = tape.begin();
  if (!alloc_ok) return;

  // Sensible input-map defaults (preserved when a map was already assigned)
  if (!fast_gate.enabled()) fast_gate.SetGateInput(0); // Trig1
  if (!slow_gate.enabled()) slow_gate.SetGateInput(1); // Trig2

  PatchCable(input, 0, tape, 0);
  PatchCable(input, 1, tape, 1);
  PatchCable(input, 0, mix_l, 0); // dry
  PatchCable(input, 1, mix_r, 0);
  PatchCable(tape, 0, mix_l, 1); // wet
  PatchCable(tape, 1, mix_r, 1);
  PatchCable(mix_l, 0, output, 0);
  PatchCable(mix_r, 0, output, 1);
  // 8 cables total (<= 32)
}

FLASHMEM void TapewormCore::Unload() {
  tape.end();
  alloc_ok = false;
  AllowRestart();
}

FLASHMEM void TapewormCore::mainloop() {
  if (!alloc_ok) return;
  tape.decayPeak();
}

FLASHMEM AudioStream* TapewormCore::InputStream() {
  return &input;
}

FLASHMEM AudioStream* TapewormCore::OutputStream() {
  return &output;
}

FLASHMEM void TapewormCore::ApplyTransport() {
  // momentary +/- gates: Trig1 held = x2, Trig2 held = x0.5, both = x1
  const bool fast = fast_gate.Gate();
  const bool slow = slow_gate.Gate();
  const float moment = (fast ? 2.0f : 1.0f) * (slow ? 0.5f : 1.0f);

  // Locals first: the constrain() macro evaluates its argument repeatedly.
  const float eff_spd
    = SpeedFromUnits(speed) * fastpow2(spd_cv.InF()) * moment;
  tape.setSpeed(constrain(eff_spd, 0.25f, 4.0f));

  const float eff_wow_d
    = constrain(wow_depth + wow_cv.InF() * 50.0f, 0.0f, 100.0f);
  tape.setWow(WowRateFromUnits(wow_rate), eff_wow_d * 0.01f);

  const float eff_fl_d
    = constrain(flut_depth + flut_cv.InF() * 50.0f, 0.0f, 100.0f);
  tape.setFlutter(FlutRateFromUnits(flut_rate), eff_fl_d * 0.01f);

  tape.setRumble(rumble * 0.01f);
  tape.setSpread(spread * 0.01f);

  const float eff_age = constrain(age + age_cv.InF() * 50.0f, 0.0f, 100.0f);
  tape.setAge(eff_age * 0.01f);

  const float eff_drv = constrain(drive + drive_cv.InF() * 12.0f, 0.0f, 24.0f);
  tape.setDrive(eff_drv);

  const float eff_dr
    = constrain(drop_rate + drop_cv.InF() * 50.0f, 0.0f, 100.0f);
  tape.setDropout(eff_dr * 0.01f, drop_depth * 0.01f);

  const float eff_crk
    = constrain(crackle + crackle_cv.InF() * 50.0f, 0.0f, 100.0f);
  tape.setCrackle(eff_crk * 0.01f);

  // equal-power dry/wet mix (block-rate gains, ClockLoop pattern)
  float dry_gain, wet_gain;
  const float m = constrain(mix + mix_cv.InF() * 50.0f, 0.0f, 100.0f) * 0.01f;
  EqualPowerFade(dry_gain, wet_gain, m);
  mix_l.gain(0, dry_gain);
  mix_l.gain(1, wet_gain);
  mix_r.gain(0, dry_gain);
  mix_r.gain(1, wet_gain);
}

// 16.6 kHz control tick: CV folding + parameter pushes (all atomic stores).
FLASHMEM void TapewormCore::Controller() {
  if (!alloc_ok) return;
  ApplyTransport();
}

// --- UI --------------------------------------------------------------------

FLASHMEM void TapewormCore::View() {
  if (!alloc_ok) {
    gfxPrint(2, 15, PSTR("Out of RAM!!"));
    return;
  }

  const int8_t page = (cursor >= AGE) + (cursor >= MIX);
  gfxRect(page * 15, 10, 14, 3); // page tab

  if (page == 0) {
    // ---- transport / warble ----
    gfxPrint(1, 15, PSTR("Spd"));
    gfxStartCursor();
    gfxPrint(SpeedFromUnits(speed), 2);
    gfxEndCursor(cursor == SPEED);
    gfxStartCursor();
    gfxPrint(spd_cv);
    gfxEndCursor(cursor == SPEED_CV, false, spd_cv.InputName());

    gfxPrint(1, 25, PSTR("Wr:"));
    gfxStartCursor();
    gfxPrint(WowRateFromUnits(wow_rate), 1);
    gfxEndCursor(cursor == WOW_RATE);
    gfxStartCursor();
    graphics.printf(PSTR("%3d"), wow_depth);
    gfxEndCursor(cursor == WOW_DEPTH);

    gfxPrint(1, 35, PSTR("Wc:"));
    gfxStartCursor();
    gfxPrint(wow_cv);
    gfxEndCursor(cursor == WOW_CV, false, wow_cv.InputName());
    gfxStartCursor();
    gfxPrint(FlutRateFromUnits(flut_rate), 1);
    gfxEndCursor(cursor == FLUT_RATE);

    gfxPrint(1, 45, PSTR("Flt"));
    gfxStartCursor();
    graphics.printf(PSTR("%3d"), flut_depth);
    gfxEndCursor(cursor == FLUT_DEPTH);
    gfxStartCursor();
    gfxPrint(flut_cv);
    gfxEndCursor(cursor == FLUT_CV, false, flut_cv.InputName());

    gfxPrint(1, 55, PSTR("Spr"));
    gfxStartCursor();
    graphics.printf(PSTR("%3d"), spread);
    gfxEndCursor(cursor == SPREAD);
  } else if (page == 1) {
    // ---- wear ----
    gfxPrint(1, 15, PSTR("Age"));
    gfxStartCursor();
    graphics.printf(PSTR("%3d"), age);
    gfxEndCursor(cursor == AGE);
    gfxStartCursor();
    gfxPrint(age_cv);
    gfxEndCursor(cursor == AGE_CV, false, age_cv.InputName());

    gfxPrint(1, 25, PSTR("Drv"));
    gfxStartCursor();
    graphics.printf(PSTR("%2ddB"), drive);
    gfxEndCursor(cursor == DRIVE);
    gfxStartCursor();
    gfxPrint(drive_cv);
    gfxEndCursor(cursor == DRIVE_CV, false, drive_cv.InputName());

    gfxPrint(1, 35, PSTR("Dr:"));
    gfxStartCursor();
    graphics.printf(PSTR("%3d"), drop_rate);
    gfxEndCursor(cursor == DROP_RATE);
    gfxStartCursor();
    graphics.printf(PSTR("%3d"), drop_depth);
    gfxEndCursor(cursor == DROP_DEPTH);

    gfxPrint(1, 45, PSTR("Dc:"));
    gfxStartCursor();
    gfxPrint(drop_cv);
    gfxEndCursor(cursor == DROP_CV, false, drop_cv.InputName());
    gfxStartCursor();
    graphics.printf(PSTR("%3d"), crackle);
    gfxEndCursor(cursor == CRACKLE);

    gfxPrint(1, 55, PSTR("Cc:"));
    gfxStartCursor();
    gfxPrint(crackle_cv);
    gfxEndCursor(cursor == CRACKLE_CV, false, crackle_cv.InputName());
    gfxStartCursor();
    graphics.printf(PSTR("%3d"), rumble);
    gfxEndCursor(cursor == RUMBLE);
  } else {
    // ---- output / momentary transport ----
    gfxPrint(1, 15, PSTR("Mix"));
    gfxStartCursor();
    graphics.printf(PSTR("%3d"), mix);
    gfxEndCursor(cursor == MIX);
    gfxStartCursor();
    gfxPrint(mix_cv);
    gfxEndCursor(cursor == MIX_CV, false, mix_cv.InputName());

    gfxPrint(1, 25, PSTR("Fst:"));
    gfxStartCursor();
    gfxPrint(fast_gate);
    gfxEndCursor(cursor == DFAST, false, fast_gate.InputName());

    gfxPrint(1, 35, PSTR("Slw:"));
    gfxStartCursor();
    gfxPrint(slow_gate);
    gfxEndCursor(cursor == DSLOW, false, slow_gate.InputName());

    // wet peak meter + buffer note
    gfxPrint(1, 45, PSTR("Pk:"));
    gfxInvert(20, 45, Proportion((int)tape.peak(), 32767, 20), 7);
    if (!tape.IsPSRAM()) {
      gfxPos(44, 45);
      gfxPrint(PSTR("RAM2"));
    }
    gfxPrint(1, 55, PSTR("Wow->Spd"));
  }

  gfxDisplayInputMapEditor();
}

FLASHMEM void TapewormCore::DrawFullScreen() {
  if (!alloc_ok) return;
  const int x0 = 64 - gfx_offset;

  // transport speed indicator (tick = current rate around center)
  graphics.drawLine(x0, 22, 127 - gfx_offset, 22, 1);
  const float ratio = SpeedFromUnits(speed) * fastpow2(spd_cv.InF());
  const int bx = x0 + 32 + (int)(constrain(ratio - 1.0f, -1.0f, 1.0f) * 30.0f);
  graphics.drawLine(bx, 18, bx, 26, 3);

  // wet peak meter (32k == 30px)
  const int h = Proportion((int)tape.peak(), 32767, 30);
  graphics.drawFrame(x0 + 15, 28, 10, 30);
  if (h > 0) graphics.drawRect(x0 + 16, 57 - h, 8, h);

  graphics.drawLine(x0, 26, 127 - gfx_offset, 26, 3);
  View();
}

FLASHMEM void TapewormCore::OnButtonPress() {
  // CV rows open the attenuversion editor; the momentary gate rows use the
  // encoder directly (ChangeSource) so they are excluded here.
  if (CheckEditInputMapPress(
        cursor,
        IndexedInput(SPEED_CV, spd_cv),
        IndexedInput(WOW_CV, wow_cv),
        IndexedInput(FLUT_CV, flut_cv),
        IndexedInput(AGE_CV, age_cv),
        IndexedInput(DRIVE_CV, drive_cv),
        IndexedInput(DROP_CV, drop_cv),
        IndexedInput(CRACKLE_CV, crackle_cv),
        IndexedInput(MIX_CV, mix_cv)
      ))
    return;
  CursorToggle();
}

FLASHMEM void TapewormCore::OnEncoderMove(int direction) {
  if (!EditMode()) {
    MoveCursor(cursor, direction, CURSOR_LENGTH - 1);
    return;
  }
  if (EditSelectedInputMap(direction)) return;

  switch (cursor) {
    case SPEED: speed = constrain(speed + direction, 0, 127); break;
    case SPEED_CV: spd_cv.ChangeSource(direction); break;
    case WOW_RATE: wow_rate = constrain(wow_rate + direction, 0, 127); break;
    case WOW_DEPTH: wow_depth = constrain(wow_depth + direction, 0, 100); break;
    case WOW_CV: wow_cv.ChangeSource(direction); break;
    case FLUT_RATE: flut_rate = constrain(flut_rate + direction, 0, 31); break;
    case FLUT_DEPTH: flut_depth = constrain(flut_depth + direction, 0, 100); break;
    case FLUT_CV: flut_cv.ChangeSource(direction); break;
    case SPREAD: spread = constrain(spread + direction, 0, 100); break;

    case AGE: age = constrain(age + direction, 0, 100); break;
    case AGE_CV: age_cv.ChangeSource(direction); break;
    case DRIVE: drive = constrain(drive + direction, 0, 24); break;
    case DRIVE_CV: drive_cv.ChangeSource(direction); break;
    case DROP_RATE: drop_rate = constrain(drop_rate + direction, 0, 100); break;
    case DROP_DEPTH: drop_depth = constrain(drop_depth + direction, 0, 100); break;
    case DROP_CV: drop_cv.ChangeSource(direction); break;
    case CRACKLE: crackle = constrain(crackle + direction, 0, 100); break;
    case CRACKLE_CV: crackle_cv.ChangeSource(direction); break;
    case RUMBLE: rumble = constrain(rumble + direction, 0, 100); break;

    case MIX: mix = constrain(mix + direction, 0, 100); break;
    case MIX_CV: mix_cv.ChangeSource(direction); break;
    case DFAST: fast_gate.ChangeSource(direction); break;
    case DSLOW: slow_gate.ChangeSource(direction); break;

    default: return;
  }
}

// 256-bit state layout (4 x 64-bit words):
//   word 0 (61b): speed(7) wow_rate(7) wow_depth(7) flut_rate(5)
//                 flut_depth(7) age(7) drop_rate(7) drop_depth(7) crackle(7)
//   word 1 (58b): rumble(7) drive(5) mix(7) spread(7)
//                 fast_src(8) slow_src(8) spd_cv(16)
//   word 2 (64b): wow_cv, flut_cv, age_cv, drive_cv (4 x 16)
//   word 3 (48b): drop_cv, crackle_cv, mix_cv (3 x 16)
// 231 bits used, 25 spare.  Digital maps persist only their source selector
// (momentary gates ignore div/euclidean state; Unpack() re-defaults it).
FLASHMEM void TapewormCore::OnDataRequest(std::array<uint64_t, CONFIG_SIZE>& data) {
  uint8_t fast_src = (uint8_t)(fast_gate.Pack() & 0xFF);
  uint8_t slow_src = (uint8_t)(slow_gate.Pack() & 0xFF);
  data[0] = PackPackables(
    pack<7>(speed),
    pack<7>(wow_rate),
    pack<7>(wow_depth),
    pack<5>(flut_rate),
    pack<7>(flut_depth),
    pack<7>(age),
    pack<7>(drop_rate),
    pack<7>(drop_depth),
    pack<7>(crackle)
  );
  data[1] = PackPackables(
    pack<7>(rumble),
    pack<5>(drive),
    pack<7>(mix),
    pack<7>(spread),
    pack<8>(fast_src),
    pack<8>(slow_src),
    spd_cv
  );
  data[2] = PackPackables(wow_cv, flut_cv, age_cv, drive_cv);
  data[3] = PackPackables(drop_cv, crackle_cv, mix_cv);
}

FLASHMEM void TapewormCore::OnDataReceive(
  const std::array<uint64_t, CONFIG_SIZE>& data
) {
  uint8_t fast_src = 0, slow_src = 0;
  UnpackPackables(
    data[0],
    pack<7>(speed),
    pack<7>(wow_rate),
    pack<7>(wow_depth),
    pack<5>(flut_rate),
    pack<7>(flut_depth),
    pack<7>(age),
    pack<7>(drop_rate),
    pack<7>(drop_depth),
    pack<7>(crackle)
  );
  UnpackPackables(
    data[1],
    pack<7>(rumble),
    pack<5>(drive),
    pack<7>(mix),
    pack<7>(spread),
    pack<8>(fast_src),
    pack<8>(slow_src),
    spd_cv
  );
  UnpackPackables(data[2], wow_cv, flut_cv, age_cv, drive_cv);
  UnpackPackables(data[3], drop_cv, crackle_cv, mix_cv);

  fast_gate.Unpack(fast_src);
  slow_gate.Unpack(slow_src);

  // re-derive packed ranges defensively
  speed = constrain(speed, 0, 127);
  wow_rate = constrain(wow_rate, 0, 127);
  wow_depth = constrain(wow_depth, 0, 100);
  flut_rate = constrain(flut_rate, 0, 31);
  flut_depth = constrain(flut_depth, 0, 100);
  age = constrain(age, 0, 100);
  drop_rate = constrain(drop_rate, 0, 100);
  drop_depth = constrain(drop_depth, 0, 100);
  crackle = constrain(crackle, 0, 100);
  rumble = constrain(rumble, 0, 100);
  drive = constrain(drive, 0, 24);
  mix = constrain(mix, 0, 100);
  spread = constrain(spread, 0, 100);
}

FLASHMEM void TapewormCore::SetHelp() {
  help[HELP_DIGITAL1] = PSTR("Fast x2");
  help[HELP_DIGITAL2] = PSTR("Slow x.5");
  help[HELP_CV1]      = PSTR("Row CVs");
  help[HELP_CV2]      = PSTR("Row CVs");
  help[HELP_OUT1]     = PSTR("Out L");
  help[HELP_OUT2]     = PSTR("Out R");
  help[HELP_EXTRA1]   = PSTR("Tape/vinyl warp");
  help[HELP_EXTRA2]   = PSTR("wow flut age");
}
