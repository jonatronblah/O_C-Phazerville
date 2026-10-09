#pragma once

#include "../Audio/AudioMixer.h"
#include "../Audio/AudioPassthrough.h"
#include "../dsputils.h"
#include <Audio.h>
#include <smalloc.h> // extmem_calloc / extmem_free

extern "C" uint8_t external_psram_size;

// cos(pi * i/64) for i = 0..64; quarter-wave table for the Hann window.
// Explicit .progmem section: slot0.ld merges .rodata* into RAM1 .data, so
// even const arrays would cost 260 B of RAM1 without this attribute.
static const float kHarmoCosPi[65] __attribute__((section(".progmem"))) = {
  1.00000000f, 0.99879546f, 0.99518473f, 0.98917651f,
  0.98078528f, 0.97003125f, 0.95694034f, 0.94154407f,
  0.92387953f, 0.90398929f, 0.88192126f, 0.85772861f,
  0.83146961f, 0.80320753f, 0.77301045f, 0.74095113f,
  0.70710678f, 0.67155895f, 0.63439328f, 0.59569930f,
  0.55557023f, 0.51410274f, 0.47139674f, 0.42755509f,
  0.38268343f, 0.33688985f, 0.29028468f, 0.24298018f,
  0.19509032f, 0.14673047f, 0.09801714f, 0.04906767f,
  0.00000000f, -0.04906767f, -0.09801714f, -0.14673047f,
  -0.19509032f, -0.24298018f, -0.29028468f, -0.33688985f,
  -0.38268343f, -0.42755509f, -0.47139674f, -0.51410274f,
  -0.55557023f, -0.59569930f, -0.63439328f, -0.67155895f,
  -0.70710678f, -0.74095113f, -0.77301045f, -0.80320753f,
  -0.83146961f, -0.85772861f, -0.88192126f, -0.90398929f,
  -0.92387953f, -0.94154407f, -0.95694034f, -0.97003125f,
  -0.98078528f, -0.98917651f, -0.99518473f, -0.99879546f,
  -1.00000000f,
};

// Grain length options in samples (.progmem -> flash, see above)
static const int kHarmoGrainLen[4] __attribute__((section(".progmem"))) = {256, 512, 1024, 2048};

// Harmo - dual-voice CV harmonizer with pitch-shift feedback.
//
// Two harmony voices are pitch-shifted from the mono (L) input; voice 1 is
// panned-ish to L and voice 2 to R (or both centered), mixed with dry.
// Each voice shifts -12..+12 semitones: quantized intervals (semitone steps)
// or free cents, with a CV input per voice (the "interval as CV" wish).
//
// PITCH ENGINE: (c) 2-tap-granular Hann-window overlap-add (the documented
// choice of the three candidates):
//   - AudioPlaySdResmp (b) is SD-file playback and cannot resample live
//     input, so it is not adaptable here.
//   - Variable read-rate + splice crossfade (a, effect_looper's fractional
//     read) pitch-shifts by changing the delay-line read speed, which drags
//     the read head across the buffer and needs periodic resplices -
//     glitchy on big intervals (documented in AudioDelayExt::ReadStretch*).
//   - The granular engine below: input history lives in a mono ring buffer
//     (8192 samples, PSRAM when available, RAM2 fallback). Each voice owns
//     4 grain slots; every Hop output samples a slot restarts a Hann-
//     windowed grain whose read head advances at ratio r = 2^(cents/1200)
//     per output sample. Grains overlap-add at 50/67/75% overlap with
//     2*hop/grain gain normalization (periodic Hann sums to a constant at
//     these spacings). Window = 0.5 - 0.5*cos(2*pi*age/grain), evaluated
//     from a 65-point quarter-wave table (no libm trig in the audio path).
//     Linear interpolation on the fractional read. Typical artifacts:
//     mild phasiness / periodicity at extreme grain sizes, spectral
//     smearing on transients; grain 512 @ 50% overlap is the sweet spot.
//
// FEEDBACK PITCH PATH (Echobode-style): a switchable source (voice 1, voice
//   2, or both) is scaled by the Feedback amount and summed back into the
//   ring-buffer write. Every loop around the buffer passes through the
//   granular reader again, so each repeat is shifted by that voice's
//   interval. Because grains always restart behind the write head, the
//   regenerating tail stays bounded in time; decays by the feedback gain
//   per loop (~grain delay), comb-filtering is inherent to the mode.
//
// FORMANT TILT: simple 1-pole shelf on the wet voices (negative = blend to
//   lowpass, positive = high-shelf lift) - a cheap "formant-ish" tilt, not
//   a true formant filter.
//
// MEMORY LAYOUT NOTE (from the MidSider/effect_looper lessons): the
//   multiboot slot0.ld linker script runs default .text in ITCM/RAM1 at
//   32 KB bank granularity, and GCC silently drops section attributes on
//   template/inline members. So ALL state and heavy code lives in the
//   non-template HarmoPitch / HarmoCore classes below with out-of-line
//   FLASHMEM definitions (strong symbols). Even the audio ISR update() runs
//   from flash (I-cache-hot at the block rate). Harmo<Channels> is the thin
//   registered template wrapper (Channels intentionally unused, stereo I/O
//   always: mono in -> V1 + V2 + dry stereo out).
//
// Registration: `DeclareFancyApplet<Harmo<STEREO>>` (single, like MidSider;
// no MONO dual registration - the engine is mono-in/stereo-out by design).

// ---------------------------------------------------------------------------
// HarmoPitch - custom AudioStream: 2-voice granular pitch shifter with
// feedback injection and tilt. Non-template; every method out-of-line
// FLASHMEM below so section placement survives the toolchain.
// ---------------------------------------------------------------------------
class HarmoPitch : public AudioStream {
public:
  HarmoPitch();

  // ---- buffer lifecycle (Start() / Unload()) ----
  bool begin();
  void end();
  bool IsReady() const;
  bool IsPSRAM() const;

  // ---- control-context setters (atomic 32-bit stores, read by update()) ----
  void setRatio(int voice, float r); // 0.5 .. 2.0 (== -12 .. +12 st)
  void setGrain(int grain, int hop); // samples; hop = grain/(2..4)
  void setFeedback(float amt, int src); // amt 0..~0.95, src 0=V1 1=V2 2=both
  void setTilt(float t); // -1.0 .. 1.0

  virtual void update(void);

private:
  static const uint32_t RING_N = 8192; // power of two; 16 KB int16 history
  static const int GRAIN_SLOTS = 4; // covers 75% overlap (hop = grain/4)

  struct Grain {
    float d; // distance behind the write head (fractional samples)
    uint16_t age; // output-domain age, 0..grain
  };

  static float HannFromPhase(float phase01);

  audio_block_t* inputQueueArray[1];
  // -- written by control context, read by update() -------------------------
  int16_t* volatile ring_ = nullptr;
  volatile float ratio_[2] = {1.0f, 1.0f};
  volatile int32_t grain_len_ = 1024;
  volatile int32_t hop_ = 512;
  volatile float fb_amt_ = 0.0f;
  volatile int32_t fb_src_ = 0;
  volatile float tilt_ = 0.0f;
  bool in_psram_ = false;

  // -- ISR-private ---------------------------------------------------------
  uint32_t w_ = 0; // ring write index
  Grain gr_[2][GRAIN_SLOTS];
  int32_t spawn_cnt_[2] = {0, 0};
  int32_t spawn_ix_[2] = {0, 0};
  float lp_[2] = {0.0f, 0.0f};
  int32_t last_g_ = -1;
};

// ---------------------------------------------------------------------------
// HarmoCore - applet state, UI, graph wiring (all out-of-line FLASHMEM).
// ---------------------------------------------------------------------------
class HarmoCore : public HemisphereAudioApplet {
public:
  void Start() final;
  void Unload() final;
  void Controller() final;
  void View() final;
  void OnButtonPress() override;
  void OnEncoderMove(int direction) override;
  void OnDataRequest(std::array<uint64_t, CONFIG_SIZE>& data) override;
  void OnDataReceive(const std::array<uint64_t, CONFIG_SIZE>& data) override;
  void SetParams();

  AudioStream* InputStream() override;
  AudioStream* OutputStream() override;

protected:
  void SetHelp() override;

private:
  enum Cursor : int8_t {
    // Page 0: Voice 1
    V1_INT = 0, V1_INT_CV, V1_LVL, PMODE,
    // Page 1: Voice 2
    V2_INT, V2_INT_CV, V2_LVL, SPREAD,
    // Page 2: Engine
    GRAIN, OVLP, FB, FB_CV, FBSRC,
    // Page 3: Output
    MIX, MIX_CV, TILT,
    CURSOR_LENGTH,
  };

  static constexpr int GRAIN_COUNT = 4;

  int8_t cursor = V1_INT;


  // intervals stored in cents; quantized mode steps/displays whole semitones
  int16_t v1_int = 700; // -1200..1200
  int16_t v2_int = 1200;
  uint8_t v1_lvl = 70; // 0..100
  uint8_t v2_lvl = 70;
  uint8_t pmode = 0; // 0 = quantized semitones, 1 = free cents
  uint8_t grain = 1; // 0..3 -> 256/512/1024/2048 samples
  uint8_t ovlp = 0; // 0/1/2 -> hop grain/2, grain/3, grain/4
  uint8_t fb = 0; // 0..100 -> ~0..0.95
  uint8_t fbsrc = 0; // 0 = V1, 1 = V2, 2 = both
  uint8_t mix = 50; // 0..100 dry/wet
  uint8_t spread = 0; // 0 = wide (V1 L / V2 R), 1 = both centered
  int8_t tilt = 0; // -100..100

  CVInputMap v1_cv;
  CVInputMap v2_cv;
  CVInputMap fb_cv;
  CVInputMap mix_cv;

  bool alloc_ok = false;

  // smoothed effective values (Controller)
  float sm_r[2] = {1.0f, 1.0f};
  float sm_mix = 50.0f;

  AudioPassthrough<2> input;
  HarmoPitch pitch;
  AudioMixer<2> pan_l; // V1, V2 (wide/center spread gains)
  AudioMixer<2> pan_r;
  AudioMixer<2> mix_l; // dry, pan_l
  AudioMixer<2> mix_r;
  AudioPassthrough<2> output;
};

// Thin template wrapper so the registry can use the Harmo<STEREO> form.
// (All real work lives in the non-template classes so FLASHMEM placement
// survives this toolchain; Channels is intentionally unused.)
template <AudioChannels Channels>
class Harmo : public HarmoCore {
public:
  const char* applet_name() { return "Harmo"; }
};

// -- Out-of-line definitions (strong symbols, FLASHMEM honored) --------------

FLASHMEM HarmoPitch::HarmoPitch() : AudioStream(1, inputQueueArray) {}

FLASHMEM AudioStream* HarmoCore::InputStream() {
  return &input;
}

FLASHMEM AudioStream* HarmoCore::OutputStream() {
  return &output;
}

// Quarter-wave cosine: cos(pi*u) for u in [0,1], linear interpolation.
// window = 0.5 - 0.5*cos(2*pi*phase) via u = fold(2*phase).
FLASHMEM float HarmoPitch::HannFromPhase(float phase01) {
  float u = 2.0f * phase01;
  if (u > 1.0f) u = 2.0f - u;
  float fi = u * 64.0f;
  int i = (int)fi;
  if (i > 63) i = 63;
  float t = fi - (float)i;
  float c = kHarmoCosPi[i] + (kHarmoCosPi[i + 1] - kHarmoCosPi[i]) * t;
  return 0.5f - 0.5f * c;
}

FLASHMEM bool HarmoPitch::begin() {
  end();
#ifdef __IMXRT1062__
  if (external_psram_size > 0) {
    ring_ = (int16_t*)extmem_calloc(RING_N, sizeof(int16_t));
    if (ring_) {
      // extmem_calloc does not guarantee zero-fill on this core
      memset((void*)ring_, 0, RING_N * sizeof(int16_t));
      in_psram_ = true;
    }
  }
#endif
  if (!ring_) {
    ring_ = (int16_t*)calloc(RING_N, sizeof(int16_t));
    in_psram_ = false;
  }
  w_ = 0;
  last_g_ = -1;
  for (int v = 0; v < 2; v++) {
    spawn_cnt_[v] = 0;
    spawn_ix_[v] = 0;
    lp_[v] = 0.0f;
    for (int k = 0; k < GRAIN_SLOTS; k++) {
      gr_[v][k].d = 0.0f;
      gr_[v][k].age = 0xffff;
    }
  }
  return ring_ != nullptr;
}

FLASHMEM void HarmoPitch::end() {
  if (ring_) {
#ifdef __IMXRT1062__
    if (in_psram_) extmem_free((void*)ring_);
    else free((void*)ring_);
#else
    free((void*)ring_);
#endif
    ring_ = nullptr;
  }
  in_psram_ = false;
}

FLASHMEM bool HarmoPitch::IsReady() const {
  return ring_ != nullptr;
}

FLASHMEM bool HarmoPitch::IsPSRAM() const {
  return in_psram_;
}

FLASHMEM void HarmoPitch::setRatio(int voice, float r) {
  if (r < 0.5f) r = 0.5f;
  if (r > 2.0f) r = 2.0f;
  ratio_[voice & 1] = r;
}

FLASHMEM void HarmoPitch::setGrain(int grain, int hop) {
  if (grain < 32) grain = 32;
  if (hop < 1) hop = 1;
  if (hop > grain) hop = grain;
  grain_len_ = grain;
  hop_ = hop;
}

FLASHMEM void HarmoPitch::setFeedback(float amt, int src) {
  if (amt < 0.0f) amt = 0.0f;
  if (amt > 0.95f) amt = 0.95f;
  fb_amt_ = amt;
  fb_src_ = src;
}

FLASHMEM void HarmoPitch::setTilt(float t) {
  if (t < -1.0f) t = -1.0f;
  if (t > 1.0f) t = 1.0f;
  tilt_ = t;
}

// Audio ISR block processor. Runs from flash (I-cache-hot at ~375 blocks/s).
FLASHMEM void HarmoPitch::update(void) {
  if (!ring_) return;

  audio_block_t* in_block = receiveWritable();
  if (in_block == nullptr) {
    // Feedback keeps regenerating even with silence on the input (a VCA may
    // withhold blocks entirely), so process zeros rather than skipping.
    in_block = allocate();
    if (in_block == nullptr) return;
    memset(in_block->data, 0, sizeof(in_block->data));
  }

  audio_block_t* out0 = allocate();
  audio_block_t* out1 = allocate();
  if (out0 == nullptr || out1 == nullptr) {
    release(in_block);
    if (out0) release(out0);
    if (out1) release(out1);
    return;
  }

  const int G = grain_len_;
  const int H = hop_;
  if (G != last_g_) {
    // Grain-length change: fade out by retiring all grains (update()-owned
    // state, so it is safe to reset here rather than from control context)
    for (int v = 0; v < 2; v++) {
      for (int k = 0; k < GRAIN_SLOTS; k++) gr_[v][k].age = 0xffff;
      spawn_cnt_[v] = 0;
      spawn_ix_[v] = 0;
    }
    last_g_ = G;
  }

  const float gnorm = 1.0f / (float)G;
  const float ov_gain = 2.0f * (float)H * gnorm;
  const float d_spawn = (float)(G + 256);
  const float r[2] = {ratio_[0], ratio_[1]};
  const float fbk = fb_amt_;
  const int fbs = fb_src_;
  const float tilt = tilt_;

  for (int n = 0; n < AUDIO_BLOCK_SAMPLES; n++) {
    float x = (float)in_block->data[n];
    float voice[2];

    for (int v = 0; v < 2; v++) {
      if (--spawn_cnt_[v] < 0) {
        Grain& g = gr_[v][spawn_ix_[v]];
        g.d = d_spawn;
        g.age = 0;
        spawn_ix_[v] = (spawn_ix_[v] + 1) & (GRAIN_SLOTS - 1);
        spawn_cnt_[v] = H - 1;
      }
      float acc = 0.0f;
      for (int k = 0; k < GRAIN_SLOTS; k++) {
        Grain& g = gr_[v][k];
        if (g.age >= (uint16_t)G) continue;
        float pos = (float)w_ - g.d;
        int32_t i0 = (int32_t)pos;
        float frac = pos - (float)i0;
        uint32_t ix = (uint32_t)i0 & (RING_N - 1);
        uint32_t ix2 = (ix + 1) & (RING_N - 1);
        float s0 = (float)ring_[ix];
        float s = s0 + frac * ((float)ring_[ix2] - s0);
        acc += s * HannFromPhase((float)g.age * gnorm);
        g.d += 1.0f - r[v]; // read head advances at ratio r
        g.age++;
      }
      voice[v] = acc * ov_gain;
    }

    // formant-ish tilt: 1-pole shelf per voice
    for (int v = 0; v < 2; v++) {
      lp_[v] += 0.02f * (voice[v] - lp_[v]);
      if (tilt < 0.0f) voice[v] = voice[v] * (1.0f + tilt) + lp_[v] * (-tilt);
      else voice[v] += tilt * (voice[v] - lp_[v]);
    }

    // Echobode-style feedback: the shifted output re-enters the ring, so
    // every repeat is shifted again on its next pass through the grains.
    float fbsig;
    if (fbs == 0) fbsig = voice[0];
    else if (fbs == 1) fbsig = voice[1];
    else fbsig = 0.5f * (voice[0] + voice[1]);

    ring_[w_] = Clip16(x + fbk * fbsig);
    w_ = (w_ + 1) & (RING_N - 1);

    out0->data[n] = Clip16(voice[0]);
    out1->data[n] = Clip16(voice[1]);
  }

  release(in_block);
  transmit(out0, 0);
  release(out0);
  transmit(out1, 1);
  release(out1);
}

FLASHMEM void HarmoCore::Start() {
  alloc_ok = pitch.begin();

  // All mixers are AudioMixer<2> (shared instantiation elsewhere in the
  // tree - a 3-channel template would add fresh ITCM-resident weak symbols).
  PatchCable(input, 0, pitch, 0); // mono (L) source
  PatchCable(input, 0, mix_l, 0); // dry L
  PatchCable(input, 1, mix_r, 0); // dry R
  PatchCable(pitch, 0, pan_l, 0); // voice 1
  PatchCable(pitch, 1, pan_l, 1); // voice 2
  PatchCable(pitch, 0, pan_r, 0);
  PatchCable(pitch, 1, pan_r, 1);
  PatchCable(pan_l, 0, mix_l, 1);
  PatchCable(pan_r, 0, mix_r, 1);

  PatchCable(mix_l, 0, output, 0);
  PatchCable(mix_r, 0, output, 1);

  sm_r[0] = sm_r[1] = 1.0f;
  sm_mix = (float)mix;
  SetParams();
}

FLASHMEM void HarmoCore::Unload() {
  pitch.end();
  alloc_ok = false;
  AllowRestart();
}

FLASHMEM void HarmoCore::SetParams() {
  if (!alloc_ok) return;
  sm_mix = (float)mix;
}

FLASHMEM void HarmoCore::Controller() {
  if (!alloc_ok) return;

  // Effective intervals: knob + CV (bipolar via attenuversion), clamped to
  // +/-1 octave so the granular reader stays in its buffer window.
  float c1 = (float)v1_int + v1_cv.InF() * 1200.0f;
  float c2 = (float)v2_int + v2_cv.InF() * 1200.0f;
  c1 = constrain(c1, -1200.0f, 1200.0f);
  c2 = constrain(c2, -1200.0f, 1200.0f);
  if (!pmode) {
    // quantized mode: snap the total (knob + CV) to whole semitones
    int s1 = (int)(c1 * 0.01f + (c1 >= 0.0f ? 0.5f : -0.5f));
    int s2 = (int)(c2 * 0.01f + (c2 >= 0.0f ? 0.5f : -0.5f));
    c1 = (float)(s1 * 100);
    c2 = (float)(s2 * 100);
  }

  // Smoothed read ratios (one-pole at the control tick) avoid zipper noise
  // while still letting the grains' natural resplicing hide fast sweeps.
  ONE_POLE(sm_r[0], fastpow2(c1 * (1.0f / 1200.0f)), 0.05f);
  ONE_POLE(sm_r[1], fastpow2(c2 * (1.0f / 1200.0f)), 0.05f);
  pitch.setRatio(0, sm_r[0]);
  pitch.setRatio(1, sm_r[1]);

  const int G = kHarmoGrainLen[grain & 3];
  pitch.setGrain(G, G / (2 + (ovlp % 3)));

  const float fbe = constrain((float)fb + fb_cv.InF() * 100.0f, 0.0f, 100.0f);
  pitch.setFeedback(fbe * 0.0095f, fbsrc % 3);
  pitch.setTilt((float)tilt * 0.01f);

  // Dry/wet and pan are block-rate mixer gains; smooth mix at the tick rate.
  const float eff_mix = constrain(
    (float)mix + mix_cv.InF() * 100.0f, 0.0f, 100.0f
  );
  ONE_POLE(sm_mix, eff_mix, 0.2f);
  const float wet = sm_mix * 0.01f;
  const float dry = 1.0f - wet;
  const float l1 = (float)v1_lvl * 0.01f * wet;
  const float l2 = (float)v2_lvl * 0.01f * wet;

  mix_l.gain(0, dry);
  mix_r.gain(0, dry);
  mix_l.gain(1, 1.0f);
  mix_r.gain(1, 1.0f);
  if (spread) {
    pan_l.gain(0, l1 * 0.707f);
    pan_l.gain(1, l2 * 0.707f);
    pan_r.gain(0, l1 * 0.707f);
    pan_r.gain(1, l2 * 0.707f);
  } else {
    pan_l.gain(0, l1);
    pan_l.gain(1, 0.0f);
    pan_r.gain(0, 0.0f);
    pan_r.gain(1, l2);
  }
}

FLASHMEM void HarmoCore::View() {
  if (!alloc_ok) {
    gfxPrint(2, 15, PSTR("Out of RAM!!"));
    return;
  }

  const int label_x = 1;
  const int page = (cursor >= V2_INT) + (cursor >= GRAIN) + (cursor >= MIX);
  gfxRect(page * 15, 10, 14, 3); // page tab

  switch (page) {
    case 0: // ---- Voice 1 ----
      gfxPrint(label_x, 15, PSTR("I1:"));
      gfxStartCursor();
      if (pmode) graphics.printf("%+4dc", (int)v1_int);
      else graphics.printf("%+4dst", (int)(v1_int / 100));
      gfxEndCursor(cursor == V1_INT);
      gfxStartCursor();
      gfxPrint(v1_cv);
      gfxEndCursor(cursor == V1_INT_CV, false, v1_cv.InputName());

      gfxPrint(label_x, 25, PSTR("L1:"));
      gfxStartCursor();
      graphics.printf("%3d%%", v1_lvl);
      gfxEndCursor(cursor == V1_LVL);

      gfxPrint(label_x, 35, PSTR("Md:"));
      gfxStartCursor();
      gfxPrint(pmode ? PSTR("Free") : PSTR("Quant"));
      gfxEndCursor(cursor == PMODE);
      break;

    case 1: // ---- Voice 2 ----
      gfxPrint(label_x, 15, PSTR("I2:"));
      gfxStartCursor();
      if (pmode) graphics.printf("%+4dc", (int)v2_int);
      else graphics.printf("%+4dst", (int)(v2_int / 100));
      gfxEndCursor(cursor == V2_INT);
      gfxStartCursor();
      gfxPrint(v2_cv);
      gfxEndCursor(cursor == V2_INT_CV, false, v2_cv.InputName());

      gfxPrint(label_x, 25, PSTR("L2:"));
      gfxStartCursor();
      graphics.printf("%3d%%", v2_lvl);
      gfxEndCursor(cursor == V2_LVL);

      gfxPrint(label_x, 35, PSTR("Sp:"));
      gfxStartCursor();
      gfxPrint(spread ? PSTR("Both") : PSTR("Wide"));
      gfxEndCursor(cursor == SPREAD);
      break;

    case 2: // ---- Engine ----
      gfxPrint(label_x, 15, PSTR("Gr:"));
      gfxStartCursor();
      graphics.printf("%4d", kHarmoGrainLen[grain & 3]);
      gfxEndCursor(cursor == GRAIN);

      gfxPrint(label_x, 25, PSTR("Ov:"));
      gfxStartCursor();
      graphics.printf("%3d%%", 50 + (ovlp % 3) * 17);
      gfxEndCursor(cursor == OVLP);

      gfxPrint(label_x, 35, PSTR("Fb:"));
      gfxStartCursor();
      graphics.printf("%3d%%", fb);
      gfxEndCursor(cursor == FB);
      gfxStartCursor();
      gfxPrint(fb_cv);
      gfxEndCursor(cursor == FB_CV, false, fb_cv.InputName());

      gfxPrint(label_x, 45, PSTR("Src:"));
      gfxStartCursor();
      gfxPrint(fbsrc == 0 ? PSTR("V1") : (fbsrc == 1 ? PSTR("V2") : PSTR("B+")));
      gfxEndCursor(cursor == FBSRC);
      break;

    case 3: // ---- Output ----
      gfxPrint(label_x, 15, PSTR("Mix:"));
      gfxStartCursor();
      graphics.printf("%3d%%", mix);
      gfxEndCursor(cursor == MIX);
      gfxStartCursor();
      gfxPrint(mix_cv);
      gfxEndCursor(cursor == MIX_CV, false, mix_cv.InputName());

      gfxPrint(label_x, 25, PSTR("Tlt:"));
      gfxStartCursor();
      graphics.printf("%+4d", (int)tilt);
      gfxEndCursor(cursor == TILT);

      gfxPrint(label_x, 35, pitch.IsPSRAM() ? PSTR("buf: PSRAM") : PSTR("buf: RAM2"));
      break;
  }

  gfxDisplayInputMapEditor();
}

FLASHMEM void HarmoCore::OnButtonPress() {
  if (CheckEditInputMapPress(
        cursor,
        IndexedInput(V1_INT_CV, v1_cv),
        IndexedInput(V2_INT_CV, v2_cv),
        IndexedInput(FB_CV, fb_cv),
        IndexedInput(MIX_CV, mix_cv)
      ))
    return;
  CursorToggle();
}

FLASHMEM void HarmoCore::OnEncoderMove(int direction) {
  if (!EditMode()) {
    MoveCursor(cursor, direction, CURSOR_LENGTH - 1);
    return;
  }
  if (EditSelectedInputMap(direction)) return;

  switch (cursor) {
    case V1_INT:
      v1_int = constrain(
        v1_int + direction * (pmode ? 5 : 100), (int16_t)-1200, (int16_t)1200
      );
      break;
    case V1_INT_CV: v1_cv.ChangeSource(direction); break;
    case V1_LVL: v1_lvl = constrain(v1_lvl + direction, 0, 100); break;
    case PMODE: pmode = constrain(pmode + direction, 0, 1); break;

    case V2_INT:
      v2_int = constrain(
        v2_int + direction * (pmode ? 5 : 100), (int16_t)-1200, (int16_t)1200
      );
      break;
    case V2_INT_CV: v2_cv.ChangeSource(direction); break;
    case V2_LVL: v2_lvl = constrain(v2_lvl + direction, 0, 100); break;
    case SPREAD: spread = constrain(spread + direction, 0, 1); break;

    case GRAIN: grain = constrain(grain + direction, 0, 3); break;
    case OVLP: ovlp = constrain(ovlp + direction, 0, 2); break;
    case FB: fb = constrain(fb + direction, 0, 100); break;
    case FB_CV: fb_cv.ChangeSource(direction); break;
    case FBSRC: fbsrc = constrain(fbsrc + direction, 0, 2); break;

    case MIX: mix = constrain(mix + direction, 0, 100); break;
    case MIX_CV: mix_cv.ChangeSource(direction); break;
    case TILT: tilt = constrain(tilt + direction, -100, 100); break;

    default: return;
  }
  SetParams();
}

// 256-bit state layout (4 x 64-bit words):
//   word 0 (60b): v1_int(12) v2_int(12) v1_lvl(7) v2_lvl(7) pmode(1)
//                 grain(2) ovlp(2) fb(7) fbsrc(2) mix(7) spread(1)
//   word 1 (40b): tilt(8) v1_cv(16) v2_cv(16)
//   word 2 (32b): fb_cv(16) mix_cv(16)
//   word 3: unused
FLASHMEM void HarmoCore::OnDataRequest(std::array<uint64_t, CONFIG_SIZE>& data) {
  data[0] = PackPackables(
    pack<12>(v1_int),
    pack<12>(v2_int),
    pack<7>(v1_lvl),
    pack<7>(v2_lvl),
    pack<1>(pmode),
    pack<2>(grain),
    pack<2>(ovlp),
    pack<7>(fb),
    pack<2>(fbsrc),
    pack<7>(mix),
    pack<1>(spread)
  );
  data[1] = PackPackables(pack<8>(tilt), v1_cv, v2_cv);
  data[2] = PackPackables(fb_cv, mix_cv);
  data[3] = 0;
}

FLASHMEM void HarmoCore::OnDataReceive(
  const std::array<uint64_t, CONFIG_SIZE>& data
) {
  UnpackPackables(
    data[0],
    pack<12>(v1_int),
    pack<12>(v2_int),
    pack<7>(v1_lvl),
    pack<7>(v2_lvl),
    pack<1>(pmode),
    pack<2>(grain),
    pack<2>(ovlp),
    pack<7>(fb),
    pack<2>(fbsrc),
    pack<7>(mix),
    pack<1>(spread)
  );
  UnpackPackables(data[1], pack<8>(tilt), v1_cv, v2_cv);
  UnpackPackables(data[2], fb_cv, mix_cv);

  // re-derive packed ranges defensively
  v1_int = constrain(v1_int, (int16_t)-1200, (int16_t)1200);
  v2_int = constrain(v2_int, (int16_t)-1200, (int16_t)1200);
  v1_lvl = constrain(v1_lvl, 0, 100);
  v2_lvl = constrain(v2_lvl, 0, 100);
  pmode = constrain(pmode, 0, 1);
  grain = constrain(grain, 0, 3);
  ovlp = constrain(ovlp, 0, 2);
  fb = constrain(fb, 0, 100);
  fbsrc = constrain(fbsrc, 0, 2);
  mix = constrain(mix, 0, 100);
  spread = constrain(spread, 0, 1);
  tilt = constrain(tilt, -100, 100);

  SetParams();
}

FLASHMEM void HarmoCore::SetHelp() {
  //                    "-------" <-- Label size guide
  help[HELP_DIGITAL1] = PSTR(" - ");
  help[HELP_DIGITAL2] = PSTR(" - ");
  help[HELP_CV1]      = PSTR("V1 Int");
  help[HELP_CV2]      = PSTR("V2 Int");
  help[HELP_OUT1]     = PSTR("Harm 1");
  help[HELP_OUT2]     = PSTR("Harm 2");
  help[HELP_EXTRA1] = PSTR("CV harmonizer,");
  help[HELP_EXTRA2] = PSTR("pitch feedback");
  //                  "---------------------" <-- Extra text size guide
}
