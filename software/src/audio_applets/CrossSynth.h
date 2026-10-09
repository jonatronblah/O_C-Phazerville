#pragma once

#include "../HemisphereAudioApplet.h"
#include "../Audio/AudioPassthrough.h"
#include "../dsputils.h"
#include <Audio.h>

extern "C" uint8_t external_psram_size;
extern "C" void* extmem_malloc(size_t);
extern "C" void extmem_free(void*);

// CrossSynth - two-input spectral morph / cross-synthesis.
//
// SOURCES: L and R stereo inputs are source A and source B - the platform's
// true two-input strength (no commercial module has two full audio ins at
// this price; MW #2 vocoder gap).  Source select:
//   BSrc = "L/R"  : A = live L, B = live R (default)
//   BSrc = "Cap"  : A = live L, B = frozen capture (the B spectrum latched at
//                   the moment Freeze B engaged - the "Convolver-not-in-my-
//                   euro" live-vs-frozen morph).  Both configurations ship.
//
// Signal chain (XSStftEngine does all STFT work; graph objects are stock):
//
//   in L (A) -> [XSCrossFx] -> out L = dryA*A + wet*morph
//   in R (B) ->               out R = dryB*B + wet*morph
//
// STFT CONFIG (duplicate of SpecFreeze.h's machinery, distinct XS names so
//   both headers compile in one TU at integration): N = 512, hop = 256 (50%
//   overlap), sqrt-Hann analysis + synthesis (product = Hann, COLA 1.0 at
//   hop N/2).  Two forward transforms (A, B) + one inverse per hop.
//
// MODES (out per bin, m = morph 0..1):
//   0 MORPH    : out_mag = lerp(|A|,|B|,m); out_phasor = normalize(lerp(
//                A/|A|, B/|B|, m))   -> lerp_wrap(phaseA,phaseB,m) with no
//                trig: the chordal unit-phasor lerp takes the short arc, so
//                wrap-around is handled for free.
//   1 MagA*PhB : out = |A| * B/|B|   (classic cross-synthesis / vocoder-ish)
//   2 MagB*PhA : out = |B| * A/|A|
//
// FORMANT (tilt -1..+1): per-bin band weighting g(k) = 2^(tilt*(log2(k+1)-5))
//   = a dB/octave pivot at ~1.2 kHz (bin 32 of 128); pole-emphasis-style
//   weighting of the cross product.  fastpow2/fastlog2, no libm.
//
// FREEZE B (gate OR AuxButton latch): holds B's frame set (analysis pauses).
// DRY/WET: per-source dry gains + shared wet gain; dry hops are delayed one
//   hop to match the wet (WOLA timing, same as SpecFreeze) so the mix stays
//   time-aligned.
//
// PHASE HANDLING: everything is complex-domain (sqrt per bin; no atan2/sin/
//   cos anywhere in the path).  Bins 257..511 are re-mirrored before the IFFT
//   so conjugate symmetry (and real output) is guaranteed.
//
// MEMORY LAYOUT NOTE (MidSider/Harmo lesson): slot0.ld puts default .text in
//   ITCM (32 KB bank granularity) and *(.rodata*) into DTCM (RAM1!), and GCC
//   silently drops section attrs on template/inline members.  So ALL code is
//   out-of-line FLASHMEM in the non-template classes below (strong symbols),
//   no const float tables exist (twiddles/window are built at begin() into a
//   RAM2 malloc), and the only string literal is PSTR/PROGMEM.  CMSIS
//   arm_cfft_f32 was evaluated and rejected: its archive .text lands in ITCM
//   (one spilled 32 KB bank kills the stack) and its const tables land in
//   RAM1.  Zero RAM1 cost here beyond vtables.
//
// CYCLE MEASUREMENT: ProcessHops() wraps the spectral hop in ARM_DWT_CYCCNT;
//   mainloop() prints avg/max us every 2 s (budget ~333 us/block for the
//   whole graph; this runs in mainloop, NOT the audio ISR, and one hop covers
//   2 blocks).  Fallback if the path measurably breaks the budget: single-
//   STFT morph live-vs-frozen only (drop B's forward FFT and the MagX*PhY
//   modes), or N=256 / hop=128 to halve transform cost.
//
// Registration: `DeclareFancyApplet<CrossSynth<STEREO>>` (single entry).
// Docs bullet: `* CrossSynth - spectral morph and cross-synthesis of two inputs`

static const char XS_NAME[] PROGMEM = "CrossSyn";

// ---------------------------------------------------------------------------
// XSStftEngine - STFT analysis/morph/resynthesis machinery.  Non-template so
// every method below is a strong FLASHMEM symbol.  Not an AudioStream;
// XSCrossFx adapts it to the audio graph.
// ---------------------------------------------------------------------------
class XSStftEngine {
public:
  static const int XS_N = 512;   // FFT size
  static const int XS_HOP = 256; // 50% overlap
  static const int XS_BINS = 257;
  static const int XS_CAP = 2048; // capture ring, samples per channel
  static const int XS_OUT = 2048; // output ring, samples per channel

  enum Mode : uint8_t {
    MODE_MORPH = 0,
    MODE_MAGA_PHB,
    MODE_MAGB_PHA,
  };

  bool begin();
  void end();

  // -- audio ISR side (block-atomic, both channels per call) ----------------
  void PushInput(const int16_t* l, const int16_t* r);
  bool PullOutput(int16_t* l, int16_t* r); // false = starved

  // -- mainloop side --------------------------------------------------------
  void ProcessHops(int max_hops);
  void DebugReport(); // Serial, mainloop context only

  // -- control context (plain 32-bit stores; single writer) -----------------
  void setMorph(float m) { morph_ = m; }
  void setMode(uint8_t mode) { mode_ = mode > MODE_MAGB_PHA ? MODE_MORPH : mode; }
  void setTilt(float t) { tilt_ = t; }
  void setFreezeB(bool f);
  void setBFromCapture(bool c) { b_capture_ = c; }
  void setDryA(float g) { dry_a_ = g; }
  void setDryB(float g) { dry_b_ = g; }
  void setWet(float g) { wet_ = g; }

  volatile uint32_t drops = 0;

private:
  static float SinApprox(float x); // trig-free, begin() only
  void Fft(float* re, float* im, bool inverse);
  void Analyze(const float* hist, float* frame);
  void ProcessHop();

  // hot buffers (RAM2 malloc, one block split by hand)
  int16_t* cap_[2] = {nullptr, nullptr}; // capture ring (ISR -> mainloop)
  int16_t* out_[2] = {nullptr, nullptr}; // output ring (mainloop -> ISR)
  float* win_ = nullptr;                 // sqrt-Hann, XS_N
  float* tw_re_ = nullptr;               // twiddles, XS_N/2
  float* tw_im_ = nullptr;
  uint16_t* bitrev_ = nullptr;           // XS_N
  float* work_re_ = nullptr;             // XS_N
  float* work_im_ = nullptr;             // XS_N
  float* hist_a_ = nullptr;              // XS_N
  float* hist_b_ = nullptr;              // XS_N
  float* frame_a_ = nullptr;             // XS_BINS * 2 (re,im)
  float* frame_b_ = nullptr;             // XS_BINS * 2
  float* frame_bfrz_ = nullptr;          // XS_BINS * 2 (frozen capture)
  float* ola_ = nullptr;                 // XS_N (mono wet)
  int16_t dry_dly_[2][XS_HOP];           // previous hop's dry (aligned emit)
  int16_t dry_cur_[2][XS_HOP];
  uint8_t* hot_ = nullptr;

  // SPSC ring indices (free-running uint16; capacities divide 65536)
  volatile uint16_t cap_w_ = 0, cap_r_ = 0;
  volatile uint16_t out_w_ = 0, out_r_ = 0;

  float morph_ = 0.5f;
  uint8_t mode_ = MODE_MORPH;
  float tilt_ = 0.0f;
  bool freeze_b_ = false;
  bool b_capture_ = false;
  float dry_a_ = 1.0f;
  float dry_b_ = 1.0f;
  float wet_ = 1.0f;

  // profiling
  uint32_t cyc_sum_ = 0, cyc_max_ = 0, cyc_n_ = 0, report_ms_ = 0;
};

// ---------------------------------------------------------------------------
// XSCrossFx - thin AudioStream adapter (2 in / 2 out); update() is FLASHMEM
// like effect_looper.cpp (ITCM bank budget exhausted).
// ---------------------------------------------------------------------------
class XSCrossFx : public AudioStream {
public:
  explicit XSCrossFx(XSStftEngine& e);

  void update() override;

private:
  XSStftEngine& engine;
  audio_block_t* inputQueueArray[2];
};

// ---------------------------------------------------------------------------
// CrossSynthCore - applet state, UI, graph wiring.
// ---------------------------------------------------------------------------
class CrossSynthCore : public HemisphereAudioApplet {
public:
  void Start() final;
  void Unload() final;
  void mainloop() final;
  void Controller() final;
  void View() final;
  void OnButtonPress() override;
  void OnEncoderMove(int direction) override;
  void OnDataRequest(std::array<uint64_t, CONFIG_SIZE>& data) override;
  void OnDataReceive(const std::array<uint64_t, CONFIG_SIZE>& data) override;
  void AuxButton() override;

  AudioStream* InputStream() override { return &input; }
  AudioStream* OutputStream() override { return &output; }

protected:
  void SetHelp() override;

private:
  enum Cursor : int8_t {
    // Page 0: morph / mode / formant
    MORPH = 0, MORPH_CV,
    MODE,
    TILT, TILT_CV,
    // Page 1: freeze / sources / mix
    FRZ_SRC, B_SRC,
    DRY_A, DRY_B,
    WET, WET_CV,
    CURSOR_LENGTH,
  };

  int8_t cursor = MORPH;

  uint8_t morph = 50;    // 0..100
  uint8_t mode = 0;      // Mode enum
  int8_t tilt = 0;       // -100..100 -> -1..+1
  uint8_t dry_a = 100;   // 0..100%
  uint8_t dry_b = 100;   // 0..100%
  uint8_t wet = 100;     // 0..100%
  bool b_capture = false; // B source: live R vs frozen capture

  CVInputMap morph_cv;
  CVInputMap tilt_cv;
  CVInputMap wet_cv;
  DigitalInputMap freeze_input;
  bool manual_freeze = false; // AuxButton latch

  bool alloc_ok = false;

  XSStftEngine engine;
  XSCrossFx fx{engine};
  AudioPassthrough<2> input;
  AudioPassthrough<2> output;
};

// Thin template wrapper so the registry can use the CrossSynth<STEREO> form.
// (All real work lives in the non-template core so FLASHMEM placement survives
// this toolchain; Channels is intentionally unused.)
template <AudioChannels Channels>
class CrossSynth : public CrossSynthCore {
public:
  const char* applet_name() {
    return XS_NAME;
  }
};

// -- Out-of-line definitions (strong symbols, FLASHMEM honored) --------------

FLASHMEM float XSStftEngine::SinApprox(float x) {
  // sin(x) for |x| <= pi/2, 9th-order Taylor (max error ~1e-6), then fold.
  // Trig-free on purpose: libm sin/cos pull ~4 KB of reduction code into ITCM.
  const float pi_2 = 1.57079632679f;
  const float pi = 3.14159265359f;
  const float two_pi = 6.28318530718f;
  x += pi;
  x -= two_pi * static_cast<int>((x + pi) / two_pi); // wrap to (-pi, pi]
  if (x > pi_2) x = pi - x;
  else if (x < -pi_2) x = -pi - x;
  const float x2 = x * x;
  return x * (1.0f + x2 * (-1.0f / 6.0f
    + x2 * (1.0f / 120.0f
    + x2 * (-1.0f / 5040.0f
    + x2 * (1.0f / 362880.0f)))));
}

FLASHMEM bool XSStftEngine::begin() {
  end();

  // ---- hot buffers (RAM2 heap) --------------------------------------------
  const size_t hot_bytes =
    2 * XS_CAP * sizeof(int16_t) +          // cap rings
    2 * XS_OUT * sizeof(int16_t) +          // out rings
    XS_N * sizeof(float) +                  // window
    (XS_N / 2) * sizeof(float) +            // tw_re
    (XS_N / 2) * sizeof(float) +            // tw_im
    XS_N * sizeof(uint16_t) +               // bitrev
    XS_N * sizeof(float) +                  // work_re
    XS_N * sizeof(float) +                  // work_im
    2 * XS_N * sizeof(float) +              // hist_a, hist_b
    3 * XS_BINS * 2 * sizeof(float) +       // frame_a, frame_b, frame_bfrz
    XS_N * sizeof(float) +                  // ola
    512;                                    // alignment slack for take()
  hot_ = static_cast<uint8_t*>(malloc(hot_bytes));
  if (!hot_) return false;

  uint8_t* p = hot_;
  auto take = [&p](size_t n) -> uint8_t* {
    uintptr_t v = (reinterpret_cast<uintptr_t>(p) + 31) & ~uintptr_t(31);
    p = reinterpret_cast<uint8_t*>(v) + n;
    return reinterpret_cast<uint8_t*>(v);
  };
  for (int c = 0; c < 2; ++c)
    cap_[c] = reinterpret_cast<int16_t*>(take(XS_CAP * sizeof(int16_t)));
  for (int c = 0; c < 2; ++c)
    out_[c] = reinterpret_cast<int16_t*>(take(XS_OUT * sizeof(int16_t)));
  win_ = reinterpret_cast<float*>(take(XS_N * sizeof(float)));
  tw_re_ = reinterpret_cast<float*>(take((XS_N / 2) * sizeof(float)));
  tw_im_ = reinterpret_cast<float*>(take((XS_N / 2) * sizeof(float)));
  bitrev_ = reinterpret_cast<uint16_t*>(take(XS_N * sizeof(uint16_t)));
  work_re_ = reinterpret_cast<float*>(take(XS_N * sizeof(float)));
  work_im_ = reinterpret_cast<float*>(take(XS_N * sizeof(float)));
  hist_a_ = reinterpret_cast<float*>(take(XS_N * sizeof(float)));
  hist_b_ = reinterpret_cast<float*>(take(XS_N * sizeof(float)));
  frame_a_ = reinterpret_cast<float*>(take(XS_BINS * 2 * sizeof(float)));
  frame_b_ = reinterpret_cast<float*>(take(XS_BINS * 2 * sizeof(float)));
  frame_bfrz_ = reinterpret_cast<float*>(take(XS_BINS * 2 * sizeof(float)));
  ola_ = reinterpret_cast<float*>(take(XS_N * sizeof(float)));

  // ---- sqrt-Hann window (trig-free) --------------------------------------
  for (int i = 0; i < XS_N; ++i) {
    const float w = 0.5f - 0.5f * SinApprox(6.28318530718f * i / XS_N);
    win_[i] = __builtin_sqrtf(w);
  }

  // ---- twiddles + bit-reversal (FFT scratch, RAM2) -----------------------
  for (int k = 0; k < XS_N / 2; ++k) {
    const float a = -6.28318530718f * k / XS_N;
    tw_re_[k] = SinApprox(a + 1.57079632679f); // cos(a)
    tw_im_[k] = SinApprox(a);                  // sin(a)
  }
  int bits = 0;
  while ((1 << bits) < XS_N) ++bits;
  for (int i = 0; i < XS_N; ++i) {
    int r = 0;
    for (int b = 0; b < bits; ++b)
      if (i & (1 << b)) r |= 1 << (bits - 1 - b);
    bitrev_[i] = static_cast<uint16_t>(r);
  }

  memset(hist_a_, 0, XS_N * sizeof(float));
  memset(hist_b_, 0, XS_N * sizeof(float));
  memset(frame_a_, 0, XS_BINS * 2 * sizeof(float));
  memset(frame_b_, 0, XS_BINS * 2 * sizeof(float));
  memset(frame_bfrz_, 0, XS_BINS * 2 * sizeof(float));
  memset(ola_, 0, XS_N * sizeof(float));
  for (int c = 0; c < 2; ++c) {
    memset(cap_[c], 0, XS_CAP * sizeof(int16_t));
    memset(out_[c], 0, XS_OUT * sizeof(int16_t));
    memset(dry_dly_[c], 0, sizeof(dry_dly_[c]));
    memset(dry_cur_[c], 0, sizeof(dry_cur_[c]));
  }
  cap_w_ = cap_r_ = out_w_ = out_r_ = 0;
  drops = 0;
  cyc_sum_ = cyc_max_ = cyc_n_ = 0;

  // enable the cycle counter for the debug instrumentation
  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
  return true;
}

FLASHMEM void XSStftEngine::end() {
  if (hot_) {
    free(hot_);
    hot_ = nullptr;
  }
}

FLASHMEM void XSStftEngine::setFreezeB(bool f) {
  if (f && !freeze_b_) {
    // freeze edge: latch the current B spectrum as the frozen capture
    memcpy(frame_bfrz_, frame_b_, XS_BINS * 2 * sizeof(float));
  }
  freeze_b_ = f;
}

FLASHMEM void XSStftEngine::PushInput(const int16_t* l, const int16_t* r) {
  if (static_cast<uint16_t>(cap_w_ - cap_r_) + AUDIO_BLOCK_SAMPLES > XS_CAP) {
    ++drops; // capture starved (mainloop stalled) - drop the whole block pair
    return;
  }
  const uint16_t w = cap_w_;
  const uint16_t mask = XS_CAP - 1;
  int16_t* rl = cap_[0];
  int16_t* rr = cap_[1];
  for (int i = 0; i < AUDIO_BLOCK_SAMPLES; ++i) {
    rl[(uint16_t)(w + i) & mask] = l ? l[i] : 0;
    rr[(uint16_t)(w + i) & mask] = r ? r[i] : 0;
  }
  cap_w_ = static_cast<uint16_t>(w + AUDIO_BLOCK_SAMPLES); // commit both
}

FLASHMEM bool XSStftEngine::PullOutput(int16_t* l, int16_t* r) {
  if (static_cast<uint16_t>(out_w_ - out_r_) < AUDIO_BLOCK_SAMPLES) return false;
  const uint16_t rd = out_r_;
  const uint16_t mask = XS_OUT - 1;
  const int16_t* rl = out_[0];
  const int16_t* rr = out_[1];
  for (int i = 0; i < AUDIO_BLOCK_SAMPLES; ++i) {
    l[i] = rl[(uint16_t)(rd + i) & mask];
    r[i] = rr[(uint16_t)(rd + i) & mask];
  }
  out_r_ = static_cast<uint16_t>(rd + AUDIO_BLOCK_SAMPLES); // commit both
  return true;
}

// In-place radix-2 DIT FFT.  inverse = conjugate twiddles + 1/N scale.
FLASHMEM void XSStftEngine::Fft(float* re, float* im, bool inverse) {
  for (int i = 0; i < XS_N; ++i) {
    const int j = bitrev_[i];
    if (j > i) {
      float t = re[i]; re[i] = re[j]; re[j] = t;
      t = im[i]; im[i] = im[j]; im[j] = t;
    }
  }
  for (int len = 2; len <= XS_N; len <<= 1) {
    const int half = len >> 1;
    const int step = XS_N / len;
    for (int i = 0; i < XS_N; i += len) {
      for (int j = 0; j < half; ++j) {
        const int t = j * step;
        const float wr = tw_re_[t];
        const float wi = inverse ? tw_im_[t] : -tw_im_[t];
        const int a = i + j;
        const int b = a + half;
        const float xr = re[b] * wr - im[b] * wi;
        const float xi = re[b] * wi + im[b] * wr;
        re[b] = re[a] - xr;
        im[b] = im[a] - xi;
        re[a] += xr;
        im[a] += xi;
      }
    }
  }
  if (inverse) {
    const float s = 1.0f / XS_N;
    for (int i = 0; i < XS_N; ++i) {
      re[i] *= s;
      im[i] *= s;
    }
  }
}

// Windowed forward transform of a history buffer into a packed frame
// (frame[2k] = re(k), frame[2k+1] = im(k), k = 0..XS_BINS-1).
FLASHMEM void XSStftEngine::Analyze(const float* hist, float* frame) {
  for (int i = 0; i < XS_N; ++i) {
    work_re_[i] = hist[i] * win_[i];
    work_im_[i] = 0.0f;
  }
  Fft(work_re_, work_im_, false);
  for (int k = 0; k < XS_BINS; ++k) {
    frame[2 * k] = work_re_[k];
    frame[2 * k + 1] = work_im_[k];
  }
}

FLASHMEM void XSStftEngine::ProcessHop() {
  // previous hop's dry matches this wet hop (WOLA timing) - rotate the stash
  for (int c = 0; c < 2; ++c)
    memcpy(dry_dly_[c], dry_cur_[c], XS_HOP * sizeof(int16_t));

  // ---- pop one hop per channel; slide analysis history --------------------
  for (int c = 0; c < 2; ++c) {
    const int16_t* ring = cap_[c];
    const uint16_t r = cap_r_;
    const uint16_t mask = XS_CAP - 1;
    float* h = c ? hist_b_ : hist_a_;
    memcpy(h, h + XS_HOP, (XS_N - XS_HOP) * sizeof(float));
    float* dst = h + (XS_N - XS_HOP);
    for (int i = 0; i < XS_HOP; ++i) {
      const int16_t s = ring[(uint16_t)(r + i) & mask];
      dry_cur_[c][i] = s;
      dst[i] = static_cast<float>(s);
    }
  }
  cap_r_ = static_cast<uint16_t>(cap_r_ + XS_HOP);

  // ---- analysis: A always live; B pauses while frozen --------------------
  Analyze(hist_a_, frame_a_);
  if (!freeze_b_) Analyze(hist_b_, frame_b_);
  const float* fb = b_capture_ ? frame_bfrz_ : frame_b_;

  // ---- morph (complex domain, trig-free) ---------------------------------
  const float m = morph_;
  const float tilt = tilt_;
  float* re = work_re_;
  float* im = work_im_;
  for (int k = 0; k < XS_BINS; ++k) {
    const float ar = frame_a_[2 * k], ai = frame_a_[2 * k + 1];
    const float br = fb[2 * k], bi = fb[2 * k + 1];
    const float ma = __builtin_sqrtf(ar * ar + ai * ai);
    const float mb = __builtin_sqrtf(br * br + bi * bi);
    float rr, ii;

    switch (mode_) {
      case MODE_MAGA_PHB: // |A| * B/|B|
        if (mb > 1e-6f) {
          const float s = ma / mb;
          rr = br * s;
          ii = bi * s;
        } else {
          rr = ar;
          ii = ai;
        }
        break;
      case MODE_MAGB_PHA: // |B| * A/|A|
        if (ma > 1e-6f) {
          const float s = mb / ma;
          rr = ar * s;
          ii = ai * s;
        } else {
          rr = br;
          ii = bi;
        }
        break;
      default: { // MODE_MORPH: magnitude lerp + chordal unit-phasor lerp
        const float mag = ma + (mb - ma) * m;
        if (ma > 1e-6f && mb > 1e-6f) {
          float ur = ar / ma * (1.0f - m) + br / mb * m;
          float ui = ai / ma * (1.0f - m) + bi / mb * m;
          const float un = __builtin_sqrtf(ur * ur + ui * ui);
          if (un > 1e-6f) {
            rr = ur / un * mag;
            ii = ui / un * mag;
          } else {
            rr = ar * (1.0f - m) + br * m;
            ii = ai * (1.0f - m) + bi * m;
          }
        } else {
          rr = ar * (1.0f - m) + br * m;
          ii = ai * (1.0f - m) + bi * m;
        }
        break;
      }
    }

    // formant / spectral tilt: 2^(tilt*(log2(k+1)-5)) dB/oct around bin 31
    if (tilt != 0.0f) {
      const float g = fastpow2(tilt * (fastlog2(static_cast<float>(k) + 1.0f) - 5.0f));
      rr *= g;
      ii *= g;
    }
    re[k] = rr;
    im[k] = ii;
  }
  // rebuild conjugate mirror for the inverse FFT
  for (int k = 1; k < XS_BINS - 1; ++k) {
    re[XS_N - k] = re[k];
    im[XS_N - k] = -im[k];
  }
  im[0] = 0.0f;
  im[XS_BINS - 1] = 0.0f;

  Fft(re, im, true);

  // ---- synthesis window + OLA + per-source dry mix ------------------------
  for (int i = 0; i < XS_N; ++i) ola_[i] += re[i] * win_[i];
  const uint16_t w = out_w_;
  const uint16_t mask = XS_OUT - 1;
  int16_t* ol = out_[0];
  int16_t* orr = out_[1];
  const int16_t* dla = dry_dly_[0];
  const int16_t* dlb = dry_dly_[1];
  for (int i = 0; i < XS_HOP; ++i) {
    const float wetv = wet_ * ola_[i];
    ol[(uint16_t)(w + i) & mask] =
      Clip16(wetv + dry_a_ * static_cast<float>(dla[i]));
    orr[(uint16_t)(w + i) & mask] =
      Clip16(wetv + dry_b_ * static_cast<float>(dlb[i]));
  }
  memcpy(ola_, ola_ + XS_HOP, (XS_N - XS_HOP) * sizeof(float));
  memset(ola_ + (XS_N - XS_HOP), 0, XS_HOP * sizeof(float));
  out_w_ = static_cast<uint16_t>(w + XS_HOP);
}

FLASHMEM void XSStftEngine::ProcessHops(int max_hops) {
  for (int h = 0; h < max_hops; ++h) {
    if (static_cast<uint16_t>(cap_w_ - cap_r_) < XS_HOP) break;
    if (static_cast<uint16_t>(out_w_ - out_r_) + XS_HOP > XS_OUT) break;
    const uint32_t t0 = ARM_DWT_CYCCNT;
    ProcessHop();
    const uint32_t dt = ARM_DWT_CYCCNT - t0;
    cyc_sum_ += dt;
    ++cyc_n_;
    if (dt > cyc_max_) cyc_max_ = dt;
  }
}

FLASHMEM void XSStftEngine::DebugReport() {
  const uint32_t now = millis();
  if (now - report_ms_ < 2000) return;
  report_ms_ = now;
  if (cyc_n_ == 0) return;
  const uint32_t avg = cyc_sum_ / cyc_n_;
  // 600 MHz core: cycles / 600 = microseconds
  Serial.print(PSTR("XS hop: avg "));
  Serial.print(avg / 600);
  Serial.print(PSTR(" us, max "));
  Serial.print(cyc_max_ / 600);
  Serial.print(PSTR(" us, n="));
  Serial.print(cyc_n_);
  Serial.print(PSTR(", drops="));
  Serial.println(drops);
  cyc_sum_ = cyc_max_ = cyc_n_ = 0;
}

// -- XSCrossFx (audio ISR) ---------------------------------------------------

FLASHMEM XSCrossFx::XSCrossFx(XSStftEngine& e)
  : AudioStream(2, inputQueueArray), engine(e) {}

FLASHMEM void XSCrossFx::update() {
  audio_block_t* inl = receiveReadOnly(0);
  audio_block_t* inr = receiveReadOnly(1);
  if (inl || inr) {
    engine.PushInput(inl ? inl->data : nullptr, inr ? inr->data : nullptr);
    if (inl) release(inl);
    if (inr) release(inr);
  }
  audio_block_t* ol = allocate();
  audio_block_t* orr = allocate();
  if (ol && orr) {
    if (!engine.PullOutput(ol->data, orr->data)) {
      memset(ol->data, 0, AUDIO_BLOCK_SAMPLES * sizeof(int16_t));
      memset(orr->data, 0, AUDIO_BLOCK_SAMPLES * sizeof(int16_t));
      ++engine.drops;
    }
    transmit(ol, 0);
    transmit(orr, 1);
  }
  if (ol) release(ol);
  if (orr) release(orr);
}

// -- CrossSynthCore ----------------------------------------------------------

FLASHMEM void CrossSynthCore::Start() {
  alloc_ok = engine.begin();
  if (alloc_ok) {
    PatchCable(input, 0, fx, 0);
    PatchCable(input, 1, fx, 1);
    PatchCable(fx, 0, output, 0);
    PatchCable(fx, 1, output, 1);
    engine.setFreezeB(freeze_input.Gate() || manual_freeze);
    engine.setBFromCapture(b_capture);
  }
}

FLASHMEM void CrossSynthCore::Unload() {
  engine.end();
  alloc_ok = false;
  AllowRestart();
}

FLASHMEM void CrossSynthCore::mainloop() {
  if (!alloc_ok) return;
  engine.ProcessHops(4); // catch up after UI stalls, bounded
  engine.DebugReport();
}

FLASHMEM void CrossSynthCore::Controller() {
  if (!alloc_ok) return;

  const float eff_morph = constrain(morph + morph_cv.InF() * 50.0f, 0.0f, 100.0f);
  const float eff_tilt = constrain(
    static_cast<float>(tilt) + tilt_cv.InF() * 50.0f, -100.0f, 100.0f);
  const float eff_wet = constrain(wet + wet_cv.InF() * 50.0f, 0.0f, 100.0f);

  engine.setMorph(eff_morph * 0.01f);
  engine.setTilt(eff_tilt * 0.01f);
  engine.setDryA(dry_a * 0.01f);
  engine.setDryB(dry_b * 0.01f);
  engine.setWet(eff_wet * 0.01f);
  engine.setBFromCapture(b_capture);
  engine.setFreezeB(freeze_input.Gate() || manual_freeze);
}

FLASHMEM void CrossSynthCore::View() {
  if (!alloc_ok) {
    gfxPrint(2, 15, PSTR("Out of RAM!!"));
    return;
  }

  // (no string tables here: .rodata lands in DTCM/RAM1 - PSTR only)
  const int label_x = 1;
  const int page = (cursor >= FRZ_SRC);
  gfxRect(page * 15, 10, 14, 3); // page tab

  switch (page) {
    case 0:
      gfxPrint(label_x, 15, PSTR("Mrp:"));
      gfxStartCursor();
      graphics.printf(PSTR("%3d%%"), morph);
      gfxEndCursor(cursor == MORPH);
      gfxStartCursor();
      gfxPrint(morph_cv);
      gfxEndCursor(cursor == MORPH_CV, false, morph_cv.InputName());

      gfxPrint(label_x, 25, PSTR("Mod:"));
      gfxStartCursor();
      switch (mode % 3) {
        case 1: gfxPrint(PSTR("A*Bp")); break;
        case 2: gfxPrint(PSTR("B*Ap")); break;
        default: gfxPrint(PSTR("Morph")); break;
      }
      gfxEndCursor(cursor == MODE);

      gfxPrint(label_x, 35, PSTR("Frm:"));
      gfxStartCursor();
      graphics.printf(PSTR("%4d"), tilt);
      gfxEndCursor(cursor == TILT);
      gfxStartCursor();
      gfxPrint(tilt_cv);
      gfxEndCursor(cursor == TILT_CV, false, tilt_cv.InputName());
      break;

    case 1:
      // Freeze row: label inverts while latched (print first, then invert)
      gfxPrint(label_x, 15, PSTR("Frz:"));
      if (manual_freeze) gfxInvert(1, 15, 20, 8);
      gfxStartCursor();
      gfxPrint(freeze_input);
      gfxEndCursor(cursor == FRZ_SRC, true, freeze_input.InputName());

      gfxPrint(label_x, 25, PSTR("Src:"));
      gfxStartCursor();
      gfxPrint(b_capture ? PSTR("A/Cap") : PSTR("A/B"));
      gfxEndCursor(cursor == B_SRC);

      gfxPrint(label_x, 35, PSTR("DrA:"));
      gfxStartCursor();
      graphics.printf(PSTR("%3d%%"), dry_a);
      gfxEndCursor(cursor == DRY_A);

      gfxPrint(label_x, 45, PSTR("DrB:"));
      gfxStartCursor();
      graphics.printf(PSTR("%3d%%"), dry_b);
      gfxEndCursor(cursor == DRY_B);

      gfxPrint(label_x, 55, PSTR("Wet:"));
      gfxStartCursor();
      graphics.printf(PSTR("%3d%%"), wet);
      gfxEndCursor(cursor == WET);
      gfxStartCursor();
      gfxPrint(wet_cv);
      gfxEndCursor(cursor == WET_CV, false, wet_cv.InputName());
      break;
  }

  gfxDisplayInputMapEditor();
}

FLASHMEM void CrossSynthCore::OnButtonPress() {
  if (CheckEditInputMapPress(
        cursor,
        IndexedInput(MORPH_CV, morph_cv),
        IndexedInput(TILT_CV, tilt_cv),
        IndexedInput(FRZ_SRC, freeze_input),
        IndexedInput(WET_CV, wet_cv)
      ))
    return;
  CursorToggle();
}

FLASHMEM void CrossSynthCore::AuxButton() {
  manual_freeze ^= 1; // latch/unlatch Freeze B (performance use)
}

FLASHMEM void CrossSynthCore::OnEncoderMove(int direction) {
  if (!EditMode()) {
    MoveCursor(cursor, direction, CURSOR_LENGTH - 1);
    return;
  }
  if (EditSelectedInputMap(direction)) return;

  switch (cursor) {
    case MORPH: morph = constrain(morph + direction, 0, 100); break;
    case MORPH_CV: morph_cv.ChangeSource(direction); break;
    case MODE: mode = constrain(mode + direction, 0, 2); break;
    case TILT: tilt = constrain(tilt + direction, -100, 100); break;
    case TILT_CV: tilt_cv.ChangeSource(direction); break;

    case FRZ_SRC: freeze_input.ChangeSource(direction); break;
    case B_SRC: b_capture = !b_capture; break;
    case DRY_A: dry_a = constrain(dry_a + direction, 0, 100); break;
    case DRY_B: dry_b = constrain(dry_b + direction, 0, 100); break;
    case WET: wet = constrain(wet + direction, 0, 100); break;
    case WET_CV: wet_cv.ChangeSource(direction); break;

    default: return;
  }
}

// 256-bit state layout (4 x 64-bit words), everything in-bounds:
//   word 0 (40b): morph(8) mode(2) tilt(8) dry_a(7) dry_b(7) wet(7) b_cap(1)
//   word 1 (48b): morph_cv tilt_cv wet_cv (3 x 16)
//   word 2 (32b): freeze_input(32)
//   word 3 (0b):  spare (0)
FLASHMEM void CrossSynthCore::OnDataRequest(std::array<uint64_t, CONFIG_SIZE>& data) {
  data[0] = PackPackables(
    pack<8>(morph),
    pack<2>(mode),
    tilt,
    pack<7>(dry_a),
    pack<7>(dry_b),
    pack<7>(wet),
    pack<1>(b_capture)
  );
  data[1] = PackPackables(morph_cv, tilt_cv, wet_cv);
  data[2] = PackPackables(freeze_input);
  data[3] = 0;
}

FLASHMEM void CrossSynthCore::OnDataReceive(
  const std::array<uint64_t, CONFIG_SIZE>& data
) {
  UnpackPackables(
    data[0],
    pack<8>(morph),
    pack<2>(mode),
    tilt,
    pack<7>(dry_a),
    pack<7>(dry_b),
    pack<7>(wet),
    pack<1>(b_capture)
  );
  UnpackPackables(data[1], morph_cv, tilt_cv, wet_cv);
  UnpackPackables(data[2], freeze_input);

  // re-derive packed ranges defensively
  morph = constrain(morph, 0, 100);
  mode = constrain(mode, 0, 2);
  tilt = constrain(tilt, -100, 100);
  dry_a = constrain(dry_a, 0, 100);
  dry_b = constrain(dry_b, 0, 100);
  wet = constrain(wet, 0, 100);
}

FLASHMEM void CrossSynthCore::SetHelp() {
  help[HELP_DIGITAL1] = PSTR("Frz B");
  help[HELP_DIGITAL2] = PSTR(" - ");
  help[HELP_CV1] = PSTR("Morph");
  help[HELP_CV2] = PSTR("Wet");
  help[HELP_OUT1] = PSTR("A/morph");
  help[HELP_OUT2] = PSTR("B/morph");
  help[HELP_EXTRA1] = PSTR("L,R = A,B src");
  help[HELP_EXTRA2] = PSTR("Aux = frz latch");
}
