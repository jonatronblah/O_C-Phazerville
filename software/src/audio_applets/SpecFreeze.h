#pragma once

#include "../HemisphereAudioApplet.h"
#include "../Audio/AudioPassthrough.h"
#include "../dsputils.h"
#include <Audio.h>

extern "C" uint8_t external_psram_size;
extern "C" void* extmem_malloc(size_t);
extern "C" void extmem_free(void*);

// SpecFreeze - live spectral freeze + stretch-lite with CV.
//
// Signal chain (one custom AudioStream does capture + resynthesis; the rest is
// stock graph glue):
//
//   in L/R -> [SFSpecFx] -> out L/R
//
//   SFSpecFx::update() (audio ISR) only moves samples across SPSC rings:
//     input blocks -> capture ring        output ring -> output blocks
//   all STFT work runs from mainloop() on hop-sized chunks of the capture ring
//   (double-buffered capture, house-rule pattern - the per-hop FFT path may
//   exceed the audio ISR budget and there is no target hardware here to prove
//   otherwise, so it is kept out of the ISR by construction; latency below).
//
// STFT config: N = 512, hop = 256 (50% overlap), sqrt-Hann analysis AND
// synthesis window (product = Hann = COLA 1.0 at hop N/2).  The window is
// built at begin() with a trig-free 9th-order Taylor sine (quadrant folded):
// libm sin/cos pull ~4 KB of __kernel_* reduction code into ITCM (effect_width
// lesson) which blows the 32 KB ITCM bank budget.
//
// The FFT is a hand-rolled in-place radix-2, all methods FLASHMEM (flash),
// with twiddles + bit-reversal generated at begin() into a RAM2 malloc.
// CMSIS-DSP arm_cfft_f32 was evaluated and REJECTED on measurements of the
// memory map: its const twiddle/bitrev tables live in .rodata and slot0.ld
// maps *(.rodata*) into DTCM (RAM1!) - several KB against a ~19 KB budget with
// a 64 B/applet gate - and its archive .text lands in ITCM where the tree
// baseline already sits close to a 32 KB bank boundary (one spilled bank costs
// 32 KB of stack).  Runtime twiddles cost RAM1 nothing.
//
// FRAME RING (rolling spectral capture, PSRAM preferred):
//   every hop the 257-bin complex spectrum of each channel is appended to a
//   power-of-two frame ring.  Depth 128 frames PSRAM (~0.68 s of history at
//   hop 256), 16 frames RAM2 fallback ("No PSRAM" note in the UI).  The read
//   position is measured in "frames behind newest" and driven by the params:
//
//   Frz  - Freeze: gate OR manual latch (AuxButton).  Holds the entire frame
//          set: analysis stops appending, resynthesis keeps re-emitting, and
//          Scrub/Blur navigate the frozen capture.  Freeze under CV is the
//          core wish.
//   Str  - Stretch 1x..50x (log): the read head advances 1/stretch frames per
//          hop while capture keeps appending, so frames are re-emitted before
//          advancing (stretch-lite / "blur over time").  Because the ring is
//          bounded, the read head trails off the old end after
//          depth/(1-1/stretch) hops (~0.7 s at 50x) and re-syncs to the live
//          edge - an audible seam that Blur softens.  TRUE live Paulstretch
//          (randomized phase propagation) needs unbounded buffering and a
//          per-bin phase accumulator; that is the documented dropped feature.
//   Blu  - Blur: magnitude-averages the read frame with up to +/-3 neighbours
//          (phase of the blended frame kept) = spectral smear.
//   Scr  - Scrub: absolute read position, 0 = newest frame .. 1 = oldest.
//          Active when Scrub > 0 or its CV is mapped; otherwise the read head
//          follows live / Stretch.
//   Gat  - spectral gate: bins below the threshold are zeroed ("chop").
//   Mix  - dry/wet, equal-power.
//
// Latency / alignment: the wet hop emitted by a ProcessHop covers input
// samples [T-512, T-256) where T is the capture frontier (window completion =
// 512 samples = 10.7 ms), so the dry hop from the PREVIOUS ProcessHop is the
// matching content: dry is delayed exactly one hop and mixed hop-for-hop, so
// dry/wet stays time-aligned.  End-to-end wet latency is ~512-768 samples
// (11-16 ms) including ring priming.  If mainloop() stalls past the capture
// ring depth the block is dropped and counted (debug print / drops counter).
//
// MEMORY LAYOUT NOTE (MidSider build): GCC drops section attrs on template/
// inline members, so ALL heavy code lives in the non-template classes below
// with strong FLASHMEM symbols; SpecFreeze<Channels> is the thin registered
// wrapper.  UI strings use PSTR() so they land in flash (.progmem) instead of
// the DTCM .rodata pool (RAM1 stack budget is only ~19 KB).

static const char SF_NAME[] PROGMEM = "SpecFrz";

// ---------------------------------------------------------------------------
// SFStftEngine - STFT capture/resynthesis machinery.  Non-template so every
// method below is a strong FLASHMEM symbol (no template FLASHMEM no-op trap).
// Not an AudioStream; SFSpecFx adapts it to the audio graph.
// ---------------------------------------------------------------------------
class SFStftEngine {
public:
  static const int SF_N = 512;          // FFT size
  static const int SF_HOP = 256;        // 50% overlap
  static const int SF_BINS = 257;       // stored bins (DC..Nyquist)
  static const int SF_CAP = 2048;       // capture ring, samples per channel
  static const int SF_OUT = 2048;       // output ring, samples per channel
  static const int SF_FRAMES_PS = 128;  // frame history in PSRAM
  static const int SF_FRAMES_RAM = 16;  // RAM2 fallback

  bool begin();
  void end();
  bool IsPSRAM() const { return frames_psram_; }

  // -- audio ISR side (block-atomic, both channels per call) ----------------
  void PushInput(const int16_t* l, const int16_t* r); // AUDIO_BLOCK_SAMPLES
  bool PullOutput(int16_t* l, int16_t* r);            // false = starved

  // -- mainloop side --------------------------------------------------------
  void ProcessHops(int max_hops);
  void DebugReport(); // Serial, mainloop context only

  // -- control context (plain 32-bit stores; single writer) -----------------
  void setFreeze(bool f) { frozen_ = f; }
  void setStretch(float s) { stretch_ = s; }
  void setBlur(float b) { blur_ = b; }
  void setScrub(float pos01, bool active) {
    scrub_pos_ = pos01;
    scrub_active_ = active;
  }
  void setGateDb(float db) { gate_db_ = db; }
  void setMix(float m) { mix_ = m; }

  volatile uint32_t drops = 0;

private:
  static float SinApprox(float x); // trig-free, begin() only
  void Fft(float* re, float* im, bool inverse);
  void ProcessHop();
  float* FrameSlot(int idx, int ch) {
    return frames_ + ((((uint16_t)idx) & (frames_d_ - 1)) * 2 + ch) * (SF_BINS * 2);
  }
  const float* FrameSlot(int idx, int ch) const {
    return frames_ + ((((uint16_t)idx) & (frames_d_ - 1)) * 2 + ch) * (SF_BINS * 2);
  }

  // hot buffers (RAM2 malloc, one block split by hand)
  int16_t* cap_[2] = {nullptr, nullptr};  // capture ring (ISR -> mainloop)
  int16_t* out_[2] = {nullptr, nullptr};  // output ring (mainloop -> ISR)
  float* win_ = nullptr;                  // sqrt-Hann, SF_N
  float* tw_re_ = nullptr;                // twiddles, SF_N/2
  float* tw_im_ = nullptr;
  uint16_t* bitrev_ = nullptr;            // SF_N
  float* work_re_ = nullptr;              // SF_N
  float* work_im_ = nullptr;              // SF_N
  float* hist_[2] = {nullptr, nullptr};   // input history, SF_N each
  float* ola_[2] = {nullptr, nullptr};    // OLA buffer, SF_N each
  int16_t dry_dly_[2][SF_HOP];            // previous hop's dry (aligned emit)
  int16_t dry_cur_[2][SF_HOP];            // this hop's dry
  uint8_t* hot_ = nullptr;

  // frame history (PSRAM or RAM2)
  float* frames_ = nullptr;
  int frames_d_ = 0;
  bool frames_psram_ = false;

  // SPSC ring indices (free-running uint16; capacities divide 65536)
  volatile uint16_t cap_w_ = 0, cap_r_ = 0;
  volatile uint16_t out_w_ = 0, out_r_ = 0;

  int write_idx_ = 0;
  float read_pos_ = 0.0f; // frames behind newest
  bool frozen_ = false;
  bool scrub_active_ = false;
  float stretch_ = 1.0f;
  float blur_ = 0.0f;
  float scrub_pos_ = 0.0f;
  float gate_db_ = -90.0f;
  float mix_ = 1.0f;

  // profiling
  uint32_t cyc_sum_ = 0, cyc_max_ = 0, cyc_n_ = 0, report_ms_ = 0;
};

// ---------------------------------------------------------------------------
// SFSpecFx - thin AudioStream adapter (2 in / 2 out).  update() is FLASHMEM
// like effect_looper.cpp: the ITCM bank budget is exhausted.
// ---------------------------------------------------------------------------
class SFSpecFx : public AudioStream {
public:
  explicit SFSpecFx(SFStftEngine& e);

  void update() override;

private:
  SFStftEngine& engine;
  audio_block_t* inputQueueArray[2];
};

// ---------------------------------------------------------------------------
// SpecFreezeCore - all applet state + UI + param plumbing.
// ---------------------------------------------------------------------------
class SpecFreezeCore : public HemisphereAudioApplet {
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
    // Page 0: freeze / stretch / blur
    FRZ_SRC = 0, STRETCH, STRETCH_CV,
    BLUR, BLUR_CV,
    // Page 1: scrub / gate / mix
    SCRUB, SCRUB_CV,
    GATE, GATE_CV,
    MIX, MIX_CV,
    CURSOR_LENGTH,
  };

  int8_t cursor = FRZ_SRC;

  uint8_t stretch = 0;   // 0..100 -> 1x..50x log
  uint8_t blur = 0;      // 0..100%
  uint8_t scrub = 0;     // 0..255 -> 0..1 of the frame history
  int8_t gate_db = -90;  // -90..0 dB
  uint8_t mix = 100;     // 0..100% dry..wet

  CVInputMap stretch_cv;
  CVInputMap blur_cv;
  CVInputMap scrub_cv;
  CVInputMap gate_cv;
  CVInputMap mix_cv;
  DigitalInputMap freeze_input;
  bool manual_freeze = false; // AuxButton latch

  bool alloc_ok = false;

  SFStftEngine engine;
  SFSpecFx fx{engine};
  AudioPassthrough<2> input;
  AudioPassthrough<2> output;
};

// Thin template wrapper so the registry can use the SpecFreeze<STEREO> form.
// (All real work lives in the non-template core so FLASHMEM placement survives
// this toolchain; Channels is intentionally unused.)
template <AudioChannels Channels>
class SpecFreeze : public SpecFreezeCore {
public:
  const char* applet_name() {
    return SF_NAME;
  }
};

// -- Out-of-line definitions (strong symbols, FLASHMEM honored) --------------

FLASHMEM float SFStftEngine::SinApprox(float x) {
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

FLASHMEM bool SFStftEngine::begin() {
  end();

  // ---- hot buffers (RAM2 heap) --------------------------------------------
  const size_t hot_bytes =
    2 * SF_CAP * sizeof(int16_t) +          // cap rings
    2 * SF_OUT * sizeof(int16_t) +          // out rings
    SF_N * sizeof(float) +                  // window
    (SF_N / 2) * sizeof(float) +            // tw_re
    (SF_N / 2) * sizeof(float) +            // tw_im
    SF_N * sizeof(uint16_t) +               // bitrev
    SF_N * sizeof(float) +                  // work_re
    SF_N * sizeof(float) +                  // work_im
    2 * SF_N * sizeof(float) +              // hist
    2 * SF_N * sizeof(float) +              // ola
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
    cap_[c] = reinterpret_cast<int16_t*>(take(SF_CAP * sizeof(int16_t)));
  for (int c = 0; c < 2; ++c)
    out_[c] = reinterpret_cast<int16_t*>(take(SF_OUT * sizeof(int16_t)));
  win_ = reinterpret_cast<float*>(take(SF_N * sizeof(float)));
  tw_re_ = reinterpret_cast<float*>(take((SF_N / 2) * sizeof(float)));
  tw_im_ = reinterpret_cast<float*>(take((SF_N / 2) * sizeof(float)));
  bitrev_ = reinterpret_cast<uint16_t*>(take(SF_N * sizeof(uint16_t)));
  work_re_ = reinterpret_cast<float*>(take(SF_N * sizeof(float)));
  work_im_ = reinterpret_cast<float*>(take(SF_N * sizeof(float)));
  for (int c = 0; c < 2; ++c)
    hist_[c] = reinterpret_cast<float*>(take(SF_N * sizeof(float)));
  for (int c = 0; c < 2; ++c)
    ola_[c] = reinterpret_cast<float*>(take(SF_N * sizeof(float)));

  // ---- frame history (PSRAM preferred, RAM2 fallback) ---------------------
  frames_d_ = SF_FRAMES_PS;
  frames_psram_ = false;
  frames_ = nullptr;
  if (external_psram_size > 0) {
    frames_ = static_cast<float*>(
      extmem_malloc(frames_d_ * 2 * SF_BINS * 2 * sizeof(float)));
    if (frames_) {
      // extmem_calloc/malloc does NOT guarantee zero-fill on this core
      memset(frames_, 0, frames_d_ * 2 * SF_BINS * 2 * sizeof(float));
      frames_psram_ = true;
    }
  }
  if (!frames_) {
    frames_d_ = SF_FRAMES_RAM;
    frames_ = static_cast<float*>(
      malloc(frames_d_ * 2 * SF_BINS * 2 * sizeof(float)));
    if (!frames_) {
      end();
      return false;
    }
    memset(frames_, 0, frames_d_ * 2 * SF_BINS * 2 * sizeof(float));
  }

  // ---- sqrt-Hann window (trig-free) --------------------------------------
  for (int i = 0; i < SF_N; ++i) {
    const float w = 0.5f - 0.5f * SinApprox(6.28318530718f * i / SF_N);
    win_[i] = __builtin_sqrtf(w);
  }

  // ---- twiddles + bit-reversal (FFT scratch, RAM2) -----------------------
  for (int k = 0; k < SF_N / 2; ++k) {
    const float a = -6.28318530718f * k / SF_N;
    tw_re_[k] = SinApprox(a + 1.57079632679f); // cos(a)
    tw_im_[k] = SinApprox(a);                  // sin(a)
  }
  int bits = 0;
  while ((1 << bits) < SF_N) ++bits;
  for (int i = 0; i < SF_N; ++i) {
    int r = 0;
    for (int b = 0; b < bits; ++b)
      if (i & (1 << b)) r |= 1 << (bits - 1 - b);
    bitrev_[i] = static_cast<uint16_t>(r);
  }

  for (int c = 0; c < 2; ++c) {
    memset(hist_[c], 0, SF_N * sizeof(float));
    memset(ola_[c], 0, SF_N * sizeof(float));
    memset(cap_[c], 0, SF_CAP * sizeof(int16_t));
    memset(out_[c], 0, SF_OUT * sizeof(int16_t));
    memset(dry_dly_[c], 0, sizeof(dry_dly_[c]));
    memset(dry_cur_[c], 0, sizeof(dry_cur_[c]));
  }
  cap_w_ = cap_r_ = out_w_ = out_r_ = 0;
  write_idx_ = 0;
  read_pos_ = 0.0f;
  drops = 0;
  cyc_sum_ = cyc_max_ = cyc_n_ = 0;

  // enable the cycle counter for the debug instrumentation
  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
  return true;
}

FLASHMEM void SFStftEngine::end() {
  if (frames_) {
    if (frames_psram_) extmem_free(frames_);
    else free(frames_);
    frames_ = nullptr;
  }
  if (hot_) {
    free(hot_);
    hot_ = nullptr;
  }
  frames_d_ = 0;
  frames_psram_ = false;
}

FLASHMEM void SFStftEngine::PushInput(const int16_t* l, const int16_t* r) {
  if (static_cast<uint16_t>(cap_w_ - cap_r_) + AUDIO_BLOCK_SAMPLES > SF_CAP) {
    ++drops; // capture starved (mainloop stalled) - drop the whole block pair
    return;
  }
  const uint16_t w = cap_w_;
  const uint16_t mask = SF_CAP - 1;
  int16_t* rl = cap_[0];
  int16_t* rr = cap_[1];
  for (int i = 0; i < AUDIO_BLOCK_SAMPLES; ++i) {
    rl[(uint16_t)(w + i) & mask] = l ? l[i] : 0;
    rr[(uint16_t)(w + i) & mask] = r ? r[i] : 0;
  }
  cap_w_ = static_cast<uint16_t>(w + AUDIO_BLOCK_SAMPLES); // commit both
}

FLASHMEM bool SFStftEngine::PullOutput(int16_t* l, int16_t* r) {
  if (static_cast<uint16_t>(out_w_ - out_r_) < AUDIO_BLOCK_SAMPLES) return false;
  const uint16_t rd = out_r_;
  const uint16_t mask = SF_OUT - 1;
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
FLASHMEM void SFStftEngine::Fft(float* re, float* im, bool inverse) {
  for (int i = 0; i < SF_N; ++i) {
    const int j = bitrev_[i];
    if (j > i) {
      float t = re[i]; re[i] = re[j]; re[j] = t;
      t = im[i]; im[i] = im[j]; im[j] = t;
    }
  }
  for (int len = 2; len <= SF_N; len <<= 1) {
    const int half = len >> 1;
    const int step = SF_N / len;
    for (int i = 0; i < SF_N; i += len) {
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
    const float s = 1.0f / SF_N;
    for (int i = 0; i < SF_N; ++i) {
      re[i] *= s;
      im[i] *= s;
    }
  }
}

FLASHMEM void SFStftEngine::ProcessHop() {
  // ---- stash dry (previous hop is the content matching this wet) ----------
  for (int c = 0; c < 2; ++c)
    memcpy(dry_dly_[c], dry_cur_[c], SF_HOP * sizeof(int16_t));

  // ---- pop one hop per channel; slide analysis history --------------------
  for (int c = 0; c < 2; ++c) {
    const int16_t* ring = cap_[c];
    const uint16_t r = cap_r_;
    const uint16_t mask = SF_CAP - 1;
    float* h = hist_[c];
    memcpy(h, h + SF_HOP, (SF_N - SF_HOP) * sizeof(float));
    float* dst = h + (SF_N - SF_HOP);
    for (int i = 0; i < SF_HOP; ++i) {
      const int16_t s = ring[(uint16_t)(r + i) & mask];
      dry_cur_[c][i] = s;
      dst[i] = static_cast<float>(s);
    }
  }
  cap_r_ = static_cast<uint16_t>(cap_r_ + SF_HOP);

  // ---- analysis: window + FFT, append frame to the rolling capture --------
  if (!frozen_) {
    for (int c = 0; c < 2; ++c) {
      const float* h = hist_[c];
      for (int i = 0; i < SF_N; ++i) {
        work_re_[i] = h[i] * win_[i];
        work_im_[i] = 0.0f;
      }
      Fft(work_re_, work_im_, false);
      float* fr = FrameSlot(write_idx_, c);
      for (int k = 0; k < SF_BINS; ++k) {
        fr[2 * k] = work_re_[k];
        fr[2 * k + 1] = work_im_[k];
      }
    }
    ++write_idx_;
    // read_pos_ is "frames behind newest": a new frame pushes every stored
    // frame one step further behind
    read_pos_ += 1.0f;
  }

  // ---- read head ----------------------------------------------------------
  const int depth = frames_d_;
  if (frozen_ || scrub_active_) {
    // absolute positioning inside the (frozen) capture window
    read_pos_ = scrub_pos_ * static_cast<float>(depth - 2);
    if (read_pos_ < 0.0f) read_pos_ = 0.0f;
    if (read_pos_ > depth - 2) read_pos_ = depth - 2;
  } else {
    // stretch: advance toward the live edge 1/stretch frames per hop
    read_pos_ -= 1.0f / stretch_;
    if (read_pos_ < 0.0f) read_pos_ = 0.0f;      // live edge
    if (read_pos_ > depth - 2) read_pos_ = 0.0f; // trailed off: re-sync seam
  }

  const int f0 = static_cast<int>(read_pos_);
  const float ft = read_pos_ - static_cast<float>(f0);
  const float gate_lin = dbToScalar(gate_db_);
  const float gate2 = gate_lin * gate_lin;
  const int kblur = blur_ > 0.01f
    ? (blur_ < 0.34f ? 1 : (blur_ < 0.67f ? 2 : 3))
    : 0;

  // ---- resynthesis --------------------------------------------------------
  float wet_gain, dry_gain;
  EqualPowerFade(dry_gain, wet_gain, mix_);

  for (int c = 0; c < 2; ++c) {
    const float* fa = FrameSlot(write_idx_ - 1 - f0, c); // newer
    const float* fb = FrameSlot(write_idx_ - 2 - f0, c); // older
    float* re = work_re_;
    float* im = work_im_;

    for (int k = 0; k < SF_BINS; ++k) {
      const float ar = fa[2 * k], ai = fa[2 * k + 1];
      const float br = fb[2 * k], bi = fb[2 * k + 1];
      const float ma = __builtin_sqrtf(ar * ar + ai * ai);
      const float mb = __builtin_sqrtf(br * br + bi * bi);

      // magnitude lerp + chord-interpolated unit phasor (wrap-safe, trig-free
      // stand-in for linear phase interpolation)
      float mag = ma + (mb - ma) * ft;
      float rr, ii;
      if (ma > 1e-6f && mb > 1e-6f) {
        float ur = ar / ma * (1.0f - ft) + br / mb * ft;
        float ui = ai / ma * (1.0f - ft) + bi / mb * ft;
        const float un = __builtin_sqrtf(ur * ur + ui * ui);
        if (un > 1e-6f) {
          rr = ur / un * mag;
          ii = ui / un * mag;
        } else {
          rr = ar * (1.0f - ft) + br * ft;
          ii = ai * (1.0f - ft) + bi * ft;
        }
      } else {
        rr = ar * (1.0f - ft) + br * ft;
        ii = ai * (1.0f - ft) + bi * ft;
      }

      // blur: magnitude smear over neighbouring frames, phase preserved
      if (kblur > 0) {
        float msum = mag;
        float wsum = 1.0f;
        for (int d = 1; d <= kblur; ++d) {
          for (int s = -1; s <= 1; s += 2) {
            const float* fn = FrameSlot(write_idx_ - 1 - f0 + s * d, c);
            const float nr = fn[2 * k], ni = fn[2 * k + 1];
            msum += __builtin_sqrtf(nr * nr + ni * ni);
            wsum += 1.0f;
          }
        }
        const float mavg = msum / wsum;
        const float mc = __builtin_sqrtf(rr * rr + ii * ii);
        if (mc > 1e-6f) {
          const float g = mavg / mc;
          rr *= g;
          ii *= g;
        }
      }

      // spectral gate ("chop")
      if (rr * rr + ii * ii < gate2) {
        rr = 0.0f;
        ii = 0.0f;
      }
      re[k] = rr;
      im[k] = ii;
    }
    // rebuild conjugate mirror for the inverse FFT
    for (int k = 1; k < SF_BINS - 1; ++k) {
      re[SF_N - k] = re[k];
      im[SF_N - k] = -im[k];
    }
    im[0] = 0.0f;
    im[SF_BINS - 1] = 0.0f;

    Fft(re, im, true);

    // synthesis window + overlap-add, emit one hop
    float* ola = ola_[c];
    for (int i = 0; i < SF_N; ++i) ola[i] += re[i] * win_[i];
    int16_t* oring = out_[c];
    const uint16_t w = out_w_;
    const uint16_t mask = SF_OUT - 1;
    const int16_t* dly = dry_dly_[c]; // hop content matching this wet hop
    for (int i = 0; i < SF_HOP; ++i) {
      const float wet = ola[i];
      const float dry = static_cast<float>(dly[i]);
      oring[(uint16_t)(w + i) & mask] =
        Clip16(wet_gain * wet + dry_gain * dry);
    }
    memcpy(ola, ola + SF_HOP, (SF_N - SF_HOP) * sizeof(float));
    memset(ola + (SF_N - SF_HOP), 0, SF_HOP * sizeof(float));
  }
  out_w_ = static_cast<uint16_t>(out_w_ + SF_HOP);
}

FLASHMEM void SFStftEngine::ProcessHops(int max_hops) {
  for (int h = 0; h < max_hops; ++h) {
    if (static_cast<uint16_t>(cap_w_ - cap_r_) < SF_HOP) break;
    if (static_cast<uint16_t>(out_w_ - out_r_) + SF_HOP > SF_OUT) break;
    const uint32_t t0 = ARM_DWT_CYCCNT;
    ProcessHop();
    const uint32_t dt = ARM_DWT_CYCCNT - t0;
    cyc_sum_ += dt;
    ++cyc_n_;
    if (dt > cyc_max_) cyc_max_ = dt;
  }
}

FLASHMEM void SFStftEngine::DebugReport() {
  const uint32_t now = millis();
  if (now - report_ms_ < 2000) return;
  report_ms_ = now;
  if (cyc_n_ == 0) return;
  const uint32_t avg = cyc_sum_ / cyc_n_;
  // 600 MHz core: cycles / 600 = microseconds
  Serial.print(PSTR("SF hop: avg "));
  Serial.print(avg / 600);
  Serial.print(PSTR(" us, max "));
  Serial.print(cyc_max_ / 600);
  Serial.print(PSTR(" us, n="));
  Serial.print(cyc_n_);
  Serial.print(PSTR(", drops="));
  Serial.println(drops);
  cyc_sum_ = cyc_max_ = cyc_n_ = 0;
}

// -- SFSpecFx (audio ISR) ----------------------------------------------------

FLASHMEM SFSpecFx::SFSpecFx(SFStftEngine& e)
  : AudioStream(2, inputQueueArray), engine(e) {}

FLASHMEM void SFSpecFx::update() {
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

// -- SpecFreezeCore ----------------------------------------------------------

FLASHMEM void SpecFreezeCore::Start() {
  alloc_ok = engine.begin();
  if (alloc_ok) {
    PatchCable(input, 0, fx, 0);
    PatchCable(input, 1, fx, 1);
    PatchCable(fx, 0, output, 0);
    PatchCable(fx, 1, output, 1);
    engine.setFreeze(freeze_input.Gate() || manual_freeze);
    engine.setStretch(1.0f);
  }
}

FLASHMEM void SpecFreezeCore::Unload() {
  engine.end();
  alloc_ok = false;
  AllowRestart();
}

FLASHMEM void SpecFreezeCore::mainloop() {
  if (!alloc_ok) return;
  engine.ProcessHops(4); // catch up after UI stalls, bounded
  engine.DebugReport();
}

FLASHMEM void SpecFreezeCore::Controller() {
  if (!alloc_ok) return;

  // effective params = knob + CV (bipolar via attenuversion)
  const float eff_stretch = constrain(
    stretch + stretch_cv.InF() * 50.0f, 0.0f, 100.0f);
  const float eff_blur = constrain(blur + blur_cv.InF() * 50.0f, 0.0f, 100.0f);
  const float eff_scrub = constrain(
    static_cast<float>(scrub) + scrub_cv.InF() * 128.0f, 0.0f, 255.0f);
  const float eff_gate = constrain(
    static_cast<float>(gate_db) + gate_cv.InF() * 45.0f, -90.0f, 0.0f);
  const float eff_mix = constrain(mix + mix_cv.InF() * 50.0f, 0.0f, 100.0f);

  // 0..100 -> 1x..50x (log): 2^(u*5.644/100) ... u in 0..100
  engine.setStretch(fastpow2(eff_stretch * 0.05644f));
  engine.setBlur(eff_blur * 0.01f);
  engine.setScrub(eff_scrub / 255.0f, eff_scrub > 0.5f || scrub_cv.enabled());
  engine.setGateDb(eff_gate);
  engine.setMix(eff_mix * 0.01f);
  engine.setFreeze(freeze_input.Gate() || manual_freeze);
}

FLASHMEM void SpecFreezeCore::View() {
  if (!alloc_ok) {
    gfxPrint(2, 15, PSTR("Out of RAM!!"));
    return;
  }

  const int label_x = 1;
  const int page = (cursor >= SCRUB);
  gfxRect(page * 15, 10, 14, 3); // page tab

  switch (page) {
    case 0:
      // Freeze row: label inverts while latched (print first, then invert)
      gfxPrint(label_x, 15, PSTR("Frz:"));
      if (manual_freeze) gfxInvert(1, 15, 20, 8);
      gfxStartCursor();
      gfxPrint(freeze_input);
      gfxEndCursor(cursor == FRZ_SRC, true, freeze_input.InputName());

      gfxPrint(label_x, 25, PSTR("Str:"));
      gfxStartCursor();
      gfxPrint(fastpow2(stretch * 0.05644f), 1);
      gfxPrint(PSTR("x"));
      gfxEndCursor(cursor == STRETCH);
      gfxStartCursor();
      gfxPrint(stretch_cv);
      gfxEndCursor(cursor == STRETCH_CV, false, stretch_cv.InputName());

      gfxPrint(label_x, 35, PSTR("Blu:"));
      gfxStartCursor();
      graphics.printf(PSTR("%3d%%"), blur);
      gfxEndCursor(cursor == BLUR);
      gfxStartCursor();
      gfxPrint(blur_cv);
      gfxEndCursor(cursor == BLUR_CV, false, blur_cv.InputName());

      // frame read-head indicator (0 = live edge)
      gfxPrint(label_x, 45, PSTR("Pos:"));
      gfxDottedLine(24, 48, 60, 48);
      gfxInvert(24 + Proportion(scrub, 255, 34), 45, 2, 7);
      break;

    case 1:
      gfxPrint(label_x, 15, PSTR("Scr:"));
      gfxStartCursor();
      graphics.printf(PSTR("%3d%%"), scrub * 100 / 255);
      gfxEndCursor(cursor == SCRUB);
      gfxStartCursor();
      gfxPrint(scrub_cv);
      gfxEndCursor(cursor == SCRUB_CV, false, scrub_cv.InputName());

      gfxPrint(label_x, 25, PSTR("Gat:"));
      gfxStartCursor();
      graphics.printf(PSTR("%4ddB"), gate_db);
      gfxEndCursor(cursor == GATE);
      gfxStartCursor();
      gfxPrint(gate_cv);
      gfxEndCursor(cursor == GATE_CV, false, gate_cv.InputName());

      gfxPrint(label_x, 35, PSTR("Mix:"));
      gfxStartCursor();
      graphics.printf(PSTR("%3d%%"), mix);
      gfxEndCursor(cursor == MIX);
      gfxStartCursor();
      gfxPrint(mix_cv);
      gfxEndCursor(cursor == MIX_CV, false, mix_cv.InputName());

      // buffer depth note ("No PSRAM" precedent)
      gfxPrint(label_x, 45, engine.IsPSRAM() ? PSTR("128fr") : PSTR("16fr!"));
      break;
  }

  gfxDisplayInputMapEditor();
}

FLASHMEM void SpecFreezeCore::OnButtonPress() {
  if (CheckEditInputMapPress(
        cursor,
        IndexedInput(FRZ_SRC, freeze_input),
        IndexedInput(STRETCH_CV, stretch_cv),
        IndexedInput(BLUR_CV, blur_cv),
        IndexedInput(SCRUB_CV, scrub_cv),
        IndexedInput(GATE_CV, gate_cv),
        IndexedInput(MIX_CV, mix_cv)
      ))
    return;
  CursorToggle();
}

FLASHMEM void SpecFreezeCore::AuxButton() {
  manual_freeze ^= 1; // latch/unlatch manual freeze (performance use)
}

FLASHMEM void SpecFreezeCore::OnEncoderMove(int direction) {
  if (!EditMode()) {
    MoveCursor(cursor, direction, CURSOR_LENGTH - 1);
    return;
  }
  if (EditSelectedInputMap(direction)) return;

  switch (cursor) {
    case FRZ_SRC: freeze_input.ChangeSource(direction); break;
    case STRETCH: stretch = constrain(stretch + direction, 0, 100); break;
    case STRETCH_CV: stretch_cv.ChangeSource(direction); break;
    case BLUR: blur = constrain(blur + direction, 0, 100); break;
    case BLUR_CV: blur_cv.ChangeSource(direction); break;

    case SCRUB: scrub = constrain(scrub + direction * 2, 0, 255); break;
    case SCRUB_CV: scrub_cv.ChangeSource(direction); break;
    case GATE: gate_db = constrain(gate_db + direction, -90, 0); break;
    case GATE_CV: gate_cv.ChangeSource(direction); break;
    case MIX: mix = constrain(mix + direction, 0, 100); break;
    case MIX_CV: mix_cv.ChangeSource(direction); break;

    default: return;
  }
}

// 256-bit state layout (4 x 64-bit words), everything in-bounds:
//   word 0 (37b): stretch(7) blur(7) scrub(8) gate_db(8) mix(7)
//   word 1 (64b): stretch_cv blur_cv scrub_cv gate_cv (4 x 16)
//   word 2 (48b): mix_cv(16) freeze_input(32)
//   word 3 (0b):  spare (0)
FLASHMEM void SpecFreezeCore::OnDataRequest(std::array<uint64_t, CONFIG_SIZE>& data) {
  data[0] = PackPackables(
    pack<7>(stretch),
    pack<7>(blur),
    pack<8>(scrub),
    gate_db,
    pack<7>(mix)
  );
  data[1] = PackPackables(stretch_cv, blur_cv, scrub_cv, gate_cv);
  data[2] = PackPackables(mix_cv, freeze_input);
  data[3] = 0;
}

FLASHMEM void SpecFreezeCore::OnDataReceive(
  const std::array<uint64_t, CONFIG_SIZE>& data
) {
  UnpackPackables(
    data[0],
    pack<7>(stretch),
    pack<7>(blur),
    pack<8>(scrub),
    gate_db,
    pack<7>(mix)
  );
  UnpackPackables(data[1], stretch_cv, blur_cv, scrub_cv, gate_cv);
  UnpackPackables(data[2], mix_cv, freeze_input);

  // re-derive packed ranges defensively
  stretch = constrain(stretch, 0, 100);
  blur = constrain(blur, 0, 100);
  scrub = constrain(scrub, 0, 255);
  gate_db = constrain(gate_db, -90, 0);
  mix = constrain(mix, 0, 100);
}

FLASHMEM void SpecFreezeCore::SetHelp() {
  help[HELP_DIGITAL1] = PSTR("Frz gate");
  help[HELP_DIGITAL2] = PSTR(" - ");
  help[HELP_CV1] = PSTR("Stretch");
  help[HELP_CV2] = PSTR("Scrub");
  help[HELP_OUT1] = PSTR("Out L");
  help[HELP_OUT2] = PSTR("Out R");
  help[HELP_EXTRA1] = PSTR("Aux = frz latch");
  help[HELP_EXTRA2] = PSTR("STFT 512/256");
}
