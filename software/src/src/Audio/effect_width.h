/* Audio Library for Teensy 3.X / 4.X
 * Mid-Side Width Dynamics - side-channel gain keyed by a detector feed
 *
 * Created for Phazerville Suite (generated with AI assistance).
 *
 * Envelope follower / soft-knee gain computer math cribbed from
 * AudioEffectDynamics by Marc Paquette (see effect_dynamics.h), originally
 * published under the MIT license:
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef effect_width_h_
#define effect_width_h_

#if !defined(KINETISL)

#include <Arduino.h>
#include <AudioStream.h>

// Computes RBJ Audio-EQ-Cookbook biquad coefficients, a0-normalized:
//   coef = {b0, b1, b2, a1, a2}
// mode: 0 = LP, 1 = HP, 2 = BP, 3 = Notch.
//
// Trig-free on purpose: libm's sin/cos pull ~4 KB of __kernel_* reduction
// code into RAM1/ITCM (where this multiboot build runs all default .text),
// which blows the 32 KB ITCM bank budget. A 9th-order Taylor sine with
// quadrant folding is exact to ~1e-6 for filter-coefficient math.
void WidthDynamicsBiquadCoeffs(int mode, float freq, float q, float coef[5]);

// 2-in / 1-out dynamics stage for the Side channel of a Mid-Side matrix:
//   input 0 = detector feed (e.g. filtered Mid signal; not in the audio path)
//   input 1 = S audio
//   output  = S audio with width gain applied
//
// Per-sample math is float32 and allocation-free in update().
class AudioEffectWidthDynamics : public AudioStream {
public:
  AudioEffectWidthDynamics(void) : AudioStream(2, inputQueueArray) {
    keyFilter(1000.0f, 1.0f, 2);
    dynamics(-20.0f, 4.0f, 0.05f, 0.3f);
    amount(1.0f);
    reset();
  }

  // Detector key filter on input 0 only (band-keyed ducking, de-esser style).
  // mode: 0 = LP, 1 = HP, 2 = BP, 3 = Notch. RBJ biquad, direct form I.
  void keyFilter(float freq, float q, int mode);

  // threshold_db: [-90, 0], ratio: [1, 60] (x:1), attack/release in seconds.
  // Knee is fixed at 6 dB (soft), matching AudioEffectDynamics' default.
  void dynamics(float threshold_db, float ratio, float attack_s, float release_s);

  // Width Envelope Amount, [0, 1]:
  //   0 = Side untouched, 1 = Side gain fully follows the gain computer.
  void amount(float amt);

  // Smoothed gain-computer output in dB (<= 0), for the UI meter.
  float get_gain_db() const { return compdb_; }

  void reset();

private:
  // NB: effect_dynamics.h macros MIN_DB/MAX_DB/MIN_T/MAX_T are deliberately
  // not reused here; keep this translation unit self-contained.
  static constexpr float WD_MIN_DB = -90.0f;
  static constexpr float WD_MAX_DB = 0.0f;
  static constexpr float WD_MIN_T = 0.003f;
  static constexpr float WD_MAX_T = 4.0f;
  static constexpr float WD_KNEE = 6.0f;

  audio_block_t *inputQueueArray[2];

  // key filter state (direct form I)
  float kf_b0_ = 0.25f, kf_b1_ = 0.5f, kf_b2_ = 0.25f, kf_a1_ = 0.0f, kf_a2_ = 0.0f;
  float kf_x1_ = 0.0f, kf_x2_ = 0.0f, kf_y1_ = 0.0f, kf_y2_ = 0.0f;

  // envelope follower (peak, per-sample one-pole)
  float env_ = 0.0f;
  float a_env_att_ = 0.0f, a_env_rel_ = 0.0f;

  // soft-knee gain computer (same form as effect_dynamics.cpp)
  float threshold_db_ = -20.0f;
  float comp_ratio_ = 0.25f;  // 1 / ratio
  float knee_ratio_ = -0.75f; // comp_ratio - 1
  float two_knee_width_ = 1.0f / (WD_KNEE * 2.0f);
  float low_knee_ = -23.0f, high_knee_ = -17.0f;
  float compdb_ = 0.0f;
  float a_comp_att_ = 0.0f, a_comp_rel_ = 0.0f;

  // width application
  float amount_ = 1.0f;
  float width_gain_ = 1.0f;

  // 10% -> 90% smoothing constant, same convention as AudioEffectDynamics,
  // evaluated per sample (effect_dynamics evaluates its alphas per block).
  static float timeToAlpha(float time) {
    return expf(-0.9542f / (AUDIO_SAMPLE_RATE_EXACT * time));
  }

  virtual void update(void);
};

#endif // !KINETISL

#endif // effect_width_h_
