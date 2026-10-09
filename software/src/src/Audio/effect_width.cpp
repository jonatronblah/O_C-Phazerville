/* Audio Library for Teensy 4.X
 * Mid-Side Width Dynamics - side-channel gain keyed by a detector feed
 * Created for Phazerville Suite (generated with AI assistance).
 *
 * Gain computer / envelope smoothing adapted from effect_dynamics.cpp
 * (Marc Paquette / PJRC Audio Library, MIT licensed).
 */
#if defined(ARDUINO_TEENSY41)

#include "effect_width.h"
#include "../../dsputils.h" // Clip16, dbToScalar, scalarToDb (fast exp/log)

// --- trig-free sine (see effect_width.h for why libm is avoided) -----------

// sin(x) for |x| <= pi/2, 9th-order Taylor (max error ~1e-6)
static float sin_poly(float x) {
  const float x2 = x * x;
  return x * (1.0f + x2 * (-1.0f / 6.0f
    + x2 * (1.0f / 120.0f
    + x2 * (-1.0f / 5040.0f
    + x2 * (1.0f / 362880.0f)))));
}

// sin(x) for any x in [-2pi, 2pi] via quadrant folding (no range-reduction
// libm calls; the while loops iterate at most once for our inputs)
static float sine_approx(float x) {
  const float kPI = 3.14159265f;
  const float kTWO_PI = 6.28318531f;
  while (x > kPI) x -= kTWO_PI;
  while (x < -kPI) x += kTWO_PI;
  const bool neg = x < 0.0f;
  if (neg) x = -x;
  if (x > 1.57079633f) x = kPI - x; // fold [pi/2, pi]
  const float s = sin_poly(x);
  return neg ? -s : s;
}

FLASHMEM void WidthDynamicsBiquadCoeffs(int mode, float freq, float q, float coef[5]) {
  if (!(freq > 0.0f)) freq = 1000.0f;
  if (freq < 20.0f) freq = 20.0f;
  if (freq > 20000.0f) freq = 20000.0f;
  if (q < 0.5f) q = 0.5f;
  if (q > 8.0f) q = 8.0f;
  if (mode < 0 || mode > 3) mode = 0;

  const float w0 = freq * (2.0f * 3.14159265358979f / AUDIO_SAMPLE_RATE_EXACT);
  const float sn = sine_approx(w0);
  const float cs = sine_approx(w0 + 1.57079633f); // cos(x) = sin(x + pi/2)
  const float alpha = sn / (2.0f * q);
  float b0, b1, b2;
  switch (mode) {
    case 1: // highpass
      b0 = (1.0f + cs) * 0.5f;
      b1 = -(1.0f + cs);
      b2 = b0;
      break;
    case 2: // bandpass (0 dB peak)
      b0 = alpha;
      b1 = 0.0f;
      b2 = -alpha;
      break;
    case 3: // notch
      b0 = 1.0f;
      b1 = -2.0f * cs;
      b2 = 1.0f;
      break;
    default: // lowpass
      b0 = (1.0f - cs) * 0.5f;
      b1 = 1.0f - cs;
      b2 = b0;
      break;
  }
  const float a0 = 1.0f + alpha;
  coef[0] = b0 / a0;
  coef[1] = b1 / a0;
  coef[2] = b2 / a0;
  coef[3] = -2.0f * cs / a0;
  coef[4] = (1.0f - alpha) / a0;
}

FLASHMEM void AudioEffectWidthDynamics::keyFilter(float freq, float q, int mode) {
  float coef[5];
  WidthDynamicsBiquadCoeffs(mode, freq, q, coef);
  kf_b0_ = coef[0];
  kf_b1_ = coef[1];
  kf_b2_ = coef[2];
  kf_a1_ = coef[3];
  kf_a2_ = coef[4];
}

FLASHMEM void AudioEffectWidthDynamics::dynamics(
  float threshold_db, float ratio, float attack_s, float release_s
) {
  if (threshold_db < WD_MIN_DB) threshold_db = WD_MIN_DB;
  if (threshold_db > WD_MAX_DB) threshold_db = WD_MAX_DB;
  if (ratio < 1.0f) ratio = 1.0f;
  if (ratio > 60.0f) ratio = 60.0f;
  if (attack_s < WD_MIN_T) attack_s = WD_MIN_T;
  if (attack_s > WD_MAX_T) attack_s = WD_MAX_T;
  if (release_s < WD_MIN_T) release_s = WD_MIN_T;
  if (release_s > WD_MAX_T) release_s = WD_MAX_T;

  threshold_db_ = threshold_db;
  comp_ratio_ = 1.0f / ratio;
  knee_ratio_ = comp_ratio_ - 1.0f;
  two_knee_width_ = 1.0f / (WD_KNEE * 2.0f);
  low_knee_ = threshold_db - WD_KNEE * 0.5f;
  high_knee_ = threshold_db + WD_KNEE * 0.5f;

  a_env_att_ = timeToAlpha(attack_s);
  a_env_rel_ = timeToAlpha(release_s);
  a_comp_att_ = timeToAlpha(attack_s);
  a_comp_rel_ = timeToAlpha(release_s);
}

void AudioEffectWidthDynamics::amount(float amt) {
  if (amt < 0.0f) amt = 0.0f;
  if (amt > 1.0f) amt = 1.0f;
  amount_ = amt;
}

void AudioEffectWidthDynamics::reset() {
  kf_x1_ = kf_x2_ = kf_y1_ = kf_y2_ = 0.0f;
  env_ = 0.0f;
  compdb_ = 0.0f;
  width_gain_ = 1.0f;
}

void AudioEffectWidthDynamics::update(void) {
  audio_block_t *det = receiveReadOnly(0);
  audio_block_t *side = receiveWritable(1);
  if (!side) {
    if (det) release(det);
    return;
  }
  if (!det) {
    // No detector feed; leave Side untouched
    transmit(side);
    release(side);
    return;
  }

  float env = env_;
  float compdb = compdb_;
  float wgain = width_gain_;
  float x1 = kf_x1_, x2 = kf_x2_, y1 = kf_y1_, y2 = kf_y2_;

  for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
    // --- detector key filter (input 0 only, never in the S audio path) ---
    float d = det->data[i] * (1.0f / 32768.0f);
    float y = kf_b0_ * d + kf_b1_ * x1 + kf_b2_ * x2 - kf_a1_ * y1 - kf_a2_ * y2;
    x2 = x1; x1 = d; y2 = y1; y1 = y;

    // --- envelope follower (peak), attack/release one-pole ---
    float a = (y < 0.0f) ? -y : y;
    env += (a - env) * ((a > env) ? a_env_att_ : a_env_rel_);

    // --- soft-knee gain computer (from effect_dynamics.cpp) ---
    float inputdb = (env > 0.0f) ? scalarToDb(env) : WD_MIN_DB;
    float attdb = WD_MAX_DB;
    if (inputdb >= low_knee_) {
      if (inputdb <= high_knee_) {
        float knee = inputdb - low_knee_;
        attdb = knee_ratio_ * knee * knee * two_knee_width_;
      } else {
        attdb = threshold_db_ + ((inputdb - threshold_db_) * comp_ratio_) - inputdb;
      }
    }
    if (attdb <= compdb)
      compdb = (a_comp_att_ * compdb) + ((1.0f - a_comp_att_) * attdb);
    else
      compdb = (a_comp_rel_ * compdb) + ((1.0f - a_comp_rel_) * attdb);

    // --- width gain: amount scales the computed gain reduction ---
    // width_gain = 10^((amount * gain_db) / 20), lightly smoothed
    float target = dbToScalar(amount_ * compdb);
    wgain += (target - wgain) * 0.02f; // ~2 ms smoothing at 48 kHz

    // --- apply to Side ---
    float s = side->data[i] * (1.0f / 32768.0f);
    side->data[i] = Clip16(static_cast<int32_t>(s * wgain * 32768.0f));
  }

  env_ = env;
  compdb_ = compdb;
  width_gain_ = wgain;
  kf_x1_ = x1; kf_x2_ = x2; kf_y1_ = y1; kf_y2_ = y2;

  transmit(side);
  release(side);
}

#endif // ARDUINO_TEENSY41
