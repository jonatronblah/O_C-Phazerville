#pragma once

#include "../src/Audio/effect_width.h"
#include "../Audio/AudioMixer.h"
#include "../Audio/AudioPassthrough.h"
#include "../dsputils.h"
#include <Audio.h>

// MidSider - creative mid-side dynamics processor + stereo filter.
//
// Signal chain (per-sample float math inside the custom object; graph objects
// are stock Teensy AudioStream nodes):
//
//            ┌─ (0.5/0.5) ─ M ─ drive(tanh) ─ M filter ─┬─ decode L/R (M leg)
//   L ──┬────┤                                          └─ detector feed ─┐
//   R ──┴────┤                                                          ▼
//            └─ (0.5/-0.5) ─ S ─ drive(tanh) ─ S filter ── [WidthDynamics] ─ decode L/R (±w·S leg)
//
//   decode: L = M + w·S, R = M − w·S, with w = Stereo Output Width [0..2].
//
// AudioEffectWidthDynamics keys the Side gain off a band-filtered Mid feed
// (key filter is inside the object, on the detector feed only): loud/narrow
// content in the key band ducks the sides, quiet content lets them bloom.
// Width Envelope Amount blends between S untouched and S fully ducked.
//
// Drive: AudioEffectWaveshaper per domain with a 129-point tanh transfer
// table. The drive amount is baked into the table as input gain
// (f(x) = tanh(g·x)); because AudioEffectWaveshaper::shape() copies the table
// with delete/new, tables are (re)baked from mainloop() (never from the audio
// ISR), rate-limited, and only when the drive value actually changed.
//
// Filters: one AudioFilterBiquad per domain (M and S independent) plus the
// key-filter biquad inside the width object. AudioFilterBiquad has no
// audio-rate modulation input, so coefficients are refreshed per control tick
// (16.6 kHz) from Controller(), with the frequency target one-pole smoothed --
// the same approach as FilterFolderApplet/ThreeBandz. All other CV'd params
// are one-pole smoothed in Controller() before being pushed to the graph,
// avoiding zipper noise; the width object smooths its gain per sample.
//
// MEMORY LAYOUT NOTE (important!): the multiboot slot0.ld linker script runs
// default .text in ITCM/RAM1 (32KB-bank granular) and only *(.flashmem*) runs
// from flash. GCC silently ignores __attribute__((section(...))) on vague-
// linkage entities (inline/template member functions) -- which is why the
// FLASHMEM markers on other template applets do nothing. To make FLASHMEM
// effective, all state + heavy methods live in the non-template MidSiderCore
// below (out-of-class definitions = strong symbols). Only the audio ISR
// (effect_width update) stays RAM-resident; everything else is FLASHMEM.
// MidSider<Channels> is the registered template wrapper.

class MidSiderCore : public HemisphereAudioApplet {
public:
  void Start() final;
  void Unload() final;
  void mainloop() final;
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
    // Page 0: Mid
    M_FREQ = 0, M_FREQ_CV,
    M_TYPE, M_Q,
    M_DRIVE, M_DRIVE_CV,
    // Page 1: Side
    S_FREQ, S_FREQ_CV,
    S_TYPE, S_Q,
    S_DRIVE, S_DRIVE_CV,
    // Page 2: Width dynamics
    DYN_THRESH, DYN_THRESH_CV,
    DYN_RATIO, DYN_RATIO_CV,
    DYN_ATTACK, DYN_RELEASE,
    // Page 3: Width output / key filter
    WIDTH_ENV, WIDTH_ENV_CV,
    OUT_WIDTH, OUT_WIDTH_CV,
    KF_FREQ, KF_FREQ_CV,
    KF_MODE, KF_Q,
    CURSOR_LENGTH,
  };

  static constexpr float FREQ_CV_SCALE = 400.0f; // +/-2 octaves
  static constexpr uint32_t BAKE_INTERVAL_MS = 20; // table rebuild limit
  static const char* const FILTER_TYPES[4];

  int8_t cursor = M_FREQ;

  // Mid
  uint16_t m_freq = 2047; // 0..2047 units, log 20 Hz..20 kHz
  uint8_t m_type = 0; // LP/HP/BP/Notch
  uint8_t m_q = 10; // 0..127 -> Q 0.5..8
  uint8_t m_drive = 0; // 0..24 dB
  // Side
  uint16_t s_freq = 2047;
  uint8_t s_type = 0;
  uint8_t s_q = 10;
  uint8_t s_drive = 0;
  // Width dynamics
  int8_t dyn_thresh = -20; // dB
  uint8_t dyn_ratio = 4; // x:1
  uint8_t dyn_attack = 20; // 0..63, log 3 ms..4 s
  uint8_t dyn_release = 33;
  uint8_t width_env = 50; // Width Env Amount, 0..100%
  // Key filter + output
  uint16_t key_freq = 1024;
  uint8_t key_q = 10;
  uint8_t key_mode = 2; // BP
  uint8_t out_width = 100; // Stereo Output Width, 0..200% (100% = w 1.0)

  CVInputMap m_freq_cv;
  CVInputMap s_freq_cv;
  CVInputMap mdrive_cv;
  CVInputMap sdrive_cv;
  CVInputMap thr_cv;
  CVInputMap ratio_cv;
  CVInputMap width_cv;
  CVInputMap keyf_cv;
  CVInputMap outw_cv;

  bool alloc_ok = false;

  // smoothed effective values (see Controller)
  float sm_m_freq = 0, sm_s_freq = 0, sm_key_freq = 0;
  float sm_m_drive = 0, sm_s_drive = 0;
  float sm_thresh = -20, sm_ratio = 4;
  float sm_width_env = 50, sm_out_width = 100;

  // last values applied to the biquads (skip redundant coefficient updates)
  int ap_m_type = -1, ap_s_type = -1, ap_k_type = -1;
  float ap_m_freq = -1, ap_s_freq = -1;
  int ap_m_q = -1, ap_s_q = -1;

  uint32_t last_bake_ms = 0;
  float baked_m_drive = -1000, baked_s_drive = -1000;

  // 0..2047 units -> 20 Hz..20 kHz, log (10 octaves)
  static float FreqFromUnits(float units);
  // 0..127 units -> Q 0.5..8
  static float QFromUnits(int units);
  // 0..63 units -> 3 ms..4 s, log
  static float TimeFromUnits(int units);
  static int MillisFromUnits(int units);

  void ApplyFilter(
    AudioFilterBiquad& f,
    int type,
    float freq_units,
    int q_units,
    int& ap_type,
    float& ap_freq,
    int& ap_q
  );

  // 129-point tanh transfer table with drive baked in as input gain:
  // f(x) = tanh(g*x), g = 10^(drive_db/20); power-of-two + 1 length as
  // required by AudioEffectWaveshaper::shape().
  static void BakeShaper(AudioEffectWaveshaper& shaper, float* table, float drive_db);

  AudioPassthrough<2> input;
  AudioMixer<2> encode_m; // 0.5*(L+R)
  AudioMixer<2> encode_s; // 0.5*(L-R)
  AudioEffectWaveshaper shaper_m;
  AudioEffectWaveshaper shaper_s;
  AudioFilterBiquad filt_m;
  AudioFilterBiquad filt_s;
  AudioEffectWidthDynamics width;
  AudioMixer<2> decode_l; // M + w*S
  AudioMixer<2> decode_r; // M - w*S
  AudioPassthrough<2> output;

  float table_m[129];
  float table_s[129];
};

// Thin template wrapper so the registry can use the MidSider<STEREO> form.
// (All real work lives in the non-template core so FLASHMEM section placement
// survives this toolchain; Channels is intentionally unused.)
template <AudioChannels Channels>
class MidSider : public MidSiderCore {
public:
  const char* applet_name() {
    return "MidSider";
  }
};

// -- Out-of-line definitions (strong symbols, FLASHMEM honored) --------------

inline const char* const MidSiderCore::FILTER_TYPES[4] = {
  "LP ", "HP ", "BP ", "Nch"
};

FLASHMEM float MidSiderCore::FreqFromUnits(float units) {
  return 20.0f * fastpow2(units * (9.965784f / 2047.0f));
}

FLASHMEM float MidSiderCore::QFromUnits(int units) {
  return 0.5f + units * (7.5f / 127.0f);
}

FLASHMEM float MidSiderCore::TimeFromUnits(int units) {
  return 0.003f * fastpow2(units * (10.3803f / 63.0f));
}

FLASHMEM int MidSiderCore::MillisFromUnits(int units) {
  return static_cast<int>(TimeFromUnits(units) * 1000.0f + 0.5f);
}

FLASHMEM void MidSiderCore::Start() {
  // Encode: M = 0.5*(L+R), S = 0.5*(L-R) (negative gain on the S path)
  encode_m.gain(0, 0.5f);
  encode_m.gain(1, 0.5f);
  encode_s.gain(0, 0.5f);
  encode_s.gain(1, -0.5f);

  PatchCable(input, 0, encode_m, 0);
  PatchCable(input, 1, encode_m, 1);
  PatchCable(input, 0, encode_s, 0);
  PatchCable(input, 1, encode_s, 1);

  PatchCable(encode_m, 0, shaper_m, 0);
  PatchCable(shaper_m, 0, filt_m, 0);
  PatchCable(filt_m, 0, width, 0); // detector feed (filtered M)
  PatchCable(filt_m, 0, decode_l, 0); // M to decode
  PatchCable(filt_m, 0, decode_r, 0);

  PatchCable(encode_s, 0, shaper_s, 0);
  PatchCable(shaper_s, 0, filt_s, 0);
  PatchCable(filt_s, 0, width, 1); // S audio
  PatchCable(width, 0, decode_l, 1);
  PatchCable(width, 0, decode_r, 1);

  // Decode: L = M + w*S, R = M - w*S (w set from Stereo Output Width)
  decode_l.gain(0, 1.0f);
  decode_r.gain(0, 1.0f);

  PatchCable(decode_l, 0, output, 0);
  PatchCable(decode_r, 0, output, 1);

  width.reset();
  alloc_ok = true;

  // Bake the drive tables once here (safe context), then SetParams()
  sm_m_drive = m_drive;
  sm_s_drive = s_drive;
  BakeShaper(shaper_m, table_m, sm_m_drive);
  baked_m_drive = sm_m_drive;
  BakeShaper(shaper_s, table_s, sm_s_drive);
  baked_s_drive = sm_s_drive;
  SetParams();
}

FLASHMEM void MidSiderCore::Unload() {
  // No factory-owned objects to release; cables are disconnected by the host
  alloc_ok = false;
  AllowRestart();
}

FLASHMEM void MidSiderCore::mainloop() {
  if (!alloc_ok) return;
  // Waveshaper tables are rebuilt here, NOT in Controller()/update() --
  // shape() does delete/new and update() reads the table concurrently, so
  // the copy is guarded with AudioNoInterrupts() inside BakeShaper().
  const uint32_t now = millis();
  if (now - last_bake_ms < BAKE_INTERVAL_MS) return;
  if (fabsf(sm_m_drive - baked_m_drive) > 0.1f) {
    BakeShaper(shaper_m, table_m, sm_m_drive);
    baked_m_drive = sm_m_drive;
    last_bake_ms = now;
  }
  if (fabsf(sm_s_drive - baked_s_drive) > 0.1f) {
    BakeShaper(shaper_s, table_s, sm_s_drive);
    baked_s_drive = sm_s_drive;
    last_bake_ms = now;
  }
}

FLASHMEM void MidSiderCore::SetParams() {
  if (!alloc_ok) return;
  // Snap smoothed values to the base params; Controller() adds CV on top
  sm_m_freq = m_freq;
  sm_s_freq = s_freq;
  sm_key_freq = key_freq;
  sm_m_drive = m_drive;
  sm_s_drive = s_drive;
  sm_thresh = dyn_thresh;
  sm_ratio = dyn_ratio;
  sm_width_env = width_env;
  sm_out_width = out_width;
  // Force filter re-apply on the next Controller() tick
  ap_m_type = -1;
  ap_s_type = -1;
  ap_k_type = -1;
}

// 16.6 kHz control tick. FLASHMEM is fine here (same as LowerRenz::Controller);
// only the audio ISR's update() loop must stay in RAM.
FLASHMEM void MidSiderCore::Controller() {
  if (!alloc_ok) return;

  // Effective values = base param + CV (bipolar, via attenuversion).
  // Locals first: the constrain() macro evaluates its argument repeatedly.
  const float eff_m_freq
    = constrain(m_freq + m_freq_cv.InF() * FREQ_CV_SCALE, 0.0f, 2047.0f);
  const float eff_s_freq
    = constrain(s_freq + s_freq_cv.InF() * FREQ_CV_SCALE, 0.0f, 2047.0f);
  const float eff_key_freq
    = constrain(key_freq + keyf_cv.InF() * FREQ_CV_SCALE, 0.0f, 2047.0f);
  const float eff_m_drive
    = constrain(m_drive + mdrive_cv.InF() * 12.0f, 0.0f, 24.0f);
  const float eff_s_drive
    = constrain(s_drive + sdrive_cv.InF() * 12.0f, 0.0f, 24.0f);
  const float eff_thresh
    = constrain(float(dyn_thresh) + thr_cv.InF() * 24.0f, -90.0f, 0.0f);
  const float eff_ratio
    = constrain(dyn_ratio + ratio_cv.InF() * 12.0f, 1.0f, 60.0f);
  const float eff_width_env
    = constrain(width_env + width_cv.InF() * 50.0f, 0.0f, 100.0f);
  const float eff_out_width
    = constrain(out_width + outw_cv.InF() * 80.0f, 0.0f, 200.0f);

  ONE_POLE(sm_m_freq, eff_m_freq, 0.2f);
  ONE_POLE(sm_s_freq, eff_s_freq, 0.2f);
  ONE_POLE(sm_key_freq, eff_key_freq, 0.2f);
  ONE_POLE(sm_m_drive, eff_m_drive, 0.05f);
  ONE_POLE(sm_s_drive, eff_s_drive, 0.05f);
  ONE_POLE(sm_thresh, eff_thresh, 0.1f);
  ONE_POLE(sm_ratio, eff_ratio, 0.1f);
  ONE_POLE(sm_width_env, eff_width_env, 0.1f);
  ONE_POLE(sm_out_width, eff_out_width, 0.1f);

  ApplyFilter(filt_m, m_type, sm_m_freq, m_q, ap_m_type, ap_m_freq, ap_m_q);
  ApplyFilter(filt_s, s_type, sm_s_freq, s_q, ap_s_type, ap_s_freq, ap_s_q);

  width.dynamics(
    sm_thresh, sm_ratio, TimeFromUnits(dyn_attack), TimeFromUnits(dyn_release)
  );
  width.amount(sm_width_env * 0.01f);
  width.keyFilter(FreqFromUnits(sm_key_freq), QFromUnits(key_q), key_mode);

  // Decode width w is a block-rate gain (AudioMixer<2>); smoothing it here
  // avoids zipper noise when sweeping Stereo Output Width
  const float w = sm_out_width * 0.01f;
  decode_l.gain(1, w);
  decode_r.gain(1, -w);
}

FLASHMEM void MidSiderCore::ApplyFilter(
  AudioFilterBiquad& f,
  int type,
  float freq_units,
  int q_units,
  int& ap_type,
  float& ap_freq,
  int& ap_q
) {
  if (type == ap_type && fabsf(freq_units - ap_freq) < 0.5f && q_units == ap_q)
    return;
  ap_type = type;
  ap_freq = freq_units;
  ap_q = q_units;
  // NB: AudioFilterBiquad::setLowpass()/setHighpass()/... use double sin/cos,
  // which pull ~4 KB of libm reduction code into ITCM. Compute the same RBJ
  // coefficients with trig-free math and feed setCoefficients() directly.
  float coef[5];
  WidthDynamicsBiquadCoeffs(
    type, FreqFromUnits(freq_units), QFromUnits(q_units), coef
  );
  double dcoef[5];
  for (int i = 0; i < 5; i++) dcoef[i] = coef[i];
  f.setCoefficients(0, dcoef);
}

FLASHMEM void MidSiderCore::View() {
  if (!alloc_ok) {
    gfxPrint(2, 15, "Out of RAM!!");
    return;
  }

  const int label_x = 1;
  const int page
    = (cursor >= S_FREQ) + (cursor >= DYN_THRESH) + (cursor >= WIDTH_ENV);
  gfxRect(page * 15, 10, 14, 3); // page tab

  switch (page) {
    case 0: // ---- Mid ----
      gfxPrint(label_x, 15, "MF:");
      gfxStartCursor();
      graphics.printf("%5d", static_cast<int>(FreqFromUnits(m_freq)));
      gfxEndCursor(cursor == M_FREQ);
      gfxStartCursor();
      gfxPrint(m_freq_cv);
      gfxEndCursor(cursor == M_FREQ_CV, false, m_freq_cv.InputName());

      gfxPrint(label_x, 25, "T:");
      gfxStartCursor();
      gfxPrint(FILTER_TYPES[m_type & 3]);
      gfxEndCursor(cursor == M_TYPE);
      gfxStartCursor();
      gfxPrint(QFromUnits(m_q), 1);
      gfxEndCursor(cursor == M_Q);

      gfxPrint(label_x, 35, "MD:");
      gfxStartCursor();
      graphics.printf("%3ddB", m_drive);
      gfxEndCursor(cursor == M_DRIVE);
      gfxStartCursor();
      gfxPrint(mdrive_cv);
      gfxEndCursor(cursor == M_DRIVE_CV, false, mdrive_cv.InputName());
      break;

    case 1: // ---- Side ----
      gfxPrint(label_x, 15, "SF:");
      gfxStartCursor();
      graphics.printf("%5d", static_cast<int>(FreqFromUnits(s_freq)));
      gfxEndCursor(cursor == S_FREQ);
      gfxStartCursor();
      gfxPrint(s_freq_cv);
      gfxEndCursor(cursor == S_FREQ_CV, false, s_freq_cv.InputName());

      gfxPrint(label_x, 25, "T:");
      gfxStartCursor();
      gfxPrint(FILTER_TYPES[s_type & 3]);
      gfxEndCursor(cursor == S_TYPE);
      gfxStartCursor();
      gfxPrint(QFromUnits(s_q), 1);
      gfxEndCursor(cursor == S_Q);

      gfxPrint(label_x, 35, "SD:");
      gfxStartCursor();
      graphics.printf("%3ddB", s_drive);
      gfxEndCursor(cursor == S_DRIVE);
      gfxStartCursor();
      gfxPrint(sdrive_cv);
      gfxEndCursor(cursor == S_DRIVE_CV, false, sdrive_cv.InputName());
      break;

    case 2: // ---- Width dynamics ----
      gfxPrint(label_x, 15, "Thr:");
      gfxStartCursor();
      graphics.printf("%3ddB", dyn_thresh);
      gfxEndCursor(cursor == DYN_THRESH);
      gfxStartCursor();
      gfxPrint(thr_cv);
      gfxEndCursor(cursor == DYN_THRESH_CV, false, thr_cv.InputName());

      gfxPrint(label_x, 25, "Rat:");
      gfxStartCursor();
      graphics.printf("%2d:1", dyn_ratio);
      gfxEndCursor(cursor == DYN_RATIO);
      gfxStartCursor();
      gfxPrint(ratio_cv);
      gfxEndCursor(cursor == DYN_RATIO_CV, false, ratio_cv.InputName());

      gfxPrint(label_x, 35, "Atk:");
      gfxStartCursor();
      graphics.printf("%4dms", MillisFromUnits(dyn_attack));
      gfxEndCursor(cursor == DYN_ATTACK);

      gfxPrint(label_x, 45, "Rel:");
      gfxStartCursor();
      graphics.printf("%4dms", MillisFromUnits(dyn_release));
      gfxEndCursor(cursor == DYN_RELEASE);

      // GR meters: Mid gain reduction + Side width modulation (60dB == 20px)
      gfxPrint(label_x, 55, "GR:");
      gfxInvert(20, 55, Proportion(-int(width.get_gain_db()), 60, 20), 7);
      gfxInvert(44, 55,
                Proportion(-int(width.get_gain_db() * sm_width_env * 0.01f),
                           60, 20),
                7);
      break;

    case 3: // ---- Output width / key filter ----
      gfxPrint(label_x, 15, "Amt:");
      gfxStartCursor();
      graphics.printf("%3d%%", width_env);
      gfxEndCursor(cursor == WIDTH_ENV);
      gfxStartCursor();
      gfxPrint(width_cv);
      gfxEndCursor(cursor == WIDTH_ENV_CV, false, width_cv.InputName());

      gfxPrint(label_x, 25, "Wid:");
      gfxStartCursor();
      graphics.printf("%3d%%", out_width);
      gfxEndCursor(cursor == OUT_WIDTH);
      gfxStartCursor();
      gfxPrint(outw_cv);
      gfxEndCursor(cursor == OUT_WIDTH_CV, false, outw_cv.InputName());

      gfxPrint(label_x, 35, "KF:");
      gfxStartCursor();
      graphics.printf("%5d", static_cast<int>(FreqFromUnits(key_freq)));
      gfxEndCursor(cursor == KF_FREQ);
      gfxStartCursor();
      gfxPrint(keyf_cv);
      gfxEndCursor(cursor == KF_FREQ_CV, false, keyf_cv.InputName());

      gfxPrint(label_x, 45, "T:");
      gfxStartCursor();
      gfxPrint(FILTER_TYPES[key_mode & 3]);
      gfxEndCursor(cursor == KF_MODE);
      gfxStartCursor();
      gfxPrint(QFromUnits(key_q), 1);
      gfxEndCursor(cursor == KF_Q);

      // Small M/S balance indicator: tick = Stereo Output Width (0..2)
      gfxPrint(label_x, 55, "Bal:");
      gfxDottedLine(20, 58, 60, 58);
      gfxInvert(20 + Proportion(int(sm_out_width + 0.5f), 200, 40), 55, 2, 7);
      break;
  }

  gfxDisplayInputMapEditor();
}

FLASHMEM void MidSiderCore::DrawFullScreen() {
  if (!alloc_ok) return;
  const int x0 = 64 - gfx_offset;
  graphics.drawLine(x0, 26, 127 - gfx_offset, 26, 3);

  // Mid gain-reduction bar (from the width object's gain computer)
  int h = Proportion(static_cast<int>(width.get_gain_db()), 60, 30); // 60dB == 30px
  if (h > 0) graphics.drawRect(x0 + 15, 26 - h, 10, h);
  else graphics.drawRect(x0 + 15, 26, 10, -h);

  // Side width-modulation bar (gain actually applied to S)
  h = Proportion(
    static_cast<int>(width.get_gain_db() * sm_width_env * 0.01f), 60, 30
  );
  if (h > 0) graphics.drawRect(x0 + 35, 26 - h, 10, h);
  else graphics.drawRect(x0 + 35, 26, 10, -h);

  // M/S balance indicator: tick = Stereo Output Width (0..2)
  const int bx = x0 + Proportion(static_cast<int>(sm_out_width + 0.5f), 200, 63);
  graphics.drawLine(bx, 22, bx, 30, 1);

  View();
}

FLASHMEM void MidSiderCore::OnButtonPress() {
  if (CheckEditInputMapPress(
        cursor,
        IndexedInput(M_FREQ_CV, m_freq_cv),
        IndexedInput(M_DRIVE_CV, mdrive_cv),
        IndexedInput(S_FREQ_CV, s_freq_cv),
        IndexedInput(S_DRIVE_CV, sdrive_cv),
        IndexedInput(DYN_THRESH_CV, thr_cv),
        IndexedInput(DYN_RATIO_CV, ratio_cv),
        IndexedInput(WIDTH_ENV_CV, width_cv),
        IndexedInput(OUT_WIDTH_CV, outw_cv),
        IndexedInput(KF_FREQ_CV, keyf_cv)
      ))
    return;
  CursorToggle();
}

FLASHMEM void MidSiderCore::OnEncoderMove(int direction) {
  if (!EditMode()) {
    MoveCursor(cursor, direction, CURSOR_LENGTH - 1);
    return;
  }
  if (EditSelectedInputMap(direction)) return;

  switch (cursor) {
    case M_FREQ:
      m_freq = constrain(m_freq + direction * 4, 0, 2047);
      break;
    case M_FREQ_CV: m_freq_cv.ChangeSource(direction); break;
    case M_TYPE: m_type = constrain(m_type + direction, 0, 3); break;
    case M_Q: m_q = constrain(m_q + direction, 0, 127); break;
    case M_DRIVE: m_drive = constrain(m_drive + direction, 0, 24); break;
    case M_DRIVE_CV: mdrive_cv.ChangeSource(direction); break;

    case S_FREQ:
      s_freq = constrain(s_freq + direction * 4, 0, 2047);
      break;
    case S_FREQ_CV: s_freq_cv.ChangeSource(direction); break;
    case S_TYPE: s_type = constrain(s_type + direction, 0, 3); break;
    case S_Q: s_q = constrain(s_q + direction, 0, 127); break;
    case S_DRIVE: s_drive = constrain(s_drive + direction, 0, 24); break;
    case S_DRIVE_CV: sdrive_cv.ChangeSource(direction); break;

    case DYN_THRESH:
      dyn_thresh = constrain(dyn_thresh + direction, -90, 0);
      break;
    case DYN_THRESH_CV: thr_cv.ChangeSource(direction); break;
    case DYN_RATIO: dyn_ratio = constrain(dyn_ratio + direction, 1, 60); break;
    case DYN_RATIO_CV: ratio_cv.ChangeSource(direction); break;
    case DYN_ATTACK:
      dyn_attack = constrain(dyn_attack + direction, 0, 63);
      break;
    case DYN_RELEASE:
      dyn_release = constrain(dyn_release + direction, 0, 63);
      break;

    case WIDTH_ENV:
      width_env = constrain(width_env + direction, 0, 100);
      break;
    case WIDTH_ENV_CV: width_cv.ChangeSource(direction); break;
    case OUT_WIDTH:
      out_width = constrain(out_width + direction, 0, 200);
      break;
    case OUT_WIDTH_CV: outw_cv.ChangeSource(direction); break;
    case KF_FREQ:
      key_freq = constrain(key_freq + direction * 4, 0, 2047);
      break;
    case KF_FREQ_CV: keyf_cv.ChangeSource(direction); break;
    case KF_Q: key_q = constrain(key_q + direction, 0, 127); break;
    case KF_MODE: key_mode = constrain(key_mode + direction, 0, 3); break;

    default: return;
  }
  SetParams();
}

// 256-bit state layout (4 x 64-bit words), everything in-bounds:
//   word 0 (64b): m_type(2) m_freq(11) m_q(7) m_drive(5)
//                 s_type(2) s_freq(11) s_q(7) s_drive(5)
//                 dyn_thresh(8) dyn_ratio(6)
//   word 1 (63b): dyn_attack(6) dyn_release(6) width_env(7) out_width(8)
//                 key_freq(11) key_q(7) key_mode(2) m_freq_cv(16)
//   word 2 (64b): s_freq_cv, width_cv, thr_cv, outw_cv (4 x 16)
//   word 3 (64b): keyf_cv, ratio_cv, mdrive_cv, sdrive_cv (4 x 16)
// 255 bits used, 1 spare. No SetData() spillover needed.
FLASHMEM void MidSiderCore::OnDataRequest(std::array<uint64_t, CONFIG_SIZE>& data) {
  data[0] = PackPackables(
    pack<2>(m_type),
    pack<11>(m_freq),
    pack<7>(m_q),
    pack<5>(m_drive),
    pack<2>(s_type),
    pack<11>(s_freq),
    pack<7>(s_q),
    pack<5>(s_drive),
    dyn_thresh,
    pack<6>(dyn_ratio)
  );
  data[1] = PackPackables(
    pack<6>(dyn_attack),
    pack<6>(dyn_release),
    pack<7>(width_env),
    out_width,
    pack<11>(key_freq),
    pack<7>(key_q),
    pack<2>(key_mode),
    m_freq_cv
  );
  data[2] = PackPackables(s_freq_cv, width_cv, thr_cv, outw_cv);
  data[3] = PackPackables(keyf_cv, ratio_cv, mdrive_cv, sdrive_cv);
}

FLASHMEM void MidSiderCore::OnDataReceive(
  const std::array<uint64_t, CONFIG_SIZE>& data
) {
  UnpackPackables(
    data[0],
    pack<2>(m_type),
    pack<11>(m_freq),
    pack<7>(m_q),
    pack<5>(m_drive),
    pack<2>(s_type),
    pack<11>(s_freq),
    pack<7>(s_q),
    pack<5>(s_drive),
    dyn_thresh,
    pack<6>(dyn_ratio)
  );
  UnpackPackables(
    data[1],
    pack<6>(dyn_attack),
    pack<6>(dyn_release),
    pack<7>(width_env),
    out_width,
    pack<11>(key_freq),
    pack<7>(key_q),
    pack<2>(key_mode),
    m_freq_cv
  );
  UnpackPackables(data[2], s_freq_cv, width_cv, thr_cv, outw_cv);
  UnpackPackables(data[3], keyf_cv, ratio_cv, mdrive_cv, sdrive_cv);

  // re-derive packed ranges defensively
  m_type = constrain(m_type, 0, 3);
  s_type = constrain(s_type, 0, 3);
  key_mode = constrain(key_mode, 0, 3);
  dyn_ratio = constrain(dyn_ratio, 1, 60);
  m_drive = constrain(m_drive, 0, 24);
  s_drive = constrain(s_drive, 0, 24);
  width_env = constrain(width_env, 0, 100);
  out_width = constrain(out_width, 0, 200);
  dyn_thresh = constrain(dyn_thresh, -90, 0);

  SetParams();
}

FLASHMEM void MidSiderCore::SetHelp() {
  help[HELP_DIGITAL1] = " - ";
  help[HELP_DIGITAL2] = " - ";
  help[HELP_CV1]      = "M/S Frq";
  help[HELP_CV2]      = "Width";
  help[HELP_OUT1]     = "Out L";
  help[HELP_OUT2]     = "Out R";
  help[HELP_EXTRA1] = "Per-row CV maps";
  help[HELP_EXTRA2] = "band-keyed width";
}

FLASHMEM void MidSiderCore::BakeShaper(
  AudioEffectWaveshaper& shaper, float* table, float drive_db
) {
  const float g = dbToScalar(drive_db);
  for (int i = 0; i < 129; i++) {
    const float x = g * ((i - 64) * (1.0f / 64.0f));
    // tanh(x) = 1 - 2/(exp(2x)+1)
    table[i] = 1.0f - 2.0f / (fastexp(2.0f * x) + 1.0f);
  }
  AudioNoInterrupts(); // audio update() reads the table concurrently
  shaper.shape(table, 129);
  AudioInterrupts();
}
