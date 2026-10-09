// Copyright (c) 2026, Phazerville contributors
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

/* AnalysisCV - audio analysis to CV.
 *
 * Audio enters on the CV inputs at the 16.6 kHz control tick (the EnvFollow
 * precedent: In(ch) is sampled once per Controller() call, so analysis sees
 * content up to ~8.3 kHz Nyquist). Each input keeps its own 128-sample
 * rolling window (7.7 ms, 130 Hz bins) and runs a decimated analysis frame
 * every 32/64/128/256 ticks (520/260/130/65 Hz update), with the two
 * channels' frames staggered so they never land on the same tick:
 *   - Spectral centroid ("brightness"), radix-2 FFT, Hann window
 *   - Spectral flux (onset energy: rectified magnitude deltas)
 *   - RMS level
 *   - Zero-crossing rate
 * Two CV outputs each select one of the four features, with range (0-5V or
 * +-5V), gain 0-200%, offset, slew (0-200 ms) and sample-and-hold mode
 * (Clock 1 / Clock 2 trig).
 *
 * NOTE: a 256-sample window is too expensive for the 60 us Controller
 * budget: the radix-2 butterfly inner loop is ~25 insns, so 1024 butterflies
 * (256-pt) run ~25k cycles ~ 45+ us before the time/magnitude passes, while
 * the 128-point FFT (~448 butterflies) lands around 20-30 us total. Both
 * channels use the 128-point FFT. Live per-frame cost is displayed on the
 * Hop parameter row in microseconds (ARM_DWT_CYCCNT, worst case kept).
 */

#pragma once

#define ACV_WINDOW 128 // samples per analysis frame (power of two)
#define ACV_BINS 64    // magnitude bins per frame (window / 2)
#define ACV_TW 128     // twiddle table: exp(-i*2*pi*k/256), k = 0..127

class AnalysisCV : public HemisphereApplet {
public:
    const char* applet_name() { return "AnlyzCV"; }
    const uint8_t* applet_icon() { return PhzIcons::scope; }

    enum AnalysisCVCursor {
        CH, FEAT, RANGE, SCALE, OFFSET, SLEW, SH, DECIM,
        MAX_CURSOR = DECIM
    };

    enum AcvFeature {
        ACV_CENTROID = 0,
        ACV_FLUX,
        ACV_RMS,
        ACV_ZCR
    };

    void Start();
    void Controller();
    void View();
    void OnEncoderMove(int direction);
    uint64_t OnDataRequest();
    void OnDataReceive(uint64_t data);

protected:
    void SetHelp();

private:
    int cursor;

    // Settings (packed into 49 bits)
    uint8_t ch_sel;       // channel whose parameters are shown/edited
    uint8_t feat_sel[2];  // AcvFeature routed to each output
    uint8_t scale[2];     // 0..63, 32 = unity gain
    int8_t offset[2];     // -63..63, full-range offset at +-63
    uint8_t slew[2];      // 0..31 -> 0..200 ms
    uint8_t decim[2];     // 0..3 -> analysis hop 32/64/128/256 ticks
    bool sh[2];           // sample-and-hold per output
    bool bipolar[2];      // false = 0-5V, true = +-5V

    // Analysis results, normalized 0..HEMISPHERE_MAX_CV
    int16_t feat_val[2][4];
    int16_t sh_hold[2];    // held output target (post scale/offset)
    int16_t out_signal[2]; // slewed output

    // Signal capture + analysis scratch (heap-allocated with the applet)
    int16_t buf[2][ACV_WINDOW];
    uint16_t buf_head[2];
    uint16_t countdown[2];
    uint32_t cycles_max[2]; // worst-case analysis cost, ARM_DWT_CYCCNT
    float re[ACV_WINDOW];
    float im[ACV_WINDOW];
    float prev_mag[2][ACV_BINS]; // previous frame magnitudes, for flux
    float win[ACV_WINDOW];       // Hann window
    float tw_re[ACV_TW];         // exp(-i*2*pi*k/256), k = 0..127
    float tw_im[ACV_TW];

    void DrawInterface();
    void BuildTables();
    void AnalyzeChannel(int ch);
    void FftRadix2(float* re, float* im, int n);
    int FeatureOut(int ch);
    int SlewStep(int ch);
    uint16_t HopTicks(int ch);
    const char* FeatLabel(int f);
};

FLASHMEM void AnalysisCV::Start() {
    BuildTables();
    ARM_DEMCR |= ARM_DEMCR_TRCENA;
    ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

    ForEachChannel(ch) {
        feat_sel[ch] = (uint8_t)ch; // defaults: Centroid / Flux
        scale[ch] = 32;
        offset[ch] = 0;
        slew[ch] = 6; // ~40 ms
        decim[ch] = 0; // every 32 ticks
        sh[ch] = false;
        bipolar[ch] = false;
        sh_hold[ch] = 0;
        out_signal[ch] = 0;
        buf_head[ch] = 0;
        // Stagger channel 1's analysis phase so the two FFT frames never land
        // on the same ISR tick (keeps worst-case Controller() cost halved).
        countdown[ch] = ch ? (HopTicks(ch) >> 1) + 1 : HopTicks(ch);
        cycles_max[ch] = 0;
        for (int i = 0; i < ACV_WINDOW; ++i) buf[ch][i] = 0;
        for (int i = 0; i < ACV_BINS; ++i) prev_mag[ch][i] = 0.0f;
        for (int f = 0; f < 4; ++f) feat_val[ch][f] = 0;
    }
    ch_sel = 0;
}

FLASHMEM void AnalysisCV::Controller() {
    ForEachChannel(ch) {
        // 1. Capture one audio sample per tick (EnvFollow input path)
        buf[ch][buf_head[ch]] = (int16_t)In(ch);
        buf_head[ch] = (buf_head[ch] + 1) & (ACV_WINDOW - 1);

        // 2. Decimated analysis frame
        if (--countdown[ch] == 0) {
            AnalyzeChannel(ch);
            countdown[ch] = HopTicks(ch);
        }

        // 3. Output: feature -> range/scale/offset -> S&H -> slew -> DAC
        if (!sh[ch] || Clock(ch)) sh_hold[ch] = (int16_t)FeatureOut(ch);

        int step = SlewStep(ch);
        if (out_signal[ch] < sh_hold[ch]) {
            out_signal[ch] += step;
            if (out_signal[ch] > sh_hold[ch]) out_signal[ch] = sh_hold[ch];
        } else if (out_signal[ch] > sh_hold[ch]) {
            out_signal[ch] -= step;
            if (out_signal[ch] < sh_hold[ch]) out_signal[ch] = sh_hold[ch];
        }
        Out(ch, out_signal[ch]);
    }
}

FLASHMEM void AnalysisCV::View() {
    DrawInterface();
}

FLASHMEM void AnalysisCV::OnEncoderMove(int direction) {
    if (!EditMode()) {
        MoveCursor(cursor, direction, MAX_CURSOR);
        return;
    }

    switch (cursor) {
    case CH:
        ch_sel = constrain(ch_sel + direction, 0, 1);
        break;
    case FEAT:
        feat_sel[ch_sel] = constrain(feat_sel[ch_sel] + direction, 0, 3);
        break;
    case RANGE:
        bipolar[ch_sel] = !bipolar[ch_sel];
        break;
    case SCALE:
        scale[ch_sel] = constrain(scale[ch_sel] + direction, 0, 63);
        break;
    case OFFSET:
        offset[ch_sel] = constrain(offset[ch_sel] + direction, -63, 63);
        break;
    case SLEW:
        slew[ch_sel] = constrain(slew[ch_sel] + direction, 0, 31);
        break;
    case SH:
        sh[ch_sel] = !sh[ch_sel];
        break;
    case DECIM:
        decim[ch_sel] = constrain(decim[ch_sel] + direction, 0, 3);
        break;
    }
    ResetCursor();
}

FLASHMEM uint64_t AnalysisCV::OnDataRequest() {
    uint64_t data = 0;
    Pack(data, PackLocation {0, 1}, ch_sel);
    Pack(data, PackLocation {1, 2}, feat_sel[0]);
    Pack(data, PackLocation {3, 2}, feat_sel[1]);
    Pack(data, PackLocation {5, 6}, scale[0]);
    Pack(data, PackLocation {11, 6}, scale[1]);
    Pack(data, PackLocation {17, 7}, (uint8_t)(offset[0] + 64));
    Pack(data, PackLocation {24, 7}, (uint8_t)(offset[1] + 64));
    Pack(data, PackLocation {31, 5}, slew[0]);
    Pack(data, PackLocation {36, 5}, slew[1]);
    Pack(data, PackLocation {41, 1}, sh[0]);
    Pack(data, PackLocation {42, 1}, sh[1]);
    Pack(data, PackLocation {43, 2}, decim[0]);
    Pack(data, PackLocation {45, 2}, decim[1]);
    Pack(data, PackLocation {47, 1}, bipolar[0]);
    Pack(data, PackLocation {48, 1}, bipolar[1]);
    return data;
}

FLASHMEM void AnalysisCV::OnDataReceive(uint64_t data) {
    ch_sel = Unpack(data, PackLocation {0, 1});
    feat_sel[0] = Unpack(data, PackLocation {1, 2});
    feat_sel[1] = Unpack(data, PackLocation {3, 2});
    scale[0] = Unpack(data, PackLocation {5, 6});
    scale[1] = Unpack(data, PackLocation {11, 6});
    offset[0] = (int8_t)Unpack(data, PackLocation {17, 7}) - 64;
    offset[1] = (int8_t)Unpack(data, PackLocation {24, 7}) - 64;
    slew[0] = Unpack(data, PackLocation {31, 5});
    slew[1] = Unpack(data, PackLocation {36, 5});
    sh[0] = Unpack(data, PackLocation {41, 1});
    sh[1] = Unpack(data, PackLocation {42, 1});
    decim[0] = Unpack(data, PackLocation {43, 2});
    decim[1] = Unpack(data, PackLocation {45, 2});
    bipolar[0] = Unpack(data, PackLocation {47, 1});
    bipolar[1] = Unpack(data, PackLocation {48, 1});
}

FLASHMEM void AnalysisCV::SetHelp() {
    //                    "-------" <-- Label size guide
    help[HELP_DIGITAL1] = PSTR("SH Trg1");
    help[HELP_DIGITAL2] = PSTR("SH Trg2");
    help[HELP_CV1]      = PSTR("Audio 1");
    help[HELP_CV2]      = PSTR("Audio 2");
    help[HELP_OUT1]     = PSTR("Feat 1");
    help[HELP_OUT2]     = PSTR("Feat 2");
    help[HELP_EXTRA1] = PSTR("Set: Feature/Range");
    help[HELP_EXTRA2] = PSTR("     Gain/Ofs/Slw");
    //                  "---------------------" <-- Extra text size guide
}

// ---- Analysis engine ----

FLASHMEM void AnalysisCV::BuildTables() {
    const float kTwoPi = 6.283185307f;
    for (int k = 0; k < ACV_TW; ++k) {
        float ang = kTwoPi * (float)k / 256.0f;
        tw_re[k] = cosf(ang);
        tw_im[k] = -sinf(ang); // exp(-i*ang)
    }
    for (int i = 0; i < ACV_WINDOW; ++i)
        win[i] = 0.5f - 0.5f * cosf(kTwoPi * (float)i / (float)ACV_WINDOW);
}

// In-place iterative radix-2 DIT FFT with bit reversal.
// Twiddles come from the 256-point table via stride = 256 / len.
FLASHMEM void AnalysisCV::FftRadix2(float* re, float* im, int n) {
    int j = 0;
    for (int i = 1; i < n; ++i) {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) {
            float t = re[i]; re[i] = re[j]; re[j] = t;
            t = im[i]; im[i] = im[j]; im[j] = t;
        }
    }
    for (int len = 2; len <= n; len <<= 1) {
        const int half = len >> 1;
        const int step = 256 / len;
        for (int i = 0; i < n; i += len) {
            for (int k = 0; k < half; ++k) {
                const float wr = tw_re[k * step];
                const float wi = tw_im[k * step];
                const int a = i + k;
                const int b = a + half;
                const float tr = re[b] * wr - im[b] * wi;
                const float ti = re[b] * wi + im[b] * wr;
                re[b] = re[a] - tr;
                im[b] = im[a] - ti;
                re[a] += tr;
                im[a] += ti;
            }
        }
    }
}

// One analysis frame from the rolling window of a channel.
// Cost measured with ARM_DWT_CYCCNT (shown on the Hop parameter row).
FLASHMEM void AnalysisCV::AnalyzeChannel(int ch) {
    const uint32_t t0 = ARM_DWT_CYCCNT;

    const int n = ACV_WINDOW;
    const int half = n >> 1;
    const int head = buf_head[ch];

    // Time domain: RMS + zero-crossing rate + windowed copy
    float sumsq = 0.0f;
    int zc = 0;
    int prev = buf[ch][head];
    for (int i = 0; i < n; ++i) {
        const int s = buf[ch][(head + i) & (ACV_WINDOW - 1)];
        sumsq += (float)s * (float)s;
        if ((s < 0) != (prev < 0)) ++zc;
        prev = s;
        re[i] = (float)s * win[i];
        im[i] = 0.0f;
    }

    float rms = sqrtf(sumsq / (float)n);
    int v = (int)(rms * (float)HEMISPHERE_MAX_CV / (float)HEMISPHERE_MAX_INPUT_CV);
    feat_val[ch][ACV_RMS] = (int16_t)constrain(v, 0, HEMISPHERE_MAX_CV);

    v = zc * HEMISPHERE_MAX_CV / (n - 1);
    feat_val[ch][ACV_ZCR] = (int16_t)constrain(v, 0, HEMISPHERE_MAX_CV);

    // Frequency domain: centroid + flux
    FftRadix2(re, im, n);

    float magsum = 0.0f, wsum = 0.0f, flux = 0.0f;
    for (int i = 0; i < half; ++i) {
        float m = sqrtf(re[i] * re[i] + im[i] * im[i]);
        float d = m - prev_mag[ch][i];
        if (d > 0.0f) flux += d;
        prev_mag[ch][i] = m;
        magsum += m;
        wsum += (float)i * m;
    }

    if (magsum > 1e-6f) {
        v = (int)(wsum / magsum * (float)HEMISPHERE_MAX_CV / (float)(half - 1));
    } else v = 0;
    feat_val[ch][ACV_CENTROID] = (int16_t)constrain(v, 0, HEMISPHERE_MAX_CV);

    if (magsum > 1e-6f) {
        float r = flux / magsum;
        if (r > 1.0f) r = 1.0f;
        v = (int)(r * (float)HEMISPHERE_MAX_CV);
    } else v = 0;
    feat_val[ch][ACV_FLUX] = (int16_t)constrain(v, 0, HEMISPHERE_MAX_CV);

    const uint32_t dt = ARM_DWT_CYCCNT - t0;
    if (dt > cycles_max[ch]) cycles_max[ch] = dt;
}

// Feature -> output CV, through range, gain and offset
FLASHMEM int AnalysisCV::FeatureOut(int ch) {
    int v = feat_val[ch][feat_sel[ch]];
    if (bipolar[ch]) v = 2 * v - HEMISPHERE_MAX_CV;
    v = v * (int)scale[ch] / 32;
    v += (int)offset[ch] * HEMISPHERE_MAX_CV / 63;
    return constrain(v, bipolar[ch] ? -HEMISPHERE_MAX_CV : 0, HEMISPHERE_MAX_CV);
}

// Slew: 0..31 -> 0..200 ms per full range
FLASHMEM int AnalysisCV::SlewStep(int ch) {
    const uint32_t ticks = 1 + (uint32_t)slew[ch] * 108;
    int step = (int)((2 * (int32_t)HEMISPHERE_MAX_CV) / ticks);
    return step < 1 ? 1 : step;
}

// Analysis hop in ticks: 32/64/128/256 -> 520/260/130/65 Hz update
FLASHMEM uint16_t AnalysisCV::HopTicks(int ch) {
    return (uint16_t)(32 << decim[ch]);
}

FLASHMEM const char* AnalysisCV::FeatLabel(int f) {
    switch (f) {
    case ACV_CENTROID: return PSTR("Cn");
    case ACV_FLUX:     return PSTR("Fl");
    case ACV_RMS:      return PSTR("Rm");
    default:           return PSTR("Zc");
    }
}

// ---- Display ----

FLASHMEM void AnalysisCV::DrawInterface() {
    // Output routing summary
    gfxPrint(1, 2, PSTR("1:"));
    gfxPrint(13, 2, FeatLabel(feat_sel[0]));
    gfxPrint(31, 2, PSTR("2:"));
    gfxPrint(43, 2, FeatLabel(feat_sel[1]));

    // Feature bars + numeric readout for the selected channel
    for (int f = 0; f < 4; ++f) {
        int y = 11 + f * 10;
        int pct = (int)feat_val[ch_sel][f] * 100 / HEMISPHERE_MAX_CV;
        gfxPrint(1, y, FeatLabel(f));
        gfxFrame(14, y + 1, ProportionCV(feat_val[ch_sel][f], 27) + 1, 5);
        gfxPrint(43, y, pct);
        if (feat_sel[ch_sel] == f) gfxInvert(0, y - 1, 13, 9);
    }

    // Parameter strip
    gfxLine(0, 49, 63, 49);
    switch (cursor) {
    case CH:
        gfxPrint(1, 52, PSTR("In"));
        gfxPrint(31, 52, (int)ch_sel + 1);
        break;
    case FEAT:
        gfxPrint(1, 52, PSTR("Out"));
        gfxPrint(19, 52, (int)ch_sel + 1);
        gfxPrint(31, 52, FeatLabel(feat_sel[ch_sel]));
        break;
    case RANGE:
        gfxPrint(1, 52, PSTR("Rng"));
        gfxPrint(31, 52, bipolar[ch_sel] ? PSTR("+-5V") : PSTR("0-5V"));
        break;
    case SCALE:
        gfxPrint(1, 52, PSTR("Gain"));
        gfxPrint(31, 52, (int)scale[ch_sel] * 100 / 32);
        gfxPrint(PSTR("%"));
        break;
    case OFFSET:
        gfxPrint(1, 52, PSTR("Ofs"));
        gfxPrint(31, 52, (int)offset[ch_sel] * 100 / 63);
        gfxPrint(PSTR("%"));
        break;
    case SLEW:
        gfxPrint(1, 52, PSTR("Slw"));
        gfxPrint(31, 52, (int)slew[ch_sel] * 200 / 31);
        gfxPrint(PSTR("ms"));
        break;
    case SH:
        gfxPrint(1, 52, PSTR("S&H"));
        gfxPrint(31, 52, sh[ch_sel] ? PSTR("Trig") : PSTR("Cont"));
        break;
    case DECIM:
        gfxPrint(1, 52, PSTR("Hop"));
        gfxPrint(22, 52, (int)HopTicks(ch_sel));
        gfxPrint(44, 52, (int)(cycles_max[ch_sel] / (F_CPU / 1000000)));
        gfxPrint(PSTR("u"));
        break;
    }
    if (EditMode()) gfxInvert(0, 51, 63, 10);
}
