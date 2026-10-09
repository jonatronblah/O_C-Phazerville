#pragma once

// Copyright (c) 2026, Phazerville contributors
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

/* CVTime - CV and gate delay with dynamically CV-able gate lengths.
 *
 * Two independent channels, each switchable between:
 *
 *  - CV Delay: a slow-CV delay line at the 16.6 kHz control tick. Delay time
 *    is knob + CV (log-mapped ~60 us..60 s), time changes are slewed and the
 *    read is linearly interpolated (fractional delay), and Feedback regenerates
 *    the tail (delay-line-as-looper-ish). Buffer: 60 s per channel in PSRAM,
 *    ~5 s (shrinking) RAM2 fallback when there is no PSRAM.
 *
 *  - Gate Proc: gate in -> gate out with a delay tap (time knob + CV, or
 *    clock-synced divisions of the measured clock cycle) and a length
 *    multiplier/divider 0.125x..4x of the input gate's length, modulated
 *    LIVE by length CV (In on the channel, plus a CV map on ch1) - the
 *    "dynamically shorten and lengthen the gates via cv" wish. The output
 *    length follows mult * the previous gate's measured length while the
 *    gate is running, and locks to mult * this gate's length once the input
 *    falls (so a held gate can be cut short mid-gate and an ended gate can
 *    be extended). A Minimum length floor keeps triggers from vanishing.
 *
 * How this differs from GateDelay (which this suite already ships):
 *  - GateDelay is a fixed 1-bit tape delay: 0..2000 ms, 1 ms resolution,
 *    one shared time per channel, no length processing at all.
 *  - CVTime adds (1) full CV delay with feedback + slewed/interpolated time,
 *    (2) gate length multiply/divide with live length CV + min floor,
 *    (3) clock-synced delay, (4) 60 s buffers, (5) per-row CV source maps.
 *    Overlap: both can delay a gate by a fixed time; GateDelay is simpler
 *    and cheaper if that is all you need.
 *
 * Implementation note: every method lives in FLASHMEM (out-of-line) because
 * the 32 KB bank boundary - even Controller() runs from flash, where it is
 * I-cache-hot at the 16.6 kHz call rate (a few us, well inside the 60 us
 * tick). No allocation, I/O or Serial in the ISR.
 */

#include "../dsputils.h" // fastpow2 (log delay mapping)
#ifdef __IMXRT1062__
#include <smalloc.h> // extmem_calloc / extmem_free
#endif

extern "C" uint8_t external_psram_size;

class CVTime : public HemisphereApplet {
public:
    static constexpr int CVT_PS_SAMPLES = 60 * 16667;  // ~60 s of control ticks in PSRAM
    static constexpr int CVT_RAM_SAMPLES = 5 * 16667;  // ~5 s fallback in RAM2
    static constexpr int CVT_MIN_SAMPLES = 16667;      // ~1 s floor for shrinking allocs

    const char* applet_name() { return "CVTime"; }
    const uint8_t* applet_icon() { return PhzIcons::clock_warp_B; }

    void Start();
    void Unload();
    void Controller();
    void View();
    void OnButtonPress() override;
    void OnEncoderMove(int direction);
    uint64_t OnDataRequest();
    void OnDataReceive(uint64_t data);

protected:
    void SetHelp();

private:
    enum CVTRow {
        R_MODE1, R_TIME1, R_T1CV, R_LEN1,
        R_MODE2, R_TIME2, R_T2CV, R_LEN2,
        R_GCV1, R_FB1, R_FB2, R_FBCV,
        R_SYNC, R_SLW1, R_SLW2, R_MIN,
        R_LAST = R_MIN
    };

    // --- Saved configuration: 64-bit word (62 bits used) + CV maps in PhzConfig
    uint8_t mode[2];     // 0 = CV delay, 1 = gate processor
    uint8_t sync;        // 0 = Free, 1 = /4, 2 = /2, 3 = x1 clock cycle
    uint16_t time_u[2];  // 0..2047 -> log-mapped 1..1000000 ticks (60us..60s)
    uint8_t len[2];      // 0..31 -> gate length mult 0.125x..4x
    uint8_t fb[2];       // 0..63 -> feedback ~0..95%
    uint8_t slew[2];     // 0..15 -> delay smoothing tau = 1<<(slew+1) ticks
    uint8_t minlen;      // 0..63 -> minimum gate length = minlen<<8 ticks

    CVInputMap dt_cv[2];  // delay time ch1 / ch2
    CVInputMap glen_cv;   // gate length ch1 (live!)
    CVInputMap fb_cv;     // feedback

    // --- Runtime state (not persisted)
    int16_t* buf[2];
    int32_t buf_len[2];
    int32_t w_ix[2];
    bool psram[2];
    bool cfg_loaded;
    int cursor;

    // CV delay state (Q8 smoothed delay in ticks)
    int32_t sm_delay[2];

    // Gate processor state
    int32_t g_delay_left[2];
    int32_t g_out_elapsed[2]; // -1 = output inactive
    int32_t g_in_elapsed[2];
    int32_t g_in_len[2];      // previous gate's measured length (latched on fall)
    bool g_in_last[2];

    // --- helpers (defined out-of-line below, all FLASHMEM) ---
    void ApplyDefaults();
    void Sanitize();
    void FreeBuffers();
    static int32_t TicksFromUnits(int u);
    void DrawInterface();
    void DrawRow(int row, int y);
    void PrintTime(int ch);
};

/* -- Function bodies (all in FLASHMEM to conserve ITCM) -- */

FLASHMEM void CVTime::ApplyDefaults() {
    mode[0] = 1;     // ch1: gate processor
    mode[1] = 0;     // ch2: CV delay
    sync = 0;        // Free
    time_u[0] = 1320; // ~100 ms
    time_u[1] = 1320;
    len[0] = len[1] = 7;   // 1.0x
    fb[0] = fb[1] = 0;
    slew[0] = slew[1] = 2;
    minlen = 2;             // ~30 ms floor
    cfg_loaded = true;
}

FLASHMEM void CVTime::Sanitize() {
    ForEachChannel(ch) {
        mode[ch] = mode[ch] & 1;
        time_u[ch] = constrain(time_u[ch], 0, 2047);
        len[ch] = constrain(len[ch], 0, 31);
        fb[ch] = constrain(fb[ch], 0, 63);
        slew[ch] = constrain(slew[ch], 0, 15);
    }
    sync = constrain(sync, 0, 3);
    minlen = constrain(minlen, 0, 63);
}

FLASHMEM void CVTime::FreeBuffers() {
    ForEachChannel(ch) {
        if (buf[ch]) {
#ifdef __IMXRT1062__
            if (psram[ch]) extmem_free(buf[ch]);
            else free(buf[ch]);
#else
            free(buf[ch]);
#endif
            buf[ch] = nullptr;
        }
    }
}

// 0..2047 units -> 1..1000000 control ticks (60us..60s), log
FLASHMEM int32_t CVTime::TicksFromUnits(int u) {
    return (int32_t)fastpow2((float)u * (19.9316f / 2047.0f));
}

FLASHMEM void CVTime::Start() {
    if (!cfg_loaded) ApplyDefaults();
    ForEachChannel(ch) {
        if (!buf[ch]) {
            // 60 s per channel in PSRAM when available...
#ifdef __IMXRT1062__
            if (external_psram_size > 0) {
                buf[ch] = (int16_t*)extmem_calloc(CVT_PS_SAMPLES, sizeof(int16_t));
                if (buf[ch]) {
                    buf_len[ch] = CVT_PS_SAMPLES;
                    psram[ch] = true;
                    // extmem_calloc does not guarantee zero-fill
                    memset(buf[ch], 0, CVT_PS_SAMPLES * sizeof(int16_t));
                }
            }
#endif
            // ...otherwise ~5 s from the RAM2 heap, shrinking if crowded
            if (!buf[ch]) {
                int32_t n = CVT_RAM_SAMPLES;
                while (n >= CVT_MIN_SAMPLES && !buf[ch]) {
                    buf[ch] = (int16_t*)calloc(n, sizeof(int16_t));
                    if (buf[ch]) buf_len[ch] = n;
                    n /= 2;
                }
                psram[ch] = false;
            }
        }
        w_ix[ch] = 0;
        sm_delay[ch] = TicksFromUnits(time_u[ch]) * 256;
        g_delay_left[ch] = 0;
        g_out_elapsed[ch] = -1;
        g_in_elapsed[ch] = 0;
        g_in_len[ch] = 256;
        g_in_last[ch] = false;
    }
    cursor = 0;
}

FLASHMEM void CVTime::Unload() {
    FreeBuffers();
    AllowRestart();
}

FLASHMEM void CVTime::Controller() {
    ForEachChannel(ch) {
        if (!buf[ch]) { Out(ch, 0); continue; }

        // Record the input CV every tick in both modes, so switching modes
        // mid-flight already has fresh history. One int16 store - cheap.
        int32_t in = In(ch);
        buf[ch][w_ix[ch]] = (int16_t)constrain(in, -32768, 32767);

        // ---- Delay time: knob + CV map, log units, mode-dependent use ----
        float u = (float)time_u[ch] + dt_cv[ch].InF() * 2047.0f;
        CONSTRAIN(u, 0.0f, 2047.0f);
        int32_t dticks = TicksFromUnits((int)u);
        if (dticks > buf_len[ch] - 2) dticks = buf_len[ch] - 2;
        if (dticks < 1) dticks = 1;

        if (mode[ch] == 0) {
            // ---- CV delay: slewed fractional read + feedback ----
            int32_t tau = 1 << (slew[ch] + 1);
            sm_delay[ch] += (dticks * 256 - sm_delay[ch]) / tau;
            int32_t d_q8 = sm_delay[ch];
            if (d_q8 < 256) d_q8 = 256;
            int32_t d = d_q8 >> 8;
            int32_t frac = d_q8 & 0xff;

            int32_t i0 = w_ix[ch] - d;
            while (i0 < 0) i0 += buf_len[ch];
            int32_t i1 = i0 - 1;
            if (i1 < 0) i1 += buf_len[ch];
            int32_t a = buf[ch][i0];
            int32_t y = a + (((buf[ch][i1] - a) * frac) >> 8);

            // feedback: the delayed signal re-enters the line
            int32_t wv = in + (y * fb[ch]) / 66;
            buf[ch][w_ix[ch]] = (int16_t)constrain(wv, -32768, 32767);

            Out(ch, constrain(y, HEMISPHERE_MIN_CV, HEMISPHERE_MAX_CV));
        } else {
            // ---- Gate processor: delay tap + live length multiply ----
            int32_t gd = dticks;
            if (sync != 0) {
                uint32_t cyc = ClockCycleTicks(ch);
                if (cyc > 0) {
                    gd = (sync == 1) ? (int32_t)(cyc / 4)
                       : (sync == 2) ? (int32_t)(cyc / 2)
                       : (int32_t)cyc;
                }
            }

            bool g = Gate(ch);
            bool rise = g && !g_in_last[ch];
            g_in_last[ch] = g;

            if (rise) {
                g_in_elapsed[ch] = 1;
                if (gd <= 0) {
                    g_out_elapsed[ch] = 0;
                    g_delay_left[ch] = 0;
                } else {
                    g_delay_left[ch] = gd;
                    g_out_elapsed[ch] = -1;
                }
            } else {
                if (g) g_in_elapsed[ch]++;
                else if (g_in_elapsed[ch] > 0) {
                    // latch this gate's measured length; the output tail
                    // now extends from the real length
                    g_in_len[ch] = g_in_elapsed[ch];
                    g_in_elapsed[ch] = 0;
                }
                if (g_delay_left[ch] > 0 && --g_delay_left[ch] == 0)
                    g_out_elapsed[ch] = 0;
            }

            bool out_high = false;
            if (g_out_elapsed[ch] >= 0) {
                g_out_elapsed[ch]++;
                // length multiplier (Q8): knob 0.125x..4x + length CV,
                // live - re-evaluated every tick so CV lengthens/shortens
                // the gate as it plays
                int32_t mult_q8 = ((int32_t)len[ch] + 1) << 5;
                mult_q8 += (DetentedIn(ch) * 512) / HEMISPHERE_MAX_INPUT_CV;
                if (ch == 0)
                    mult_q8 += (glen_cv.In() * 512) / HEMISPHERE_MAX_INPUT_CV;
                if (mult_q8 < 32) mult_q8 = 32;    // 0.125x
                if (mult_q8 > 1024) mult_q8 = 1024; // 4x

                int32_t target = (g_in_len[ch] * mult_q8) >> 8;
                int32_t min_ticks = (int32_t)minlen << 8;
                if (target < min_ticks) target = min_ticks;
                if (target < 1) target = 1;

                out_high = (g_out_elapsed[ch] < target);
                if (!out_high) g_out_elapsed[ch] = -1;
            }
            GateOut(ch, out_high);
        }

        if (++w_ix[ch] >= buf_len[ch]) w_ix[ch] = 0;
    }
}

FLASHMEM void CVTime::View() {
    DrawInterface();
}

FLASHMEM void CVTime::DrawInterface() {
    // Status line (y 14-26): buffer tier + per-channel output meters + page
    gfxPrint(1, 14, PSTR("Buf "));
    if (buf[0]) {
        if (psram[0]) {
            gfxPrint(60);
            gfxPrint("s");
        } else {
            gfxPrint(buf_len[0] / 16667);
            gfxPrint("s*");
        }
    } else {
        gfxPrint("--");
    }
    gfxPrint(58, 14, 1 + (cursor / 4));

    ForEachChannel(ch) {
        int y = 15 + (ch * 7);
        gfxPrint(28, y, 1 + ch);
        int len_px = ProportionCV(abs(ViewOut(ch)), 30);
        if (mode[ch]) {
            // gate out: level meter
            gfxRect(34, y, (ViewOut(ch) > GATE_THRESHOLD) ? 30 : 1, 5);
        } else {
            gfxRect(34, y, len_px, 5);
        }
    }

    // 4 param rows per page, 4 pages (y 28, 37, 46, 55)
    int page = cursor / 4;
    for (int i = 0; i < 4; i++) DrawRow(page * 4 + i, 28 + i * 9);

    gfxDisplayInputMapEditor();
}

FLASHMEM void CVTime::DrawRow(int row, int y) {
    if (row > R_LAST) return;
    bool sel = (cursor == row);
    int ch = (row >= R_MODE2 && row < R_GCV1) ? 1 : 0;

    switch (row) {
    case R_MODE1:
    case R_MODE2:
        gfxPrint(1, y, "Md");
        gfxPrint(ch + 1);
        gfxPrint(32, y, mode[ch] ? PSTR("Gate") : PSTR("CVDly"));
        if (sel) gfxCursor(31, y + 8, 31, "Mode", "CV delay / Gate");
        break;
    case R_TIME1:
    case R_TIME2:
        gfxPrint(1, y, "Tm");
        gfxPrint(ch + 1);
        gfxPrint(32, y);
        PrintTime(ch);
        if (sel) gfxCursor(31, y + 8, 31, "Time", "Delay / tap");
        break;
    case R_T1CV:
    case R_T2CV:
        gfxPrint(1, y, "T");
        gfxPrint(ch + 1);
        gfxPos(32, y);
        gfxPrint(dt_cv[ch]);
        if (sel) gfxCursor(31, y + 8, 31, "Time CV", dt_cv[ch].InputName());
        break;
    case R_LEN1:
    case R_LEN2:
        gfxPrint(1, y, "Ln");
        gfxPrint(ch + 1);
        gfxPrint(32, y, "x");
        gfxPrint((len[ch] + 1) / 8);
        gfxPrint(".");
        gfxPrint((((len[ch] + 1) % 8) * 125) / 100);
        if (sel) gfxCursor(31, y + 8, 31, "Gate Len", "0.1x .. 4x");
        break;
    case R_GCV1:
        gfxPrint(1, y, "Gcv");
        gfxPos(32, y);
        gfxPrint(glen_cv);
        if (sel) gfxCursor(31, y + 8, 31, "Len CV", glen_cv.InputName());
        break;
    case R_FB1:
    case R_FB2:
        gfxPrint(1, y, "Fb");
        gfxPrint(ch + 1);
        gfxPrint(32, y, (fb[ch] * 100) / 63);
        gfxPrint("%");
        if (sel) gfxCursor(31, y + 8, 31, "Feedback", "CV delay only");
        break;
    case R_FBCV:
        gfxPrint(1, y, "Fcv");
        gfxPos(32, y);
        gfxPrint(fb_cv);
        if (sel) gfxCursor(31, y + 8, 31, "Fdbk CV", fb_cv.InputName());
        break;
    case R_SYNC:
        // string literals -> .rodata in flash (a local static pointer table
        // would land in RAM1 .data under LTO)
        gfxPrint(1, y, "Syn");
        gfxPrint(32, y, sync == 1 ? "/4" : sync == 2 ? "/2" : sync == 3 ? "x1" : "Free");
        if (sel) gfxCursor(31, y + 8, 31, "Clock Sync", "Gate delay tap");
        break;
    case R_SLW1:
    case R_SLW2:
        gfxPrint(1, y, "Sl");
        gfxPrint(ch + 1);
        gfxPrint(32, y, (1 << (slew[ch] + 1)) / 17);
        gfxPrint("ms");
        if (sel) gfxCursor(31, y + 8, 31, "Slew", "Time smoothing");
        break;
    case R_MIN:
        gfxPrint(1, y, "Min");
        gfxPrint(32, y, ((int32_t)minlen << 8) / 17);
        gfxPrint("ms");
        if (sel) gfxCursor(31, y + 8, 31, "Min Length", "Gate floor");
        break;
    }
}

FLASHMEM void CVTime::PrintTime(int ch) {
    int32_t ticks = TicksFromUnits(time_u[ch]);
    if (buf[ch] && ticks > buf_len[ch] - 2) ticks = buf_len[ch] - 2;
    int32_t ms = (ticks * 3) / 50; // ticks -> ms at 16.6 kHz
    if (ms < 1000) {
        gfxPrint(ms);
        gfxPrint("ms");
    } else {
        gfxPrint(ms / 1000);
        gfxPrint(".");
        gfxPrint((ms % 1000) / 100);
        gfxPrint("s");
    }
}

FLASHMEM void CVTime::OnButtonPress() {
    if (CheckEditInputMapPress(
          cursor,
          IndexedInput(R_T1CV, dt_cv[0]),
          IndexedInput(R_T2CV, dt_cv[1]),
          IndexedInput(R_GCV1, glen_cv),
          IndexedInput(R_FBCV, fb_cv)
        ))
        return;
    CursorToggle();
}

FLASHMEM void CVTime::OnEncoderMove(int direction) {
    if (!EditMode()) {
        MoveCursor(cursor, direction, R_LAST);
        return;
    }
    if (EditSelectedInputMap(direction)) return;

    switch (cursor) {
    case R_MODE1: mode[0] = 1 - mode[0]; break;
    case R_MODE2: mode[1] = 1 - mode[1]; break;
    case R_TIME1:
    case R_TIME2: {
        int ch = (cursor == R_TIME2) ? 1 : 0;
        if (time_u[ch] > 700) direction *= 2;
        if (time_u[ch] > 1400) direction *= 2;
        time_u[ch] = constrain(time_u[ch] + direction, 0, 2047);
        sm_delay[ch] = TicksFromUnits(time_u[ch]) * 256;
        break;
    }
    case R_T1CV: dt_cv[0].ChangeSource(direction); break;
    case R_T2CV: dt_cv[1].ChangeSource(direction); break;
    case R_LEN1: len[0] = constrain(len[0] + direction, 0, 31); break;
    case R_LEN2: len[1] = constrain(len[1] + direction, 0, 31); break;
    case R_GCV1: glen_cv.ChangeSource(direction); break;
    case R_FB1: fb[0] = constrain(fb[0] + direction, 0, 63); break;
    case R_FB2: fb[1] = constrain(fb[1] + direction, 0, 63); break;
    case R_FBCV: fb_cv.ChangeSource(direction); break;
    case R_SYNC: sync = constrain(sync + direction, 0, 3); break;
    case R_SLW1: slew[0] = constrain(slew[0] + direction, 0, 15); break;
    case R_SLW2: slew[1] = constrain(slew[1] + direction, 0, 15); break;
    case R_MIN: minlen = constrain(minlen + direction, 0, 63); break;
    }
}

// 64-bit word (62 bits used, 2 spare):
//   mode1(1) mode2(1) sync(2) time1(11) time2(11) len1(5) len2(5)
//   fb1(6) fb2(6) slew1(4) slew2(4) minlen(6)
// CV source maps (16 bits each) live in PhzConfig extended storage
FLASHMEM uint64_t CVTime::OnDataRequest() {
    uint64_t data = PackPackables(
        pack<1>(mode[0]), pack<1>(mode[1]), pack<2>(sync),
        pack<11>(time_u[0]), pack<11>(time_u[1]),
        pack<5>(len[0]), pack<5>(len[1]),
        pack<6>(fb[0]), pack<6>(fb[1]),
        pack<4>(slew[0]), pack<4>(slew[1]),
        pack<6>(minlen)
    );
    SetData(0, dt_cv[0].Pack());
    SetData(1, dt_cv[1].Pack());
    SetData(2, glen_cv.Pack());
    SetData(3, fb_cv.Pack());
    return data;
}

FLASHMEM void CVTime::OnDataReceive(uint64_t data) {
    UnpackPackables(
        data,
        pack<1>(mode[0]), pack<1>(mode[1]), pack<2>(sync),
        pack<11>(time_u[0]), pack<11>(time_u[1]),
        pack<5>(len[0]), pack<5>(len[1]),
        pack<6>(fb[0]), pack<6>(fb[1]),
        pack<4>(slew[0]), pack<4>(slew[1]),
        pack<6>(minlen)
    );
    cfg_loaded = true;
    Sanitize();

    uint64_t v;
    if (GetData(0, v)) dt_cv[0].Unpack((uint16_t)v); else dt_cv[0].Unpack(0);
    if (GetData(1, v)) dt_cv[1].Unpack((uint16_t)v); else dt_cv[1].Unpack(0);
    if (GetData(2, v)) glen_cv.Unpack((uint16_t)v); else glen_cv.Unpack(0);
    if (GetData(3, v)) fb_cv.Unpack((uint16_t)v); else fb_cv.Unpack(0);
}

FLASHMEM void CVTime::SetHelp() {
    //                    "-------" <-- Label size guide
    help[HELP_DIGITAL1] = PSTR("Gate 1");
    help[HELP_DIGITAL2] = PSTR("Gate 2");
    help[HELP_CV1]      = PSTR("CV In1");
    help[HELP_CV2]      = PSTR("CV In2");
    help[HELP_OUT1]     = PSTR("Time 1");
    help[HELP_OUT2]     = PSTR("Time 2");
    help[HELP_EXTRA1] = PSTR("CV/gate delay,");
    help[HELP_EXTRA2] = PSTR("gate length CV");
    //                  "---------------------" <-- Extra text size guide
}
