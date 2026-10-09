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

/* CueLoop - a CV looper / sampler for control voltage.
 *
 * Records In(0) at the 16.6 kHz control tick into a big mono sample buffer
 * (8 s in PSRAM, ~2 s fallback in RAM2) and plays it back through 8
 * user-editable cue points with interpolated, glided playback.
 *
 *  - Trig1 = play/retrigger from the selected cue (sample-playback style)
 *  - Trig2 = record arm toggle (primary); with RecMd=Gate it instead free-runs
 *    recording while Trig2 is held high
 *  - CV2 (In 1) offsets the Cue select by -7..+7 cues (quantized to the 8
 *    cues): set the Cue param to 0 and 0..+6V "keyboards" the buffer
 *  - One-shot plays from the cue to the next cue boundary (or the end of the
 *    recording) then HOLDS the last sample; Loop wraps with an ~5 ms
 *    (80 sample) wrap crossfade; Reverse plays down to the previous boundary
 *  - Recording while playing overdubs at playhead A (Mix %)
 *
 * Interpolation for non-integer playhead positions: None (nearest), Linear,
 * or Cubic (4-point Catmull-Rom). The Glide param is a one-pole output slew
 * toward the interpolated value, smoothing cue jumps, wraps and rate changes.
 *
 * Recorded audio is volatile (RAM only): cue points and settings persist
 * with presets, the sample buffer itself does not.
 *
 * Implementation note: every method lives in FLASHMEM (out-of-line) because
 * the ITCM budget has less than 152 bytes of headroom at the 32K block
 * boundary - even Controller() runs from flash, where it is I-cache-hot at
 * the 16.6 kHz call rate (worst case a few us, well inside the 60 us tick).
 */

// Loop ring with a cue marker in the middle
alignas(8) constexpr uint8_t CUELOOP_ICON[8] = {
    0x3c, 0x42, 0x81, 0x81, 0x99, 0x81, 0x42, 0x3c
};

extern "C" uint8_t external_psram_size;

class CueLoop : public HemisphereApplet {
public:
    static constexpr int CL_CUES = 8;
    static constexpr int CL_PS_SAMPLES = 8 * 16667;   // ~8 s of control ticks in PSRAM
    static constexpr int CL_RAM_SAMPLES = 2 * 16667;  // ~2 s fallback in RAM2
    static constexpr int CL_XFADE = 80;               // ~5 ms wrap crossfade
    static constexpr int CL_POS_STEP = 32;            // cue nudge per detent (~2 ms)
    static constexpr int CL_Q = 256;                  // playhead fixed-point scale

    const char* applet_name() { return "CueLoop"; }
    const uint8_t* applet_icon() { return CUELOOP_ICON; }

    void Start();
    void Controller();
    void View();
    void OnEncoderMove(int direction);
    uint64_t OnDataRequest();
    void OnDataReceive(uint64_t data);

protected:
    void SetHelp();

private:
    enum CLRow {
        R_MODE, R_CUE, R_POS, R_RATE,
        R_REV, R_INTERP, R_GLIDE, R_OUT1,
        R_CLKQ, R_RECLEN, R_RECMODE, R_MIX,
        R_LAST = R_MIX
    };

    // --- Saved configuration (64-bit word + extended storage for cues)
    // Members are zero-initialized by the registry's placement-new; real
    // defaults are applied by Start() unless a preset was already received.
    int8_t mode;        // 0 = one-shot, 1 = loop
    int8_t reverse;
    int8_t rate;        // 0..4 -> 0.25x 0.5x 1x 1.5x 2x
    int8_t interp;      // 0 = nearest, 1 = linear, 2 = cubic (Catmull-Rom)
    int16_t glide;      // output slew, 0-500 ms
    int8_t out1;        // 0-7 = playhead B cue, 8 = envelope follower
    int8_t clkq;        // cue jumps snap to clock edges
    int8_t rec_len_sel; // 0 = Free, 1-5 = 1/2/4/8/16 clock cycles
    int8_t rec_mode;    // 0 = Trig2 toggles arm, 1 = free-run while gated
    int8_t mix;         // overdub mix, 0-100% (100 = replace)
    int8_t cue_param;   // base cue select, 0-7
    int32_t cue_pos[CL_CUES];

    // --- Runtime state (not persisted)
    int32_t* buf;
    int32_t buf_len;
    int32_t rec_len;     // recorded extent (samples)
    int32_t rec_pos;     // capture head
    int32_t rec_target;  // capture length for this pass
    bool rec_armed;
    bool capturing;
    bool last_gate_rec;
    bool cues_auto;      // spread cues evenly after first capture
    bool cfg_loaded;     // preset data received; skip defaulting
    int8_t cue_active;
    int8_t cue_pending;  // clock-quantized jump, -1 = none

    // playheads: 0 = A (Out 0), 1 = B (Out 1)
    int32_t ph_pos[2];   // Q8 sample position
    int32_t ph_hold[2];  // last sample produced (one-shot hold)
    bool ph_active[2];
    int32_t xfade_left[2];
    int32_t xfade_tail[2]; // Q8 position of the continuing tail voice
    simfloat glide_acc[2];
    int32_t env;

    int cursor;

    // playhead step in Q8
    // (rate 0..4 -> 0.25 / 0.5 / 1 / 1.5 / 2 samples per tick)
    int32_t rate_q;

    // --- helpers (defined out-of-line below, all FLASHMEM) ---
    void ApplyDefaults();
    void UpdateRateQ();
    int32_t MaxSample() const;

    // Region for a cue: `from` is where playback starts, `to` is the far
    // boundary in the direction of travel (next cue / end of recording when
    // forward, previous cue / start when reversed).
    void ComputeRegion(int cue_idx, uint32_t &from, uint32_t &to);

    // Interpolated buffer read at Q8 position. Reads are clamped to the
    // recorded extent so unrecorded memory is never played.
    int32_t ReadInterp(int32_t p);

    void JumpCue(int h, int cue_idx);
    void StartPlay();

    // Advance one playhead and produce this tick's target sample
    int32_t HeadTick(int h);

    void DrawInterface();
    void DrawRow(int row, int y);
    void SanitizeCues();
    void FinishCapture();
};

/* -- Function bodies (all in FLASHMEM to conserve ITCM) -- */

FLASHMEM void CueLoop::ApplyDefaults() {
    mode = 1;        // Loop
    reverse = 0;
    rate = 2;        // 1x
    interp = 1;      // Linear
    glide = 0;
    out1 = 0;        // playhead B on cue 0
    clkq = 0;        // free-running cue jumps
    rec_len_sel = 0; // Free
    rec_mode = 0;    // Trig2 toggle
    mix = 100;       // replace
    cue_param = 0;
    cues_auto = true;
    cfg_loaded = true;
}

FLASHMEM void CueLoop::UpdateRateQ() {
    rate_q = 64 * (rate == 0 ? 1 : rate == 1 ? 2 : rate == 2 ? 4 : rate == 3 ? 6 : 8);
}

FLASHMEM int32_t CueLoop::MaxSample() const {
    return rec_len > 0 ? rec_len - 1 : -1;
}

FLASHMEM void CueLoop::Start() {
    if (!buf) {
        // 8 s buffer in PSRAM when available (audio applet pattern)...
        if (external_psram_size > 0) {
            buf = (int32_t*)extmem_calloc(CL_PS_SAMPLES, sizeof(int32_t));
            if (buf) {
                buf_len = CL_PS_SAMPLES;
                // extmem_calloc does not guarantee zero-fill
                for (int32_t i = 0; i < buf_len; i++) buf[i] = 0;
            }
        }
        // ...otherwise ~2 s from the RAM2 heap (CVRecV2 pattern),
        // shrinking if the heap is crowded
        if (!buf) {
            int32_t n = CL_RAM_SAMPLES;
            while (n >= 4096 && !buf) {
                buf = (int32_t*)calloc(n, sizeof(int32_t));
                if (buf) buf_len = n;
                n /= 2;
            }
        }
    }
    if (!cfg_loaded) ApplyDefaults();
    rec_pos = 0;
    rec_target = 0;
    rec_armed = false;
    capturing = false;
    last_gate_rec = false;
    cue_active = cue_param;
    cue_pending = -1;
    ph_active[0] = ph_active[1] = false;
    ph_pos[0] = ph_pos[1] = 0;
    ph_hold[0] = ph_hold[1] = 0;
    xfade_left[0] = xfade_left[1] = 0;
    glide_acc[0] = glide_acc[1] = 0;
    env = 0;
    cursor = 0;
    UpdateRateQ();
    SanitizeCues();
}

FLASHMEM void CueLoop::SanitizeCues() {
    if (buf_len < 1) return; // before allocation: leave cues for Start()
    int32_t max = buf_len - 1;
    for (int i = 0; i < CL_CUES; i++) {
        cue_pos[i] = constrain(cue_pos[i], 0, max);
    }
}

FLASHMEM void CueLoop::FinishCapture() {
    capturing = false;
    rec_armed = false;
    if (rec_pos > rec_len) rec_len = rec_pos;
    // First capture: spread the cue points evenly for instant playability
    if (cues_auto && rec_len > CL_CUES) {
        for (int i = 0; i < CL_CUES; i++) cue_pos[i] = rec_len * i / CL_CUES;
    }
}

FLASHMEM void CueLoop::ComputeRegion(int cue_idx, uint32_t &from, uint32_t &to) {
    uint32_t last = (uint32_t)(rec_len > 0 ? rec_len - 1
                              : (buf_len > 0 ? buf_len - 1 : 0));
    uint32_t c = (uint32_t)cue_pos[cue_idx];
    if (c > last) c = last;
    from = c;
    if (!reverse) {
        to = (rec_len > 0) ? (uint32_t)rec_len : (uint32_t)buf_len;
        for (int i = 0; i < CL_CUES; i++) {
            uint32_t p = (uint32_t)cue_pos[i];
            if (p > c && p < to) to = p;
        }
    } else {
        to = 0;
        for (int i = 0; i < CL_CUES; i++) {
            uint32_t p = (uint32_t)cue_pos[i];
            if (p < c && p > to) to = p;
        }
    }
}

FLASHMEM int32_t CueLoop::ReadInterp(int32_t p) {
    int32_t last = MaxSample();
    if (last < 0) return 0;
    int32_t idx = p >> 8;
    int32_t frac = p & 0xff;
    if (idx < 0) { idx = 0; frac = 0; }
    if (idx > last) { idx = last; frac = 0; }
    if (interp == 0 || idx >= last) return buf[idx];

    int32_t a = buf[idx];
    int32_t b = buf[idx + 1];
    if (interp == 1) return a + (((b - a) * frac) >> 8);

    // Catmull-Rom, Horner's rule in Q8 (int32-safe at CV ranges)
    int32_t p0 = buf[idx > 0 ? idx - 1 : 0];
    int32_t p3 = buf[idx + 2 <= last ? idx + 2 : last];
    int32_t t1 = 3 * (a - b) + p3 - p0;
    int32_t t2 = 2 * p0 - 5 * a + 4 * b - p3;
    int32_t t3 = b - p0;
    return a + ((frac * (t3 + ((frac * (t2 + ((frac * t1) >> 8)) >> 8))) >> 9));
}

FLASHMEM void CueLoop::JumpCue(int h, int cue_idx) {
    uint32_t from, to;
    ComputeRegion(cue_idx, from, to);
    ph_pos[h] = (int32_t)from * CL_Q;
    ph_active[h] = (rec_len > 0) && (reverse ? (from > to) : (from < to));
    xfade_left[h] = 0;
}

FLASHMEM void CueLoop::StartPlay() {
    JumpCue(0, cue_active);
    if (out1 < CL_CUES) JumpCue(1, out1);
    else ph_active[1] = false;
}

FLASHMEM int32_t CueLoop::HeadTick(int h) {
    if (!ph_active[h]) return ph_hold[h];

    int cue_idx = (h == 0) ? cue_active : out1;
    uint32_t from, to;
    ComputeRegion(cue_idx, from, to);

    int32_t step = rate_q;
    if (reverse) step = -step;

    int32_t v;
    if (xfade_left[h] > 0) {
        // wrap crossfade: blend the continuing tail into the wrapped head
        int32_t head = ReadInterp(ph_pos[h]);
        int32_t tail = ReadInterp(xfade_tail[h]);
        int32_t f = ((CL_XFADE - xfade_left[h]) << 8) / CL_XFADE;
        v = ((tail * (256 - f)) + (head * f)) >> 8;
        xfade_tail[h] += step;
        xfade_left[h]--;
    } else {
        v = ReadInterp(ph_pos[h]);
    }
    ph_hold[h] = v;

    ph_pos[h] += step;

    bool hit = reverse ? ((ph_pos[h] >> 8) <= (int32_t)to)
                       : ((ph_pos[h] >> 8) >= (int32_t)to);
    if (hit) {
        if (mode) { // loop: wrap with crossfade
            int32_t span = reverse ? ((int32_t)from - (int32_t)to)
                                   : ((int32_t)to - (int32_t)from);
            ph_pos[h] = (int32_t)from * CL_Q;
            if (span >= 2 * CL_XFADE) {
                xfade_left[h] = CL_XFADE;
                xfade_tail[h] = (int32_t)to * CL_Q;
            } else {
                xfade_left[h] = 0;
            }
        } else { // one-shot: stop and hold the last sample
            ph_pos[h] = (int32_t)to * CL_Q;
            ph_active[h] = false;
        }
    }
    return v;
}

// The Controller runs in the 60 us ISR tick - no allocation, no I/O
FLASHMEM void CueLoop::Controller() {
    if (!buf) { Out(0, 0); Out(1, 0); return; }

    // ---- Record arming -------------------------------------------------
    if (Clock(1) && rec_mode == 0) rec_armed = !rec_armed;
    bool gate_rec = (rec_mode == 1) ? Gate(1) : rec_armed;
    bool rec_rise = gate_rec && !last_gate_rec;
    last_gate_rec = gate_rec;

    // a fresh capture pass starts on the arming edge while stopped
    if (rec_rise && !ph_active[0] && !capturing) {
        capturing = true;
        rec_pos = 0;
        if (rec_len_sel == 0) rec_target = buf_len;
        else {
            uint32_t cyc = ClockCycleTicks(0);
            rec_target = cyc ? (int32_t)cyc * (1 << (rec_len_sel - 1)) : buf_len;
            if (rec_target > buf_len) rec_target = buf_len;
            if (rec_target < 1) rec_target = buf_len;
        }
    }

    // ---- Cue select: param + CV2, quantized to the 8 cues --------------
    int cue_now = constrain(cue_param + Proportion(In(1), HEMISPHERE_MAX_INPUT_CV, 7),
                            0, CL_CUES - 1);
    if (cue_now != cue_active) {
        if (clkq) cue_pending = (int8_t)cue_now;
        else {
            cue_active = (int8_t)cue_now;
            cue_pending = -1;
            if (ph_active[0]) JumpCue(0, cue_active); // live cue jump
        }
    }

    // ---- Trigger: play/retrigger from the current cue ------------------
    if (Clock(0)) {
        if (capturing) FinishCapture();
        if (cue_pending >= 0) {
            cue_active = cue_pending;
            cue_pending = -1;
        }
        StartPlay();
    }

    // ---- Recording -----------------------------------------------------
    if (gate_rec && capturing) {
        if (rec_pos < rec_target && rec_pos < buf_len) buf[rec_pos++] = In(0);
        if (rec_pos >= rec_target || rec_pos >= buf_len) FinishCapture();
    } else if (capturing && !gate_rec) {
        FinishCapture();
    }
    if (gate_rec && ph_active[0]) {
        // record-while-playing: overdub at playhead A
        int32_t idx = ph_pos[0] >> 8;
        if (idx >= 0 && idx < buf_len) {
            int32_t in = In(0);
            buf[idx] = (mix >= 100) ? in : (buf[idx] * (100 - mix) + in * mix) / 100;
        }
    }

    // ---- Playback ------------------------------------------------------
    int32_t tau = ((int32_t)glide * 83) / 5; // ms -> ticks (16.6 kHz)
    int32_t target = HeadTick(0);
    if (tau < 1) glide_acc[0] = int2simfloat(target);
    else glide_acc[0] += (int2simfloat(target) - glide_acc[0]) / tau;
    Out(0, simfloat2int(glide_acc[0]));

    if (out1 < CL_CUES) {
        target = HeadTick(1);
        if (tau < 1) glide_acc[1] = int2simfloat(target);
        else glide_acc[1] += (int2simfloat(target) - glide_acc[1]) / tau;
        Out(1, simfloat2int(glide_acc[1]));
    } else {
        // envelope follower of the record input
        int32_t a = In(0);
        if (a < 0) a = -a;
        if (a > env) env += (a - env) >> 5;
        else env -= env >> 9;
        Out(1, env);
    }

    if (capturing) Out(0, In(0)); // monitor the input while capturing
}

FLASHMEM void CueLoop::View() {
    DrawInterface();
}

FLASHMEM void CueLoop::DrawInterface() {
    int32_t span = rec_len > 0 ? rec_len : buf_len;

    // Buffer position bar with cue tick marks (y 14-19)
    gfxFrame(0, 14, 63, 6);
    if (buf && span > 1) {
        for (int i = 0; i < CL_CUES; i++) {
            int x = 1 + (int)((int64_t)cue_pos[i] * 61 / span);
            if (x > 62) x = 62;
            gfxLine(x, 15, x, 18);
        }
        if (rec_len > 0 && rec_len < buf_len) {
            int x = 1 + (int)((int64_t)rec_len * 61 / buf_len);
            gfxLine(x, 14, x, 19, true);
        }
        if (ph_active[0]) {
            int x = 1 + (int)(((int64_t)(ph_pos[0] >> 8) * 61) / span);
            if (x > 61) x = 61;
            gfxInvert(x, 14, 2, 6);
        }
        if (out1 < CL_CUES && ph_active[1]) {
            int x = 1 + (int)(((int64_t)(ph_pos[1] >> 8) * 61) / span);
            if (x > 61) x = 61;
            gfxFrame(x, 14, 2, 6);
        }
    }

    // Status line (y 20-27): transport icons + readout + buffer size + page
    gfxIcon(1, 20, mode ? LOOP_ICON : PLAYONCE_ICON);
    if (capturing) {
        if (CursorBlink()) gfxIcon(11, 20, RECORD_ICON);
    } else if (ph_active[0]) {
        gfxIcon(11, 20, PLAY_ICON);
    }
    if (cursor == R_POS && buf && rec_len > 0) {
        int pct = (int)(cue_pos[cue_param] * 100 / rec_len);
        gfxPrint(21, 20, pct);
        gfxPrint("%");
    } else {
        gfxPrint(21, 20, interp == 0 ? "None" : (interp == 1 ? "Lin" : "Cub"));
    }
    gfxPrint(46, 20, buf_len == CL_PS_SAMPLES ? "8s" : (buf ? "2s" : "--"));
    gfxPrint(58, 20, 1 + (cursor / 4));

    // 4 param rows per page, 3 pages (y 28, 37, 46, 55)
    int page = cursor / 4;
    for (int i = 0; i < 4; i++) DrawRow(page * 4 + i, 28 + i * 9);
}

FLASHMEM void CueLoop::DrawRow(int row, int y) {
    if (row > R_LAST) return;
    bool sel = (cursor == row);
    switch (row) {
    case R_MODE:
        gfxPrint(1, y, "Mode");
        gfxPrint(32, y, mode ? "Loop" : "One");
        if (sel) gfxCursor(31, y + 8, 31, "Mode");
        break;
    case R_CUE:
        gfxPrint(1, y, "Cue");
        gfxPrint(32, y, (int)cue_active);
        if (sel) gfxCursor(31, y + 8, 12, "Cue", "CV2 adds");
        break;
    case R_POS:
        gfxPrint(1, y, "Pos");
        if (buf && rec_len > 0) {
            gfxPrint(32, y, (int)((cue_pos[cue_param] * 5) / 83)); // ticks -> ms
            gfxPrint("ms");
        } else {
            gfxPrint(32, y, "--");
        }
        if (sel) gfxCursor(31, y + 8, 31, "Cue Pos", "Nudge cue");
        break;
    case R_RATE: {
        static const char* const RATES[5] = {"0.25x", "0.5x", "1x", "1.5x", "2x"};
        gfxPrint(1, y, "Rate");
        gfxPrint(32, y, RATES[rate]);
        if (sel) gfxCursor(31, y + 8, 31, "Rate");
        break;
    }
    case R_REV:
        gfxPrint(1, y, "Rev");
        gfxPrint(32, y, reverse ? "On" : "Off");
        if (sel) gfxCursor(31, y + 8, 31, "Reverse");
        break;
    case R_INTERP:
        gfxPrint(1, y, "Intrp");
        gfxPrint(32, y, interp == 0 ? "None" : (interp == 1 ? "Lin" : "Cubic"));
        if (sel) gfxCursor(31, y + 8, 31, "Interp");
        break;
    case R_GLIDE:
        gfxPrint(1, y, "Glide");
        gfxPrint(32, y, glide);
        gfxPrint("ms");
        if (sel) gfxCursor(31, y + 8, 31, "Glide", "Output slew");
        break;
    case R_OUT1:
        gfxPrint(1, y, "Out2");
        if (out1 < CL_CUES) {
            gfxPrint(32, y, "PB:");
            gfxPrint((int)out1);
        } else {
            gfxPrint(32, y, "Env");
        }
        if (sel) gfxCursor(31, y + 8, 31, "Out2 Src");
        break;
    case R_CLKQ:
        gfxPrint(1, y, "ClkQ");
        gfxPrint(32, y, clkq ? "Snap" : "Free");
        if (sel) gfxCursor(31, y + 8, 31, "Cue Snap", "Jump at clocks");
        break;
    case R_RECLEN:
        gfxPrint(1, y, "RecL");
        if (rec_len_sel == 0) gfxPrint(32, y, "Free");
        else {
            gfxPrint(32, y, 1 << (rec_len_sel - 1));
            gfxPrint("cyc");
        }
        if (sel) gfxCursor(31, y + 8, 31, "Rec Length");
        break;
    case R_RECMODE:
        gfxPrint(1, y, "RecMd");
        gfxPrint(32, y, rec_mode ? "Gate" : "Tog");
        if (sel) gfxCursor(31, y + 8, 31, "Rec Mode", "Trig2 behavior");
        break;
    case R_MIX:
        gfxPrint(1, y, "Mix");
        gfxPrint(32, y, mix);
        gfxPrint("%");
        if (sel) gfxCursor(31, y + 8, 31, "Overdub Mix");
        break;
    }
}

FLASHMEM void CueLoop::OnEncoderMove(int direction) {
    if (!EditMode()) {
        MoveCursor(cursor, direction, R_LAST);
        return;
    }
    switch (cursor) {
    case R_MODE: mode = 1 - mode; break;
    case R_CUE: cue_param = constrain(cue_param + direction, 0, CL_CUES - 1); break;
    case R_POS: {
        int32_t max = (buf_len > 0 ? buf_len : CL_PS_SAMPLES) - 1;
        cue_pos[cue_param] = constrain(cue_pos[cue_param] + direction * CL_POS_STEP, 0, max);
        cues_auto = false;
        break;
    }
    case R_RATE: rate = constrain(rate + direction, 0, 4); UpdateRateQ(); break;
    case R_REV: reverse = 1 - reverse; break;
    case R_INTERP: interp = constrain(interp + direction, 0, 2); break;
    case R_GLIDE: glide = constrain(glide + direction * 5, 0, 500); break;
    case R_OUT1: out1 = constrain(out1 + direction, 0, CL_CUES); break;
    case R_CLKQ: clkq = 1 - clkq; break;
    case R_RECLEN: rec_len_sel = constrain(rec_len_sel + direction, 0, 5); break;
    case R_RECMODE: rec_mode = 1 - rec_mode; break;
    case R_MIX: mix = constrain(mix + direction, 0, 100); break;
    }
}

FLASHMEM uint64_t CueLoop::OnDataRequest() {
    uint64_t data = 0;
    Pack(data, PackLocation {0, 1}, (uint64_t)mode);
    Pack(data, PackLocation {1, 1}, (uint64_t)reverse);
    Pack(data, PackLocation {2, 3}, (uint64_t)rate);
    Pack(data, PackLocation {5, 2}, (uint64_t)interp);
    Pack(data, PackLocation {7, 9}, (uint64_t)glide);
    Pack(data, PackLocation {16, 4}, (uint64_t)out1);
    Pack(data, PackLocation {20, 1}, (uint64_t)clkq);
    Pack(data, PackLocation {21, 3}, (uint64_t)rec_len_sel);
    Pack(data, PackLocation {24, 1}, (uint64_t)rec_mode);
    Pack(data, PackLocation {25, 7}, (uint64_t)mix);
    Pack(data, PackLocation {32, 3}, (uint64_t)cue_param);
    // redundant fractional copies of the first two cues, used only as a
    // fallback if the extended storage is missing from a preset
    int32_t span = buf_len > 1 ? buf_len : 1;
    Pack(data, PackLocation {35, 12}, (uint64_t)(cue_pos[0] * 4095 / span));
    Pack(data, PackLocation {47, 12}, (uint64_t)(cue_pos[1] * 4095 / span));

    // exact cue positions go to extended storage (keys 0-7, one per cue)
    for (int i = 0; i < CL_CUES; i++) {
        SetData(i, (uint64_t)(uint32_t)cue_pos[i]);
    }
    return data;
}

FLASHMEM void CueLoop::OnDataReceive(uint64_t data) {
    mode = Unpack(data, PackLocation {0, 1});
    reverse = Unpack(data, PackLocation {1, 1});
    rate = constrain(Unpack(data, PackLocation {2, 3}), 0, 4);
    UpdateRateQ();
    interp = constrain(Unpack(data, PackLocation {5, 2}), 0, 2);
    glide = constrain(Unpack(data, PackLocation {7, 9}), 0, 500);
    out1 = constrain(Unpack(data, PackLocation {16, 4}), 0, CL_CUES);
    clkq = Unpack(data, PackLocation {20, 1});
    rec_len_sel = constrain(Unpack(data, PackLocation {21, 3}), 0, 5);
    rec_mode = Unpack(data, PackLocation {24, 1});
    mix = constrain(Unpack(data, PackLocation {25, 7}), 0, 100);
    cue_param = constrain(Unpack(data, PackLocation {32, 3}), 0, CL_CUES - 1);
    cfg_loaded = true;

    int32_t span = buf_len > 1 ? buf_len : CL_PS_SAMPLES;
    for (int i = 0; i < CL_CUES; i++) {
        uint64_t v;
        if (GetData(i, v)) {
            cue_pos[i] = (int32_t)(uint32_t)v;
        } else if (i == 0) {
            cue_pos[i] = (int32_t)(Unpack(data, PackLocation {35, 12}) * span / 4095);
        } else if (i == 1) {
            cue_pos[i] = (int32_t)(Unpack(data, PackLocation {47, 12}) * span / 4095);
        } else {
            cue_pos[i] = 0;
        }
    }
    cues_auto = false;
    cue_active = cue_param;
    SanitizeCues();
}

FLASHMEM void CueLoop::SetHelp() {
    //                    "-------" <-- Label size guide
    help[HELP_DIGITAL1] = "Play/Trg";
    help[HELP_DIGITAL2] = "RecArm";
    help[HELP_CV1]      = "Rec In";
    help[HELP_CV2]      = "CueSel";
    help[HELP_OUT1]     = "Loop A";
    help[HELP_OUT2]     = "B / Env";
    help[HELP_EXTRA1] = "CV looper w/cues";
    // buffer-size note: without PSRAM the buffer is ~2 s instead of 8 s
    help[HELP_EXTRA2] = (buf && buf_len != CL_PS_SAMPLES) ? "No PSRAM: 2s buf!"
                                                          : "a sampler for CV!";
    //                  "---------------------" <-- Extra text size guide
}
