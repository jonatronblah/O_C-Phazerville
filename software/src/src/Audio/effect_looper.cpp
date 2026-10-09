/* Audio Library for Teensy 4.X
 * Clockable stereo looper - see effect_looper.h for the full contract.
 * Created for Phazerville Suite (generated with AI assistance).
 *
 * Placement policy (multiboot slot0.ld): every method here is FLASHMEM
 * (runs from flash, *(.flashmem*) section), update() included, so this
 * translation unit adds nothing to the RAM1/ITCM budget.
 */
#if defined(ARDUINO_TEENSY41)

#include "effect_looper.h"
#include "../../dsputils.h" // Clip16
#include <Audio.h>           // AudioNoInterrupts()/AudioInterrupts()
#include <smalloc.h>         // extmem_calloc / extmem_free

extern "C" uint8_t external_psram_size;

// ---- lifecycle ------------------------------------------------------------

FLASHMEM bool AudioLooper::begin() {
  end();
  if (external_psram_size > 0) {
    buf_ = (int16_t*)extmem_calloc(MAX_FRAMES * 2, sizeof(int16_t));
    if (buf_) {
      in_psram_ = true;
      cap_frames_ = MAX_FRAMES;
    }
  }
  if (!buf_) {
    // No PSRAM (T4.0 or PSRAM-less T4.1): progressively smaller buffer from
    // the fast RAM2 heap, so a low-RAM build still loops (shorter) audio.
    for (size_t frames = FALLBACK_FRAMES; frames >= FALLBACK_FRAMES / 8;
         frames /= 2) {
      buf_ = (int16_t*)malloc(frames * 2 * sizeof(int16_t));
      if (buf_) {
        in_psram_ = false;
        cap_frames_ = frames;
        memset(buf_, 0, frames * 2 * sizeof(int16_t));
        break;
      }
    }
  }
  if (buf_) {
    AudioNoInterrupts();
    loop_len_ = 0;
    loop_len_q13_ = 0;
    pos_q13_ = 0;
    target_frames_ = 0;
    state_ = LOOPER_STOPPED;
    write_ix_ = 0;
    rec_frames_ = 0;
    pos_req_ = POSREQ_NONE;
    splice_len_ = 0;
    rec_done_ = false;
    oneshot_done_ = false;
    peak_ = 0;
    AudioInterrupts();
  }
  return buf_ != nullptr;
}

FLASHMEM void AudioLooper::end() {
  AudioNoInterrupts();
  state_ = LOOPER_STOPPED;
  loop_len_ = 0;
  loop_len_q13_ = 0;
  target_frames_ = 0;
  splice_len_ = 0;
  pos_req_ = POSREQ_NONE;
  int16_t* b = buf_;
  buf_ = nullptr;
  cap_frames_ = 0;
  AudioInterrupts();
  if (b) {
    if (in_psram_) extmem_free(b);
    else free(b);
    in_psram_ = false;
  }
}

// ---- transport transitions -------------------------------------------------

FLASHMEM void AudioLooper::startRecording(uint32_t target_frames) {
  if (!buf_) return;
  // control-written words first, command latch last (consumed at the next
  // audio block boundary by update(), keeping the transition block-atomic)
  target_frames_ = (target_frames >= 2 && target_frames <= cap_frames_)
    ? target_frames : 0;
  loop_len_ = 0;
  loop_len_q13_ = 0;
  rec_done_ = false;
  oneshot_done_ = false;
  state_ = LOOPER_RECORDING;
  req_pos_q13_ = 0;
  pos_req_ = POSREQ_RESTART_REC;
}

FLASHMEM void AudioLooper::stopRecordingToPlay() {
  if (!buf_) return;
  // rec_frames_ lags the ISR by at most one block (128 samples) - the loop
  // is simply up to 2.7 ms shorter than the fully-written tail. Acceptable.
  uint32_t frames = rec_frames_;
  if (target_frames_ != 0 && target_frames_ < frames) frames = target_frames_;
  target_frames_ = 0;
  rec_done_ = false;
  if (frames >= 2) {
    loop_len_ = frames;
    loop_len_q13_ = frames << 13;
    splice_len_ = frames; // applied from mainloop() via commitSplice()
    state_ = LOOPER_PLAYING;
    req_pos_q13_ = 0;
    pos_req_ = POSREQ_RESET;
  } else {
    loop_len_ = 0;
    loop_len_q13_ = 0;
    splice_len_ = 0;
    state_ = LOOPER_STOPPED;
    pos_req_ = POSREQ_RESET;
  }
}

FLASHMEM void AudioLooper::spliceLoop(uint32_t frames) {
  if (!buf_ || frames < 4 || frames > cap_frames_) return;
  int n = XFADE_LEN;
  if (frames < (uint32_t)(2 * n)) n = (int)(frames / 2);
  const uint32_t tail0 = frames - (uint32_t)n;
  for (int i = 0; i < n; i++) {
    const int32_t t = (int32_t)(((uint32_t)i << 15) / (uint32_t)n);
    for (int c = 0; c < 2; c++) {
      const int32_t head = buf_[(i << 1) | c];
      const int32_t ti = (int32_t)(((tail0 + (uint32_t)i) << 1) | (uint32_t)c);
      const int32_t tail = buf_[ti];
      buf_[ti] = Clip16((tail * (32768 - t) + head * t) >> 15);
    }
  }
}

FLASHMEM void AudioLooper::commitSplice() {
  uint32_t n = splice_len_;
  if (!n) return;
  splice_len_ = 0;
  AudioNoInterrupts();
  spliceLoop(n);
  AudioInterrupts();
}

FLASHMEM void AudioLooper::setOverdub(bool on) {
  if (loop_len_ < 2) return;
  const uint8_t s = state_;
  if (s != LOOPER_PLAYING && s != LOOPER_OVERDUB) return;
  state_ = on ? (uint8_t)LOOPER_OVERDUB : (uint8_t)LOOPER_PLAYING;
}

FLASHMEM void AudioLooper::stop() {
  state_ = LOOPER_STOPPED; // update() fades to silence on the state change
}

FLASHMEM void AudioLooper::restartPlayback(bool oneshot) {
  if (loop_len_ < 2) return;
  oneshot_ = oneshot;
  oneshot_done_ = false;
  state_ = LOOPER_PLAYING;
  req_pos_q13_ = 0;
  pos_req_ = POSREQ_RESET;
}

FLASHMEM void AudioLooper::restartWithLength(uint32_t frames) {
  if (!buf_ || frames < 2) return;
  if (frames > cap_frames_) frames = cap_frames_;
  loop_len_ = frames;
  loop_len_q13_ = frames << 13;
  req_pos_q13_ = 0;
  pos_req_ = POSREQ_RESET;
}

FLASHMEM void AudioLooper::setPlayMode(uint8_t mode) {
  if (mode >= MODE_COUNT) mode = MODE_LOOP;
  play_mode_ = mode;
  applySpeedForMode();
}

FLASHMEM void AudioLooper::applySpeedForMode() {
  switch (play_mode_) {
    case MODE_REVERSE: speed_q13_ = -8192; break;
    case MODE_HALF:    speed_q13_ = 4096; break;
    case MODE_DOUBLE:  speed_q13_ = 16384; break;
    default:           speed_q13_ = 8192; break; // LOOP and SCRUB
  }
}

FLASHMEM void AudioLooper::setDubLevel(uint8_t percent) {
  if (percent > 100) percent = 100;
  dub_q15_ = (int32_t)(((uint32_t)percent * 32768u) / 100u);
}

FLASHMEM void AudioLooper::setFeedback(uint8_t percent) {
  if (percent > 100) percent = 100;
  feedback_q15_ = (int32_t)(((uint32_t)percent * 32768u) / 100u);
}

FLASHMEM void AudioLooper::setScrubTarget(uint32_t pos_q13) {
  const uint32_t l = loop_len_q13_;
  if (pos_q13 > l) pos_q13 = l ? (l - 1) : 0;
  scrub_target_q13_ = pos_q13;
}

// ---- audio ISR -------------------------------------------------------------
// Per-sample work: linear-interpolated read, optional overdub write, Q13
// playhead advance with wrap, output-domain click-fade on discontinuities.
// No allocation and no transport transitions (done-flags only).
//
// FLASHMEM on purpose: the RAM1/ITCM bank budget is full (the hard gate is
// RAM1 free >= ~19 KB), so even the audio ISR runs from flash here.  That
// is functionally fine - the flash cache absorbs it and the PSRAM buffer
// accesses dominate - it is just somewhat slower than ITCM execution.

FLASHMEM void AudioLooper::update(void) {
  audio_block_t* inl = receiveReadOnly(0);
  audio_block_t* inr = receiveReadOnly(1);

  int16_t* buf = buf_;
  if (!buf) {
    if (inl) release(inl);
    if (inr) release(inr);
    return;
  }

  audio_block_t* outl = allocate();
  audio_block_t* outr = allocate();
  if (!outl || !outr) {
    if (outl) release(outl);
    if (outr) release(outr);
    if (inl) release(inl);
    if (inr) release(inr);
    return;
  }

  // consume the block-boundary command latch
  const uint8_t preq = pos_req_;
  if (preq) {
    pos_req_ = POSREQ_NONE;
    pos_q13_ = req_pos_q13_;
    if (preq == POSREQ_RESTART_REC) {
      write_ix_ = 0;
      rec_frames_ = 0;
    }
    xfade_left_ = XFADE_LEN;
  }

  // snapshot shared state into locals
  uint32_t loop_len = loop_len_;
  uint32_t loop_q13 = loop_len_q13_;
  const uint32_t cap = cap_frames_;
  const uint32_t target = target_frames_;
  const uint8_t state = state_;
  const int32_t speed = speed_q13_;
  const int32_t dub = dub_q15_;
  const int32_t fb = feedback_q15_;
  const bool replace = rec_replace_;
  const bool oneshot = oneshot_;
  const uint8_t mode = play_mode_;
  uint32_t pos = pos_q13_;
  uint32_t wix = write_ix_;
  uint32_t recf = rec_frames_;
  int32_t xfade = xfade_left_;
  int32_t hl = hold_l_, hr = hold_r_;
  int32_t peak = peak_;
  bool finished = oneshot_done_ && oneshot;

  if (mode != last_mode_) {
    xfade = XFADE_LEN; // mode jump: click-prevention fade
    last_mode_ = mode;
    if (mode == MODE_SCRUB) scrub_target_q13_ = pos;
  }
  if (state != last_state_) {
    xfade = XFADE_LEN; // transport change: fade out / in
    last_state_ = state;
  }

  // recording runs until the target length (0 = until the applet stops us)
  bool rec_active = (state == LOOPER_RECORDING) && !rec_done_ && !finished;

  for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
    const int16_t il = inl ? inl->data[i] : 0;
    const int16_t ir = inr ? inr->data[i] : 0;
    int32_t ol = 0, orr = 0;

    if (rec_active) {
      if (wix < cap) {
        buf[wix << 1] = il;
        buf[(wix << 1) | 1] = ir;
        wix++;
        recf++;
        pos = wix << 13;
      }
      const bool hit = (wix >= cap) || (target != 0 && recf >= target);
      if (hit && state_ == LOOPER_RECORDING) {
        // auto-stop: commit the length here (data only - the applet flips
        // the transport in Controller() and runs the tail/head splice)
        const uint32_t frames = (target != 0) ? target : recf;
        rec_active = false;
        rec_done_ = true;
        if (frames >= 2) {
          loop_len = frames;
          loop_q13 = frames << 13;
          loop_len_ = frames;
          loop_len_q13_ = loop_q13;
        }
        pos = 0;
        xfade = XFADE_LEN;
      }
    } else if (!finished && state != LOOPER_STOPPED && loop_len >= 2) {
      if (mode == MODE_SCRUB) {
        // one-pole playhead slewing toward the CV target (~1.3 ms): the
        // click-prevention for scrubbing (no per-step fades)
        const int64_t d = (int64_t)scrub_target_q13_ - (int64_t)pos;
        pos += (uint32_t)(d >> 6);
      }

      uint32_t i0 = pos >> 13;
      if (i0 >= loop_len) i0 = loop_len - 1;
      uint32_t i1 = i0 + 1;
      if (i1 >= loop_len) i1 = 0;
      const int32_t frac = (int32_t)(pos & 0x1FFF);

      const int32_t l0 = buf[i0 << 1], r0 = buf[(i0 << 1) | 1];
      const int32_t l1 = buf[i1 << 1], r1 = buf[(i1 << 1) | 1];
      ol = l0 + (((l1 - l0) * frac) >> 13);
      orr = r0 + (((r1 - r0) * frac) >> 13);

      if (state == LOOPER_OVERDUB && (inl || inr)) {
        // Mix:     buf = old*feedback + in*dub
        // Replace: buf = old*feedback*(1-dub) + in*dub
        const int32_t oldl = buf[i0 << 1], oldr = buf[(i0 << 1) | 1];
        const int32_t keep = replace ? ((fb * (32768 - dub)) >> 15) : fb;
        buf[i0 << 1] = Clip16(((oldl * keep) + (il * dub)) >> 15);
        buf[(i0 << 1) | 1] = Clip16(((oldr * keep) + (ir * dub)) >> 15);
      }

      // playhead advance with wrap (all 32-bit math; |speed| <= 2.0x)
      if (mode != MODE_SCRUB) {
        if (speed >= 0) {
          uint32_t nx = pos + (uint32_t)speed;
          if (nx >= loop_q13) {
            nx -= loop_q13;
            if (oneshot) {
              finished = true;
              oneshot_done_ = true;
              nx = loop_q13 - 8192;
            } else {
              xfade = XFADE_LEN; // loop wrap: fade from the last sample
            }
          }
          pos = nx;
        } else {
          const uint32_t sp = (uint32_t)(-speed);
          if (pos < sp) {
            pos = loop_q13 - (sp - pos);
            xfade = XFADE_LEN;
          } else {
            pos -= sp;
          }
        }
      }
    }

    // output-domain click fade: crossfade from the last emitted sample
    if (xfade > 0) {
      const int32_t t = (int32_t)(((uint32_t)(XFADE_LEN - xfade) << 15)
                                  / (uint32_t)XFADE_LEN);
      ol = (hl * (32768 - t) + ol * t) >> 15;
      orr = (hr * (32768 - t) + orr * t) >> 15;
      xfade--;
    }
    hl = ol;
    hr = orr;

    if (ol > peak) peak = ol;
    if (-ol > peak) peak = -ol;
    if (orr > peak) peak = orr;
    if (-orr > peak) peak = -orr;

    outl->data[i] = Clip16(ol);
    outr->data[i] = Clip16(orr);
  }

  pos_q13_ = pos;
  write_ix_ = wix;
  rec_frames_ = recf;
  xfade_left_ = xfade;
  hold_l_ = (int16_t)hl;
  hold_r_ = (int16_t)hr;
  peak_ = (uint16_t)peak;

  transmit(outl, 0);
  transmit(outr, 1);
  release(outl);
  release(outr);
  if (inl) release(inl);
  if (inr) release(inr);
}

#endif // ARDUINO_TEENSY41
