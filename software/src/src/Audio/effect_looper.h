/* Audio Library for Teensy 4.X
 * Clockable stereo looper with fractional playhead, scrubbing, overdub and
 * feedback dub.  Created for Phazerville Suite (generated with AI assistance).
 *
 * The stereo loop buffer is interleaved int16 (L,R) and lives in PSRAM
 * (~8 s).  On PSRAM-less hardware (T4.0 / T4.1 without PSRAM) it falls back
 * to a shorter malloc()'d RAM2 buffer (see begin()).
 *
 * MEMORY LAYOUT NOTE: the multiboot slot0.ld linker script runs default
 * .text in ITCM/RAM1 (32KB-bank granular) and only *(.flashmem*) runs from
 * flash.  Therefore every method here (update() included - the ITCM bank
 * budget is exhausted) is defined out-of-line in effect_looper.cpp and
 * marked FLASHMEM.  This class is deliberately NOT a template (GCC drops
 * section attributes on vague-linkage members) so FLASHMEM placement
 * survives the toolchain.
 *
 * Concurrency contract:
 *  - Fields in the "written by control" group are only ever written from
 *    Controller()/mainloop()/Start()/Unload() and only read by update();
 *    single 32-bit stores are atomic so they are safe from any context.
 *  - Fields in the "ISR-owned" group (playhead, write index, fade state,
 *    peak) are written back by update() at the end of every block.  Control
 *    code never writes them directly: positional jumps go through a command
 *    latch (pos_req_ / req_pos_q13_) which update() consumes at the next
 *    block boundary.  This keeps every transition block-atomic without ever
 *    performing a transport transition inside update() itself.
 *  - Click prevention: a ~5 ms output-domain crossfade from the last
 *    emitted sample is triggered by update() on loop wrap, transport state
 *    changes, playback-mode jumps and latched playhead jumps.  A one-time
 *    tail/head splice crossfade (also ~5 ms) is applied to the recorded
 *    material when a recording is committed to a loop (see commitSplice(),
 *    called from mainloop()) so the loop wrap itself is seamless material.
 *  - Scrub mode uses playhead slewing (one-pole toward the CV target) as
 *    its click-prevention instead of per-step fades.
 */

#ifndef effect_looper_h_
#define effect_looper_h_

#if !defined(KINETISL)

#include <Arduino.h>
#include <AudioStream.h>

class AudioLooper : public AudioStream {
public:
  // Transport state (transitions happen outside update(), see above)
  enum Transport : uint8_t {
    LOOPER_STOPPED = 0,
    LOOPER_RECORDING,
    LOOPER_PLAYING,
    LOOPER_OVERDUB,
  };

  // Playback / scrub modes (Q13 fixed-point speeds, 8192 == 1.0x)
  enum PlayMode : uint8_t {
    MODE_LOOP = 0, // forward looping at 1x
    MODE_SCRUB,    // CV directly positions the playhead (slew-smoothed)
    MODE_REVERSE,  // -1x
    MODE_HALF,     // +0.5x
    MODE_DOUBLE,   // +2x
    MODE_COUNT,
  };

  // ~8 s of stereo int16 @ 48 kHz in PSRAM
  static const size_t MAX_FRAMES = 384000;
  // No-PSRAM fallback starts at ~2 s and halves until malloc() succeeds
  static const size_t FALLBACK_FRAMES = 96000;
  // Click-prevention crossfade / tail-head splice length (~5 ms @ 48 kHz)
  static const int XFADE_LEN = 240;

  AudioLooper(void) : AudioStream(2, inputQueueArray) {}

  // ---- loop-context lifecycle (Start() / Unload()) -------------------------
  bool begin();
  void end();
  bool IsReady() const { return buf_ != nullptr; }
  bool IsPSRAM() const { return in_psram_; }
  size_t CapacityFrames() const { return cap_frames_; }

  // ---- transport transitions (safe from Controller()/mainloop()) -----------
  // Recording starts at playhead 0.  target_frames = 0 means "record until
  // stopRecordingToPlay()" (manual length).  If target_frames > 0 the
  // recording commits itself at that length (auto-stop).
  void startRecording(uint32_t target_frames);
  // Record stop: fixes loop_len at the current length, queues the tail/head
  // splice (mainloop: commitSplice()) and restarts playback at 0.
  void stopRecordingToPlay();
  // PLAYING <-> OVERDUB (no-op unless a loop exists)
  void setOverdub(bool on);
  // To stopped (update() fades to silence on the state change)
  void stop();
  // Jump playhead to 0 (oneshot: play once, then auto-stop)
  void restartPlayback(bool oneshot);
  // Clean restart with a new loop length (Clock Div change edge case)
  void restartWithLength(uint32_t frames);

  // ---- control-context parameter setters (atomic stores) -------------------
  void setPlayMode(uint8_t mode);
  void setOneShot(bool on) { oneshot_ = on; }
  void setRecReplace(bool replace) { rec_replace_ = replace; }
  void setDubLevel(uint8_t percent);    // 0..100
  void setFeedback(uint8_t percent);    // 0..100
  void setScrubTarget(uint32_t pos_q13); // clamped to the loop

  // ---- UI / control-context getters ---------------------------------------
  uint8_t state() const { return state_; }
  uint32_t loopLenFrames() const { return loop_len_; }
  uint32_t playheadQ13() const { return pos_q13_; } // frames * 8192
  bool recComplete() const { return rec_done_; }
  bool oneShotDone() const { return oneshot_done_; }
  void clearFlags() {
    rec_done_ = false;
    oneshot_done_ = false;
  }

  // mainloop(): apply the pending tail/head splice for a committed recording
  void commitSplice();

  // Wet-output peak meter (~40 ms half-life after decayPeak() ticks)
  uint16_t peak() const { return peak_; }
  void decayPeak() {
    uint16_t p = peak_;
    if (p > 0) peak_ = p - (p >> 5) - 1;
  }

private:
  enum PosRequest : uint8_t {
    POSREQ_NONE = 0,
    POSREQ_RESET,       // playhead -> 0 + fade (retrigger)
    POSREQ_RESTART_REC, // playhead/write/rec counters -> 0 + fade
  };

  audio_block_t* inputQueueArray[2];

  // -- written by control context, read by update() -------------------------
  int16_t* volatile buf_ = nullptr;
  volatile uint32_t cap_frames_ = 0;
  volatile uint32_t loop_len_ = 0;     // frames; >= 2 while a loop exists
  volatile uint32_t loop_len_q13_ = 0; // loop_len_ << 13
  volatile uint32_t scrub_target_q13_ = 0;
  volatile uint32_t target_frames_ = 0; // record auto-stop length (0 = manual)
  volatile int32_t speed_q13_ = 8192;
  volatile int32_t dub_q15_ = 0;          // 0..32768
  volatile int32_t feedback_q15_ = 32768; // 0..32768
  volatile uint8_t state_ = LOOPER_STOPPED;
  volatile uint8_t play_mode_ = MODE_LOOP;
  volatile bool rec_replace_ = false;
  volatile bool oneshot_ = false;
  volatile uint8_t pos_req_ = POSREQ_NONE;
  volatile uint32_t req_pos_q13_ = 0;
  volatile uint32_t splice_len_ = 0; // pending commit length (mainloop)
  bool in_psram_ = false;

  // -- ISR-owned (written back by update(), block-atomic) -------------------
  volatile uint32_t pos_q13_ = 0;   // fractional playhead, Q13
  volatile uint32_t write_ix_ = 0;
  volatile uint32_t rec_frames_ = 0;
  volatile bool rec_done_ = false;
  volatile bool oneshot_done_ = false;
  volatile uint16_t peak_ = 0;

  // -- ISR-private ---------------------------------------------------------
  int32_t xfade_left_ = 0;
  int16_t hold_l_ = 0, hold_r_ = 0;
  uint8_t last_mode_ = MODE_LOOP;
  uint8_t last_state_ = LOOPER_STOPPED;

  void applySpeedForMode();
  void spliceLoop(uint32_t frames);

  virtual void update(void);
};

#endif // !KINETISL

#endif // effect_looper_h_
