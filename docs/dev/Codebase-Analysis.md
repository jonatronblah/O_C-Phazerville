# Phazerville Suite — Codebase Analysis

> Developer-facing analysis of the firmware architecture, with an eye toward (a) writing new
> apps/applets and (b) evolving or replacing the framework. Compiled from a full-tree review;
> file:line references are to the current tree and may drift.
>
> Companion docs: [Writing-Apps-and-Applets.md](Writing-Apps-and-Applets.md) ·
> [Next-Gen-Framework.md](Next-Gen-Framework.md)

---

## 1. What this is

**Phazerville Suite** (`djphazer/O_C-Phazerville`) is the active firmware fork for the
*ornament & crime* eurorack platform. Lineage: stock o_C (mxmxmx / plD / Tim Churches) →
**Hemisphere** (Jason Justian / Chysn, 2018 — the dual-applet concept) → **Benisphere**
(benirose) → Phazerville. It runs on two shields × three MCUs:

| Shield | I/O | Buses | MCU targets |
|---|---|---|---|
| **o_C** (mxmxmx orig.) | 4 ADC / 4 DAC | DAC **and** OLED share one SPI | Teensy 3.2, 4.0, 4.1 |
| **O.R.N.8** ("O_C T4.1", Paul Stoffregen, [O_C_T41](https://github.com/PaulStoffregen/O_C_T41)) | 8 ADC (MCP33131D-16b via 8:1 mux/FlexIO) / 8 DAC (DAC8568-16b) / stereo I2S audio / DIN MIDI / USB-host / SDIO SD / 8–16 MB PSRAM | 3 independent SPI + I2S ("all 3 have fully independent bandwidth") | Teensy 4.1 only |

The dominant user-facing model is the **Hemisphere/Quadrants front-end**: many small
*applets* run 2-at-a-time (Hemisphere) or 4-at-a-time (Quadrants, T4.1/ORN8), each owning
half a screen and 2 CV-out + 2 trig/CV-in channels. Full-screen *Apps* (the stock o_C
lineage: Quantermain, Piqued, Sequins, Calibr8or…) coexist in the same firmware, selected
from a menu. An **audio-applet** subsystem (T4.1 only) chains up to 5 DSP modules through
the Teensy Audio library at 48 kHz.

**Build surface:** PlatformIO envs `T32/T40/T41/T41_audio/T41_MTP` (+ `nlm*`, `stock*`,
`custom`) in `software/platformio.ini`; T41 images are packed as 3× 1 MB **multiboot slots**
(`slot0-3.ld`, `res/assemble-multiboot.py`) selectable at power-up. App selection is
entirely compile-time (`-DENABLE_APP_*` flags + linker GC). CI builds 5 envs; there is a
GitHub-discussion-driven "Custom Build" bot for T3.2.

---

## 2. Execution model

Two interrupt sources plus cooperative `loop()` (real entry point is `software/src/Main.cpp`
— `src.ino` is deliberately empty for Arduino IDE compat):

| Context | Rate / prio | Does |
|---|---|---|
| `CORE_timer_ISR` (`Main.cpp:115-142`, `FASTRUN`) | **60 µs / 16.666 kHz**, prio 80 | `display::Flush()` → `DAC::Update()` → `display::Update()` (DMA page) → `ADC::Scan_DMA()` → `DigitalInputs::Scan()` → `app_switcher.Process(&io_frame)` |
| `UI_timer_ISR` (`Main.cpp:100-106`) | 1 ms, prio 128 | `OC::ui.Poll()` — buttons/encoders → 16-deep event queue. **No dispatch.** |
| GPIO pin-change ISR | async, prio 112 (T3.2) | latch trigger edges into flags |
| `loop()` (`Main.cpp:387-581`) | cooperative | redraw (~1 ms cadence), app `Loop()`, `CORE::FlushTasks()`, `ui.DispatchEvents()`, USB host / MTP, serial |

**The 60 µs wall.** Everything on the core ISR shares one period, and the DAC/display SPI
transaction on legacy hardware ends "2 µS before next ISR" (`OC_config.h:19`). Miss the
window and the shared-SPI display driver corrupts frames **unrecoverably**
(`src/drivers/SH1106_128x64_driver.cpp:180-189`). App `Process()`/`Controller()` therefore
runs in hard-real-time context: no heap, no file/USB I/O, no unbounded loops; floats are
expensive on T3.2 (no FPU). Instrumentation exists: `OC_DEBUG_PROFILE_SCOPE` cycle counters
(`OC_debug.h`), debug trace pins, and on-screen STACK/HEAP/FREE/PSRAM gauges (`OC_debug.cpp`).

**ISR dispatch avoids the vtable.** `AppSwitcher::Process` (`OC_app_switcher.h:60-66`) calls
`current_app_.Process(instance, ioframe)` through a plain function pointer in `RuntimeSlot`
(`OC_app_switcher.h:36-43`); every *other* app entry point is virtual through `AppBase*`.

**Cross-context rules (implicit, load-bearing):**
- Draw only inside `GRAPHICS_BEGIN_FRAME…END_FRAME`, only from `DrawMenu()/DrawScreensaver()`.
- `Process()` (ISR) and `View()/OnEncoderMove()/Loop()` (loop) mutate shared state
  concurrently. There are **no locks**; `CORE::DeferTask()` (`OC_core.cpp:9-25`) queues
  loop-side work from the ISR but its comment admits it drops tasks rather than synchronizing.
- `DigitalInputs::Scan()` must run every ISR tick or edge flags are lost.
- A few shipped apps violate the model by reading MIDI inside `Controller()` (e.g.
  `apps/Calibr8or.h:420`, `apps/TheDarkestTimeline.h:85`) — pre-existing hazard, not a pattern
  to copy.

---

## 3. The three (and a half) application frameworks

This is the most important structural fact: **there is no single app API**. There are three
coexisting layers, in increasing abstraction, plus an audio variant.

```
        ┌────────────────────────────────────────────────────────────┐
        │  Hosts: AppHemisphere / AppQuadrants (apps/Hemisphere.h)    │
        │        AudioAppletSubapp<5> (audio_applets)                │
        ├────────────────────────────────────────────────────────────┤
 L3     │  HS::HemisphereApplet   — half-screen applet (~90 of them) │
        │  HS::HSApplication      — full-screen, Hemisphere-style    │
        │  HemisphereAudioApplet  — L3 + Teensy AudioStream ports    │
        ├────────────────────────────────────────────────────────────┤
 L2     │  OC::AppBase            — full-screen legacy app contract  │
        │  OC_APP_CLASS / OC_APP_INTERFACE_DECLARE macros            │
        ├────────────────────────────────────────────────────────────┤
 L1     │  OC::IOFrame / IO::Read/Write, DAC/ADC/DigitalInputs,      │
        │  HS::HSIOFrame (trigmap/cvmap input mapping), gfx utils    │
        ├────────────────────────────────────────────────────────────┤
 L0     │  Main.cpp timers, OC_gpio runtime board detect, drivers,   │
        │  calibration/autotune, PhzConfig/PageStorage persistence   │
        └────────────────────────────────────────────────────────────┘
```

### 3.1 `OC::AppBase` (L2) — the real full-screen contract

`OC_apps.h:70-123`. 13 pure virtuals (`Init`, `appdata_storage_size`, `SaveAppData`,
`RestoreAppData`, `HandleAppEvent`, `Loop`, `DrawMenu`, `DrawScreensaver`,
`HandleButtonEvent`, `HandleEncoderEvent`, `GetIOConfig`, `DrawDebugInfo`) plus a
macro-declared non-virtual `Process(OC::IOFrame*)`. Registration is a two-part macro:

```cpp
OC_APP_CLASS(AppQuadQuantizer, TWOCCS("QQ"), "Quantermain", "4x Quantizer") {
public:
  OC_APP_INTERFACE_DECLARE(AppQuadQuantizer, 4 * QuantizerChannel::storageSize());
  ...
```

All apps are header-only and `#include`d into one translation unit (`apps/_config.h`, pulled
in by `OC_apps.cpp:73`), then instantiated in the `AppContainer<...>` tuple
(`OC_app_switcher.h:108-158`) whose order is behavior (init order) and whose total storage
is `static_assert`ed against the EEPROM app-data budget (`apps/_config.h:141`). Unlisted
apps are dead-stripped by the linker — registration *is* the feature-selection mechanism.

⚠ `apps/APP_Boilerplate.h.txt` documents a **removed** function-pointer interface. It is
stale. For apps, copy `apps/SETTINGS.h` (modern `HSApplication` style) instead.

### 3.2 `HS::HemisphereApplet` (L3) — the applet contract

`HemisphereApplet.h:92-118`. Pure: `applet_name()` (≤9 chars), `Start()`, `Controller()`
(ISR), `View()`, `OnDataRequest() → uint64_t`, `OnDataReceive(uint64_t)`,
`OnEncoderMove(int)`, `SetHelp()`. Defaults exist for `applet_icon()`, `Reset()`,
`OnButtonPress()`, `Unload()`, `DrawFullScreen()`, `AuxButton()`. Registration is in
`applets/_config.h`: an `#include` inside a `#define applet_name …` macro region (which
rewrites the name/icon methods into `static constexpr` for compile-time lists) plus a
`DeclareApplet<T, ID, Category>` entry in the constexpr `Registry` (`AppletRegistry.h`).
IDs are 1–255, persisted in presets; duplicates fail the build, renumbering silently
corrupts user presets (unknown IDs resolve to index 0, `_config.h:276`).

Each applet owns 2 channels via two macros (`HSUtils.h:34-35`):
`gfx_offset = (hemisphere & 1) * 64` (screen column) and `io_offset = hemisphere * 2`
(DAC/trig/cv pair). T4.1 Quadrants extends to 4 slots × 2 channels over 8 hardware
channels. I/O is fully mediated: applets call `Clock/Gate/In/DetentedIn/Out/ClockOut/
GateOut/Modulate` against `HS::frame` (an `HSIOFrame`), which applies the **input mapping
layer** (`CVInputMap`/`DigitalInputMap`, `CVInputMap.h`) — per-channel source select
(physical, DAC-feedback, MIDI, internal clock), attenuversion, clock divide/multiply,
Euclidean gate filtering — plus output slew/attenuation on the way to the DAC. This mapping
layer is one of the codebase's best ideas: applets are oblivious to physical patching.

**State model:** 64 bits per applet per preset (`Pack/Unpack` bitfields), extended storage
via `SetData/GetData` → `PhzConfig` "PX" key/value namespace (8-bit key | 3-bit slot |
5-bit preset id). Full details in the authoring guide.

### 3.3 `HS::HSApplication` (L3) — full-screen, Hemisphere-style

`HSApplication.h:54-101`: `Start()/Controller()/View() const/Resume()` + `BaseController()`
wrapping `HS::frame.Load → Controller → Send` for absolute (0–127) coordinates. The
recommended path for new full-screen apps: inherit *both* `AppBase` (via the macros) and
`HSApplication`, forwarding 4–5 stubs (`apps/SETTINGS.h` is the reference).

### 3.4 `HemisphereAudioApplet` (L3+) — DSP modules

Adds `InputStream()/OutputStream()` (Teensy `AudioStream*`), `PatchCable()` graph wiring,
`mainloop()` for loop-context I/O (SD), 4×64-bit state words, and shared effect factories
(reverb/compressor). Hosted by `AudioAppletSubapp<5>` (`AudioAppletSubapp.h`) which chains
slot outputs into the next slot's input, ending at `AudioOutputI2S2`. Audio is T4.1-only,
128-sample blocks @ 48 kHz, 16-bit, `AudioMemory(252)`.

---

## 4. Hardware abstraction & signal conventions (L0/L1)

- **Channels:** `DAC_CHANNEL_COUNT`/`ADC_CHANNEL_COUNT` = 4 (or 8 on `ARDUINO_TEENSY41`),
  `OC_config.h:9-16`. Channel *indices* are runtime variables (`OC_gpio.cpp:216-233`) so
  board variants can remap.
- **Runtime board detection** (T4.1): ID voltage on pin A17 at boot (`OC_gpio.cpp:145-248`)
  selects ORN8 default / `DAC_20Vpp` (±10 V) / NLMSerge (inverted, ±5 V) / CalSynthXL
  (swapped channel maps, remapped buttons) / Buchla-style NLM. This is a proven
  hardware-abstraction seam — keep it in any redesign.
- **Units (exact, from source):**
  - ADC raw = 12-bit signed after offset (`OC_ADC.h:86-88`), IIR-smoothed; 10 V ≙ 2^12.
  - Pitch units: **128 = 1 semitone**, `ONE_OCTAVE = (12 << 7) = 1536` (`HSUtils.h:16`)
    ⇒ 1536 units ≙ 1 V at 1 V/oct. Full ADC span = 10 octaves
    (`kDefaultPitchCVScale = SEMITONES<<7`, `OC_ADC.h:42`).
  - Output modes: `OUTPUT_MODE_{PITCH,GATE,UNI,RAW}` (`OC_io_settings.h:36-41`); PITCH goes
    through `DAC::PitchToScaledDAC` (per-octave calibration interpolation, optional
    autotune table), GATE through calibrated octave points (`OC_DAC.h:206-267`).
  - Hemisphere constants: `HEMISPHERE_MAX_CV = octave_max * ONE_OCTAVE`,
    `HEMISPHERE_MAX_INPUT_CV = 6*ONE_OCTAVE` (6 V, or 10 V on NLM), gate threshold 1 V.
- **Triggers:** edge semantics diverge per platform — T3.2 pin-change ISR + flags (FALLING);
  T4.0 GPIO `ICR |= 0x03`; T4.1 `ICR = 0x02` **rising** with polarity inversion when the
  ORN8 FlexIO ADC path is active (`OC_digital_inputs.h:113-127`). Not portable-safe to
  assume one convention.
- **Calibration:** `CalibrationData` = 11 DAC octave points/channel + ADC offsets/scales +
  UI flags, `static_assert`-frozen sizes (116 B T32 / 212 B T41, FOURCC `'CAL1'`),
  `OC_calibration.h:136-211`. Autotuner (FreqMeasure-based) writes a parallel per-channel
  table into GlobalSettings (`OC_autotuner.h`, `OC_DAC.h:131-169`).

---

## 5. Persistence

Two backends chosen by MCU, one logical model:

| | T3.2 | T4.0/T4.1 |
|---|---|---|
| Calibration | PageStorage over emulated EEPROM (2 KB) | same (3–4.2 KB emulated) |
| Global settings | PageStorage | **LittleFS** `GLOBALS.CFG` via `PhzConfig` KV hashmap (`PhzConfig.h`) |
| App blobs | `AppData` PageStorage, ~956 B for **all** apps | same (~3900 B) or per-app LFS files |
| Presets | 8× 32 B structs in EEPROM | 50 slots, `HEM_PRESETS.DAT` (key space `preset<<9`) |

Format: `AppChunkHeader{uint16 id; uint16 length}` chunks in ONE blob (FOURCC `'OCA4'`),
`PageStorage` pages carry `{FOURCC, generation}` with round-robin wear leveling
(`util/util_pagestorage.h`). **Compatibility is length-based**: change a settings struct's
`storageSize()` and the old chunk is silently skipped → app resets (safe, but no migration
path; there is no per-app schema/version field). On save overflow the offending app is
silently skipped with a serial error (`OC_apps.cpp:271-274`). Save triggers are manual
(R-encoder long-press in app menu, SETTINGS save button) or auto on suspend/screensaver
(T3.2 wraps EEPROM writes with `app_isr_enabled=false` — ~2 ms hitch).

Migration hooks that exist today: FOURCC bumps, a v2.0 first-run validity bit
(`OC_apps.cpp:119`), post-load `Validate()` of scales/chords/waveforms, and one explicit
v1→v2 trigmap migration in Hemisphere (`apps/Hemisphere.h:513-525`).

---

## 6. Build system & hard constraints

**Feature selection** = `-DENABLE_APP_X` flags (per env in `platformio.ini:70-93` etc.) ×
`#ifdef` around `#include`+registration in `apps/_config.h` × linker GC (`--gc-sections`,
LTO). `NO_HEMISPHERE` strips the whole applet subsystem. `applets/_config.h` gates extras on
`PEWPEWPEW`/`__IMXRT1062__`/`ARDUINO_TEENSY41`. Arduino IDE path exists via `OC_options.h`.
Toolchain is pinned to a **forked** `framework-arduinoteensy` + forked
`teensy-variable-playback` submodule — reproducibility depends on those forks.

**Budgets (the numbers that matter):**

| Resource | T3.2 | T4.0 | T4.1 / ORN8 |
|---|---|---|---|
| CPU | 120 MHz M4, soft float | 300 MHz M7 | 600 MHz M7, FPU+DSP |
| Flash | 256 KB (**apps don't all fit in one image**) | ~2 MB | 8 MB, but **1 MB/multiboot slot** (linker `ASSERT`) |
| RAM | 64 KB | 512K TCM + 512K RAM2 | + 8–16 MB PSRAM (~44 MB/s burst, high latency) |
| App-data storage | ~956 B (static_assert) | ~3900 B | ~3900 B + LittleFS 512 KB + SD |
| ISR budget | 60 µs shared w/ DAC+display SPI | same | same, but buses are independent |
| Audio | ✗ | ✗ | 128-sample blocks, 48 kHz, 16-bit, ~6–12 ms round-trip |

**Tests:** `software/test/` is a native gtest harness for quantizer math and settings
bit-packing — **bit-rotted** (Makefile points at deleted `../o_c_REV/`) and not run in CI.
All app/applet logic has zero automated coverage. Testability is a first-class problem for
any redesign.

---

## 7. Findings summary — what helps and hurts

**Strengths to preserve**
1. **Input mapping layer** (`CVInputMap`/`DigitalInputMap`) — patching, clock math, and
   attenuation abstracted away from applets. Excellent seam.
2. **Runtime board detection by ID voltage** — one image, many hardware variants.
3. **Deferred task queue + Factory<T> pool with PSRAM fallback** (`OC_core.h`) — pragmatic
   cross-context and memory policy.
4. **ISR hot-path / cold-path split** via `FLASHMEM`/`FASTRUN` discipline and the
   non-virtual `RuntimeSlot` dispatch.
5. **Compile-time registries** (constexpr `Registry`, duplicate-ID static_asserts, linker
   GC) — zero runtime cost, and the safety checks are already automated.
6. **Calibration/autotune pipeline** with per-octave interpolation — hardware-honest.
7. Multiboot slot scheme = free A/B firmware test harness.

**Pain points (the case for change)**
1. **Three parallel app frameworks** with different coordinates, I/O conventions, and
   state models; the member-`gfx*` 64 px offset vs free-`gfx*` absolute split is a standing
   footgun (documented in `applets/Boilerplate.h:123-125`).
2. **Single-TU header-only app model** — ODR hazards, `#define` macro rewrites of
   `applet_name()` (a literal-shape constraint on how you write the method), stale
   boilerplate files, and "registration is in one 300-line file" friction.
3. **State has no schema versioning** — length-match or silent reset; `SetData` blobs flush
   only on whole-config saves; keys embed preset IDs that can be invalid at write time.
4. **Global compile-time storage budget** couples every app (`apps/_config.h:141`) — adding
   state to one app can break all builds.
5. **60 µs shared ISR with no synchronization** to loop-context UI code; DeferTask drops
   tasks by design; MIDI handling is ad hoc and sometimes in the ISR.
6. **Registry-order and 8-bit ID stability** are load-bearing for user data but enforced
   only by convention (filter masks are keyed by *index*, so reordering reshuffles user
   hidden-applet settings).
7. **Zero test coverage**; host-side test harness is dead.
8. **Persistence is siloed per MCU family** (EEPROM PageStorage vs PhzConfig LFS) with
   divergent save semantics and manual-save UX.
9. Docs drift: `API-notes.md` doesn't mention `AppBase`/`HSApplication` at all;
   `hemisphere_config.h` referenced in README no longer exists (it's `applets/_config.h`).

These nine points are the design brief for [Next-Gen-Framework.md](Next-Gen-Framework.md).
