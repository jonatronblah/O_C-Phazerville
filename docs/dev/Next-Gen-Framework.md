# PHZ3 — Next-Gen Firmware Framework Design

> A from-scratch framework for the O.R.N.8 / Teensy 4.1 module class, designed against the
> constraints documented in [Codebase-Analysis.md](Codebase-Analysis.md) (cross-references to
> "§7.x" point at that doc's findings list). Authoring guide for the *current* firmware:
> [Writing-Apps-and-Applets.md](Writing-Apps-and-Applets.md).

**Target:** O.R.N.8 / O_C_T41-class shields on Teensy 4.1 (primary), Teensy 4.0 (tier-2), Teensy 3.2 (frozen legacy line).

---

## 0. Platform stance (read this first — it constrains everything)

| Target | Verdict | Reasoning |
|---|---|---|
| **Teensy 4.1 + O.R.N.8** | Tier-1, primary | 600MHz M7 w/ FPU+DSP, 512K TCM + 512K RAM2, up to 16MB QSPI PSRAM, SDIO SD, USB-host, 8 UARTs, 32 DMA chans, 2× I2S/TDM. The 8-in/16-bit CV + DAC8568-on-dedicated-SPI + stereo I2S hardware is exactly the module class this framework targets. |
| **Teensy 4.0** | Tier-2, degraded build | Same core/SPI/I2S; loses the SDIO slot, the USB-host connector, the second PSRAM position, and pins. Builds with `PHZ_TIER=2`: smaller pools, no sample streaming, 64-sample audio frames, reduced module set. Cheap to support *if* the framework never hardcodes T4.1-only peripherals outside HAL. |
| **Teensy 3.2 (legacy o_C shield)** | **Frozen on Phazerville 2.x maintenance** | 120 MHz M4 (enforced in `OC_config.h:4-6`), 64KB RAM, no PSRAM, DAC+OLED sharing one SPI bus (the historic bottleneck in `OC_config.h:19` "SPI transfer ends 2uS before next ISR" and `SH1106_128x64_driver.cpp:180-189` unrecoverable-overrun behavior), ~956B total EEPROM for ALL apps (`static_assert` in the app container). A float32 audio engine + schema store + composition graph cannot be made to fit without designing the whole framework around T3.2's limits — which would defeat "radical changes". |

**Honest recommendation:** do **not** build a lowest-common-denominator framework. Ship PHZ3 on T4.x only; keep `phazerville-2.x` branch alive for T3.2 with critical fixes only, and declare an EOL date. The alternative — a `#ifdef`-riddled dual-target framework — is how the current codebase accumulated two app frameworks, two persistence formats, and two MIDI stacks. One product, one platform family.

**What T4.0 buys us almost for free:** the O.R.N.8 shield's *dedicated* DAC SPI decouples DAC writes from display DMA — the single biggest structural win over the legacy ISR design. The scheduler below is built around that fact.

---

## 1. Goals and non-goals

### Goals
1. **One module model** replacing `OC::AppBase` (`OC_apps.h:70-123`), `HS::HSApplication` (`HSApplication.h:54-101`), and `HS::HemisphereApplet` (`HemisphereApplet.h:92-118`) — full-screen, half/quarter-screen, and audio modules are one interface with rate/region capabilities, not three frameworks.
2. **Audio-rate DSP beyond the 16-bit block library**: float32 processing graph on I2S DMA cadence, 32-sample frames, with audio-rate CV taps and audio-rate CV out; PSRAM/SD sample buffers as first-class pool types.
3. **Explicit multi-rate scheduling** replacing the monolithic 60µs ISR (`Main.cpp:115-142`) + `loop()` split, with per-group cycle budgeting and overrun observability (the `OC_DEBUG_PROFILE_SCOPE` cycle-scope idea, promoted to a product feature).
4. **Schema-versioned state store** replacing FOURCC + exact-length matching (silent resets today), with field-keyed TLV, per-module versions, and tested migration paths.
5. **Host-testable logic**: every module's control/DSP/state code compiles and runs on the host against a simulated HAL; CI runs unit tests (fix the bit-rotted `software/test/` harness whose Makefile points at deleted `../o_c_REV/`).
6. **Polyphony and 8×8 utilization**: voices are a port property; the MIDI voice-allocation logic currently buried in `HSIOFrame.h` (`MIDIFrame::FindNextAvailPolyVoice`) becomes a host-tested router service.
7. **Real preset portability**: named presets = patch graph + module states + input maps, atomic on flash, importable from Phazerville 2.x data.
8. **Dynamic composition**: patches are data (module instances + wires), not `#include` order; container modules (the Hemisphere/Quadrants successors) host children recursively.
9. **Fast iteration**: multiboot 3×1MB slots (`Main.cpp:146-169`) as permanent A/B test + rollback harness; host simulator as the fast inner loop.

### Non-goals (deliberately NOT redesigned)
- **Register-level peripheral drivers.** DAC8568 SPI, MCP33131D+FlexIO, PCM1808/PCM1754 I2S, SH1106/SSD1306 init sequences: port behind `hal/` and keep the working register pokes.
- **The CV numeric convention.** 128 units = 1 semitone, `ONE_OCTAVE = 1536` (`HSUtils.h:16`), 10-octave span, per-channel calibration tables (`OC_calibration.h`). This is load-bearing for ~90 applets and every saved preset. Audio paths use float32; conversion happens only at the HAL boundary.
- **Runtime board detection by A17 ID voltage** (`OC_gpio.cpp:145-248`, ORN8/20Vpp/NLM/CalSynthXL pinouts + polarity). Keep as-is, extend per-channel cal storage.
- **The input-map UX** (cvmap/trigmap attenuversion, div/mult, skip probability; `CVInputMap`/`DigitalInputMap`). The *concept* is right (§7.6 flags index-keyed masks, not the layer itself). Keep it, move it to a router service, key it by stable IDs.
- **The 1-bit weegfx framebuffer** and 128×64 OLED as the display target. No display upgrade in scope.
- **Button/encoder interaction paradigm** (4 buttons + 2 encoders, cursor blink, edit mode).
- **No RTOS, no exceptions, no RTTI, no heap in hot paths.** Bare metal + priority ISRs + static pools.
- **No scripting VM / interpreter in v1.** "Higher-level authoring" = scaffolding tooling + declarative descriptors (§6), not a DSL runtime. Revisit only if module counts exceed what C++ compilation comfortably handles.
- **No binary compatibility of flash images or saved presets** with 2.x. One-way *importer*, never a shared on-disk format.

---

## 2. Layered architecture

```
┌──────────────────────────────────────────────────────────────────────┐
│ L7 host/        phz-cli, SimHal, gtest corpus, UI snapshot renderer  │  (PC, CI)
├──────────────────────────────────────────────────────────────────────┤
│ L6 ui/          DrawCtx (region-local), WidgetSet, Layout, Screensaver│  loop ctx
│ L5 state/       Store (TLV records, A/B atomic), Preset, Migrate      │  idle ctx
│ L4 module/      Module API, Registry, Patch graph, Containers, Router │
│ L3 dsp/         Quantizer, Slew/Atten, Envs, Filters, SampleBuffer,   │  audio+ctrl
│                 VoiceAllocator, ParamSmoother  (hardware-free headers)│
│ L2 io/          SignalBus (32 virtual chans), InputMap, MidiRouter,   │  ctrl tick
│                 EventQueue (SPSC), Calibration application            │
│ L1 kernel/      Scheduler (rate groups), TaskQueue, Pool<T,N>,        │  ISRs + idle
│                 BudgetGauge, Crc, StaticRing<T,N>                     │
│ L0 hal/         BoardDetect (A17 ID), PinMap, drivers: DAC8568,       │  ISR/driver
│                 MCP33131D+FlexIO, I2S2+DMA, SDIO, QSPI PSRAM, UART,   │
│                 USBHost, OLED+SPI DMA                                 │
└──────────────────────────────────────────────────────────────────────┘
```

**Responsibilities and concrete interfaces per layer:**

| Layer | Owns | Key types (namespace `phz::`) | Never does |
|---|---|---|---|
| `hal/` | Peripheral registers, DMA descriptors, board ID | `Board`, `BoardModel`, `PinMap`, `CalTable`, `Dac8568`, `Mcp33131d`, `I2sPort`, `OledBus`, `SdCard`, `Psram`, `UsbHostMidi`, `UartMidi` | Scheduling, units conversion policy, app-visible types |
| `kernel/` | Time, memory, budgets | `Scheduler`, `RateGroup`, `TaskQueue`, `Pool<T,N>`, `StaticRing<T,N>`, `BudgetGauge`, `Crc32` | Module semantics |
| `io/` | The signal fabric: calibration → units, maps, routing, events | `SignalBus`, `ControlFrame`, `InputMap`, `DigitalInputMap`, `MidiRouter`, `EventQueue`, `Event` | Rendering, persistence |
| `dsp/` | Pure math/services shared by modules | `Quantizer`, `Slew`, `ParamSmoother`, `Biquad`, `EnvFollow`, `VoiceAllocator`, `SampleBuffer`, `Interp` | Any `#include` of HAL/Arduino |
| `module/` | The authoring API + composition | `Module`, `ModuleDesc`, `PortDef`, `ParamDef`, `FieldDef`, `ControlCtx`, `AudioCtx`, `Patch`, `Container`, `Registry` | Peripheral access |
| `state/` | Persistence | `Store`, `StateWriter`, `StateReader`, `Preset`, `Migrate`, `LegacyImport` | Deciding *when* to save (module/system policy does) |
| `ui/` | Rendering contract | `DrawCtx`, `Widget`, `Layout`, `UiKind`, `Icon` | Reading inputs (events come from `io/`) |
| `host/` | Dev loop | `SimHal`, `SimClock`, `PngRenderer`, `phz-cli` | Shipping in firmware |

**Dependency rule (enforced by include-graph lint in CI):** arrows point strictly downward. `dsp/` and `module/` headers must compile with zero Arduino/Teensy includes — this is the load-bearing requirement for goal #5.

**Layer interface in one line each:**
- `hal::Board hal::detectBoard();` — once at boot, returns `{model, pinmap, cal, caps{psram_bytes, has_sd, has_usb_host, audio_chans}}`.
- `kernel::Scheduler::start(const Board&)` — owns the PIT + I2S DMA interrupts.
- `io::SignalBus` — publishes one `ControlFrame` per control tick (consumed by modules) and commits `OutFrame` to the DAC (with slew/atten/skip applied — semantics preserved from `HS::IOFrame::Send`).
- `module::Registry` — constexpr table of `ModuleFactory{id_hash, desc, construct}`, `static_assert`-checked unique IDs (keeps the good part of `AppletRegistry.h:62-64`).
- `state::Store` — `Store::instance().slot(ModuleId, instance)` returning `StateWriter/StateReader` over an atomic flash record.
- `ui::DrawCtx` — *region-local* drawing surface (fixes the 64px `gfx*` member-offset trap, §7.1, by construction).
- `host::SimHal` — same `hal` concepts, in-memory; modules run unmodified.

**Proposed file tree** (new code only; `software/src/legacy/` holds 2.x during transition):

```
software/
  firmware/phz/            hal/{board,dac8568,mcp33131d,i2s,oled,sd,psram,usbmidi}.h/.cpp
                           kernel/{scheduler,pool,taskqueue,budget,ring,crc}.*
                           io/{signal_bus,input_map,midi_router,event_queue}.*
                           dsp/{quantizer,slew,env,filters,sample_buffer,voice_alloc}.h
                           module/{module,desc,ctx,registry,patch,container}.*
                           state/{store,preset,migrate,legacy_import}.*
                           ui/{draw_ctx,widgets,layout,screensaver}.*
                           modules/  (one .h/.cpp pair per module)
  host/                    sim/{sim_hal,sim_clock,sim_main}.cpp
                           tests/  (gtest: dsp, state corpus, module property tests)
                           tools/  (phz-cli: inspect presets, lint modules, scaffold)
  platformio.ini           envs: T41, T40_tier2, native(host)
```

---

## 3. Processing model

### 3.1 What replaces the 60µs ISR + loop split

The current design forces one ISR to interleave display DMA flush, DAC SPI, display page start, ADC scan, digital scan, *and* app processing, with a 2µs SPI tail margin (`OC_config.h:19`) and unrecoverable glitches on overrun (`SH1106_128x64_driver.cpp:180-189`). On O.R.N.8 the DAC has its **own SPI** and the ADC is on **FlexIO + DMA** — the hardware no longer requires that interleaving. New model: **four rate groups, each with its own trigger source, all publishing/consuming double-buffered frames with no locks** (ISR writes slot A while loop reads slot B; single-word "publish index" flips with `std::atomic_ref`-style barriers).

| Group | Rate | Period | Trigger | Priority (NVIC) | Cycles budget @600MHz | Work |
|---|---|---|---|---|---|---|
| **Audio** | 48kHz, 32-sample frames | 666.7µs | I2S2 DMA complete ISR | 32 (highest app) | **333k (55%)** incl. ISR overhead | `Module::processAudio` over static graph; float32, DMAMEM/TCM buffers |
| **Control** | 2kHz | 500µs | PIT timer ISR | 64 | **30k (5%)** | `SignalBus` publish → `Module::processControl` → `OutFrame` commit to DAC8568 (SPI DMA) |
| **ADC scan** | ~5.5kHz (empirical, matches current `Scan_DMA`) | ~180µs | FlexIO/DMA completion | 96 | 15k (2.5%) | Raw 16-bit → cal tables → per-ch filter/EMF; publishes `AdcScan`; feeds control group |
| **UI** | 30Hz render, 1kHz input poll | 33ms / 1ms | loop + existing UI PIT | 128 / 112 (GPIO) | 8ms soft | `Module::draw`, encoder/button `EventQueue` drain, screensaver |
| **Idle** | best-effort | — | `loop()` | — | rest | SD prefetch refill, USB/MTP, serial, `TaskQueue`, autosave coalescing |

Notes grounded in hardware:
- **DAC commit at 2kHz** is ample for CV (10-octave span, 16-bit DAC = 0.15mV/LSB; slewing already modeled per-channel). A module may opt into **audio-rate CV out** (`PortDef.rate = PortRate::AudioCv`): its port is then serviced by SPI DMA at audio-frame rate from a ring buffer. 8ch×24bit×48kHz ≈ 9.2Mbit/s — fine on a dedicated SPI clocked at 30MHz+.
- **Audio frame = 32 samples (0.67ms)** vs Teensy Audio lib's 128 (2.67ms): 4× lower latency and 1kHz-rate CV→audio modulation without zipper artifacts; costs 4× more ISR entries (~1.5k/s, trivial) and slightly worse cache behavior. Build-configurable to 64 for tier-2 (T4.0) and heavy patches. Per-frame total budget 400k cycles; hard cap at 333k (55%) leaves headroom for UI/USB jitter.
- **Display flush moves to UI group + DMA completion**, not the hard real-time tick. This is *the* structural fix for the shared-SPI glitch class; on legacy o_C hardware it's impossible, which is another reason T3.2 stays frozen.
- **Events** (MIDI in from UART/USB-host/USB-device, buttons, encoders, digital-input edges) are queued in the highest-priority ISRs into `StaticRing<Event, 128>` (SPSC) and drained in the control tick and loop. MIDI handling is thereby *never* done in an app's own ISR context (fixes §7.5's ad-hoc in-ISR MIDI): modules see events via `ControlCtx::events()`; audio modules see them at frame boundaries.

### 3.2 How modules declare rates and budgets

In `ModuleDesc`: `uint8_t rates` (bitmask of `Rate::Audio|Control|Ui`) and `Cost cost` (`Cheap|Moderate|Heavy`). Rules:
- `processAudio` implemented ⟺ `Rate::Audio` bit set. The **scheduler rejects at patch-build time** (not runtime) if `patch.audio_cost_sum > kAudioBudget` — cost classes carry nominal cycle weights (Cheap=5k, Moderate=25k, Heavy=90k cycles/frame) calibrated by `BudgetGauge` measurements in debug builds. Conservative, not exact — exact enforcement is a runtime gauge.
- `BudgetGauge` (promoted from `OC_debug.h` profiling scopes) measures each group every N ticks via `DWT->CYCCNT`, keeps `max/last/overrun_count`, and the UI surfaces it in a debug screen. In debug builds, a group overrun emits `Serial.printf` and sets a sticky flag shown in the menu bar. **No silent overruns** — the current failure mode is invisible until the OLED corrupts.

### 3.3 Scheduler sketch

```cpp
// kernel/scheduler.h
namespace phz::kernel {

enum class Rate : uint8_t { Audio = 1, Control = 2, Adc = 4, Ui = 8, Idle = 16 };

struct GroupStats { uint32_t last_us, max_us, cycles_last, cycles_max; uint16_t overruns; };

class Scheduler {
public:
  struct Config {
    uint16_t audio_frame = 32;      // samples
    uint16_t control_hz  = 2000;
    uint32_t audio_budget_us = 333; // per frame
    uint32_t control_budget_us = 150;
  };
  static void init(const hal::Board&, const Config&);
  static void start();                       // arms I2S DMA + PIT
  static void stop();

  // ISR entry points (called from vector table thunks in hal/)
  static void onAudioDmaIsr();               // double-buffer swap, runs audio graph
  static void onControlTickIsr();            // publishes ControlFrame, dispatches modules
  static void onAdcScanIsr(const hal::AdcScan&); // folds into io::SignalBus
  static void onUiPollIsr();                 // 1ms buttons/encoders -> EventQueue

  // loop() context
  static void runIdle(TaskQueue&);           // deferred work, SD, USB, autosave

  static const GroupStats& stats(Rate);
  static uint32_t ticks() ;                  // monotonic control ticks (replaces OC::CORE::ticks)
};

} // namespace
```

```cpp
// kernel/taskqueue.h  (keeps Main.cpp "deferred tasks" idea, formalized)
class TaskQueue {
public:
  enum class Priority : uint8_t { Save = 0, Storage = 1, Usb = 2, Serial = 3 };
  bool post(Priority, void (*fn)(void*), void* ctx);   // static-fn only, no std::function
  void drain(uint32_t budget_us);                      // returns to loop when budget spent
};
```

**Static allocation contract:** all audio buffers in `DMAMEM` (RAM2) or TCM (hot coefficients), module instances in a fixed `Pool<ModuleInstance, kMaxModules>` arena (`alignas(32)`, `static_assert(sizeof(T) <= kModuleMaxSize)` at registration), event/state rings as `StaticRing`. Two concrete hazards from the current code this removes: `std::vector` in `HSIOFrame.h` note buffers (`NoteBuffer`, and `shrink_to_fit` inside MIDI handling) and `new AudioOutputI2S2()` in `AudioIO.cpp:53` — heap in MIDI/audio paths is banned in PHZ3.

**Dispatch style in hot paths:** keep the `RuntimeSlot` non-virtual trick (`OC_app_switcher.h:36-58`) for the audio graph — at patch build, each node stores a type-erased thunk `void(*)(Module&, AudioCtx&)`; the audio ISR walks a flat `AudioNode[]` array with no vtable. Virtual dispatch is fine for control/UI rates (2kHz/30Hz) and for authoring ergonomics.

---

## 4. Module API

**Decision: ONE model, with capability opt-ins** (not two/three). Rationale: the differences between "full-screen app", "half-screen applet", and "audio module" are (a) which rates they hook, (b) how much screen estate they get — both orthogonal to their identity. Three frameworks is exactly the current disease. The only concession to two *shapes*: CV/control modules and audio modules use different context types (`ControlCtx` vs `AudioCtx`) because their units, frame sizes, and latency contracts genuinely differ; a module may implement both (hybrid audio+CV modules, e.g. envelope follower → CV out).

### 4.1 Core interfaces

```cpp
// module/desc.h   (C++20, no RTTI/exceptions required, header-safe: pure aggregates)
namespace phz {

enum class UiKind  : uint8_t { Full, Half, Quarter, Minimal };
enum class Cost    : uint8_t { Cheap, Moderate, Heavy };
enum class PortRate: uint8_t { Cv, Gate, Audio, AudioCv };
using Cat = uint8_t;   // bitmask, keeps HemisphereApplet::AppletCategory semantics

struct PortDef  { const char* name; PortRate rate; uint8_t max_voices; bool bidirectional; };
struct ParamDef { const char* name; int16_t min, max, def; uint8_t unit; uint8_t cv_mod_port; };
struct FieldDef { const char* key; uint8_t type; uint16_t count; };  // type: U8,I16,I32,F32,Blob

struct ModuleDesc {
  const char*     name;          // <= 9 chars (OLED constraint, same as applet_name())
  const char*     id;            // stable reverse-dns string, e.g. "phz.slew"
  uint32_t        id_hash;       // FNV-1a of id, persisted in presets (replaces 8-bit applet IDs)
  uint16_t        state_version; // bumped on state layout change
  Cat             categories;
  uint8_t         rates;         // Rate bitmask
  UiKind          ui;
  Cost            cost;
  const PortDef*  ports;   uint8_t port_count;
  const ParamDef* params;  uint8_t param_count;
  const FieldDef* fields;  uint8_t field_count;
  const uint8_t*  icon;          // same 8x8 bitmap convention
};
```

```cpp
// module/module.h
class Module {
public:
  virtual ~Module() = default;
  virtual const ModuleDesc& desc() const = 0;

  // lifecycle
  virtual void init(ModuleServices&) {}        // services: rng, quantizer, storage handle
  virtual void reset() {}
  virtual void suspend() {}                    // e.g. release PSRAM, mute audio
  virtual void resume() {}

  // hot paths — empty defaults = capability absent
  virtual void processAudio(AudioCtx&) {}      // audio ISR
  virtual void processControl(ControlCtx&) {}  // control ISR
  virtual void onEvent(const Event&) {}        // UI/MIDI events, loop or control context

  // state — keyed TLV, versioned (see §5). No more Save/Restore stream + length match.
  virtual void save(StateWriter&) const {}
  virtual void load(StateReader&) {}           // reader = defaults + found fields + version
  virtual void migrate(uint16_t from_version, StateReader&) { /* default: best-effort load */ }

  // ui — region-local coords, always (0,0 = top-left of MY region)
  virtual void draw(DrawCtx&) const {}
  virtual void drawScreensaver(DrawCtx&) const {}
  virtual void onEdit(EditCtx&) {}             // optional custom editor (Scope, WaveformEditor)
};
```

```cpp
// module/ctx.h
struct ControlCtx {
  static constexpr int kMaxVoices = 8;
  int32_t   in (uint8_t port, uint8_t voice = 0) const;  // 128 = 1 semitone (unchanged units!)
  void      out(uint8_t port, int32_t v, uint8_t voice = 0);
  int32_t   param(uint8_t idx, uint8_t voice = 0) const; // post-CV-modulation, smoothed
  bool      gate(uint8_t port) const;
  bool      clocked(uint8_t port) const;
  uint32_t  cycleTicks(uint8_t port) const;              // what ClockCycleTicks() gives today
  EventSpan events() const;                              // since previous tick
  uint32_t  tick() const;
};

struct AudioCtx {
  static constexpr int kFrame = 32;
  std::span<float> in (uint8_t port, uint8_t voice = 0); // kFrame samples, deinterleaved
  std::span<float> out(uint8_t port, uint8_t voice = 0);
  float    sampleRate() const;                           // 48000
  float    cvTap(uint8_t port) const;                    // audio-rate CV sample (upsampled), volts-normalized
  void     notifyUnderrun();                             // stats hook
};
```

### 4.2 Registration (fixes the single-TU + token-rewrite hazards)

One `.h/.cpp` pair per module — **not** all headers `#include`d into one TU (§7.2: ODR hazards, minutes-long single-TU compiles). Registration is an explicit X-macro list (explicit beats magic; predictable linker GC; no static-init-order issues from self-registration):

```cpp
// module/registry.h
struct ModuleFactory {
  const ModuleDesc* desc;
  Module* (*construct)(void* arena);   // placement-new into Pool arena
  size_t instance_size;
};
#define PHZ_REGISTER_MODULE(T) \
  static_assert(sizeof(T) <= phz::kModuleMaxSize, #T " exceeds module arena slot"); \
  extern const phz::ModuleFactory T##_factory; \
  const phz::ModuleFactory T##_factory{ &T::kDesc, \
    +[](void* p){ return new (p) T(); }, sizeof(T) };

// registry/modules.inc        — the ONE list that matters (successor to applets/_config.h)
PHZ_MODULE(SlewOffset)  PHZ_MODULE(Quant8)  PHZ_MODULE(EnvFollow) ...

// registry/registry.cpp
#define PHZ_MODULE(T) extern const phz::ModuleFactory T##_factory;
#include "modules.inc"
constexpr ModuleFactory kFactories[] = {
#define PHZ_MODULE(T) T##_factory,
#include "modules.inc"
};
static_assert(unique_ids(kFactories), "duplicate module id_hash");  // keeps AppletRegistry.h:63 guarantee
```

The `#define applet_name applet_name() const final { return applet_name_(); }` macro hack in `applets/_config.h:15-19` dies: descriptors are plain `static constexpr` aggregates, so name/icon/params are compile-time visible without token rewriting.

### 4.3 UI surface: **declarative primary, `draw()` escape hatch**

Decision: declarative. Justification:
1. **It kills the 64px offset trap** (§7.1): `DrawCtx` is region-local by contract; `g.x()` is always relative to the module's allocated region whether it's full-screen, half-screen (Hemisphere), or quarter (Quadrants). No more "member `gfx*` adds offset, free `gfx*` doesn't".
2. Host-side **PNG snapshot rendering** for UI tests (impossible with imperative draw code that reads live globals).
3. Free, consistent widgets: cursors, value fields, meters, and the input-map editor currently re-implemented per applet (`HSApplication::gfxDisplayInputMapEditor` and its Hemisphere twin become one widget).
4. Screensaver generation for free (`drawScreensaver` default = meters view, matching `HSApplication::BaseScreensaver`).

Escape hatch: `onEdit(EditCtx&)` / raw `draw()` may issue arbitrary primitives — required for Scope, PongGame, WaveformEditor. Declarative-ness is a property of the *descriptor + widgets*, not enforced dogma.

```cpp
// ui/draw_ctx.h  — primitives are region-local
class DrawCtx {
public:
  void text(int x, int y, const char*);
  void value(int x, int y, int32_t raw, uint8_t unit);   // formatted with unit (semitone, ms, %)
  void bar(int x, int y, int w, int32_t v, int32_t vmax);
  void meter(int x, int y, int w, int h, int32_t v, int32_t vmax);
  void icon(int x, int y, const uint8_t* bmp);
  void cursor(const Cursor&);                            // blink/edit state, framework-owned
  void invert(int x, int y, int w, int h);
  int  width() const, height() const;                    // region size (64x64, 64x32, 128x64...)
  EditState edit() const;                                // replaces HSApplication::isEditing + CursorBlink()
};
```

### 4.4 Composition: patches and containers

```cpp
// module/patch.h
struct Wire   { NodeRef src; NodeRef dst; int16_t amount; };  // CV-side "cable"; audio uses same struct
struct NodeRef{ uint16_t instance; uint8_t port; };
struct Patch  { ModuleInstance inst[kMaxModules]; Wire wires[kMaxWires]; uint16_t count; };

// module/container.h — Hemisphere/Quadrants successors are just Modules with children
class Container : public Module {   // region allocator + child event routing
protected:
  uint8_t addChild(Module*, UiKind region);
  Module* child(uint8_t);
};
```

- **CV-side wiring** = the input-map layer, upgraded: `InputMap`/`DigitalInputMap` become router-owned objects (attenuversion, div/mult, skip semantics preserved from `HSIOFrame`/`CVInputMap`), keyed by **stable module id_hash + port**, fixing §7.6's index-keyed masks.
- **Audio-side wiring** = `AudioGraph` rebuilt only on patch change (amortized), then walked as a flat thunk array per frame (§3.3). `PatchCable` from `HemisphereAudioApplet.h:72` maps 1:1 onto `Wire`.
- Recursion depth limited to 2 (container → leaf) to bound UI routing and instance memory; asserted at patch build.
- The 32-channel virtual I/O idea from `HSIOFrame.h:31` (`IO_CHANNEL_COUNT = 32`) is **kept** — `SignalBus` has 32 slots; physical 8 in / 8 out are slots 0-7, soft/routed channels 8-31. Chaining applets ("multiple stages of processing" per the header comment) becomes explicit patch wires.

---

## 5. State / persistence

### 5.1 Record format (replaces FOURCC + exact length)

Current failure mode: app state chunks matched by FOURCC + exact length; any struct change silently resets the app (§7.3). New format: **field-keyed TLV with per-module version**:

```
Record  = [magic u32 'PHZS'][fmt u16=1][id_hash u32][state_ver u16][instance u16]
          [payload_len u32][TLV...][crc32]
TLV     = [key16 = FNV1a(field.key)][type u8][len u16][bytes...]
```

```cpp
// state/store.h
class StateWriter { public:
  void u8 (std::string_view key, uint8_t);  void i16(std::string_view key, int16_t);
  void i32(std::string_view key, int32_t);  void f32(std::string_view key, float);
  void blob(std::string_view key, const void*, uint16_t);
};
class StateReader { public:
  uint8_t u8 (std::string_view key, uint8_t def) const;   // missing field -> def (NOT reset!)
  int16_t i16(std::string_view key, int16_t def) const;   /* ... */
  bool has(std::string_view key) const;
  uint16_t version() const;                                // version the data was written with
};
```

**Semantics:** unknown fields are preserved round-trip (forward compat when a module adds fields), missing fields take defaults (backward compat), type mismatch = field ignored + debug warning. `Module::migrate(from_version, reader)` is invoked when `reader.version() < desc().state_version`; a migration chain in `state/migrate.h` can walk multiple versions. **Module state layout changes no longer reset the module** unless a field genuinely disappears.

### 5.2 Storage roles

| Medium | Role | Notes |
|---|---|---|
| **LittleFS (program flash)** on T4 | Presets, global settings, calibration, patch graph records | Keeps PhzConfig's "key→value in a file" concept (`PhzConfig.h:11-32`) but replaces the `unordered_map<KEY, uint64_t>` (heap, 64-bit values only) with record files. LittleFS wear-levels; writes only on explicit save + 30s dirty-coalescing in `TaskQueue`. |
| **SD (SDIO)** | Preset banks, sample content, backups, logs | Directory layout `/phz/presets/*.phzp`, `/phz/samples/*.wav`. Card absence degrades features, never blocks boot. |
| **QSPI PSRAM (8-16MB)** | *Cache, not persistence*: sample pools, granular buffers, delay lines, undo buffer for editors | Allocated at patch build from `dsp::SampleBuffer` pools; freed on patch change. Explicit tiers: TCM (hot coefficients) / RAM2+DMAMEM (audio bufs) / PSRAM (sample pools). |
| **Emulated EEPROM (T3.2)** | — | Legacy line only. |

### 5.3 Atomicity and wear

- **Two-slot A/B commit per record**: write slot B, read-back CRC verify, then flip a tiny pointer record (itself A/B). Power loss mid-save leaves the previous good record intact. Sequence numbers order slots.
- **Preset = one file**: `[PresetHeader{magic, fmt, seq, patch_hash, module_count, crc}] [per-module records] [input maps] [global refs]`. Saves are atomic at file granularity via the same A/B scheme over two file names.
- **Wear**: LittleFS on 2MB+ program flash with rare writes is a non-issue; the discipline is *when* writes happen (never in ISRs; only from `TaskQueue::Priority::Save`; coalesced when dirty).
- **Unknown module IDs** (e.g. preset from a build with different `modules.inc`): resolve to a **placeholder module** that preserves its TLV blob untouched and shows "???" in the UI — never silent index-0 resolution (fixes §7.6's corruption mode).

### 5.4 Preset model

`Preset` = named (SD/LittleFS) snapshot of: patch graph (module instances + wires), module states, input maps (cvmap/trigmap equivalents), MIDI router maps, and references to global scale/calibration. 50 packed-word Hemisphere preset slots map to 50 named preset slots in the importer (§7). MIDI program-change selects presets (keeps `MIDIFrame::pc_channel` behavior).

---

## 6. Authoring experience

### 6.1 A simple CV module, end to end (~40 lines)

```cpp
// modules/mod_slew.h
#pragma once
#include "module/module.h"
class SlewOffset final : public phz::Module {
public:
  static constexpr phz::ParamDef kParams[] = {
    {"Slew",   0,   255,  64, phz::Unit::Ms,        0xFF},
    {"Offset", -256, 256,  0, phz::Unit::Semitone,  0xFF},
    {"Att",   -128, 128,  64, phz::Unit::PctX2,     0},   // cv_mod_port 0 = mod from In
  };
  static constexpr phz::PortDef kPorts[] = {
    {"In",  phz::PortRate::Cv,  1, false},
    {"Out", phz::PortRate::Cv,  1, false},
  };
  static constexpr phz::FieldDef kFields[] = {{"cur", phz::Field::I32, 1}};
  static constexpr phz::ModuleDesc kDesc {
    .name = "Slew", .id = "phz.slew", .id_hash = phz::hash("phz.slew"),
    .state_version = 1, .categories = phz::Cat::Utility,
    .rates = phz::Rate::Control, .ui = phz::UiKind::Half, .cost = phz::Cost::Cheap,
    .ports = kPorts, .port_count = 2,
    .params = kParams, .param_count = 3,
    .fields = kFields, .field_count = 1, .icon = phz::icons::ZAP,
  };
  const phz::ModuleDesc& desc() const override { return kDesc; }

  void processControl(phz::ControlCtx& c) override {
    int32_t target = c.in(0) * c.param(2) / 64 + (c.param(1) << 7);
    int32_t step = 1 + c.param(0) * 4;
    cur_ += phz::clamp(target - cur_, -step, step);
    c.out(0, cur_);
  }
  void draw(phz::DrawCtx& g) const override {
    g.text(0, 0, "Slew");  g.bar(0, 16, g.width(), cur_, 12288);
    g.value(0, 40, g.param(0), phz::Unit::Ms);
  }
  void save(phz::StateWriter& w) const override { w.i32("cur", cur_); }
  void load(phz::StateReader& r) override { cur_ = r.i32("cur", 0); }
private:
  int32_t cur_ = 0;
};
PHZ_REGISTER_MODULE(SlewOffset);   // + one line in registry/modules.inc
```

This is the bar: **<100 lines for a simple CV processor** including params, ports, state with defaults, and UI. Audio modules add `processAudio(AudioCtx&)` and, if they want samples, a `dsp::SampleBuffer` from module services.

### 6.2 Iteration loop

1. **Host sim is the inner loop** (seconds): `host/tools/phz-cli sim --module phz.slew --stim tests/stim/slew.csv` runs the module against recorded ADC/MIDI stimulus and prints/asserts outputs; `--png` renders the UI to a PNG for eyeballing. No hardware, no flashing.
2. **Unit tests in CI** (replaces bit-rotted `software/test/`): `host/tests/` with gtest via CMake (or the fixed Makefile) running on every push alongside the 5 PlatformIO env builds. `dsp/` and `state/` tests are pure; module property tests assert invariants (output bounded, no NaN after 10k ticks, state round-trips, migration from vN fixture loads).
3. **On hardware**: `pio run -e T41 && pio run -t upload` + the existing **multiboot slot scheme as A/B harness** (`Main.cpp:146-169`, 3×1MB slots): flash candidate to slot1, keep known-good in slot0, boot-select with button holds; a crash boot-loop falls back (see risk §8.4). No hot-reload of code is possible (no MPU/MMU, flash execution) — be honest about that; the *simulation* loop is the fast path, multiboot A/B is the safety path.
4. **Scaffolding**: `phz-cli new --name Foo --rate control --ui half` writes the skeleton + `modules.inc` entry + a test stub. Optional light codegen from a YAML param table is M5 sugar only if boilerplate still hurts; the descriptor format is already plain C++ (YAGNI on a DSL).

### 6.3 Higher-level authoring (the honest scope)

The declarative descriptor *is* the higher-level layer: params, ports, state fields, and UI labels come from one aggregate, from which the framework derives: parameter menus, input-map editing widgets, preset serialization, host test harnesses, and the module-lint report (ID uniqueness, state size, declared cost vs measured). A DSL/codegen layer on top is deferred until there's evidence it's needed (e.g. a community module influx).

---

## 7. Migration path from Phazerville

**Verdict: clean break at the framework level, incremental at the module level.** Ship a compatibility shim so the ~90 applets run on the new core *before* being ported; retire the shim on a schedule. A full binary/preset-compatible evolution is not credible given the two app frameworks, two state formats, and single-TU registry.

### 7.1 Where each old concept lands

| Old concept | Fate | New home |
|---|---|---|
| `OC::AppBase` (13 pure virtuals, `OC_apps.h:70-123`) | retired | `Module` (§4) |
| `HS::HSApplication` (`HSApplication.h:54-101`) | retired | `Module` + `ControlCtx`; `BaseController` Load/Send becomes `SignalBus` publish/commit |
| `HS::HemisphereApplet` (`HemisphereApplet.h:92-118`) | shim'd, then ported | `Module` with `UiKind::Half`; `OnDataRequest/Receive` 64-bit words ↔ state schema (shim packs/unpacks) |
| `HemisphereAudioApplet` / `AudioAppletSubapp` (`InputStream()/OutputStream()/PatchCable`) | map cleanly | `processAudio(AudioCtx&)` + `Wire`; Teensy `AudioStream` blocks are replaced over time by float32 modules (16-bit block adapters provided) |
| `OC::IOFrame` (`cv.pitch_values`) | kept, renamed | `io::ControlFrame` (raw calibrated samples, same 128/semitone units) |
| `HS::IOFrame` 32ch soft IO (slew/atten/skip/clock countdowns) | kept semantics, moved | `io::SignalBus` + `io::PortRouter` (slew/atten/skip become router-owned per-port params, not per-applet code) |
| `CVInputMap` / `DigitalInputMap` (trigmap/cvmap) | kept, upgraded | `io::InputMap`, keyed by stable id_hash + port (fixes index-keyed masks) |
| `MIDIMapping` / `MIDIFrame` (`HSIOFrame.h:74-786`) | kept design, extracted | `io::MidiRouter` + `dsp::VoiceAllocator` (poly_buffer logic becomes host-tested); the existing inline schema migration in `MIDIMapping::Unpack` (`HSIOFrame.h:398-432`) is exactly the right idea — that becomes the standard `migrate()` pattern |
| Scales / quantizer (`OC_scales`, `util_semitone_quantizer`) | kept | `dsp::Quantizer` (hardware-free header, host-tested — this is where `oc_test_quantizer.cpp` gets revived) |
| Calibration + board ID (`OC_calibration.h`, `OC_gpio.cpp:145-248`) | kept | `hal::CalTable` + `hal::detectBoard()`; per-channel cal persists in `state::Store` records |
| `AppContainer` tuple + global EEPROM `static_assert` | retired | `registry::modules.inc` + per-module state budget check (no global coupling — §7.4) |
| Deferred tasks in `loop()` (`Main.cpp:387-581`) | kept, formalized | `kernel::TaskQueue` |
| 1ms UI poll ISR + event queue | kept | `hal::UiPoll` → `io::EventQueue` |
| Multiboot 3-slot flash | kept, promoted | A/B test + rollback harness (§6.2) |
| `DECLARE_APP`/`applet_name` macro rewrite | deleted | plain constexpr `ModuleDesc` |
| Stale `APP_Boilerplate.h.txt` | deleted | `phz-cli new` scaffolding |
| `std::vector` in MIDI note buffers, `new` in `AudioIO.cpp` | deleted | `StaticRing`/`Pool` |

### 7.2 The shim (`module/legacy_applet_adapter.h`)

A `LegacyAppletModule : public Module` wrapper that owns a `HemisphereApplet`-derived instance and translates:
- `processControl` → `BaseController`-equivalent: `frame.Load()` → `ControlCtx` reads, `Controller()`, `frame.Send()` → writes.
- `draw` → `BaseView()`/`DrawFullScreen()` with a `DrawCtx` that applies the region offset *once*, inside the adapter (so legacy `gfx*` calls still work).
- `save/load` → `OnDataRequest()`/`OnDataReceive()` 64-bit words packed into the TLV as a single opaque `legacy` blob field, plus a per-applet adapter hook for gradual field-level adoption.
- `OnEncoderMove/OnButtonPress` → `onEvent`.

The shim **freezes on M2**: after that, ports happen per-applet, shim gains are rejected.

### 7.3 Phasing of the ~90 applets

1. **Tier A (port first, ~10):** the ones every patch uses — ClockDivider, DualQuant/Chordinator, ADSREG/ADEG, AttenuateOffset, Logic/Combin8, EuclidO, ScaleDuet, Scope. These validate the API and become the reference implementations.
2. **Tier B (~30):** sequencers and modulators (CVRec, Sequins-era, EnvSeq, Relabi, Pigeons...). Mostly mechanical: `Start()`→`init()`, `Controller()`→`processControl()`, `View()`→`draw()` + descriptor extraction.
3. **Tier C (the long tail):** run on the shim indefinitely; each release milestone lists "ported or dropped" (§8, risk 5).

**One-way data import** (`state/legacy_import.cpp`, runs once on first PHZ3 boot): reads PhzConfig hashmap files + the 50× packed applet-word preset slots and emits PHZ3 presets; original 2.x files are left untouched so users can roll back firmware.

---

## 8. Risks and trade-offs — top 5 failure modes on real hardware

**1. Audio-rate float DSP overruns the 600MHz budget once 8ch + effects + polyphony stack up.**
The 333k-cycle/frame cap is ~55% of one core; a naive reverb/granular on 8 voices blows it, and audio underruns are audible, not graceful.
*Mitigation:* cost-class admission control at patch build (§3.2) with conservative weights; runtime `BudgetGauge` per node (DWT cycle counts, visible in a debug screen); graceful degradation ladder declared per module (drop float→Q15, reduce voices 8→4, lengthen frame 32→64) instead of glitching; keep a fixed-point fast path in `dsp/` for the hottest nodes. The fork's existing `AudioProcessorUsageMax()` stats (`AudioAppletSubapp.h:99-108`) prove we already know how to measure — make it per-node and enforced.

**2. The unified Module API leaks: three disciplines in one interface becomes a worst-of-all-worlds abstraction.**
Audio wants spans and static graphs; CV wants ticks and int32 semitone units; UI wants declarative widgets. If every module is forced through all of it, authoring gets *worse* than HemisphereApplet and the shim never dies.
*Mitigation:* capability opt-ins with empty defaults (a CV module never sees `AudioCtx`); two context types rather than one god-context; static thunk dispatch in the audio graph so the "unified" layer costs nothing on the hot path; keep the §6.1 <100-line bar as an acceptance test for the API — if a simple slew module needs ceremony, the API is wrong.

**3. State migration corrupts user presets — the fastest way to destroy trust in a module.**
TLV keying + versions is strictly better than length-match, but a bad `migrate()` or a renamed field key ships silent data loss to real users' SD cards.
*Mitigation:* golden migration corpus in CI (fixture records from every released module version must load and round-trip); field keys are frozen once released (lint enforces: renaming = new key + migrate); A/B record slots mean a failed save never clobbers the previous good state; placeholder module preserves unknown blobs byte-for-byte; import tool never mutates 2.x originals.

**4. ISR nesting + bus contention (USB host, SDIO, OLED SPI DMA) wrecks audio timing.**
USB host servicing and SDIO transactions can block bus masters for milliseconds; if they preempt or starve the I2S DMA ISR, we get underruns — the exact class of failure the old "SPI transfer ends 2uS before next ISR" comment was fighting.
*Mitigation:* NVIC priority order fixed and documented (I2S DMA 32 > ADC DMA 64 > control PIT 80 > GPIO/UI 112/128); all SDIO/USB work strictly in idle/loop context via `TaskQueue` with time-slicing (`drain(budget_us)`); audio double-buffers in RAM2 with DMA *ahead* of the ISR (hal must schedule I2S DMA as clock master, underrun watermark counters exposed in stats); TCM placement for graph node state to dodge cache/TCM allocation bugs; bring-up test = audio loopback (PCM1808→PCM1754) with measured jitter histogram before any module work starts.

**5. Migration fatigue: the shim becomes permanent, 90 applets half-ported, and the project carries two frameworks forever.**
This is how the codebase got to two app models in the first place; a rewrite that doesn't finish is worse than no rewrite.
*Mitigation:* hard milestone gates (M2 = parity with shim, M5 = "shim removed *or* explicitly frozen with a published list of dropped applets"); per-release "ported or dropped" ledger; port Tier A (the 10 workhorse applets) *before* building new features so the porting cost is measured early with real data; single maintainer + contributor PRs require new modules in the new API only (no new shim usage after M2).

*Runner-up risk (watch, not top-5):* framework over-abstraction (patch graph, containers, wires) ahead of user demand — YAGNI check at M4: if nobody composes multi-module patches, ship the fixed container layouts and cut the free-form patch editor.

---

## 9. Phased roadmap (each milestone independently valuable)

**M0 — Host harness + CI hygiene** *(1-2 weeks; value: tests that actually run)*
Fix/replace `software/test/`: CMake+gtest on host, `dsp/` quantizer + slew + `state` TLV round-trip tests green in CI alongside the existing 5 PlatformIO env builds. Golden preset corpus infrastructure. *Deliverable:* PR CI that blocks regressions in logic, not just compilation.

**M1 — Kernel + HAL: scheduler, I/O fabric, board detect** *(3-4 weeks; value: the timing foundation, measurable on hardware)*
`hal::detectBoard()` (ported from `OC_gpio.cpp:145-248`), DAC8568/MCP33131D/I2S drivers behind `hal/`, `Scheduler` with Audio/Control/ADC/UI groups, `SignalBus` + InputMap + EventQueue, `BudgetGauge` debug screen. I2S loopback + jitter histogram validation. *Deliverable:* T4.1 boots PHZ3 skeleton that outputs calibrated CV at 2kHz and audio at 48kHz with measured budget headroom, multiboot A/B in place. Old firmware remains the daily driver.

**M2 — Module API + state store + legacy shim (parity)** *(4-6 weeks; value: the crossover point — same instrument, new core)*
`Module`/`ModuleDesc`/`ControlCtx`/`DrawCtx`, registry + `modules.inc`, TLV `state::Store` with A/B commit, `Container` (Hemisphere/Quadrants successors), `LegacyAppletModule` shim running the existing applets, `state/legacy_import` for 2.x presets. Port Tier A (10 workhorse applets) natively. *Deliverable:* PHZ3 beta plays existing patches via shim; host sim runs shim'd modules.

**M3 — Audio-rate DSP framework + polyphony** *(4-6 weeks; value: the "radical" capability lands)*
Float32 `AudioCtx` on 32-sample frames, `AudioGraph` static thunk dispatch, `dsp::SampleBuffer` with PSRAM pools + SD double-buffered prefetch (idle `TaskQueue`), `VoiceAllocator` extracted from `MIDIFrame` and host-tested, audio-rate CV tap/out ports, 3 reference modules (float filter bank, delay/reverb with PSRAM tails, sampler/granular). Degradation ladder + per-node cost gauges. *Deliverable:* 8ch CV + stereo float audio processing demos, measured budgets published in docs.

**M4 — Composition + MIDI/USB event fabric + presets v2** *(3-5 weeks; value: modular-in-a-box programmability)*
Patch graph data model + patch editor UI (with the YAGNI gate on free-form patching), `MidiRouter` unified across UART/USB-host/USB-device (today's `midi_msgtx_disable` mask sprawl in `HSIOFrame.h` collapses to one router), named preset banks on SD, program-change preset recall. *Deliverable:* save/load/recall of composed patches; MIDI controller integration tested against USB-host keyboards.

**M5 — Authoring polish + shipping** *(3-4 weeks; value: community-ready)*
Declarative widget completeness (shared input-map editor widget, screensavers), `phz-cli` scaffold/lint/inspect, UI PNG snapshot tests, docs (module authoring guide + porting guide per §7), Tier B applet ports continued, T4.0 tier-2 build green, shim frozen with published "ported/dropped" ledger. *Deliverable:* **PHZ3 v3.0 release** for T4.1/T4.0; Phazerville 2.x branch declared maintenance-only with T3.2 EOL date.

Total: ~5-7 months of focused work. The honest sequencing note: **M0 and M1 are unglamorous and non-negotiable** — skipping them is how the current codebase ended up with zero test coverage and a 60µs ISR holding five unrelated jobs.

---

### Appendix: kept-vs-fixed quick reference

| Keep (right idea) | Fix (design debt) |
|---|---|
| Runtime A17 board-ID detection | — |
| Input-map layer (cvmap/trigmap) semantics | key masks by stable ID, not registry index |
| Deferred task pattern in `loop()` | formalize as `TaskQueue` with budgets |
| RuntimeSlot non-virtual ISR dispatch | reuse for audio graph thunks |
| PhzConfig key→value store concept | TLV records, A/B atomic, versions |
| Multiboot slots | promote to A/B test/rollback harness |
| Category bitmask + duplicate-ID `static_assert` | keep in new registry |
| 32-channel virtual I/O bus | make wiring explicit (`Wire`) |
| MIDI mapping table + inline `Unpack` migration | extract to `MidiRouter`/`VoiceAllocator`, host-test |
| `OC_debug.h` cycle scopes | promote to per-group/node `BudgetGauge` |
| 128/semitone CV units, cal tables | freeze as the CV convention |
| — | 60µs multi-job ISR → rate-group scheduler |
| — | 3 app frameworks → 1 `Module` API |
| — | FOURCC+length state → keyed TLV + versions |
| — | single-TU header registry → per-module TUs + `modules.inc` |
| — | `#define applet_name` token rewrite → plain descriptors |
| — | gfx 64px offset trap → region-local `DrawCtx` |
| — | global EEPROM `static_assert` → per-module budgets |
| — | heap (`std::vector`, `new`) in hot paths → static pools |
| — | silent index-0 ID fallback → placeholder module |
