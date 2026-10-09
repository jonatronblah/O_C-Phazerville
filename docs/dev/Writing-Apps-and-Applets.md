# Writing New Apps & Applets — Practical Guide

> How to add functionality to Phazerville today. Based on the actual contracts in
> `software/src/`; see [Codebase-Analysis.md](Codebase-Analysis.md) for architecture context.
>
> **Ignore `apps/APP_Boilerplate.h.txt`** — it documents a removed interface. The live
> templates are `applets/Boilerplate.h` (applets) and `apps/SETTINGS.h` (full apps).

---

## 0. Choose your lane

| You want… | Write a… | Canvas | I/O | State | Effort |
|---|---|---|---|---|---|
| A small CV/gate processor or generator | **Applet** | 64×64 (half screen) | 2 out + 2 in (mapped) | 64 bits + `SetData` | ~150 lines |
| A DSP/audio effect or source (T4.1) | **Audio applet** | 64×64 | AudioStream ports + 2 CV | 4×64 bits | ~200 lines |
| A full-screen instrument/sequencer | **HSApplication app** | 128×64 | all channels | own file / EEPROM chunk | ~500+ lines |

Everything runs `Controller()` in a **60 µs ISR budget** (16.666 kHz) regardless of lane.
Draw code runs in `loop()` and must not do processing.

---

## 1. Applet walkthrough (the common case)

### 1.1 Create `software/src/applets/YourApp.h`

Copy `applets/Boilerplate.h` and rename. The skeleton:

```cpp
class YourApp : public HemisphereApplet {
public:
    // MUST be literally this shape — applets/_config.h #defines rewrite this method
    const char* applet_name() { return "YourApp"; }   // max 9 chars!
    const uint8_t* applet_icon() { return PhzIcons::wave; }   // 8x8, icons.h

    void Start() {              // once, when selected (not on every switch — see gotcha G7)
        for (int ch = 0; ch < 2; ++ch) level[ch] = 128;
    }

    void Controller() {         // ISR @ 16.6kHz — keep it tiny, fixed-cost
        for (int ch = 0; ch < 2; ++ch) {
            if (Clock(ch)) phase[ch] = 0;
            phase[ch] += Inc(ch);
            Out(ch, Proportion(phase[ch], HEMISPHERE_MAX_CV));
        }
    }

    void View() {               // 64x64 local coords, x in [0,63], start at y≈14
        for (int ch = 0; ch < 2; ++ch)
            gfxBitmap(0, 16 + ch * 20, 8, wave_icon);
    }

    void OnButtonPress() { CursorToggle(); }
    void OnEncoderMove(int direction) {
        if (!EditMode()) MoveCursor(cursor, direction);
        else level[cursor] = constrain(level[cursor] + direction, 0, 255);
    }

    uint64_t OnDataRequest() {  // pack state into 64 bits for presets
        uint64_t data = 0;
        Pack(data, PackLocation{0, 8}, level[0]);
        Pack(data, PackLocation{8, 8}, level[1]);
        Pack(data, PackLocation{16, 8}, wave);
        return data;
    }
    void OnDataReceive(uint64_t data) {
        level[0] = Unpack(data, PackLocation{0, 8});
        level[1] = Unpack(data, PackLocation{8, 8});
        wave     = Unpack(data, PackLocation{16, 8});
    }

protected:
    void SetHelp() {            // 8 labels for the help screen
        help[HELP_DIGITAL1] = "Clock";
        help[HELP_CV1] = "Freq CV";
        help[HELP_OUT1] = "Out";
        // ...
    }

private:
    int cursor = 0;
    int level[2], wave = 0;
    uint32_t phase[2] = {0, 0};
};
```

Put non-hot methods out-of-line and mark them `FLASHMEM` (RAM1 is scarce; see
`Boilerplate.h:82-88`). Put large lookup tables in `PROGMEM`/flash-resident arrays.

### 1.2 Register it — `software/src/applets/_config.h`

Two edits, both required:

```cpp
// (1) inside the macro region — BEFORE the #undef applet_name / #undef applet_icon lines
#include "YourApp.h"

// (2) in the Registry list
constexpr Registry reg = Registry<HemisphereApplet, HS::APPLET_SLOTS
    , DeclareApplet<ADSREG, 8, CAT_MODULATOR>
    ...
    , DeclareApplet<YourApp, 151, CAT_UTILITY>   // ← unused ID! see below
>{};
```

**Applet IDs:**
- Range **1–255** (0 = "empty preset"). Highest currently used: 150 (`hMIDIIn`).
- IDs are what presets store. **Never renumber or recycle.** Unknown IDs silently load as
  applet index 0 — user presets quietly break.
- Duplicates fail the build (`static_assert(NoDuplicateIDs<...>)`).
- Categories (`CAT_MODULATOR`, `CAT_UTILITY`, …) are declared but currently unused at
  runtime — safe metadata for filtering UI later.

### 1.3 The name/icon macro trap

Between the `#define applet_name` and `#undef applet_name` in `applets/_config.h`, your

```cpp
const char* applet_name() { return "YourApp"; }
```

is textually rewritten into a `static constexpr` + `final` override so the registry can list
names at compile time. Consequences:
- It must be **exactly** that one-line literal form (no computed strings, no extra `const`,
  no line breaks). A mismatch breaks the rewrite in confusing ways.
- Include your header **only** via `applets/_config.h` (Boilerplate even comments out its
  own `#include "HemisphereApplet.h"` for this reason).
- Name ≤ **9 characters** (`HemisphereApplet.h:103` — several applet comments say 10;
  9 is the real contract).

### 1.4 I/O quick reference (applet, 2 channels)

| Call | Meaning |
|---|---|
| `In(ch)` / `InF(ch, max)` | mapped CV, 0..`HEMISPHERE_MAX_INPUT_CV` (6 V; 10 V NLM) |
| `DetentedIn(ch)` | zero inside ±~1/4 tone |
| `SemitoneIn(ch)` | hysteresis-quantized small-range reading |
| `Clock(ch)` | rising edge of the **mapped** clock (div/mult/Euclid applied) |
| `Gate(ch)` | level of mapped trig input |
| `Out(ch, value)` | DAC value: **128 = 1 semitone, `ONE_OCTAVE` = 1536 = 1 V** |
| `ClockOut(ch, ticks)` / `GateOut(ch, on)` | pulse / level output |
| `Modulate(param, ch, min, max)` | apply CV as bipolar parameter offset |
| `StartADCLag/EndOfADCLag(ch)` | compensate ADC latency after a clock edge |

Inputs are per-channel remappable at runtime (`trigmap[]`/`cvmap[]` — source: physical jack,
another output (self-patching), MIDI, internal clock; with attenuversion, clock math). You
don't deal with physical jacks at all.

### 1.5 Graphics quick reference

- Local 64×64 box; **member** `gfx*` calls add the half-screen offset automatically.
- Two families: member `gfxPrint/gfxLine/gfxFrame/gfxBitmap/gfxCursor/...` (offset-aware) vs
  **free** `HS::gfx*` functions (absolute 128×64 — used by full-screen apps and
  `DrawFullScreen()`). Mixing them up is the #1 drawing bug: on the right-hand applet,
  member calls land at x+64, free calls don't.
- Header is drawn for you (`gfxHeader` at y=2); start content at **y≈14**. Font is 6 px/char.
- Icons: 8×8, column-major MSB-bottom (`icons.h`, `PhzIcons::` namespace).

### 1.6 State beyond 64 bits

`OnDataRequest/OnDataReceive` cover the preset word (≈8×8-bit params). For sequences and
larger data use `SetData(key, value)` / `GetData(key, out)` (256 × 64-bit keys per
slot/preset, backed by the PhzConfig `"PX"` namespace on T4.x). Two caveats: values are
flushed to flash only when a full config save runs, and the key embeds the current preset ID
— don't write before a preset context exists.

### 1.7 Audio applet delta (T4.1)

Subclass `HemisphereAudioApplet` instead (or in addition):
- Implement `InputStream()/OutputStream()` returning your graph endpoints; wire with
  `PatchCable(src, ch, dst, ch)` inside `Start()` (≤32 cables). Release shared resources
  (reverb/comp factories) in `Unload()` + call `AllowRestart()`.
- Register in `audio_applets/_config.h` via `DeclareFancyApplet<T>` — **the ID is a hash of
  the name**; renaming orphans saved data unless you override `applet_id()`.
- SD/file I/O goes in `mainloop()` (loop context), never `Controller()`.
- State is 4×64-bit words (`CONFIG_SIZE`). See `audio_applets/PassthruApplet.h` (28 lines,
  minimal) and `VCAApplet.h` (real example with CV mapping UI).

---

## 2. Full-screen app walkthrough

Model on `apps/SETTINGS.h`; declare with the two macros and add `HSApplication`:

```cpp
OC_APP_CLASS(AppYourApp, TWOCCS("YY"), "YourApp", "Boring Name"),
  public HSApplication {
public:
  OC_APP_INTERFACE_DECLARE(AppYourApp, 0);   // EEPROM bytes; 0 is fine on T4.x
  void Start();  void Resume();  void Controller();  void View() const;
  // then define the AppBase finals below the class:
  // Init, SaveAppData, RestoreAppData, HandleAppEvent, Loop, DrawMenu,
  // DrawScreensaver, HandleButtonEvent, HandleEncoderEvent, Process,
  // GetIOConfig, DrawDebugInfo
};
```

Minimal forwarding bodies:

```cpp
void AppYourApp::Init() { BaseStart(); }
void AppYourApp::Process(OC::IOFrame *ioframe) { BaseController(ioframe); }
void AppYourApp::DrawMenu() const { View(); }
void AppYourApp::HandleAppEvent(AppEvent ev) {
  if (ev == APP_EVENT_RESUME) Resume();
}
void AppYourApp::Loop() { /* MIDI, SD — loop context */ }
```

Then register in `apps/_config.h`: `#include` at top **and** add the type to the
`AppContainer<...>` template list, guarded by `#ifdef ENABLE_APP_YOURAPP` (also add that
flag per-env in `platformio.ini` if you want it selectable). Notes:

- **Container order is behavior** — `Init()` runs in list order and later apps can clobber
  globals (scales, patterns). Read the comment at `apps/_config.h:51-56`.
- The `static_assert` on total app storage (`apps/_config.h:141`) applies to *everyone's*
  build: keep `kAppDataStorageSize` honest (0 unless you use the EEPROM chunk path). On
  T4.x prefer your own `PhzConfig` file (the pattern used by Hemisphere, Scenery,
  Calibr8or, CaptainMIDI).
- `HSApplication::View()` is **`const`**; `HemisphereApplet::View()` is not. Copy-paste
  between the layers trips const-correctness.
- Implement `GetIOConfig(IOConfig&)` so the Z+A I/O-settings menu labels your channels;
  `DrawScreensaver()` usually just calls `BaseScreensaver(true)` or `ZapScreensaver()`.
- Timing/coord conventions: absolute 0–127 coords (no `gfx_offset`), `Out(ch, value,
  octave)`, 128/semitone units as above.

---

## 3. Build & flash

```bash
cd software
pio run -e T41_audio -t upload    # ORN8 + USB audio
pio run -e T41 -t upload          # ORN8 plain
pio run -e T40 -t upload          # Teensy 4.0 on classic o_C shield
pio run -e T32 -t upload          # Teensy 3.2 (deprecated; keep applets small)
```

App/applet inclusion is per-environment `ENABLE_APP_*` flags in `platformio.ini` +
registration + linker GC. To make your app optional everywhere, wrap both the `#include`
and the registration in `#ifdef ENABLE_APP_YOURAPP` and add the flag to the envs you care
about. Arduino IDE works too (`src/src.ino`, flags in `OC_options.h`) but PlatformIO is the
primary path. CI (`.github/workflows/firmware.yml`) builds T40, nlm_cardoc_T40, T41,
T41_audio, T41_MTP — your change must at least link in those.

---

## 4. Testing checklist (there is no automated coverage)

The `software/test/` gtest harness is bit-rotted and not run in CI — assume manual testing:

1. **Compile** for T41 + T32 if you touched shared headers (the T32 build catches storage
   budget overruns via the `static_assert`).
2. **Select** your applet/app from the UI; check the name fits (9 chars) and the icon shows
   on both left and right sides.
3. **Right-side / Quadrants rendering**: verify on side 2 (and slot 3/4 on T4.1) — this
   catches gfx offset mistakes.
4. **Presets**: save → load → confirm state round-trips exactly (bit-packing bugs hide here);
   then switch away and back (in-session state retention).
5. **ISR safety**: watch the debug stats (L press in app menu shows cycles); confirm ISR
   stays under 60 µs with your worst case; check display doesn't glitch.
6. **Input mapping**: test `Clock()` semantics with divided/multiplied clocks and with the
   internal clock source; test gate vs trigger behavior.
7. **Help screen** layout (`SetHelp`) and edit-mode header (`SetLabel`).
8. If you changed any settings struct: old firmware → new firmware upgrade behavior
   (expect silent reset of your app's state; warn users in release notes).

---

## 5. Gotchas (condensed, high-value)

- **G1 — 60 µs is a hard budget** shared with DAC/display/ADC work. No heap, no SD/flash,
  no `Serial.print`, no unbounded loops in `Controller()`. Profile with
  `OC_DEBUG_PROFILE_SCOPE(DEBUG::ISR_cycles)`.
- **G2 — ISR and loop race freely.** `Controller()` can interrupt `View()`/`OnEncoderMove()`.
  Never publish half-updated shared values; no locks exist.
- **G3 — Storage is length-matched.** Changing a settings struct silently resets the app's
  stored state on next load (chunk length mismatch → skip). App IDs (`TWOCCS`, applet
  numeric IDs) must stay stable forever.
- **G4 — Global EEPROM budget.** Every byte of `appdata_storage_size` counts against a
  single `static_assert` on T3.2 (~956 B total). Overflow at save time silently skips your
  app's data.
- **G5 — `Start()` isn't re-run on every selection** (`applet_started` latch). Call
  `AllowRestart()` from `Unload()` if you need re-init (audio applets must, to rebuild
  patch cables).
- **G6 — gfx offset**: member `gfx*` += 64 on the right side; free `gfx*` never offset. In
  `DrawFullScreen()` on the right side, member calls draw off-screen.
- **G7 — `Clock(ch)` is the mapped clock** (post div/mult/Euclid/internal), not the raw
  jack. `Gate(ch)` for levels. Use `StartADCLag/EndOfADCLag` around ADC reads after edges.
- **G8 — OnButtonPress fires on release**, and the same button drives the host's select
  mode when you're not in edit mode. `AuxButton()` only fires while `EditMode()` is true.
- **G9 — `help[]`/`enc_edit[]` are static and shared** across applets; `SetHelp()` re-runs
  on every help draw — assignments only, no computation.
- **G10 — Filter masks cap applets at 128** and are keyed by registry *index*: reordering
  the Registry reshuffles users' hidden-applet settings. Append, don't reorder.
- **G11 — FLASHMEM discipline**: mark cold methods `FLASHMEM`, hot ISR code can use
  `FASTRUN`; big buffers go to `DMAMEM`/PSRAM via the `Factory<T>` pool, not the stack.
- **G12 — Trigger polarity differs per board** (T4.1/ORN8 inverts when the FlexIO ADC is
  active). Test on target hardware; don't hardcode edge assumptions.

---

## 6. Where things live (index)

| What | Where |
|---|---|
| Applet contract + I/O + gfx members | `software/src/HemisphereApplet.h/.cpp` |
| Constants, Pack/Unpack, free gfx | `software/src/HSUtils.h/.cpp` |
| Applet registry & ID rules | `software/src/AppletRegistry.h`, `applets/_config.h` |
| Full-app contract / macros | `software/src/OC_apps.h`, `OC_app_base.cpp` |
| App registration & storage assert | `software/src/apps/_config.h` |
| Hemisphere host (gestures, presets) | `software/src/apps/Hemisphere.h` |
| Input mapping (trigmap/cvmap) | `software/src/CVInputMap.h`, `HSIOFrame.h/.cpp` |
| Audio applet base / host | `HemisphereAudioApplet.h`, `AudioAppletSubapp.h` |
| Audio I/O graph | `software/src/AudioIO.cpp` |
| ISR/loop entry, timers | `software/src/Main.cpp`, `OC_config.h` |
| Board detection / pins | `software/src/OC_gpio.h/.cpp` |
| Calibration | `OC_calibration.h/.cpp`, `OC_autotuner.h` |
| Persistence | `OC_storage.h`, `util/util_pagestorage.h`, `PhzConfig.h` |
| Build envs / app flags | `software/platformio.ini`, `OC_options.h` |
| Live templates | `applets/Boilerplate.h`, `apps/SETTINGS.h`, `audio_applets/PassthruApplet.h` |
