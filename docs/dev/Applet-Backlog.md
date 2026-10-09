# Applet Backlog — reconciled from community research (Oct 2026)

> Sources: (1) desire mining — Lines (llllllll.co) + r/modular; (2) market gap scan —
> ModularGrid function-tag census + Modwiggler wish threads (incl. the 2014→ "modules that
> should exist but don't" megathread, 1907 replies). Criteria: dramatic outcomes, heavy CV
> control, **no common existing solution**. Feasibility = Teensy 4.1 / Phazerville audio
> applet constraints (48 kHz, 128-sample blocks, 60 µs control tick, PSRAM buffers).

## Batch 1 — build now (gap confirmed by both sources × high feasibility)

| # | Applet | What | Sources | Feasibility |
|---|---|---|---|---|
| 1 | **SpectraClust** | Steerable resonant filter cluster: movable band centers, per-band gain/Q, morphable scenes, animated sweeps. Optionally spectral panning (per-band stereo placement) | r/modular 1jvwdqf "filter cluster that could be animated" → *"Not really"*, *"I don't know of any module actually doing it"*, *"only sensible way is integrate VCV"*; MW megathread "spectral panner… wicked! Someone make it, please" | HIGH — parallel biquad bank + scene morph |
| 2 | **Tapeworm** | Tape/vinyl transport character: wow, flutter, age, varispeed, dropout — every parameter CV'd | Lines 11201 *"cv control of the tape age"*, 30429 *"Magneto… CV control of wow and flutter"*, 6875, 11420; MW: only 1 wow/flutter module exists, vinyl sim **ABSENT** | HIGH — fractional delay + sat + filtering |
| 3 | **FreqShift** | Bode frequency shifter inside a feedback ring (shift + feedback under CV) | r/modular mvpbee *"Echobode… only a VST"*, *"bode frequency shifter that doesn't cost 2,000$"*; MG freq-shifter = ~10 units, several discontinued/$$$ | HIGH — quadrature allpass + ring mod |
| 4 | **CascDyn** | Multiband / OTT-style dynamics: upward + downward compression, cascading sidechain (band/chan ducks the next), one macro | r/modular 1d2gr7r *"multiband compressor with upward and downward compression"*, *"cascading compressor…"*; MW #6; MG multiband THIN | HIGH — extends in-tree `effect_dynamics` math |
| 5 | **FdbkOsc** | Infinite-feedback delay played as VCO: delay-time = V/Oct, freeze, feedback CV | r/modular 1d2gr7r *"infinitely feed back, and then play the delay time as if it were a vco"* (workaround via Disting) | HIGH — extends existing delay DSP |
| 6 | **Harmo** | CV'd harmonizer / pitch shifter (interval or free, per-voice CV) | Lines 21704 *"That harmonizer is awesome, would love to see that in the suite!"*; r/modular 1d2gr7r HM2-with-CV; MG pitch-shifter MODERATE but no deep-CV | MEDIUM — variable-rate resampler (teensy-variable-playback) or delay-line pitch |
| 7 | **CombChorus** | CV comb filter + deep-CV stereo chorus/ensemble (complex LFO) | MW megathread *"precious few [comb filters]"*, *"stereo spacial chorus with lots of parameters… in cv"* | HIGH — short-delay DSP |
| 8 | **CVTime** (CV applet) | CV/Gate delay with delay-time CV + gate-length processor (dynamically lengthen/shorten gates via CV) | MW t=223371 *"nobody would ever build something that suits"*; t=297276 (2026) *"dynamically shorten and lengthen the gates… via cv?"* | HIGH — check existing GateDelay applet coverage first |

## Batch 2 — attempt next (harder DSP; honest drop OK per user)

| # | Applet | What | Sources | Feasibility |
|---|---|---|---|---|
| 9 | **SpecFreeze** | Live spectral freeze + stretch-lite ("Paulstretch w/ CV": stretch factor, freeze under CV, always-updating buffer) | r/modular 1d2gr7r *"FUCK I would snap buy this"*; 1qsxr3u *"true time stretch live?… not aware of a way"*; MG: 0 hits "spectral freeze", only Panharmonium ~$550/scarce | MEDIUM — CMSIS-DSP FFT (512–1024 pt) + PSRAM; budget-critical |
| 10 | **CrossSynth** | Two-input spectral morph / cross-synthesis (A's magnitude × B's phase; light vocoder/formant morph) | MW #2 vocoder gap, THIN+expensive, **Behringer VC16 window closing**; Lines 31738/33495 (SoundHack mutation, MORPH) | MEDIUM — stereo in = 2 sources; FFT budget shared with #9 |
| 11 | **AnalysisCV** (CV applet) | Spectral features → CV: centroid, flux, brightness, band energy | Lines 20902 *"my biggest wish… single CV output of the tracked centroid"*; 21453 | MEDIUM — control/block-rate FFT → CV out |
| 12 | **ChaosAudio** *(optional)* | Audio-rate chaotic/feedback processor (PLL-ish tracking, Lorenz cross-mod) | MW #4 *"multi-pendulum… loosely sync… variable degree of looseness"*; chaos exists only as CV sources today | MEDIUM — pick simple chaotic ODE + feedback |

## Vetoed — market saturated (do not build)

Plain distortion/drive/folder (859+681 modules, we ship Grit), plain filters (>1600), plain
delay (553; we ship one), plain reverb (298; FX AID ≈ $170), ring mod (304), panning (377;
we ship Crosspan), VCA/mixer, bitcrush (we ship Dr LoFi), basic granular (Clouds clones
<$200; we ship Misty), basic looper/sampler (we ship ClockLoop/CueLoop), phaser/flanger
*basic*, resonators (Rings clones), tilt EQ, gate comp/limiter (we ship Dynamics).

## Deferred ideas worth remembering
- CV-morphing quantizer (Locrian↔Lydian morph under CV) — r/modular 1d2gr7r; CV-side, moderate demand
- HRTF/3D spatializer w/ CV az/el/distance — dream-thread item, feasible but niche
- FFT-domain waveshaping ("spectrum as a wave") — explicitly requested on MW, dramatic, hard
- Paulstretch proper (full phase-vocoder randomization) if SpecFreeze's stretch-lite lands well
