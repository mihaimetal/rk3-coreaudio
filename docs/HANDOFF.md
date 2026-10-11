> **Historical (updated 2026-10).** This is the original bring-up log. Since
> then: 1 ms transfers are the default, the fill "setpoint"/"floor" constants
> changed meaning, the `ClearPipeStallBothEnds` before `AUDIO_PARAMS` was removed
> (it caused the retries described here), and the capture grace period was fixed.
> See the README's *Latency tuning* and the comments in `rk3_engine.h`.

# RK3 CoreAudio — Handoff

Porting the Native Instruments **RigKontrol 3** (VID `0x17cc` / PID `0x1940`,
bcdDevice `0x000b`) USB audio interface to **macOS Apple Silicon**, userspace.
Source-of-truth protocol = the Linux `snd-usb-caiaq` kernel driver in
`/Users/mihai/DEV/caiaq` (read-only reference). New code lives in
`/Users/mihai/DEV/rk3-coreaudio`.

Approach (user-confirmed): **userspace CoreAudio AudioServerPlugin (HAL plugin) +
IOKit IOUSBLib**, NOT a DriverKit dext. Goal: full-duplex audio in+out.
Plan file: `/Users/mihai/.claude/plans/this-folder-contains-the-jolly-beacon.md`

## CURRENT TASK — ALWAYS-ON "CON" MIDI mode (2026-06-22)

**Goal (user, 2026-06-22):** ship MIDI **on by default** — no UI, no checkbox, no
`/tmp/rk3_midi_on` sentinel — and works regardless of audio. This mirrors NI's
Controller-Editor "Start in MIDI mode" (`CON` on the display). Reverse-engineering
the NI macOS driver + Controller Editor (notes in `/Users/mihai/DEV/caiaq/mac_drivers/
FINDINGS.md`) confirmed there is **no device-side mode command to replicate**: NI's
`NIHardwareAgent` daemon just owns the device, reads controls over the same caiaq EP1
protocol, and publishes the CoreMIDI endpoints itself — exactly our engine+helper split.
"CON" is only a cosmetic display the agent writes. So always-on = our engine arming the
relay at open + writing "CON". Decisions (user): de-risk the read storm first;
unconditional always-on (no override file); write "CON".

**IMPLEMENTED + HW-VALIDATED (2026-06-22):** confirmed on hardware — MIDI works
both with NO audio and alongside audio in Reaper; no storm/reboot across the
session. Shipped.
- `rk3_engine.c` — the MIDI relay now **arms unconditionally in the engine thread setup**
  (right after `running` is set), so MIDI is live the moment USB is claimed at plugin
  load. The `/tmp/rk3_midi_on` sentinel and `midi_sentinel_tick` are **removed**.
- **Storm de-risk = THROTTLED re-arm (replaces the disable-latch).** `midi_arm_read`
  stamps `midi_last_arm`; `midi_read_done` re-arms inline only if `>= RK3_MIDI_REARM_MIN_MS`
  (1 ms) has elapsed, else defers via a one-shot `midi_rearm_timer` on the engine run
  loop. This caps the EP 0x81 re-arm rate to ~1 kHz no matter how fast completions fire
  (0-byte success / stall / immediate error), so a runaway physically can't peg the
  CFRunLoop / storm coreaudiod. Real input (~100/s) clears the 1 ms floor every time →
  re-arms inline, zero added latency. The old 256-cb/50 ms guard is kept only as a
  rate-limited WARNING log (`RK3_MIDI_CB_WINDOW`/`WARN_MS`); it no longer disables.
- **"CON" on the 4-digit display** via `midi_show_con` (one async `WRITE_IO` at relay
  start): digit1=`C`(0x39) digit2=`O`(0x3F) digit3=`n`(0x54). Segment map confirmed
  (bit6=g, from the centered-dash test in hardware-facts memory).
- Helper (`rk3midihelper` + LaunchAgent) unchanged — still owns the "RigKontrol 3"
  CoreMIDI source over loopback UDP :17890 (coreaudiod sandbox blocks `MIDIClientCreate`).

**VALIDATED (2026-06-22):** installed, `CON` shows, buttons/pedal hit the "RigKontrol 3"
CoreMIDI source with no audio AND alongside audio in Reaper; no storm/reboot.

---

## PRIOR TASK — MIDI without audio: SINGLE-OWNER engine (2026-06-21)

**Goal (user, 2026-06-21):** make the foot-controls MIDI work **even when no audio is
streaming** — not only while the audio driver streams. Buttons 1–9 → CC20–28 127/0 ch1;
pedal → CC11 ch1.

**Hard constraint (unchanged):** all 4 endpoints share ONE USB interface; macOS lets only
one process `USBInterfaceOpen` it. Earlier Phase-2 design folded the relay into the engine,
but the engine only owned USB during `StartIO` → MIDI only flowed while audio streamed.

**Architecture chosen (user-confirmed 2026-06-21): SINGLE OWNER in coreaudiod.** The engine
(inside coreaudiod) claims the interface as soon as the plugin loads — for MIDI **and**
audio — and holds it for its whole life. Audio isoc chains are layered on/off at
StartIO/StopIO. Because the engine is the *only* USB owner ever, there is **no interface
handoff** and the wedge-race class is structurally gone. The user-space helper still owns
the "RigKontrol 3" CoreMIDI source and receives reports over **UDP 127.0.0.1:17890**
(coreaudiod's sandbox blocks `MIDIClientCreate` → `-10833`, and UDP loopback is the proven
egress; both de-risked on hw earlier).

**IMPLEMENTED + HW-VALIDATED (2026-06-21):** MIDI without audio confirmed on hardware —
log shows `engine USB up … audio idle` at Initialize (not StartIO) and `engine open: USB
claimed (MIDI relay live; audio idle)`; with `/tmp/rk3_midi_on` present and NO audio stream,
the helper received buttons 1–9 → CC20–28 127/0 and pedal `analog2 raw=48 → CC11`. No storm
trip during the session. Details:
- `rk3_engine.{h,c}` — lifecycle split into `rk3_engine_open` (claim USB + handshake +
  spawn thread/run-loop; **no** AUDIO_PARAMS, **no** isoc), `rk3_engine_audio_start`/
  `_audio_stop` (set AUDIO_PARAMS + prime/abort isoc), `rk3_engine_close`. Isoc callbacks
  now gate on new `audio_running` (not `running`, which means "thread alive"). `engine_sync`
  marshals prime/abort onto the engine run loop from the StartIO/StopIO thread (bounded
  wait). `rk3_engine_start/stop` kept as composites for the tools. `rk3_engine_usb_dead`
  exposes a removal flag.
- **MIDI relay is now audio-independent + a LIVE sentinel toggle.** A ~1 Hz
  `CFRunLoopTimer` (`midi_sentinel_tick`) on the engine run loop watches `/tmp/rk3_midi_on`
  and arms/disarms the relay to match — so `touch`/`rm` enable/disable MIDI **live, with no
  reinstall and no audio needed**. Storm guard kept; on a storm trip it now **latches**
  (`midi_storm_tripped`) so the poller won't re-arm until the sentinel is `rm`'d + re-touched.
  Device-removal result codes (`kIOReturnNoDevice`/`NotResponding`/`NotAttached`) set
  `usb_dead`.
- `plugin/rk3_device.c` — a **supervisor thread** (spawned in `Drv_Initialize`) opens the
  engine at load, **retries every 2 s** while the device is absent/claimed, and recycles it
  on `usb_dead` while audio is idle. `StartIO` → `audio_start` (opens on demand under the
  same mutex if the supervisor hasn't yet), `StopIO` → `audio_stop` (engine + MIDI **stay
  up**). All `gEngine/gEngineOpen/gIOCount` under `gStateMutex`; open happens only under that
  lock so the device is never opened twice.
- **Critical correctness point baked in:** `AUDIO_PARAMS` reads its ack on EP 0x81 — the
  SAME bulk-in pipe the relay's async read uses. `audio_start` therefore runs the command
  I/O *on the engine thread* and quiesces the relay (abort EP 0x81) around it, then re-arms.
  Two readers on one pipe = bug; this avoids it.

**⚠️ Storm history (still relevant):** an earlier relay test (zero-delay self-resubmit)
stormed coreaudiod to a reboot. The storm guard (256-cb window / 50 ms floor → disable) is
the real protection and is retained. The opt-in sentinel is the belt-and-suspenders: the
storm fix is still **UNTESTED on hardware**, so the default install is byte-for-byte the
known-good audio path until the user `touch`es the sentinel.

**NEXT (test, do gently — install is the reboot-risky step; idle the device + quit audio
apps first):**
1. `make install-driver` (SAFE: relay dormant w/o sentinel; engine now opens USB at load).
   Verify audio still fine in Reaper, and `make driver-log` shows `engine USB up … audio idle`.
2. `./build/rk3midihelper -v` (or `make install-midihelper`).
3. **MIDI with NO audio (the new capability):** `touch /tmp/rk3_midi_on` — within ~1 s the
   log shows `arming foot-controls relay`; press buttons / move pedal → helper prints CCs,
   **no audio stream needed.** `rm /tmp/rk3_midi_on` → disarms live.
4. **MIDI + audio:** start RK3 audio in Reaper with the sentinel present; controls keep
   working alongside audio (relay is quiesced only briefly across AUDIO_PARAMS).
   - Storm trips → `driver-log`: `completion storm … last result=0x……` (capture the code).
     Latched off; `rm` + re-`touch` to retry. Then switch to `ReadPipeAsyncTO`/poll-thread.
   - Removal while idle → supervisor logs `device gone … reopen on attach`; replug recovers.

Full blow-by-blow detail in memory: `rk3-coreaudio-project.md` +
`rk3-coreaudio-hardware-facts.md` (button bit-scramble, pedal axis2 raw 40–270, LED/display
programmability for the deferred output phase).

---

## TL;DR status

- **STEP 4 CORE DONE & validated (2026-06-21): the RK3 is a working CoreAudio device.** The
  `RigKontrol3.driver` AudioServerPlugin loads in the sandboxed out-of-process
  `Core-Audio-Driver-Service.helper`, appears in Audio MIDI Setup / System Settings as a
  2-in/2-out 48 kHz USB device, and does real full-duplex audio:
  - **Playback: clean 440 Hz tone confirmed BY EAR through CoreAudio** (`tools/iotest`),
    `pb_underruns=0 pb_overruns=0 in_errs=0 out_errs=0` over 5–8 s.
  - **Capture: real-time real signal through CoreAudio** (`iotest --in` -> `iotest_cap.wav`,
    240128 frames = exactly 5.00 s, peak −27 dBFS — not silence).
  - The USB engine opens + streams isoc with **0 errors from inside the sandboxed driver
    helper** (even stronger than the Step-4 probe verdict).
  - Architecture: `rk3_engine.{h,c}` (USB+isoc on its own thread, SPSC rings) + `plugin/
    rk3_device.c` (HAL object model, Float32<->S24_3BE bridges). Build/install:
    `make driver` / `make install-driver` (sudo) / `make driver-log`.
  - **STEP 5 LATENCY TUNING (2026-06-21): 81.5 ms → ~30 ms round-trip best case**, via the
    buffer-size discovery (below) + `RK3_N_OUT_XFERS=2` + `RK3_OUT_CUSHION=6*320`. BUT latency is
    currently UNSTABLE run-to-run (30–69 ms) — see "OUT-fill instability" below. NEXT TASK = a
    FILL SERVO to make it stable+low. See Step 5 section.
  - Remaining for later: a FILL SERVO (the real next task — see below), real Reaper guitar
    play-test, multi-rate (only 48 kHz now), CoreMIDI bundle.

  - **⚠️ NEXT-AGENT START HERE — two big findings after the 51.7 ms "ship" claim below, which
    is now SUPERSEDED:**
    1. **Buffer-size breakthrough.** The cushion floor was being set by the HOST IO BUFFER size,
       not by drift. `iotest` had defaulted to CoreAudio's ~512-frame buffer; Reaper uses 64.
       Added `iotest --buf N` (default 64). With a small/smooth producer the floor becomes the
       CONSUMER bite (`fill_out` pops ~1 OUT transfer ≈ 8 ms ≈ 384 frames per `out_done`).
       Sweep @64-frame buffer, N_OUT=3: cushion 512→46.2 ms, 384→36.75 ms, 320→35.5 ms; then
       N_OUT=2/cushion 320 → **30.0 ms**, pb_underruns=0 over 8 s. Current shipped constants now
       reflect this (N_OUT=2, cushion 320). **Only valid when the host uses a small buffer** —
       at the ~512 default this underruns, so do NOT ship <768 without confirming Reaper's 64.
    2. **DRIFT HYPOTHESIS OVERTURNED — DO NOT BUILD CLOCK SYNC as a latency fix.** Added
       telemetry (a `dev_frames` device-clock counter + a once/sec `clk:` log of OUT ring fill)
       and ran a 90 s loopback. Result: `dev_rate ≈ 47999 Hz` = only ~0.002 % off nominal
       (negligible; ~25 min to drain the cushion, NOT ~once/min). So the comments below that say
       "clock sync is the prerequisite" / "drift drains the ring in ~67 s" are WRONG — ignore.
    3. **The REAL problem = OUT-fill INSTABILITY.** The 90 s telemetry showed: fill seeds ~320,
       a startup transient drops it to ~54 in ~3 s → bottoms out → 2 underruns → the underrun
       self-heal then BALLOONS the standing fill to ~1550 frames, where it sits stable for the
       rest of the run (→ 69 ms median that run). `RK3_OUT_CUSHION` only sets the INITIAL fill,
       NOT the steady state — so the "30 ms" / "51.7 ms" wins were LUCKY runs. True latency is
       unstable 30–69 ms run-to-run.
    4. **THE FIX (next task) = a FILL SERVO.** Hold the OUT standing fill at a low setpoint by
       trimming the output clock RATE via the `Drv_GetZeroTimeStamp` slope in `plugin/rk3_device.c`
       (click-free; also absorbs the tiny residual drift for free). Alternative/complement: bound
       the self-heal balloon in `fill_out` (rk3_engine.c). Telemetry (`dev_frames`, `clk:` log)
       is left INSTALLED in the engine to tune the servo — remove once done.

### Pre-Step-4 standalone-harness validation (tools/stream.c, still true)

- **Capture (input): DONE & validated** — real-time, real audio, `input_panic=0`, 0 errors.
- **Playback (output): DONE & validated by ear (2026-06-21).** Clean 440 Hz tone, correct
  pitch, no distortion. 499/499 transfers, 0 drops, `output_panic=0`.
- **THE root-cause bug (the one that mattered): missing OUT-stream phase offset.** Linux
  seeds the playback read pointer at `audio_out_buf_pos = out_pos = BYTES_PER_SAMPLE + 1
  = 4` (`audio.c` `snd_usb_caiaq_pcm_prepare`). We started the OUT audio bytestream at
  phase 0. That constant **4-byte** phase error byte-rotates *every* sample, so a real
  signal plays as a distorted, pitch-shifted buzz while **silence stays clean** (rotated
  zeros are still zeros). Fixed in `stream.c` by seeding `tone_pos = 4` (the `s.off`
  default). Listening sweep confirmed: `--off 0/2/3/5` buzz, `--off 1/4` clean (1 and 4
  differ by 3 bytes = a channel swap, inaudible on an identical-both-channels tone; **4**
  is the principled value matching Linux and giving correct L/R).
- **Step 4 sandbox de-risk DONE (2026-06-21): sandboxed `coreaudiod` CAN open + claim the RK3
  via IOUSBLib** (euid=202 probe: USBDeviceOpen/SetConfiguration/USBInterfaceOpen/SetAlternate
  all ✓). No helper daemon needed — the HAL plugin owns USB directly. (HAL plugins must live in
  system `/Library/Audio/Plug-Ins/HAL/`, not `~/Library`.)
- **Next action:** drive the working driver in Reaper (real-world shakeout), then Step 5 (latency
  + multi-rate + device-clock sync). See "Next steps".

## End goal (user, 2026-06-21): guitar rig in Reaper

The driver exists to play **guitar through Reaper at sub-10ms** round-trip, plus use the RK3's
foot buttons/pedal as MIDI. Three deliverables, in order:

1. **Audio plugin (Step 4) → DONE** → RK3 is a standard CoreAudio 2-in/2-out device, visible in
   System Settings/Audio MIDI Setup (and therefore Reaper), full-duplex audio validated. This is
   what makes "play guitar in Reaper" possible. NOT YET tried inside Reaper by the user.
2. **Latency tuning (Step 5, NEXT)** → sub-10ms is *realistic but must be measured, not
   promised*. The engine today buffers heavily for safety (`RK3_N_XFERS=8` × `RK3_FRAMES_PER_XFER=64`
   microframes ≈ 64ms in flight, plus `RK3_OUT_CUSHION` ≈ 85ms playback prefill); shrink transfer
   size + primed depth + cushion and declare a small CoreAudio buffer/safety offset. **Hard floor =
   the OUT drift cushion**: EP `0x06` OUT is a *no-sync* isoch endpoint (no feedback EP), so the
   host must keep a margin or it underruns. Realistic ~8–15ms round-trip, sub-10ms reachable at
   small buffers — confirm empirically.
3. **CoreMIDI driver (later, SEPARATE bundle)** → the audio HAL plugin does NOT carry MIDI. RK3
   buttons/pedal arrive as caiaq control reports over **bulk EP `0x01`/`0x81`** (decoded by Linux
   `input.c`/`midi.c`). A separate CoreMIDI MIDIDriver plugin turns those into a system MIDI
   source visible in Reaper/any app.

## Build & run

THE driver (Step 4, the product):
```
cd /Users/mihai/DEV/rk3-coreaudio
make driver                 # build build/RigKontrol3.driver bundle
make install-driver         # SUDO: rm -> cp -> codesign in place -> killall coreaudiod
make driver-log             # os_log from subsystem com.mihai.rk3 (factory/Initialize/engine)
make iotest                 # build the AUHAL end-to-end tester
./build/iotest 8            # play 8s 440Hz tone OUT the RK3 (listen on its analog out)
./build/iotest 5 --in       # capture 5s from RK3 -> iotest_cap.wav ; afplay it
./build/iotest 6 --latency  # LOOPBACK round-trip measurement (patch RK3 out->in cable);
                            # 1 impulse/s, reports min/median/mean/max spls+ms (GROUND TRUTH)
make uninstall-driver       # SUDO: remove + restart
```
IMPORTANT install gotchas (both cost real time — see "Next steps" notes):
- HAL plugins load ONLY from system `/Library/Audio/Plug-Ins/HAL/` (root:wheel), NEVER
  `~/Library` (coreaudiod's `$HOME` is invalid under sandbox).
- ALWAYS `rm -rf` the installed bundle + re-`codesign` IN PLACE before restart, or the
  out-of-process loader SIGKILLs at dlopen (CODESIGNING / Invalid Page). `make install-driver`
  does this; do NOT hand-`cp` over an existing install.
- `coreaudiod` restart needs `sudo` — the agent CANNOT do it non-interactively. Ask the user to
  run `make install-driver` (or `sudo killall coreaudiod`) themselves and report back.
- After a code change you MUST reinstall+restart for it to take effect; the old bundle stays
  loaded otherwise.

The standalone pre-Step-4 harness (still useful for transport experiments, owns the device
EXCLUSIVELY so it conflicts with the installed driver — stop IO first):
```
make stream                 # also: usbprobe handshake codectest ; make test (codec unit test)
./build/stream 4 --play     # 4s capture+playback (clean tone); writes capture.wav
./build/epdump              # dump config descriptor + isoc EP sync types
```
Run commands yourself (user expects this). Device must be plugged in. `afplay capture.wav` /
`afplay iotest_cap.wav` to audition input. The device's *output* (the tone) can only be judged
by ear on the RK3's analog out — ask the user. NOTE: zsh — use `&&` not `and`; the user's shell
shadows `log` with a function, so call `/usr/bin/log` for log queries.

## Confirmed hardware facts (live device)

- USB high-speed, 1 config (value 1), 1 vendor-specific interface (class 255/255/0).
  macOS does NOT auto-configure it → must `SetConfiguration(1)` before interface nubs appear.
- **alt 1** (streaming): bulk EP `0x01` OUT / `0x81` IN (commands+MIDI), isoc EP `0x82`
  IN (capture), isoc EP `0x06` OUT (playback). All maxPacket 512.
- device_spec: num_analog_audio_in/out = 2/2, num_midi 1/1, **data_alignment = 2**,
  fw_version 11, hw_subtype 6. n_streams = 1. Audio = **S24_3BE** (confirmed in
  `caiaq/audio.c:49` = `SNDRV_PCM_FMTBIT_S24_3BE`). bpp(48k) = 88.
- Isoc is **microframe-based**: IOKit isoc frame list counts microframes (8 = 1ms), but
  `frame_start` (GetBusFrameNumber) is in 1ms-frames. A transfer of N microframes spans
  N/8 ms-frames → advance frame_start by N/8, not N. `FRAME_STRIDE = FRAMES_PER_XFER/8`.

## The hard-won transport findings (THE important part)

These came from many live experiments; do not re-derive, do not regress:

1. **Capture works** as a free-running self-resubmitting chain: prime `N_XFERS=8`
   transfers at consecutive frame_starts; each completion resubmits at
   `frame_start += N_XFERS*FRAME_STRIDE`. Never re-read the bus per submit. With
   `FRAMES_PER_XFER=64` (8ms) this is perfect real-time, 0 errors.

2. **OUT scheduling** must use the SAME free-running self-resubmitting chain, primed at
   STARTUP in `main` (NOT mid-stream — submitting OUT from inside an IN completion
   callback fails hard). Each `out_done` does `frame_start += N_OUT_XFERS*FRAME_STRIDE`,
   refill, resubmit. `submit_out` is self-healing (retries bumping frame ahead of bus on
   `kIOReturnBadArgument`) so a slot never silently drops out of the chain.

3. **`WriteIsochPipeAsync` rejects a transfer that ends in 0-length frame(s)** with
   `kIOReturnBadArgument` (0xe00002c2). Interior and leading 0-length frames are FINE.
   Fix: if the last frame is 0, move the last non-zero packet into the final slot
   (preserves total bytes). This unlocked sending the bursty cadence with the basic API
   (no need for the low-latency isoch API).

4. **IOKit isoc packs frame data CONTIGUOUSLY by frReqCount** — microframe `f`'s bytes
   live at `buf + Σ(frReqCount[0..f-1])`, NOT at `f*FRAME_BYTES`. **THE bug**: playback
   wrote each microframe at `f*FRAME_BYTES` (512 stride). That works for capture only
   because every capture frReqCount==512. For variable OUT lengths it sent garbage in the
   gaps → "random distorted noise." Fixed by packing at a running `off`. This is what
   turned noise into a recognizable tone.

5. **`output_panic` (bit 0x80 in capture check bytes) IS a reliable signal of OUT
   *framing* correctness** (corrects an earlier wrong claim). Proof: sending OUT with the
   check byte at the wrong slot (`--off`-unrelated `--chk0`, i.e. `i%4==0` instead of the
   Linux `i%4==2`) sets the device's `output_panic` bit on ~16927/17212 packets; with the
   correct slot it is 0 on 100%. So a clean `output_panic` confirms the check-byte layout
   and packet sizing are right — but it does **not** police the audio sample *phase* (the
   real bug had panic=0 yet buzzed). Use it as a framing check, not an audio-quality check.

6. **OUT cadence (bursty vs steady) does NOT matter for correctness.** The device produces
   capture in a bursty 88-or-0 per-microframe cadence (~6/11 carry bpp=88) averaging the
   rate; we can mirror it (`--default`, FIFO pattern queue `pat_push`/`pat_pop`) OR send a
   steady 6-frames-every-microframe stream (`--const`). With the phase fix, **both play a
   clean tone**; without it, both buzz identically. (This is why earlier cadence
   experiments all "buzzed" — the phase bug, not the cadence, was the cause.) Bursty mirror
   is kept as the default because it is rate-locked to the device clock for free.

7. **OUT audio-stream phase: seed the read pointer at `BYTES_PER_SAMPLE + 1 = 4`.** See the
   TL;DR root-cause note. This is the single fix that turned the buzz into clean audio.

## Codec (validated, byte-exact, unit-tested in tools/codectest.c)

- `codec_mode2.c`: `caiaq_capture_mode2` (check byte at `i%4==0`, value `(~(i>>2))&1`,
  device sets 0x80=output panic / low 6 bits = check) and `caiaq_playback_mode2` (data
  byte then check at `i%4==2`, value `(~(i>>2))&1`). These mirror Linux
  `read_in_urb_mode2` / `fill_out_urb_mode_0` (mode_0 is used for alignment 0 AND 2).
- Tone is S24_3BE, 440 Hz, both channels identical. **Proven clean** end-to-end via
  `sent.wav` (decode-what-we-send diagnostic) → FFT shows a single 440 Hz peak, 40+ dB SNR.

## Current state of tools/stream.c

Heavy diagnostics (the `sent.wav` writer + decode loop, and the non-latching
`op_set`/`op_clear` panic-bit counters + `output_panic first set` print) have been
REMOVED now that playback works. What remains:
- `len_hist` + the end-of-run OUT audio-rate audit (accurate: divides by
  microframes/8000, reads ~+0.01% at 48k) — keep; handy for the rate matrix (Step 5).
- `pat_synth` / `out_acc` — the startup/underrun cadence fallback (first ~8 primed
  transfers before the capture-pattern queue fills); `out_underruns` should read ~8.

Experiment flags (kept; cheap and useful for bring-up): `--silence` (digital zero),
`--const` (steady cadence), `--off N` (OUT phase, **default 4** = the fix), `--chk0`
(wrong check slot — triggers output_panic, for regression demos), `--freq N`.

Key constants: `N_XFERS=8`, `N_OUT_XFERS=8`, `FRAMES_PER_XFER=64`, `FRAME_BYTES=512`,
`FRAME_STRIDE=8`, `OUT_MIN_LEAD=24`, `PAT_RING=256`.

`fill_out()` is the heart of playback: `pat_pop` (or `pat_const`) the cadence → ensure
last frame non-zero → contiguous-pack via `caiaq_playback_mode2` from the looping tone
ring (seeded at phase `tone_pos = 4`).

## Driver architecture (Step 4, the product)

- `rk3_engine.{h,c}` — **the validated stream.c transport, extracted into a reusable module.**
  Owns the RK3 (USB open + EP1 handshake + AUDIO_PARAMS), runs the capture chain + free-running
  OUT chain on its OWN pthread with a dedicated CFRunLoop, and exchanges audio through two
  lock-free single-producer/single-consumer byte rings of native S24_3BE. `rk3_engine_start/stop`
  manage the thread; `rk3_engine_read_input` / `rk3_engine_write_output` are the Float32 bridges
  called from CoreAudio's IO thread. The proven OUT 4-byte phase is preserved via a STARTUP GATE:
  the engine sends silence on OUT until CoreAudio has buffered `RK3_OUT_CUSHION`, then seeds
  `pb_r = RK3_OUT_PHASE (4)` and plays out. Constants mirror stream.c (`RK3_N_XFERS=8`,
  `RK3_FRAMES_PER_XFER=64`, `RK3_FRAME_STRIDE=8`, `RK3_OUT_MIN_LEAD=24`, `RK3_PAT_RING=256`).
  Diagnostics counters: `pb_underruns/pb_overruns/cap_overruns/in_errs/out_errs` (in driver-log).
- `plugin/rk3_device.c` — **the AudioServerPlugin.** HAL object model (plugin obj 1 → device 2 →
  input stream 3 / output stream 4), Float32 48 kHz stereo, all the property getters, `StartIO`/
  `StopIO` (refcounted; start/stop the engine), host-clock `GetZeroTimeStamp`, `WillDoIOOperation`
  (claims ReadInput + WriteMix, in-place), `DoIOOperation` (Float32 ↔ engine bridge). Factory
  `RK3DeviceFactory`, UUID `8A0197C9-09B4-4CB9-A7E9-54488A30DE49`. `plugin/rk3_device.plist` =
  Info.plist (bundle id `com.mihai.rk3.audio`, AudioServerPlugIn type UUID).
- `plugin/sandbox_probe.c` + `plugin/Info.plist` — the throwaway de-risk probe (Step 4 sandbox
  verdict). Keep for reference; `make install-probe`/`uninstall-probe`. Not part of the product.
- `tools/iotest.c` — AUHAL tester bound to OUR device by UID (does not disturb the system default
  device). `iotest [s]` plays a tone; `iotest [s] --in` records to `iotest_cap.wav`.

## Files (shared protocol + standalone harness)

- `usb_iokit.{h,c}` — IOUSBLib transport (`IOUSBInterfaceInterface300`). open/close, bulk
  EP1, isoc EP2/EP6 pipe resolution. `rk3_usb` struct holds pipe refs.
- `caiaq_protocol.{h,c}` — device spec, GET_DEVICE_INFO, AUDIO_PARAMS, bpp calc. Validated.
- `codec_mode2.{h,c}` — packet (de)interleave + check bytes. Validated.
- `tools/usbprobe.c` (step 1, done), `tools/handshake.c` (step 2, done),
  `tools/codectest.c` (codec unit test, all pass), `tools/stream.c` (step 3, **DONE** —
  capture + playback both clean), `tools/epdump.c` (config-descriptor / EP sync dump).

## Next steps (in order)

1. ~~Step 3 (isoc transport) + Step 4 sandbox de-risk + Step 4 driver~~ — **ALL DONE & validated
   2026-06-21.** RK3 is a working CoreAudio device (see TL;DR). Two install gotchas are now baked
   into `make install-driver` and documented below.

2. **FIRST: drive it in Reaper (real-world shakeout).** The user's end goal is guitar through
   Reaper. The device works in `iotest`; the natural next validation is selecting "RigKontrol 3"
   as Reaper's audio device and confirming guitar-in + monitoring-out hold up under a real DAW's
   IO patterns (varied buffer sizes, exclusive/shared, start/stop). Watch `make driver-log` for
   `pb_underruns/pb_overruns/in_errs/out_errs`. This will likely surface what to prioritize in
   Step 5. (Ask the user to do this; the agent can't drive Reaper.)

3. **Step 5: latency tuning + rate matrix. IN PROGRESS (2026-06-21).**
   - Reaper showed `~6.6/6.6ms` but that was ONLY SafetyOffset(256)+buffer(64)=320 frames each way;
     `kAudioDevicePropertyLatency` was reported as 0, so the display was a large UNDER-count.
   - **The output path has TWO big hidden terms, not one:** (1) `RK3_OUT_CUSHION` — a *standing*
     fill of the OUT ring (producer=CoreAudio, consumer=USB are rate-matched, so it never drains);
     and (2) the **isoc pipeline depth** = `RK3_N_OUT_XFERS`(8) transfers each `RK3_FRAMES_PER_XFER/8`
     ms (8ms) scheduled ahead — each refilled buffer is rescheduled `N_OUT_XFERS*stride` ahead, so
     popped audio isn't heard for ~64ms. True OUT latency was ≈ cushion(85ms)+pipeline(64ms) ≈ 150ms.
   - DONE this cycle: (a) `RK3_OUT_CUSHION` 4096→**512 frames** (~85ms→~10.7ms), one #define in
     rk3_engine.h. (b) Honest scope-aware `kAudioDevicePropertyLatency` in rk3_device.c: OUT =
     cushion + `N_OUT_XFERS*xferFrames`, IN = one xfer; reported on the device object only (streams
     return 0 to avoid double-count). (c) New **`iotest --latency`** loopback tool = GROUND TRUTH.
   - **MEASURED (2026-06-21, iotest --latency loopback, out-L→in-L cable):** cushion=512,
     FRAMES_PER_XFER=64, N_OUT_XFERS=8 → **83.8 ms median** round-trip (±0.2ms over 7 impulses,
     input −5 dBFS). Decomposition confirmed: OUT cushion 512 + OUT pipeline (N_OUT_XFERS*FRAMES/8
     = 8*8ms = 64ms = 3072 frames) + ~440 frames input/CoreAudio/safety ≈ 4024 frames. **The OUT
     isoc pipeline (3072 frames/64ms) is now the dominant term**, not the cushion.
   - OUT pipeline latency formula = `N_OUT_XFERS * (FRAMES_PER_XFER/8) ms`. In prime_chains the 8
     OUT xfers are scheduled at start+i*FRAME_STRIDE; out_done reschedules each finished buffer
     N_OUT_XFERS*FRAME_STRIDE ahead, so popped audio waits the full pipeline. OUT_MIN_LEAD(24) is
     only the self-heal retry floor, NOT a steady-state term.
   - **DO NOT shrink `RK3_FRAMES_PER_XFER`.** Tried 64→16 (2ms): STORMED coreaudiod + the whole
     audio subsystem to 100% CPU, required a REBOOT (2026-06-21). Root cause = isoc completion
     callback RATE — smaller transfers = 4× more completions/sec on the sandboxed helper CFRunLoop,
     which overloaded it. NOT a blocking hang (the bulk handshake already has 200/500ms timeouts).
     Reverted to 64. Re-measured baseline after revert: **81.5 ms median** (consistent with 83.8).
   - SAFE latency lever = `RK3_N_OUT_XFERS` (in-flight OUT transfers). It does NOT change the isoc
     completion rate (a transfer still completes every 8ms regardless of queue depth) so no extra
     helper load; worst case is an underrun (silence), never a storm.
   - **LATENCY TUNING COMPLETE (2026-06-21). Final measured sweep (iotest --latency, out-L→in-L):**
     | config | median RT | counters |
     |---|---|---|
     | cushion 512, N_OUT 8 (baseline) | 81.5 ms | clean |
     | cushion 512, N_OUT 4 | 51.9 ms | **pb_underruns=27/8s — clicks** |
     | cushion 768, N_OUT 4 | 59.8 ms | clean |
     | cushion 768, N_OUT 4, SafetyOffset 128 | 62.3 ms | clean but WORSE |
     | **cushion 1024, N_OUT 3, SafetyOffset 256** | **51.7 ms** | **clean ← SHIPPED** |
   - **CURRENT INSTALLED/SHIPPED CLEAN CONFIG: `RK3_N_OUT_XFERS=3`, `RK3_OUT_CUSHION = 6*1024`,
     `RK3_FRAMES_PER_XFER=64`, `kSafetyOffsetFrames=256`, OUT phase 4. → 51.7 ms median round-trip,
     pb_underruns=0, all counters clean. 37% cut from the 81.5 ms baseline, guitar-clean.** (The
     51.7 ms run's tail — mean 55, max 64.7 — is discrete-impulse scheduling phase jitter, NOT audio
     glitching; steady-state continuous latency = the median.)
   - **Lessons (do not re-derive):** (a) `RK3_OUT_CUSHION` is the FINE-grain jitter absorber; it's
     what kills pb_underruns when you cut the pipeline. N_OUT=4 needed cushion 768, N_OUT=3 needed
     1024. (b) `RK3_N_OUT_XFERS` sets BOTH pipeline depth (8ms/step) AND hiccup resilience; N_OUT=4
     is the resilience knee at cushion 512. (c) `kSafetyOffsetFrames` is a DEAD END — lowering it
     (256→128) made round-trip WORSE and collapsed variance to 0; it only shifts CoreAudio scheduling
     PHASE, not the physical path. Reverted to 256.
   - **Parameter shaving is now EXHAUSTED.** N_OUT=2 = only double-buffered (will glitch, needs yet
     more cushion = net wash); FRAMES_PER_XFER can't shrink (storm); SafetyOffset is a dead end.
     **The only remaining big latency lever is device-clock sync** — the 21 ms cushion exists PURELY
     to absorb host/device drift; a real device clock derived from isoc frame timing would let it
     shrink substantially. That's the proper Step-5 follow-on (overlaps item (c) below).
   - NOT YET DONE: user has not play-tested in Reaper with a real guitar. Recommended before/instead
     of building clock sync — confirm 51.7 ms *feels* right and runs clean for minutes first.
   - DEV-LOOP HAZARDS (both cost real time 2026-06-21, both needed physical recovery):
     (1) killing coreaudiod while the RK3 streams, OR a tool that opens the device and exits without
     releasing, WEDGES the USB interface → engine_start logs "device absent/claimed?" and ALL StartIO
     fail (AudioOutputUnitStart returns 'what'=2003329396) even after coreaudiod restart → fix =
     PHYSICAL unplug/replug. (2) a driver that overloads/blocks the helper storms coreaudiod + every
     audio client (GuitarPro etc spin-retry) to 100% CPU; `uninstall-driver` may NOT clear it → fix =
     quit audio apps / REBOOT. ALWAYS idle the RK3 + close audio apps before `install-driver`; iotest
     now cleans up its AudioUnit on every error path. (b) **Multi-rate**: today the driver hard-codes 48 kHz
   (`kSampleRate` in `rk3_device.c`, single entry in AvailableNominalSampleRates). Add
   44100/88200/96000/192000 — make the device advertise them, re-run AUDIO_PARAMS on rate change
   (`PerformDeviceConfigurationChange`/restart engine), verify S24_3BE byte order + OUT phase per
   rate. (c) **Device-clock sync**: `GetZeroTimeStamp` currently uses the host clock at the
   nominal rate; the rings absorb the ~0.01% device/host drift (fine for minutes, drifts over long
   runs). Proper fix: derive the timestamp from the actual isoc completion cadence (device clock).

4. **Step 6 (separate CoreMIDI bundle):** foot buttons/expression pedal as a system MIDI source
   (caiaq control reports over bulk EP `0x01`/`0x81`; ref Linux `input.c`/`midi.c`). The audio
   plugin does NOT carry MIDI. Also cosmetic: RK3 7-seg display init.

### The two install gotchas (both already handled by `make install-driver`; do not regress)

- **Location:** HAL plugins load **only** from system `/Library/Audio/Plug-Ins/HAL/` (root:wheel).
  A `~/Library` install is silently ignored — sandboxed coreaudiod's `$HOME` is invalid.
- **Signing/copy:** on macOS 26 third-party HAL plugins load out-of-process in the sandboxed
  `com.apple.audio.Core-Audio-Driver-Service.helper`. If you `cp -R` a rebuilt bundle *over* an
  existing install, the in-place page overwrite desyncs the on-disk pages from the code-signature
  seal: `codesign -vv` still says "valid on disk", but at `dlopen` the kernel hashes the mapped
  pages, they mismatch, and the helper is **SIGKILLed (CODESIGNING / Invalid Page)** before our
  factory runs — coreaudiod logs only "Error loading driver bundle … Couldn't communicate with a
  helper application." Fix = `rm -rf` install dir → `cp` → `codesign --force --sign -` IN PLACE →
  `killall coreaudiod` (what `make install-driver` does). Diagnose helper crashes via
  `/Library/Logs/DiagnosticReports/com.apple.audio.Core-Audio-Driver-Service.helper-*.ips`
  (root-readable; parse: skip the first header line, the rest is one JSON object).
- The sandbox de-risk probe (`plugin/sandbox_probe.c`) already proved sandboxed coreaudiod
  (euid=202) can fully open+claim the RK3 over IOUSBLib — the real driver then confirmed it with
  0 USB errors from inside the helper. No helper-daemon/shared-memory fallback is needed.

## Memory

Project memory at `/Users/mihai/.claude/projects/-Users-mihai-DEV-caiaq/memory/`:
`rk3-coreaudio-project.md`, `rk3-coreaudio-hardware-facts.md` (kept current with the
findings above).
