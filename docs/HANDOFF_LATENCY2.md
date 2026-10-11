> **Superseded (2026-10).** The premises below did not hold up on macOS 26:
> ~1000 isoc completions/s is *not* a hard ceiling — 1000, 1500 and 2000/s all run
> clean inside the driver process — and the shipped geometry is now 1 ms transfers
> (see `RK3_DEFAULT_*` in `rk3_engine.h` and *Latency tuning* in the README).
> The fill setpoint and capture floor described here have changed meaning (they
> are now the ring *trough* and the post-read *margin*). Kept for the history.

# RK3 CoreAudio — LATENCY HANDOFF #2 (for a fresh agent after /clear)

**Your one job:** squeeze more round-trip / monitoring latency out of the RigKontrol 3
CoreAudio driver, in **userland only** (DriverKit is a confirmed hard dead end — free
Apple account can't sign a dext; see `HANDOFF_DRIVERKIT.md` if curious, don't re-chase it).

This file is self-contained. The prior pass is in `HANDOFF_LATENCY.md` (read its STATUS +
§1.55 "things proven dead" if you want the war stories); the full project history is in
`HANDOFF.md` and the memory files. **You do not need them to start — start here.**

---

## STATUS — where we are

Shipped/committed floor: **14.31 ms** loopback round-trip (`iotest --latency`),
~10–15 ms in Reaper under real plugin load, clean. Config in `rk3_engine.h`:

| Stream | size | rate | knob |
|--------|------|------|------|
| IN (capture) | 32 µframes = **4 ms** | 250/s | `RK3_FRAMES_PER_XFER=32` |
| OUT (playback) | 16 µframes = **2 ms** | 500/s | `RK3_OUT_FRAMES_PER_XFER=16` |
| | | **750/s total** | |

Other live settings: `RK3_N_OUT_XFERS=2`, `RK3_OUT_FILL_SETPOINT=224`,
`RK3_CAP_FILL_FLOOR=140`, `RK3_USE_LOWLAT=0`, `kSafetyOffsetFrames=256`.

**The transfer-size lever is MAXED.** Storm ceiling (sandboxed coreaudiod CFRunLoop):
375 / 500 / 750 per-sec callback rates all PROVEN SAFE; ~1000/s (global FPX=16) PROVEN
FATAL (reboot). The LowLatency isoch API that would have decoupled transfer size from
callback rate is a **no-op on Apple Silicon** (`updateFrequency` ignored — IOUSBLib copies
the frame list only at completion). So smaller transfers are exhausted.

**Remaining fat latency terms** (these, not transfer size, are what's left to attack):
- OUT isoc pipeline = `N_OUT_XFERS(2) × OUT_FPX(16)/8` = **4 ms** (can't shrink without storm)
- OUT standing fill = `OUT_FILL_SETPOINT(224)` ≈ **4.7 ms** ← **stale, Tier 1 below**
- IN standing fill / capture bite ≈ a few ms (already shaved hard via `CAP_FILL_FLOOR`)
- host buffer + safety offset ≈ a few ms (host-side, mostly fixed)

---

## BUILD / INSTALL / MEASURE (you can't run these — the USER does)

```
make driver                 # build build/RigKontrol3.driver
make install-driver         # SUDO; rm -rf -> cp -> codesign IN PLACE -> killall coreaudiod
make driver-log             # os_log stream (subsystem com.mihai.rk3) — counters + clk: line
make iotest
./build/iotest 10 --latency # GROUND TRUTH loopback. Patch RK3 out-L -> in-L with a cable.
                            # Run >=10s so the one-time cap drain (~5.5s) fires; report:
                            #   - median + MIN (min = post-drain steady) in frames/ms
                            #   - the clk: line: out_fill (min), cap_fill (min), pb_underruns
```
**You cannot run `install-driver` (sudo + restarts coreaudiod) or drive Reaper.** After
each build, have the user install, run `iotest --latency`, and paste the median/min + the
`clk:` line (`out_fill min`, `cap_fill min`, `pb_underruns`/`in_errs`/`out_errs`). One build
= one data point; latency is empirical and run-to-run noisy.

**Always tell the user to idle the RK3 + quit audio apps before installing.**

---

## THE PLAN (do in order; measure after every change)

### TIER 1 — do FIRST (cheap, overdue, zero storm risk)

**1. Re-sweep `RK3_OUT_FILL_SETPOINT`** (`rk3_engine.h:201`, currently `224`).
It's **stale**: 224 was tuned when OUT transfers were 32 µframes; ENGINE #4 then halved them
to 16 µframes (`RK3_OUT_FRAMES_PER_XFER=16`), so the OUT consumer bite roughly halved (~96
frames now) but the setpoint was never re-tuned. It's ~4.7 ms of standing fill — the 2nd-
fattest term. Pure ring-fill knob: worst case is an audible click, **never a storm**.
- Sweep **192 → 176 → 160 → 144** (one notch per build).
- Telemetry already in `rk3_engine.c:392` (`clk:` line). Keep the lowest setpoint where
  `out_fill_min` stays healthy (well clear of 0) AND `pb_underruns=0` across a full run.
- Expected recovery: **~0.7–1.7 ms**, no USB callback-rate change.
- Revert = one line back to 224.

### TIER 2 — decide after Tier 1 results

**2. Software direct monitoring in the engine** — the ONLY serious sub-10 ms *userland* path.
HW direct monitoring is **confirmed dead** (RK3 firmware has no analog/DSP monitor path; cmd
`0x0e` ACKs but is inert — ignore stale monitor hints in older memory/docs). BUT the engine
owns BOTH the capture ring and the playback ring on one RT thread, so mix captured IN samples
straight into the outgoing OUT ring **inside the engine**, bypassing CoreAudio/Reaper entirely.
- Collapses *monitoring* latency to ~(IN bite + OUT bite + standing fills) ≈ **~5–8 ms**.
- No new USB transfers, no callback-rate change → **no storm risk.**
- **Caveat:** this is the **dry** signal only (no Reaper amp-sim/FX). It does NOT lower the
  wet DAW round-trip — it's for live tracking *feel*.
- Requirements: expose as a HAL control (gain + on/off), **OFF/muted by default** so it
  doesn't double with Reaper's wet monitoring; add clip protection on the mix.
- Implementation hint: mirror the existing in-engine mix points; the OUT path already does an
  in-place Float32 multiply for volume (`plugin/rk3_device.c` WriteMix) — the monitor mix can
  ride the same S24_3BE OUT ring write the playback uses. Keep it on the engine thread, SPSC-safe.

**3. Try 96 kHz as a real measurement path** (multi-rate already works, `plugin/rk3_device.c:678`).
USB transfer time stays 4 ms IN / 2 ms OUT (frames-per-xfer are rate-scaled, `RK3_SCALE_TO_RATE`),
but **fixed frame-count terms** (CoreAudio buffer, safety offset) shrink in *milliseconds* at the
higher rate. Free host-side shave if CPU/plugins tolerate it. Worth one `iotest --latency` pass at
96k (switch in Audio MIDI Setup; watch `out_fill min`/`cap_fill min`/`underruns` — the 48k fill
tuning is scaled-but-unverified at other rates, re-measure don't assume).

### TIER 3 — low value, at most one pass each

- **Make buffer-size handling deterministic.** Confirmed: `plugin/rk3_device.c` has **no**
  `kAudioDevicePropertyBufferFrameSize` handler — CoreAudio's generic HAL shell negotiates it
  (`iotest.c:294` sets it and reads back "IO buffer = 64 frames", so it IS honored). If Reaper
  genuinely runs 64, leave it. Implementing explicit reporting/clamping would mainly improve
  measurement *consistency*, not median latency.
- **`RK3_CAP_FILL_FLOOR` 140→120** (`rk3_engine.h:247`): ~0.4 ms, but eats into the already-thin
  ~80-frame post-drain capture trough — closer to capture underrun. Risky; probably not worth it.
- **32-frame Reaper buffer:** ~0.67 ms at 48k, raises HAL/plugin pressure. Marginal.
- **`kSafetyOffsetFrames` 256→128:** prior notes say it got *worse* (only shifts CoreAudio
  scheduling phase, not the physical path). One re-test ONLY if a Tier-2 change alters pipeline
  length; otherwise skip.

### DO NOT CHASE (proven dead / wrong term — don't burn time here)
- `RK3_OUT_FRAMES_PER_XFER=8` (1 ms OUT → ~1250/s total, past the fatal storm ceiling → reboot).
- `RK3_USE_LOWLAT` / `updateFrequency` (no-op on Apple Silicon).
- Hardware direct-monitor commands (`0x0e`/`0x12`) — inert on the RK3 (no monitor path).
- `RK3_N_XFERS`, prime-lead, `RK3_OUT_MIN_LEAD` — none are the steady-state latency term.
- DriverKit — free-account wall, proven by Apple's own tooling. Not userland.

**Bottom line for the wet DAW path:** sweeping the setpoint is the last real userland median
win. True *wet, DAW-processed* sub-10 ms round-trip would need DriverKit (kernel-side isoc, no
coreaudiod storm ceiling) — out of scope. The userland sub-10 ms answer is dry software
monitoring (Tier 2 #2).

---

## HAZARDS / RULES (these cost reboots — internalize before touching the engine)

- **Anything inside coreaudiod that busy-loops or over-submits storms the whole audio
  subsystem to 100% CPU → may need a reboot.** Prove new transport paths in `tools/stream.c`
  (exclusive device owner, CANNOT storm coreaudiod) BEFORE putting them in the engine.
- **Storm ceiling:** 750/s callback rate is safe; ~1000/s is fatal. Don't raise transfer rate.
- **Install correctly or the bundle is SIGKILLed:** HAL plugins load ONLY from system
  `/Library/Audio/Plug-Ins/HAL/` (root:wheel); `make install-driver` does the required
  `rm -rf` + re-`codesign` IN PLACE — never hand-roll a `cp` over an existing install.
- **Wedged USB:** if a tool opens the device and exits without releasing, or coreaudiod is
  killed mid-stream, StartIO fails for everyone until a physical **unplug/replug**. Never
  replug mid-stream; idle + quit audio apps before each install.
- The driver is the single USB owner and relays **always-on MIDI** (reads EP `0x81`, the same
  bulk-in pipe AUDIO_PARAMS acks on); audio layers on/off at StartIO/StopIO. Don't break the
  MIDI relay while editing isoc — see `HANDOFF.md` for the quiesce-around-AUDIO_PARAMS rule.
- zsh: the user's `log` is shadowed by a function — use `/usr/bin/log` for queries. Use `&&`.

---

## FILES YOU'LL TOUCH
- `rk3_engine.h` — all the latency knobs (Tier 1 setpoint, Tier 3 cap floor).
- `rk3_engine.c` — engine transport + telemetry (`clk:` log ~line 392); Tier 2 monitor mix.
- `plugin/rk3_device.c` — HAL object model, controls, reported latency (Tier 2 monitor control,
  Tier 3 buffer-size). `/Users/mihai/DEV/caiaq` is the read-only Linux reference.

Keep the committed 14.31 ms config as the known-good fallback. Confirm with the user before
starting Tier 2 (it's a feature, not a knob).
