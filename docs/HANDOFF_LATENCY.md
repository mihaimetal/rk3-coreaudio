> **Superseded (2026-10).** See the note at the top of `HANDOFF_LATENCY2.md`.

# RK3 CoreAudio — LATENCY HANDOFF (for a fresh agent after /clear)

**Your one job:** reduce the round-trip audio latency of the RigKontrol 3 CoreAudio
driver, guided by what we learned reverse-engineering NI's own macOS driver. Read this
whole file first — it is self-contained. The broader project state is in `HANDOFF.md`
(audio bring-up, always-on MIDI) and in the memory files listed at the bottom; you do
NOT need them to start, but `HANDOFF.md` "Step 5" has the full latency-tuning history.

---

## STATUS — 2026-06-22: campaign essentially COMPLETE (14.31 ms, Reaper-validated)

This pass took round-trip from **23.66 ms → 14.31 ms** (loopback `iotest`), and the user
confirmed **~10–15 ms in Reaper under real plugin load, clean**. The transport
small-transfer levers are now **exhausted** — read this before re-opening the plan below.

**Final shipped config** (`rk3_engine.h`): `RK3_FRAMES_PER_XFER=32` (IN, 250/s),
`RK3_OUT_FRAMES_PER_XFER=16` (OUT, 500/s) → **750/s total callback rate**,
`RK3_CAP_FILL_FLOOR=140`, `RK3_N_OUT_XFERS=2`, `RK3_USE_LOWLAT=0` (basic path).
`kSafetyOffsetFrames=256`. Progression, all `underruns=0`:

| Step | Change | Round-trip | Rate |
|------|--------|-----------|------|
| baseline | FPX=64 | 23.66 ms | 375/s |
| #2 | IN FPX 64→32 | 19.79 ms | 500/s |
| #3 | cap-floor 208→140 | 16.98 ms | 500/s |
| #4 | OUT_FPX 32→16 | **14.31 ms** | 750/s |

**Key results that close out the plan in §2/§3 below:**
- **§3-A (low-latency isoch API) is a DEAD END on Apple Silicon.** The LL path was fully
  ported (gated behind `RK3_USE_LOWLAT`, still in `rk3_engine.c`) and proven 1:1-equivalent
  to the basic API. `updateFrequency` is a **NO-OP**: IOUSBLib is a shim over
  IOUSBHost/DriverKit that copies the frame list only AT COMPLETION — no mid-flight
  microframe status. So LL does NOT dissolve the callback-rate storm wall NI's Intel kext
  relied on. **Smaller transfers are the only real lever, and they are now maxed out.**
- **Storm ceiling re-mapped:** 375/500/750 per-sec callback rates all PROVEN SAFE this pass
  (750 over a 10 s bench + a multi-minute Reaper session). ~1000/s (global FPX=16) is still
  PROVEN FATAL (reboot). OUT_FPX cannot go below 16 (static asserts forbid it at IN FPX=32).
- The earlier "FPX=64→16 stormed" wall (§1.55 #1) was a **global** FPX=16 (~1000/s); the
  win here was lowering IN and OUT *separately* to land at 750/s, under that ceiling.

**What's left (all optional, confirm with user before starting):**
1. **Hardware direct monitoring** (`setDirectMonitoringMode`, EP1 cmd `0x0e`) — the only
   path to *near-zero* monitoring latency for live guitar (bypasses the host loop entirely).
   Payload not yet captured; needs more RE. This is the real sub-10 ms answer.
2. Marginal: `RK3_CAP_FILL_FLOOR` 140→~120 (~0.4 ms, trough margin already only ~80 frames
   — risky, not worth it) or re-sweep `RK3_N_OUT_XFERS` (already at the 2 knee).
3. If 750/s ever proves flaky on the user's machine under heavier load: revert
   `RK3_OUT_FRAMES_PER_XFER 16→32` = back to the rock-solid 16.98 ms / 500/s.

The rest of this file is the (now largely executed) plan that drove the pass; keep it for
the levers/hazard context, but the levers above supersede the "suggested plan" in §3.

---

## 0. Orientation (30 seconds)

- Product: a userspace **CoreAudio AudioServerPlugin (HAL plugin)** that turns the NI
  **RigKontrol 3** (USB VID `0x17cc`/PID `0x1940`) into a 2-in/2-out macOS audio device
  on Apple Silicon. It is working and shipping; this is a tuning task, not bring-up.
- Code lives in `/Users/mihai/DEV/rk3-coreaudio` (a git repo, branch `master`).
  `/Users/mihai/DEV/caiaq` is the read-only Linux `snd-usb-caiaq` reference + the RE
  notes (`caiaq/mac_drivers/FINDINGS.md`).
- The two files you will touch: `rk3_engine.{c,h}` (USB + isoc transport, runs on its
  own pthread/CFRunLoop inside coreaudiod) and `plugin/rk3_device.c` (the HAL object
  model + reported latency/clock). Audio format is **S24_3BE**, microframe-based isoc.

### Build / install / measure
```
make driver                 # build build/RigKontrol3.driver
make install-driver         # SUDO; rm -> cp -> codesign IN PLACE -> killall coreaudiod
make driver-log             # os_log stream (subsystem com.mihai.rk3) — counters live here
make iotest
./build/iotest 6 --latency  # GROUND TRUTH: loopback round-trip. Patch RK3 out-L -> in-L
                            # with a cable; reports min/median/mean/max in frames + ms.
```
**You cannot run `install-driver` (needs sudo + restarts coreaudiod) or drive Reaper.**
Make the user run those and report the `--latency` median + the `pb_underruns / pb_overruns
/ in_errs / out_errs` line from `make driver-log`. Always tell them to idle the RK3 and
quit audio apps before installing.

---

## 1. Where latency stands today (the starting line)

Shipped clean config (in `rk3_engine.h` / `plugin/rk3_device.c`):
`RK3_FRAMES_PER_XFER=64` (8 ms IN transfers), `RK3_OUT_FRAMES_PER_XFER=32` (4 ms OUT),
`RK3_N_XFERS=8` (IN), `RK3_N_OUT_XFERS=2` (OUT), `RK3_OUT_CUSHION=512 frames`,
`kSafetyOffsetFrames=256`, OUT phase = 4 bytes.

History (do NOT re-derive — see `HANDOFF.md` Step 5 for the full table):
- Baseline was **81.5 ms** median round-trip. Got to ~30–52 ms by cutting the OUT isoc
  pipeline depth (`N_OUT_XFERS`) and the standing `OUT_CUSHION`.
- **The dominant remaining term is the OUT isoc pipeline:** popped audio isn't heard for
  `N_OUT_XFERS * (OUT_FRAMES_PER_XFER/8) ms` because each finished OUT buffer is
  rescheduled that far ahead of the bus. Cushion is the fine-grain jitter absorber on top.
- Latency was also **unstable run-to-run (30–69 ms)** — an underrun self-heal can balloon
  the OUT standing fill. A **fill servo** was the proposed fix (see §3, option C).

### Three things already proven DEAD or dangerous — do not repeat:
1. **Shrinking `RK3_FRAMES_PER_XFER` (e.g. 64→16) STORMED coreaudiod to 100% CPU and
   required a REBOOT.** Root cause: with the *basic* isoch API, smaller transfers =
   proportionally more completion callbacks/sec on the sandboxed helper CFRunLoop, which
   overloads it. This is the single biggest wall — and §2 is exactly how NI gets around it.
2. **`kSafetyOffsetFrames` is a dead end.** Lowering 256→128 made round-trip *worse* and
   only shifted CoreAudio's scheduling phase, not the physical path. Leave at 256.
3. **A relay/callback that self-resubmits with zero delay storms coreaudiod** (cost a
   reboot during MIDI work). Any new completion path must bound its own re-arm rate.

---

## 2. The key finding from NI's driver (this is your main lever)

We reverse-engineered NI's official Intel kext `NIUSBAudioDriver.kext` 2.8.0
(`caiaq/mac_drivers/FINDINGS.md`; memory `rk3-official-mac-driver-findings`). The latency
machinery: **IN ring = 64 isoc IRPs, OUT ring = 128, and crucially it uses the
`IOUSBLowLatencyIsocFrame` API** (`LowLatency{Read,Write}IsochPipeAsync` +
`LowLatencyCreateBuffer`), not the basic `{Read,Write}IsochPipeAsync` we use today.

**Why this matters — it dissolves wall #1 above.** The low-latency isoch API uses a frame
list in a kernel-shared buffer that the kernel updates *as each microframe completes*,
without firing a user completion callback per transfer. You poll/read frame status from
shared memory and re-arm on a timer, so you can run **much smaller / deeper transfer rings
for low latency WITHOUT multiplying the callback rate** that stormed us. NI runs 64/128
deep precisely because the low-latency API makes that cheap. This is the path to getting
under the current floor that the basic API blocks.

Also confirmed in NI's driver (secondary levers / facts):
- **`setDirectMonitoringMode` = EP1 cmd `0x0e`** is hardware zero-latency input
  monitoring (input routed to output *in the device*, bypassing the host round-trip
  entirely). Not RT latency reduction for the DAW path, but the right answer if the user
  wants near-zero monitoring latency for live guitar. Payload not yet captured — would
  need more RE. Treat as an optional follow-on, confirm with the user before building.
- `get/setHardwareBufferSize` user-client methods in NI's kext are **stubs** — there is
  **no device command that sets a latency/buffer floor.** The floor is purely host-side:
  isoc ring depth + the IOAudioEngine sample buffer. So all real levers are in our engine,
  not in a magic device register. Don't go hunting for one.
- RK3 isoc is microframe mode 2 (matches our `data_alignment=2`); NI builds per-microframe
  sizes from that same mode byte. Our codec/framing already matches — don't touch it.

---

## 3. Suggested plan (ranked; measure after every change)

Work in **small steps, reinstall, and get a fresh `--latency` median + counters from the
user after each one.** Latency here is empirical and run-to-run noisy; one build = one data
point. Keep the shipped config as the known-good fallback (it's committed).

**A. (Biggest win, biggest effort) Port the isoc transport to the low-latency isoch API.**
This is the NI-validated path and the only thing that breaks the FRAMES_PER_XFER storm
wall. Plan: allocate frame buffers with `LowLatencyCreateBuffer`
(`kUSBLowLatencyFrameListBuffer` + `kUSBLowLatencyWriteBuffer`/`ReadBuffer`), submit with
`LowLatency{Read,Write}IsochPipeAsync` passing `updateFrequency` (how often the kernel
writes back frame status), and drive re-arm from a bounded run-loop timer reading frame
status from shared memory instead of one callback per transfer. Then you can safely shrink
transfer size (toward 1–2 ms) and/or run a deeper ring for low latency. **Validate in
isolation first:** the standalone `tools/stream.c` harness owns the device exclusively and
is the right place to prove the low-latency path produces a clean tone + 0 errors before
moving it into the in-coreaudiod engine (where a storm = reboot). Re-read transport
findings 1–7 in `HANDOFF.md` ("hard-won transport findings") before editing — contiguous
frReqCount packing, the 0-length-last-frame rule, and OUT phase 4 all still apply.

**B. (Cheap, do FIRST as a warm-up + data point) Re-sweep the existing knobs.** With the
4 ms OUT transfers + N_OUT_XFERS=2 now shipping, re-measure and try N_OUT_XFERS and
OUT_CUSHION one notch each way to confirm we're still at the clean knee. This is low-risk
(worst case = underrun/silence, never a storm) and re-establishes the baseline number on
the user's current hardware/OS before you invest in (A).

**C. (Stability, complements A/B) Fill servo on the OUT standing fill.** The run-to-run
instability is the self-heal ballooning the OUT ring. Hold the standing fill at a low
setpoint by trimming the reported output clock *rate* via the `Drv_GetZeroTimeStamp` slope
in `plugin/rk3_device.c` (click-free; also absorbs the ~0.01% residual device/host drift).
Telemetry to tune it (`dev_frames` counter + a once/sec `clk:` fill log) was left installed
in the engine — use it, then remove when done. NOTE the drift hypothesis was OVERTURNED:
device clock is only ~0.002% off nominal, so this is about *stability*, not drift — do not
build heavyweight clock sync as a latency fix.

Rough target: the user wants guitar-through-Reaper as low as is clean. Sub-10 ms is the
aspiration; be honest about what measures clean for minutes vs. a lucky single run.

---

## 4. Hazards / rules (these cost reboots — internalize them)

- **Anything that runs inside coreaudiod and busy-loops or over-submits storms the whole
  audio subsystem to 100% CPU and may need a reboot.** Prove new transport paths in
  `tools/stream.c` (exclusive owner, no coreaudiod) before putting them in the engine.
- **Install correctly or the bundle is SIGKILLed:** HAL plugins load ONLY from system
  `/Library/Audio/Plug-Ins/HAL/` (root:wheel), and you MUST `rm -rf` + re-`codesign`
  IN PLACE before restart (never `cp` over an existing install — page-hash desync →
  CODESIGNING kill). `make install-driver` already does this; don't hand-roll it.
- **Wedged USB interface:** if a tool opens the device and exits without releasing, or
  coreaudiod is killed mid-stream, StartIO fails for everyone until a **physical
  unplug/replug.** Idle the device + quit audio apps before each install.
- zsh shell: the user's `log` is shadowed by a function — call `/usr/bin/log` for queries.
  Use `&&`, not `and`.
- The driver is the single USB owner and now also relays **always-on MIDI**; audio layers
  on/off at StartIO/StopIO. Don't break the MIDI relay while refactoring isoc — the relay
  reads EP `0x81` (the same bulk-in pipe AUDIO_PARAMS acks on); see `HANDOFF.md` for the
  quiesce-around-AUDIO_PARAMS rule.

---

## 5. Pointers

- `HANDOFF.md` — full project status: audio bring-up (Step 4), the complete Step 5 latency
  table + lessons, always-on MIDI. The latency history in §1 above is the condensed form.
- `caiaq/mac_drivers/FINDINGS.md` — the NI driver RE write-up (EP1 opcode table, isoc/IRP
  internals, low-latency API usage).
- Memory (`/Users/mihai/.claude/projects/-Users-mihai-DEV-caiaq/memory/`):
  `rk3-coreaudio-project.md`, `rk3-coreaudio-hardware-facts.md`,
  `rk3-official-mac-driver-findings.md`.
- First action suggestion: do §3-B (cheap re-baseline) to get a current number, then start
  §3-A in `tools/stream.c`.
