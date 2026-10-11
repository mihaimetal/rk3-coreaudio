> **Context changed (2026-10).** The premise that userspace cannot go below
> ~14 ms did not hold: the HAL plugin now measures ~8-10 ms loopback with 1 ms
> transfers (README, *Latency tuning*). The DriverKit notes are kept for reference.

# HANDOFF — DriverKit dext path (sub-10 ms round-trip)

Date: 2026-06-22. Author: prior agent. Read this top-to-bottom before touching code.
Companion memory: `[[rk3-coreaudio-project]]`, `[[rk3-coreaudio-hardware-facts]]`,
`[[rk3-official-mac-driver-findings]]` (in `~/.claude/.../memory/`). Prior latency writeups:
`HANDOFF_LATENCY.md`, `HANDOFF.md`.

---

## TL;DR

- The **userspace HAL plugin is feature-complete and shipped**: 2-in/2-out audio, foot
  controls→CoreMIDI ("CON" always-on), LED/7-seg output, master volume+mute, 44.1/48/96/192k.
- **Latency floor reached in userspace = 14.31 ms loopback** (750/s callback rate), ~40% under
  baseline. All small-transfer levers are exhausted: `OUT_FPX` can't go below 16, `updateFrequency`
  is a confirmed no-op on Apple Silicon (IOUSBLib emulates the LL isoch API, copying frame lists
  only AT COMPLETION), and ~1000/s callbacks reboot-storm the machine.
- **Why we can't go lower in userspace:** our isoc completions run on the **sandboxed coreaudiod
  CFRunLoop**, which has a hard callback-rate ceiling (~750/s safe, ~1000/s fatal). NI's original
  Mac driver beat us because it was a **kernel kext** with no such ceiling.
- **Hardware direct monitoring is a DEAD END on the RK3** (confirmed this session — see below). Do
  not revisit it.
- **The only remaining path to sub-10 ms round-trip is a DriverKit dext** (AudioDriverKit +
  USBDriverKit), where isoc runs on the dext's own dispatch queue, not coreaudiod's CFRunLoop.
- **YOUR FIRST JOB IS A FEASIBILITY SPIKE, NOT A REWRITE.** See "Phase 0" below. Two gates can kill
  this before any audio code: (1) DriverKit entitlements, (2) whether USBDriverKit isoc on Apple
  Silicon actually delivers high completion rates without the same ceiling.

---

## Immediate housekeeping (do this first, ~2 min)

The currently *installed* driver is a stale experimental build (it sends two harmless no-op EP1
commands and shows "– –" instead of "CON" during audio). The source tree is already reverted to the
clean known-good state and builds clean. Restore it:

```
cd ~/DEV/rk3-coreaudio
make driver
make install-driver      # idle the RK3 + close audio apps first (reboot-risky install step)
make driver-log          # confirm "engine USB up ... MIDI relay armed"; display shows CON
```

This is the **fallback product** if the DriverKit spike fails. It is committed (latency campaign =
commit `0f19e6f`; the post-DM revert is uncommitted as of this handoff — commit it if you want a
clean baseline, message e.g. "Back out inert RK3 direct-monitoring wiring; keep stall self-heal").

---

## What "direct monitoring is dead" means (so you don't re-chase it)

Confirmed inert on the RK3 this session. `setDirectMonitoringMode` (EP1 0x0e) is a 1-byte command
(kext disasm line 2616), the device ACKs mode 1 after AUDIO_PARAMS, but produces NO monitor audio —
silent both standalone AND under full isoc streaming. The RK3's kext channel table (PID 0x1940) has
only `Output L/R` + `Input 1/2` — **no Monitor channel pair** (the bigger NI boxes that monitor have
`Out: Monitor L/R`). NI's RK3 per-PID init `WRITE_IO [00 40 40 00 00 00]` is just a "– –" display
paint (0x40 = 7-seg middle bar), not monitor setup. The RK3 firmware has no analog/DSP monitor path
behind 0x0e. Monitoring on this device must go through the host → which is exactly why round-trip
latency matters and why DriverKit is the lever.

---

## The goal and the architecture

**Goal:** guitar through Reaper at **sub-10 ms round-trip**, keeping everything the userspace driver
already does (audio + MIDI + LEDs + controls).

**Why DriverKit wins:** In a dext, USBDriverKit isoc completions are serviced on an
`IODispatchQueue` owned by the driver, in the DriverKit runtime — NOT on coreaudiod's shared
sandboxed CFRunLoop. That removes the storm ceiling that caps us at 750/s. Smaller isoc transfers
(more completions/sec) → shorter pipeline → lower latency, without melting the audio subsystem.

**The shape of the solution: ONE dext using two DriverKit families.**
- **AudioDriverKit** (`IOUserAudioDriver`/`IOUserAudioDevice`/`IOUserAudioStream`, macOS 12+):
  presents the CoreAudio device (replaces the entire `plugin/rk3_device.c` HAL plugin). It gives you
  an IO ring buffer that CoreAudio reads/writes; you map your captured/playback samples into it.
- **USBDriverKit** (`IOUSBHostInterface`/`IOUSBHostPipe`): claims the RK3 interface and runs the
  isoc IN/OUT + bulk EP1 traffic (replaces `usb_iokit.c`'s IOUSBLib usage). Isoc via
  `IOUSBHostPipe::AsyncIO` / the isochronous frame-list API in DriverKit.

These can live in a single dext (the audio device driver also owns the USB interface), or two dexts
that communicate — prefer single-dext to avoid IPC. NI's own kext did exactly this: one driver class
owned USB + presented audio + fanned MIDI.

---

## Phase 0 — FEASIBILITY SPIKE (do this before committing to the port)

Do NOT port the engine first. Answer the two kill-questions first, cheaply.

### Gate 1 — Entitlements / can you even load a dext on this machine?
DriverKit dexts need restricted entitlements:
- `com.apple.developer.driverkit` (umbrella)
- `com.apple.developer.driverkit.transport.usb` (USB family)
- `com.apple.developer.driverkit.family.audio` (AudioDriverKit) — **this one in particular is
  tightly gated by Apple.**
- A matching provisioning profile, and the dext embedded in a host app (dexts ship inside an app
  bundle, activated via `OSSystemExtensionRequest`).

Action: determine whether the user's Apple Developer account can obtain these entitlements (the audio
family entitlement historically requires a request/justification to Apple). For LOCAL development you
can sometimes run with `systemextensionsctl developer on` + SIP considerations, but verify on THIS
machine (Apple Silicon, macOS 25.x). **If entitlements are unobtainable, the project stops here** —
say so to the user; the 14.31 ms userspace driver stays the product. Ask the user about their
developer-account status early.

### Gate 2 — Does USBDriverKit isoc actually beat the ceiling on Apple Silicon?
This is the technical crux. Write the SMALLEST possible USBDriverKit dext that:
1. Matches + claims the RK3 (VID 0x17cc / PID 0x1940), sets config + alt setting 1 (same as
   `usb_iokit.c` `rk3_usb_open`).
2. Sends AUDIO_PARAMS (reuse the wire protocol below) and runs the isoc IN pipe (EP 0x82) ONLY —
   capture, discard samples — at the SMALL transfer size we couldn't reach in userspace (e.g. 16- or
   8-µframe transfers → 500–1000+ completions/sec).
3. Counts completions/sec on its dispatch queue and logs it (os_log from a dext).
4. Run it and confirm it sustains the high completion rate with **no system storm / no audio
   subsystem impact** (there's no coreaudiod involvement yet, so this isolates the USB side).

If Gate 2 shows DriverKit isoc sustains ~1000+/s cleanly, the core premise is proven and the full
port is worth it. If it ALSO chokes, DriverKit doesn't help → stop, keep 14.31 ms userspace.

**Only after both gates pass** do you port the audio engine.

---

## Phase 1 — the port (only if Phase 0 passes)

Reuse as much proven logic as possible — the hard-won DSP/protocol facts are transport-agnostic.
**Directly portable (logic unchanged, just the I/O calls swapped):**
- `caiaq_protocol.{h,c}` — EP1 command wire format, AUDIO_PARAMS, bpp computation, device-info
  handshake. Pure byte manipulation over a "send command / read reply" primitive; swap the
  `rk3_bulk_write/read` underneath for `IOUSBHostPipe` calls.
- `codec_mode2.c` — the S24_3BE ⟷ interleaved pack/unpack (mode-2, data_alignment=2). Pure math.
- `rk3_engine.c` transport core — the **cadence mirror** (`pat_*`), the **OUT 4-byte phase**, the
  **bursty 88/0 cadence**, the **SPSC rings**, the **capture-drain / fill-servo** logic. This is the
  crown jewel; it encodes everything we learned about the device's no-feedback OUT clock. It moves
  almost verbatim; only the isoc submit/complete calls (`rk3_read_isoc`/`rk3_write_isoc`/`in_done`/
  `out_done`) change to DriverKit's isoc API, and the run loop becomes an `IODispatchQueue`.

**Replaced wholesale:**
- `plugin/rk3_device.c` (HAL AudioServerPlugin) → AudioDriverKit `IOUserAudioDevice` subclass. The
  servo currently lives in `Drv_GetZeroTimeStamp` (timestamp-slope rate trim to hold OUT-ring fill);
  in AudioDriverKit the equivalent is your timestamp/zero-timestamp generation — port the servo math
  there. Volume/mute controls → AudioDriverKit controls.
- `usb_iokit.c` (IOUSBLib) → USBDriverKit `IOUSBHostInterface`/`IOUSBHostPipe`.

**MIDI / LED / controls:** in userspace these ride EP1 0x81 bulk reads relayed over loopback UDP to a
LaunchAgent helper (because the coreaudiod sandbox blocks MIDIClientCreate). In a dext the
constraints differ — investigate whether a dext can publish CoreMIDI directly, or keep the
helper+relay split (the relay transport changes from UDP to a dext user-client / IOUserClient).
Lower priority than audio; get audio working first.

---

## Wire-protocol facts the dext needs (from memory, condensed)

- Device: VID `0x17cc` PID `0x1940`. Configure, then **SetAlternateInterface(1)** to expose streaming
  pipes. Endpoints: bulk EP **0x01 OUT** / **0x81 IN** (EP1 vendor commands + MIDI/control reports);
  isoc EP **0x82 IN** (capture) / **0x06 OUT** (playback). `data_alignment = 2`, mode-2 codec.
- **AUDIO_PARAMS (EP1 cmd 0x09)** payload = `[rate_code, depth_code, ch_lo, ch_hi, 0x01]`.
  rate_code: 44100→0, 48000→1, 96000→2, 192000→3, 88200→4. depth: 24→2, 16→1. Device ACKs on EP 0x81
  (poll ~1000×1ms). Send rate=0xFF first to stop/flush.
- **bpp** (bytes/microframe) = `caiaq_compute_bpp(rate, data_alignment=2, n_streams)`: 44.1→80,
  48→88, 96→136, 192→232 (all < 512 max-packet).
- **OUT is a no-feedback isoch endpoint.** It needs the **bursty 88/0 cadence** = a delayed
  rate-locked mirror of the device's own IN cadence (NOT a uniform feed — uniform `--const` drifts
  its FIFO → clicks). The engine's `pat_pop_out` staging does this. And the OUT bytestream MUST be
  seeded at a **4-byte phase offset** (`out_pos = 4`) or every sample garbles into a buzz.
- Audio format on the wire = **S24_3BE**, interleaved per `codec_mode2.c`.
- Storm ceiling (userspace, for reference): 375/500/750 per-sec proven safe, ~1000 fatal. The whole
  POINT of DriverKit is that this ceiling shouldn't apply on the dext dispatch queue — Gate 2 proves
  it.

---

## Risks / unknowns (rank-ordered)

1. **Entitlements** (Gate 1) — may be a hard stop for distribution; verify the user's account.
2. **DriverKit isoc rate ceiling** (Gate 2) — the core premise; unproven until measured.
3. **AudioDriverKit servo timing** — porting the GetZeroTimeStamp slope servo to AudioDriverKit's
   timestamp model is non-trivial; the monotonic-timestamp rule that caused reboot-storms in
   userspace (re-pivoting every call) has a DriverKit analog — keep timestamps monotonic.
4. **Single-owner USB** — only one driver can claim the interface. The dext claiming it means the old
   userspace HAL plugin must be UNINSTALLED while the dext is active (they'll fight over the device).
5. **Dev-loop friction** — dext install/activate/debug is much heavier than `make install-driver`
   (system extension approval, reboots, `log stream` for dext logs). Budget for it.

---

## Recommendation to the next agent

Start with **Phase 0, Gate 1** — a 30-minute investigation + a direct question to the user about
their Apple Developer entitlement situation. Do not write audio code until both gates pass. If either
gate fails, the honest answer is: **14.31 ms userspace is the product**, and sub-10 ms is not
reachable on this hardware/OS without resources (entitlements) or a capability (DriverKit isoc rate)
we don't have. The user explicitly wanted lower latency "because the original drivers do it on
supported systems" — the original drivers were kernel kexts on Intel; DriverKit is the modern
equivalent and the only legitimate path, gated by the two questions above.
