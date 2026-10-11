# rk3-coreaudio

A userspace **macOS audio + MIDI driver for the Native Instruments RigKontrol 3**
(USB VID `0x17cc` / PID `0x1940`), built for Apple Silicon.

<img width="900" height="687" alt="image" src="https://github.com/user-attachments/assets/88b9b720-f3f7-4763-ba35-023e423bfd19" />


NI never shipped an Apple-Silicon driver for the RigKontrol 3 — the official
package is a 32-bit-era kext that no longer loads on modern macOS. This project
reimplements the device protocol (originally from the Linux `snd-usb-caiaq`
kernel driver) as a modern **CoreAudio AudioServer plugin** plus a small CoreMIDI
helper, so the RK3 works as a normal sound card and MIDI foot controller again.

What you get:

- **2-in / 2-out USB audio** at 44.1 / 48 / 96 / 192 kHz, switchable live in
  Audio MIDI Setup (all four verified with an analog loopback). Appears as a standard CoreAudio device usable by Reaper,
  the system, or any audio app.
- **Round-trip latency** — ~10 ms analog loopback at 48 kHz / 64-frame buffer
  and ~8 ms at 96 / 192 kHz (was ~12–13 ms at 48 kHz), using 1 ms USB transfers
  and margins that adapt to the host. See *Latency tuning* below.
- **MIDI foot controls** — the 9 foot buttons and the pedals are published as a
  CoreMIDI source named **"RigKontrol 3"** (buttons → CC20–28; built-in treadle →
  CC11; the two rear expression-pedal jacks → CC12 / CC13). Always on; works with
  or without audio streaming. The display shows `Con`. The mapping is
  **configurable** without a UI — see *MIDI mapping config* below.
- **Tempo flash** — the helper also exposes a CoreMIDI *destination* named
  **"RigKontrol 3"**. Send it MIDI Beat Clock from your DAW and the display's
  decimal point blinks on the beat (`Con` → `Con.`), aligned to the transport's
  Start/Continue and cleared on Stop.
- **LED / 7-segment control** over a loopback socket (see `tools/rk3led`).
- **Power-on animation** — when the driver claims the device, `rig kontrol 3`
  scrolls across the display with an LED chase, a counter races from 000 to 999
  with the LEDs as a bar graph, a segment spinner runs, and it settles on `Con`.

> **Unofficial / unaffiliated.** "RigKontrol" and "Native Instruments" are
> trademarks of Native Instruments GmbH. This project is an independent,
> community-built driver and is **not** produced, endorsed by, or affiliated
> with Native Instruments. Use at your own risk.

## Requirements

- Apple Silicon Mac (built with `-mmacosx-version-min=12.0`; macOS 12 Monterey or
  later).
- A RigKontrol 3 unit.
- For building from source: the Xcode Command Line Tools (`xcode-select --install`).

## Install from a prebuilt release

> The driver bundle is **ad-hoc signed**, not notarized. macOS quarantines files
> downloaded from the internet, and sandboxed `coreaudiod` refuses to load a
> quarantined ad-hoc bundle — so you must strip the quarantine attribute. The
> `install.sh` script does this for you.

1. Download `rk3-coreaudio-macos.zip` from the
   [Releases](../../releases) page and unzip it.
2. In Terminal, from the unzipped folder:

   ```sh
   ./install.sh
   ```

   It will (with `sudo`, so you'll be prompted for your password):
   - strip the `com.apple.quarantine` attribute,
   - copy `RigKontrol3.driver` to `/Library/Audio/Plug-Ins/HAL/`,
   - re-sign it in place (required — see note below),
   - install the MIDI helper and the `rk3-*` companion scripts to
     `/usr/local/bin` + a per-user LaunchAgent,
   - restart `coreaudiod` on a first install, or only the driver's own host
     process on an upgrade.

3. Plug in the RigKontrol 3. It should appear in **Audio MIDI Setup** as a
   2-in/2-out device, the display should read `CON`, and a **"RigKontrol 3"**
   MIDI source should be available to your DAW.

To remove everything:

```sh
./uninstall.sh
```

## Build from source

```sh
git clone <this repo>
cd rk3-coreaudio

make test            # codec unit test, no hardware needed
make driver          # build build/RigKontrol3.driver
sudo make install-driver
make midihelper && sudo make install-midihelper   # MIDI foot controls
```

Useful targets (see the `Makefile` for the full list):

| Target | What it does |
| --- | --- |
| `make driver` / `sudo make install-driver` | build / install the audio driver |
| `make midihelper` / `sudo make install-midihelper` | build / install the CoreMIDI helper |
| `sudo make reload-driver` | install the driver and restart only its own host process (no `coreaudiod` restart) |
| `make uninstall-driver`, `uninstall-midihelper` | remove them |
| `make enginetest` | build the engine test harness (runs the engine outside `coreaudiod`; needs the driver uninstalled) |
| `make driver-log`, `midihelper-log` | tail the live os_log / helper log |
| `make all` | build the hardware probe/diagnostic tools |
| `make iotest` | build the loopback audio test tool |
| `make test` | run the codec and MIDI-scaling unit tests (no hardware) |

## MIDI mapping config

The built-in mapping (buttons → CC20–28, treadle → CC11, rear jacks → CC12/13,
all momentary) works out of the box. To change it, drop a config file at
`~/.config/rk3/midi.conf` — the helper reads it at startup and again whenever you
run `rk3-reload-config`, so edits apply live without a restart:

```sh
mkdir -p ~/.config/rk3
cp docs/midi.conf.example ~/.config/rk3/midi.conf
$EDITOR ~/.config/rk3/midi.conf
rk3-reload-config
```

A missing file is fine — everything falls back to the built-in default, and bad
lines are logged (`make midihelper-log`) and skipped. The file lets you set:

- **Output channel** (`channel 1..16`).
- **Per-button behaviour** by the *printed* P1..P9 label (firmware bit-scramble is
  handled for you): `momentary` (default), `toggle` (press flips a latch),
  `hold` (momentary CC plus a second CC when held past `time` ms), `tap` (quick
  tap vs hold fire different CCs, mutually exclusive), or `off`.
- **Per-pedal mapping** for `treadle` / `exp1` / `exp2`: continuous `cc`, raw-ADC
  `calibMin`/`calibMax`, and an optional derived on/off `switch` CC with
  `son`/`soff` hysteresis thresholds. The pedals are read as 10-bit values
  (0–1023; the built-in treadle only swings about 40–270) and sent as ordinary
  7-bit CCs by default. Add `hires` to a pedal line to send 14-bit CC instead
  (MSB on `cc`, which must be 0–31, LSB on `cc`+32): about 230 steps for the
  treadle and 1024 for the rear jacks, if the receiving software reads 14-bit
  CC.

Calibrate a pedal's travel with `rk3-calibrate <1|2|3>` (1 = treadle, 2 = EXP1,
3 = EXP2): it asks the running helper to capture the raw min/max while you sweep
the pedal, then writes `calibMin`/`calibMax` into the config and applies them
live. See `docs/midi.conf.example` for the full annotated syntax.

(`rk3-reload-config` and `rk3-calibrate` are installed to `/usr/local/bin` by
`make install-midihelper`.)

## Latency tuning

The driver ships with 1 ms isochronous transfers in both directions (2000 USB
completions per second). Measured with `./build/iotest 22 --latency` (cable from
an RK3 output to an RK3 input) on macOS 26 / Apple Silicon, idle machine, settled
round trip in ms:

| Host buffer | 44.1 kHz | 48 kHz | 96 kHz | 192 kHz |
| --- | --- | --- | --- | --- |
| 64 frames | 10.3 | 10.0 | 8.0 | 8.4 |
| 128 frames | 10.5 | 11.1 | 8.6 | 7.8 |
| 256 frames | 14.6 | 13.9 | 9.8 | 9.9 |
| 512 frames | 20.4 | 18.6 | 12.3 | 10.5 |

No underruns, rejected-packet mutes or handshake retries in any of the 16 runs.
About half of them include one extra output transfer in flight (+1 ms), added
by the driver after a rejected USB submit — see below.

Those are the driver's tightest margins, measured against an idle test client.
Under a loaded DAW the driver widens them by itself: a playback or capture
underrun adds about a millisecond (plus the shortfall) to that ring's margin, a
rejected USB submit adds one transfer to the output pipeline, and the extra is
given back slowly once the stream has been quiet. Expect the round trip to
settle a few milliseconds higher in a heavy session; the current extras show as
`margin=+out/+capture` in the `clk:` log line.

The small transfers cost CPU in the driver's own process (roughly 20–40 % of one
core while streaming, versus ~14 % with the previous 4 ms / 2 ms transfers).
`rk3-tune` (in `tools/`, installed to `/usr/local/bin`) changes the geometry and the ring margins on the running driver
— it takes effect the next time an app starts audio, and is not persistent:

```sh
tools/rk3-tune 16 8 3        # 2 ms in / 1 ms out: ~1 ms more latency, less CPU
tools/rk3-tune 32 16 2       # the previous geometry
tools/rk3-tune 8 8 3 48 48   # tighter ring margins (may click under load)
```

Arguments are `<in_fpx> <out_fpx> <n_out> [out-ring trough] [capture margin]`
(microframes per transfer, transfers in flight, frames at 48 kHz). `make
driver-log` shows a `clk:` line per second with the ring troughs, underrun
counts and USB error counters; a `BREAKER TRIPPED` line means the engine stopped
a runaway completion loop and will restart the stream by itself.

To try a geometry outside `coreaudiod` first, uninstall the driver and run
`make enginetest && ./build/enginetest 20 --in-fpx 8 --out-fpx 8 --n-out 3`.

### Why the in-place re-sign matters

A HAL plugin is loaded by a **sandboxed** `coreaudiod` helper. If you copy a new
bundle *over* an existing one, stale code pages remain and the helper is
`SIGKILL`ed at `dlopen` with a code-signature/invalid-page error — even though
`codesign -vv` reports "valid on disk". The install path always does
`rm -rf` → copy → `codesign --force --sign -` **in place** to avoid this. Keep
that ordering if you script your own install.

## Troubleshooting

- **Device doesn't appear / no audio:** `make driver-log` and look for USB claim
  or codesign errors. Unplug/replug the RK3 if the interface got wedged.
- **`coreaudiod` pegged at 100% CPU:** quit audio apps and reboot; never
  `killall coreaudiod` or replug the device mid-stream while it's wedged. On
  macOS 26 each HAL plugin runs in its own `Core Audio Driver (…)` process, so
  check which process is actually busy: a restart of `coreaudiod` can also set
  off a re-enumeration storm among the machine's other audio clients that has
  nothing to do with this driver (seen with the driver uninstalled).
  `sudo make reload-driver` updates the driver without restarting `coreaudiod`.
- **Capture is silent after switching sample rate (older builds):** fixed — the
  first packets after a rate change carry bad check bytes and used to mute
  capture for the whole session. `chk_errs` in the `clk:` log line counts any
  packet muted for that reason after start-up.
- **No MIDI:** confirm the helper is running (`make midihelper-log`), and that
  the "RigKontrol 3" source is enabled in your DAW. MIDI works with or without
  an active audio stream.

## How it works

- `usb_iokit.c`, `caiaq_protocol.c`, `codec_mode2.c` — USB transport (IOUSBLib),
  the caiaq vendor protocol, and the mode-2 sample codec.
- `rk3_engine.c` — the isochronous audio engine (capture + free-running output
  rings, the OUT 4-byte phase seed, the fill servo) plus the MIDI/LED relay.
- `plugin/rk3_device.c` — the AudioServerPlugin (device, streams, volume/mute,
  multi-rate, clock).
- `rk3_midi.c` + `tools/rk3midihelper.c` — decode foot controls → CC and publish
  the CoreMIDI source from a normal-user LaunchAgent (the sandboxed driver
  cannot publish CoreMIDI itself). The helper also publishes the matching
  destination and turns incoming MIDI Beat Clock into the display's beat flash,
  which it sends to the engine over the same loopback LED socket.

To use the tempo flash, enable the **"RigKontrol 3"** output in your DAW and turn
on its "send MIDI clock" option (in Reaper: *Preferences → MIDI Devices*), then
start playback.

Design notes and the reverse-engineering history are in [`docs/`](docs/).

## License

GPL-2.0. This driver reimplements protocol logic derived from the
GPL-2.0-licensed Linux `snd-usb-caiaq` driver; see [LICENSE](LICENSE).
