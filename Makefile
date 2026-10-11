# rk3-coreaudio — RigKontrol 3 audio driver for macOS (userspace)
#
# Targets:
#   make usbprobe   build the step-1 USB enumeration probe (tools/usbprobe.c)
#   make clean      remove build artifacts

CC      := clang
CFLAGS  := -Wall -Wextra -O2 -fobjc-arc -mmacosx-version-min=12.0
FRAMEWORKS := -framework CoreFoundation -framework IOKit

BUILD := build

PROTO_SRC := usb_iokit.c caiaq_protocol.c
STREAM_SRC := usb_iokit.c caiaq_protocol.c codec_mode2.c

# Sandboxed coreaudiod loads HAL plugins from the SYSTEM dir only (root:wheel),
# never ~/Library — confirmed empirically. install-probe uses sudo.
HAL_DIR := /Library/Audio/Plug-Ins/HAL
PROBE_BUNDLE := $(BUILD)/RK3SandboxProbe.driver
MIDI_PROBE_BUNDLE := $(BUILD)/RK3MidiProbe.driver
DRIVER_BUNDLE := $(BUILD)/RigKontrol3.driver
ENGINE_SRC := usb_iokit.c caiaq_protocol.c codec_mode2.c rk3_engine.c

.PHONY: all usbprobe handshake codectest stream epdump test clean \
        sandboxprobe install-probe uninstall-probe probe-log \
        midiprobe install-midi-probe uninstall-midi-probe midi-probe-log \
        driver install-driver reload-driver uninstall-driver driver-log rk3midi rk3led \
        rk3monitor install uninstall enginetest enginetest-ll \
        midihelper install-midihelper uninstall-midihelper midihelper-log

all: usbprobe handshake codectest stream epdump

usbprobe: $(BUILD)/usbprobe
handshake: $(BUILD)/handshake
codectest: $(BUILD)/codectest
stream: $(BUILD)/stream
epdump: $(BUILD)/epdump

$(BUILD)/stream: tools/stream.c $(STREAM_SRC) usb_iokit.h caiaq_protocol.h codec_mode2.h | $(BUILD)
	$(CC) $(CFLAGS) tools/stream.c $(STREAM_SRC) $(FRAMEWORKS) -o $@
	@echo "Built $@ — run ./build/stream <seconds> [--play] with the RK3 plugged in."

# Unit tests — no hardware required.
test: $(BUILD)/codectest $(BUILD)/miditest
	./$(BUILD)/codectest
	./$(BUILD)/miditest

$(BUILD)/miditest: tools/miditest.c rk3_midi.c rk3_midi.h caiaq_protocol.h | $(BUILD)
	$(CC) $(CFLAGS) -Wno-unused-function tools/miditest.c \
		$(FRAMEWORKS) -framework CoreMIDI -o $@

$(BUILD)/codectest: tools/codectest.c codec_mode2.c codec_mode2.h | $(BUILD)
	$(CC) $(CFLAGS) tools/codectest.c codec_mode2.c -o $@

$(BUILD)/usbprobe: tools/usbprobe.c | $(BUILD)
	$(CC) $(CFLAGS) $< $(FRAMEWORKS) -o $@
	@echo "Built $@ — run it with the RigKontrol 3 plugged in."

$(BUILD)/handshake: tools/handshake.c $(PROTO_SRC) usb_iokit.h caiaq_protocol.h | $(BUILD)
	$(CC) $(CFLAGS) tools/handshake.c $(PROTO_SRC) $(FRAMEWORKS) -o $@
	@echo "Built $@ — run it with the RigKontrol 3 plugged in."

# Hardware direct-monitoring (EP1 0x0e) sweep tool — discovers the mode byte.
# Cannot run while the audio driver is streaming (shared USB interface).
rk3monitor: $(BUILD)/rk3monitor
$(BUILD)/rk3monitor: tools/rk3monitor.c $(PROTO_SRC) usb_iokit.h caiaq_protocol.h | $(BUILD)
	$(CC) $(CFLAGS) tools/rk3monitor.c $(PROTO_SRC) $(FRAMEWORKS) -o $@
	@echo "Built $@ — run ./build/rk3monitor with the RK3 plugged in (audio idle)."

# Standalone "RK3 controls -> MIDI" bridge (CoreMIDI virtual source).
# Cannot run while the audio driver is streaming (shared USB interface).
rk3midi: $(BUILD)/rk3midi
$(BUILD)/rk3midi: tools/rk3midi.c $(PROTO_SRC) rk3_midi.c \
		usb_iokit.h caiaq_protocol.h rk3_midi.h | $(BUILD)
	$(CC) $(CFLAGS) tools/rk3midi.c $(PROTO_SRC) rk3_midi.c \
		$(FRAMEWORKS) -framework CoreMIDI -o $@
	@echo "Built $@ — run ./build/rk3midi with the RK3 plugged in (audio idle)."

# Standalone RK3 LED / 7-segment display driver (sends EP1_CMD_WRITE_IO).
# Hardware-validation tool for the reverse-engineered control_state bit map.
# Cannot run while the audio driver is streaming (shared USB interface).
rk3led: $(BUILD)/rk3led
$(BUILD)/rk3led: tools/rk3led.c $(PROTO_SRC) \
		usb_iokit.h caiaq_protocol.h rk3_midi.h | $(BUILD)
	$(CC) $(CFLAGS) tools/rk3led.c $(PROTO_SRC) $(FRAMEWORKS) -o $@
	@echo "Built $@ — run ./build/rk3led --walk with the RK3 plugged in (audio idle)."

# User-space MIDI helper: owns the "RigKontrol 3" CoreMIDI source and turns the
# foot-control reports the audio engine relays over loopback UDP into MIDI. Runs
# alongside the audio driver (touches no USB). No proto/USB sources needed —
# rk3_midi.c uses caiaq_protocol.h only for the EP1_CMD_* constants (macros).
#   make midihelper            build ./build/rk3midihelper (run manually to test)
#   make install-midihelper    install to /usr/local/bin + load LaunchAgent
#   make uninstall-midihelper  unload LaunchAgent + remove
#   make midihelper-log        tail the helper's log
HELPER_PLIST := launchd/com.mihai.rk3.midihelper.plist
HELPER_AGENT := $(HOME)/Library/LaunchAgents/com.mihai.rk3.midihelper.plist

midihelper: $(BUILD)/rk3midihelper
$(BUILD)/rk3midihelper: tools/rk3midihelper.c rk3_midi.c \
		rk3_midi.h caiaq_protocol.h | $(BUILD)
	$(CC) $(CFLAGS) tools/rk3midihelper.c rk3_midi.c \
		$(FRAMEWORKS) -framework CoreMIDI -o $@
	@echo "Built $@ — run ./build/rk3midihelper while the RK3 audio driver streams."

# A LaunchAgent must be (un)loaded in the invoking user's GUI domain, NOT the
# root/system domain. If this target is run under sudo, launchctl run as root
# would target the wrong domain ("Load failed: 5") — so resolve the real user's
# uid via $SUDO_UID (set under sudo) and fall back to id -u, and use the modern
# bootstrap/bootout which take an explicit gui/<uid> domain. The plist is also
# chowned back to that user so a sudo run doesn't leave it root-owned.
install-midihelper: $(BUILD)/rk3midihelper
	sudo cp $(BUILD)/rk3midihelper /usr/local/bin/rk3midihelper
	sudo cp tools/rk3-reload-config tools/rk3-calibrate tools/rk3-tune /usr/local/bin/
	sudo chmod +x /usr/local/bin/rk3-reload-config /usr/local/bin/rk3-calibrate \
		/usr/local/bin/rk3-tune
	mkdir -p $(HOME)/Library/LaunchAgents
	cp $(HELPER_PLIST) $(HELPER_AGENT)
	@uid=$${SUDO_UID:-$$(id -u)}; gid=$${SUDO_GID:-$$(id -g)}; \
	chown $$uid:$$gid $(HELPER_AGENT); \
	launchctl bootout gui/$$uid $(HELPER_AGENT) 2>/dev/null || true; \
	launchctl bootstrap gui/$$uid $(HELPER_AGENT) && \
	  echo "Loaded LaunchAgent in gui/$$uid. 'RigKontrol 3' MIDI source should now be present." || \
	  echo "launchctl bootstrap failed; run WITHOUT sudo: make install-midihelper"

uninstall-midihelper:
	@uid=$${SUDO_UID:-$$(id -u)}; \
	launchctl bootout gui/$$uid $(HELPER_AGENT) 2>/dev/null || true
	rm -f $(HELPER_AGENT)
	sudo rm -f /usr/local/bin/rk3midihelper
	sudo rm -f /usr/local/bin/rk3-reload-config /usr/local/bin/rk3-calibrate \
		/usr/local/bin/rk3-tune
	@echo "Uninstalled MIDI helper."

midihelper-log:
	@tail -n 50 /tmp/rk3midihelper.log 2>/dev/null || echo "(no log yet)"

# Watch the "RigKontrol 3" MIDI source and summarise a pedal's CC stream
# (message rate, distinct values, 7-bit vs 14-bit). ./build/midimon [secs] [cc]
midimon: $(BUILD)/midimon
$(BUILD)/midimon: tools/midimon.c | $(BUILD)
	$(CC) $(CFLAGS) tools/midimon.c $(FRAMEWORKS) -framework CoreMIDI -o $@

# Config-descriptor / isoc endpoint sync-type dump (no audio).
$(BUILD)/epdump: tools/epdump.c usb_iokit.c usb_iokit.h | $(BUILD)
	$(CC) $(CFLAGS) tools/epdump.c usb_iokit.c $(FRAMEWORKS) -o $@

# Runs the real engine in an ordinary process (device must be free: uninstall the
# driver first). Proves a transport geometry outside coreaudiod; see the file header.
enginetest: $(BUILD)/enginetest
$(BUILD)/enginetest: tools/enginetest.c $(ENGINE_SRC) rk3_engine.h usb_iokit.h \
		caiaq_protocol.h codec_mode2.h | $(BUILD)
	$(CC) $(CFLAGS) -DRK3_TEST_HOOKS tools/enginetest.c $(ENGINE_SRC) \
		$(FRAMEWORKS) -o $@

# Same harness on the LowLatency isoch API (kernel-shared, pre-wired buffers).
enginetest-ll: $(BUILD)/enginetest-ll
$(BUILD)/enginetest-ll: tools/enginetest.c $(ENGINE_SRC) rk3_engine.h usb_iokit.h \
		caiaq_protocol.h codec_mode2.h | $(BUILD)
	$(CC) $(CFLAGS) -DRK3_TEST_HOOKS -DRK3_USE_LOWLAT=1 tools/enginetest.c \
		$(ENGINE_SRC) $(FRAMEWORKS) -o $@

# Exercises the installed RigKontrol3.driver (AUHAL bound to our device by UID).
iotest: $(BUILD)/iotest
$(BUILD)/iotest: tools/iotest.c | $(BUILD)
	$(CC) $(CFLAGS) tools/iotest.c -framework CoreFoundation \
		-framework CoreAudio -framework AudioToolbox -o $@

# ---- Step 4 de-risk: sandboxed-coreaudiod USB probe (HAL plugin bundle) ----
#
# Builds an AudioServerPlugin .driver that, when loaded by coreaudiod, attempts
# to open the RK3 USB interface and logs the result via os_log. See plugin/.
#   make install-probe   build + copy to ~/Library/Audio/Plug-Ins/HAL + restart coreaudiod
#   make probe-log       print the probe's os_log output from the last 2 min
#   make uninstall-probe remove it + restart coreaudiod
sandboxprobe: $(PROBE_BUNDLE)

$(PROBE_BUNDLE): plugin/sandbox_probe.c plugin/Info.plist | $(BUILD)
	rm -rf $(PROBE_BUNDLE)
	mkdir -p $(PROBE_BUNDLE)/Contents/MacOS
	cp plugin/Info.plist $(PROBE_BUNDLE)/Contents/Info.plist
	$(CC) $(CFLAGS) -bundle plugin/sandbox_probe.c \
		-framework CoreFoundation -framework CoreAudio -framework IOKit \
		-o $(PROBE_BUNDLE)/Contents/MacOS/RK3SandboxProbe
	codesign --force --sign - $(PROBE_BUNDLE) 2>/dev/null || true
	@echo "Built $(PROBE_BUNDLE)"

install-probe: $(PROBE_BUNDLE)
	sudo rm -rf $(HAL_DIR)/RK3SandboxProbe.driver
	sudo cp -R $(PROBE_BUNDLE) $(HAL_DIR)/
	@echo "Installed to $(HAL_DIR). Restarting coreaudiod..."
	sudo killall coreaudiod || true
	@echo "Wait ~3s, then: make probe-log"

uninstall-probe:
	sudo rm -rf $(HAL_DIR)/RK3SandboxProbe.driver
	sudo killall coreaudiod || true
	@echo "Uninstalled."

probe-log:
	log show --last 3m --predicate 'subsystem == "com.mihai.rk3.probe"' --info --debug

# ---- Phase-2 de-risk: can sandboxed coreaudiod publish a CoreMIDI source? ----
#   make install-midi-probe   build + copy to HAL dir + restart coreaudiod
#   make midi-probe-log       print the probe's os_log output
#   make uninstall-midi-probe remove it + restart coreaudiod
midiprobe: $(MIDI_PROBE_BUNDLE)

$(MIDI_PROBE_BUNDLE): plugin/midi_probe.c plugin/midi_probe.plist | $(BUILD)
	rm -rf $(MIDI_PROBE_BUNDLE)
	mkdir -p $(MIDI_PROBE_BUNDLE)/Contents/MacOS
	cp plugin/midi_probe.plist $(MIDI_PROBE_BUNDLE)/Contents/Info.plist
	$(CC) $(CFLAGS) -bundle plugin/midi_probe.c \
		-framework CoreFoundation -framework CoreAudio \
		-o $(MIDI_PROBE_BUNDLE)/Contents/MacOS/RK3MidiProbe
	codesign --force --sign - $(MIDI_PROBE_BUNDLE) 2>/dev/null || true
	@echo "Built $(MIDI_PROBE_BUNDLE)"

install-midi-probe: $(MIDI_PROBE_BUNDLE)
	sudo rm -rf $(HAL_DIR)/RK3MidiProbe.driver
	sudo cp -R $(MIDI_PROBE_BUNDLE) $(HAL_DIR)/
	sudo codesign --force --sign - $(HAL_DIR)/RK3MidiProbe.driver
	@echo "Installed to $(HAL_DIR). Restarting coreaudiod..."
	sudo killall coreaudiod || true
	@echo "Wait ~3s, then: make midi-probe-log"

uninstall-midi-probe:
	sudo rm -rf $(HAL_DIR)/RK3MidiProbe.driver
	sudo killall coreaudiod || true
	@echo "Uninstalled."

midi-probe-log:
	log show --last 3m --predicate 'subsystem == "com.mihai.rk3.midiprobe"' --info --debug

# ---- Step 4: the real RigKontrol 3 AudioServerPlugin device ----
#   make install-driver   build + copy to /Library/Audio/Plug-Ins/HAL + restart coreaudiod
#   make driver-log       print the driver's os_log output
#   make uninstall-driver remove it + restart coreaudiod
driver: $(DRIVER_BUNDLE)

$(DRIVER_BUNDLE): plugin/rk3_device.c plugin/rk3_device.plist $(ENGINE_SRC) \
		rk3_engine.h usb_iokit.h caiaq_protocol.h codec_mode2.h | $(BUILD)
	rm -rf $(DRIVER_BUNDLE)
	mkdir -p $(DRIVER_BUNDLE)/Contents/MacOS
	cp plugin/rk3_device.plist $(DRIVER_BUNDLE)/Contents/Info.plist
	$(CC) $(CFLAGS) -bundle plugin/rk3_device.c $(ENGINE_SRC) \
		-framework CoreFoundation -framework CoreAudio -framework IOKit \
		-o $(DRIVER_BUNDLE)/Contents/MacOS/RigKontrol3
	codesign --force --sign - $(DRIVER_BUNDLE) 2>/dev/null || true
	@echo "Built $(DRIVER_BUNDLE)"

install-driver: $(DRIVER_BUNDLE)
	sudo rm -rf $(HAL_DIR)/RigKontrol3.driver
	sudo cp -R $(DRIVER_BUNDLE) $(HAL_DIR)/
	# Re-sign in place so the seal matches the exact installed bytes; without a
	# clean rm + this, a copy-over-existing leaves stale pages and the sandboxed
	# Core-Audio-Driver-Service helper is SIGKILLed at dlopen (Code Signature
	# Invalid / Invalid Page) even though `codesign -vv` reports "valid on disk".
	sudo codesign --force --sign - $(HAL_DIR)/RigKontrol3.driver
	@echo "Installed to $(HAL_DIR). Restarting coreaudiod..."
	sudo killall coreaudiod || true
	@echo "Wait ~3s, then check Audio MIDI Setup / 'make driver-log'."

# Replace the installed driver WITHOUT restarting coreaudiod: on macOS versions
# that host each HAL plugin in its own "Core Audio Driver (<name>.driver)" process,
# killing just that process makes coreaudiod relaunch it from the new bundle. Use
# this when a full coreaudiod restart is disruptive (every audio client on the
# machine re-enumerates). If the device does not come back within ~10 s, fall
# back to `sudo killall coreaudiod`.
reload-driver: $(DRIVER_BUNDLE)
	sudo rm -rf $(HAL_DIR)/RigKontrol3.driver
	sudo cp -R $(DRIVER_BUNDLE) $(HAL_DIR)/
	sudo codesign --force --sign - $(HAL_DIR)/RigKontrol3.driver
	# SIGKILL: the driver host ignores a plain SIGTERM (it kept running the old
	# code after the bundle was replaced underneath it).
	sudo pkill -9 -f 'Core Audio Driver \(RigKontrol3' || true
	@echo "Driver process restarted; coreaudiod left running."

uninstall-driver:
	sudo rm -rf $(HAL_DIR)/RigKontrol3.driver
	sudo killall coreaudiod || true
	@echo "Uninstalled."

# Convenience: build + install BOTH components (audio driver + MIDI helper).
# Helper first, so the coreaudiod restart in install-driver happens last. Works
# with or without a sudo prefix (both sub-targets resolve the user domain).
install: install-midihelper install-driver
	@echo "Driver + MIDI helper installed."

uninstall: uninstall-midihelper uninstall-driver
	@echo "Driver + MIDI helper removed."

driver-log:
	log show --last 3m --predicate 'subsystem == "com.mihai.rk3"' --info --debug

$(BUILD):
	mkdir -p $(BUILD)

clean:
	rm -rf $(BUILD)
