#!/bin/sh
# Install the prebuilt rk3-coreaudio driver + MIDI helper from a release zip.
# Run from the unzipped release folder:  ./install.sh
set -eu

HAL_DIR="/Library/Audio/Plug-Ins/HAL"
DRIVER="RigKontrol3.driver"
HELPER_BIN="rk3midihelper"
AGENT_PLIST="com.mihai.rk3.midihelper.plist"
AGENT_DST="$HOME/Library/LaunchAgents/$AGENT_PLIST"

cd "$(dirname "$0")"

if [ ! -d "$DRIVER" ]; then
	echo "error: $DRIVER not found next to this script." >&2
	exit 1
fi

echo "==> Removing quarantine attribute (downloaded files)"
xattr -dr com.apple.quarantine "$DRIVER" 2>/dev/null || true
[ -f "$HELPER_BIN" ] && xattr -d com.apple.quarantine "$HELPER_BIN" 2>/dev/null || true

echo "==> Installing $DRIVER to $HAL_DIR (sudo)"
sudo rm -rf "$HAL_DIR/$DRIVER"
sudo cp -R "$DRIVER" "$HAL_DIR/"
# Re-sign in place: a copy-over leaves stale pages and the sandboxed coreaudiod
# helper is SIGKILLed at dlopen even though codesign reports "valid on disk".
sudo codesign --force --sign - "$HAL_DIR/$DRIVER"

if [ -f "$HELPER_BIN" ]; then
	echo "==> Installing MIDI helper + LaunchAgent"
	sudo mkdir -p /usr/local/bin
	sudo cp "$HELPER_BIN" /usr/local/bin/rk3midihelper
	# companion scripts: reload the MIDI mapping, calibrate a pedal, tune latency
	for t in rk3-reload-config rk3-calibrate rk3-tune; do
		if [ -f "$t" ]; then
			sudo cp "$t" "/usr/local/bin/$t"
			sudo chmod +x "/usr/local/bin/$t"
		fi
	done
	mkdir -p "$HOME/Library/LaunchAgents"
	cp "$AGENT_PLIST" "$AGENT_DST"
	# A LaunchAgent loads in the user's GUI domain, not root's. Resolve the real
	# user's uid (SUDO_UID if this script was run with sudo, else our own) and
	# use bootstrap/bootout with an explicit gui/<uid> domain so it works either
	# way; chown the plist back so a sudo run doesn't leave it root-owned.
	uid="${SUDO_UID:-$(id -u)}"
	gid="${SUDO_GID:-$(id -g)}"
	chown "$uid:$gid" "$AGENT_DST" 2>/dev/null || true
	launchctl bootout "gui/$uid" "$AGENT_DST" 2>/dev/null || true
	launchctl bootstrap "gui/$uid" "$AGENT_DST" || \
		echo "warning: could not load the LaunchAgent; load it as your user:" \
		     "launchctl bootstrap gui/$uid $AGENT_DST"
fi

# An UPGRADE only needs the driver's own host process restarted: recent macOS
# runs each HAL plugin in a "Core Audio Driver (<name>.driver)" process, and
# coreaudiod relaunches it from the new bundle when it is killed (SIGKILL — it
# ignores SIGTERM). That avoids restarting coreaudiod, which makes every audio
# client on the machine re-enumerate. A FIRST install has no such process yet, so
# coreaudiod itself must restart to discover the plugin.
if pgrep -f 'Core Audio Driver \(RigKontrol3' >/dev/null 2>&1; then
	echo "==> Restarting the RigKontrol 3 driver process (coreaudiod keeps running)"
	sudo pkill -9 -f 'Core Audio Driver \(RigKontrol3' || true
else
	echo "==> Restarting coreaudiod (first install)"
	sudo killall coreaudiod 2>/dev/null || true
fi

echo
echo "Done. Plug in the RigKontrol 3 and check Audio MIDI Setup."
echo "The 'RigKontrol 3' MIDI source should also be available to your DAW."
