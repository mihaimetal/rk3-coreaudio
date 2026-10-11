#!/bin/sh
# Remove the rk3-coreaudio driver + MIDI helper.
set -eu

HAL_DIR="/Library/Audio/Plug-Ins/HAL"
AGENT_DST="$HOME/Library/LaunchAgents/com.mihai.rk3.midihelper.plist"

echo "==> Unloading + removing MIDI helper"
uid="${SUDO_UID:-$(id -u)}"
launchctl bootout "gui/$uid" "$AGENT_DST" 2>/dev/null || true
rm -f "$AGENT_DST"
sudo rm -f /usr/local/bin/rk3midihelper /usr/local/bin/rk3-reload-config \
	/usr/local/bin/rk3-calibrate /usr/local/bin/rk3-tune

echo "==> Removing driver (sudo)"
sudo rm -rf "$HAL_DIR/RigKontrol3.driver"

echo "==> Restarting coreaudiod"
sudo killall coreaudiod 2>/dev/null || true

echo "Done."
