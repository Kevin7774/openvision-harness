#!/bin/bash
# Build + install + launch the XR1 external-camera recorder ON THE MAC.
# Run on 192.168.123.138 (user `apple`).  Pushed and invoked automatically by
# the robot side: python3 scripts/xr1_cam.py install
#
# The Mac has no ffmpeg and no Homebrew, but it does have the Command Line
# Tools, so the recorder is a ~400-line Swift/AVFoundation binary compiled here.
#
# It MUST end up inside an .app bundle:
#   * Info.plist's NSCameraUsageDescription is required or macOS kills the
#     process on first camera touch;
#   * TCC keys the camera grant to the bundle, and the consent prompt can only
#     be answered inside the GUI login session -- which is why the last step is
#     `open -a`, not a direct exec.  A binary spawned by sshd can never get the
#     camera.
set -u

ROOT="${XR1REC_ROOT:-$HOME/xr1rec}"
APP="$ROOT/XR1Rec.app"
SRC="$ROOT/src"
DEVICE="${XR1REC_DEVICE:-FHD C3}"     # external UVC camera; "" = auto-pick external
FPS="${XR1REC_FPS:-30}"
PRESET="${XR1REC_PRESET:-1080}"

mkdir -p "$ROOT" "$SRC" "$ROOT/ctl" "$ROOT/clips"
cd "$ROOT" || exit 1

step() { printf '\n== %s\n' "$*"; }

step "toolchain"
command -v swiftc >/dev/null || { echo "FATAL: swiftc missing (install Command Line Tools)"; exit 2; }
swiftc --version | head -1

# XR1REC_NOBUILD=1 skips compile+sign and only relaunches.  This matters: the
# camera grant is keyed to the cdhash, so recompiling an already-authorized app
# THROWS THE GRANT AWAY and re-prompts.  Use it for "the daemon died, start it
# again" (xr1_cam.py relaunch); full install only when the source changed.
if [ "${XR1REC_NOBUILD:-0}" = "1" ] && [ -x "$APP/Contents/MacOS/xr1rec" ]; then
    step "build/sign SKIPPED (XR1REC_NOBUILD=1, keeping cdhash and its TCC grant)"
    codesign -dv "$APP" 2>&1 | grep -E 'Identifier|CDHash' || true
else
step "build"
mkdir -p "$APP/Contents/MacOS"
cp "$SRC/Info.plist" "$APP/Contents/Info.plist" || exit 3
# -O because the polling loop and H.264 hand-off run on the main thread.
if ! swiftc -O \
        -framework AVFoundation -framework CoreMedia -framework Foundation \
        -o "$APP/Contents/MacOS/xr1rec" "$SRC/xr1rec.swift" 2>"$ROOT/build.log"; then
    echo "FATAL: compile failed"; cat "$ROOT/build.log"; exit 4
fi
grep -v '^ *$' "$ROOT/build.log" | head -20 || true
ls -l "$APP/Contents/MacOS/xr1rec"

step "sign (ad-hoc)"
# TCC needs a stable code identity.  Ad-hoc is enough, but the grant is keyed to
# the cdhash: REBUILDING RE-PROMPTS.  That is why the build is idempotent and
# not re-run on every experiment.
# NO --options runtime: the hardened runtime denies camera access unless the
# binary carries com.apple.security.device.camera, which an ad-hoc signature
# cannot vouch for.  Symptom is authorizationStatus flipping straight to
# "denied" with no prompt and no TCC record at all.
codesign --force --sign - --identifier com.astrabot.xr1rec "$APP" 2>&1 | tail -2
codesign -dv "$APP" 2>&1 | grep -E 'Identifier|Signature' || true
fi

step "cameras visible to the bundle"
"$APP/Contents/MacOS/xr1rec" devices || true

step "stop any previous daemon"
touch "$ROOT/ctl/quit"; sleep 1.6
pkill -f 'XR1Rec.app/Contents/MacOS/xr1re[c]' 2>/dev/null && sleep 0.5
rm -f "$ROOT/ctl/quit" "$ROOT/ctl/start" "$ROOT/ctl/stop"

step "launch in the GUI session"
# `open` hands the launch to the Aqua session's launchd, so the camera consent
# dialog can actually appear on the Mac's screen.  `--stdout/--stderr` keep the
# daemon's own diagnostics even though it is detached from this ssh session.
rm -f "$ROOT/state.json"
open -a "$APP" --stdout "$ROOT/stdout.log" --stderr "$ROOT/stderr.log" \
     --args daemon --dir "$ROOT" --device "$DEVICE" --fps "$FPS" --preset "$PRESET" \
  || { echo "FATAL: open failed (no GUI session?)"; exit 5; }

step "wait for the daemon to publish state"
for i in $(seq 1 40); do
    [ -f "$ROOT/state.json" ] && break
    sleep 0.5
done
if [ -f "$ROOT/state.json" ]; then
    cat "$ROOT/state.json"
else
    echo "NO state.json after 20 s"
    echo "--- stderr"; cat "$ROOT/stderr.log" 2>/dev/null | tail -20
fi
echo
echo "root=$ROOT app=$APP"
