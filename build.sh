#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
OUT_DIR="$ROOT/build"
RUNTIME_LIB="${RUNTIME_LIB:-$ROOT/../runtime/usr/local/lib}"
LINK_DIR="${LINK_DIR:-${TMPDIR:-/tmp}/mini-hud-link}"
READELF="${READELF:-$(command -v llvm-readelf || true)}"
if [ -z "$READELF" ] && [ -x "$HOME/Library/Android/sdk/ndk/28.2.13676358/toolchains/llvm/prebuilt/darwin-x86_64/bin/llvm-readelf" ]; then
  READELF="$HOME/Library/Android/sdk/ndk/28.2.13676358/toolchains/llvm/prebuilt/darwin-x86_64/bin/llvm-readelf"
fi

echo "[0/4] Running host unit test test_hud_render..."
cc -Wall -Wextra -Werror -O2 -I"$ROOT/include" "$ROOT/tests/test_hud_render.c" "$ROOT/src/hud_render.c" -lm -o "${TMPDIR:-/tmp}/test_hud_render_host"
"${TMPDIR:-/tmp}/test_hud_render_host"
rm -f "${TMPDIR:-/tmp}/test_hud_render_host"

if [ ! -f "$RUNTIME_LIB/libvmostclient.so.1.6.0" ] || [ ! -f "$RUNTIME_LIB/libvmost.so.1.6.0" ]; then
  echo "ERROR: Missing EntryNav2 runtime libraries under RUNTIME_LIB=$RUNTIME_LIB" >&2
  echo "Expected libvmostclient.so.1.6.0 and libvmost.so.1.6.0 pulled from /usr/local/lib on the head unit." >&2
  exit 1
fi

mkdir -p "$OUT_DIR" "$LINK_DIR"

# Ensure exact SONAME symlinks libvmostclient.so.1 and libvmost.so.1
cp -f "$RUNTIME_LIB/libvmostclient.so.1.6.0" "$LINK_DIR/libvmostclient.so.1"
ln -sf "libvmostclient.so.1" "$LINK_DIR/libvmostclient.so"
cp -f "$RUNTIME_LIB/libvmost.so.1.6.0" "$LINK_DIR/libvmost.so.1"
ln -sf "libvmost.so.1" "$LINK_DIR/libvmost.so"

echo "[1/4] Building hud_probe..."
zig cc \
  -target arm-linux-gnueabihf.2.15 \
  -mcpu=cortex_a15+neon \
  -O2 -Wall -Wextra \
  -I"$ROOT/include" \
  "$ROOT/src/hud_probe.c" \
  -L"$LINK_DIR" -lvmostclient -lvmost \
  -Wl,-rpath,/usr/local/lib \
  -Wl,--allow-shlib-undefined \
  -o "$OUT_DIR/hud_probe"

echo "[2/4] Building carplay-shim.so..."
zig cc \
  -target arm-linux-gnueabihf.2.15 \
  -mcpu=cortex_a15+neon \
  -O2 -Wall -Wextra -shared -fPIC \
  -I"$ROOT/include" \
  "$ROOT/src/carplay_shim.c" \
  -ldl -lpthread \
  -Wl,-soname,carplay-shim.so \
  -o "$OUT_DIR/carplay-shim.so"

echo "[3/4] Building rgd_listen..."
zig cc \
  -target arm-linux-gnueabihf.2.15 \
  -mcpu=cortex_a15+neon \
  -O2 -Wall -Wextra \
  -I"$ROOT/include" \
  "$ROOT/src/rgd_listen.c" \
  -o "$OUT_DIR/rgd_listen"

echo "[4/4] Building hud_bridge..."
zig cc \
  -target arm-linux-gnueabihf.2.15 \
  -mcpu=cortex_a15+neon \
  -O2 -Wall -Wextra \
  -I"$ROOT/include" \
  "$ROOT/src/hud_bridge.c" \
  "$ROOT/src/hud_render.c" \
  -L"$LINK_DIR" -lvmostclient -lvmost -lm \
  -Wl,-rpath,/usr/local/lib \
  -Wl,--allow-shlib-undefined \
  -o "$OUT_DIR/hud_bridge"

echo "=== Verification ==="
for art in "$OUT_DIR/hud_probe" "$OUT_DIR/carplay-shim.so" "$OUT_DIR/rgd_listen" "$OUT_DIR/hud_bridge"; do
  echo "--- $art ---"
  file "$art"
  if [ -n "$READELF" ] && [ -x "$READELF" ]; then
    "$READELF" -h -d --version-info "$art" | grep -E "Machine:|Flags:|NEEDED|Name: GLIBC" | head -n 20
  fi
done
