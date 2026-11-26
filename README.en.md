# EntryNav2 CarPlay → OEM Optical MOST HUD (`hud-port`)

**Language / 语言：** [简体中文](README.md) | [English](README.en.md)

Real-time **iPhone CarPlay Route Guidance** (maneuver icons, exit angles & junction branches, distance/units, UTF-16BE street names, and lane guidance) rendered onto the factory windshield Head-Up Display of **MINI / BMW EntryNav2** head units over MOST `FBlock 0x82 / Inst 0x00` (`HUDTerminal`).

> This repository contains only the pure-C head-unit core (`carplay-shim.so`, `hud_bridge`, `hud_probe`, `rgd_listen`, renderer, unit tests, and `systemd` unit templates). It excludes the Android companion APK and proprietary head-unit libraries.

---

## 1. Verified Vehicle & Head-Unit Platform

| Item | Verified Specification |
|---|---|
| **Vehicle** | 2021 MINI F56 JCW (F-series, factory optical windshield HUD) |
| **Head Unit** | **EntryNav2** (`xse-j6-apn-entryevo`, TI Jacinto 6 ARMv7 Cortex-A15 + NEON, hard-float) |
| **Linux Kernel** | `Linux 3.12.25-FR21.0` |
| **Firmware Build** | `202005181837` |
| **C Runtime ABI** | `eglibc 2.15` (symbol versions `GLIBC_2.4` / `GLIBC_2.7`, `arm-linux-gnueabihf`) |
| **OEM HUD Target** | Optical MOST bus `FBlock 0x82` (`HUDTerminal`), `Inst 0x00`, `Dev 0x0101` |
| **HUD Surfaces** | `TopFrame` container + `Distance` / `Unit` / `ComingStreet` text areas + `1.GuidingIcon` (`120×72`, double-buffered) + `1.LaneGuidance` (`184×17`, double-buffered) |
| **CarPlay Service** | Cinemo `/usr/local/bin/CARPLAY` (`User=carplay` UID `1081`, `WatchdogSec=75`, `/var/run/vmost_sockstream`) |
| **Persistence Note** | `/var/local` (`/dev/mmcblk0p2`) is mounted `noexec,nosuid,nodev` and cannot execute binaries; persistent installs must use a dedicated directory (e.g., `/opt/hud-port/`) and `/etc/systemd/system/` |

---

## 2. Architecture & Data Flow

```text
┌───────────────────┐     iAP2 (0x1D01/0x1D02/0x5200..0x5204)     ┌──────────────────────────────┐
│   iPhone CarPlay  │ ──────────────────────────────────────────► │  /usr/local/bin/CARPLAY      │
│ (Apple/Amap/etc.) │ ◄────────────────────────────────────────── │  + LD_PRELOAD carplay-shim.so│
└───────────────────┘   0x1D01 +0x001E & 0x5200 StartRGD          └──────────────┬───────────────┘
                                                                                 │
                                           Non-blocking trylock ring + worker    │ SOCK_DGRAM (non-blocking)
                                           /tmp/carplay_rgd.sock (chmod 0666)    ▼
┌───────────────────┐     MOST 0x82/0x00 + MHP (0xF1/0x05)        ┌──────────────────────────────┐
│  OEM Optical HUD  │ ◄────────────────────────────────────────── │  hud_bridge + hud_render.c   │
│ (FBlock 0x82/00)  │   /var/run/vmost_sockstream (libvmostclient)│  (State machine + V4 vectors)│
└───────────────────┘                                             └──────────────────────────────┘
```

### Why the two-process split (`carplay-shim.so` + `hud_bridge`)?
- `/usr/local/bin/CARPLAY` runs as unprivileged `User=carplay` (UID `1081`) under `WatchdogSec=75`. Blocking the Cinemo iAP2 RX thread or calling MOST directly from inside `CARPLAY` risks triggering watchdog resets.
- `carplay-shim.so` only patches identification TLVs and performs lock-safe non-blocking capture, forwarding raw iAP2 packets over a Unix datagram socket to `hud_bridge`. Even if `hud_bridge` stops or restarts, `CARPLAY` stays responsive.

---

## 3. Repository Layout & Module Details

```text
hud-port/
├── build.sh                          # Zig cross-compilation + host unit tests + ELF ABI check
├── include/
│   ├── iap2_rgd.h                    # iAP2 0x1D01 / 0x5200..0x5204 TLV constants & BE helpers
│   ├── vmost_api.h                   # EntryNav2 libvmostclient.so.1 structs & 0x82 HUDTerminal IDs
│   └── hud_render.h                  # RgdState state machine, polygon & text frame definitions
├── src/
│   ├── carplay_shim.c                # Non-blocking LD_PRELOAD iAP2 hook for CARPLAY
│   ├── hud_bridge.c                  # Main MOST/MHP session daemon, keepalive & diff renderer
│   ├── hud_render.c                  # iAP2 state machine, V4 smooth vector renderer & UTF-16BE encoder
│   ├── hud_probe.c                   # Read-only MOST 0x82/0x00 probe & 8s standalone draw test
│   └── rgd_listen.c                  # Live /tmp/carplay_rgd.sock iAP2 decoder (--redact supported)
├── tests/
│   └── test_hud_render.c             # Host-side unit tests (-Wall -Wextra -Werror)
└── systemd/
    ├── hud-bridge.service            # Standalone systemd unit for RAM-only (/tmp) testing
    ├── hud-bridge-persist.service    # Persistent systemd unit (/opt/hud-port)
    └── CARPLAY.service.d/
        ├── hud-shim.conf             # RAM-only (/run) CARPLAY drop-in template
        └── hud-shim-persist.conf     # Persistent (/etc) drop-in with 3-crash auto-fallback
```

### Module Breakdown

#### 1. `src/carplay_shim.c` → `carplay-shim.so`
Injected via `LD_PRELOAD` into `/usr/local/bin/CARPLAY` (verifies `/proc/self/exe` so it activates only in `CARPLAY`):
- **Hooked Symbols** (verified on EntryNav2 `202005181837` `libNmeSDK.so` / `libNmeBaseClasses.so`):
  - `CinemoCreateIAP`: captures the `ICinemoIAP*` instance pointer.
  - `NmeIAP2Message::Encode(NmeArray<uint8_t>&)`: intercepts `0x1D01` (`IdentificationInformation`) and injects:
    - `0x0006 MessagesSentByAccessory`: `0x5200` (`StartRouteGuidanceUpdates`), `0x5203` (`StopRouteGuidanceUpdates`);
    - `0x0007 MessagesReceivedFromDevice`: `0x5201`, `0x5202`, `0x5204`;
    - `0x001E RouteGuidanceDisplayComponent` (dynamically picks a unique `ComponentID` and advertises road/maneuver capacities plus `MaxLaneGuidanceStorageCapacity = 8`).
  - `NmeIAP2Message::Decode(const uint8_t*, int)`: captures `0x1D02`, `0x1D03`, and all `0x52xx` messages:
    - Upon `0x1D02 IdentificationAccepted`, schedules a background call to `ICinemoIAP_StartRouteGuidanceUpdates(iap, 1, &comp_id)` (with one automatic refresh if no `0x52xx` arrives within 2.5s).
    - Copies raw iAP2 packets into a **32-slot × 4096B `pthread_mutex_trylock` ring buffer**, drained by a background thread over `/tmp/carplay_rgd.sock` so the RX thread never blocks.

#### 2. `src/hud_render.c` & `include/hud_render.h`
Pure-C state machine and procedural vector renderer — **zero bitmap/PNG assets; every icon and lane strip is computed on the fly from live CarPlay data**:
- **State Machine & Debounce**:
  - Active on `route_state` `1/2/5/6` and `3/4` when maneuvers exist;
  - `2500ms` reroute debounce (`REROUTE_DEBOUNCE_MS`) prevents flickering or premature maneuver cache wipes during brief `route_state=0` transitions; `0x5203` clears immediately;
  - Filters out `0xFFFFFFFF` sentinel distance values and formats both metric (`m`/`km`) and imperial (`ft`/`yd`/`mi`) units.
- **V4 Seamless Single-Polygon Vector Rendering (`HUD_MAX_POLY_PTS = 64`)**:
  - Verified against factory `libVMostAdapter.so` (`0x021D ImageAddPolygon` accepts a `uint8_t` point count up to 255); EntryNav2 renders 38–58 vertex solid polygons cleanly with zero `[HUD_ERR]`.
  - **Turns**: constant-radius circular fillets (`R=15..26px`) plus dimmed `JunctionElementAngles[]` side roads.
  - **U-turns**: entire stem + 26-segment U-bend merged into **one continuous polygon** (2 polygons total including arrowhead), eliminating seam artifacts.
  - **Roundabouts**: dual semicircular grey base ring; highlighted route (`entry stem → G1 tangent fillet → ring arc → G1 exit fillet → exit stem`) merged via `add_polyline_ribbon()` into **one seamless polygon**, with `fillet_deg <= sweep * 0.4f` clamping for early exits.
- **`1.LaneGuidance` (`184×17`)**:
  - Parses `0x5204` lanes (`position`, `status` 0/1/2, multiple `angles[]`, `highlighted_angle`) with fallback rendering if `0x0012 lane_guidance_showing` is omitted.
- **MOST String Encoding (`hud_encode_most_string()`)**:
  - Pure ASCII: `0x01` prefix + 8-bit bytes + `0x00`;
  - CJK / non-ASCII: `0x00` prefix + **UTF-16 Big-Endian** + `0x0000` terminator (verified against `libvmostcpp.so.1.6.0` `writeMOSTString`).

#### 3. `src/hud_bridge.c` → `hud_bridge`
- Connects to `/var/run/vmost_sockstream` via `vmostConnect(..., use_glib_mainloop=0)` (`poll()`-driven) and binds `/tmp/carplay_rgd.sock` (`chmod 0666` for UID `1081`).
- Opens an MHP session (`0xF1/0x05/0x000C`) and creates `TopFrame` (`0x08`), `Distance`/`Unit`/`ComingStreet` (`0x03`), `1.GuidingIcon`/`2.GuidingIcon` (`0x04`), and `1.LaneGuidance`/`2.LaneGuidance` (`0x04`) on `0x82/0x00`.
- Uses `FNV-1a content_hash` diffing + **double-buffered `SwitchVisibility` (`0x020E`)**.
- **Long-Route Keepalive & Factory Yield**:
  - Every **15 seconds** while active and visually unchanged (e.g., long straight highway stretches), re-asserts `SetVisibility(TopFrame, 1)` + active icon visibility and flushes MHP so `HUDTerminal` cannot silently time out or stay hidden after cluster popups;
  - Detects `ErrorAck (0x0F)` and yields `0x82/0x00` for 5 seconds if factory navigation takes over;
  - Includes `--test-lanes [dev_hex]` (8s auto-cleaning test pattern) and PII-free `[RGD]` transition/counter logs.

#### 4. `src/hud_probe.c` & `src/rgd_listen.c`
- `hud_probe`: diagnostic tool for inspecting MOST own address, `MPR`, registered `0x82/0x00` device address (`--status` / `--probe-read` / `--test-draw`).
- `rgd_listen`: standalone `/tmp/carplay_rgd.sock` packet decoder (`--redact` strips street and destination strings).

---

## 4. Cross-Compilation

### Prerequisites
1. Install [Zig](https://ziglang.org/) (tested with `zig 0.17.1`).
2. Pull the two proprietary runtime libraries from your head unit's `/usr/local/lib/` (**kept local for cross-linking only; do not commit to public repos**):
   - `libvmostclient.so.1.6.0`
   - `libvmost.so.1.6.0`

### Build & Test

```bash
export RUNTIME_LIB=/path/to/entrynav2/usr/local/lib
./build.sh
```

`build.sh` runs:
1. Host-side unit tests (`tests/test_hud_render.c`, `-Wall -Wextra -Werror`);
2. ARMv7 cross-compilation via `zig cc -target arm-linux-gnueabihf.2.15 -mcpu=cortex_a15+neon` for `hud_probe`, `carplay-shim.so`, `rgd_listen`, and `hud_bridge`;
3. ELF machine/flag and `GLIBC_2.4`/`2.7` symbol verification.

---

## 5. Head-Unit Deployment

> **Safety Rule**: **Never power off the ignition while `/` is mounted read-write (`rw`).** Always test with the RAM-only setup first.

### Option A: RAM-Only Testing (Reverts completely on ignition off / reboot)

```sh
# 1. Copy binaries from build/ to /tmp on the head unit
scp build/hud_bridge build/carplay-shim.so build/hud_probe build/rgd_listen root@169.254.1.0:/tmp/
chmod 0755 /tmp/hud_bridge /tmp/carplay-shim.so /tmp/hud_probe /tmp/rgd_listen

# 2. Install ephemeral systemd units under /run (RAM only)
cp systemd/hud-bridge.service /run/systemd/system/hud-bridge.service
mkdir -p /run/systemd/system/CARPLAY.service.d
cp systemd/CARPLAY.service.d/hud-shim.conf /run/systemd/system/CARPLAY.service.d/hud-shim.conf

systemctl daemon-reload
systemctl start hud-bridge.service
systemctl restart CARPLAY.service
```

Stop and restore stock `CARPLAY` immediately:

```sh
rm -rf /run/systemd/system/CARPLAY.service.d /run/systemd/system/hud-bridge.service
systemctl daemon-reload
systemctl stop hud-bridge.service || true
systemctl restart CARPLAY.service
```

### Option B: Guarded Persistent Installation (Auto-start on boot)

- On EntryNav2, `/var/local` (`/dev/mmcblk0p2`) is mounted `noexec`, so executables cannot run from there.
- Place `hud_bridge` and `carplay-shim.so` under `/opt/hud-port/`, and install `systemd/hud-bridge-persist.service` (`WantedBy=navigation.target`) and `systemd/CARPLAY.service.d/hud-shim-persist.conf` (includes a `/tmp/.hud_shim_tries` **3-crash boot-loop guard** that falls back to stock `CARPLAY`) under `/etc/systemd/system/`.
- Always finish with `sync && mount -o remount,ro /` and verify `awk '$2=="/" && $1!="rootfs"' /proc/mounts` reports `ro` before turning off ignition.

---

## 6. Navigation App Compatibility

| App | Maneuver Icon / Distance / Road Name | Lane Guidance (`0x5204`) | Notes |
|---|---|---|---|
| **Apple Maps** | Supported | **Supported** (2–8 lanes near junctions) | Emits `0x5201`, `0x5202`, and `0x5204` |
| **Amap (高德地图)** | Supported | **Not emitted by app** | iOS CarPlay Amap draws lanes on the main CarPlay screen only and sends 0 `0x5204` frames over iAP2 |
| **Baidu Maps (百度地图)** | Supported | Supported (protocol-level `0x5204`) | Depends on app version |
