# EntryNav2 CarPlay → OEM Optical MOST HUD (`hud-port`)

**Language / 语言：** [简体中文](README.md) | [English](README.en.md)

> **If this project helped or inspired you, please consider giving it a Star ⭐!**
>
> **Original Work & Noncommercial License**: All pure-C core code in this repository is 100% original work written from scratch via real-vehicle captures and protocol analysis. Permitted for personal learning, research, and noncommercial use only — **commercial use is strictly prohibited** (see [LICENSE](LICENSE)).

Real-time **iPhone CarPlay Route Guidance** (vector turn/roundabout/destination icons, exit angles & junction branches, distance/units, UTF-16BE street names, and lane guidance) rendered onto the factory windshield Head-Up Display of **MINI / BMW EntryNav2** head units over MOST `FBlock 0x82 / Inst 0x00` (`HUDTerminal`).

> This repository contains only the pure-C head-unit core (`carplay-shim.so`, `hud_bridge`, `hud_probe`, `rgd_listen`, renderer, unit tests, and `systemd` templates). It excludes the Android companion APK and proprietary head-unit libraries.

---

## 1. Real-Vehicle Photos & Vector Render Showcase

| Optical HUD Close-Up (CJK + Distance + Arrow) | Center CarPlay Navigation Sync | Cockpit View (HUD + Cluster + Center) |
| :---: | :---: | :---: |
| ![OEM HUD Close-Up](docs/realcar_hud_closeup.jpg) | ![CarPlay Center Screen](docs/realcar_carplay_screen.jpg) | ![Real-Car Cockpit View](docs/realcar_hud_cockpit.jpg) |

![EntryNav2 HUD Vector Render Showcase](docs/hud_render_showcase.png)

*Rasterized directly from real `0x021D` polygon and `0x021E` circle/arc draw commands emitted by `hud_render.c` — zero static bitmap assets.*

---

## 2. Architecture Principles & Advantages (Apple iAP2 + OEM EntryNav2 Stack)

The input is **Apple iAP2 Route Guidance**; the output is the **native EntryNav2 MOST HUD**. A bridge that maps every right turn to the same icon discards the exit angle. This renderer keeps that angle and can draw the other branches of the junction alongside the chosen one.

### 1. Reading Apple iAP2 Route Guidance directly
- **Compliant Identification & Native Subscription**: `NmeIAP2Message::Encode` patches `0x1D01` (`IdentificationInformation`) with `0x001E RouteGuidanceDisplayComponent` (`0x0000..0x0008` in exact iAP2 order) plus `0x0006` (`0x5200/0x5203`) and `0x0007` (`0x5201/0x5202/0x5204`), then triggers native `ICinemoIAP_StartRouteGuidanceUpdates` upon `0x1D02 IdentificationAccepted` (verified against `libNmeSDK.so`, `libNmeBaseClasses.so`, and `libNmeVfs.so`).
- **Junction and lane geometry**: `0x5201` carries route state, road and destination names, and distance; `0x5202` carries `ManeuverType`, `ExitAngle` (−180° to +180°), `JunctionType`, `JunctionElementAngles[]` and `DrivingSide`; `0x5204` carries each lane’s position, status, directions and highlighted direction. The renderer uses the fields the map app sends; it does not infer missing lanes from video.

### 2. Native Optical MOST HUD Vector Rendering (Verified Against OEM Libraries)
Verified against factory `libVMostAdapter.so`, `libvmostclient.so.1.6.0`, and `libvmostcpp.so.1.6.0`, `hud_render.c` emits native `HUDTerminal` (`FBlock 0x82 / Inst 0x00`, `MHP 0xF1/0x05`) drawing primitives with zero bitmap assets:
- **Continuous-Angle Turns & Junction Context**: Smooth circular-fillet turns rotated by exact `ExitAngle`, paired with `Open V-Chevron` arrowheads and dimmed (`HUD_COLOR_DIM = 3`) `JunctionElementAngles[]` non-navigated branches.
- **Hardware `0x021E` Arcs & Seamless `0x021D` Polygons**: Aligns strictly with EntryNav2's verified `0x021E` coordinates (`0 = 3 o'clock counter-clockwise`, `0.1°` units) for turn fillets, roundabout base rings (`HUD_CIRCLE_FULL`) and active arcs, U-turn arches, tangent-matched **S-curves** (`HUD_CIRCLE_ARC`) on highway lane changes / offset arrivals, and solid arrival markers (`HUD_CIRCLE_FILLED`), paired with `0x021D` polygons (38–58 vertices verified cleanly on hardware).
- **Lane guidance**: `0x5204` supplies the position, status and branch angles drawn in `1.LaneGuidance`. White marks usable or preferred directions; grey preserves the alternatives. The strip is not cropped from the CarPlay video.
- **OEM `DestinationMan`-Style Arrival**: Mirrors `libVMostAdapter.so` (`GI: DestinationMan`), combining a native `0x021E HUD_CIRCLE_FILLED` dot with an upward arrow for `ManeuverType = 10/12/24/25/27` (including left/right S-curves) and a `RouteState = 2 (Arrived)` fallback that hides stale lane strips and prioritizes `DestinationName`.
- **Native CJK / UTF-16BE Text Encoding**: Implements `libvmostcpp.so.1.6.0` `writeMOSTString` (`0x01` for pure ASCII, `0x00` + **UTF-16 Big-Endian** + `0x0000` for CJK/non-ASCII), displaying Chinese street and destination names natively on the OEM HUD.
- **Refresh and recovery**: `hud_bridge` compares frame contents with FNV-1a, swaps front and back canvases, and sends a keepalive after 15 seconds of unchanged guidance. A transient no-route state has a 2.5-second grace period; a HUD error releases the session for five seconds.

---

## 3. Verified Vehicle & Head-Unit Platform

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

## 4. Architecture & Data Flow

```text
┌───────────────────┐     iAP2 (0x1D01/0x1D02/0x5200..0x5204)     ┌──────────────────────────────┐
│   iPhone CarPlay  │ ──────────────────────────────────────────► │  /usr/local/bin/CARPLAY      │
│ (Apple/Amap/etc.) │ ◄────────────────────────────────────────── │  + LD_PRELOAD carplay-shim.so│
└───────────────────┘   0x1D01 +0x001E & 0x5200 StartRGD          └──────────────┬───────────────┘
                                                                                 │
                                           trylock ring + worker thread          │ SOCK_DGRAM (non-blocking)
                                           /tmp/carplay_rgd.sock (chmod 0666)    ▼
┌───────────────────┐     MOST 0x82/0x00 + MHP (0xF1/0x05)        ┌──────────────────────────────┐
│  OEM Optical HUD  │ ◄────────────────────────────────────────── │  hud_bridge + hud_render.c   │
│ (FBlock 0x82/00)  │   /var/run/vmost_sockstream (libvmostclient)│  (State machine + vector/arc)│
└───────────────────┘                                             └──────────────────────────────┘
```

### Why the two-process split (`carplay-shim.so` + `hud_bridge`)?
- `/usr/local/bin/CARPLAY` runs as unprivileged `User=carplay` (UID `1081`) under `WatchdogSec=75`. Blocking the Cinemo iAP2 RX thread or calling MOST directly from inside `CARPLAY` risks triggering watchdog resets.
- `carplay-shim.so` handles component advertisement, subscription and packet copying. The receive thread uses `pthread_mutex_trylock`: a busy lock or full queue drops that copy rather than waiting for the HUD. A worker forwards copies via non-blocking Unix datagrams, so a stopped or restarting bridge does not make the head unit’s CarPlay receive callback wait for MOST.

---

## 5. Repository Layout & Module Details

```text
hud-port/
├── build.sh                          # Zig cross-compilation + host unit tests + ELF ABI check
├── docs/                             # Real-vehicle photos and vector render showcase
├── include/
│   ├── iap2_rgd.h                    # iAP2 0x1D01 / 0x5200..0x5204 TLV constants & BE helpers
│   ├── vmost_api.h                   # EntryNav2 libvmostclient.so.1 structs & 0x82 HUDTerminal IDs
│   └── hud_render.h                  # RgdState state machine, polygon & text frame definitions
├── src/
│   ├── carplay_shim.c                # Non-blocking LD_PRELOAD iAP2 hook for CARPLAY
│   ├── hud_bridge.c                  # Main MOST/MHP session daemon, keepalive & diff renderer
│   ├── hud_render.c                  # iAP2 state machine, hardware arcs, polygons and UTF-16BE text
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

#### 1. [`src/carplay_shim.c`](src/carplay_shim.c) → `carplay-shim.so`
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

#### 2. [`src/hud_render.c`](src/hud_render.c) & [`include/hud_render.h`](include/hud_render.h)
A pure-C state machine generates maneuver and lane graphics from navigation data. The frame consists of polygons, arcs and text; there are no bitmap icons.
- **State Machine & Debounce**:
  - Active on `route_state` `1/2/5/6` and `3/4` when maneuvers exist;
  - `2500ms` reroute debounce (`REROUTE_DEBOUNCE_MS`) prevents flickering or premature maneuver cache wipes during brief `route_state=0` transitions; `0x5203` clears immediately;
  - Filters out `0xFFFFFFFF` sentinel distance values and formats both metric (`m`/`km`) and imperial (`ft`/`yd`/`mi`) units.
- **Vector polygons and native `0x021E` arcs (`HUD_MAX_POLY_PTS = 64`)**:
  - Verified against factory `libVMostAdapter.so` (`0x021D ImageAddPolygon` accepts a `uint8_t` point count up to 255, and `0x021E ImageAddCircle` draws hardware circles/arcs); EntryNav2 renders 38–58 vertex solid polygons cleanly with zero `[HUD_ERR]`.
  - **Turns**: a constant-radius hardware arc joins the entry and exit segments. The radius and tangent points adapt to the exit angle; `JunctionElementAngles[]` supplies the dim side roads.
  - **U-turns**: two straight stems meet a 180° `0x021E` upper arc. Left and right U-turns mirror the stems; the arc runs across the top from 3 to 9 o’clock in both cases.
  - **Roundabouts**: the grey full ring and unused exits are drawn first, then the white entry stem, selected arc, exit stem and open chevron. `ExitAngle` selects the exit; `DrivingSide` selects which way around the ring to draw.
  - **Highway Lane Changes (`52/53`) & Destination Arrival (`10/12/24/25/27` / `route_state=2`)**: dual-arc `0x021E HUD_CIRCLE_ARC` S-curves for smooth lateral transitions, paired with OEM `GI: DestinationMan` styling (`0x021E HUD_CIRCLE_FILLED` dot + upward arrow).
- **`1.LaneGuidance` (`184×17`)**:
  - Parses `0x5204` lanes (`position`, `status` 0/1/2, multiple `angles[]`, `highlighted_angle`) with fallback rendering if `0x0012 lane_guidance_showing` is omitted.
  - This version uses a separate lane canvas. Up to 7 lanes fit directly; with 8 lanes it shows a 6-lane window around usable or preferred lanes and marks omitted lanes with dots on either side.
- **MOST String Encoding (`hud_encode_most_string()`)**:
  - Pure ASCII: `0x01` prefix + 8-bit bytes + `0x00`;
  - CJK / non-ASCII: `0x00` prefix + **UTF-16 Big-Endian** + `0x0000` terminator (verified against `libvmostcpp.so.1.6.0` `writeMOSTString`).

#### 3. [`src/hud_bridge.c`](src/hud_bridge.c) → `hud_bridge`
- Connects to `/var/run/vmost_sockstream` via `vmostConnect(..., use_glib_mainloop=0)` (`poll()`-driven) and binds `/tmp/carplay_rgd.sock` (`chmod 0666` for UID `1081`).
- Opens an MHP session (`0xF1/0x05/0x000C`) and creates `TopFrame` (`0x08`), `Distance`/`Unit`/`ComingStreet` (`0x03`), `1.GuidingIcon`/`2.GuidingIcon` (`0x04`), and `1.LaneGuidance`/`2.LaneGuidance` (`0x04`) on `0x82/0x00`.
- Uses `FNV-1a content_hash` diffing + **double-buffered `SwitchVisibility` (`0x020E`)**.
- **Long-Route Keepalive & Factory Yield**:
  - Every **15 seconds** while active and visually unchanged (e.g., long straight highway stretches), re-asserts `SetVisibility(TopFrame, 1)` + active icon visibility and flushes MHP so `HUDTerminal` cannot silently time out or stay hidden after cluster popups;
  - A HUD `Error` or `ErrorAck` releases the session and pauses retries for five seconds to give factory navigation room. The reply itself does not identify a particular OEM alert.
  - `--test-lanes` displays a test frame for eight seconds, then releases the session. In normal guidance mode, `[RGD]` logs state changes and packet counts without street names.

#### 4. [`src/hud_probe.c`](src/hud_probe.c) & [`src/rgd_listen.c`](src/rgd_listen.c)
- `hud_probe`: diagnostic tool for inspecting MOST own address, `MPR`, registered `0x82/0x00` device address (`--status` / `--probe-read` / `--test-draw`).
- `rgd_listen`: standalone decoder for `/tmp/carplay_rgd.sock` (`--redact` hides street and destination names). It binds the same receive address as `hud_bridge`; do not run the two receivers together.

---

## 6. Cross-Compilation

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

## 7. Head-Unit Deployment

> **Safety Rule**: **Never power off the ignition while `/` is mounted read-write (`rw`).** Validate with the RAM-only setup first.

- **Option A: RAM-Only Testing (Reverts completely on reboot)**

```sh
scp build/hud_bridge build/carplay-shim.so build/hud_probe build/rgd_listen root@169.254.1.0:/tmp/
chmod 0755 /tmp/hud_bridge /tmp/carplay-shim.so /tmp/hud_probe /tmp/rgd_listen
cp systemd/hud-bridge.service /run/systemd/system/hud-bridge.service
mkdir -p /run/systemd/system/CARPLAY.service.d
cp systemd/CARPLAY.service.d/hud-shim.conf /run/systemd/system/CARPLAY.service.d/hud-shim.conf
systemctl daemon-reload && systemctl start hud-bridge.service && systemctl restart CARPLAY.service
```

- **Option B: Guarded Persistent Install**
  `/var/local` is mounted `noexec`. Place binaries under `/opt/hud-port/`, install `systemd/hud-bridge-persist.service` and `systemd/CARPLAY.service.d/hud-shim-persist.conf` (includes `/tmp/.hud_shim_tries` **3-crash fallback** to stock `CARPLAY`) under `/etc/systemd/system/`, then run `sync && mount -o remount,ro /` and verify `/` is `ro`.

---

## 8. Navigation App Compatibility

| App | Maneuver Icon / Distance / Road Name | Lane Guidance (`0x5204`) | Notes |
|---|---|---|---|
| **Apple Maps** | Supported | **Supported** (2–8 lanes near junctions) | Emits `0x5201`, `0x5202`, and `0x5204` |
| **Amap (高德地图)** | Supported | **Not emitted by app** | The tested iOS version drew lanes on the CarPlay screen but sent no `0x5204` updates over iAP2 |
| **Baidu Maps (百度地图)** | Supported | Supported (protocol-level `0x5204`) | Depends on app version |

---

## 9. Copyright & Noncommercial License

- **Copyright**: `Copyright (c) 2026 cerary. All rights reserved.`
- **Originality**: All pure-C source files in this repository are **original work written from scratch** by the author based on live vehicle captures and binary analysis, with zero third-party code or static image assets; `libvmostclient.so.1` and `libvmost.so.1` belong to your own head unit and are not distributed here.
- **License**: Licensed under **[PolyForm Noncommercial License 1.0.0](LICENSE)**:
  - ✅ **Permitted**: Personal study, research, inspection, noncommercial personal vehicle use, and noncommercial forks/modifications (must retain the original copyright notice and `LICENSE`).
  - ❌ **Commercial Use Prohibited**: Commercial use, bundling, paid retrofit/coding services, or commercial hardware/software distribution is strictly prohibited.

> **If you like this project, please drop a Star ⭐ — it really helps and motivates continued work!**
