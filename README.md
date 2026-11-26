# EntryNav2 CarPlay → 原厂光学 MOST HUD 实时导航投射（`hud-port`）

**语言 / Language：** [简体中文](README.md) | [English](README.en.md)

将 iPhone **CarPlay 实时导航引导（转向图标、出口角度与岔路、距离/单位、中文/英文路名、车道引导）** 实时投射到 **MINI / BMW EntryNav2** 主机的原厂光学风挡抬头显示（MOST `FBlock 0x82 / Inst 0x00` `HUDTerminal`）。

> 本仓库仅包含车机端纯 C 核心工程（`carplay-shim.so`、`hud_bridge`、`hud_probe`、`rgd_listen`、渲染器、单元测试与 systemd 部署模板），不含 Android 调试端 APK 与车机专有闭源库。

---

## 一、已验证目标车型与车机平台规格

| 项目 | 实车验证版本 / 规格 |
|---|---|
| **车辆** | 2021 MINI F56 JCW（F 底盘，配原厂光学风挡 HUD） |
| **主机平台** | **EntryNav2**（`xse-j6-apn-entryevo`，TI Jacinto 6 ARMv7 Cortex-A15 + NEON，hard-float） |
| **Linux 内核** | `Linux 3.12.25-FR21.0` |
| **车机固件构建** | `202005181837` |
| **C 运行库 ABI** | `eglibc 2.15`（符号版本需兼容 `GLIBC_2.4` / `GLIBC_2.7`，`arm-linux-gnueabihf`） |
| **原厂 HUD 通道** | 光学 MOST 总线 `FBlock 0x82` (`HUDTerminal`), `Inst 0x00`, `Dev 0x0101` |
| **HUD 画布规格** | `TopFrame` 容器 + `Distance`/`Unit`/`ComingStreet` 文本区 + `1.GuidingIcon` (`120×72`, 双缓冲) + `1.LaneGuidance` (`184×17`, 双缓冲) |
| **CarPlay 服务** | Cinemo `/usr/local/bin/CARPLAY`（`User=carplay` UID `1081`, `WatchdogSec=75`, `/var/run/vmost_sockstream`） |
| **持久化限制** | `/var/local` (`/dev/mmcblk0p2`) 为 `noexec,nosuid,nodev` 挂载，不可执行二进制；持久化须使用独立受控目录（如 `/opt/hud-port/`）与 `/etc/systemd/system/` |

---

## 二、整体架构与实时数据流

```text
┌───────────────────┐     iAP2 (0x1D01/0x1D02/0x5200..0x5204)     ┌──────────────────────────────┐
│   iPhone CarPlay  │ ──────────────────────────────────────────► │  /usr/local/bin/CARPLAY      │
│ (Apple地图/高德等) │ ◄────────────────────────────────────────── │  + LD_PRELOAD carplay-shim.so│
└───────────────────┘   0x1D01 注入 0x001E + 0x5200 StartRGD      └──────────────┬───────────────┘
                                                                                 │
                                           非阻塞 trylock 环形队列 + 后台线程      │ SOCK_DGRAM (非阻塞)
                                           /tmp/carplay_rgd.sock (chmod 0666)    ▼
┌───────────────────┐     MOST 0x82/0x00 + MHP (0xF1/0x05)        ┌──────────────────────────────┐
│ 原厂光学风挡 HUD   │ ◄────────────────────────────────────────── │  hud_bridge + hud_render.c   │
│ (FBlock 0x82/00)  │   /var/run/vmost_sockstream (libvmostclient)│  (状态机 + V4矢量多边形渲染) │
└───────────────────┘                                             └──────────────────────────────┘
```

### 为什么需要 `carplay-shim.so` + `hud_bridge` 两段式解耦？
- `/usr/local/bin/CARPLAY` 以普通用户 `carplay` (UID 1081) 运行，受 `WatchdogSec=75` 看门狗监控，**绝不能在 Cinemo iAP2 接收线程内执行阻塞操作或直接操作 MOST 总线**。
- `carplay-shim.so` 仅负责协议补全与无锁零阻塞捕获，把原始 iAP2 报文通过 Unix DGRAM 丢给独立进程 `hud_bridge`；即使 `hud_bridge` 重启或未启动，`CARPLAY` 也不会卡死或触发看门狗重启。

---

## 三、目录结构与核心模块说明

```text
hud-port/
├── build.sh                          # Zig 交叉编译脚本 + 主机单元测试 + ABI 校验
├── include/
│   ├── iap2_rgd.h                    # iAP2 0x1D01 / 0x5200..0x5204 TLV 常量与大端读写辅助
│   ├── vmost_api.h                   # EntryNav2 libvmostclient.so.1 结构体与 0x82 HUDTerminal 功能码
│   └── hud_render.h                  # RgdState 状态机、矢量多边形/文本帧结构定义
├── src/
│   ├── carplay_shim.c                # CARPLAY 进程 LD_PRELOAD 非阻塞 iAP2 钩子
│   ├── hud_bridge.c                  # 主桥接守护进程：MHP/MOST 会话管理、保活与热渲染
│   ├── hud_render.c                  # iAP2 状态机 + V4 单块连续矢量多边形渲染 + UTF-16BE 编码
│   ├── hud_probe.c                   # MOST 0x82/0x00 只读探测 / 8 秒独立画图自检工具
│   └── rgd_listen.c                  # /tmp/carplay_rgd.sock 实时抓包解码器（支持 --redact 脱敏）
├── tests/
│   └── test_hud_render.c             # 主机端单元测试（-Wall -Wextra -Werror）
└── systemd/
    ├── hud-bridge.service            # hud_bridge 独立 systemd 服务模板
    └── CARPLAY.service.d/
        ├── hud-shim.conf             # 内存态 (/run) 临时注入模板
        └── hud-shim-persist.conf     # 持久化 (/etc) 带 3 次崩溃自愈回退模板
```

### 各模块详细职责

#### 1. `src/carplay_shim.c` → `carplay-shim.so`
通过 `LD_PRELOAD` 注入 `/usr/local/bin/CARPLAY`（构造函数校验 `/proc/self/exe` 仅对 `CARPLAY` 生效）：
- **符号 Hook**（已对固件 `202005181837` 的 `libNmeSDK.so` / `libNmeBaseClasses.so` 验证）：
  - `CinemoCreateIAP`：捕获 `ICinemoIAP*` 实例指针。
  - `NmeIAP2Message::Encode(NmeArray<uint8_t>&)`：拦截 `0x1D01` (`IdentificationInformation`)，自动追加：
    - `0x0006 MessagesSentByAccessory`：`0x5200` (`StartRouteGuidanceUpdates`)、`0x5203` (`StopRouteGuidanceUpdates`)；
    - `0x0007 MessagesReceivedFromDevice`：`0x5201`、`0x5202`、`0x5204`；
    - `0x001E RouteGuidanceDisplayComponent`（动态分配唯一 `ComponentID`，声明路口/路名/动作长度与车道容量 `MaxLaneGuidanceStorageCapacity = 8`）。
  - `NmeIAP2Message::Decode(const uint8_t*, int)`：捕获 `0x1D02`、`0x1D03` 与所有 `0x52xx` 报文：
    - 收到 `0x1D02 IdentificationAccepted` 后由后台线程调用 `ICinemoIAP_StartRouteGuidanceUpdates(iap, 1, &comp_id)`（若 2.5s 内未收到 `0x52xx` 会自动补发一次刷新）。
    - 采用 **32 槽 × 4096B `pthread_mutex_trylock` 环形缓冲区** 将原始报文交给后台线程发送至 `/tmp/carplay_rgd.sock`，**绝不阻塞解码线程**。

#### 2. `src/hud_render.c` & `include/hud_render.h`
纯 C 状态机与 HUD 矢量画面生成器，**无任何静态图片素材，全部图标与车道条均由 CarPlay 实时数据计算多边形**：
- **状态机与防抖**：
  - 支持 `route_state` `1/2/5/6` 及带有效转向的 `3/4`；
  - `2500ms` 重算路防抖（`REROUTE_DEBOUNCE_MS`）：短暂 `route_state=0` 不闪烁清屏，超时后才调用 `rgd_state_reset_route()`；收到 `0x5203` 立即清屏；
  - 自动过滤 `0xFFFFFFFF` 无效距离哨兵值，支持公制（`m`/`km`）与英制（`ft`/`yd`/`mi`）格式化。
- **V4 单块连续矢量轮廓渲染（`HUD_MAX_POLY_PTS = 64`）**：
  - 经对车机原厂 `libVMostAdapter.so` 的 `0x021D`（`ImageAddPolygon`）核查，`nPts` 为 `uint8_t`（上限 255 点），EntryNav2 实车稳定支持单块 38~58 顶点连续填充多边形。
  - **普通转向弯道**：等半径圆弧倒角（`R=15~26px`），同时绘制 `JunctionElementAngles[]` 灰色非行进岔路。
  - **掉头**：整条弯杆 + 26 段半圆弧合并为 **1 个整块连续多边形**（加 1 个箭头共 2 面），消除矩形接缝。
  - **环岛**：灰色底环用 2 个等宽半圆；高亮路径（直杆 → G1 切线入环 → 环岛圆弧 → G1 切线出环 → 出口箭杆）由 `add_polyline_ribbon()` 合并为 **1 个整块无缝多边形**，带 `fillet_deg <= sweep * 0.4f` 小角度早出保护。
- **`1.LaneGuidance`（`184×17`）车道条**：
  - 解析 `0x5204` 各车道的 `position`、`status`（0=不可用灰、1=可用白、2=推荐白）、多方向 `angles[]` 与 `highlighted_angle` 现场绘制；兼容未下发 `0x0012` 显隐标志时的索引兜底显示。
- **中文路名编码 `hud_encode_most_string()`**：
  - 纯 ASCII 使用 `0x01` + 8-bit + `0x00`；
  - 含中文/非 ASCII 字符自动转为 `0x00` + **UTF-16 Big-Endian** + `0x0000` 结束符（已核对 `libvmostcpp.so.1.6.0` `writeMOSTString`）。

#### 3. `src/hud_bridge.c` → `hud_bridge`
- 连接 `/var/run/vmost_sockstream`（`vmostConnect(..., use_glib_mainloop=0)`，纯 `poll()` 事件驱动）并创建 `/tmp/carplay_rgd.sock`（`chmod 0666` 确保 UID 1081 可写）。
- 向 `0xF1/0x05/0x000C` 打开 MHP 通道，向 `0x82/0x00` 创建：
  - `TopFrame` (`0x08`)、`Distance`/`Unit`/`ComingStreet` (`0x03`)、`1.GuidingIcon`/`2.GuidingIcon` (`0x04`)、`1.LaneGuidance`/`2.LaneGuidance` (`0x04`)。
- 采用 `FNV-1a content_hash` 差异渲染 + **双缓冲 `SwitchVisibility` (`0x020E`)**。
- **长途保活与防抢占**：
  - 导航激活且画面静止（如长直路段数公里数字不变）时，每 **15 秒** 自动发送一次 `SetVisibility(TopFrame, 1)` + 图标可见性刷新并 `Flush`，防止 `HUDTerminal` 长时无报文后隐藏；
  - 监听原厂 `ErrorAck (0x0F)`：若原厂导航抢占 `0x82/0x00`，立即释放并让位 5 秒；
  - 内置 `--test-lanes [dev_hex]`（8 秒离线测试帧，退出自动销毁）与脱敏 `[RGD]` 日志统计。

#### 4. `src/hud_probe.c` 与 `src/rgd_listen.c`
- `hud_probe`：用于验证本机 MOST 地址、`MPR`、注册表中的 `0x82/0x00` 设备地址（`--status` / `--probe-read` / `--test-draw`）。
- `rgd_listen`：独立调试抓取 `/tmp/carplay_rgd.sock` 解码输出，`--redact` 可隐藏街道与目的地明文。

---

## 四、交叉编译

### 依赖准备
1. 安装 [Zig](https://ziglang.org/)（实测使用 `zig 0.17.1`）。
2. 从车机 `/usr/local/lib/` 提取以下两个运行时共享库（**仅供本地交叉链接使用，请勿提交到公开仓库**）：
   - `libvmostclient.so.1.6.0`
   - `libvmost.so.1.6.0`

### 编译与单元测试

```bash
export RUNTIME_LIB=/path/to/entrynav2/usr/local/lib
./build.sh
```

`build.sh` 会依次执行：
1. 主机端编译并运行 `tests/test_hud_render.c`（`-Wall -Wextra -Werror`）；
2. 使用 `zig cc -target arm-linux-gnueabihf.2.15 -mcpu=cortex_a15+neon` 编译 `hud_probe`、`carplay-shim.so`、`rgd_listen`、`hud_bridge`；
3. 检查 ELF 架构（ARM EABI5 hard-float）与 GLIBC 符号版本（仅依赖 `GLIBC_2.4` / `2.7`）。

---

## 五、车机部署（先纯内存验证，确认无误再考虑持久化）

> **安全铁律**：**绝对不要在根分区 `/` 处于 `rw`（读写）状态时熄火断电！** 先使用纯内存方案验证。

### 方案 A：纯内存临时测试（熄火/重启即完全恢复原厂）

```sh
# 1. 上传 build/ 产物到车机 /tmp
scp build/hud_bridge build/carplay-shim.so build/hud_probe build/rgd_listen root@169.254.1.0:/tmp/
chmod 0755 /tmp/hud_bridge /tmp/carplay-shim.so /tmp/hud_probe /tmp/rgd_listen

# 2. 启动独立 hud_bridge systemd 单元（放 /run 内存，不落盘，不与 SSH 会话同 cgroup）
cp hud-bridge.service /run/systemd/system/hud-bridge.service
mkdir -p /run/systemd/system/CARPLAY.service.d
cp hud-shim.conf /run/systemd/system/CARPLAY.service.d/hud-shim.conf

systemctl daemon-reload
systemctl start hud-bridge.service
systemctl restart CARPLAY.service
```

停止并立即恢复原厂 `CARPLAY`：

```sh
rm -rf /run/systemd/system/CARPLAY.service.d /run/systemd/system/hud-bridge.service
systemctl daemon-reload
systemctl stop hud-bridge.service || true
systemctl restart CARPLAY.service
```

### 方案 B：受控持久化（开机自启）

- 实测 EntryNav2 `/var/local`（`/dev/mmcblk0p2`）挂载参数含 `noexec`，**不能**作为持久化可执行文件目录。
- 需将 `hud_bridge` 与 `carplay-shim.so` 放入独立目录 `/opt/hud-port/`，并将 `systemd/hud-bridge-persist.service`（`ExecStart=/opt/hud-port/hud_bridge`、`WantedBy=navigation.target`）与 `systemd/CARPLAY.service.d/hud-shim-persist.conf`（内置 `/tmp/.hud_shim_tries` **3 次启动崩溃自动回退原厂 `CARPLAY` 保护**）写入 `/etc/systemd/system/`。
- 写完后务必执行 `sync && mount -o remount,ro /`，通过 `awk '$2=="/" && $1!="rootfs"' /proc/mounts` 确认根分区回到 `ro`。

---

## 六、地图 App 实测表现

| 导航 App | 转向箭头/距离/中文路名 | 多车道引导 (`0x5204`) | 说明 |
|---|---|---|---|
| **Apple 地图** | 正常 | **支持**（接近复杂路口自动弹出 2~8 车道） | 完整下发 `0x5201` + `0x5202` + `0x5204` |
| **高德地图** | 正常 | **暂不显示** | 高德 iOS CarPlay 版目前仅在中控屏自绘车道线，**不通过 iAP2 下发 `0x5204`** |
| **百度地图** | 正常 | 协议支持 `0x5204` | 依赖版本下发情况 |
