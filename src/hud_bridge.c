/*
 * hud_bridge.c - CarPlay iAP2 RGD -> EntryNav2 MOST HUDTerminal (0x82/0x00) live bridge
 *
 * Receives iAP2 0x5201 / 0x5202 / 0x5203 / 0x5204 packets over /tmp/carplay_rgd.sock
 * from carplay-shim.so and renders maneuver icon (1.GuidingIcon 120x72), lane guidance
 * (1.LaneGuidance 184x17), distance, unit, and road name on MOST FBlock 0x82/0x00.
 */

#include <errno.h>
#include <poll.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

#include "hud_render.h"
#include "iap2_rgd.h"
#include "vmost_api.h"

/* Creation handles */
enum {
    H_FRAME = 1,
    H_DIST  = 2,
    H_UNIT  = 3,
    H_STREET = 4,
    H_ICON1 = 5,
    H_ICON2 = 6,
    H_LANE1 = 7,
    H_LANE2 = 8,
    H_MAX   = 16
};

static volatile sig_atomic_t g_running = 1;
static volatile sig_atomic_t g_term_sig = 0;
static const char * volatile g_stage = "init";
static FILE *g_log_fp = NULL;
static int g_force_ascii = 0;

static uint16_t g_created_inst[H_MAX];
static volatile int g_created_seen[H_MAX];
static volatile int g_mhp_ready = 0;
static volatile uint16_t g_queried_w = 0;
static volatile uint16_t g_queried_h = 0;
static volatile uint16_t g_queried_inst = 0;
static volatile uint64_t g_factory_busy_until_ms = 0;
static volatile int g_hud_error_seen = 0;

static uint16_t g_icon_w = 120;
static uint16_t g_icon_h = 72;
static uint16_t g_lane_w = 184;
static uint16_t g_lane_h = 17;

typedef struct {
    int      open;
    int      shadows_reg;
    uint16_t dev;
    uint16_t frame_id;
    uint16_t dist_id;
    uint16_t unit_id;
    uint16_t street_id;
    uint16_t icon1_id;
    uint16_t icon2_id;
    uint16_t lane1_id;
    uint16_t lane2_id;
    uint16_t icon_front;
    uint16_t lane_front;
    uint8_t  lane_was_visible;
    uint32_t last_hash;
    uint64_t last_keepalive_ms;
    char     last_dist[32];
    char     last_unit[16];
    char     last_street[HUD_STR_MAX];
} HudSession;

static HudSession g_hud;

static uint64_t now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)ts.tv_nsec / 1000000ULL;
}

static void bridge_log(const char *fmt, ...) {
    uint64_t ms = now_ms();
    va_list ap;
    va_start(ap, fmt);
    printf("[%llu.%03llu] ", (unsigned long long)(ms / 1000ULL), (unsigned long long)(ms % 1000ULL));
    vprintf(fmt, ap);
    putchar('\n');
    fflush(stdout);
    va_end(ap);

    if (g_log_fp) {
        va_start(ap, fmt);
        fprintf(g_log_fp, "[%llu.%03llu] ", (unsigned long long)(ms / 1000ULL), (unsigned long long)(ms % 1000ULL));
        vfprintf(g_log_fp, fmt, ap);
        fputc('\n', g_log_fp);
        fflush(g_log_fp);
        va_end(ap);
    }
}

static void on_term_sig(int sig) {
    g_term_sig = sig;
    g_running = 0;
}

static void on_crash_sig(int sig) {
    fprintf(stderr, "\n[hud_bridge CRASH] sig=%d stage=%s\n", sig, g_stage ? g_stage : "?");
    fflush(stderr);
    _exit(128 + sig);
}

static void put_be16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)(v & 0xFF);
}

static uint16_t get_be16(const uint8_t *p) {
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static int is_our_handle(uint16_t h) {
    return h >= H_FRAME && h <= H_LANE2;
}

static void on_net_event(uint32_t net_event, void *user_data) {
    (void)user_data;
    bridge_log("[NET_EVENT] code=0x%08x", net_event);
}

static void on_most_msg(const VMost_Msg_t *msg, void *user_data) {
    (void)user_data;
    if (!msg) return;

    if (msg->fblock_id == 0xF1 && msg->inst_id == 0x06 &&
        (msg->func_id == 0x0001 || msg->func_id == 0x0005) &&
        msg->op_type == VMOST_OP_STATUS && msg->data_len >= 8) {
        g_mhp_ready = 1;
        return;
    }

    if (msg->fblock_id == HUD_FBLOCK_ID) {
        if (msg->op_type == VMOST_OP_ERROR || msg->op_type == 0x09) {
            g_hud_error_seen = 1;
            bridge_log("[HUD_ERR] func=0x%04x op=0x%02x len=%u", msg->func_id, msg->op_type, msg->data_len);
            return;
        }
        if (msg->func_id == HUD_FUNC_CREATE_INSTANCE &&
            (msg->op_type == 0x0D || msg->op_type == 0x0C) &&
            msg->data && msg->data_len >= 4) {
            uint16_t handle = get_be16(msg->data);
            uint16_t inst_id = get_be16(msg->data + 2);
            if (is_our_handle(handle)) {
                g_created_inst[handle] = inst_id;
                g_created_seen[handle] = 1;
                bridge_log("[HUD_RX] CREATE handle=%u -> inst=0x%04x", (unsigned)handle, (unsigned)inst_id);
            } else if (!g_hud.open) {
                /* Another client (factory EBNavi) created a HUD instance */
                g_factory_busy_until_ms = now_ms() + 8000ULL;
            }
            return;
        }
        if (msg->func_id == HUD_FUNC_IMAGE_GET_SIZE &&
            (msg->op_type == 0x0D || msg->op_type == 0x0C) &&
            msg->data && msg->data_len >= 6) {
            g_queried_inst = get_be16(msg->data);
            g_queried_w = get_be16(msg->data + 2);
            g_queried_h = get_be16(msg->data + 4);
            if (g_queried_w > 0 && g_queried_h > 0) {
                if (g_queried_inst == g_hud.icon1_id || g_queried_inst == g_hud.icon2_id) {
                    g_icon_w = g_queried_w;
                    g_icon_h = g_queried_h;
                } else if (g_queried_inst == g_hud.lane1_id || g_queried_inst == g_hud.lane2_id) {
                    g_lane_w = g_queried_w;
                    g_lane_h = g_queried_h;
                }
            }
            bridge_log("[HUD_RX] SIZE inst=0x%04x -> %ux%u",
                       (unsigned)g_queried_inst, (unsigned)g_queried_w, (unsigned)g_queried_h);
        }
    }
}

static int send_msg_type(uint16_t dev_addr, uint8_t fb, uint8_t inst, uint16_t func, uint32_t op,
                         uint32_t msg_type, const uint8_t *payload, uint16_t len) {
    static uint8_t empty_buf[4] = { 0, 0, 0, 0 };
    VMost_Msg_t m;
    memset(&m, 0, sizeof(m));
    m.dev_addr = dev_addr;
    m.fblock_id = fb;
    m.inst_id = inst;
    m.func_id = func;
    m.op_type = op;
    m.data_len = len;
    m.data = (payload && len > 0) ? (uint8_t *)payload : empty_buf;
    m.reserved_14 = msg_type;
    return vmostMostMsgSend(&m);
}

static int mhp_is_ready(void) {
    return g_mhp_ready != 0;
}

static int all_core_created(void) {
    return g_created_seen[H_FRAME] && g_created_seen[H_DIST] &&
           g_created_seen[H_UNIT] && g_created_seen[H_STREET] &&
           g_created_seen[H_ICON1];
}

static int all_all_created(void) {
    return all_core_created() && g_created_seen[H_ICON2] &&
           g_created_seen[H_LANE1] && g_created_seen[H_LANE2];
}

static void pump_until(int vmost_fd, int duration_ms, int (*pred)(void)) {
    uint64_t end = now_ms() + (uint64_t)duration_ms;
    while (g_running && now_ms() < end) {
        if (pred && pred()) break;
        int rem = (int)(end - now_ms());
        if (rem <= 0) break;
        struct pollfd pfd = { .fd = vmost_fd, .events = POLLIN, .revents = 0 };
        int rc = poll(&pfd, 1, rem > 50 ? 50 : rem);
        if (rc > 0 && (pfd.revents & POLLIN)) {
            if (vmostProcessMessages() != 0) break;
        } else if (rc < 0 && errno != EINTR) {
            break;
        }
    }
}

static int send_mhp_conn(uint16_t hud_dev, int open) {
    uint8_t buf[7];
    put_be16(buf, hud_dev);
    buf[2] = HUD_FBLOCK_ID;
    buf[3] = HUD_INST_ID;
    put_be16(buf + 4, HUD_FUNC_TERMINAL_INTERFACE);
    buf[6] = 0x00;
    return send_msg_type(0x0100, 0xF1, 0x05, 0x000C, open ? VMOST_OP_SET : VMOST_OP_GET,
                         0, buf, sizeof(buf));
}

static int send_hud_create(uint16_t dev, uint16_t handle, uint8_t type, uint8_t sub, const char *name) {
    uint8_t buf[128];
    size_t nlen = strlen(name);
    if (nlen > 64) nlen = 64;
    put_be16(buf, handle);
    buf[2] = type;
    buf[3] = sub;
    buf[4] = 0x01;
    memcpy(buf + 5, name, nlen);
    buf[5 + nlen] = 0x00;
    return send_msg_type(dev, HUD_FBLOCK_ID, HUD_INST_ID, HUD_FUNC_CREATE_INSTANCE,
                         VMOST_OP_START_RES_ACK, 2, buf, (uint16_t)(6 + nlen));
}

static int send_hud_add_item(uint16_t dev, uint16_t handle, uint16_t frame_id, uint8_t pos, uint16_t item_id) {
    uint8_t buf[7];
    put_be16(buf, handle);
    put_be16(buf + 2, frame_id);
    buf[4] = pos;
    put_be16(buf + 5, item_id);
    return send_msg_type(dev, HUD_FBLOCK_ID, HUD_INST_ID, HUD_FUNC_FRAME_ADD_ITEM,
                         VMOST_OP_START_RES_ACK, 2, buf, sizeof(buf));
}

static int send_hud_set_vis(uint16_t dev, uint16_t inst_id, uint8_t vis) {
    uint8_t buf[3];
    put_be16(buf, inst_id);
    buf[2] = vis;
    return send_msg_type(dev, HUD_FBLOCK_ID, HUD_INST_ID, HUD_FUNC_SET_VISIBILITY,
                         VMOST_OP_START_RES_ACK, 2, buf, sizeof(buf));
}

static int send_hud_switch_vis(uint16_t dev, uint16_t inst1, uint16_t inst2) {
    uint8_t buf[4];
    put_be16(buf, inst1);
    put_be16(buf + 2, inst2);
    return send_msg_type(dev, HUD_FBLOCK_ID, HUD_INST_ID, HUD_FUNC_SWITCH_VISIBILITY,
                         VMOST_OP_START_RES_ACK, 2, buf, sizeof(buf));
}

static int send_hud_release(uint16_t dev, uint16_t inst_id) {
    uint8_t buf[2];
    put_be16(buf, inst_id);
    return send_msg_type(dev, HUD_FBLOCK_ID, HUD_INST_ID, HUD_FUNC_RELEASE_INSTANCE,
                         VMOST_OP_START_RES_ACK, 2, buf, sizeof(buf));
}

static int send_hud_text(uint16_t dev, uint16_t inst_id, const char *text) {
    uint8_t buf[260];
    put_be16(buf, inst_id);
    size_t enc_len = hud_encode_most_string(text, g_force_ascii, buf + 2, sizeof(buf) - 2);
    return send_msg_type(dev, HUD_FBLOCK_ID, HUD_INST_ID, HUD_FUNC_TEXTAREA_SET_TEXT,
                         VMOST_OP_START_RES_ACK, 2, buf, (uint16_t)(2 + enc_len));
}

static int send_hud_u16_u8(uint16_t dev, uint16_t func, uint32_t op, uint16_t inst_id, uint8_t val) {
    uint8_t buf[3];
    put_be16(buf, inst_id);
    buf[2] = val;
    return send_msg_type(dev, HUD_FBLOCK_ID, HUD_INST_ID, func, op, 2, buf, sizeof(buf));
}

static int send_hud_u16(uint16_t dev, uint16_t func, uint32_t op, uint16_t inst_id) {
    uint8_t buf[2];
    put_be16(buf, inst_id);
    return send_msg_type(dev, HUD_FBLOCK_ID, HUD_INST_ID, func, op, 2, buf, sizeof(buf));
}

static int send_hud_polygon(uint16_t dev, uint16_t inst_id, const HudPolygonCmd *cmd) {
    uint8_t buf[512];
    if (!cmd || cmd->num_pts < 2 || 6 + cmd->num_pts * 4 > (int)sizeof(buf)) return -1;
    put_be16(buf, inst_id);
    put_be16(buf + 2, 0x0000);
    buf[4] = cmd->poly_mode;
    buf[5] = cmd->num_pts;
    for (uint8_t i = 0; i < cmd->num_pts; i++) {
        put_be16(buf + 6 + i * 4, cmd->xy[i * 2]);
        put_be16(buf + 6 + i * 4 + 2, cmd->xy[i * 2 + 1]);
    }
    return send_msg_type(dev, HUD_FBLOCK_ID, HUD_INST_ID, HUD_FUNC_IMAGE_ADD_POLYGON,
                         VMOST_OP_START_RES_ACK, 2, buf, (uint16_t)(6 + cmd->num_pts * 4));
}

static void draw_poly_list(uint16_t dev, uint16_t inst_id,
                           const HudPolygonCmd *polys, uint8_t count) {
    send_hud_u16(dev, HUD_FUNC_IMAGE_REMOVE_ALL_ITEMS, VMOST_OP_START_RES_ACK, inst_id);
    int cur_lw = -1, cur_lc = -1, cur_fm = -1, cur_fc = -1;
    for (uint8_t i = 0; i < count; i++) {
        const HudPolygonCmd *p = &polys[i];
        if ((int)p->line_width != cur_lw) {
            send_hud_u16_u8(dev, HUD_FUNC_IMAGE_SET_LINE_WIDTH, VMOST_OP_START_RES_ACK, inst_id, p->line_width);
            cur_lw = p->line_width;
        }
        if ((int)p->color != cur_lc) {
            send_hud_u16_u8(dev, HUD_FUNC_IMAGE_SET_LINE_COLOR, VMOST_OP_SET, inst_id, p->color);
            cur_lc = p->color;
        }
        if ((int)p->fill_mode != cur_fm) {
            send_hud_u16_u8(dev, HUD_FUNC_IMAGE_SET_FILL_MODE, VMOST_OP_START_RESULT, inst_id, p->fill_mode);
            cur_fm = p->fill_mode;
        }
        if ((int)p->fill_color != cur_fc) {
            send_hud_u16_u8(dev, HUD_FUNC_IMAGE_SET_FILL_COLOR, VMOST_OP_START_RESULT, inst_id, p->fill_color);
            cur_fc = p->fill_color;
        }
        send_hud_polygon(dev, inst_id, p);
    }
}

static uint16_t find_hud_dev(void) {
    VMost_FBlock_Registration_t *regs = calloc(1024, sizeof(VMost_FBlock_Registration_t));
    uint32_t cnt = 0;
    uint16_t found = 0;
    if (regs && vmostGetRegisteredFblocks(regs, &cnt) == 0) {
        for (uint32_t i = 0; i < cnt && i < 1024; i++) {
            if (regs[i].fblock_id == HUD_FBLOCK_ID && regs[i].inst_id == HUD_INST_ID &&
                regs[i].dev_addr != 0 && regs[i].dev_addr != 0xFFFF) {
                found = regs[i].dev_addr;
                break;
            }
        }
    }
    free(regs);
    return found ? found : 0x0101;
}

static void hud_session_close(int vmost_fd) {
    if (!g_hud.open && !g_hud.shadows_reg) return;
    g_stage = "hud_session_close";
    uint16_t dev = g_hud.dev ? g_hud.dev : 0x0101;

    if (g_hud.open) {
        if (g_hud.frame_id) send_hud_set_vis(dev, g_hud.frame_id, 0);
        const uint16_t rel_ids[] = {
            g_hud.lane2_id, g_hud.lane1_id,
            g_hud.icon2_id, g_hud.icon1_id,
            g_hud.street_id, g_hud.unit_id, g_hud.dist_id,
            g_hud.frame_id
        };
        int any = 0;
        for (size_t i = 0; i < sizeof(rel_ids) / sizeof(rel_ids[0]); i++) {
            if (rel_ids[i]) {
                send_hud_release(dev, rel_ids[i]);
                any = 1;
            }
        }
        if (any) {
            vmostMostPacketizerFlush(dev, HUD_FBLOCK_ID, HUD_INST_ID, HUD_FUNC_TERMINAL_INTERFACE, 0);
            if (vmost_fd >= 0) pump_until(vmost_fd, 400, NULL);
        }
        send_mhp_conn(dev, 0);
        if (vmost_fd >= 0) pump_until(vmost_fd, 200, NULL);
    }

    if (g_hud.shadows_reg) {
        vmostFblockShadowDeregister(HUD_FBLOCK_ID, HUD_INST_ID);
        vmostFblockShadowDeregister(0xF1, 0x06);
        vmostFblockShadowDeregister(0xF1, 0x00);
    }
    memset(&g_hud, 0, sizeof(g_hud));
    bridge_log("HUD session closed cleanly");
}

static int hud_session_open(int vmost_fd, uint16_t dev_override) {
    g_stage = "hud_session_open";
    memset(&g_hud, 0, sizeof(g_hud));
    memset((void *)g_created_inst, 0, sizeof(g_created_inst));
    memset((void *)g_created_seen, 0, sizeof(g_created_seen));
    g_mhp_ready = 0;
    g_hud_error_seen = 0;

    uint16_t dev = dev_override ? dev_override : find_hud_dev();
    g_hud.dev = dev;

    int s1 = vmostFblockShadowRegister(0xF1, 0x06);
    int s2 = vmostFblockShadowRegister(0xF1, 0x00);
    int s3 = vmostFblockShadowRegister(HUD_FBLOCK_ID, HUD_INST_ID);
    g_hud.shadows_reg = 1;
    bridge_log("Opening HUD session dev=0x%04x shadows=(%d,%d,%d)", (unsigned)dev, s1, s2, s3);
    pump_until(vmost_fd, 250, NULL);

    send_mhp_conn(dev, 1);
    pump_until(vmost_fd, 300, mhp_is_ready);
    bridge_log("MHP ready=%d", (int)g_mhp_ready);

    for (int attempt = 0; attempt < 2 && !all_all_created(); attempt++) {
        if (attempt > 0) {
            bridge_log("Retrying missing HUD CREATE_INSTANCE handles (attempt=%d)...", attempt + 1);
            g_mhp_ready = 0;
            send_mhp_conn(dev, 1);
            pump_until(vmost_fd, 800, mhp_is_ready);
        }
        if (!g_created_seen[H_FRAME])  send_hud_create(dev, H_FRAME,  HUD_TYPE_FRAME,    0x00, "TopFrame");
        if (!g_created_seen[H_DIST])   send_hud_create(dev, H_DIST,   HUD_TYPE_TEXTAREA, 0x00, "Distance");
        if (!g_created_seen[H_UNIT])   send_hud_create(dev, H_UNIT,   HUD_TYPE_TEXTAREA, 0x00, "Unit");
        if (!g_created_seen[H_STREET]) send_hud_create(dev, H_STREET, HUD_TYPE_TEXTAREA, 0x00, "ComingStreet");
        if (!g_created_seen[H_ICON1])  send_hud_create(dev, H_ICON1,  HUD_TYPE_IMAGE,    0x00, "1.GuidingIcon");
        if (!g_created_seen[H_ICON2])  send_hud_create(dev, H_ICON2,  HUD_TYPE_IMAGE,    0x00, "2.GuidingIcon");
        if (!g_created_seen[H_LANE1])  send_hud_create(dev, H_LANE1,  HUD_TYPE_IMAGE,    0x00, "1.LaneGuidance");
        if (!g_created_seen[H_LANE2])  send_hud_create(dev, H_LANE2,  HUD_TYPE_IMAGE,    0x00, "2.LaneGuidance");

        int frc = vmostMostPacketizerFlush(dev, HUD_FBLOCK_ID, HUD_INST_ID, HUD_FUNC_TERMINAL_INTERFACE, 0);
        if (frc != 0) bridge_log("Flush(create) -> %d (%s)", frc, vmostErrToString(frc));
        pump_until(vmost_fd, 3500, all_all_created);
        if (all_core_created() && attempt == 0 && !all_all_created()) {
            /* Brief extra settle for optional 2nd buffer / lanes */
            pump_until(vmost_fd, 400, all_all_created);
            break;
        }
    }

    g_hud.frame_id  = g_created_seen[H_FRAME]  ? g_created_inst[H_FRAME]  : 0;
    g_hud.dist_id   = g_created_seen[H_DIST]   ? g_created_inst[H_DIST]   : 0;
    g_hud.unit_id   = g_created_seen[H_UNIT]   ? g_created_inst[H_UNIT]   : 0;
    g_hud.street_id = g_created_seen[H_STREET] ? g_created_inst[H_STREET] : 0;
    g_hud.icon1_id  = g_created_seen[H_ICON1]  ? g_created_inst[H_ICON1]  : 0;
    g_hud.icon2_id  = g_created_seen[H_ICON2]  ? g_created_inst[H_ICON2]  : 0;
    g_hud.lane1_id  = g_created_seen[H_LANE1]  ? g_created_inst[H_LANE1]  : 0;
    g_hud.lane2_id  = g_created_seen[H_LANE2]  ? g_created_inst[H_LANE2]  : 0;
    g_hud.open = 1;

    if (!all_core_created()) {
        bridge_log("HUD open FAILED: missing core InstanceIDs (frame=%u dist=%u unit=%u street=%u icon1=%u)",
                   g_hud.frame_id, g_hud.dist_id, g_hud.unit_id, g_hud.street_id, g_hud.icon1_id);
        hud_session_close(vmost_fd);
        return -1;
    }

    send_hud_add_item(dev, 20, g_hud.frame_id, 0x14, g_hud.dist_id);
    send_hud_add_item(dev, 21, g_hud.frame_id, 0x14, g_hud.unit_id);
    send_hud_add_item(dev, 22, g_hud.frame_id, 0x14, g_hud.street_id);
    send_hud_add_item(dev, 23, g_hud.frame_id, 0x14, g_hud.icon1_id);
    if (g_hud.icon2_id) send_hud_add_item(dev, 24, g_hud.frame_id, 0x14, g_hud.icon2_id);
    if (g_hud.lane1_id) send_hud_add_item(dev, 25, g_hud.frame_id, 0x14, g_hud.lane1_id);
    if (g_hud.lane2_id) send_hud_add_item(dev, 26, g_hud.frame_id, 0x14, g_hud.lane2_id);

    send_hud_u16(dev, HUD_FUNC_IMAGE_GET_SIZE, VMOST_OP_START_RES_ACK, g_hud.icon1_id);
    if (g_hud.lane1_id) {
        send_hud_u16(dev, HUD_FUNC_IMAGE_GET_SIZE, VMOST_OP_START_RES_ACK, g_hud.lane1_id);
    }

    send_hud_set_vis(dev, g_hud.dist_id, 1);
    send_hud_set_vis(dev, g_hud.unit_id, 1);
    send_hud_set_vis(dev, g_hud.street_id, 1);
    send_hud_set_vis(dev, g_hud.icon1_id, 1);
    if (g_hud.icon2_id) send_hud_set_vis(dev, g_hud.icon2_id, 0);
    if (g_hud.lane1_id) send_hud_set_vis(dev, g_hud.lane1_id, 0);
    if (g_hud.lane2_id) send_hud_set_vis(dev, g_hud.lane2_id, 0);

    g_hud.icon_front = g_hud.icon1_id;
    g_hud.lane_front = g_hud.lane1_id;
    g_hud.last_keepalive_ms = now_ms();
    vmostMostPacketizerFlush(dev, HUD_FBLOCK_ID, HUD_INST_ID, HUD_FUNC_TERMINAL_INTERFACE, 0);
    pump_until(vmost_fd, 150, NULL);

    bridge_log("HUD ready: frame=0x%04x icon=(0x%04x,0x%04x %ux%u) lane=(0x%04x,0x%04x %ux%u)",
               g_hud.frame_id, g_hud.icon1_id, g_hud.icon2_id, g_icon_w, g_icon_h,
               g_hud.lane1_id, g_hud.lane2_id, g_lane_w, g_lane_h);
    return 0;
}

static void hud_session_render_frame(int vmost_fd, const HudRenderFrame *frame) {
    if (!g_hud.open || !frame || !frame->visible) return;
    if (frame->content_hash == g_hud.last_hash) return;
    g_stage = "hud_session_render";

    uint16_t dev = g_hud.dev;

    /* GuidingIcon: draw on back buffer and swap (or hide-draw-show if single buffer) */
    uint16_t icon_back = (g_hud.icon2_id && g_hud.icon_front == g_hud.icon1_id)
                             ? g_hud.icon2_id
                             : g_hud.icon1_id;
    int icon_double = (g_hud.icon2_id != 0 && icon_back != g_hud.icon_front);
    if (!icon_double) {
        send_hud_set_vis(dev, icon_back, 0);
    }
    draw_poly_list(dev, icon_back, frame->icon_polys, frame->icon_poly_count);
    if (icon_double) {
        send_hud_switch_vis(dev, g_hud.icon_front, icon_back);
        g_hud.icon_front = icon_back;
    } else {
        send_hud_set_vis(dev, icon_back, 1);
    }

    /* LaneGuidance (184x17) */
    if (g_hud.lane1_id) {
        if (frame->lane_visible && frame->lane_poly_count > 0) {
            uint16_t lane_back = (g_hud.lane2_id && g_hud.lane_front == g_hud.lane1_id)
                                     ? g_hud.lane2_id
                                     : g_hud.lane1_id;
            int lane_double = (g_hud.lane2_id != 0 && g_hud.lane_was_visible && lane_back != g_hud.lane_front);
            if (!lane_double && g_hud.lane_was_visible) {
                send_hud_set_vis(dev, lane_back, 0);
            }
            draw_poly_list(dev, lane_back, frame->lane_polys, frame->lane_poly_count);
            if (lane_double) {
                send_hud_switch_vis(dev, g_hud.lane_front, lane_back);
                g_hud.lane_front = lane_back;
            } else {
                if (g_hud.lane2_id) {
                    uint16_t other = (lane_back == g_hud.lane1_id) ? g_hud.lane2_id : g_hud.lane1_id;
                    send_hud_set_vis(dev, other, 0);
                }
                send_hud_set_vis(dev, lane_back, 1);
                g_hud.lane_front = lane_back;
            }
            g_hud.lane_was_visible = 1;
        } else if (g_hud.lane_was_visible) {
            send_hud_set_vis(dev, g_hud.lane1_id, 0);
            if (g_hud.lane2_id) send_hud_set_vis(dev, g_hud.lane2_id, 0);
            g_hud.lane_was_visible = 0;
        }
    }

    /* Update text fields only when changed */
    if (strcmp(g_hud.last_dist, frame->dist_text) != 0) {
        send_hud_text(dev, g_hud.dist_id, frame->dist_text);
        snprintf(g_hud.last_dist, sizeof(g_hud.last_dist), "%s", frame->dist_text);
    }
    if (strcmp(g_hud.last_unit, frame->unit_text) != 0) {
        send_hud_text(dev, g_hud.unit_id, frame->unit_text);
        snprintf(g_hud.last_unit, sizeof(g_hud.last_unit), "%s", frame->unit_text);
    }
    if (strcmp(g_hud.last_street, frame->street_text) != 0) {
        send_hud_text(dev, g_hud.street_id, frame->street_text);
        snprintf(g_hud.last_street, sizeof(g_hud.last_street), "%s", frame->street_text);
    }

    if (g_hud.last_hash == 0) {
        send_hud_set_vis(dev, g_hud.frame_id, 1);
    }

    int frc = vmostMostPacketizerFlush(dev, HUD_FBLOCK_ID, HUD_INST_ID, HUD_FUNC_TERMINAL_INTERFACE, 0);
    if (frc != 0) {
        bridge_log("Flush(render) -> %d (%s)", frc, vmostErrToString(frc));
    }
    g_hud.last_hash = frame->content_hash;
    g_hud.last_keepalive_ms = now_ms();
    pump_until(vmost_fd, 60, NULL);
}

static void hud_session_keepalive(int vmost_fd, uint64_t tnow) {
    if (!g_hud.open || g_hud.frame_id == 0) return;
    uint16_t dev = g_hud.dev;
    send_hud_set_vis(dev, g_hud.frame_id, 1);
    if (g_hud.icon_front) send_hud_set_vis(dev, g_hud.icon_front, 1);
    if (g_hud.lane_was_visible && g_hud.lane_front) {
        send_hud_set_vis(dev, g_hud.lane_front, 1);
    }
    int frc = vmostMostPacketizerFlush(dev, HUD_FBLOCK_ID, HUD_INST_ID, HUD_FUNC_TERMINAL_INTERFACE, 0);
    if (frc != 0) {
        bridge_log("Flush(keepalive) -> %d (%s)", frc, vmostErrToString(frc));
    }
    g_hud.last_keepalive_ms = tnow;
    pump_until(vmost_fd, 30, NULL);
}

static int bind_rgd_socket(void) {
    unlink(CARPLAY_RGD_SOCK_PATH);
    int fd = socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK, 0);
    if (fd < 0) {
        perror("socket");
        return -1;
    }
    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", CARPLAY_RGD_SOCK_PATH);
    if (bind(fd, (const struct sockaddr *)&addr, sizeof(addr)) != 0) {
        perror("bind");
        close(fd);
        return -1;
    }
    chmod(CARPLAY_RGD_SOCK_PATH, 0666);
    return fd;
}

static void print_usage(const char *prog) {
    printf("Usage:\n"
           "  %s [--dev 0x0101] [--ascii] [--log /tmp/hud_bridge.log]\n"
           "  %s --test-lanes [--dev 0x0101] [--ascii] [street_text]\n",
           prog, prog);
}

int main(int argc, char **argv) {
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);
    signal(SIGSEGV, on_crash_sig);
    signal(SIGABRT, on_crash_sig);
    signal(SIGHUP, SIG_IGN);
    signal(SIGPIPE, SIG_IGN);

    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_term_sig;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    uint16_t dev_override = 0;
    const char *log_path = NULL;
    int test_lanes = 0;
    const char *test_street = "HUD 车道引导测试";

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--dev") == 0 && i + 1 < argc) {
            dev_override = (uint16_t)strtoul(argv[++i], NULL, 16);
        } else if (strcmp(argv[i], "--ascii") == 0) {
            g_force_ascii = 1;
        } else if (strcmp(argv[i], "--log") == 0 && i + 1 < argc) {
            log_path = argv[++i];
        } else if (strcmp(argv[i], "--test-lanes") == 0) {
            test_lanes = 1;
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                test_street = argv[++i];
            }
        } else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            print_usage(argv[0]);
            return 0;
        }
    }

    if (log_path) {
        g_log_fp = fopen(log_path, "w");
    }

    LogDefaultDomainSet("hud_bridge");
    LogSetDLTApplicationID("HUDB", "CarPlay HUD Bridge");

    int sock_fd = -1;
    if (!test_lanes) {
        sock_fd = bind_rgd_socket();
        if (sock_fd < 0) return 1;
        bridge_log("Listening on %s (ascii=%d)", CARPLAY_RGD_SOCK_PATH, g_force_ascii);
    }

    int rc = vmostConnect(on_most_msg, NULL, 0);
    if (rc != 0) {
        bridge_log("vmostConnect failed: %d (%s)", rc, vmostErrToString(rc));
        if (sock_fd >= 0) {
            close(sock_fd);
            unlink(CARPLAY_RGD_SOCK_PATH);
        }
        return 1;
    }
    vmostRegisterToEventsExt(on_net_event);
    int vmost_fd = -1;
    vmostFileDescriptorGet(&vmost_fd);
    bridge_log("Connected to vmostd: fd=%d", vmost_fd);

    if (test_lanes) {
        if (g_force_ascii && strcmp(test_street, "HUD 车道引导测试") == 0) {
            test_street = "Lane Test";
        }
        if (hud_session_open(vmost_fd, dev_override) == 0) {
            HudRenderFrame frame;
            hud_render_build_test_frame(g_icon_w, g_icon_h, g_lane_w, g_lane_h, test_street, &frame);
            bridge_log("Drawing test-lanes frame: icon_polys=%u lane_polys=%u street='%s'",
                       (unsigned)frame.icon_poly_count, (unsigned)frame.lane_poly_count, frame.street_text);
            hud_session_render_frame(vmost_fd, &frame);
            pump_until(vmost_fd, 8000, NULL);
            hud_session_close(vmost_fd);
        }
        vmostDisconnect();
        if (g_log_fp) fclose(g_log_fp);
        return 0;
    }

    RgdState st;
    rgd_state_init(&st);
    uint64_t next_retry_ms = 0;
    uint64_t last_rgd_log_ms = 0;
    uint32_t rx5201_cnt = 0, rx5202_cnt = 0, rx5204_cnt = 0;
    uint8_t  prev_log_rs = 0xFF, prev_log_supp = 0xFF, prev_log_lane_show = 0xFF;
    uint16_t prev_log_man_idx = 0xFFFF;
    uint32_t prev_log_5204 = 0;

    while (g_running) {
        struct pollfd pfds[2];
        nfds_t nfds = 0;
        pfds[nfds].fd = sock_fd;
        pfds[nfds].events = POLLIN;
        pfds[nfds].revents = 0;
        nfds++;
        if (vmost_fd >= 0) {
            pfds[nfds].fd = vmost_fd;
            pfds[nfds].events = POLLIN;
            pfds[nfds].revents = 0;
            nfds++;
        }

        g_stage = "main_poll";
        int prc = poll(pfds, nfds, 100);
        if (prc < 0 && errno != EINTR) break;

        uint64_t tnow = now_ms();
        int state_dirty = 0;

        if (prc > 0 && (pfds[0].revents & POLLIN)) {
            uint8_t buf[4096];
            for (int burst = 0; burst < 32; burst++) {
                ssize_t n = recv(sock_fd, buf, sizeof(buf), MSG_DONTWAIT);
                if (n <= 0) break;
                if (n >= 6 && buf[0] == 0x40 && buf[1] == 0x40) {
                    uint16_t mid = ((uint16_t)buf[4] << 8) | buf[5];
                    if (mid == IAP2_MSG_ROUTE_GUIDANCE_UPDATE) rx5201_cnt++;
                    else if (mid == IAP2_MSG_ROUTE_GUIDANCE_MANEUVER) rx5202_cnt++;
                    else if (mid == IAP2_MSG_LANE_GUIDANCE_INFO) rx5204_cnt++;
                }
                if (rgd_state_ingest(&st, buf, (size_t)n, tnow)) {
                    state_dirty = 1;
                }
            }
        }

        if (state_dirty) {
            int changed = (st.route_state != prev_log_rs ||
                           st.source_supports_rg != prev_log_supp ||
                           st.man_current_idx != prev_log_man_idx ||
                           st.lane_guidance_showing != prev_log_lane_show ||
                           (rx5204_cnt != prev_log_5204 && (prev_log_5204 == 0 || rx5204_cnt - prev_log_5204 >= 10)));
            if (changed || (tnow - last_rgd_log_ms) >= 60000ULL) {
                bridge_log("[RGD] rs=%u supp=%u(%u) man=%u/%u lane=%u(idx=%u,has=%u) dist=%u rx=(%u,%u,%u)",
                           (unsigned)st.route_state, (unsigned)st.source_supports_rg,
                           (unsigned)st.has_source_supports_rg,
                           (unsigned)st.man_current_idx, (unsigned)st.man_count,
                           (unsigned)st.lane_guidance_showing, (unsigned)st.lane_guidance_idx,
                           (unsigned)st.has_lane_idx, (unsigned)st.dist_maneuver_m,
                           (unsigned)rx5201_cnt, (unsigned)rx5202_cnt, (unsigned)rx5204_cnt);
                prev_log_rs = st.route_state;
                prev_log_supp = st.source_supports_rg;
                prev_log_man_idx = st.man_current_idx;
                prev_log_lane_show = st.lane_guidance_showing;
                prev_log_5204 = rx5204_cnt;
                last_rgd_log_ms = tnow;
            }
        }

        if (nfds > 1 && prc > 0 && (pfds[1].revents & POLLIN)) {
            if (vmostProcessMessages() != 0) {
                bridge_log("vmostProcessMessages error; closing HUD session");
                hud_session_close(vmost_fd);
                break;
            }
        }

        if (g_hud_error_seen && g_hud.open) {
            bridge_log("HUD ErrorAck seen; yielding 5s in case factory nav took 0x82/0x00");
            g_hud_error_seen = 0;
            g_factory_busy_until_ms = tnow + 5000ULL;
            hud_session_close(vmost_fd);
        }

        int should_show = rgd_state_should_show(&st, tnow);
        if (should_show && tnow < g_factory_busy_until_ms) {
            should_show = 0;
        }

        if (should_show) {
            if (!g_hud.open && tnow >= next_retry_ms) {
                if (hud_session_open(vmost_fd, dev_override) != 0) {
                    next_retry_ms = tnow + 3000ULL;
                }
            }
            if (g_hud.open) {
                HudRenderFrame frame;
                hud_render_build_frame(&st, tnow, g_icon_w, g_icon_h, g_lane_w, g_lane_h, &frame);
                if (frame.visible && (state_dirty || frame.content_hash != g_hud.last_hash)) {
                    hud_session_render_frame(vmost_fd, &frame);
                } else if (frame.visible && (tnow - g_hud.last_keepalive_ms) >= 15000ULL) {
                    hud_session_keepalive(vmost_fd, tnow);
                }
            }
        } else if (g_hud.open) {
            bridge_log("Guidance inactive (route_state=%u stopped=%u rx=(%u,%u,%u)); releasing HUD",
                       (unsigned)st.route_state, (unsigned)st.stopped_explicit,
                       (unsigned)rx5201_cnt, (unsigned)rx5202_cnt, (unsigned)rx5204_cnt);
            hud_session_close(vmost_fd);
        }
    }

    bridge_log("Shutting down hud_bridge (sig=%d)...", (int)g_term_sig);
    hud_session_close(vmost_fd);
    vmostDisconnect();
    close(sock_fd);
    unlink(CARPLAY_RGD_SOCK_PATH);
    if (g_log_fp) fclose(g_log_fp);
    return 0;
}
