/*
 * hud_probe.c - EntryNav2 MOST HUDTerminal (0x82/0x00) non-destructive probe
 *
 * Step 2 verification tool:
 *   1. Connects to /var/run/vmost_sockstream (use_glib_mainloop = 0, poll()-driven)
 *   2. Queries own MOST addr, MPR, registered FBlocks & Shadows (checks 0x82/0x00)
 *   3. --probe-read : registers Shadow 0x82/0x00, queries FrameIsVisible(0x205) /
 *      ImageGetSize(0x220) / C04 with target address, logs every reply/ErrorAck
 *   4. --inject <dev.fb.inst.func.op[.hex]> : single raw telegram test
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <poll.h>
#include <time.h>
#include <signal.h>

#include "vmost_api.h"

static volatile int g_rx_count = 0;
static const char * volatile g_stage = "init";
static uint16_t g_created_inst[16];
static volatile int g_created_seen[16];
static volatile int g_mhp_ready = 0;

static void put_be16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)(v & 0xFF);
}

static uint16_t get_be16(const uint8_t *p) {
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static void on_crash_sig(int sig) {
    fprintf(stderr, "\n[CRASH] sig=%d stage=%s rx_count=%d\n", sig, g_stage ? g_stage : "?", g_rx_count);
    fflush(stderr);
    _exit(128 + sig);
}

static uint64_t now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)ts.tv_nsec / 1000000ULL;
}

static void on_net_event(uint32_t net_event, void *user_data) {
    (void)user_data;
    printf("[NET_EVENT] code=0x%08x\n", net_event);
    fflush(stdout);
}

static void on_most_msg(const VMost_Msg_t *msg, void *user_data) {
    (void)user_data;
    g_stage = "on_most_msg";
    if (!msg) return;
    g_rx_count++;
    printf("[RX #%d] dev=0x%04x fb=0x%02x inst=0x%02x func=0x%04x op=0x%02x len=%u type=%u\n",
           g_rx_count, msg->dev_addr, msg->fblock_id, msg->inst_id,
           msg->func_id, msg->op_type, msg->data_len, msg->reserved_14);
    if (msg->data && msg->data_len > 0 && msg->data_len <= 4096) {
        printf("        hex:");
        for (uint16_t i = 0; i < msg->data_len && i < 64; i++) {
            printf(" %02x", msg->data[i]);
        }
        if (msg->data_len > 64) printf(" ...");
        printf("\n");
        if (msg->fblock_id == 0xF1 && msg->inst_id == 0x06 &&
            (msg->func_id == 0x0001 || msg->func_id == 0x0005) &&
            msg->op_type == 0x0C) {
            g_mhp_ready = 1;
        }
        if (msg->fblock_id == HUD_FBLOCK_ID && msg->func_id == HUD_FUNC_CREATE_INSTANCE &&
            (msg->op_type == 0x0D || msg->op_type == 0x0C) && msg->data_len >= 4) {
            uint16_t handle = get_be16(msg->data);
            uint16_t inst_id = get_be16(msg->data + 2);
            printf("        -> CREATED handle=0x%04x => InstanceID=0x%04x\n", handle, inst_id);
            if (handle < 16) {
                g_created_inst[handle] = inst_id;
                g_created_seen[handle] = 1;
            }
        }
    }
    fflush(stdout);
}

static int all_test_created(void) {
    return g_created_seen[1] && g_created_seen[2] && g_created_seen[3] &&
           g_created_seen[4] && g_created_seen[5];
}

static int mhp_is_ready(void) {
    return g_mhp_ready != 0;
}

static void pump_until(int fd, int duration_ms, int (*pred)(void)) {
    uint64_t end = now_ms() + (uint64_t)duration_ms;
    while (now_ms() < end) {
        if (pred && pred()) break;
        int rem = (int)(end - now_ms());
        if (rem <= 0) break;
        struct pollfd pfd = { .fd = fd, .events = POLLIN, .revents = 0 };
        g_stage = "poll";
        int rc = poll(&pfd, 1, rem > 100 ? 100 : rem);
        if (rc > 0 && (pfd.revents & POLLIN)) {
            g_stage = "vmostProcessMessages";
            int prc = vmostProcessMessages();
            g_stage = "after_vmostProcessMessages";
            if (prc != 0) {
                printf("[WARN] vmostProcessMessages() -> %d (%s)\n", prc, vmostErrToString(prc));
                break;
            }
        } else if (rc < 0) {
            perror("poll");
            break;
        }
    }
}

static void pump_events(int fd, int duration_ms) {
    pump_until(fd, duration_ms, NULL);
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

    printf("[TX] dev=0x%04x fb=0x%02x inst=0x%02x func=0x%04x op=0x%02x type=%u len=%u\n",
           dev_addr, fb, inst, func, op, msg_type, len);
    int rc = vmostMostMsgSend(&m);
    printf("[TX_DONE] rc=%d (%s)\n", rc, vmostErrToString(rc));
    fflush(stdout);
    return rc;
}

static int send_msg(uint16_t dev_addr, uint8_t fb, uint8_t inst, uint16_t func, uint32_t op,
                    const uint8_t *payload, uint16_t len) {
    return send_msg_type(dev_addr, fb, inst, func, op, 0, payload, len);
}

static int send_hud_create(uint16_t dev, uint16_t handle, uint8_t type, uint8_t sub, const char *name) {
    uint8_t buf[128];
    size_t nlen = strlen(name);
    if (nlen > 64) nlen = 64;
    put_be16(buf, handle);
    buf[2] = type;
    buf[3] = sub;
    buf[4] = 0x01; /* ESvcCharacterEncoding=0 -> ISO-8859-1 / 8-bit tag */
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

static int send_hud_release(uint16_t dev, uint16_t inst_id) {
    uint8_t buf[2];
    put_be16(buf, inst_id);
    return send_msg_type(dev, HUD_FBLOCK_ID, HUD_INST_ID, HUD_FUNC_RELEASE_INSTANCE,
                         VMOST_OP_START_RES_ACK, 2, buf, sizeof(buf));
}

static int send_hud_text(uint16_t dev, uint16_t inst_id, const char *text) {
    uint8_t buf[128];
    size_t tlen = strlen(text);
    if (tlen > 100) tlen = 100;
    put_be16(buf, inst_id);
    buf[2] = 0x01;
    memcpy(buf + 3, text, tlen);
    buf[3 + tlen] = 0x00;
    return send_msg_type(dev, HUD_FBLOCK_ID, HUD_INST_ID, HUD_FUNC_TEXTAREA_SET_TEXT,
                         VMOST_OP_START_RES_ACK, 2, buf, (uint16_t)(4 + tlen));
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

static int send_hud_polygon(uint16_t dev, uint16_t inst_id, uint8_t poly_mode,
                            const uint16_t *xy_pairs, uint8_t num_pts) {
    uint8_t buf[256];
    if (6 + num_pts * 4 > sizeof(buf)) return -1;
    put_be16(buf, inst_id);
    put_be16(buf + 2, 0x0000);
    buf[4] = poly_mode;
    buf[5] = num_pts;
    for (uint8_t i = 0; i < num_pts; i++) {
        put_be16(buf + 6 + i * 4, xy_pairs[i * 2]);
        put_be16(buf + 6 + i * 4 + 2, xy_pairs[i * 2 + 1]);
    }
    return send_msg_type(dev, HUD_FBLOCK_ID, HUD_INST_ID, HUD_FUNC_IMAGE_ADD_POLYGON,
                         VMOST_OP_START_RES_ACK, 2, buf, (uint16_t)(6 + num_pts * 4));
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

static int parse_hex_bytes(const char *hex, uint8_t *out, size_t max_len, uint16_t *out_len) {
    size_t n = strlen(hex);
    if (n % 2 != 0 || n / 2 > max_len) return -1;
    for (size_t i = 0; i < n / 2; i++) {
        unsigned int b = 0;
        if (sscanf(hex + i * 2, "%2x", &b) != 1) return -1;
        out[i] = (uint8_t)b;
    }
    *out_len = (uint16_t)(n / 2);
    return 0;
}

static void print_usage(const char *prog) {
    printf("Usage:\n"
           "  %s --status                          # Read-only: ownAddr, MPR, FBlocks/Shadows\n"
           "  %s --probe-read [dev_addr_hex]       # Register 0x82/0x00 shadow + read-only HUD query (default dev=0xFFFF)\n"
           "  %s --test-draw [dev_addr_hex]        # Non-destructive 8s HUD arrow+text test (auto-cleans on exit)\n"
           "  %s --inject <dev.fb.inst.func.op[.hex]> [wait_ms]\n",
           prog, prog, prog, prog);
}

int main(int argc, char **argv) {
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);
    signal(SIGSEGV, on_crash_sig);
    signal(SIGABRT, on_crash_sig);

    int mode_status = 1;
    int mode_probe_read = 0;
    int mode_test_draw = 0;
    uint16_t probe_dev = 0xFFFF;
    uint16_t draw_dev = 0;
    const char *inject_spec = NULL;
    int wait_ms = 2000;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--status") == 0) {
            mode_status = 1;
        } else if (strcmp(argv[i], "--probe-read") == 0) {
            mode_probe_read = 1;
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                probe_dev = (uint16_t)strtoul(argv[++i], NULL, 16);
            }
        } else if (strcmp(argv[i], "--test-draw") == 0) {
            mode_test_draw = 1;
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                draw_dev = (uint16_t)strtoul(argv[++i], NULL, 16);
            }
        } else if (strcmp(argv[i], "--inject") == 0 && i + 1 < argc) {
            inject_spec = argv[++i];
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                wait_ms = atoi(argv[++i]);
            }
        } else {
            print_usage(argv[0]);
            return 1;
        }
    }

    LogDefaultDomainSet("hud_probe");
    LogSetDLTApplicationID("HUDT", "HUD probe");

    int rc = vmostConnect(on_most_msg, NULL, 0);
    if (rc != 0) {
        fprintf(stderr, "vmostConnect failed: %d (%s)\n", rc, vmostErrToString(rc));
        return 1;
    }

    vmostRegisterToEventsExt(on_net_event);

    int fd = -1;
    int fd_rc = vmostFileDescriptorGet(&fd);
    uint16_t own_addr = 0;
    uint8_t mpr = 0;
    uint32_t nm_status = 0;
    vmostOwnAddrGet(&own_addr);
    vmostMPRGet(&mpr);
    vmostGetNetworkMasterConfigurationStatus(&nm_status);

    printf("Connected to vmostd: fd=%d (rc=%d) own_addr=0x%04x mpr=%u nm_status=%u\n",
           fd, fd_rc, own_addr, mpr, nm_status);

    if (mode_status) {
        VMost_FBlock_Registration_t *regs = calloc(1024, sizeof(VMost_FBlock_Registration_t));
        uint32_t cnt = 0;
        if (regs && vmostGetRegisteredFblocks(regs, &cnt) == 0) {
            printf("Registered FBlocks (%u):\n", cnt);
            for (uint32_t i = 0; i < cnt && i < 1024; i++) {
                if (regs[i].fblock_id == HUD_FBLOCK_ID || regs[i].fblock_id == 0x52 ||
                    regs[i].fblock_id == 0x24 || regs[i].fblock_id == 0xD1 ||
                    regs[i].fblock_id == 0x10) {
                    printf("  FB=0x%02x Inst=0x%02x Dev=0x%04x PID=%u\n",
                           regs[i].fblock_id, regs[i].inst_id, regs[i].dev_addr, regs[i].pid);
                }
            }
        }
        cnt = 0;
        if (regs && vmostGetRegisteredFblockShadows(regs, &cnt) == 0) {
            int hud_shadows = 0;
            for (uint32_t i = 0; i < cnt && i < 1024; i++) {
                if (regs[i].fblock_id == HUD_FBLOCK_ID) {
                    hud_shadows++;
                    printf("  Shadow 0x%02x/0x%02x Dev=0x%04x PID=%u\n",
                           regs[i].fblock_id, regs[i].inst_id, regs[i].dev_addr, regs[i].pid);
                }
            }
            printf("Total Shadows=%u (0x82 matches=%d)\n", cnt, hud_shadows);
        }
        free(regs);
    }

    if (mode_probe_read) {
        printf("Registering Shadow 0x%02x/0x%02x...\n", HUD_FBLOCK_ID, HUD_INST_ID);
        rc = vmostFblockShadowRegister(HUD_FBLOCK_ID, HUD_INST_ID);
        printf("vmostFblockShadowRegister -> %d (%s)\n", rc, vmostErrToString(rc));
        if (rc == 0) {
            pump_events(fd, 500);
            /* Read-only MOST Get (op=0x01) on FktIDs (0x0000) & 0x82/0x00 FktIDs */
            const uint16_t get_funcs[] = {
                0x0000, /* FktIDs */
                0x0200, 0x020E, 0x020F, 0x0210, 0x0212,
                0x0221, 0x0226, 0x0228, 0x0229, 0x022A, 0x022B, 0x022C,
                0x0247, 0x0248, 0x0300, 0x0301, 0x0C01, 0x0C0A
            };
            for (size_t i = 0; i < sizeof(get_funcs) / sizeof(get_funcs[0]); i++) {
                send_msg(probe_dev, HUD_FBLOCK_ID, HUD_INST_ID, get_funcs[i], VMOST_OP_GET, NULL, 0);
                pump_events(fd, 200);
            }

            vmostFblockShadowDeregister(HUD_FBLOCK_ID, HUD_INST_ID);
        }
    }

    if (inject_spec) {
        unsigned dev = 0, fb = 0, inst = 0, func = 0, op = 0;
        char hexbuf[1024] = {0};
        int n = sscanf(inject_spec, "%x.%x.%x.%x.%x.%1023s", &dev, &fb, &inst, &func, &op, hexbuf);
        if (n < 5) {
            fprintf(stderr, "Bad --inject format. Expected dev.fb.inst.func.op[.hex]\n");
            vmostDisconnect();
            return 3;
        }
        uint8_t payload[512];
        uint16_t plen = 0;
        if (n == 6 && parse_hex_bytes(hexbuf, payload, sizeof(payload), &plen) != 0) {
            fprintf(stderr, "Bad hex payload\n");
            vmostDisconnect();
            return 3;
        }
        rc = vmostFblockShadowRegister((uint8_t)fb, (uint8_t)inst);
        printf("vmostFblockShadowRegister(0x%02x, 0x%02x) -> %d (%s)\n", fb, inst, rc, vmostErrToString(rc));
        pump_events(fd, 300);
        send_msg((uint16_t)dev, (uint8_t)fb, (uint8_t)inst, (uint16_t)func, (uint32_t)op, payload, plen);
        pump_events(fd, wait_ms);
        vmostFblockShadowDeregister((uint8_t)fb, (uint8_t)inst);
    }

    if (mode_test_draw) {
        if (draw_dev == 0) draw_dev = find_hud_dev();
        printf("\n=== TEST DRAW on dev=0x%04x (0x82/0x00) ===\n", draw_dev);
        memset((void *)g_created_inst, 0, sizeof(g_created_inst));
        memset((void *)g_created_seen, 0, sizeof(g_created_seen));
        g_mhp_ready = 0;

        int s_f1_6 = vmostFblockShadowRegister(0xF1, 0x06);
        int s_f1_0 = vmostFblockShadowRegister(0xF1, 0x00);
        int s_82 = vmostFblockShadowRegister(HUD_FBLOCK_ID, HUD_INST_ID);
        printf("Shadows: 0xF1/06->%d, 0xF1/00->%d, 0x82/00->%d\n", s_f1_6, s_f1_0, s_82);
        pump_events(fd, 250);

        send_mhp_conn(draw_dev, 1);
        pump_until(fd, 1200, mhp_is_ready);
        printf("MHP ready=%d\n", g_mhp_ready);

        for (int attempt = 0; attempt < 2 && !all_test_created(); attempt++) {
            if (attempt > 0) {
                printf("Retrying missing CREATE_INSTANCE handles (attempt=%d)...\n", attempt + 1);
                g_mhp_ready = 0;
                send_mhp_conn(draw_dev, 1);
                pump_until(fd, 800, mhp_is_ready);
            }
            if (!g_created_seen[1]) send_hud_create(draw_dev, 1, HUD_TYPE_FRAME, 0x00, "TopFrame");
            if (!g_created_seen[2]) send_hud_create(draw_dev, 2, HUD_TYPE_TEXTAREA, 0x00, "Distance");
            if (!g_created_seen[3]) send_hud_create(draw_dev, 3, HUD_TYPE_TEXTAREA, 0x00, "Unit");
            if (!g_created_seen[4]) send_hud_create(draw_dev, 4, HUD_TYPE_TEXTAREA, 0x00, "ComingStreet");
            if (!g_created_seen[5]) send_hud_create(draw_dev, 5, HUD_TYPE_IMAGE, 0x00, "1.GuidingIcon");

            int flush1 = vmostMostPacketizerFlush(draw_dev, HUD_FBLOCK_ID, HUD_INST_ID, HUD_FUNC_TERMINAL_INTERFACE, 0);
            printf("vmostMostPacketizerFlush(create, attempt=%d) -> %d (%s)\n",
                   attempt + 1, flush1, vmostErrToString(flush1));
            pump_until(fd, 3500, all_test_created);
        }

        uint16_t frame_id  = g_created_seen[1] ? g_created_inst[1] : 0;
        uint16_t dist_id   = g_created_seen[2] ? g_created_inst[2] : 0;
        uint16_t unit_id   = g_created_seen[3] ? g_created_inst[3] : 0;
        uint16_t street_id = g_created_seen[4] ? g_created_inst[4] : 0;
        uint16_t icon_id   = g_created_seen[5] ? g_created_inst[5] : 0;
        printf("Resolved IDs: frame=0x%04x dist=0x%04x unit=0x%04x street=0x%04x icon=0x%04x\n",
               frame_id, dist_id, unit_id, street_id, icon_id);

        if (frame_id && dist_id && unit_id && street_id && icon_id) {
            send_hud_add_item(draw_dev, 10, frame_id, 0x14, dist_id);
            send_hud_add_item(draw_dev, 11, frame_id, 0x14, unit_id);
            send_hud_add_item(draw_dev, 12, frame_id, 0x14, street_id);
            send_hud_add_item(draw_dev, 13, frame_id, 0x14, icon_id);

            send_hud_u16(draw_dev, HUD_FUNC_IMAGE_GET_SIZE, VMOST_OP_START_RES_ACK, icon_id);
            send_hud_u16(draw_dev, HUD_FUNC_IMAGE_REMOVE_ALL_ITEMS, VMOST_OP_START_RES_ACK, icon_id);
            send_hud_u16_u8(draw_dev, HUD_FUNC_IMAGE_SET_LINE_WIDTH, VMOST_OP_START_RES_ACK, icon_id, 2);
            send_hud_u16_u8(draw_dev, HUD_FUNC_IMAGE_SET_LINE_COLOR, VMOST_OP_SET, icon_id, 4);
            send_hud_u16_u8(draw_dev, HUD_FUNC_IMAGE_SET_FILL_MODE, VMOST_OP_START_RESULT, icon_id, 3);
            send_hud_u16_u8(draw_dev, HUD_FUNC_IMAGE_SET_FILL_COLOR, VMOST_OP_START_RESULT, icon_id, 4);

            /* Solid filled straight-ahead arrow polygon strictly inside 120x72 GuidingIcon box */
            static const uint16_t arrow_pts[14] = {
                60, 8,
                84, 34,
                69, 34,
                69, 64,
                51, 64,
                51, 34,
                36, 34
            };
            send_hud_polygon(draw_dev, icon_id, 3, arrow_pts, 7);

            send_hud_text(draw_dev, dist_id, "150");
            send_hud_text(draw_dev, unit_id, "m");
            send_hud_text(draw_dev, street_id, "HUD Test");

            send_hud_set_vis(draw_dev, dist_id, 1);
            send_hud_set_vis(draw_dev, unit_id, 1);
            send_hud_set_vis(draw_dev, street_id, 1);
            send_hud_set_vis(draw_dev, icon_id, 1);
            send_hud_set_vis(draw_dev, frame_id, 1);

            int flush2 = vmostMostPacketizerFlush(draw_dev, HUD_FBLOCK_ID, HUD_INST_ID, HUD_FUNC_TERMINAL_INTERFACE, 0);
            printf("vmostMostPacketizerFlush(draw) -> %d (%s)\n", flush2, vmostErrToString(flush2));
            printf("HOLDING ON HUD FOR 8s -- CHECK WINDSHIELD HUD NOW!\n");
            pump_events(fd, 8000);

            printf("Cleaning up HUD instances...\n");
            send_hud_set_vis(draw_dev, frame_id, 0);
            send_hud_release(draw_dev, icon_id);
            send_hud_release(draw_dev, street_id);
            send_hud_release(draw_dev, unit_id);
            send_hud_release(draw_dev, dist_id);
            send_hud_release(draw_dev, frame_id);
            vmostMostPacketizerFlush(draw_dev, HUD_FBLOCK_ID, HUD_INST_ID, HUD_FUNC_TERMINAL_INTERFACE, 0);
            pump_events(fd, 600);
        } else {
            printf("ABORT TEST DRAW: missing InstanceID ResultAck from 0x82/0x00.\n");
            int any_created = 0;
            for (int h = 5; h >= 1; h--) {
                if (g_created_seen[h] && g_created_inst[h]) {
                    send_hud_release(draw_dev, g_created_inst[h]);
                    any_created = 1;
                }
            }
            if (any_created) {
                vmostMostPacketizerFlush(draw_dev, HUD_FBLOCK_ID, HUD_INST_ID, HUD_FUNC_TERMINAL_INTERFACE, 0);
                pump_events(fd, 400);
            }
        }

        send_mhp_conn(draw_dev, 0);
        pump_events(fd, 200);
        vmostFblockShadowDeregister(HUD_FBLOCK_ID, HUD_INST_ID);
        vmostFblockShadowDeregister(0xF1, 0x06);
        vmostFblockShadowDeregister(0xF1, 0x00);
    }

    vmostDisconnect();
    printf("Done (rx_count=%d).\n", g_rx_count);
    return 0;
}
