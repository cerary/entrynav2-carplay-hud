/*
 * rgd_listen.c - Standalone receiver & decoder for /tmp/carplay_rgd.sock
 *
 * Receives raw iAP2 packets forwarded by carplay-shim.so (0x1D01/0x1D02/0x5201/0x5202/0x5204)
 * and prints decoded RouteGuidance / Maneuver / LaneGuidance fields in real time.
 * Privacy-safe: --redact hides street/destination names in shared logs if desired.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <poll.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>

#include "iap2_rgd.h"

static volatile int g_running = 1;
static int g_redact = 0;

static void on_sig(int s) {
    (void)s;
    g_running = 0;
}

static void print_safe_str(const char *label, const uint8_t *p, uint16_t plen) {
    char buf[256];
    size_t n = plen < sizeof(buf) - 1 ? plen : sizeof(buf) - 1;
    memcpy(buf, p, n);
    buf[n] = '\0';
    if (g_redact) {
        printf(" %s=[len:%zu]", label, strlen(buf));
    } else {
        printf(" %s=\"%s\"", label, buf);
    }
}

static void decode_lane_group(const uint8_t *p, uint16_t len) {
    size_t pos = 0;
    uint16_t lpos = 0xFFFF;
    uint8_t lstat = 0xFF;
    int16_t angles[16];
    int n_angles = 0;
    int16_t hi_angle = 1000;
    while (pos + 4 <= len) {
        uint16_t tlen = iap2_be16(p + pos);
        uint16_t pid  = iap2_be16(p + pos + 2);
        if (tlen < 4 || pos + tlen > len) break;
        const uint8_t *pay = p + pos + 4;
        uint16_t plen = tlen - 4;
        if (pid == IAP2_LANE_POSITION && plen >= 2) lpos = iap2_be16(pay);
        else if (pid == IAP2_LANE_STATUS && plen >= 1) lstat = pay[0];
        else if (pid == IAP2_LANE_ANGLE && plen >= 2 && n_angles < 16) angles[n_angles++] = (int16_t)iap2_be16(pay);
        else if (pid == IAP2_LANE_HIGHLIGHTED_ANGLE && plen >= 2) hi_angle = (int16_t)iap2_be16(pay);
        pos += tlen;
    }
    printf(" [lane pos=%u status=%u angles=[", lpos, (unsigned)lstat);
    for (int i = 0; i < n_angles; i++) {
        printf("%s%d", i ? "," : "", (int)angles[i]);
    }
    printf("]");
    if (hi_angle != 1000) printf(" hi=%d", (int)hi_angle);
    printf("]");
}

static void decode_packet(const uint8_t *buf, size_t len) {
    if (len < 6 || buf[0] != 0x40 || buf[1] != 0x40) return;
    uint16_t msg_id = iap2_be16(buf + 4);

    printf("[iAP2 msg=0x%04x len=%zu]", msg_id, len);
    size_t pos = 6;
    while (pos + 4 <= len) {
        uint16_t tlen = iap2_be16(buf + pos);
        uint16_t pid  = iap2_be16(buf + pos + 2);
        if (tlen < 4 || pos + tlen > len) break;
        const uint8_t *pay = buf + pos + 4;
        uint16_t plen = tlen - 4;

        if (msg_id == IAP2_MSG_ROUTE_GUIDANCE_UPDATE) {
            switch (pid) {
                case IAP2_RGU_STATE:
                    if (plen >= 1) printf(" state=%u", pay[0]);
                    break;
                case IAP2_RGU_MANEUVER_STATE:
                    if (plen >= 1) printf(" m_state=%u", pay[0]);
                    break;
                case IAP2_RGU_CURRENT_ROAD_NAME:
                    print_safe_str("cur_road", pay, plen);
                    break;
                case IAP2_RGU_DESTINATION_NAME:
                    print_safe_str("dest", pay, plen);
                    break;
                case IAP2_RGU_DIST_TO_NEXT_MANEUVER:
                    if (plen >= 4) printf(" dist_m=%u", iap2_be32(pay));
                    break;
                case IAP2_RGU_DIST_TO_NEXT_MANEUVER_STR:
                    print_safe_str("dist_str", pay, plen);
                    break;
                case IAP2_RGU_DIST_TO_NEXT_MANEUVER_UNITS:
                    if (plen >= 1) printf(" dist_unit=%u", pay[0]);
                    break;
                case IAP2_RGU_MANEUVER_CURRENT_LIST:
                    printf(" active_list(%uB):", plen);
                    for (uint16_t i = 0; i + 1 < plen; i += 2) {
                        printf(" #%u", iap2_be16(pay + i));
                    }
                    break;
                case IAP2_RGU_MANEUVER_COUNT:
                    if (plen >= 2) printf(" m_count=%u", iap2_be16(pay));
                    break;
                case IAP2_RGU_BEING_SHOWN_IN_APP:
                    if (plen >= 1) printf(" in_app=%u", pay[0]);
                    break;
                case IAP2_RGU_LANE_GUIDANCE_INDEX:
                    if (plen >= 2) printf(" lg_idx=%u", iap2_be16(pay));
                    break;
                case IAP2_RGU_LANE_GUIDANCE_TOTAL:
                    if (plen >= 2) printf(" lg_total=%u", iap2_be16(pay));
                    break;
                case IAP2_RGU_LANE_GUIDANCE_SHOWING:
                    if (plen >= 1) printf(" lg_show=%u", pay[0]);
                    break;
                case IAP2_RGU_SOURCE_NAME:
                    print_safe_str("src", pay, plen);
                    break;
                case IAP2_RGU_SOURCE_SUPPORTS_RGD:
                    if (plen >= 1) printf(" src_rg=%u", pay[0]);
                    break;
                default:
                    break;
            }
        } else if (msg_id == IAP2_MSG_ROUTE_GUIDANCE_MANEUVER) {
            switch (pid) {
                case IAP2_RGM_INDEX:
                    if (plen >= 2) printf(" idx=%u", iap2_be16(pay));
                    break;
                case IAP2_RGM_MANEUVER_TYPE:
                    if (plen >= 1) printf(" type=%u", pay[0]);
                    break;
                case IAP2_RGM_MANEUVER_DESCRIPTION:
                    print_safe_str("desc", pay, plen);
                    break;
                case IAP2_RGM_AFTER_MANEUVER_ROAD_NAME:
                    print_safe_str("after_road", pay, plen);
                    break;
                case IAP2_RGM_DIST_BETWEEN_MANEUVERS:
                    if (plen >= 4) printf(" dist_between=%u", iap2_be32(pay));
                    break;
                case IAP2_RGM_JUNCTION_TYPE:
                    if (plen >= 1) printf(" junc=%u", pay[0]);
                    break;
                case IAP2_RGM_LINKED_LANE_GUIDANCE_INFO:
                    if (plen >= 2) printf(" lane_idx=%u", iap2_be16(pay));
                    break;
                default:
                    break;
            }
        } else if (msg_id == IAP2_MSG_LANE_GUIDANCE_INFO) {
            switch (pid) {
                case IAP2_LGI_INDEX:
                    if (plen >= 2) printf(" lane_idx=%u", iap2_be16(pay));
                    break;
                case IAP2_LGI_LANE_INFO_GROUP:
                    decode_lane_group(pay, plen);
                    break;
                case IAP2_LGI_INSTRUCTION_TEXT:
                    print_safe_str("lane_txt", pay, plen);
                    break;
                default:
                    break;
            }
        } else {
            printf(" p0x%04x(%uB)", pid, plen);
            if ((msg_id == IAP2_MSG_IDENTIFICATION_REJECTED || msg_id == IAP2_MSG_START_ROUTE_GUIDANCE) && plen > 0) {
                putchar('=');
                uint16_t lim = plen < 16 ? plen : 16;
                for (uint16_t k = 0; k < lim; k++) printf("%02x", pay[k]);
                if (plen > 16) printf("..");
            }
        }
        pos += tlen;
    }
    putchar('\n');
    fflush(stdout);
}

int main(int argc, char **argv) {
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--redact") == 0) g_redact = 1;
    }

    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_sig;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0; /* Do NOT set SA_RESTART */
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    unlink(CARPLAY_RGD_SOCK_PATH);
    int fd = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (fd < 0) {
        perror("socket");
        return 1;
    }

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", CARPLAY_RGD_SOCK_PATH);
    if (bind(fd, (const struct sockaddr *)&addr, sizeof(addr)) != 0) {
        perror("bind");
        close(fd);
        return 1;
    }
    chmod(CARPLAY_RGD_SOCK_PATH, 0666);

    printf("Listening on %s (redact=%d)...\n", CARPLAY_RGD_SOCK_PATH, g_redact);
    fflush(stdout);

    uint8_t buf[4096];
    while (g_running) {
        struct pollfd pfd = { .fd = fd, .events = POLLIN, .revents = 0 };
        int prc = poll(&pfd, 1, 200);
        if (prc < 0) {
            if (errno == EINTR) continue;
            break;
        }
        if (prc == 0 || !(pfd.revents & POLLIN)) continue;
        ssize_t n = recv(fd, buf, sizeof(buf), MSG_DONTWAIT);
        if (n > 0) {
            decode_packet(buf, (size_t)n);
        }
    }

    close(fd);
    unlink(CARPLAY_RGD_SOCK_PATH);
    return 0;
}
