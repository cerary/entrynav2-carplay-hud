/*
 * carplay_shim.c - Non-blocking LD_PRELOAD shim for EntryNav2 /usr/local/bin/CARPLAY
 *
 * Verified against EntryNav2 firmware 202005181837:
 *   - CinemoCreateIAP (CARPLAY -> libNmeSDK.so)
 *   - ICinemoIAP_StartRouteGuidanceUpdates (libNmeSDK.so)
 *   - NmeIAP2Message::Encode(NmeArray<uint8_t>&) (libNmeBaseClasses.so)
 *   - NmeIAP2Message::Decode(const uint8_t *buf, int len) (libNmeBaseClasses.so)
 *
 * Behavior:
 *   1. Captures ICinemoIAP* from CinemoCreateIAP().
 *   2. On NmeIAP2Message::Encode() for 0x1D01 (IdentificationInformation):
 *      - Ensures 0x5200, 0x5203 are in param 0x0006 (MessagesSentByAccessory)
 *      - Ensures 0x5201, 0x5202, 0x5204 are in param 0x0007 (MessagesReceivedFromDevice)
 *      - Ensures param 0x001E (RouteGuidanceDisplayComponent, unique comp_id) is present
 *   3. On NmeIAP2Message::Decode() for 0x1D02 / 0x1D03 / 0x52xx:
 *      - Copies raw iAP2 packet into lock-protected (trylock) ring buffer
 *      - Background worker sends packets via SOCK_DGRAM to /tmp/carplay_rgd.sock
 *      - On 0x1D02 (IdentificationAccepted), calls ICinemoIAP_StartRouteGuidanceUpdates(iap, 1, &comp_id)
 *      - Never blocks Cinemo RX thread (avoids 75s watchdog).
 */

#define _GNU_SOURCE
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <dlfcn.h>
#include <pthread.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <errno.h>
#include <time.h>

#include "iap2_rgd.h"

/* NmeArray<uint8_t> verified from libNmeBaseClasses.so _ZNK14NmeIAP2Message6EncodeER8NmeArrayIhE */
typedef struct {
    uint8_t *data;
    int32_t  len;
    int32_t  cap;
    int32_t  elem_size;
} NmeArrayU8;

typedef int (*CinemoCreateIAP_fn)(void **out_iap);
typedef int (*ICinemoIAP_StartRGD_fn)(void *iap, uint32_t count, const uint16_t *comp_ids);
typedef int (*NmeEncode_fn)(const void *msg_this, NmeArrayU8 *out_arr);
typedef int (*NmeDecode_fn)(void *msg_this, const uint8_t *buf, int len);

static CinemoCreateIAP_fn      orig_CinemoCreateIAP = NULL;
static ICinemoIAP_StartRGD_fn  sym_StartRGD         = NULL;
static NmeEncode_fn            orig_Encode          = NULL;
static NmeDecode_fn            orig_Decode          = NULL;

static void * volatile g_iap_ptr = NULL;
static volatile int    g_debug_log = 0;

/* Ring buffer for non-blocking RX handoff to background sender thread */
#define RING_SLOTS 32
#define RING_MAX_BYTES 4096

typedef struct {
    uint16_t len;
    uint8_t  data[RING_MAX_BYTES];
} RingSlot;

static RingSlot        g_ring[RING_SLOTS];
static volatile int    g_ring_head = 0;
static volatile int    g_ring_tail = 0;
static volatile int    g_start_rgd_pending = 0;
static volatile int    g_got_52xx = 0;
static volatile uint16_t g_rgd_comp_id = 1;
static pthread_mutex_t g_ring_mu = PTHREAD_MUTEX_INITIALIZER;
static pthread_t       g_worker_thread;
static pthread_once_t  g_init_once = PTHREAD_ONCE_INIT;
static int             g_dgram_fd = -1;

static void shim_log(const char *fmt, ...) {
    if (!g_debug_log) return;
    int fd = open("/tmp/carplay_shim.log", O_WRONLY | O_CREAT | O_APPEND, 0666);
    if (fd < 0) return;
    FILE *f = fdopen(fd, "a");
    if (!f) { close(fd); return; }
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    fprintf(f, "[%ld.%03ld] ", (long)ts.tv_sec, ts.tv_nsec / 1000000L);
    va_list ap;
    va_start(ap, fmt);
    vfprintf(f, fmt, ap);
    va_end(ap);
    fputc('\n', f);
    fclose(f);
}

static void resolve_syms(void) {
    const char *dbg = getenv("HUD_SHIM_LOG");
    if (dbg && dbg[0] == '1') g_debug_log = 1;

    orig_CinemoCreateIAP = (CinemoCreateIAP_fn)dlsym(RTLD_NEXT, "CinemoCreateIAP");
    orig_Encode = (NmeEncode_fn)dlsym(RTLD_NEXT, "_ZNK14NmeIAP2Message6EncodeER8NmeArrayIhE");
    orig_Decode = (NmeDecode_fn)dlsym(RTLD_NEXT, "_ZN14NmeIAP2Message6DecodeEPKhi");
    sym_StartRGD = (ICinemoIAP_StartRGD_fn)dlsym(RTLD_DEFAULT, "ICinemoIAP_StartRouteGuidanceUpdates");

    shim_log("init: CreateIAP=%p Encode=%p Decode=%p StartRGD=%p",
             (void *)orig_CinemoCreateIAP, (void *)orig_Encode,
             (void *)orig_Decode, (void *)sym_StartRGD);
}

static void send_dgram_packet(const uint8_t *buf, uint16_t len) {
    if (g_dgram_fd < 0) {
        g_dgram_fd = socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK, 0);
        if (g_dgram_fd < 0) return;
    }
    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", CARPLAY_RGD_SOCK_PATH);

    ssize_t rc = sendto(g_dgram_fd, buf, len, MSG_DONTWAIT,
                        (const struct sockaddr *)&addr, sizeof(addr));
    (void)rc;
}

static void *worker_main(void *arg) {
    (void)arg;
    while (1) {
        int did_work = 0;

        if (__sync_bool_compare_and_swap(&g_start_rgd_pending, 1, 0)) {
            usleep(500 * 1000);
            void *iap = g_iap_ptr;
            if (!sym_StartRGD) {
                sym_StartRGD = (ICinemoIAP_StartRGD_fn)dlsym(RTLD_DEFAULT, "ICinemoIAP_StartRouteGuidanceUpdates");
            }
            if (iap && sym_StartRGD) {
                uint16_t comp_id = g_rgd_comp_id;
                int rc = -1;
                for (int attempt = 0; attempt < 3; attempt++) {
                    rc = sym_StartRGD(iap, 1, &comp_id);
                    shim_log("StartRGD#%d(iap=%p, comp=%u) -> %d", attempt + 1, iap, comp_id, rc);
                    if (rc == 0) break;
                    usleep(800 * 1000);
                    iap = g_iap_ptr;
                    if (!iap) break;
                }
                if (rc == 0) {
                    for (int w = 0; w < 25; w++) {
                        if (g_got_52xx || g_start_rgd_pending) break;
                        usleep(100 * 1000);
                    }
                    if (!g_got_52xx && !g_start_rgd_pending && g_iap_ptr == iap) {
                        rc = sym_StartRGD(iap, 1, &comp_id);
                        shim_log("StartRGD-refresh(iap=%p, comp=%u) -> %d", iap, comp_id, rc);
                    }
                }
            } else {
                shim_log("StartRGD skipped: iap=%p sym=%p", iap, (void *)sym_StartRGD);
            }
            did_work = 1;
        }

        while (1) {
            uint8_t local_buf[RING_MAX_BYTES];
            uint16_t local_len = 0;

            pthread_mutex_lock(&g_ring_mu);
            if (g_ring_tail != g_ring_head) {
                RingSlot *slot = &g_ring[g_ring_tail];
                local_len = slot->len;
                if (local_len > 0 && local_len <= RING_MAX_BYTES) {
                    memcpy(local_buf, slot->data, local_len);
                }
                g_ring_tail = (g_ring_tail + 1) % RING_SLOTS;
            }
            pthread_mutex_unlock(&g_ring_mu);

            if (local_len == 0) break;

            did_work = 1;
            send_dgram_packet(local_buf, local_len);
            if (g_debug_log && local_len >= 6) {
                uint16_t mid = iap2_be16(local_buf + 4);
                shim_log("RX->sock msg=0x%04x len=%u", mid, local_len);
            }
        }

        if (!did_work) {
            usleep(10 * 1000);
        }
    }
    return NULL;
}

static void ensure_init(void) {
    pthread_once(&g_init_once, resolve_syms);
    static volatile int thread_started = 0;
    if (__sync_bool_compare_and_swap(&thread_started, 0, 1)) {
        pthread_attr_t attr;
        pthread_attr_init(&attr);
        pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
        pthread_create(&g_worker_thread, &attr, worker_main, NULL);
        pthread_attr_destroy(&attr);
    }
}

static int is_carplay_process(void) {
    char exe[256];
    ssize_t n = readlink("/proc/self/exe", exe, sizeof(exe) - 1);
    if (n <= 0) return 0;
    exe[n] = '\0';
    const char *base = strrchr(exe, '/');
    base = base ? base + 1 : exe;
    return strcmp(base, "CARPLAY") == 0;
}

__attribute__((constructor))
static void carplay_shim_ctor(void) {
    if (!is_carplay_process()) return;
    const char *dbg = getenv("HUD_SHIM_LOG");
    if (dbg && dbg[0] == '1') g_debug_log = 1;
    shim_log("ctor: loaded pid=%d uid=%d euid=%d", (int)getpid(), (int)getuid(), (int)geteuid());
}

static void enqueue_packet(const uint8_t *buf, int len) {
    if (!buf || len < 6 || len > RING_MAX_BYTES) return;
    if (pthread_mutex_trylock(&g_ring_mu) != 0) return; /* Never block RX thread */
    int next = (g_ring_head + 1) % RING_SLOTS;
    if (next != g_ring_tail) {
        g_ring[g_ring_head].len = (uint16_t)len;
        memcpy(g_ring[g_ring_head].data, buf, (size_t)len);
        g_ring_head = next;
    }
    pthread_mutex_unlock(&g_ring_mu);
}

/* Ensure NmeArrayU8 has capacity for new_len */
static int nme_ensure_cap(NmeArrayU8 *arr, int32_t new_len) {
    if (new_len <= arr->cap && arr->data != NULL) return 0;
    int32_t new_cap = (new_len + 255) & ~255;
    uint8_t *p = (uint8_t *)realloc(arr->data, (size_t)new_cap);
    if (!p) return -1;
    arr->data = p;
    arr->cap = new_cap;
    return 0;
}

/* Check if a big-endian uint16 list [payload, payload_len) contains val */
static int u16_list_contains(const uint8_t *payload, size_t payload_len, uint16_t val) {
    for (size_t i = 0; i + 1 < payload_len; i += 2) {
        if (iap2_be16(payload + i) == val) return 1;
    }
    return 0;
}

/* Append missing uint16 IDs into an existing TLV param inside wire iAP2 message,
 * or create the param TLV if it did not exist yet. */
static void tlv_append_u16_ids(NmeArrayU8 *arr, uint16_t target_param,
                               const uint16_t *ids, size_t id_count) {
    if (!arr->data || arr->len < 6) return;
    size_t pos = 6;
    while (pos + 4 <= (size_t)arr->len) {
        uint16_t tlen = iap2_be16(arr->data + pos);
        uint16_t pid  = iap2_be16(arr->data + pos + 2);
        if (tlen < 4 || pos + tlen > (size_t)arr->len) break;

        if (pid == target_param) {
            uint16_t add_ids[8];
            size_t add_n = 0;
            const uint8_t *pay = arr->data + pos + 4;
            size_t pay_len = tlen - 4;
            for (size_t i = 0; i < id_count && add_n < 8; i++) {
                if (!u16_list_contains(pay, pay_len, ids[i])) {
                    add_ids[add_n++] = ids[i];
                }
            }
            if (add_n == 0) return;

            int32_t extra = (int32_t)(add_n * 2);
            int32_t old_len = arr->len;
            if (nme_ensure_cap(arr, old_len + extra) != 0) return;

            size_t insert_at = pos + tlen;
            if (insert_at < (size_t)old_len) {
                memmove(arr->data + insert_at + extra,
                        arr->data + insert_at,
                        (size_t)old_len - insert_at);
            }
            for (size_t i = 0; i < add_n; i++) {
                iap2_put_be16(arr->data + insert_at + i * 2, add_ids[i]);
            }
            iap2_put_be16(arr->data + pos, (uint16_t)(tlen + extra));
            arr->len = old_len + extra;
            iap2_put_be16(arr->data + 2, (uint16_t)arr->len);
            return;
        }
        pos += tlen;
    }

    /* Target param was absent: append a brand-new TLV containing all ids */
    uint8_t buf[32];
    if (id_count * 2 > sizeof(buf)) return;
    for (size_t i = 0; i < id_count; i++) {
        iap2_put_be16(buf + i * 2, ids[i]);
    }
    uint16_t tlen = (uint16_t)(4 + id_count * 2);
    int32_t old_len = arr->len;
    if (nme_ensure_cap(arr, old_len + tlen) != 0) return;
    iap2_put_be16(arr->data + old_len, tlen);
    iap2_put_be16(arr->data + old_len + 2, target_param);
    memcpy(arr->data + old_len + 4, buf, id_count * 2);
    arr->len = old_len + tlen;
    iap2_put_be16(arr->data + 2, (uint16_t)arr->len);
}

static int has_param(const NmeArrayU8 *arr, uint16_t target_param) {
    size_t pos = 6;
    while (pos + 4 <= (size_t)arr->len) {
        uint16_t tlen = iap2_be16(arr->data + pos);
        uint16_t pid  = iap2_be16(arr->data + pos + 2);
        if (tlen < 4 || pos + tlen > (size_t)arr->len) break;
        if (pid == target_param) return 1;
        pos += tlen;
    }
    return 0;
}

static void append_tlv(NmeArrayU8 *arr, uint16_t param_id, const uint8_t *pay, uint16_t pay_len) {
    uint16_t tlen = (uint16_t)(4 + pay_len);
    int32_t old_len = arr->len;
    if (nme_ensure_cap(arr, old_len + tlen) != 0) return;
    iap2_put_be16(arr->data + old_len, tlen);
    iap2_put_be16(arr->data + old_len + 2, param_id);
    if (pay_len > 0 && pay) {
        memcpy(arr->data + old_len + 4, pay, pay_len);
    }
    arr->len = old_len + tlen;
    iap2_put_be16(arr->data + 2, (uint16_t)arr->len);
}

static uint16_t pick_unique_comp_id(const NmeArrayU8 *arr) {
    uint16_t max_id = 0;
    int found_any = 0;
    size_t pos = 6;
    while (pos + 4 <= (size_t)arr->len) {
        uint16_t tlen = iap2_be16(arr->data + pos);
        uint16_t pid  = iap2_be16(arr->data + pos + 2);
        if (tlen < 4 || pos + tlen > (size_t)arr->len) break;
        if (pid >= 0x000F && pid <= 0x0025 && tlen >= 10) {
            uint16_t sub_len = iap2_be16(arr->data + pos + 4);
            uint16_t sub_pid = iap2_be16(arr->data + pos + 6);
            if (sub_len == 6 && sub_pid == 0x0000) {
                uint16_t cid = iap2_be16(arr->data + pos + 8);
                shim_log("0x1D01 existing comp p0x%04x -> comp_id=%u", pid, cid);
                if (!found_any || cid >= max_id) {
                    max_id = cid;
                    found_any = 1;
                }
            }
        }
        pos += tlen;
    }
    uint16_t chosen = found_any ? (uint16_t)(max_id + 1) : 1;
    if (chosen == 0) chosen = 1;
    return chosen;
}

static void append_rgd_component(NmeArrayU8 *arr, uint16_t comp_id) {
    /*
     * Build RouteGuidanceDisplayComponent (Param 0x001E) matching libNmeVfs.so 0x1153a4..0x115748:
     *   0x0000 ComponentID = comp_id (u16 BE, unique across all 0x1D01 components)
     *   0x0001 ComponentName = "HUD\0"
     *   0x0002 MaxCurrentRoadNameLength = 64
     *   0x0003 MaxDestinationNameLength = 64
     *   0x0004 MaxAfterManeuverRoadNameLength = 64
     *   0x0005 MaxManeuverDescriptionLength = 128
     *   0x0006 MaxGuidanceManeuverCapacity = 10
     */
    uint8_t grp[128];
    size_t gpos = 0;

    struct { uint16_t id; uint16_t val; } u16_fields[] = {
        { IAP2_RGD_COMP_ID, comp_id },
        { IAP2_RGD_COMP_MAX_CUR_ROAD_LEN, 64 },
        { IAP2_RGD_COMP_MAX_DEST_LEN, 64 },
        { IAP2_RGD_COMP_MAX_AFTER_ROAD_LEN, 64 },
        { IAP2_RGD_COMP_MAX_MANEUVER_DESC_LEN, 128 },
        { IAP2_RGD_COMP_MAX_MANEUVER_CAP, 10 },
        { IAP2_RGD_COMP_MAX_LANE_DESC_LEN, 64 },
        { IAP2_RGD_COMP_MAX_LANE_CAP, 8 },
    };

    /* 0x0000 ComponentID */
    iap2_put_be16(grp + gpos, 6);
    iap2_put_be16(grp + gpos + 2, IAP2_RGD_COMP_ID);
    iap2_put_be16(grp + gpos + 4, comp_id);
    gpos += 6;

    /* 0x0001 ComponentName ("HUD\0") */
    const char *name = "HUD";
    uint16_t nlen = 4;
    iap2_put_be16(grp + gpos, (uint16_t)(4 + nlen));
    iap2_put_be16(grp + gpos + 2, IAP2_RGD_COMP_NAME);
    memcpy(grp + gpos + 4, name, nlen);
    gpos += 4 + nlen;

    for (size_t i = 1; i < sizeof(u16_fields) / sizeof(u16_fields[0]); i++) {
        iap2_put_be16(grp + gpos, 6);
        iap2_put_be16(grp + gpos + 2, u16_fields[i].id);
        iap2_put_be16(grp + gpos + 4, u16_fields[i].val);
        gpos += 6;
    }

    append_tlv(arr, IAP2_ID_RGD_COMPONENT, grp, (uint16_t)gpos);
}

/* ==================== LD_PRELOAD Hooked Exports ==================== */

int CinemoCreateIAP(void **out_iap) {
    ensure_init();
    if (!orig_CinemoCreateIAP) return -1;
    int rc = orig_CinemoCreateIAP(out_iap);
    if (rc == 0 && out_iap && *out_iap) {
        g_iap_ptr = *out_iap;
        shim_log("CinemoCreateIAP -> iap=%p", *out_iap);
    }
    return rc;
}

int _ZNK14NmeIAP2Message6EncodeER8NmeArrayIhE(const void *msg_this, NmeArrayU8 *out_arr) {
    ensure_init();
    if (!orig_Encode) return -1;
    int rc = orig_Encode(msg_this, out_arr);
    if (rc == 0 && out_arr && out_arr->data && out_arr->len >= 6 &&
        out_arr->data[0] == 0x40 && out_arr->data[1] == 0x40) {
        uint16_t msg_id = iap2_be16(out_arr->data + 4);
        if (msg_id == IAP2_MSG_IDENTIFICATION_INFO ||
            msg_id == IAP2_MSG_IDENTIFICATION_INFO_UPDATE) {
            int had_rgd = has_param(out_arr, IAP2_ID_RGD_COMPONENT);
            int before_len = out_arr->len;

            const uint16_t sent_ids[] = {
                IAP2_MSG_START_ROUTE_GUIDANCE,
                IAP2_MSG_STOP_ROUTE_GUIDANCE
            };
            const uint16_t recv_ids[] = {
                IAP2_MSG_ROUTE_GUIDANCE_UPDATE,
                IAP2_MSG_ROUTE_GUIDANCE_MANEUVER,
                IAP2_MSG_LANE_GUIDANCE_INFO
            };
            tlv_append_u16_ids(out_arr, IAP2_ID_MSGS_SENT_BY_ACC, sent_ids, 2);
            tlv_append_u16_ids(out_arr, IAP2_ID_MSGS_RECV_FROM_DEV, recv_ids, 3);
            if (!had_rgd && msg_id == IAP2_MSG_IDENTIFICATION_INFO) {
                uint16_t comp_id = pick_unique_comp_id(out_arr);
                g_rgd_comp_id = comp_id;
                append_rgd_component(out_arr, comp_id);
            }

            shim_log("Encode msg=0x%04x had_rgd=%d comp=%u len %d -> %d",
                     msg_id, had_rgd, (unsigned)g_rgd_comp_id, before_len, out_arr->len);
            enqueue_packet(out_arr->data, out_arr->len);
        } else if ((msg_id & 0xFF00) == 0x5200) {
            shim_log("Encode msg=0x%04x len=%d", msg_id, out_arr->len);
            enqueue_packet(out_arr->data, out_arr->len);
        }
    }
    return rc;
}

int _ZN14NmeIAP2Message6DecodeEPKhi(void *msg_this, const uint8_t *buf, int len) {
    ensure_init();
    if (buf && len >= 6 && buf[0] == 0x40 && buf[1] == 0x40) {
        uint16_t msg_id = iap2_be16(buf + 4);
        if (msg_id == IAP2_MSG_IDENTIFICATION_ACCEPTED) {
            g_got_52xx = 0;
            enqueue_packet(buf, len);
            __sync_lock_test_and_set(&g_start_rgd_pending, 1);
            shim_log("Decode 0x1D02 IdentificationAccepted (len=%d) -> queue StartRGD comp=%u",
                     len, (unsigned)g_rgd_comp_id);
        } else if (msg_id == IAP2_MSG_IDENTIFICATION_REJECTED) {
            enqueue_packet(buf, len);
        } else if ((msg_id & 0xFF00) == 0x5200) {
            g_got_52xx = 1;
            enqueue_packet(buf, len);
        }
    }
    if (!orig_Decode) return -1;
    return orig_Decode(msg_this, buf, len);
}
