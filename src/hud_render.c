#include "hud_render.h"
#include "iap2_rgd.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define REROUTE_DEBOUNCE_MS 2500ULL

static void safe_copy_utf8(char *dst, size_t dst_cap, const uint8_t *src, size_t src_len) {
    if (!dst || dst_cap == 0) return;
    if (!src || src_len == 0) {
        dst[0] = '\0';
        return;
    }
    /* Strip trailing NUL bytes if present */
    while (src_len > 0 && src[src_len - 1] == '\0') src_len--;
    size_t max_copy = dst_cap - 1;
    if (src_len > max_copy) {
        src_len = max_copy;
        /* Avoid cutting inside a multi-byte UTF-8 sequence */
        while (src_len > 0 && (src[src_len] & 0xC0) == 0x80) {
            src_len--;
        }
    }
    memcpy(dst, src, src_len);
    dst[src_len] = '\0';
}

static uint32_t fnv1a_update(uint32_t h, const void *data, size_t len) {
    const uint8_t *p = (const uint8_t *)data;
    for (size_t i = 0; i < len; i++) {
        h ^= p[i];
        h *= 16777619u;
    }
    return h;
}

static int valid_angle(int16_t deg) {
    return deg >= -180 && deg <= 180;
}

void rgd_state_reset_route(RgdState *st) {
    if (!st) return;
    memset(st, 0, sizeof(*st));
    st->source_supports_rg = 1;
    st->dist_maneuver_unit = 0xFF;
}

void rgd_state_init(RgdState *st) {
    rgd_state_reset_route(st);
}

static RgdManeuver *find_or_alloc_maneuver(RgdState *st, uint16_t idx) {
    for (int i = 0; i < HUD_MAX_MANEUVERS; i++) {
        if (st->maneuvers[i].valid && st->maneuvers[i].index == idx) {
            return &st->maneuvers[i];
        }
    }
    for (int i = 0; i < HUD_MAX_MANEUVERS; i++) {
        if (!st->maneuvers[i].valid) {
            return &st->maneuvers[i];
        }
    }
    /* Evict lowest index (far behind cursor) */
    int victim = 0;
    for (int i = 1; i < HUD_MAX_MANEUVERS; i++) {
        if (st->maneuvers[i].index < st->maneuvers[victim].index) victim = i;
    }
    return &st->maneuvers[victim];
}

static const RgdManeuver *find_maneuver(const RgdState *st, uint16_t idx) {
    for (int i = 0; i < HUD_MAX_MANEUVERS; i++) {
        if (st->maneuvers[i].valid && st->maneuvers[i].index == idx) {
            return &st->maneuvers[i];
        }
    }
    return NULL;
}

static RgdLaneEvent *find_or_alloc_lane_event(RgdState *st, uint16_t idx) {
    for (int i = 0; i < HUD_MAX_LANE_EVENTS; i++) {
        if (st->lane_events[i].valid && st->lane_events[i].index == idx) {
            return &st->lane_events[i];
        }
    }
    for (int i = 0; i < HUD_MAX_LANE_EVENTS; i++) {
        if (!st->lane_events[i].valid) {
            return &st->lane_events[i];
        }
    }
    /* Evict lowest index except active lane_guidance_idx */
    int victim = -1;
    for (int i = 0; i < HUD_MAX_LANE_EVENTS; i++) {
        if (st->has_lane_idx && st->lane_events[i].index == st->lane_guidance_idx) continue;
        if (victim < 0 || st->lane_events[i].index < st->lane_events[victim].index) victim = i;
    }
    if (victim < 0) victim = 0;
    return &st->lane_events[victim];
}

static const RgdLaneEvent *find_lane_event(const RgdState *st, uint16_t idx) {
    for (int i = 0; i < HUD_MAX_LANE_EVENTS; i++) {
        if (st->lane_events[i].valid && st->lane_events[i].index == idx) {
            return &st->lane_events[i];
        }
    }
    return NULL;
}

static void prune_lanes_below(RgdState *st, uint16_t active_idx) {
    for (int i = 0; i < HUD_MAX_LANE_EVENTS; i++) {
        if (st->lane_events[i].valid && st->lane_events[i].index < active_idx) {
            memset(&st->lane_events[i], 0, sizeof(st->lane_events[i]));
        }
    }
}

static void parse_lane_group(const uint8_t *buf, size_t len, RgdLaneInfo *lane) {
    memset(lane, 0, sizeof(*lane));
    lane->position = 0xFFFF;
    lane->status = 1;
    lane->highlighted_angle = 1000;
    size_t pos = 0;
    while (pos + 4 <= len) {
        uint16_t tlen = iap2_be16(buf + pos);
        uint16_t pid  = iap2_be16(buf + pos + 2);
        if (tlen < 4 || pos + tlen > len) break;
        const uint8_t *pay = buf + pos + 4;
        size_t plen = tlen - 4;
        if (pid == IAP2_LANE_POSITION && plen >= 2) {
            lane->position = iap2_be16(pay);
        } else if (pid == IAP2_LANE_STATUS && plen >= 1) {
            lane->status = pay[0];
        } else if (pid == IAP2_LANE_ANGLE && plen >= 2) {
            int16_t a = (int16_t)iap2_be16(pay);
            if (valid_angle(a) && lane->angle_count < HUD_MAX_LANE_ANGLES) {
                lane->angles[lane->angle_count++] = a;
            }
        } else if (pid == IAP2_LANE_HIGHLIGHTED_ANGLE && plen >= 2) {
            int16_t hi = (int16_t)iap2_be16(pay);
            if (valid_angle(hi)) {
                lane->highlighted_angle = hi;
                lane->has_highlighted = 1;
            }
        }
        pos += tlen;
    }
}

static int route_state_is_active(uint8_t rs, uint16_t m_count) {
    if (rs == 1 || rs == 2 || rs == 5 || rs == 6) return 1;
    if ((rs == 3 || rs == 4) && m_count > 0) return 1;
    return 0;
}

int rgd_state_ingest(RgdState *st, const uint8_t *buf, size_t len, uint64_t now_ms) {
    if (!st || !buf || len < 6 || buf[0] != 0x40 || buf[1] != 0x40) return 0;
    uint16_t msg_id = iap2_be16(buf + 4);

    if (msg_id == IAP2_MSG_STOP_ROUTE_GUIDANCE) {
        rgd_state_reset_route(st);
        st->stopped_explicit = 1;
        return 1;
    }

    if (msg_id == IAP2_MSG_ROUTE_GUIDANCE_UPDATE) {
        size_t pos = 6;
        uint8_t prev_rs = st->route_state;
        uint8_t saw_rs = 0;
        uint8_t saw_lane_idx = 0;
        uint8_t saw_lane_show = 0;

        while (pos + 4 <= len) {
            uint16_t tlen = iap2_be16(buf + pos);
            uint16_t pid  = iap2_be16(buf + pos + 2);
            if (tlen < 4 || pos + tlen > len) break;
            const uint8_t *pay = buf + pos + 4;
            size_t plen = tlen - 4;

            switch (pid) {
                case IAP2_RGU_STATE:
                    if (plen >= 1) { st->route_state = pay[0]; saw_rs = 1; }
                    break;
                case IAP2_RGU_MANEUVER_STATE:
                    if (plen >= 1) st->maneuver_state = pay[0];
                    break;
                case IAP2_RGU_CURRENT_ROAD_NAME:
                    safe_copy_utf8(st->current_road, sizeof(st->current_road), pay, plen);
                    break;
                case IAP2_RGU_DESTINATION_NAME:
                    safe_copy_utf8(st->dest_name, sizeof(st->dest_name), pay, plen);
                    break;
                case IAP2_RGU_DIST_REMAINING:
                    if (plen >= 4) st->dist_rem_m = iap2_be32(pay);
                    break;
                case IAP2_RGU_DIST_TO_NEXT_MANEUVER:
                    if (plen >= 4) { st->dist_maneuver_m = iap2_be32(pay); st->has_dist_maneuver_m = 1; }
                    break;
                case IAP2_RGU_DIST_TO_NEXT_MANEUVER_STR:
                    safe_copy_utf8(st->dist_maneuver_str, sizeof(st->dist_maneuver_str), pay, plen);
                    break;
                case IAP2_RGU_DIST_TO_NEXT_MANEUVER_UNITS:
                    if (plen >= 1) st->dist_maneuver_unit = pay[0];
                    break;
                case IAP2_RGU_MANEUVER_CURRENT_LIST:
                    if (plen >= 2) {
                        st->man_current_idx = iap2_be16(pay);
                        st->has_man_current = 1;
                    } else {
                        st->has_man_current = 0;
                    }
                    break;
                case IAP2_RGU_MANEUVER_COUNT:
                    if (plen >= 2) st->man_count = iap2_be16(pay);
                    break;
                case IAP2_RGU_BEING_SHOWN_IN_APP:
                    if (plen >= 1) st->being_shown_in_app = pay[0];
                    break;
                case IAP2_RGU_LANE_GUIDANCE_INDEX:
                    if (plen >= 2) {
                        st->lane_guidance_idx = iap2_be16(pay);
                        st->has_lane_idx = 1;
                        saw_lane_idx = 1;
                    }
                    break;
                case IAP2_RGU_LANE_GUIDANCE_TOTAL:
                    if (plen >= 2) st->lane_guidance_total = iap2_be16(pay);
                    break;
                case IAP2_RGU_LANE_GUIDANCE_SHOWING:
                    if (plen >= 1) {
                        st->lane_guidance_showing = pay[0];
                        st->has_lane_showing = 1;
                        saw_lane_show = 1;
                    }
                    break;
                case IAP2_RGU_SOURCE_NAME:
                    safe_copy_utf8(st->source_name, sizeof(st->source_name), pay, plen);
                    break;
                case IAP2_RGU_SOURCE_SUPPORTS_RGD:
                    if (plen >= 1) {
                        st->source_supports_rg = pay[0];
                        st->has_source_supports_rg = 1;
                    }
                    break;
                default:
                    break;
            }
            pos += tlen;
        }

        if (saw_lane_idx) {
            prune_lanes_below(st, st->lane_guidance_idx);
        }
        if (saw_lane_show && st->lane_guidance_showing == 0) {
            /* Keep cache unless explicitly reset, but display stays off */
        }

        if (saw_rs) {
            if (route_state_is_active(st->route_state, st->man_count)) {
                st->ever_active = 1;
                st->stopped_explicit = 0;
                st->last_active_ms = now_ms;
            } else if (st->route_state == 0) {
                if (prev_rs == 0 && st->ever_active && (now_ms - st->last_active_ms) >= REROUTE_DEBOUNCE_MS) {
                    /* Full route reset after debounce window */
                    rgd_state_reset_route(st);
                }
            }
        } else if (route_state_is_active(st->route_state, st->man_count)) {
            st->last_active_ms = now_ms;
        }
        return 1;
    }

    if (msg_id == IAP2_MSG_ROUTE_GUIDANCE_MANEUVER) {
        RgdManeuver tmp;
        memset(&tmp, 0, sizeof(tmp));
        tmp.exit_angle = 1000;
        uint8_t has_idx = 0;

        size_t pos = 6;
        while (pos + 4 <= len) {
            uint16_t tlen = iap2_be16(buf + pos);
            uint16_t pid  = iap2_be16(buf + pos + 2);
            if (tlen < 4 || pos + tlen > len) break;
            const uint8_t *pay = buf + pos + 4;
            size_t plen = tlen - 4;

            switch (pid) {
                case IAP2_RGM_INDEX:
                    if (plen >= 2) { tmp.index = iap2_be16(pay); has_idx = 1; }
                    break;
                case IAP2_RGM_MANEUVER_DESCRIPTION:
                    safe_copy_utf8(tmp.desc, sizeof(tmp.desc), pay, plen);
                    break;
                case IAP2_RGM_MANEUVER_TYPE:
                    if (plen >= 1) tmp.type = pay[0];
                    break;
                case IAP2_RGM_AFTER_MANEUVER_ROAD_NAME:
                    safe_copy_utf8(tmp.after_road, sizeof(tmp.after_road), pay, plen);
                    break;
                case IAP2_RGM_DIST_BETWEEN_MANEUVERS:
                    if (plen >= 4) tmp.dist_between_m = iap2_be32(pay);
                    break;
                case IAP2_RGM_DRIVING_SIDE:
                    if (plen >= 1) tmp.driving_side = pay[0];
                    break;
                case IAP2_RGM_JUNCTION_TYPE:
                    if (plen >= 1) tmp.junction_type = pay[0];
                    break;
                case IAP2_RGM_JUNCTION_ELEMENT_ANGLE:
                    for (size_t off = 0; off + 1 < plen && tmp.junction_angle_count < HUD_MAX_JUNC_ANGLES; off += 2) {
                        int16_t a = (int16_t)iap2_be16(pay + off);
                        if (valid_angle(a)) tmp.junction_angles[tmp.junction_angle_count++] = a;
                    }
                    break;
                case IAP2_RGM_JUNCTION_ELEMENT_EXIT_ANGLE:
                    if (plen >= 2) {
                        int16_t ea = (int16_t)iap2_be16(pay);
                        if (valid_angle(ea)) {
                            tmp.exit_angle = ea;
                            tmp.has_exit_angle = 1;
                        }
                    }
                    break;
                case IAP2_RGM_LINKED_LANE_GUIDANCE_INFO:
                    if (plen >= 2) {
                        tmp.linked_lane_idx = iap2_be16(pay);
                        tmp.has_linked_lane = 1;
                    }
                    break;
                case IAP2_RGM_EXIT_INFO:
                    safe_copy_utf8(tmp.exit_info, sizeof(tmp.exit_info), pay, plen);
                    break;
                default:
                    break;
            }
            pos += tlen;
        }
        if (has_idx) {
            RgdManeuver *slot = find_or_alloc_maneuver(st, tmp.index);
            tmp.valid = 1;
            *slot = tmp;
            return 1;
        }
        return 0;
    }

    if (msg_id == IAP2_MSG_LANE_GUIDANCE_INFO) {
        RgdLaneEvent tmp;
        memset(&tmp, 0, sizeof(tmp));
        uint8_t has_idx = 0;

        size_t pos = 6;
        while (pos + 4 <= len) {
            uint16_t tlen = iap2_be16(buf + pos);
            uint16_t pid  = iap2_be16(buf + pos + 2);
            if (tlen < 4 || pos + tlen > len) break;
            const uint8_t *pay = buf + pos + 4;
            size_t plen = tlen - 4;

            if (pid == IAP2_LGI_INDEX && plen >= 2) {
                tmp.index = iap2_be16(pay);
                has_idx = 1;
            } else if (pid == IAP2_LGI_LANE_INFO_GROUP && tmp.lane_count < HUD_MAX_LANES) {
                RgdLaneInfo li;
                parse_lane_group(pay, plen, &li);
                tmp.lanes[tmp.lane_count++] = li;
            } else if (pid == IAP2_LGI_INSTRUCTION_TEXT) {
                safe_copy_utf8(tmp.desc, sizeof(tmp.desc), pay, plen);
            }
            pos += tlen;
        }
        if (!has_idx) return 0;

        /* Sort lanes by position ascending */
        for (int i = 0; i < (int)tmp.lane_count; i++) {
            for (int j = i + 1; j < (int)tmp.lane_count; j++) {
                if (tmp.lanes[j].position < tmp.lanes[i].position) {
                    RgdLaneInfo swap = tmp.lanes[i];
                    tmp.lanes[i] = tmp.lanes[j];
                    tmp.lanes[j] = swap;
                }
            }
        }
        RgdLaneEvent *slot = find_or_alloc_lane_event(st, tmp.index);
        tmp.valid = 1;
        *slot = tmp;
        return 1;
    }

    return 0;
}

int rgd_state_should_show(const RgdState *st, uint64_t now_ms) {
    if (!st || st->stopped_explicit) return 0;
    if (st->has_source_supports_rg && st->source_supports_rg == 0 && st->man_count == 0) return 0;
    if (route_state_is_active(st->route_state, st->man_count)) return 1;
    if (st->route_state == 0 && st->ever_active && (now_ms - st->last_active_ms) < REROUTE_DEBOUNCE_MS) {
        return 1;
    }
    return 0;
}

/* ==================== Convex Polygon Helpers ==================== */

static uint16_t clamp_u16(float v, uint16_t lo, uint16_t hi) {
    int iv = (int)lroundf(v);
    if (iv < (int)lo) return lo;
    if (iv > (int)hi) return hi;
    return (uint16_t)iv;
}

static void add_poly(HudPolygonCmd *arr, uint8_t *cnt,
                     uint16_t w, uint16_t h,
                     uint8_t color, uint8_t lw, uint8_t poly_mode,
                     const float *xy, uint8_t npts) {
    if (*cnt >= HUD_MAX_POLYS || npts < 2 || npts > HUD_MAX_POLY_PTS) return;
    uint16_t max_x = (w > 2) ? (uint16_t)(w - 2) : 1;
    uint16_t max_y = (h > 2) ? (uint16_t)(h - 2) : 1;

    uint16_t qx[HUD_MAX_POLY_PTS], qy[HUD_MAX_POLY_PTS];
    uint8_t qn = 0;
    for (uint8_t i = 0; i < npts; i++) {
        uint16_t px = clamp_u16(xy[i * 2], 1, max_x);
        uint16_t py = clamp_u16(xy[i * 2 + 1], 1, max_y);
        if (qn > 0 && px == qx[qn - 1] && py == qy[qn - 1]) continue;
        qx[qn] = px;
        qy[qn] = py;
        qn++;
    }
    if (qn > 1 && qx[qn - 1] == qx[0] && qy[qn - 1] == qy[0]) qn--;
    if (poly_mode == HUD_POLY_FILL && qn < 3) return;
    if (qn < 2) return;

    HudPolygonCmd *c = &arr[*cnt];
    memset(c, 0, sizeof(*c));
    c->kind = HUD_CMD_POLYGON;
    c->color = color;
    c->line_width = lw;
    c->fill_mode = (poly_mode == HUD_POLY_FILL) ? 3 : 0;
    c->fill_color = color;
    c->poly_mode = poly_mode;
    c->num_pts = qn;
    for (uint8_t i = 0; i < qn; i++) {
        c->xy[i * 2]     = qx[i];
        c->xy[i * 2 + 1] = qy[i];
    }
    (*cnt)++;
}

/* Convert screen angle (0=top, +90=right, 180=bottom, -90=left)
 * to MOST 0x021E angle (0.1 deg units, 0=9 o'clock/left, CW: 900=top, 1800=right, 2700=bottom). */
static uint32_t screen_deg_to_hud_01deg(float screen_deg) {
    float d = fmodf(screen_deg + 90.0f, 360.0f);
    if (d < 0.0f) d += 360.0f;
    int v = (int)lroundf(d * 10.0f) % 3600;
    if (v < 0) v += 3600;
    return (uint32_t)v;
}

/* Emit native MOST 0x021E (ImageAddCircle / Kreisbogen) command */
static void add_circle(HudPolygonCmd *arr, uint8_t *cnt,
                       uint16_t w, uint16_t h,
                       uint8_t color, uint8_t lw, uint8_t circle_type,
                       float cx, float cy, float radius,
                       uint32_t start_01deg, uint32_t end_01deg) {
    if (*cnt >= HUD_MAX_POLYS) return;
    uint16_t max_x = (w > 2) ? (uint16_t)(w - 2) : 1;
    uint16_t max_y = (h > 2) ? (uint16_t)(h - 2) : 1;
    uint16_t max_r = (w > h) ? w : h;
    if (lw < 1) lw = 1;

    HudPolygonCmd *c = &arr[*cnt];
    memset(c, 0, sizeof(*c));
    c->kind = HUD_CMD_CIRCLE;
    c->color = color;
    c->line_width = lw;
    c->fill_mode = 0;
    c->fill_color = color;
    c->circle_type = circle_type;
    c->cx = clamp_u16(cx, 1, max_x);
    c->cy = clamp_u16(cy, 1, max_y);
    c->radius = clamp_u16(radius, 1, max_r);
    c->start_01deg = start_01deg % 3600u;
    c->end_01deg = end_01deg % 3600u;
    (*cnt)++;
}

/* Draw a road segment from (x0,y0) to (x1,y1) with optional semicircle round caps in 1 polygon */
static void add_round_seg(HudPolygonCmd *arr, uint8_t *cnt,
                          uint16_t w, uint16_t h,
                          uint8_t color, float x0, float y0, float x1, float y1,
                          float thick, int cap_start, int cap_end) {
    float dx = x1 - x0;
    float dy = y1 - y0;
    float len = hypotf(dx, dy);
    if (len < 0.35f) return;
    float ux = dx / len;
    float uy = dy / len;
    float r = thick * 0.5f;
    float nx = -uy;
    float ny =  ux;
    float pts[HUD_MAX_POLY_PTS * 2];
    int n = 0;

    if (cap_end) {
        /* Semicircle dome around (x1, y1) from +n through +u to -n */
        for (int i = 0; i <= 4; i++) {
            float a = (float)i * ((float)M_PI / 4.0f);
            float cn = cosf(a), cu = sinf(a);
            pts[n * 2]     = x1 + (nx * cn + ux * cu) * r;
            pts[n * 2 + 1] = y1 + (ny * cn + uy * cu) * r;
            n++;
        }
    } else {
        pts[n * 2] = x1 + nx * r; pts[n * 2 + 1] = y1 + ny * r; n++;
        pts[n * 2] = x1 - nx * r; pts[n * 2 + 1] = y1 - ny * r; n++;
    }

    if (cap_start) {
        /* Semicircle dome around (x0, y0) from -n through -u to +n */
        for (int i = 0; i <= 4; i++) {
            float a = (float)i * ((float)M_PI / 4.0f);
            float cn = -cosf(a), cu = -sinf(a);
            pts[n * 2]     = x0 + (nx * cn + ux * cu) * r;
            pts[n * 2 + 1] = y0 + (ny * cn + uy * cu) * r;
            n++;
        }
    } else {
        pts[n * 2] = x0 - nx * r; pts[n * 2 + 1] = y0 - ny * r; n++;
        pts[n * 2] = x0 + nx * r; pts[n * 2 + 1] = y0 + ny * r; n++;
    }

    add_poly(arr, cnt, w, h, color, 1, HUD_POLY_FILL, pts, (uint8_t)n);
}

/* Draw a thick segment (convex rectangle) from (x0,y0) to (x1,y1) of width `thick` */
static void add_thick_seg(HudPolygonCmd *arr, uint8_t *cnt,
                          uint16_t w, uint16_t h,
                          uint8_t color, float x0, float y0, float x1, float y1, float thick) {
    add_round_seg(arr, cnt, w, h, color, x0, y0, x1, y1, thick, 0, 0);
}

/* Extrude a smooth quadratic Bezier centerline P0->P1->P2 into 1 continuous seamless polygon */
static void add_bezier_ribbon(HudPolygonCmd *arr, uint8_t *cnt,
                              uint16_t w, uint16_t h,
                              uint8_t color, float x0, float y0,
                              float x1, float y1, float x2, float y2,
                              float thick, int steps) {
    if (steps < 2) steps = 2;
    if (steps > 14) steps = 14;
    float hw = thick * 0.5f;
    float pts[HUD_MAX_POLY_PTS * 2];
    float rx[15], ry[15];

    for (int i = 0; i <= steps; i++) {
        float t = (float)i / (float)steps;
        float mt = 1.0f - t;
        float px = mt * mt * x0 + 2.0f * mt * t * x1 + t * t * x2;
        float py = mt * mt * y0 + 2.0f * mt * t * y1 + t * t * y2;
        float tx = 2.0f * mt * (x1 - x0) + 2.0f * t * (x2 - x1);
        float ty = 2.0f * mt * (y1 - y0) + 2.0f * t * (y2 - y1);
        float tlen = hypotf(tx, ty);
        if (tlen < 0.01f) { tx = x2 - x0; ty = y2 - y0; tlen = hypotf(tx, ty); }
        if (tlen < 0.01f) { tx = 0.0f; ty = -1.0f; tlen = 1.0f; }
        float nx = (-ty / tlen) * hw;
        float ny = ( tx / tlen) * hw;
        pts[i * 2]     = px + nx;
        pts[i * 2 + 1] = py + ny;
        rx[i] = px - nx;
        ry[i] = py - ny;
    }
    int idx = steps + 1;
    for (int i = steps; i >= 0; i--) {
        pts[idx * 2]     = rx[i];
        pts[idx * 2 + 1] = ry[i];
        idx++;
    }
    add_poly(arr, cnt, w, h, color, 1, HUD_POLY_FILL, pts, (uint8_t)idx);
}

/* Apple CarPlay style open V-chevron arrowhead with rounded apex & rounded wing tips */
static void add_open_chevron(HudPolygonCmd *arr, uint8_t *cnt,
                             uint16_t w, uint16_t h,
                             uint8_t color, float tip_x, float tip_y, float deg,
                             float arm_len, float wing_deg, float thick) {
    for (int sign = -1; sign <= 1; sign += 2) {
        float a = (deg + 180.0f + (float)sign * wing_deg) * ((float)M_PI / 180.0f);
        float wx = tip_x + sinf(a) * arm_len;
        float wy = tip_y - cosf(a) * arm_len;
        add_round_seg(arr, cnt, w, h, color, wx, wy, tip_x, tip_y, thick, 1, 1);
    }
}

/* Draw a smooth S-curve between two vertical tangents (x0, y0) -> (x1, y1) using
 * two tangent MOST 0x021E circular arcs (zero polygon quantization wobble). */
static void add_s_curve_arcs(HudPolygonCmd *arr, uint8_t *cnt,
                             uint16_t w, uint16_t h,
                             uint8_t color, uint8_t lw,
                             float x0, float y0, float x1, float y1,
                             float s) {
    float dx = x1 - x0;
    float dy = y0 - y1; /* positive upward */
    float half_dx = fabsf(dx) * 0.5f;
    if (half_dx < 1.0f || dy < 2.0f) {
        add_round_seg(arr, cnt, w, h, color, x0, y0, x1, y1, (float)lw, 1, 1);
        return;
    }
    float dir = (dx >= 0.0f) ? 1.0f : -1.0f;
    float half_dy = dy * 0.5f;
    /* Circle tangent to vertical line at (x0, y0) passing through midpoint (x0+dx/2, y0-dy/2):
     * (R - half_dx)^2 + half_dy^2 = R^2  =>  R = (half_dx^2 + half_dy^2) / (2 * half_dx) */
    float R = (half_dx * half_dx + half_dy * half_dy) / (2.0f * half_dx);
    float theta = asinf(fminf(1.0f, half_dy / R)) * (180.0f / (float)M_PI);

    /* Ensure center X stays inside canvas bounds */
    float cx1 = x0 + dir * R * s;
    float cx2 = x1 - dir * R * s;
    float r_px = R * s;
    if (cx1 < 2.0f || cx1 > (float)w - 2.0f || cx2 < 2.0f || cx2 > (float)w - 2.0f) {
        add_bezier_ribbon(arr, cnt, w, h, color,
                          x0, y0,
                          x0, (y0 + y1) * 0.5f,
                          x1, y1,
                          (float)lw, 10);
        return;
    }

    if (dir > 0.0f) {
        /* Bottom arc around (x0+R, y0): -90 deg -> -90 + theta deg (CW) */
        add_circle(arr, cnt, w, h, color, lw, HUD_CIRCLE_ARC,
                   cx1, y0, r_px,
                   screen_deg_to_hud_01deg(-90.0f),
                   screen_deg_to_hud_01deg(-90.0f + theta));
        /* Top arc around (x1-R, y1): 90 deg -> 90 + theta deg (CW) */
        add_circle(arr, cnt, w, h, color, lw, HUD_CIRCLE_ARC,
                   cx2, y1, r_px,
                   screen_deg_to_hud_01deg(90.0f),
                   screen_deg_to_hud_01deg(90.0f + theta));
    } else {
        /* Bottom arc around (x0-R, y0): 90 - theta deg -> 90 deg (CW) */
        add_circle(arr, cnt, w, h, color, lw, HUD_CIRCLE_ARC,
                   cx1, y0, r_px,
                   screen_deg_to_hud_01deg(90.0f - theta),
                   screen_deg_to_hud_01deg(90.0f));
        /* Top arc around (x1+R, y1): -90 - theta deg -> -90 deg (CW) */
        add_circle(arr, cnt, w, h, color, lw, HUD_CIRCLE_ARC,
                   cx2, y1, r_px,
                   screen_deg_to_hud_01deg(-90.0f - theta),
                   screen_deg_to_hud_01deg(-90.0f));
    }
    /* Smooth rounded inflection bridge so integer-radius flat-capped arcs meet without a notch */
    float mx = (x0 + x1) * 0.5f;
    float my = (y0 + y1) * 0.5f;
    float tr = (dir * theta) * ((float)M_PI / 180.0f);
    float tux = sinf(tr), tuy = -cosf(tr);
    add_round_seg(arr, cnt, w, h, color,
                  mx - tux * 1.5f * s, my - tuy * 1.5f * s,
                  mx + tux * 1.5f * s, my + tuy * 1.5f * s,
                  (float)lw, 1, 1);
}

/* ==================== Distance & Street Formatting ==================== */

static void format_distance(const RgdState *st, char *dist_out, size_t dcap, char *unit_out, size_t ucap) {
    dist_out[0] = '\0';
    unit_out[0] = '\0';

    if (!st->has_dist_maneuver_m && st->dist_maneuver_str[0] == '\0') {
        return;
    }

    uint8_t u = st->dist_maneuver_unit;
    if (u == 1 || u == 3 || u == 4) {
        /* Imperial: mi / yd / ft — preserve iPhone string if present */
        const char *u_str = (u == 1) ? "mi" : (u == 3) ? "yd" : "ft";
        snprintf(unit_out, ucap, "%s", u_str);
        if (st->dist_maneuver_str[0] != '\0') {
            snprintf(dist_out, dcap, "%s", st->dist_maneuver_str);
        } else {
            snprintf(dist_out, dcap, "%u", (unsigned)st->dist_maneuver_m);
        }
        return;
    }

    /* Metric (meters from 0x000A) */
    uint32_t m = st->dist_maneuver_m;
    if (m >= 10000000u) {
        return;
    }
    if (m >= 10000) {
        uint32_t km = (m + 500) / 1000;
        snprintf(dist_out, dcap, "%u", (unsigned)km);
        snprintf(unit_out, ucap, "km");
    } else if (m >= 1000) {
        uint32_t tenths = (m + 50) / 100;
        if (tenths % 10 == 0) {
            snprintf(dist_out, dcap, "%u", (unsigned)(tenths / 10));
        } else {
            snprintf(dist_out, dcap, "%u.%u", (unsigned)(tenths / 10), (unsigned)(tenths % 10));
        }
        snprintf(unit_out, ucap, "km");
    } else if (m >= 200) {
        uint32_t r = ((m + 25) / 50) * 50;
        snprintf(dist_out, dcap, "%u", (unsigned)r);
        snprintf(unit_out, ucap, "m");
    } else if (m >= 20) {
        uint32_t r = ((m + 5) / 10) * 10;
        snprintf(dist_out, dcap, "%u", (unsigned)r);
        snprintf(unit_out, ucap, "m");
    } else {
        snprintf(dist_out, dcap, "%u", (unsigned)m);
        snprintf(unit_out, ucap, "m");
    }
}

/* ==================== GuidingIcon (120 x 72) Renderer ==================== */

static int resolve_turn_angle(const RgdManeuver *man) {
    if (!man) return 0;
    if (man->has_exit_angle) {
        int ea = man->exit_angle;
        /* Keep side consistent with explicit ManeuverType */
        switch (man->type) {
            case 1: case 13: case 20: case 22: case 47: case 49: case 52:
                if (ea > 0) ea = -ea;
                if (ea == 0) ea = (man->type == 47) ? -135 : (man->type == 13 || man->type == 49 || man->type == 22 || man->type == 52) ? -45 : -90;
                return ea;
            case 2: case 14: case 21: case 23: case 48: case 50: case 53:
                if (ea < 0) ea = -ea;
                if (ea == 0) ea = (man->type == 48) ? 135 : (man->type == 14 || man->type == 50 || man->type == 23 || man->type == 53) ? 45 : 90;
                return ea;
            case 3:
                return 0;
            default:
                return ea;
        }
    }
    switch (man->type) {
        case 1: case 20: return -90;
        case 2: case 21: return 90;
        case 13: case 49: case 22: case 52: return -45;
        case 14: case 50: case 23: case 53: return 45;
        case 47: return -135;
        case 48: return 135;
        case 8: return (man->driving_side == 1) ? -45 : 45;
        case 9: return (man->driving_side == 1) ? 45 : -45;
        case 4: case 18: case 26: case 19: return (man->driving_side == 1) ? 180 : -180;
        default: return 0;
    }
}

static int is_roundabout_maneuver(const RgdManeuver *man) {
    if (!man) return 0;
    if (man->type == 6 || man->type == 7 || man->type == 19) return 1;
    if (man->type >= 28 && man->type <= 46) return 1;
    return man->junction_type == 1 && man->type != 0 && man->type != 5;
}

static void render_guiding_icon(const RgdManeuver *man, int is_arrived,
                                uint16_t w, uint16_t h,
                                HudPolygonCmd *polys, uint8_t *cnt) {
    *cnt = 0;
    float sx = (float)w / 120.0f;
    float sy = (float)h / 72.0f;
    float s = (sx < sy) ? sx : sy;
    float ox = ((float)w - 120.0f * s) * 0.5f;
    float oy = ((float)h - 72.0f * s) * 0.5f;

    #define X(v) (ox + (v) * s)
    #define Y(v) (oy + (v) * s)

    int is_arrival_man = man && (man->type == 10 || man->type == 12 ||
                                 man->type == 24 || man->type == 25 || man->type == 27);

    if (is_arrived || is_arrival_man) {
        /* OEM GI: DestinationMan (libVMostAdapter.so @0x2e0adc) style:
         * native 0x021E HUD_CIRCLE_FILLED destination dot at top + arrow pointing up to it */
        uint8_t mtype = is_arrival_man ? man->type : 12;
        float fx = (mtype == 24) ? 46.0f : (mtype == 25) ? 74.0f : 60.0f;
        uint8_t lw = (uint8_t)lroundf(8.2f * s);
        if (lw < 2) lw = 2;

        if (mtype == 24 || mtype == 25) {
            add_circle(polys, cnt, w, h, HUD_COLOR_WHITE, 2, HUD_CIRCLE_FILLED,
                       X(fx), Y(18.5f), 7.2f * s, 0u, 3599u);
            add_s_curve_arcs(polys, cnt, w, h, HUD_COLOR_WHITE, lw, X(60), Y(62), X(fx), Y(45.5f), s);
            add_round_seg(polys, cnt, w, h, HUD_COLOR_WHITE, X(fx), Y(45.5f), X(fx), Y(35.0f), (float)lw, 1, 0);
            add_open_chevron(polys, cnt, w, h, HUD_COLOR_WHITE, X(fx), Y(33.0f), 0.0f, 11.5f * s, 42.0f, 7.0f * s);
        } else {
            add_circle(polys, cnt, w, h, HUD_COLOR_WHITE, 2, HUD_CIRCLE_FILLED,
                       X(60.0f), Y(20.0f), 7.5f * s, 0u, 3599u);
            add_round_seg(polys, cnt, w, h, HUD_COLOR_WHITE, X(60), Y(62), X(60), Y(36.5f), (float)lw, 1, 0);
            add_open_chevron(polys, cnt, w, h, HUD_COLOR_WHITE, X(60.0f), Y(34.5f), 0.0f, 13.5f * s, 43.0f, 7.4f * s);
        }
        return;
    }

    if (!man || man->type == 0 || man->type == 3 || man->type == 5 || man->type == 15 || man->type == 17) {
        /* Straight ahead / follow road: Apple CarPlay rounded shaft + optional multi-road stubs + open chevron */
        float jx = 60.0f, jy = 38.0f;
        if (man && man->type == 3 && man->junction_angle_count > 0) {
            int has_bg = 0;
            for (int i = 0; i < man->junction_angle_count; i++) {
                int d = man->junction_angles[i];
                if (abs(d) < 18 || abs(abs(d) - 180) < 12) continue;
                has_bg = 1;
                float rad = (float)d * ((float)M_PI / 180.0f);
                add_round_seg(polys, cnt, w, h, HUD_COLOR_DIM,
                              X(jx), Y(jy),
                              X(jx + sinf(rad) * 21.0f), Y(jy - cosf(rad) * 21.0f),
                              6.2f * s, 1, 1);
            }
            if (has_bg) {
                add_round_seg(polys, cnt, w, h, HUD_COLOR_DIM, X(jx), Y(62), X(jx), Y(jy), 6.2f * s, 1, 1);
            }
        }
        add_round_seg(polys, cnt, w, h, HUD_COLOR_WHITE, X(60), Y(62), X(60), Y(14), 8.4f * s, 1, 1);
        add_open_chevron(polys, cnt, w, h, HUD_COLOR_WHITE, X(60), Y(12), 0.0f, 14.5f * s, 43.0f, 7.6f * s);
        return;
    }

    if (man->type == 52 || man->type == 53) {
        /* Highway lane change left/right: hardware S-curve arcs + open chevron */
        float dir = (man->type == 52) ? -1.0f : 1.0f;
        float x_src = 60.0f - dir * 16.0f;
        float x_dst = 60.0f + dir * 16.0f;
        uint8_t lw = (uint8_t)lroundf(8.2f * s);
        if (lw < 2) lw = 2;
        add_round_seg(polys, cnt, w, h, HUD_COLOR_DIM, X(x_src), Y(62), X(x_src), Y(14), 6.0f * s, 1, 1);
        add_round_seg(polys, cnt, w, h, HUD_COLOR_WHITE, X(x_src), Y(62), X(x_src), Y(57), (float)lw, 1, 1);
        add_s_curve_arcs(polys, cnt, w, h, HUD_COLOR_WHITE, lw, X(x_src), Y(57), X(x_dst), Y(16), s);
        add_round_seg(polys, cnt, w, h, HUD_COLOR_WHITE, X(x_dst), Y(16), X(x_dst), Y(13.5f), (float)lw, 1, 0);
        add_open_chevron(polys, cnt, w, h, HUD_COLOR_WHITE, X(x_dst), Y(12.0f), 0.0f, 13.5f * s, 43.0f, 7.4f * s);
        return;
    }

    if (man->type == 4 || man->type == 18 || man->type == 26) {
        /* U-Turn: rounded vertical entry + hardware 180 deg arc (0x021E) + exit stem + open V-chevron */
        float side = (man->driving_side == 1) ? 1.0f : -1.0f;
        float cx = 60.0f, cy = 28.0f, r = 18.0f;
        float x_in = cx - side * r, x_out = cx + side * r;
        uint8_t lw = (uint8_t)lroundf(8.2f * s);
        if (lw < 2) lw = 2;

        add_round_seg(polys, cnt, w, h, HUD_COLOR_WHITE, X(x_in), Y(62), X(x_in), Y(cy), (float)lw, 1, 1);
        add_circle(polys, cnt, w, h, HUD_COLOR_WHITE, lw, HUD_CIRCLE_ARC,
                   X(cx), Y(cy), r * s, 0u, 1800u);
        add_round_seg(polys, cnt, w, h, HUD_COLOR_WHITE, X(x_out), Y(cy), X(x_out), Y(54.5f), (float)lw, 1, 0);
        add_open_chevron(polys, cnt, w, h, HUD_COLOR_WHITE, X(x_out), Y(56.5f), 180.0f, 13.5f * s, 43.0f, 7.4f * s);
        return;
    }

    if (is_roundabout_maneuver(man)) {
        /* Apple CarPlay roundabout using native hardware circular arcs (0x021E Kreisbogen):
         * dim full ring + proportional dim stubs + white entry stem + white ring arc + white exit stem + open chevron */
        float cx = 60.0f, cy = 34.0f, r = 15.0f;
        uint8_t ring_dim_lw = (uint8_t)lroundf(5.8f * s);
        uint8_t path_lw = (uint8_t)lroundf(7.6f * s);
        if (ring_dim_lw < 2) ring_dim_lw = 2;
        if (path_lw < 2) path_lw = 2;
        float path_w = (float)path_lw;

        int exit_deg = resolve_turn_angle(man);
        if (man->type == 19) exit_deg = 180;

        /* 1. Dim background full ring via 0x021E (HUD_CIRCLE_FULL avoids 1-deg gap at 9 o'clock) */
        add_circle(polys, cnt, w, h, HUD_COLOR_DIM, ring_dim_lw, HUD_CIRCLE_FULL,
                   X(cx), Y(cy), r * s, 0u, 3599u);

        /* 2. Proportional rounded-cap dimmed exit stubs */
        for (int i = 0; i < man->junction_angle_count; i++) {
            int d = man->junction_angles[i];
            if (abs(d - exit_deg) < 18 || abs(abs(d) - 180) < 12) continue;
            float rad = (float)d * ((float)M_PI / 180.0f);
            float x0 = cx + sinf(rad) * (r + 1.0f);
            float y0 = cy - cosf(rad) * (r + 1.0f);
            float x1 = cx + sinf(rad) * (r + 9.5f);
            float y1 = cy - cosf(rad) * (r + 9.5f);
            add_round_seg(polys, cnt, w, h, HUD_COLOR_DIM, X(x0), Y(y0), X(x1), Y(y1), 5.6f * s, 1, 1);
        }

        int ccw = (man->driving_side == 0);
        int render_exit = exit_deg;
        if (abs(abs(render_exit) - 180) < 10) {
            render_exit = ccw ? -135 : 135;
        }
        uint32_t exit_hud = screen_deg_to_hud_01deg((float)render_exit);
        uint32_t entry_hud = 2700u; /* 180 deg screen (6 o'clock) -> 2700 (0.1 deg) */

        /* 3. White entry stem from bottom to ring bottom (with round caps at both ends for seamless join) */
        add_round_seg(polys, cnt, w, h, HUD_COLOR_WHITE, X(cx), Y(62.5f), X(cx), Y(cy + r), path_w, 1, 1);

        /* 4. White roundabout arc via 0x021E (RHT sweeps CCW: start=exit -> end=2700; LHT sweeps CW: start=2700 -> end=exit) */
        if (exit_hud != entry_hud) {
            uint32_t a_start = ccw ? exit_hud : entry_hud;
            uint32_t a_end   = ccw ? entry_hud : exit_hud;
            add_circle(polys, cnt, w, h, HUD_COLOR_WHITE, path_lw, HUD_CIRCLE_ARC,
                       X(cx), Y(cy), r * s, a_start, a_end);
        }

        /* 5. White exit stem + open V-chevron */
        float exr = (float)render_exit * ((float)M_PI / 180.0f);
        float ux = sinf(exr), uy = -cosf(exr);
        add_round_seg(polys, cnt, w, h, HUD_COLOR_WHITE,
                      X(cx + ux * r), Y(cy + uy * r),
                      X(cx + ux * (r + 14.5f)), Y(cy + uy * (r + 14.5f)),
                      path_w, 1, 0);
        add_open_chevron(polys, cnt, w, h, HUD_COLOR_WHITE,
                         X(cx + ux * (r + 16.0f)), Y(cy + uy * (r + 16.0f)),
                         (float)render_exit, 11.5f * s, 43.0f, 6.8f * s);
        return;
    }

    /* Standard & multi-road intersection turn / off-ramp / Y-fork (Apple CarPlay style):
     * Proportional rounded-cap dimmed junction stubs + straight entry/exit stems + hardware circular arc (0x021E) + open V-chevron */
    int turn_deg = resolve_turn_angle(man);
    int abs_deg = abs(turn_deg);
    float jx = 60.0f;
    float jy = (abs_deg >= 115) ? 21.0f : (abs_deg <= 60) ? 39.0f : 36.0f;

    /* 1. Proportional rounded-cap dimmed background stubs for other roads at the junction */
    int has_bg = 0;
    int has_forward = 0;
    for (int i = 0; i < man->junction_angle_count; i++) {
        int d = man->junction_angles[i];
        if (d == 0) has_forward = 1;
        if (abs(d - turn_deg) < 18 || abs(abs(d) - 180) < 12) continue;
        has_bg = 1;
        float rad = (float)d * ((float)M_PI / 180.0f);
        float bg_len = (abs(d) <= 45) ? 19.5f : 20.5f;
        add_round_seg(polys, cnt, w, h, HUD_COLOR_DIM,
                      X(jx), Y(jy),
                      X(jx + sinf(rad) * bg_len), Y(jy - cosf(rad) * bg_len),
                      6.0f * s, 1, 1);
    }
    if ((man->type == 8 || man->type == 22 || man->type == 23) && !has_forward) {
        has_bg = 1;
        add_round_seg(polys, cnt, w, h, HUD_COLOR_DIM, X(jx), Y(jy), X(jx), Y(jy - 19.5f), 6.0f * s, 1, 1);
    }
    if (has_bg) {
        add_round_seg(polys, cnt, w, h, HUD_COLOR_DIM, X(jx), Y(62), X(jx), Y(jy), 6.0f * s, 1, 1);
    }

    /* 2. Active white path with hardware circular arc fillet + open V-chevron */
    float trad = (float)turn_deg * ((float)M_PI / 180.0f);
    float ux = sinf(trad), uy = -cosf(trad);

    if (abs_deg < 12) {
        if (abs_deg < 5) {
            add_round_seg(polys, cnt, w, h, HUD_COLOR_WHITE, X(jx), Y(62), X(jx), Y(14), 8.4f * s, 1, 1);
            add_open_chevron(polys, cnt, w, h, HUD_COLOR_WHITE, X(jx), Y(12), 0.0f, 14.5f * s, 43.0f, 7.6f * s);
        } else {
            add_round_seg(polys, cnt, w, h, HUD_COLOR_WHITE, X(jx), Y(62), X(jx), Y(jy + 6.0f), 8.4f * s, 1, 1);
            add_round_seg(polys, cnt, w, h, HUD_COLOR_WHITE, X(jx), Y(jy + 6.0f), X(jx + ux * 24.0f), Y(jy + 6.0f + uy * 24.0f), 8.4f * s, 1, 0);
            add_open_chevron(polys, cnt, w, h, HUD_COLOR_WHITE, X(jx + ux * 25.5f), Y(jy + 6.0f + uy * 25.5f), (float)turn_deg, 13.8f * s, 43.0f, 7.6f * s);
        }
    } else {
        float side = (turn_deg > 0) ? 1.0f : -1.0f;
        float half_rad = ((float)abs_deg * 0.5f) * ((float)M_PI / 180.0f);
        float tan_h = tanf(half_rad);
        float desired_R = (abs_deg <= 60) ? 24.0f : (abs_deg <= 105) ? 18.0f : 14.0f;
        float d = desired_R * tan_h;
        if (d > 21.0f) { d = 21.0f; desired_R = d / tan_h; }
        if (d < 8.0f)  { d = 8.0f;  desired_R = d / tan_h; }
        /* Keep arc center X safely inside [8, 112] in 120-space */
        if (desired_R > 48.0f) { desired_R = 48.0f; d = desired_R * tan_h; }

        float p0y = jy + d;
        float fc_y_px = roundf(Y(p0y));
        float r_px = roundf(desired_R * s);
        if (r_px < 2.0f) r_px = 2.0f;
        float fc_x_px = roundf(X(jx) + side * r_px);
        float sx_px = fc_x_px - side * r_px;
        uint8_t lw = (uint8_t)lroundf(8.4f * s);
        if (lw < 2) lw = 2;

        /* Entry straight stem (aligned exactly to snapped 0x021E start tangent) */
        if (fc_y_px < Y(61.0f)) {
            add_round_seg(polys, cnt, w, h, HUD_COLOR_WHITE, sx_px, Y(62.0f), sx_px, fc_y_px, (float)lw, 1, 1);
        } else {
            add_round_seg(polys, cnt, w, h, HUD_COLOR_WHITE, sx_px, fc_y_px + 1.0f, sx_px, fc_y_px, (float)lw, 1, 1);
        }

        /* Hardware circular arc fillet via 0x021E */
        uint32_t a_start, a_end;
        if (turn_deg > 0) {
            a_start = screen_deg_to_hud_01deg(-90.0f);
            a_end   = screen_deg_to_hud_01deg(-90.0f + (float)turn_deg);
        } else {
            a_start = screen_deg_to_hud_01deg(90.0f + (float)turn_deg);
            a_end   = screen_deg_to_hud_01deg(90.0f);
        }
        add_circle(polys, cnt, w, h, HUD_COLOR_WHITE, lw, HUD_CIRCLE_ARC,
                   fc_x_px, fc_y_px, r_px, a_start, a_end);

        /* Exit straight stem starting cleanly at the snapped integer 0x021E endpoint */
        float abs_rad = (float)abs_deg * ((float)M_PI / 180.0f);
        float ex_px = fc_x_px - side * r_px * cosf(abs_rad);
        float ey_px = fc_y_px - r_px * sinf(abs_rad);
        float neck_px = ((abs_deg <= 60) ? 17.0f : (abs_deg <= 105) ? 15.0f : 14.0f) * s;
        float tip_px = neck_px + 1.5f * s;
        add_round_seg(polys, cnt, w, h, HUD_COLOR_WHITE,
                      ex_px - ux * 0.5f, ey_px - uy * 0.5f,
                      ex_px + ux * neck_px, ey_px + uy * neck_px,
                      (float)lw, 1, 0);
        add_open_chevron(polys, cnt, w, h, HUD_COLOR_WHITE,
                         ex_px + ux * tip_px, ey_px + uy * tip_px,
                         (float)turn_deg, 13.8f * s, 43.0f, 7.6f * s);
    }

    #undef X
    #undef Y
}

/* ==================== LaneGuidance (184 x 17) Renderer ==================== */

/* Compact symmetric 4-point notched chevron arrowhead for 17px LaneGuidance
 * (avoids thin diagonal integer-quantization dropouts while keeping an open-chevron notch). */
static void add_lane_arrowhead(HudPolygonCmd *arr, uint8_t *cnt,
                               uint16_t w, uint16_t h,
                               uint8_t color, float tip_x, float tip_y, float deg,
                               float len_px, float half_w_px, float notch_px) {
    float rad = deg * ((float)M_PI / 180.0f);
    float ux = sinf(rad), uy = -cosf(rad);
    float nx = -uy,       ny =  ux;
    float pts[8] = {
        tip_x,                                         tip_y,
        tip_x - ux * len_px + nx * half_w_px,          tip_y - uy * len_px + ny * half_w_px,
        tip_x - ux * (len_px - notch_px),              tip_y - uy * (len_px - notch_px),
        tip_x - ux * len_px - nx * half_w_px,          tip_y - uy * len_px - ny * half_w_px
    };
    add_poly(arr, cnt, w, h, color, 1, HUD_POLY_FILL, pts, 4);
}

static void draw_lane_branch(HudPolygonCmd *polys, uint8_t *cnt,
                             uint16_t w, uint16_t h,
                             float cx, float y_bot, float h_scale,
                             int16_t angle, uint8_t color, uint8_t lw,
                             float stem_thick, int draw_entry_stem) {
    int abs_a = abs((int)angle);
    float ah_len   = 3.8f * h_scale;
    float ah_hw    = 2.9f * h_scale;
    float ah_notch = 1.0f * h_scale;

    if (abs_a < 15) {
        /* Straight lane arrow: exact integer-aligned vertical stem + symmetric notched chevron */
        float y_top = 4.0f * h_scale;
        float y_tip = 2.0f * h_scale;
        if (draw_entry_stem) {
            add_round_seg(polys, cnt, w, h, color, cx, y_bot, cx, y_top, stem_thick, 1, 0);
        } else {
            add_round_seg(polys, cnt, w, h, color, cx, 10.0f * h_scale, cx, y_top, stem_thick, 1, 0);
        }
        add_lane_arrowhead(polys, cnt, w, h, color, cx, y_tip, 0.0f, ah_len, ah_hw, ah_notch);
        return;
    }

    if (abs_a >= 160) {
        /* U-Turn lane arrow using 0x021E 180-deg top semicircle (integer center & radius) */
        float side = (angle > 0) ? 1.0f : -1.0f;
        float r_px = roundf(3.5f * h_scale);
        if (r_px < 3.0f) r_px = 3.0f;
        float arc_cy = roundf(6.0f * h_scale);
        float arc_cx = roundf(cx + side * r_px);
        float ix = arc_cx - side * r_px;
        float ex = arc_cx + side * r_px;
        if (draw_entry_stem) {
            add_round_seg(polys, cnt, w, h, color, ix, y_bot, ix, arc_cy, stem_thick, 1, 0);
        }
        add_circle(polys, cnt, w, h, color, lw, HUD_CIRCLE_ARC,
                   arc_cx, arc_cy, r_px,
                   screen_deg_to_hud_01deg(-90.0f),
                   screen_deg_to_hud_01deg(90.0f));
        add_round_seg(polys, cnt, w, h, color, ex, arc_cy, ex, 11.0f * h_scale, stem_thick, 0, 0);
        add_lane_arrowhead(polys, cnt, w, h, color, ex, 13.0f * h_scale, 180.0f,
                           3.4f * h_scale, 2.7f * h_scale, 0.9f * h_scale);
        return;
    }

    /* Turn branch (15..159 deg): integer-snapped arc center & radius so stems meet 0x021E with zero notch */
    float side = (angle > 0) ? 1.0f : -1.0f;
    float rad = (float)angle * ((float)M_PI / 180.0f);
    float ux = sinf(rad), uy = -cosf(rad);
    float abs_rad = (float)abs_a * ((float)M_PI / 180.0f);

    float desired_R = (abs_a <= 55) ? 6.0f * h_scale : (abs_a <= 105) ? 4.0f * h_scale : 3.0f * h_scale;
    float r_int = roundf(desired_R);
    if (r_int < 2.0f) r_int = 2.0f;

    /* Integer arc center: p0y chosen so top of arc stays clean inside [2..15] */
    float p0y_int = roundf((abs_a <= 55) ? 11.0f * h_scale : (abs_a <= 105) ? 10.0f * h_scale : 11.0f * h_scale);
    float fc_x = roundf(cx + side * r_int);
    float fc_y = p0y_int;
    float sx = fc_x - side * r_int; /* exact tangent start X on integer grid */

    if (draw_entry_stem && p0y_int < y_bot - 0.4f * h_scale) {
        add_round_seg(polys, cnt, w, h, color, sx, y_bot, sx, p0y_int, stem_thick, 1, 0);
    }

    uint32_t a_start, a_end;
    if (angle > 0) {
        a_start = screen_deg_to_hud_01deg(-90.0f);
        a_end   = screen_deg_to_hud_01deg(-90.0f + (float)angle);
    } else {
        a_start = screen_deg_to_hud_01deg(90.0f + (float)angle);
        a_end   = screen_deg_to_hud_01deg(90.0f);
    }
    add_circle(polys, cnt, w, h, color, lw, HUD_CIRCLE_ARC,
               fc_x, fc_y, r_int, a_start, a_end);

    /* Exact endpoint of the integer 0x021E circle at angle `angle` */
    float ex = fc_x - side * r_int * cosf(abs_rad);
    float ey = fc_y - r_int * sinf(abs_rad);

    float neck_len = (abs_a <= 55) ? 4.4f * h_scale : (abs_a <= 105) ? 4.8f * h_scale : 2.8f * h_scale;
    float tip_x = ex + ux * (neck_len + 2.0f * h_scale);
    float tip_y = ey + uy * (neck_len + 2.0f * h_scale);
    /* Start exit stem 0.4px inside the arc so integer rounding never leaves a gap */
    add_round_seg(polys, cnt, w, h, color,
                  ex - ux * 0.4f, ey - uy * 0.4f,
                  ex + ux * neck_len, ey + uy * neck_len,
                  stem_thick, 0, 0);
    add_lane_arrowhead(polys, cnt, w, h, color, tip_x, tip_y, (float)angle,
                       ah_len, ah_hw, ah_notch);
}

static void add_lane_ellipsis(HudPolygonCmd *polys, uint8_t *cnt,
                              uint16_t w, uint16_t h,
                              float cx_start, float step_x, float cy, float h_scale) {
    /* BMW/MINI factory LaneGuidance style: 3 0x021E filled dots when lanes overflow */
    for (int d = 0; d < 3; d++) {
        add_circle(polys, cnt, w, h, HUD_COLOR_DIM, 1, HUD_CIRCLE_FILLED,
                   cx_start + (float)d * step_x, cy, 1.0f * h_scale, 0, 3599);
    }
}

static void render_lane_guidance(const RgdLaneEvent *le, uint16_t w, uint16_t h,
                                 HudPolygonCmd *polys, uint8_t *cnt) {
    *cnt = 0;
    if (!le || le->lane_count == 0) return;

    int total_n = le->lane_count;
    if (total_n > HUD_MAX_LANES) total_n = HUD_MAX_LANES;
    float h_scale = (float)h / 17.0f;

    /* Show up to 7 lanes directly; if > 7, focus a 6-lane window around recommended lanes + ... dots */
    int win_start = 0;
    int n = total_n;
    if (total_n > 7) {
        int good_min = -1, good_max = -1;
        for (int i = 0; i < total_n; i++) {
            if (le->lanes[i].status >= 1) {
                if (good_min < 0) good_min = i;
                good_max = i;
            }
        }
        n = 6;
        if (good_min >= 0) {
            int center = (good_min + good_max) / 2;
            win_start = center - n / 2;
            if (win_start < 0) win_start = 0;
            if (win_start + n > total_n) win_start = total_n - n;
        }
    }
    int trunc_left  = (win_start > 0);
    int trunc_right = (win_start + n < total_n);

    float avail_w = (float)w - (trunc_left ? 16.0f : 0.0f) - (trunc_right ? 16.0f : 0.0f);
    /* Keep slot_w an even integer so all lane centers cx fall on exact integer pixels */
    float slot_w = (trunc_left || trunc_right) ? 24.0f * h_scale : 26.0f * h_scale;
    if (slot_w * (float)n > avail_w) slot_w = floorf((avail_w / (float)n) * 0.5f) * 2.0f;
    if (slot_w < 16.0f) slot_w = 16.0f;

    float total_w = slot_w * (float)n;
    float left_pad = trunc_left ? 16.0f : 0.0f;
    float start_x = roundf(left_pad + (avail_w - total_w) * 0.5f);
    float y_bot = roundf((float)h - 1.5f * h_scale);

    if (trunc_left) {
        add_lane_ellipsis(polys, cnt, w, h, start_x - 12.0f, 3.5f, 8.5f * h_scale, h_scale);
    }

    for (int i = 0; i < n; i++) {
        const RgdLaneInfo *ln = &le->lanes[win_start + i];
        float cx = start_x + ((float)i + 0.5f) * slot_w;
        int is_good = (ln->status >= 1);

        int16_t hi = 1000;
        if (ln->has_highlighted) hi = ln->highlighted_angle;
        else if (ln->angle_count > 0) hi = ln->angles[0];

        uint8_t base_color = is_good ? HUD_COLOR_WHITE : HUD_COLOR_DIM;
        /* Exact integer-matched stroke width (lw = 2) so 0x021E arcs and stems join flush */
        uint8_t lw = (uint8_t)lroundf(2.0f * h_scale);
        if (lw < 2) lw = 2;
        float stem_thick = (float)lw;

        int drew_any = 0;
        int drew_hi = 0;
        for (int k = 0; k < (int)ln->angle_count; k++) {
            int16_t a = ln->angles[k];
            if (is_good && valid_angle(hi) && abs((int)a - (int)hi) <= 5) {
                drew_hi = 1;
                continue; /* draw highlighted on top */
            }
            /* Draw full dim entry stem for non-good lanes (handles multi-turn e.g. -45 + 90 cleanly) */
            int draw_stem = !is_good;
            draw_lane_branch(polys, cnt, w, h, cx, y_bot, h_scale, a, HUD_COLOR_DIM, lw, stem_thick, draw_stem);
            drew_any = 1;
        }
        if (is_good && valid_angle(hi)) {
            draw_lane_branch(polys, cnt, w, h, cx, y_bot, h_scale, hi, HUD_COLOR_WHITE, lw, stem_thick, 1);
            drew_any = 1;
            drew_hi = 1;
        }
        if (!drew_any && !drew_hi) {
            /* Fallback straight arrow if phone sent lane slot without angles */
            draw_lane_branch(polys, cnt, w, h, cx, y_bot, h_scale, 0, base_color, lw, stem_thick, 1);
        }

        /* Crisp 2-dash highway lane divider between lanes */
        if (i + 1 < n) {
            float sx = start_x + (float)(i + 1) * slot_w;
            add_thick_seg(polys, cnt, w, h, HUD_COLOR_DIM, sx, y_bot, sx, 11.5f * h_scale, 1.2f * h_scale);
            add_thick_seg(polys, cnt, w, h, HUD_COLOR_DIM, sx, 6.5f * h_scale, sx, 2.5f * h_scale, 1.2f * h_scale);
        }
    }

    if (trunc_right) {
        add_lane_ellipsis(polys, cnt, w, h, start_x + total_w + 5.0f, 3.5f, 8.5f * h_scale, h_scale);
    }
}

void hud_render_build_frame(const RgdState *st, uint64_t now_ms,
                            uint16_t icon_w, uint16_t icon_h,
                            uint16_t lane_w, uint16_t lane_h,
                            HudRenderFrame *out) {
    memset(out, 0, sizeof(*out));
    if (!icon_w) icon_w = 120;
    if (!icon_h) icon_h = 72;
    if (!lane_w) lane_w = 184;
    if (!lane_h) lane_h = 17;

    if (!rgd_state_should_show(st, now_ms)) {
        out->visible = 0;
        out->content_hash = 0;
        return;
    }

    out->visible = 1;
    format_distance(st, out->dist_text, sizeof(out->dist_text), out->unit_text, sizeof(out->unit_text));

    const RgdManeuver *man = st->has_man_current ? find_maneuver(st, st->man_current_idx) : NULL;
    if (!man && st->man_count > 0 && st->route_state != 2) man = find_maneuver(st, 0);

    int is_arrived_state = (st->route_state == 2);
    int is_arrival_man = man && (man->type == 10 || man->type == 12 ||
                                 man->type == 24 || man->type == 25 || man->type == 27);
    if (is_arrived_state && !is_arrival_man) {
        man = NULL;
    }

    if (is_arrived_state || is_arrival_man) {
        if (st->dest_name[0] != '\0') {
            snprintf(out->street_text, sizeof(out->street_text), "%s", st->dest_name);
        } else if (man && man->desc[0] != '\0') {
            snprintf(out->street_text, sizeof(out->street_text), "%s", man->desc);
        } else if (man && man->after_road[0] != '\0') {
            snprintf(out->street_text, sizeof(out->street_text), "%s", man->after_road);
        } else if (st->current_road[0] != '\0') {
            snprintf(out->street_text, sizeof(out->street_text), "%s", st->current_road);
        }
    } else if (man && man->after_road[0] != '\0') {
        snprintf(out->street_text, sizeof(out->street_text), "%s", man->after_road);
    } else if (st->current_road[0] != '\0') {
        snprintf(out->street_text, sizeof(out->street_text), "%s", st->current_road);
    } else if (man && man->desc[0] != '\0') {
        snprintf(out->street_text, sizeof(out->street_text), "%s", man->desc);
    } else if (st->dest_name[0] != '\0') {
        snprintf(out->street_text, sizeof(out->street_text), "%s", st->dest_name);
    }

    render_guiding_icon(man, is_arrived_state, icon_w, icon_h, out->icon_polys, &out->icon_poly_count);

    const RgdLaneEvent *le = NULL;
    if (!is_arrived_state) {
        if (st->lane_guidance_showing == 1 && st->has_lane_idx) {
            le = find_lane_event(st, st->lane_guidance_idx);
        } else if (!st->has_lane_showing) {
            if (st->has_lane_idx) {
                le = find_lane_event(st, st->lane_guidance_idx);
            } else if (man && man->has_linked_lane) {
                le = find_lane_event(st, man->linked_lane_idx);
            }
        }
    }
    if (le && le->lane_count > 0) {
        out->lane_visible = 1;
        render_lane_guidance(le, lane_w, lane_h, out->lane_polys, &out->lane_poly_count);
    } else {
        out->lane_visible = 0;
        out->lane_poly_count = 0;
    }

    uint32_t h = 2166136261u;
    h = fnv1a_update(h, &out->visible, 1);
    h = fnv1a_update(h, &out->lane_visible, 1);
    h = fnv1a_update(h, out->dist_text, strlen(out->dist_text));
    h = fnv1a_update(h, out->unit_text, strlen(out->unit_text));
    h = fnv1a_update(h, out->street_text, strlen(out->street_text));
    h = fnv1a_update(h, &out->icon_poly_count, 1);
    h = fnv1a_update(h, out->icon_polys, out->icon_poly_count * sizeof(HudPolygonCmd));
    h = fnv1a_update(h, &out->lane_poly_count, 1);
    h = fnv1a_update(h, out->lane_polys, out->lane_poly_count * sizeof(HudPolygonCmd));
    out->content_hash = h ? h : 1u;
}

void hud_render_build_test_frame(uint16_t icon_w, uint16_t icon_h,
                                 uint16_t lane_w, uint16_t lane_h,
                                 const char *street_text,
                                 HudRenderFrame *out) {
    RgdState st;
    rgd_state_init(&st);
    st.route_state = 1;
    st.ever_active = 1;
    st.last_active_ms = 1000;
    st.has_dist_maneuver_m = 1;
    st.dist_maneuver_m = 350;
    st.dist_maneuver_unit = 2;
    st.has_man_current = 1;
    st.man_current_idx = 0;
    st.man_count = 1;

    RgdManeuver *m = &st.maneuvers[0];
    m->valid = 1;
    m->index = 0;
    m->type = 2; /* RIGHT_TURN */
    m->has_exit_angle = 1;
    m->exit_angle = 90;
    m->junction_angle_count = 2;
    m->junction_angles[0] = 0;
    m->junction_angles[1] = 90;
    snprintf(m->after_road, sizeof(m->after_road), "%s", street_text ? street_text : "HUD Bridge Test");

    st.lane_guidance_showing = 1;
    st.has_lane_idx = 1;
    st.lane_guidance_idx = 0;
    RgdLaneEvent *le = &st.lane_events[0];
    le->valid = 1;
    le->index = 0;
    le->lane_count = 4;
    /* Lane 0: left turn (dim) */
    le->lanes[0].position = 0; le->lanes[0].status = 0;
    le->lanes[0].angle_count = 1; le->lanes[0].angles[0] = -90;
    /* Lane 1: straight (dim) */
    le->lanes[1].position = 1; le->lanes[1].status = 0;
    le->lanes[1].angle_count = 1; le->lanes[1].angles[0] = 0;
    /* Lane 2: straight + right (preferred right) */
    le->lanes[2].position = 2; le->lanes[2].status = 2;
    le->lanes[2].angle_count = 2; le->lanes[2].angles[0] = 0; le->lanes[2].angles[1] = 90;
    le->lanes[2].has_highlighted = 1; le->lanes[2].highlighted_angle = 90;
    /* Lane 3: right turn (good) */
    le->lanes[3].position = 3; le->lanes[3].status = 1;
    le->lanes[3].angle_count = 1; le->lanes[3].angles[0] = 90;
    le->lanes[3].has_highlighted = 1; le->lanes[3].highlighted_angle = 90;

    hud_render_build_frame(&st, 1000, icon_w, icon_h, lane_w, lane_h, out);
}

static uint32_t utf8_next_cp(const uint8_t **p) {
    const uint8_t *s = *p;
    uint8_t c0 = s[0];
    if (c0 < 0x80) {
        *p = s + 1;
        return c0;
    }
    if ((c0 & 0xE0) == 0xC0 && (s[1] & 0xC0) == 0x80) {
        *p = s + 2;
        return ((uint32_t)(c0 & 0x1F) << 6) | (uint32_t)(s[1] & 0x3F);
    }
    if ((c0 & 0xF0) == 0xE0 && (s[1] & 0xC0) == 0x80 && (s[2] & 0xC0) == 0x80) {
        *p = s + 3;
        return ((uint32_t)(c0 & 0x0F) << 12) | ((uint32_t)(s[1] & 0x3F) << 6) | (uint32_t)(s[2] & 0x3F);
    }
    if ((c0 & 0xF8) == 0xF0 && (s[1] & 0xC0) == 0x80 && (s[2] & 0xC0) == 0x80 && (s[3] & 0xC0) == 0x80) {
        *p = s + 4;
        return ((uint32_t)(c0 & 0x07) << 18) | ((uint32_t)(s[1] & 0x3F) << 12) |
               ((uint32_t)(s[2] & 0x3F) << 6) | (uint32_t)(s[3] & 0x3F);
    }
    *p = s + 1;
    return '?';
}

size_t hud_encode_most_string(const char *utf8, int force_ascii, uint8_t *out, size_t out_cap) {
    if (!out || out_cap < 3) return 0;
    if (!utf8) utf8 = "";

    int has_non_ascii = 0;
    for (const uint8_t *p = (const uint8_t *)utf8; *p; p++) {
        if (*p >= 0x80) {
            has_non_ascii = 1;
            break;
        }
    }

    if (force_ascii || !has_non_ascii) {
        out[0] = 0x01; /* 8-bit ISO-8859-1 / ASCII */
        size_t pos = 1;
        for (const uint8_t *p = (const uint8_t *)utf8; *p && pos + 1 < out_cap; ) {
            uint32_t cp = utf8_next_cp(&p);
            if (cp < 0x20 && cp != '\t') continue;
            if (cp >= 0x80) continue;
            out[pos++] = (uint8_t)cp;
        }
        out[pos++] = 0x00;
        return pos;
    }

    /* Verified in libvmostcpp.so.1.6.0 writeMOSTString(encoding=0) + writeUInt16():
     * tag 0x00 + UTF-16 Big-Endian code units + 0x0000 NUL terminator */
    out[0] = 0x00;
    size_t pos = 1;
    for (const uint8_t *p = (const uint8_t *)utf8; *p && pos + 4 <= out_cap; ) {
        uint32_t cp = utf8_next_cp(&p);
        if (cp < 0x20 && cp != '\t') continue;
        if (cp <= 0xFFFFu) {
            out[pos++] = (uint8_t)((cp >> 8) & 0xFF);
            out[pos++] = (uint8_t)(cp & 0xFF);
        } else if (pos + 6 <= out_cap) {
            uint32_t u = cp - 0x10000u;
            uint16_t hi = (uint16_t)(0xD800u + (u >> 10));
            uint16_t lo = (uint16_t)(0xDC00u + (u & 0x3FFu));
            out[pos++] = (uint8_t)(hi >> 8);
            out[pos++] = (uint8_t)(hi & 0xFF);
            out[pos++] = (uint8_t)(lo >> 8);
            out[pos++] = (uint8_t)(lo & 0xFF);
        }
    }
    if (pos + 2 > out_cap) pos = out_cap - 2;
    out[pos++] = 0x00;
    out[pos++] = 0x00;
    return pos;
}

