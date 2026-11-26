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

/* Draw a thick segment (convex rectangle) from (x0,y0) to (x1,y1) of width `thick` */
static void add_thick_seg(HudPolygonCmd *arr, uint8_t *cnt,
                          uint16_t w, uint16_t h,
                          uint8_t color, float x0, float y0, float x1, float y1, float thick) {
    float dx = x1 - x0;
    float dy = y1 - y0;
    float len = hypotf(dx, dy);
    if (len < 0.5f) return;
    float nx = (-dy / len) * (thick * 0.5f);
    float ny = ( dx / len) * (thick * 0.5f);
    float pts[8] = {
        x0 + nx, y0 + ny,
        x1 + nx, y1 + ny,
        x1 - nx, y1 - ny,
        x0 - nx, y0 - ny
    };
    add_poly(arr, cnt, w, h, color, 2, HUD_POLY_FILL, pts, 4);
}

/* Draw a road segment from (x0,y0) to (x1,y1) with a rounded dome tip at (x1,y1) in 1 convex polygon */
static void add_cap_seg(HudPolygonCmd *arr, uint8_t *cnt,
                        uint16_t w, uint16_t h,
                        uint8_t color, float x0, float y0, float x1, float y1, float thick) {
    float dx = x1 - x0;
    float dy = y1 - y0;
    float len = hypotf(dx, dy);
    if (len < 0.5f) return;
    float ux = dx / len;
    float uy = dy / len;
    float r = thick * 0.5f;
    float nx = -uy * r;
    float ny =  ux * r;
    float pts[12] = {
        x0 + nx, y0 + ny,
        x1 + nx, y1 + ny,
        x1 + nx * 0.65f + ux * (r * 0.72f), y1 + ny * 0.65f + uy * (r * 0.72f),
        x1 - nx * 0.65f + ux * (r * 0.72f), y1 - ny * 0.65f + uy * (r * 0.72f),
        x1 - nx, y1 - ny,
        x0 - nx, y0 - ny
    };
    add_poly(arr, cnt, w, h, color, 2, HUD_POLY_FILL, pts, 6);
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

/* Extrude a circular arc around (cx, cy) radius r from deg0 to deg1 (0=top, +90=right)
 * into 1 single continuous polygon (no internal seams/rectangles!). */
static void add_arc_ribbon(HudPolygonCmd *arr, uint8_t *cnt,
                           uint16_t w, uint16_t h,
                           uint8_t color, float cx, float cy, float r,
                           float deg0, float deg1, float thick, int steps) {
    if (steps < 2) steps = 2;
    if (steps > 24) steps = 24;
    if (2 * (steps + 1) > HUD_MAX_POLY_PTS) steps = (HUD_MAX_POLY_PTS / 2) - 1;
    float hw = thick * 0.5f;
    float r_out = r + hw;
    float r_in  = (r > hw + 1.0f) ? (r - hw) : 1.0f;
    float pts[HUD_MAX_POLY_PTS * 2];
    float ix[25], iy[25];

    for (int i = 0; i <= steps; i++) {
        float deg = deg0 + (deg1 - deg0) * ((float)i / (float)steps);
        float rad = deg * ((float)M_PI / 180.0f);
        float ux = sinf(rad);
        float uy = -cosf(rad);
        pts[i * 2]     = cx + ux * r_out;
        pts[i * 2 + 1] = cy + uy * r_out;
        ix[i] = cx + ux * r_in;
        iy[i] = cy + uy * r_in;
    }
    int idx = steps + 1;
    for (int i = steps; i >= 0; i--) {
        pts[idx * 2]     = ix[i];
        pts[idx * 2 + 1] = iy[i];
        idx++;
    }
    add_poly(arr, cnt, w, h, color, 1, HUD_POLY_FILL, pts, (uint8_t)idx);
}

/* Extrude an arbitrary centerline polyline (px[0..n-1], py[0..n-1]) into 1 single continuous ribbon polygon */
static void add_polyline_ribbon(HudPolygonCmd *arr, uint8_t *cnt,
                                uint16_t w, uint16_t h,
                                uint8_t color, const float *px, const float *py,
                                int n, float thick) {
    if (n < 2) return;
    if (2 * n > HUD_MAX_POLY_PTS) n = HUD_MAX_POLY_PTS / 2;
    float hw = thick * 0.5f;
    float pts[HUD_MAX_POLY_PTS * 2];
    float rx[HUD_MAX_POLY_PTS], ry[HUD_MAX_POLY_PTS];

    for (int i = 0; i < n; i++) {
        float tx, ty;
        if (i == 0) {
            tx = px[1] - px[0];
            ty = py[1] - py[0];
        } else if (i == n - 1) {
            tx = px[n - 1] - px[n - 2];
            ty = py[n - 1] - py[n - 2];
        } else {
            float dx1 = px[i] - px[i - 1], dy1 = py[i] - py[i - 1];
            float dx2 = px[i + 1] - px[i], dy2 = py[i + 1] - py[i];
            float l1 = hypotf(dx1, dy1), l2 = hypotf(dx2, dy2);
            if (l1 > 0.01f) { dx1 /= l1; dy1 /= l1; }
            if (l2 > 0.01f) { dx2 /= l2; dy2 /= l2; }
            tx = dx1 + dx2;
            ty = dy1 + dy2;
        }
        float tlen = hypotf(tx, ty);
        if (tlen < 0.01f) { tx = 0.0f; ty = -1.0f; tlen = 1.0f; }
        float nx = (-ty / tlen) * hw;
        float ny = ( tx / tlen) * hw;
        pts[i * 2]     = px[i] + nx;
        pts[i * 2 + 1] = py[i] + ny;
        rx[i] = px[i] - nx;
        ry[i] = py[i] - ny;
    }
    int idx = n;
    for (int i = n - 1; i >= 0; i--) {
        pts[idx * 2]     = rx[i];
        pts[idx * 2 + 1] = ry[i];
        idx++;
    }
    add_poly(arr, cnt, w, h, color, 1, HUD_POLY_FILL, pts, (uint8_t)idx);
}
/* Draw an oriented arrowhead triangle with tip at (tx,ty), pointing along angle `deg` (0=up, +90=right, -90=left) */
static void add_arrowhead(HudPolygonCmd *arr, uint8_t *cnt,
                          uint16_t w, uint16_t h,
                          uint8_t color, float tx, float ty, float deg,
                          float head_len, float head_half_w) {
    float rad = deg * ((float)M_PI / 180.0f);
    float ux = sinf(rad);
    float uy = -cosf(rad);
    float px = -uy;
    float py = ux;
    float bx = tx - ux * head_len;
    float by = ty - uy * head_len;
    float pts[6] = {
        tx, ty,
        bx + px * head_half_w, by + py * head_half_w,
        bx - px * head_half_w, by - py * head_half_w
    };
    add_poly(arr, cnt, w, h, color, 2, HUD_POLY_FILL, pts, 3);
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

static void render_guiding_icon(const RgdManeuver *man, uint16_t w, uint16_t h,
                                HudPolygonCmd *polys, uint8_t *cnt) {
    *cnt = 0;
    float sx = (float)w / 120.0f;
    float sy = (float)h / 72.0f;
    float s = (sx < sy) ? sx : sy;
    float ox = ((float)w - 120.0f * s) * 0.5f;
    float oy = ((float)h - 72.0f * s) * 0.5f;

    #define X(v) (ox + (v) * s)
    #define Y(v) (oy + (v) * s)

    if (!man || man->type == 0 || man->type == 3 || man->type == 5 || man->type == 15 || man->type == 17) {
        /* Straight ahead / follow road: sleek OEM shaft + sharp wide chevron triangle */
        add_thick_seg(polys, cnt, w, h, HUD_COLOR_WHITE, X(60), Y(66), X(60), Y(22), 10.5f * s);
        add_arrowhead(polys, cnt, w, h, HUD_COLOR_WHITE, X(60), Y(5), 0.0f, 19.0f * s, 17.5f * s);
        return;
    }

    if (man->type == 10 || man->type == 12 || man->type == 24 || man->type == 25 || man->type == 27) {
        /* Arrived destination marker */
        float fx = (man->type == 24) ? 44.0f : (man->type == 25) ? 76.0f : 60.0f;
        if (man->type == 24 || man->type == 25) {
            add_bezier_ribbon(polys, cnt, w, h, HUD_COLOR_WHITE,
                              X(60), Y(66), X(60), Y(42), X(fx), Y(22), 9.5f * s, 10);
        } else {
            add_thick_seg(polys, cnt, w, h, HUD_COLOR_WHITE, X(60), Y(66), X(60), Y(22), 9.5f * s);
        }
        float flag[8] = {
            X(fx), Y(7),
            X(fx + 14), Y(21),
            X(fx), Y(35),
            X(fx - 14), Y(21)
        };
        add_poly(polys, cnt, w, h, HUD_COLOR_WHITE, 2, HUD_POLY_FILL, flag, 4);
        return;
    }

    if (man->type == 52 || man->type == 53) {
        /* Highway lane change left/right: smooth single cubic-like double Bezier S-curve */
        float dir = (man->type == 52) ? -1.0f : 1.0f;
        float x_src = 60.0f - dir * 18.0f;
        float x_dst = 60.0f + dir * 18.0f;
        add_cap_seg(polys, cnt, w, h, HUD_COLOR_DIM, X(x_src), Y(66), X(x_src), Y(13), 6.5f * s);

        add_thick_seg(polys, cnt, w, h, HUD_COLOR_WHITE, X(x_src), Y(66), X(x_src), Y(54), 9.5f * s);
        add_bezier_ribbon(polys, cnt, w, h, HUD_COLOR_WHITE,
                          X(x_src), Y(54), X(x_src), Y(38), X(60), Y(36), 9.5f * s, 4);
        add_bezier_ribbon(polys, cnt, w, h, HUD_COLOR_WHITE,
                          X(60), Y(36), X(x_dst), Y(34), X(x_dst), Y(18), 9.5f * s, 4);
        add_arrowhead(polys, cnt, w, h, HUD_COLOR_WHITE, X(x_dst), Y(5), 0.0f, 16.0f * s, 15.0f * s);
        return;
    }

    if (man->type == 4 || man->type == 18 || man->type == 26) {
        /* U-Turn: 1 single continuous polygon with high-density 26-step arch & roomy inner radius */
        float side = (man->driving_side == 1) ? 1.0f : -1.0f;
        float cx = 60.0f, cy = 28.0f, r = 18.5f, hw = 4.5f;
        float r_out = r + hw, r_in = r - hw;
        float x_in = cx - side * r, x_out = cx + side * r;
        float pts[HUD_MAX_POLY_PTS * 2];
        int n = 0;
        int steps = 26;
        float d0 = (side < 0) ?  90.0f : -90.0f;
        float d1 = -d0;

        /* Outer path: entry bottom -> up -> outer arch -> down exit */
        pts[n * 2] = X(x_in - side * hw); pts[n * 2 + 1] = Y(66.0f); n++;
        for (int i = 0; i <= steps; i++) {
            float deg = d0 + (d1 - d0) * ((float)i / (float)steps);
            float rad = deg * ((float)M_PI / 180.0f);
            pts[n * 2]     = X(cx + sinf(rad) * r_out);
            pts[n * 2 + 1] = Y(cy - cosf(rad) * r_out);
            n++;
        }
        pts[n * 2] = X(x_out + side * hw); pts[n * 2 + 1] = Y(48.0f); n++;
        /* Inner path back */
        pts[n * 2] = X(x_out - side * hw); pts[n * 2 + 1] = Y(48.0f); n++;
        for (int i = steps; i >= 0; i--) {
            float deg = d0 + (d1 - d0) * ((float)i / (float)steps);
            float rad = deg * ((float)M_PI / 180.0f);
            pts[n * 2]     = X(cx + sinf(rad) * r_in);
            pts[n * 2 + 1] = Y(cy - cosf(rad) * r_in);
            n++;
        }
        pts[n * 2] = X(x_in + side * hw); pts[n * 2 + 1] = Y(66.0f); n++;

        add_poly(polys, cnt, w, h, HUD_COLOR_WHITE, 1, HUD_POLY_FILL, pts, (uint8_t)n);
        add_arrowhead(polys, cnt, w, h, HUD_COLOR_WHITE, X(x_out), Y(65.0f), 180.0f, 18.0f * s, 16.0f * s);
        return;
    }

    if (is_roundabout_maneuver(man)) {
        /* 1 continuous seamless ribbon polygon for entry + ring arc + exit! Zero joints, zero breaks! */
        float cx = 60.0f, cy = 33.0f, r = 14.5f, ring_w = 6.5f;
        int exit_deg = resolve_turn_angle(man);
        if (man->type == 19) exit_deg = 180;

        /* 1. Dim base ring in 2 smooth half-ring polygons + rounded unchosen exit stubs */
        for (int i = 0; i < man->junction_angle_count; i++) {
            int d = man->junction_angles[i];
            if (abs(d - exit_deg) < 18 || abs(abs(d) - 180) < 12) continue;
            float rad = (float)d * ((float)M_PI / 180.0f);
            float x0 = cx + sinf(rad) * (r + 1.0f);
            float y0 = cy - cosf(rad) * (r + 1.0f);
            float x1 = cx + sinf(rad) * (r + 10.5f);
            float y1 = cy - cosf(rad) * (r + 10.5f);
            add_cap_seg(polys, cnt, w, h, HUD_COLOR_DIM, X(x0), Y(y0), X(x1), Y(y1), 6.0f * s);
        }
        add_arc_ribbon(polys, cnt, w, h, HUD_COLOR_DIM, X(cx), Y(cy), r * s, -90.0f,  90.0f, ring_w * s, 22);
        add_arc_ribbon(polys, cnt, w, h, HUD_COLOR_DIM, X(cx), Y(cy), r * s,  90.0f, 270.0f, ring_w * s, 22);

        /* 2. Build 1 single continuous centerline from bottom (60,66) -> entry curve -> ring arc -> exit curve -> arm */
        int ccw = (man->driving_side == 0);
        float dir_sign = ccw ? -1.0f : 1.0f; /* ccw: 180 -> 90 -> 0 -> -90 */
        int render_exit = exit_deg;
        if (abs(abs(render_exit) - 180) < 10) {
            render_exit = ccw ? -130 : 130;
        }
        int norm_exit = ((render_exit % 360) + 360) % 360;
        int sweep = ccw ? ((180 - norm_exit + 360) % 360) : ((norm_exit - 180 + 360) % 360);
        if (sweep == 0) sweep = 360;

        float fillet_deg = (sweep <= 100) ? 18.0f : 22.0f;
        if (fillet_deg > (float)sweep * 0.4f) fillet_deg = (float)sweep * 0.4f;
        float r_tan = r / cosf(fillet_deg * ((float)M_PI / 180.0f)); /* exact tangent intersection radius */
        float r_stem = r_tan + 4.5f;

        float cpx[32], cpy[32];
        int k = 0;
        /* Entry straight bottom */
        cpx[k] = X(cx); cpy[k] = Y(66.0f); k++;

        /* Entry G1 tangent Bezier: (cx, cy + r_stem) -> P1=(cx, cy + r_tan) -> ring(180 + dir_sign*fillet_deg) */
        float entry_ang = 180.0f + dir_sign * fillet_deg;
        float earad = entry_ang * ((float)M_PI / 180.0f);
        float e0x = cx, e0y = cy + r_stem;
        float e1x = cx, e1y = cy + r_tan;
        float e2x = cx + sinf(earad) * r, e2y = cy - cosf(earad) * r;
        for (int i = 0; i <= 4 && k < 31; i++) {
            float t = (float)i / 4.0f, mt = 1.0f - t;
            cpx[k] = X(mt * mt * e0x + 2.0f * mt * t * e1x + t * t * e2x);
            cpy[k] = Y(mt * mt * e0y + 2.0f * mt * t * e1y + t * t * e2y);
            k++;
        }

        /* Ring arc points strictly between entry_ang and arc_end */
        float arc_end = 180.0f + dir_sign * ((float)sweep - fillet_deg);
        int arc_n = (sweep >= 240) ? 16 : (sweep >= 150) ? 12 : 7;
        for (int i = 1; i < arc_n && k < 25; i++) {
            float deg = entry_ang + (arc_end - entry_ang) * ((float)i / (float)arc_n);
            float rad = deg * ((float)M_PI / 180.0f);
            cpx[k] = X(cx + sinf(rad) * r);
            cpy[k] = Y(cy - cosf(rad) * r);
            k++;
        }

        /* Exit G1 tangent Bezier: ring(arc_end) -> P1=(cx + ux*r_tan, cy + uy*r_tan) -> (cx + ux*r_stem, cy + uy*r_stem) */
        float exr = (float)render_exit * ((float)M_PI / 180.0f);
        float ux = sinf(exr), uy = -cosf(exr);
        float aend_rad = arc_end * ((float)M_PI / 180.0f);
        float x0x = cx + sinf(aend_rad) * r, x0y = cy - cosf(aend_rad) * r;
        float x1x = cx + ux * r_tan, x1y = cy + uy * r_tan;
        float x2x = cx + ux * r_stem, x2y = cy + uy * r_stem;
        for (int i = 0; i <= 4 && k < 31; i++) {
            float t = (float)i / 4.0f, mt = 1.0f - t;
            cpx[k] = X(mt * mt * x0x + 2.0f * mt * t * x1x + t * t * x2x);
            cpy[k] = Y(mt * mt * x0y + 2.0f * mt * t * x1y + t * t * x2y);
            k++;
        }

        /* Exit straight lead into arrowhead */
        if (k < 32) {
            cpx[k] = X(cx + ux * (r + 10.5f));
            cpy[k] = Y(cy + uy * (r + 10.5f));
            k++;
        }

        add_polyline_ribbon(polys, cnt, w, h, HUD_COLOR_WHITE, cpx, cpy, k, ring_w * s);
        add_arrowhead(polys, cnt, w, h, HUD_COLOR_WHITE,
                      X(cx + ux * (r + 18.0f)), Y(cy + uy * (r + 18.0f)),
                      (float)render_exit, 13.0f * s, 12.0f * s);
        return;
    }

    /* Standard intersection turn / off-ramp / Y-fork:
     * True Bezier-curved active path + cohesive grey cross-roads behind it! */
    int turn_deg = resolve_turn_angle(man);
    float jx = 60.0f, jy = 36.0f;

    /* 1. Background non-taken junction roads: clean cohesive grey intersection */
    int has_bg = 0;
    int has_forward = 0;
    for (int i = 0; i < man->junction_angle_count; i++) {
        int d = man->junction_angles[i];
        if (d == 0) has_forward = 1;
        if (abs(d - turn_deg) < 18 || abs(abs(d) - 180) < 12) continue;
        has_bg = 1;
        float rad = (float)d * ((float)M_PI / 180.0f);
        float bg_len = (d == 0) ? 23.0f : (abs(d) <= 45) ? 24.0f : 31.0f;
        add_cap_seg(polys, cnt, w, h, HUD_COLOR_DIM,
                    X(jx), Y(jy),
                    X(jx + sinf(rad) * bg_len), Y(jy - cosf(rad) * bg_len), 6.5f * s);
    }
    if ((man->type == 8 || man->type == 22 || man->type == 23) && !has_forward) {
        has_bg = 1;
        add_cap_seg(polys, cnt, w, h, HUD_COLOR_DIM, X(jx), Y(jy), X(jx), Y(12), 7.0f * s);
    }
    if (has_bg) {
        /* Draw grey incoming base road so all grey arms connect naturally */
        add_thick_seg(polys, cnt, w, h, HUD_COLOR_DIM, X(jx), Y(66), X(jx), Y(jy), 6.5f * s);
    }

    /* 2. Active white path with TRUE constant-radius circular fillet (smooth uniform arc!) */
    int abs_deg = abs(turn_deg);
    float trad = (float)turn_deg * ((float)M_PI / 180.0f);
    float ux = sinf(trad), uy = -cosf(trad);
    float bend_jy = (abs_deg >= 115) ? 24.0f : jy;

    if (abs_deg < 8) {
        add_thick_seg(polys, cnt, w, h, HUD_COLOR_WHITE, X(jx), Y(66), X(jx), Y(22), 10.5f * s);
        add_arrowhead(polys, cnt, w, h, HUD_COLOR_WHITE, X(jx), Y(5), 0.0f, 19.0f * s, 17.5f * s);
    } else {
        float side = (turn_deg > 0) ? 1.0f : -1.0f;
        float half_rad = ((float)abs_deg * 0.5f) * ((float)M_PI / 180.0f);
        float tan_h = tanf(half_rad);
        /* Choose tangent offset d inside [14..23] and derive exact constant fillet radius R */
        float desired_R = (abs_deg <= 60) ? 26.0f : (abs_deg <= 105) ? 20.0f : 15.0f;
        float d = desired_R * tan_h;
        if (d > 22.0f) { d = 22.0f; desired_R = d / tan_h; }
        if (d < 12.0f) { d = 12.0f; desired_R = d / tan_h; }

        float p0x = jx, p0y = bend_jy + d;
        float p2x = jx + ux * d, p2y = bend_jy + uy * d;
        float fc_x = jx + side * desired_R;
        float fc_y = p0y;
        float deg0 = (side > 0) ? -90.0f : 90.0f;
        float deg1 = deg0 + (float)turn_deg;
        int arc_steps = (abs_deg <= 55) ? 6 : (abs_deg <= 105) ? 9 : 11;

        if (p0y < 65.5f) {
            add_thick_seg(polys, cnt, w, h, HUD_COLOR_WHITE, X(jx), Y(66), X(p0x), Y(p0y), 10.5f * s);
        }
        add_arc_ribbon(polys, cnt, w, h, HUD_COLOR_WHITE,
                       X(fc_x), Y(fc_y), desired_R * s,
                       deg0, deg1, 10.5f * s, arc_steps);

        float lead = (abs_deg <= 60) ? 4.0f : (abs_deg <= 105) ? 5.0f : 4.0f;
        float tip_dist = d + 17.0f;
        add_thick_seg(polys, cnt, w, h, HUD_COLOR_WHITE,
                      X(p2x), Y(p2y),
                      X(jx + ux * (d + lead)), Y(bend_jy + uy * (d + lead)), 10.5f * s);
        add_arrowhead(polys, cnt, w, h, HUD_COLOR_WHITE,
                      X(jx + ux * tip_dist), Y(bend_jy + uy * tip_dist),
                      (float)turn_deg, 17.5f * s, 16.5f * s);
    }

    #undef X
    #undef Y
}

/* ==================== LaneGuidance (184 x 17) Renderer ==================== */

static void draw_lane_branch(HudPolygonCmd *polys, uint8_t *cnt,
                             uint16_t w, uint16_t h,
                             float cx, float cy, float h_scale,
                             int16_t angle, uint8_t color, float stem_w) {
    float rad = (float)angle * ((float)M_PI / 180.0f);
    float ux = sinf(rad), uy = -cosf(rad);
    float branch_len = (abs(angle) >= 60) ? 7.4f * h_scale : 6.8f * h_scale;
    float tip_len = branch_len + 3.6f * h_scale;
    float ax = cx + ux * branch_len;
    float ay = cy + uy * branch_len;
    float tx = cx + ux * tip_len;
    float ty = cy + uy * tip_len;

    if (abs(angle) >= 30) {
        /* Smooth 2-step curved elbow for turning lane branches */
        add_bezier_ribbon(polys, cnt, w, h, color,
                          cx, cy + 2.2f * h_scale,
                          cx, cy,
                          ax, ay,
                          stem_w, 2);
    } else {
        add_thick_seg(polys, cnt, w, h, color, cx, cy, ax, ay, stem_w);
    }
    add_arrowhead(polys, cnt, w, h, color, tx, ty, (float)angle, 4.8f * h_scale, 3.8f * h_scale);
}

static void render_lane_guidance(const RgdLaneEvent *le, uint16_t w, uint16_t h,
                                 HudPolygonCmd *polys, uint8_t *cnt) {
    *cnt = 0;
    if (!le || le->lane_count == 0) return;

    int n = le->lane_count;
    if (n > HUD_MAX_LANES) n = HUD_MAX_LANES;
    float h_scale = (float)h / 17.0f;
    float slot_w = (float)w / (float)(n < 4 ? 4 : n);
    if (slot_w > 28.0f) slot_w = 28.0f;
    if (slot_w < 18.0f) slot_w = (float)w / (float)n;

    float total_w = slot_w * (float)n;
    float start_x = ((float)w - total_w) * 0.5f;

    float y_bot = (float)h - 1.5f * h_scale;
    float y_split = (float)h * 0.60f;

    for (int i = 0; i < n; i++) {
        const RgdLaneInfo *ln = &le->lanes[i];
        float cx = start_x + ((float)i + 0.5f) * slot_w;
        int is_good = (ln->status >= 1);

        int16_t hi = 1000;
        if (ln->has_highlighted) hi = ln->highlighted_angle;
        else if (ln->angle_count > 0) hi = ln->angles[0];

        uint8_t base_color = is_good ? HUD_COLOR_WHITE : HUD_COLOR_DIM;
        float stem_w = is_good ? 2.6f * h_scale : 2.0f * h_scale;

        /* Base vertical stem at bottom of lane */
        add_thick_seg(polys, cnt, w, h, base_color, cx, y_bot, cx, y_split, stem_w);

        /* Draw all non-highlighted angles first in dimmed color */
        int drew_any = 0;
        int drew_hi = 0;
        for (int k = 0; k < (int)ln->angle_count; k++) {
            int16_t a = ln->angles[k];
            if (is_good && valid_angle(hi) && abs((int)a - (int)hi) <= 5) {
                drew_hi = 1;
                continue; /* draw highlighted on top */
            }
            draw_lane_branch(polys, cnt, w, h, cx, y_split, h_scale, a, HUD_COLOR_DIM, 2.0f * h_scale);
            drew_any = 1;
        }
        if (is_good && valid_angle(hi)) {
            draw_lane_branch(polys, cnt, w, h, cx, y_split, h_scale, hi, HUD_COLOR_WHITE, 2.6f * h_scale);
            drew_any = 1;
            drew_hi = 1;
        }
        if (!drew_any && !drew_hi) {
            /* Fallback straight arrow if phone sent lane slot without angles */
            draw_lane_branch(polys, cnt, w, h, cx, y_split, h_scale, 0, base_color, stem_w);
        }

        /* Lane separator tick between lanes */
        if (i + 1 < n) {
            float sx = start_x + (float)(i + 1) * slot_w;
            add_thick_seg(polys, cnt, w, h, HUD_COLOR_DIM, sx, y_bot, sx, y_bot - 3.5f * h_scale, 1.2f);
        }
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
    if (!man && st->man_count > 0) man = find_maneuver(st, 0);

    if (man && man->after_road[0] != '\0') {
        snprintf(out->street_text, sizeof(out->street_text), "%s", man->after_road);
    } else if (st->current_road[0] != '\0') {
        snprintf(out->street_text, sizeof(out->street_text), "%s", st->current_road);
    } else if (man && man->desc[0] != '\0') {
        snprintf(out->street_text, sizeof(out->street_text), "%s", man->desc);
    } else if (st->dest_name[0] != '\0') {
        snprintf(out->street_text, sizeof(out->street_text), "%s", st->dest_name);
    }

    render_guiding_icon(man, icon_w, icon_h, out->icon_polys, &out->icon_poly_count);

    const RgdLaneEvent *le = NULL;
    if (st->lane_guidance_showing == 1 && st->has_lane_idx) {
        le = find_lane_event(st, st->lane_guidance_idx);
    } else if (!st->has_lane_showing) {
        if (st->has_lane_idx) {
            le = find_lane_event(st, st->lane_guidance_idx);
        } else if (man && man->has_linked_lane) {
            le = find_lane_event(st, man->linked_lane_idx);
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

