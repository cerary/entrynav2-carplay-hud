#include "hud_render.h"
#include "iap2_rgd.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static size_t build_msg(uint8_t *out, uint16_t msg_id, const uint8_t *tlvs, size_t tlvs_len) {
    out[0] = 0x40;
    out[1] = 0x40;
    iap2_put_be16(out + 2, (uint16_t)(6 + tlvs_len));
    iap2_put_be16(out + 4, msg_id);
    if (tlvs_len > 0) memcpy(out + 6, tlvs, tlvs_len);
    return 6 + tlvs_len;
}

static size_t add_tlv(uint8_t *p, uint16_t id, const uint8_t *val, uint16_t len) {
    iap2_put_be16(p, (uint16_t)(4 + len));
    iap2_put_be16(p + 2, id);
    if (len > 0) memcpy(p + 4, val, len);
    return 4 + len;
}

static size_t add_u8(uint8_t *p, uint16_t id, uint8_t v) {
    return add_tlv(p, id, &v, 1);
}

static size_t add_u16(uint8_t *p, uint16_t id, uint16_t v) {
    uint8_t b[2];
    iap2_put_be16(b, v);
    return add_tlv(p, id, b, 2);
}

static size_t add_u32(uint8_t *p, uint16_t id, uint32_t v) {
    uint8_t b[4];
    b[0] = (uint8_t)(v >> 24);
    b[1] = (uint8_t)(v >> 16);
    b[2] = (uint8_t)(v >> 8);
    b[3] = (uint8_t)(v & 0xFF);
    return add_tlv(p, id, b, 4);
}

static size_t add_str(uint8_t *p, uint16_t id, const char *s) {
    return add_tlv(p, id, (const uint8_t *)s, (uint16_t)(strlen(s) + 1));
}

static void assert_polys_in_bounds(const HudPolygonCmd *polys, uint8_t n, uint16_t w, uint16_t h) {
    for (uint8_t i = 0; i < n; i++) {
        assert(polys[i].num_pts >= 3 && polys[i].num_pts <= HUD_MAX_POLY_PTS);
        assert(polys[i].poly_mode == HUD_POLY_FILL);
        assert(polys[i].color == HUD_COLOR_WHITE || polys[i].color == HUD_COLOR_DIM);
        for (uint8_t k = 0; k < polys[i].num_pts; k++) {
            assert(polys[i].xy[k * 2] >= 1 && polys[i].xy[k * 2] < w);
            assert(polys[i].xy[k * 2 + 1] >= 1 && polys[i].xy[k * 2 + 1] < h);
            uint8_t nxt = (uint8_t)((k + 1) % polys[i].num_pts);
            assert(!(polys[i].xy[k * 2] == polys[i].xy[nxt * 2] &&
                     polys[i].xy[k * 2 + 1] == polys[i].xy[nxt * 2 + 1]));
        }
    }
}

int main(void) {
    RgdState st;
    rgd_state_init(&st);

    /* 1. Feed 0x5202 maneuver (left turn onto 中关村大街 with straight + left junction arms) */
    uint8_t tlvs[512];
    size_t t = 0;
    t += add_u16(tlvs + t, IAP2_RGM_INDEX, 1);
    t += add_u8(tlvs + t, IAP2_RGM_MANEUVER_TYPE, 1); /* LEFT_TURN */
    t += add_str(tlvs + t, IAP2_RGM_AFTER_MANEUVER_ROAD_NAME, "中关村大街");
    t += add_u8(tlvs + t, IAP2_RGM_JUNCTION_TYPE, 0);
    t += add_u16(tlvs + t, IAP2_RGM_JUNCTION_ELEMENT_ANGLE, 0);
    t += add_u16(tlvs + t, IAP2_RGM_JUNCTION_ELEMENT_ANGLE, (uint16_t)(-90));
    t += add_u16(tlvs + t, IAP2_RGM_JUNCTION_ELEMENT_EXIT_ANGLE, (uint16_t)(-90));

    uint8_t pkt[600];
    size_t n = build_msg(pkt, IAP2_MSG_ROUTE_GUIDANCE_MANEUVER, tlvs, t);
    assert(rgd_state_ingest(&st, pkt, n, 1000) == 1);

    /* 2. Feed 0x5204 lane guidance (3 lanes: pos=0 left preferred, pos=1 straight+left good, pos=2 straight not good) */
    uint8_t lg_tlvs[512];
    size_t lt = 0;
    lt += add_u16(lg_tlvs + lt, IAP2_LGI_INDEX, 5);

    uint8_t g0[64]; size_t g0l = 0;
    g0l += add_u16(g0 + g0l, IAP2_LANE_POSITION, 0);
    g0l += add_u8(g0 + g0l, IAP2_LANE_STATUS, 2);
    g0l += add_u16(g0 + g0l, IAP2_LANE_ANGLE, (uint16_t)(-90));
    g0l += add_u16(g0 + g0l, IAP2_LANE_HIGHLIGHTED_ANGLE, (uint16_t)(-90));
    lt += add_tlv(lg_tlvs + lt, IAP2_LGI_LANE_INFO_GROUP, g0, (uint16_t)g0l);

    uint8_t g1[64]; size_t g1l = 0;
    g1l += add_u16(g1 + g1l, IAP2_LANE_POSITION, 1);
    g1l += add_u8(g1 + g1l, IAP2_LANE_STATUS, 1);
    g1l += add_u16(g1 + g1l, IAP2_LANE_ANGLE, (uint16_t)(-90));
    g1l += add_u16(g1 + g1l, IAP2_LANE_ANGLE, 0);
    g1l += add_u16(g1 + g1l, IAP2_LANE_HIGHLIGHTED_ANGLE, (uint16_t)(-90));
    lt += add_tlv(lg_tlvs + lt, IAP2_LGI_LANE_INFO_GROUP, g1, (uint16_t)g1l);

    uint8_t g2[64]; size_t g2l = 0;
    g2l += add_u16(g2 + g2l, IAP2_LANE_POSITION, 2);
    g2l += add_u8(g2 + g2l, IAP2_LANE_STATUS, 0);
    g2l += add_u16(g2 + g2l, IAP2_LANE_ANGLE, 0);
    lt += add_tlv(lg_tlvs + lt, IAP2_LGI_LANE_INFO_GROUP, g2, (uint16_t)g2l);

    n = build_msg(pkt, IAP2_MSG_LANE_GUIDANCE_INFO, lg_tlvs, lt);
    assert(rgd_state_ingest(&st, pkt, n, 1010) == 1);

    /* 3. Feed 0x5201 RouteGuidanceUpdate activating maneuver #1 and lane #5 */
    t = 0;
    t += add_u8(tlvs + t, IAP2_RGU_STATE, 1);
    t += add_u32(tlvs + t, IAP2_RGU_DIST_TO_NEXT_MANEUVER, 1450);
    t += add_u8(tlvs + t, IAP2_RGU_DIST_TO_NEXT_MANEUVER_UNITS, 2);
    t += add_u16(tlvs + t, IAP2_RGU_MANEUVER_CURRENT_LIST, 1);
    t += add_u16(tlvs + t, IAP2_RGU_MANEUVER_COUNT, 4);
    t += add_u16(tlvs + t, IAP2_RGU_LANE_GUIDANCE_INDEX, 5);
    t += add_u8(tlvs + t, IAP2_RGU_LANE_GUIDANCE_SHOWING, 1);
    n = build_msg(pkt, IAP2_MSG_ROUTE_GUIDANCE_UPDATE, tlvs, t);
    assert(rgd_state_ingest(&st, pkt, n, 1020) == 1);

    HudRenderFrame frame;
    hud_render_build_frame(&st, 1020, 120, 72, 184, 17, &frame);
    assert(frame.visible == 1);
    assert(frame.lane_visible == 1);
    assert(strcmp(frame.dist_text, "1.5") == 0);
    assert(strcmp(frame.unit_text, "km") == 0);
    assert(strcmp(frame.street_text, "中关村大街") == 0);
    assert(frame.icon_poly_count >= 4);
    assert(frame.lane_poly_count >= 8);
    assert_polys_in_bounds(frame.icon_polys, frame.icon_poly_count, 120, 72);
    assert_polys_in_bounds(frame.lane_polys, frame.lane_poly_count, 184, 17);

    /* Verify both white (recommended) and dim (non-recommended) lane polygons exist */
    int has_white = 0, has_dim = 0;
    for (uint8_t i = 0; i < frame.lane_poly_count; i++) {
        if (frame.lane_polys[i].color == HUD_COLOR_WHITE) has_white = 1;
        if (frame.lane_polys[i].color == HUD_COLOR_DIM) has_dim = 1;
    }
    assert(has_white && has_dim);

    /* 4. Reroute debounce: transient state=0 within 2500ms keeps visible */
    t = 0;
    t += add_u8(tlvs + t, IAP2_RGU_STATE, 0);
    n = build_msg(pkt, IAP2_MSG_ROUTE_GUIDANCE_UPDATE, tlvs, t);
    assert(rgd_state_ingest(&st, pkt, n, 1200) == 1);
    assert(rgd_state_should_show(&st, 1200) == 1);
    assert(rgd_state_should_show(&st, 2500) == 1);
    assert(rgd_state_should_show(&st, 3600) == 0);

    /* 5. Re-activate then test 0x5203 StopRouteGuidance clears immediately */
    t = 0;
    t += add_u8(tlvs + t, IAP2_RGU_STATE, 6);
    n = build_msg(pkt, IAP2_MSG_ROUTE_GUIDANCE_UPDATE, tlvs, t);
    rgd_state_ingest(&st, pkt, n, 2000);
    assert(rgd_state_should_show(&st, 2000) == 1);

    n = build_msg(pkt, IAP2_MSG_STOP_ROUTE_GUIDANCE, NULL, 0);
    rgd_state_ingest(&st, pkt, n, 2050);
    assert(rgd_state_should_show(&st, 2050) == 0);

    /* 6. Test roundabout exit + test_frame builder */
    t = 0;
    t += add_u16(tlvs + t, IAP2_RGM_INDEX, 2);
    t += add_u8(tlvs + t, IAP2_RGM_MANEUVER_TYPE, 29); /* ROUNDABOUT_EXIT_2 */
    t += add_u8(tlvs + t, IAP2_RGM_JUNCTION_TYPE, 1);
    t += add_u16(tlvs + t, IAP2_RGM_JUNCTION_ELEMENT_ANGLE, 90);
    t += add_u16(tlvs + t, IAP2_RGM_JUNCTION_ELEMENT_ANGLE, 0);
    t += add_u16(tlvs + t, IAP2_RGM_JUNCTION_ELEMENT_EXIT_ANGLE, 0);
    n = build_msg(pkt, IAP2_MSG_ROUTE_GUIDANCE_MANEUVER, tlvs, t);
    rgd_state_ingest(&st, pkt, n, 2100);

    t = 0;
    t += add_u8(tlvs + t, IAP2_RGU_STATE, 1);
    t += add_u32(tlvs + t, IAP2_RGU_DIST_TO_NEXT_MANEUVER, 275);
    t += add_u8(tlvs + t, IAP2_RGU_DIST_TO_NEXT_MANEUVER_UNITS, 2);
    t += add_u16(tlvs + t, IAP2_RGU_MANEUVER_CURRENT_LIST, 2);
    t += add_u16(tlvs + t, IAP2_RGU_MANEUVER_COUNT, 3);
    n = build_msg(pkt, IAP2_MSG_ROUTE_GUIDANCE_UPDATE, tlvs, t);
    rgd_state_ingest(&st, pkt, n, 2110);

    hud_render_build_frame(&st, 2110, 120, 72, 184, 17, &frame);
    assert(frame.visible == 1);
    assert(strcmp(frame.dist_text, "300") == 0);
    assert(strcmp(frame.unit_text, "m") == 0);
    assert(frame.icon_poly_count >= 5);
    assert_polys_in_bounds(frame.icon_polys, frame.icon_poly_count, 120, 72);

    /* 6b. Test U-turn, Sharp Right (135 deg), and early small-angle roundabout exit */
    t = 0;
    t += add_u16(tlvs + t, IAP2_RGM_INDEX, 3);
    t += add_u8(tlvs + t, IAP2_RGM_MANEUVER_TYPE, 4); /* U_TURN */
    n = build_msg(pkt, IAP2_MSG_ROUTE_GUIDANCE_MANEUVER, tlvs, t);
    rgd_state_ingest(&st, pkt, n, 2120);
    t = 0; t += add_u16(tlvs + t, IAP2_RGU_MANEUVER_CURRENT_LIST, 3);
    n = build_msg(pkt, IAP2_MSG_ROUTE_GUIDANCE_UPDATE, tlvs, t);
    rgd_state_ingest(&st, pkt, n, 2125);
    hud_render_build_frame(&st, 2125, 120, 72, 184, 17, &frame);
    assert(frame.icon_poly_count == 2);
    assert_polys_in_bounds(frame.icon_polys, frame.icon_poly_count, 120, 72);

    t = 0;
    t += add_u16(tlvs + t, IAP2_RGM_INDEX, 4);
    t += add_u8(tlvs + t, IAP2_RGM_MANEUVER_TYPE, 48); /* SHARP_RIGHT_TURN */
    t += add_u16(tlvs + t, IAP2_RGM_JUNCTION_ELEMENT_EXIT_ANGLE, 135);
    n = build_msg(pkt, IAP2_MSG_ROUTE_GUIDANCE_MANEUVER, tlvs, t);
    rgd_state_ingest(&st, pkt, n, 2130);
    t = 0; t += add_u16(tlvs + t, IAP2_RGU_MANEUVER_CURRENT_LIST, 4);
    n = build_msg(pkt, IAP2_MSG_ROUTE_GUIDANCE_UPDATE, tlvs, t);
    rgd_state_ingest(&st, pkt, n, 2135);
    hud_render_build_frame(&st, 2135, 120, 72, 184, 17, &frame);
    assert(frame.icon_poly_count >= 4);
    assert_polys_in_bounds(frame.icon_polys, frame.icon_poly_count, 120, 72);

    t = 0;
    t += add_u16(tlvs + t, IAP2_RGM_INDEX, 5);
    t += add_u8(tlvs + t, IAP2_RGM_MANEUVER_TYPE, 28); /* ROUNDABOUT_EXIT_1 */
    t += add_u8(tlvs + t, IAP2_RGM_JUNCTION_TYPE, 1);
    t += add_u16(tlvs + t, IAP2_RGM_JUNCTION_ELEMENT_EXIT_ANGLE, 155); /* sweep = 25 deg */
    n = build_msg(pkt, IAP2_MSG_ROUTE_GUIDANCE_MANEUVER, tlvs, t);
    rgd_state_ingest(&st, pkt, n, 2140);
    t = 0; t += add_u16(tlvs + t, IAP2_RGU_MANEUVER_CURRENT_LIST, 5);
    n = build_msg(pkt, IAP2_MSG_ROUTE_GUIDANCE_UPDATE, tlvs, t);
    rgd_state_ingest(&st, pkt, n, 2145);
    hud_render_build_frame(&st, 2145, 120, 72, 184, 17, &frame);
    assert(frame.icon_poly_count >= 4);
    assert_polys_in_bounds(frame.icon_polys, frame.icon_poly_count, 120, 72);

    hud_render_build_test_frame(120, 72, 184, 17, "长安街", &frame);
    assert(frame.visible == 1 && frame.lane_visible == 1);
    assert_polys_in_bounds(frame.icon_polys, frame.icon_poly_count, 120, 72);
    assert_polys_in_bounds(frame.lane_polys, frame.lane_poly_count, 184, 17);

    /* 7. Verify MOST string encoding (ASCII -> 0x01, Chinese -> 0x00 + UTF-16BE + 0x0000) */
    uint8_t enc[64];
    size_t elen = hud_encode_most_string("150", 0, enc, sizeof(enc));
    assert(elen == 5 && enc[0] == 0x01 && memcmp(enc + 1, "150\0", 4) == 0);

    elen = hud_encode_most_string("长安街", 0, enc, sizeof(enc));
    /* U+957F U+5B89 U+8857 */
    static const uint8_t expected_cn[] = { 0x00, 0x95, 0x7F, 0x5B, 0x89, 0x88, 0x57, 0x00, 0x00 };
    assert(elen == sizeof(expected_cn) && memcmp(enc, expected_cn, sizeof(expected_cn)) == 0);

    /* 8. Verify route_state=4 with maneuvers stays active, 0xFFFFFFFF dist is ignored,
     *    and lane guidance renders when lane_guidance_showing (0x0012) was omitted */
    RgdState st2;
    rgd_state_init(&st2);
    t = 0;
    t += add_u16(tlvs + t, IAP2_RGM_INDEX, 0);
    t += add_u8(tlvs + t, IAP2_RGM_MANEUVER_TYPE, 3);
    n = build_msg(pkt, IAP2_MSG_ROUTE_GUIDANCE_MANEUVER, tlvs, t);
    assert(rgd_state_ingest(&st2, pkt, n, 3000) == 1);

    uint8_t lane0[32]; size_t l0 = 0;
    l0 += add_u16(lane0 + l0, IAP2_LANE_POSITION, 0);
    l0 += add_u8(lane0 + l0, IAP2_LANE_STATUS, 2);
    l0 += add_u16(lane0 + l0, IAP2_LANE_ANGLE, 0);
    t = 0;
    t += add_u16(tlvs + t, IAP2_LGI_INDEX, 7);
    t += add_tlv(tlvs + t, IAP2_LGI_LANE_INFO_GROUP, lane0, l0);
    n = build_msg(pkt, IAP2_MSG_LANE_GUIDANCE_INFO, tlvs, t);
    assert(rgd_state_ingest(&st2, pkt, n, 3005) == 1);

    t = 0;
    t += add_u8(tlvs + t, IAP2_RGU_STATE, 4);
    t += add_u16(tlvs + t, IAP2_RGU_MANEUVER_COUNT, 1);
    t += add_u16(tlvs + t, IAP2_RGU_MANEUVER_CURRENT_LIST, 0);
    t += add_u32(tlvs + t, IAP2_RGU_DIST_TO_NEXT_MANEUVER, 0xFFFFFFFFu);
    t += add_u16(tlvs + t, IAP2_RGU_LANE_GUIDANCE_INDEX, 7);
    n = build_msg(pkt, IAP2_MSG_ROUTE_GUIDANCE_UPDATE, tlvs, t);
    assert(rgd_state_ingest(&st2, pkt, n, 3010) == 1);
    assert(rgd_state_should_show(&st2, 3010) == 1);
    hud_render_build_frame(&st2, 3010, 120, 72, 184, 17, &frame);
    assert(frame.visible == 1);
    assert(frame.dist_text[0] == '\0');
    assert(frame.lane_visible == 1);

    printf("ALL test_hud_render ASSERTIONS PASSED!\n");
    return 0;
}
