#ifndef HUD_RENDER_H
#define HUD_RENDER_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HUD_MAX_MANEUVERS       32
#define HUD_MAX_LANE_EVENTS     32
#define HUD_MAX_LANES           8
#define HUD_MAX_LANE_ANGLES     16
#define HUD_MAX_JUNC_ANGLES     18
#define HUD_MAX_POLYS           64
#define HUD_MAX_POLY_PTS        64
#define HUD_STR_MAX             128

/* OEM HUDTerminal palette indices verified in libVMostAdapter.so */
#define HUD_COLOR_DIM           3
#define HUD_COLOR_WHITE         4

/* HUD_FUNC_IMAGE_ADD_POLYGON modes verified in libVMostAdapter.so */
#define HUD_POLY_LINE           2
#define HUD_POLY_FILL           3

typedef struct {
    uint8_t  valid;
    uint16_t index;
    uint8_t  type;                              /* Apple ManeuverType 0..53 */
    uint8_t  driving_side;                      /* 0=RHT, 1=LHT */
    uint8_t  junction_type;                     /* 0=intersection, 1=roundabout */
    uint32_t dist_between_m;
    int16_t  junction_angles[HUD_MAX_JUNC_ANGLES];
    uint8_t  junction_angle_count;
    int16_t  exit_angle;                        /* 1000 = unset */
    uint8_t  has_exit_angle;
    uint16_t linked_lane_idx;
    uint8_t  has_linked_lane;
    char     after_road[HUD_STR_MAX];
    char     desc[HUD_STR_MAX];
    char     exit_info[HUD_STR_MAX];
} RgdManeuver;

typedef struct {
    uint16_t position;
    uint8_t  status;                            /* 0=not good, 1=good, 2=preferred */
    int16_t  angles[HUD_MAX_LANE_ANGLES];
    uint8_t  angle_count;
    int16_t  highlighted_angle;
    uint8_t  has_highlighted;
} RgdLaneInfo;

typedef struct {
    uint8_t     valid;
    uint16_t    index;
    RgdLaneInfo lanes[HUD_MAX_LANES];
    uint8_t     lane_count;
    char        desc[HUD_STR_MAX];
} RgdLaneEvent;

typedef struct {
    uint8_t     route_state;                    /* 0=inactive, 1=active, 2=arrived, 3=loading, 5=rerouting, 6=proceeding */
    uint8_t     maneuver_state;                 /* 0=continue, 1=initial, 2=prepare, 3=execute */
    uint8_t     source_supports_rg;             /* default 1 unless explicitly 0 */
    uint8_t     has_source_supports_rg;
    uint8_t     being_shown_in_app;
    uint32_t    dist_rem_m;
    uint32_t    dist_maneuver_m;
    uint8_t     has_dist_maneuver_m;
    uint8_t     dist_maneuver_unit;             /* 0=km, 1=mi, 2=m, 3=yd, 4=ft, 0xFF=unset */
    char        dist_maneuver_str[32];
    char        current_road[HUD_STR_MAX];
    char        dest_name[HUD_STR_MAX];
    char        source_name[64];
    uint16_t    man_current_idx;
    uint8_t     has_man_current;
    uint16_t    man_count;
    uint16_t    lane_guidance_idx;
    uint8_t     has_lane_idx;
    uint16_t    lane_guidance_total;
    uint8_t     lane_guidance_showing;
    uint8_t     has_lane_showing;

    RgdManeuver  maneuvers[HUD_MAX_MANEUVERS];
    RgdLaneEvent lane_events[HUD_MAX_LANE_EVENTS];

    /* Reroute / flicker debounce */
    uint8_t     ever_active;
    uint8_t     stopped_explicit;
    uint64_t    last_active_ms;
} RgdState;

typedef struct {
    uint8_t  color;         /* HUD_COLOR_DIM (3) or HUD_COLOR_WHITE (4) */
    uint8_t  line_width;    /* 1 or 2 */
    uint8_t  fill_mode;     /* 3 = solid fill */
    uint8_t  fill_color;    /* HUD_COLOR_DIM (3) or HUD_COLOR_WHITE (4) */
    uint8_t  poly_mode;     /* HUD_POLY_FILL (3) or HUD_POLY_LINE (2) */
    uint8_t  num_pts;
    uint16_t xy[HUD_MAX_POLY_PTS * 2];
} HudPolygonCmd;

typedef struct {
    uint8_t       visible;
    uint8_t       lane_visible;
    char          dist_text[32];
    char          unit_text[16];
    char          street_text[HUD_STR_MAX];
    HudPolygonCmd icon_polys[HUD_MAX_POLYS];
    uint8_t       icon_poly_count;
    HudPolygonCmd lane_polys[HUD_MAX_POLYS];
    uint8_t       lane_poly_count;
    uint32_t      content_hash;
} HudRenderFrame;

void rgd_state_init(RgdState *st);
void rgd_state_reset_route(RgdState *st);

/* Ingest raw iAP2 session message (starts with 0x40 0x40 + len16 + msgid16 + TLVs).
 * Returns 1 if state changed, 0 otherwise. */
int rgd_state_ingest(RgdState *st, const uint8_t *buf, size_t len, uint64_t now_ms);

/* Returns 1 if HUD should currently display navigation (honoring 800ms reroute debounce). */
int rgd_state_should_show(const RgdState *st, uint64_t now_ms);

/* Build complete OEM HUD frame for given canvas dimensions (default icon 120x72, lane 184x17). */
void hud_render_build_frame(const RgdState *st, uint64_t now_ms,
                            uint16_t icon_w, uint16_t icon_h,
                            uint16_t lane_w, uint16_t lane_h,
                            HudRenderFrame *out);

/* Build a standalone test frame showing turn arrow + 4-lane guidance + street name. */
void hud_render_build_test_frame(uint16_t icon_w, uint16_t icon_h,
                                 uint16_t lane_w, uint16_t lane_h,
                                 const char *street_text,
                                 HudRenderFrame *out);

/* Encode UTF-8 string into MOST String wire format:
 *   - Pure ASCII (or force_ascii): [0x01][8-bit bytes][0x00]
 *   - Non-ASCII (Chinese/Unicode): [0x00][UTF-16BE code units][0x00][0x00]
 * Returns total encoded byte length written to `out`. */
size_t hud_encode_most_string(const char *utf8, int force_ascii, uint8_t *out, size_t out_cap);

#ifdef __cplusplus
}
#endif

#endif /* HUD_RENDER_H */
