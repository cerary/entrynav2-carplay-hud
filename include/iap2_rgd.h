#ifndef IAP2_RGD_H
#define IAP2_RGD_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* iAP2 Message IDs */
#define IAP2_MSG_IDENTIFICATION_INFO          0x1D01
#define IAP2_MSG_IDENTIFICATION_ACCEPTED      0x1D02
#define IAP2_MSG_IDENTIFICATION_REJECTED      0x1D03
#define IAP2_MSG_IDENTIFICATION_INFO_UPDATE   0x1D06

#define IAP2_MSG_START_ROUTE_GUIDANCE         0x5200
#define IAP2_MSG_ROUTE_GUIDANCE_UPDATE        0x5201
#define IAP2_MSG_ROUTE_GUIDANCE_MANEUVER      0x5202
#define IAP2_MSG_STOP_ROUTE_GUIDANCE          0x5203
#define IAP2_MSG_LANE_GUIDANCE_INFO           0x5204

/* IdentificationInformation (0x1D01) Param IDs (verified vs libNmeVfs.so & live capture) */
#define IAP2_ID_MSGS_SENT_BY_ACC              0x0006
#define IAP2_ID_MSGS_RECV_FROM_DEV            0x0007
#define IAP2_ID_RGD_COMPONENT                 0x001E

/* RouteGuidanceDisplayComponent (0x001E) Sub-Param IDs */
#define IAP2_RGD_COMP_ID                      0x0000
#define IAP2_RGD_COMP_NAME                    0x0001
#define IAP2_RGD_COMP_MAX_CUR_ROAD_LEN        0x0002
#define IAP2_RGD_COMP_MAX_DEST_LEN            0x0003
#define IAP2_RGD_COMP_MAX_AFTER_ROAD_LEN      0x0004
#define IAP2_RGD_COMP_MAX_MANEUVER_DESC_LEN   0x0005
#define IAP2_RGD_COMP_MAX_MANEUVER_CAP        0x0006
#define IAP2_RGD_COMP_MAX_LANE_DESC_LEN       0x0007
#define IAP2_RGD_COMP_MAX_LANE_CAP            0x0008

/* RouteGuidanceUpdate (0x5201) Param IDs */
#define IAP2_RGU_COMPONENT_ID                 0x0000
#define IAP2_RGU_STATE                        0x0001
#define IAP2_RGU_MANEUVER_STATE               0x0002
#define IAP2_RGU_CURRENT_ROAD_NAME            0x0003
#define IAP2_RGU_DESTINATION_NAME             0x0004
#define IAP2_RGU_EST_TIME_TO_DEST             0x0005
#define IAP2_RGU_EST_TIME_ZONE_OFFSET         0x0006
#define IAP2_RGU_DIST_REMAINING               0x0007
#define IAP2_RGU_DIST_REMAINING_STR           0x0008
#define IAP2_RGU_DIST_REMAINING_UNITS         0x0009
#define IAP2_RGU_DIST_TO_NEXT_MANEUVER        0x000A
#define IAP2_RGU_DIST_TO_NEXT_MANEUVER_STR    0x000B
#define IAP2_RGU_DIST_TO_NEXT_MANEUVER_UNITS  0x000C
#define IAP2_RGU_MANEUVER_CURRENT_LIST        0x000D
#define IAP2_RGU_MANEUVER_COUNT               0x000E
#define IAP2_RGU_BEING_SHOWN_IN_APP           0x000F
#define IAP2_RGU_LANE_GUIDANCE_INDEX          0x0010
#define IAP2_RGU_LANE_GUIDANCE_TOTAL          0x0011
#define IAP2_RGU_LANE_GUIDANCE_SHOWING        0x0012
#define IAP2_RGU_SOURCE_NAME                  0x0013
#define IAP2_RGU_SOURCE_SUPPORTS_RGD          0x0014

/* RouteGuidanceManeuverUpdate (0x5202) Param IDs */
#define IAP2_RGM_COMPONENT_ID                 0x0000
#define IAP2_RGM_INDEX                        0x0001
#define IAP2_RGM_MANEUVER_DESCRIPTION         0x0002
#define IAP2_RGM_MANEUVER_TYPE                0x0003
#define IAP2_RGM_AFTER_MANEUVER_ROAD_NAME     0x0004
#define IAP2_RGM_DIST_BETWEEN_MANEUVERS       0x0005
#define IAP2_RGM_DIST_BETWEEN_MANEUVERS_STR   0x0006
#define IAP2_RGM_DIST_BETWEEN_MANEUVERS_UNITS 0x0007
#define IAP2_RGM_DRIVING_SIDE                 0x0008
#define IAP2_RGM_JUNCTION_TYPE                0x0009
#define IAP2_RGM_JUNCTION_ELEMENT_ANGLE       0x000A
#define IAP2_RGM_JUNCTION_ELEMENT_EXIT_ANGLE  0x000B
#define IAP2_RGM_LINKED_LANE_GUIDANCE_INFO    0x000C
#define IAP2_RGM_EXIT_INFO                    0x000D

/* LaneGuidanceInformation (0x5204) Param IDs */
#define IAP2_LGI_COMPONENT_ID                 0x0000
#define IAP2_LGI_INDEX                        0x0001
#define IAP2_LGI_LANE_INFO_GROUP              0x0002
#define IAP2_LGI_INSTRUCTION_TEXT             0x0003

/* LaneInfo sub-group (0x0002 inside 0x5204) */
#define IAP2_LANE_POSITION                    0x0000  /* u16 */
#define IAP2_LANE_STATUS                      0x0001  /* u8: 0=not good, 1=good, 2=preferred */
#define IAP2_LANE_ANGLE                       0x0002  /* i16 (repeatable, degrees) */
#define IAP2_LANE_HIGHLIGHTED_ANGLE           0x0003  /* i16 (degrees) */

#define CARPLAY_RGD_SOCK_PATH "/tmp/carplay_rgd.sock"

static inline uint16_t iap2_be16(const uint8_t *p) {
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static inline uint32_t iap2_be32(const uint8_t *p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

static inline void iap2_put_be16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)((v >> 8) & 0xFF);
    p[1] = (uint8_t)(v & 0xFF);
}

#ifdef __cplusplus
}
#endif

#endif /* IAP2_RGD_H */
