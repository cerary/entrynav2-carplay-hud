#ifndef VMOST_API_H
#define VMOST_API_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Verified against EntryNav2 firmware 202005181837:
 *   /usr/local/lib/libvmostclient.so.1.6.0
 *   /opt/navi/EBNavi/libVMostAdapter.so
 */

typedef struct {
    uint16_t dev_addr;     /* +0x00: MOST logical address (0xFFFF = registry lookup / wildcard, or 0x82 node addr) */
    uint8_t  fblock_id;    /* +0x02: FBlock ID (0x82 = HUDTerminal) */
    uint8_t  inst_id;      /* +0x03: Instance ID (0x00 on 2021 MINI F56) */
    uint16_t func_id;      /* +0x04: Function ID */
    uint16_t pad_06;       /* +0x06 */
    uint32_t op_type;      /* +0x08: MOST OpType */
    uint16_t data_len;     /* +0x0C: Payload length in bytes */
    uint16_t pad_0e;       /* +0x0E */
    uint8_t *data;         /* +0x10: Pointer to payload */
    uint32_t reserved_14;  /* +0x14: 0 = normal MOST msg, 2 = PACK (MostHighPacketizer buffer) */
} VMost_Msg_t;             /* sizeof == 24 bytes */

/* Confirmed 8-byte entry from vmostIsFblockRegistered (0x3d54..0x3dc4) */
typedef struct {
    uint8_t  fblock_id;    /* +0x00 */
    uint8_t  inst_id;      /* +0x01 */
    uint16_t dev_addr;     /* +0x02 (observed in vmostctl list-fblock) */
    uint32_t pid;          /* +0x04 */
} VMost_FBlock_Registration_t; /* sizeof == 8 bytes */

/* MOST OpTypes (standard MOST / BMW vmost) */
#define VMOST_OP_SET            0x00
#define VMOST_OP_GET            0x01
#define VMOST_OP_SET_GET        0x02
#define VMOST_OP_START          0x05
#define VMOST_OP_START_RES_ACK  0x06
#define VMOST_OP_START_RESULT   0x08
#define VMOST_OP_STATUS         0x0C
#define VMOST_OP_ERROR          0x0F

/* FBlock 0x82 (HUDTerminal) Function IDs verified from libVMostAdapter.so (CPxyHUD_Terminal) */
#define HUD_FBLOCK_ID           0x82
#define HUD_INST_ID             0x00

#define HUD_FUNC_CREATE_INSTANCE         0x0200 /* OpType 0x06: UWord handle, UByte type, UByte sub, MOSTString name -> ResultAck(0x0D): UWord handle, UWord instId */
#define HUD_FUNC_RELEASE_INSTANCE        0x0203 /* OpType 0x06: UWord instId */
#define HUD_FUNC_SET_VISIBILITY          0x0204 /* OpType 0x06: UWord instId, UByte visible */
#define HUD_FUNC_IS_VISIBLE              0x0205 /* OpType 0x06: UWord instId */
#define HUD_FUNC_SWITCH_VISIBILITY       0x020E /* OpType 0x06: UWord instId1, UWord instId2 */
#define HUD_FUNC_TEXTAREA_SET_TEXT       0x0210 /* OpType 0x06: UWord instId, MOSTString text ([0x01][ascii][0x00]) */
#define HUD_FUNC_IMAGE_SET_LINE_COLOR    0x0218 /* OpType 0x00: UWord instId, UByte color */
#define HUD_FUNC_IMAGE_SET_LINE_WIDTH    0x0219 /* OpType 0x06: UWord instId, UByte width */
#define HUD_FUNC_IMAGE_SET_FILL_COLOR    0x021B /* OpType 0x08: UWord instId, UByte color */
#define HUD_FUNC_IMAGE_SET_FILL_MODE     0x021C /* OpType 0x08: UWord instId, UByte mode */
#define HUD_FUNC_IMAGE_ADD_POLYGON       0x021D /* OpType 0x06: UWord instId, UWord zero, UByte polyMode, UByte nPts, (UWord x, UWord y)* */
#define HUD_FUNC_IMAGE_REMOVE_ALL_ITEMS  0x0220 /* OpType 0x06: UWord instId */
#define HUD_FUNC_POTI_SET_TYPE           0x0226 /* OpType 0x06 */
#define HUD_FUNC_POTI_SET_RANGE          0x0229 /* OpType 0x06 */
#define HUD_FUNC_FRAME_ADD_ITEM          0x022B /* OpType 0x06: UWord handle, UWord frameId, UByte pos(0x14), UWord itemId */
#define HUD_FUNC_TERMINAL_INTERFACE      0x0300 /* MHP Packetizer flush target */
#define HUD_FUNC_IMAGE_GET_SIZE          0x0C04 /* OpType 0x06: UWord instId */

/* Instance types for HUD_FUNC_CREATE_INSTANCE (0x0200) */
#define HUD_TYPE_TEXTAREA                0x03   /* sub=0x00: "Distance", "Unit", "ComingStreet" */
#define HUD_TYPE_IMAGE                   0x04   /* sub=0x00: "1.GuidingIcon", "2.GuidingIcon" */
#define HUD_TYPE_POTENTIOMETER           0x07   /* sub=0x02: BarGraph */
#define HUD_TYPE_FRAME                   0x08   /* sub=0x00: "TopFrame" */

typedef void (*VMost_NetEventCb)(uint32_t net_event);
typedef void (*VMost_NetEventCbExt)(uint32_t net_event, void *user_data);
typedef void (*VMost_MsgReceivedCb)(const VMost_Msg_t *msg, void *user_data);

int vmostConnect(VMost_MsgReceivedCb msg_cb, void *user_data, uint8_t use_glib_mainloop);
int vmostDisconnect(void);
int vmostFileDescriptorGet(int *fd_out);
int vmostProcessMessages(void);
int vmostOwnAddrGet(uint16_t *addr_out);
int vmostMPRGet(uint8_t *mpr_out);
int vmostGetNetworkMasterConfigurationStatus(uint32_t *status_out);

int vmostRegisterToEvents(VMost_NetEventCb cb);
int vmostRegisterToEventsExt(VMost_NetEventCbExt cb);

int vmostFblockShadowRegister(uint8_t fblock_id, uint8_t inst_id);
int vmostFblockShadowDeregister(uint8_t fblock_id, uint8_t inst_id);

int vmostGetRegisteredFblocks(VMost_FBlock_Registration_t *arr, uint32_t *count);
int vmostGetRegisteredFblockShadows(VMost_FBlock_Registration_t *arr, uint32_t *count);

int vmostMostMsgSend(VMost_Msg_t *msg);
int vmostMostPacketizerFlush(uint16_t tgt_addr, uint8_t fblock_id, uint8_t inst_id, uint16_t func_id, uint32_t op_type);

const char *vmostErrToString(int err);
char *vmostMessageToString(const VMost_Msg_t *msg);
int LogDefaultDomainSet(const char *domain);
int LogSetDLTApplicationID(const char *app_id, const char *desc);

#ifdef __cplusplus
}
#endif

#endif /* VMOST_API_H */
