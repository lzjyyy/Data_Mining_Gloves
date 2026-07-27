#include "canfd_handlers.h"
#include "rs485_handlers.h"   // 复用寄存器定义、g_rs485_cfg
#include "fw_version.h"
#include "base_convert.h"
#include "data_manager.h"
#include "angle_Mapping.h"
#include "StoreTask.h"

#include <string.h>
#include <math.h>

#define MUZHIWANQU_ANGLE_MIN    83.21f
#define MUZHIWANQU_ANGLE_MAX   138.23f
#define MUZHIBAIDONG_ANGLE_MIN   6.47f
#define MUZHIBAIDONG_ANGLE_MAX  95.34f
#define SHIZHIWANQU_ANGLE_MIN   92.50f
#define SHIZHIWANQU_ANGLE_MAX  172.90f
#define ZHONGZHIWANQU_ANGLE_MIN 92.50f
#define ZHONGZHIWANQU_ANGLE_MAX 172.90f
#define WUMINGZHIWANQU_ANGLE_MIN 92.50f
#define WUMINGZHIWANQU_ANGLE_MAX 172.90f
#define XIAOMUZHIWANQU_ANGLE_MIN 92.50f
#define XIAOMUZHIWANQU_ANGLE_MAX 172.90f

static float angle_raw[6]     = {0};
static float angle_cmd[6]     = {0};
static float angle_vel_cmd[6] = {0};

static inline float clampf(float v, float lo, float hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

static inline uint16_t be_get_u16(const uint8_t *p)
{
    return (uint16_t)((p[0] << 8) | p[1]);
}

static inline void be_put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)((v >> 8) & 0xFF);
    p[1] = (uint8_t)(v & 0xFF);
}

static inline bool is_sys_reg(uint16_t addr)
{
    return (addr == REG_SYS_SLAVE_ADDR) || (addr == REG_SYS_BAUD_CODE);
}

static uint32_t baud_code_to_rate(uint16_t code)
{
    switch (code) {
    case 0x0001: return 9600;
    case 0x0002: return 19200;
    case 0x0003: return 38400;
    case 0x0004: return 57600;
    case 0x0005: return 115200;
    case 0x0006: return 230400;
    case 0x0007: return 460800;
    case 0x0008: return 921600;
    default:     return 0;
    }
}

static uint16_t baud_rate_to_code(uint32_t rate)
{
    switch (rate) {
    case 9600:   return 0x0001;
    case 19200:  return 0x0002;
    case 38400:  return 0x0003;
    case 57600:  return 0x0004;
    case 115200: return 0x0005;
    case 230400: return 0x0006;
    case 460800: return 0x0007;
    case 921600: return 0x0008;
    default:     return 0x0005;   // 默认115200
    }
}

/* CAN FD 合法 DLC 长度：
 * 0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64
 */
static uint16_t canfd_next_valid_len(uint16_t len)
{
    static const uint16_t valid_lens[] = {
        0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64
    };

    for (uint32_t i = 0; i < sizeof(valid_lens)/sizeof(valid_lens[0]); i++) {
        if (len <= valid_lens[i]) {
            return valid_lens[i];
        }
    }
    return 64;
}

static void canfd_reply_pad_to_valid_len(canfd_reply_t *rpl)
{
    uint16_t target = canfd_next_valid_len(rpl->len);
    while (rpl->len < target) {
        canfd_reply_push_u8(rpl, 0x00);
    }
}

static void dataProcess(void)
{
    mapInputAngleToModelAngle((const float*)angle_raw, angle_cmd, 6);

    angle_cmd[0] = clampf(angle_cmd[0], MUZHIWANQU_ANGLE_MIN,     MUZHIWANQU_ANGLE_MAX);
    angle_cmd[1] = clampf(angle_cmd[1], MUZHIBAIDONG_ANGLE_MIN,   MUZHIBAIDONG_ANGLE_MAX);
    angle_cmd[2] = clampf(angle_cmd[2], SHIZHIWANQU_ANGLE_MIN,    SHIZHIWANQU_ANGLE_MAX);
    angle_cmd[3] = clampf(angle_cmd[3], ZHONGZHIWANQU_ANGLE_MIN,  ZHONGZHIWANQU_ANGLE_MAX);
    angle_cmd[4] = clampf(angle_cmd[4], WUMINGZHIWANQU_ANGLE_MIN, WUMINGZHIWANQU_ANGLE_MAX);
    angle_cmd[5] = clampf(angle_cmd[5], XIAOMUZHIWANQU_ANGLE_MIN, XIAOMUZHIWANQU_ANGLE_MAX);

    DataManager_SetAngleCmdAll((const float *)angle_cmd, 6);
    DataManager_Commit();
    DataManager_SetAngleCmdAll((const float *)angle_cmd, 6);
    DataManager_Commit();

    DataManager_SetSpeedCmdAll((const float *)angle_vel_cmd, 6);
    DataManager_Commit();
    DataManager_SetSpeedCmdAll((const float *)angle_vel_cmd, 6);
    DataManager_Commit();
}

/* 0x03: 读寄存器 */
static bool handle_canfd_fc_0x03(const canfd_frame_t *frm, canfd_reply_t *rpl)
{
    if (frm->raw_len != 5) {
        canfd_reply_exception(rpl, frm->func, 0x03);
        return true;
    }

    const uint8_t *p = &frm->raw[1];
    uint16_t start = be_get_u16(p); p += 2;
    uint16_t qty   = be_get_u16(p);

    /* 1) 读本机地址 + 波特率（CANFD里直接允许，不需要485广播地址） */
    if (start == REG_SYS_SLAVE_ADDR && qty == 2) {
        uint8_t payload[4];
        uint8_t self_addr = g_rs485_cfg.slave_addr;
        uint16_t baud_code = baud_rate_to_code(g_rs485_cfg.baudrate);

        be_put_u16(&payload[0], (uint16_t)(self_addr & 0x00FF));
        be_put_u16(&payload[2], baud_code);

        canfd_reply_begin(rpl, frm->func);
        canfd_reply_push_u16_be(rpl, start);
        canfd_reply_push_u16_be(rpl, qty);
        canfd_reply_push_u8(rpl, 4);
        canfd_reply_push(rpl, payload, 4);
        canfd_reply_pad_to_valid_len(rpl);   // 10 -> 12
        return true;
    }

    /* 2) 读固件版本 */
    if (start == REG_FW_VER_BASE && qty == 5) {
        const fw_version_t *fw_ver = &g_fw_version;
        uint8_t payload[10];

        be_put_u16(&payload[0], (uint16_t)fw_ver->major);
        be_put_u16(&payload[2], (uint16_t)fw_ver->minor);
        be_put_u16(&payload[4], (uint16_t)fw_ver->patch);
        be_put_u16(&payload[6], (uint16_t)((fw_ver->data >> 16) & 0xFFFF));
        be_put_u16(&payload[8], (uint16_t)(fw_ver->data & 0xFFFF));

        canfd_reply_begin(rpl, frm->func);
        canfd_reply_push_u16_be(rpl, start);
        canfd_reply_push_u16_be(rpl, qty);
        canfd_reply_push_u8(rpl, 10);
        canfd_reply_push(rpl, payload, 10);
        canfd_reply_pad_to_valid_len(rpl);   // 16 已合法，不会补
        return true;
    }

    /* 3) 读反馈位置 / 速度 / 电流 */
    if (qty == 0 || qty > REG_FB_COUNT) {
        canfd_reply_exception(rpl, frm->func, 0x03);
        return true;
    }

    enum { SEG_NONE, SEG_FB_POS, SEG_FB_SPD, SEG_FB_CUR } seg = SEG_NONE;
    if (start == REG_FB_POS_BASE)      seg = SEG_FB_POS;
    else if (start == REG_FB_SPD_BASE) seg = SEG_FB_SPD;
    else if (start == REG_FB_CUR_BASE) seg = SEG_FB_CUR;
    else {
        canfd_reply_exception(rpl, frm->func, 0x02);
        return true;
    }

    uint16_t byteCount = (uint16_t)(qty * 2);
    uint8_t payload[12] = {0};
    float dataFeedback[6] = {0};

    switch (seg)
    {
    case SEG_FB_POS:
    {
        float userAngles[6] = {0};
        DataManager_GetAngleDuAll(dataFeedback, REG_FB_COUNT);
        mapModelAngleToInputAngle(dataFeedback, userAngles, REG_FB_COUNT);
        floats_to_bytes((const float *)userAngles, payload, qty);
        break;
    }

    case SEG_FB_SPD:
        DataManager_GetAngleSuduAll(dataFeedback, REG_FB_COUNT);
        floats_to_bytes((const float *)dataFeedback, payload, qty);
        break;

    case SEG_FB_CUR:
        DataManager_GetAllMotorCurrentsMea(dataFeedback, REG_FB_COUNT);
        for (int i = 0; i < REG_FB_COUNT; i++) {
            if (i == 0 || i == 1)
                dataFeedback[i] = dataFeedback[i] * 8.12e-6f;
            else
                dataFeedback[i] = dataFeedback[i] * 1.61e-5f;
        }
        floats_to_bytes((const float *)dataFeedback, payload, qty);
        break;

    default:
        canfd_reply_exception(rpl, frm->func, 0x04);
        return true;
    }

    canfd_reply_begin(rpl, frm->func);
    canfd_reply_push_u16_be(rpl, start);
    canfd_reply_push_u16_be(rpl, qty);
    canfd_reply_push_u8(rpl, (uint8_t)byteCount);
    canfd_reply_push(rpl, payload, byteCount);
    canfd_reply_pad_to_valid_len(rpl);   // 18 -> 20
    return true;
}

/* 0x06: 写单寄存器 */
static bool handle_canfd_fc_0x06(const canfd_frame_t *frm, canfd_reply_t *rpl)
{
    if (frm->raw_len != 5) {
        canfd_reply_exception(rpl, frm->func, 0x03);
        return true;
    }

    const uint8_t *p = &frm->raw[1];
    uint16_t reg = be_get_u16(p); p += 2;
    uint16_t val = be_get_u16(p);

    if (is_sys_reg(reg)) {
        if (reg == REG_SYS_SLAVE_ADDR) {
            uint8_t new_addr = (uint8_t)(val & 0xFF);
            if (new_addr < 1 || new_addr > 247) {
                canfd_reply_exception(rpl, frm->func, 0x03);
                return true;
            }
            g_rs485_cfg.slave_addr = new_addr;
            osEventFlagsSet(StoreEventsHandle, EVT_485ADDR_BIT);
        }
        else if (reg == REG_SYS_BAUD_CODE) {
            uint32_t baud_rate = baud_code_to_rate(val);
            if (baud_rate == 0) {
                canfd_reply_exception(rpl, frm->func, 0x03);
                return true;
            }
            g_rs485_cfg.baudrate = baud_rate;
            osEventFlagsSet(StoreEventsHandle, EVT_485BAUD_BIT);
        }

        canfd_reply_begin(rpl, frm->func);
        canfd_reply_push_u16_be(rpl, reg);
        canfd_reply_push_u16_be(rpl, val);
        return true;   // 5字节，合法
    }
    else if (reg == REG_RESET_BASE) {
        canfd_reply_begin(rpl, frm->func);
        canfd_reply_push_u16_be(rpl, reg);
        canfd_reply_push_u16_be(rpl, val);
        rpl->is_reset_reply = true;
        return true;   // 5字节，合法
    }
    else {
        canfd_reply_exception(rpl, frm->func, 0x02);
        return true;
    }
}

/* 0x10: 写多个寄存器 */
static bool handle_canfd_fc_0x10(const canfd_frame_t *frm, canfd_reply_t *rpl)
{
    if (frm->raw_len < 6) {
        canfd_reply_exception(rpl, frm->func, 0x03);
        return true;
    }

    const uint8_t *p = &frm->raw[1];
    uint16_t start = be_get_u16(p); p += 2;
    uint16_t qty   = be_get_u16(p); p += 2;
    uint8_t  byteCount = *p++;

    /* 通用字节数校验 */
    if (qty == 0 || byteCount != qty * 2) {
        canfd_reply_exception(rpl, frm->func, 0x03);
        return true;
    }

    /* 1) OTA */
//    if (start == RDG_OTA_BASE) {
//        uint8_t ota_flag[4] = {0x12, 0x34, 0x56, 0x78};

//        osEventFlagsSet(StoreEventsHandle, EVT_OTA_FLAG_BIT);

//        canfd_reply_begin(rpl, frm->func);
//        canfd_reply_push_u16_be(rpl, start);
//        canfd_reply_push_u16_be(rpl, qty);
//        canfd_reply_push_u8(rpl, 4);
//        canfd_reply_push(rpl, ota_flag, 4);
//        canfd_reply_pad_to_valid_len(rpl);   // 10 -> 12

//        rpl->is_OTA_reply = true;
//        return true;
//    }

    /* 2) 位置 + 速度命令 */
    if (start != REG_POS_BASE) {
        canfd_reply_exception(rpl, frm->func, 0x02);
        return true;
    }

    if (qty != (REG_POS_COUNT + REG_SPD_COUNT)) {
        canfd_reply_exception(rpl, frm->func, 0x03);
        return true;
    }

    if (frm->raw_len != (uint16_t)(1 + 2 + 2 + 1 + byteCount)) {
        canfd_reply_exception(rpl, frm->func, 0x03);
        return true;
    }

    const uint8_t *data = &frm->raw[6];

    bytes_to_floats(data, angle_raw, REG_POS_COUNT);
    data += REG_POS_COUNT * 2;
    bytes_to_floats(data, angle_vel_cmd, REG_SPD_COUNT);

    dataProcess();

    canfd_reply_begin(rpl, frm->func);
    canfd_reply_push_u16_be(rpl, start);
    canfd_reply_push_u16_be(rpl, qty);
    return true;   // 5字节，合法
}

void canfd_handlers_init(canfd_proto_ctx_t *ctx)
{
    canfd_proto_register(ctx, 0x03, handle_canfd_fc_0x03);
    canfd_proto_register(ctx, 0x06, handle_canfd_fc_0x06);
    canfd_proto_register(ctx, 0x10, handle_canfd_fc_0x10);
}

