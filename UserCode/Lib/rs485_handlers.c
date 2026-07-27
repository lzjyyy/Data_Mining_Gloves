#include "rs485_handlers.h"
#include <string.h>
#include <math.h>
#include "uart_redirect.h"
#include "base_convert.h"
#include "PosLoopTask.h"
#include "usart.h"
#include "StoreTask.h"
#include "data_manager.h"
#include "angle_Mapping.h"
#include "fw_version.h"
#include "LedTask.h"

/* 与原任务文件一致的限幅范围（后续可提取成单独头文件统一引用） */
#define MUZHIWANQU_ANGLE_MIN   83.21f
#define MUZHIWANQU_ANGLE_MAX  138.23f
#define MUZHIBAIDONG_ANGLE_MIN  6.47f
#define MUZHIBAIDONG_ANGLE_MAX 95.34f
#define SHIZHIWANQU_ANGLE_MIN  92.50f
#define SHIZHIWANQU_ANGLE_MAX 172.90f
#define ZHONGZHIWANQU_ANGLE_MIN 92.50f
#define ZHONGZHIWANQU_ANGLE_MAX 172.90f
#define WUMINGZHIWANQU_ANGLE_MIN 92.50f
#define WUMINGZHIWANQU_ANGLE_MAX 172.90f
#define XIAOMUZHIWANQU_ANGLE_MIN 92.50f
#define XIAOMUZHIWANQU_ANGLE_MAX 172.90f


struct rs485_cfg g_rs485_cfg = {
    .slave_addr = 0xC8,
    .baudrate   = 115200,
};

/* 本地缓存（与原逻辑等价） */
static float angle_raw[6]          = {0};
static float angle_cmd[6]          = {0};
static float angle_vel_cmd[6]      = {0};


static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
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
    default:     return 0x0005;
    }
}


/* 0x03：只允许从段“起始地址”读取反馈数据；其它地址一律异常 0x02 */
static bool handle_fc_0x03(const rs485_frame_t *frm, rs485_reply_t *rpl)
{
    if (frm->is_broadcast) {
        return false;
    }

    if (frm->raw_len != 8) {
        return false;
    }

    const uint8_t *p = &frm->raw[2];
    uint16_t start = be_get_u16(p);     p += 2;
    uint16_t qty   = be_get_u16(p);

    if (start == REG_SYS_SLAVE_ADDR) {
        if (qty != 2) {
            rs485_reply_exception(rpl, frm->addr, frm->func, MODBUS_EX_ILLEGAL_DATA_VALUE);
            return true;
        }

        uint8_t byteCount = 4;
        uint8_t payload[4];
        be_put_u16(&payload[0], (uint16_t)g_rs485_cfg.slave_addr);
        be_put_u16(&payload[2], baud_rate_to_code(g_rs485_cfg.baudrate));

        rs485_reply_begin(rpl, frm->addr, frm->func);
        rs485_reply_push_u16_be(rpl, start);
        rs485_reply_push_u16_be(rpl, qty);
        rs485_reply_push_u8(rpl, byteCount);
        rs485_reply_push(rpl, payload, byteCount);
        rs485_reply_finalize(rpl);
        return true;
    }

    if(start == REG_FW_VER_BASE){
        if (qty != 5) {
            rs485_reply_exception(rpl, frm->addr, frm->func, MODBUS_EX_ILLEGAL_DATA_VALUE);
            return true;
        }

        //返回固件版本信息
        const fw_version_t *fw_ver = &g_fw_version;
        uint8_t  byteCount = 10; /* 5个寄存器 */
        uint8_t  payload[10];
        be_put_u16(&payload[0], (uint16_t)(fw_ver->major));   /* Reg 0x0B00: 主版本 */
        be_put_u16(&payload[2], (uint16_t)(fw_ver->minor));   /* Reg 0x0B01: 次版本 */
        be_put_u16(&payload[4], (uint16_t)(fw_ver->patch));   /* Reg 0x0B02: 修订 */
        be_put_u16(&payload[6], (uint16_t)((fw_ver->data >> 16) & 0xFFFF));   /* Reg 0x0B03: 发布日期高16位 */
        be_put_u16(&payload[8], (uint16_t)(fw_ver->data & 0xFFFF));   /* Reg 0x0B04: 发布日期低16位 */
        rs485_reply_begin(rpl, frm->addr, frm->func);
        rs485_reply_push_u16_be(rpl, start);
        rs485_reply_push_u16_be(rpl, qty);
        rs485_reply_push_u8(rpl, byteCount);
        rs485_reply_push(rpl, payload, byteCount);
        rs485_reply_finalize(rpl);
        return true;
    }

    /* Modbus 限制 + 本段最多 6 个 */
    if (qty == 0 || qty > REG_FB_COUNT /* 6 */) {
        rs485_reply_exception(rpl, frm->addr, frm->func, MODBUS_EX_ILLEGAL_DATA_VALUE);
        return true;
    }

    /* 仅允许这三个“起始地址” */
    enum { SEG_NONE, SEG_FB_POS, SEG_FB_SPD, SEG_FB_CUR } seg = SEG_NONE;
    if (start == REG_FB_POS_BASE)      seg = SEG_FB_POS;  /* 0x000E */
    else if (start == REG_FB_SPD_BASE) seg = SEG_FB_SPD;  /* 0x0014 */
    else if (start == REG_FB_CUR_BASE) seg = SEG_FB_CUR;  /* 0x001A */
    else {
        rs485_reply_exception(rpl, frm->addr, frm->func, MODBUS_EX_ILLEGAL_DATA_ADDR);
        return true;
    }

    /* 打包数据：每寄存器 2 字节（half，大端），从 idx=0 开始取 qty 个 */
    uint16_t byteCount = (uint16_t)(qty * 2);
    uint8_t  payload[12] = {0};  /* 最多 6 寄存器 -> 12 字节 */


    float dataFeedback[6];
    switch (seg) {
    case SEG_FB_POS: 
            //返回位置反馈
            DataManager_GetAngleDuAll(dataFeedback,REG_FB_COUNT);
            //映射到用户角度
            float userAngles[6];
            mapModelAngleToInputAngle(dataFeedback, userAngles, REG_FB_COUNT);
            floats_to_bytes((const float *)userAngles,payload,REG_FB_COUNT);
            break;
    case SEG_FB_SPD:
            //返回速度反馈
            DataManager_GetAngleSuduAll(dataFeedback,REG_FB_COUNT);
            floats_to_bytes((const float *)dataFeedback,payload,REG_FB_COUNT);   
            break;
    case SEG_FB_CUR:
            //返回电流反馈
            DataManager_GetAllMotorCurrentsMea(dataFeedback,REG_FB_COUNT);
            for(int i=0;i<REG_FB_COUNT;i++)
            {
                if(i==0||i==1)
                    dataFeedback[i]=dataFeedback[i]*8.12e-6; //10mm电机转换为实际电流值
                else
                    dataFeedback[i]=dataFeedback[i]*1.61e-5; //12mm电机转换为实际电流值
            }
            floats_to_bytes((const float *)dataFeedback,payload,REG_FB_COUNT); 
            break;
    default:
        rs485_reply_exception(rpl, frm->addr, frm->func, MODBUS_EX_SLAVE_DEVICE_FAILURE);
        return true;
    }

    rs485_reply_begin(rpl, frm->addr, frm->func);
    rs485_reply_push_u16_be(rpl, start);
    rs485_reply_push_u16_be(rpl, qty);
    rs485_reply_push_u8(rpl, (uint8_t)byteCount);
    rs485_reply_push(rpl, payload, byteCount);
    rs485_reply_finalize(rpl);
    return true;
}




/* 判断是否为系统寄存器（仅 0x0000 与 0x0001） */
static inline bool is_sys_reg(uint16_t addr) {
    return (addr == REG_SYS_SLAVE_ADDR) || (addr == REG_SYS_BAUD_CODE);
}

/* ====== 0x06 Write Single Register ====== */
static bool handle_fc_0x06(const rs485_frame_t *frm, rs485_reply_t *rpl)
{
    /* PDU: addr func  regHi regLo  valHi valLo  CRC */
    uint16_t reg = 0;
    uint16_t val = 0;

    if (frm->is_broadcast) {
        return false;
    }

    if (frm->raw_len != 8) {
        return false;
    }

    reg = be_get_u16(&frm->raw[2]);
    val = be_get_u16(&frm->raw[4]);

    // 系统寄存器处理
    if (is_sys_reg(reg)) {
        if (reg == REG_SYS_SLAVE_ADDR) {
            uint8_t new_addr = (uint8_t)(val & 0xFF);
            if (new_addr < 1 || new_addr > 247) {
                rs485_reply_exception(rpl, frm->addr, frm->func, MODBUS_EX_ILLEGAL_DATA_VALUE);
                return true;
            }
            g_rs485_cfg.slave_addr = new_addr;
            osEventFlagsSet(StoreEventsHandle, EVT_485ADDR_BIT); //通知存储任务更新地址

        } else if (reg == REG_SYS_BAUD_CODE) {
            uint16_t baud_code = val;
            uint32_t baud_rate = 0;
            switch (baud_code) {
                case 0x0001: baud_rate = 9600; break;
                case 0x0002: baud_rate = 19200; break;
                case 0x0003: baud_rate = 38400; break;
                case 0x0004: baud_rate = 57600; break;
                case 0x0005: baud_rate = 115200; break;
                case 0x0006: baud_rate = 230400; break;
                case 0x0007: baud_rate = 460800; break;
                case 0x0008: baud_rate = 921600; break;
                default:
                    rs485_reply_exception(rpl, frm->addr, frm->func, MODBUS_EX_ILLEGAL_DATA_VALUE);
                    return true;
            }
            // 将新的波特率写入EEPROM,需要重启串口才能使用新的波特率
            g_rs485_cfg.baudrate = baud_rate;
            osEventFlagsSet(StoreEventsHandle, EVT_485BAUD_BIT);
            rpl->is_baud_change_reply = true;
            rpl->new_baudrate = baud_rate; //通知存储任务更新波特率
        }

        /* 回应回显帧（标准 0x06 应答）：addr func regHi regLo valHi valLo CRC */
        rs485_reply_begin(rpl, frm->addr, frm->func);
        rs485_reply_push_u16_be(rpl, reg);
        rs485_reply_push_u16_be(rpl, val);
        rs485_reply_finalize(rpl);
        return true;
    }else if(reg == REG_RESET_BASE){
				LedTask_SetMode(LED_MODE_RESET);
        rs485_reply_begin(rpl, frm->addr, frm->func);
        rs485_reply_push_u16_be(rpl, reg);
        rs485_reply_push_u16_be(rpl, val);
        rs485_reply_finalize(rpl);
        rpl->is_reset_reply = true; 
        return true;
    }else{
        // 非系统寄存器写命令，返回错误
        rs485_reply_exception(rpl, frm->addr, frm->func, MODBUS_EX_ILLEGAL_DATA_ADDR);
        return true;
    }

}

//模型计算
static void dataProcess(void)
{
    //映射到 angle_cmd
    mapInputAngleToModelAngle((const float*)angle_raw, angle_cmd, 6);

    /* 输入角度限幅 */
    angle_cmd[0] = clampf(angle_cmd[0], MUZHIWANQU_ANGLE_MIN,     MUZHIWANQU_ANGLE_MAX);
    angle_cmd[1] = clampf(angle_cmd[1], MUZHIBAIDONG_ANGLE_MIN,   MUZHIBAIDONG_ANGLE_MAX);
    angle_cmd[2] = clampf(angle_cmd[2], SHIZHIWANQU_ANGLE_MIN,    SHIZHIWANQU_ANGLE_MAX);
    angle_cmd[3] = clampf(angle_cmd[3], ZHONGZHIWANQU_ANGLE_MIN,  ZHONGZHIWANQU_ANGLE_MAX);
    angle_cmd[4] = clampf(angle_cmd[4], WUMINGZHIWANQU_ANGLE_MIN, WUMINGZHIWANQU_ANGLE_MAX);
    angle_cmd[5] = clampf(angle_cmd[5], XIAOMUZHIWANQU_ANGLE_MIN, XIAOMUZHIWANQU_ANGLE_MAX);

    /* 上传用户角度 */
    DataManager_SetAngleCmdAll((const float *)angle_cmd, 6);
    DataManager_Commit();
    DataManager_SetAngleCmdAll((const float *)angle_cmd, 6);
    DataManager_Commit();

    /* 上传用户角速度*/
    DataManager_SetSpeedCmdAll((const float *)angle_vel_cmd, 6);
    DataManager_Commit();
    DataManager_SetSpeedCmdAll((const float *)angle_vel_cmd, 6);
    DataManager_Commit();
}

/* ====== 0x10 Write Multiple Registers ====== */
static bool handle_fc_0x10(const rs485_frame_t *frm, rs485_reply_t *rpl)
{
    /* PDU: addr func  startHi startLo  qtyHi qtyLo  byteCount  data...  CRC */
    if (frm->is_broadcast) {
        return false;
    }

    if (frm->raw_len < 9) {
        return false;
    }

    const uint8_t *p = &frm->raw[2];
    uint16_t start = be_get_u16(p); p += 2;
    uint16_t qty   = be_get_u16(p); p += 2;
    uint8_t  byteCount = frm->raw[6];

    if (frm->raw_len != (uint16_t)(9 + byteCount)) {
        return false;
    }

    /* 必须是寄存器数×2 = 字节数 */
    if (qty == 0 || byteCount != qty * 2) {
        rs485_reply_exception(rpl, frm->addr, frm->func, MODBUS_EX_ILLEGAL_DATA_VALUE);
        return true;
    }

    //OTA判断
    if(start == RDG_OTA_BASE)
    {
        if (qty != 2 || byteCount != 4) {
            rs485_reply_exception(rpl, frm->addr, frm->func, MODBUS_EX_ILLEGAL_DATA_VALUE);
            return true;
        }

				LedTask_SetMode(LED_MODE_BOOT);
        uint8_t ota_flag[4] = {0x12, 0x34, 0x56, 0x78};
        osEventFlagsSet(StoreEventsHandle, EVT_OTA_FLAG_BIT); //通知存储任务更新OTA标志
        rs485_reply_begin(rpl, frm->addr, frm->func);
        rs485_reply_push_u16_be(rpl, start);
        rs485_reply_push_u16_be(rpl, qty);
        rs485_reply_push_u8(rpl, byteCount);
        rs485_reply_push_u8(rpl, ota_flag[0]);
        rs485_reply_push_u8(rpl, ota_flag[1]);  
        rs485_reply_push_u8(rpl, ota_flag[2]);
        rs485_reply_push_u8(rpl, ota_flag[3]);
        rs485_reply_finalize(rpl);
        rpl->is_OTA_reply = true;
        return true;
    }

    /* 仅允许从 REG_POS_BASE 起写入“位置6 + 速度6”共 12 个寄存器 */
    if (start != REG_POS_BASE) {
        rs485_reply_exception(rpl, frm->addr, frm->func, MODBUS_EX_ILLEGAL_DATA_ADDR);
        return true;
    }
    if (qty != (REG_POS_COUNT + REG_SPD_COUNT)) { /* 必须=12 */
        rs485_reply_exception(rpl, frm->addr, frm->func, MODBUS_EX_ILLEGAL_DATA_VALUE);
        return true;
    }

    /* 数据区起始指针 */
    const uint8_t *data = &frm->raw[7];
    if (frm->raw_len < (uint16_t)(7 + byteCount + 2)) { /* 防御性长度校验 */
        return false;
    }

    /* 解析位置命令（前 6 个寄存器） */
    bytes_to_floats(data, angle_raw, REG_POS_COUNT);
    data += REG_POS_COUNT * 2;
    /* 解析速度命令（后 6 个寄存器） */
    bytes_to_floats(data, angle_vel_cmd, REG_SPD_COUNT);

    /* 更新模型计算线程数据 */
    dataProcess();

    /* 标准 0x10 应答：addr func startHi startLo qtyHi qtyLo CRC（不带数据） */
    rs485_reply_begin(rpl, frm->addr, frm->func);
    rs485_reply_push_u16_be(rpl, start);
    rs485_reply_push_u16_be(rpl, qty);
    rs485_reply_finalize(rpl);
    return true;
}

void rs485_handlers_init(rs485_proto_ctx_t *ctx)
{
    rs485_proto_register(ctx, MODBUS_FC_READ_HOLDING_REGS , handle_fc_0x03);
    rs485_proto_register(ctx, MODBUS_FC_WRITE_SINGLE_REG  , handle_fc_0x06);
    rs485_proto_register(ctx, MODBUS_FC_WRITE_MULTIPLE_REG, handle_fc_0x10);
}

void SetUart3Baudrate(uint32_t baudrate)
{
    g_rs485_cfg.baudrate = baudrate; 
};
uint32_t GetUart3Baudrate(void)
{ 
    return  g_rs485_cfg.baudrate; 
};
