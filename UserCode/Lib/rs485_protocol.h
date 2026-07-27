#ifndef RS485_PROTOCOL_H
#define RS485_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* CRC16 (Modbus, 低字节在前) */
uint16_t rs485_crc16_modbus(const uint8_t *data, uint16_t len);

/* 功能码 */
typedef enum {
    MODBUS_FC_READ_HOLDING_REGS  = 0x03,
    MODBUS_FC_WRITE_SINGLE_REG   = 0x06,
    MODBUS_FC_WRITE_MULTIPLE_REG = 0x10,
} rs485_func_t;

/* 异常码 */
typedef enum {
    MODBUS_EX_ILLEGAL_FUNCTION    = 0x01,       //非法功能码
    MODBUS_EX_ILLEGAL_DATA_ADDR   = 0x02,       //非法数据地址
    MODBUS_EX_ILLEGAL_DATA_VALUE  = 0x03,       //非法数据值
    MODBUS_EX_SLAVE_DEVICE_FAILURE  = 0x04,     //从机设备故障

} modbus_exception_t;

/* 解析后的“帧视图” */
typedef struct {
    uint8_t  addr;        /* 从站地址 */
    uint8_t  func;        /* 功能码 */
    const uint8_t *raw;   /* 原始首地址（addr处） */
    uint16_t raw_len;     /* 原始长度（含CRC） */
    bool     is_broadcast;
} rs485_frame_t;

/* 回包构建器 */
typedef struct {
    uint8_t  txbuf[256];
    uint16_t len;
    bool     has_reply;
    bool     is_OTA_reply;
    bool     is_reset_reply;
    bool     is_baud_change_reply;
    uint32_t new_baudrate;
} rs485_reply_t;

/* handler 函数签名 */
typedef bool (*rs485_handler_fn)(const rs485_frame_t *frm, rs485_reply_t *rpl);

/* 上下文 */
typedef struct {
    uint8_t          slave_addr;
    rs485_handler_fn table[256];
} rs485_proto_ctx_t;

/* API */
void rs485_proto_init(rs485_proto_ctx_t *ctx, uint8_t slave_addr);
void rs485_proto_register(rs485_proto_ctx_t *ctx, uint8_t func, rs485_handler_fn fn);
bool rs485_proto_dispatch(rs485_proto_ctx_t *ctx, const uint8_t *rx, uint16_t rx_len, rs485_reply_t *rpl);

/* 回包工具 */
void rs485_reply_begin(rs485_reply_t *rpl, uint8_t addr, uint8_t func);
void rs485_reply_push(rs485_reply_t *rpl, const void *data, uint16_t n);
void rs485_reply_push_u8(rs485_reply_t *rpl, uint8_t v);
void rs485_reply_push_u16_be(rs485_reply_t *rpl, uint16_t v_be); /* 直接传入主机序，函数内部按大端写入 */
void rs485_reply_finalize(rs485_reply_t *rpl);

/* 构造异常应答（addr, func|0x80, ex_code, CRC） */
void rs485_reply_exception(rs485_reply_t *rpl, uint8_t addr, uint8_t func, modbus_exception_t ex);

#ifdef __cplusplus
}
#endif
#endif
