#include "rs485_protocol.h"
#include <string.h>

/* ================= CRC ================= */

uint16_t rs485_crc16_modbus(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
            else              crc = (crc >> 1);
        }
    }
    return crc; /* 输出时：帧尾附加 低字节 -> 高字节 */
}

/* ================= 内部：解析并分类错误 ================= */

typedef enum {
    PARSE_OK = 0,
    PARSE_IGNORE,
} parse_status_t;

static parse_status_t
parse_and_classify(const rs485_proto_ctx_t *ctx,
                   const uint8_t *rx, uint16_t rx_len,
                   rs485_frame_t *out)
{
    if (!rx || rx_len < 4) {
        if (out) memset(out, 0, sizeof(*out));
        return PARSE_IGNORE; /* Too short/noise: silent. */
    }

    uint8_t addr = rx[0];
    uint8_t func = rx[1];

    bool is_broadcast = (addr == 0x00);
    if (is_broadcast || addr != ctx->slave_addr) {
        if (out) { out->addr = addr; out->func = func; out->raw = rx; out->raw_len = rx_len; }
        return PARSE_IGNORE;
    }

    /* CRC 校验（帧尾 2 字节：低字节在前） */
    uint16_t calc = rs485_crc16_modbus(rx, rx_len - 2);
    uint8_t crcL = (uint8_t)(calc & 0xFF);
    uint8_t crcH = (uint8_t)((calc >> 8) & 0xFF);
    if (rx[rx_len - 2] != crcL || rx[rx_len - 1] != crcH) {
        if (out) { out->addr = addr; out->func = func; out->raw = rx; out->raw_len = rx_len; }
        return PARSE_IGNORE;
    }

    if (out) {
        out->addr    = addr;
        out->func    = func;
        out->raw     = rx;
        out->raw_len = rx_len;
        out->is_broadcast = false;
    }
    return PARSE_OK;
}

/* ================= 回包工具 ================= */

void rs485_reply_begin(rs485_reply_t *rpl, uint8_t addr, uint8_t func)
{
    rpl->len = 0;
    rpl->txbuf[rpl->len++] = addr;
    rpl->txbuf[rpl->len++] = func;
}

void rs485_reply_push(rs485_reply_t *rpl, const void *data, uint16_t n)
{
    if (n == 0) return;
    memcpy(&rpl->txbuf[rpl->len], data, n);
    rpl->len += n;
}

void rs485_reply_push_u8(rs485_reply_t *rpl, uint8_t v)
{
    rpl->txbuf[rpl->len++] = v;
}

void rs485_reply_push_u16_be(rs485_reply_t *rpl, uint16_t v)
{
    rpl->txbuf[rpl->len++] = (uint8_t)((v >> 8) & 0xFF); /* 高字节先放 */
    rpl->txbuf[rpl->len++] = (uint8_t)(v & 0xFF);
}

void rs485_reply_finalize(rs485_reply_t *rpl)
{
    uint16_t crc = rs485_crc16_modbus(rpl->txbuf, rpl->len);
    rpl->txbuf[rpl->len++] = (uint8_t)(crc & 0xFF);       /* 低字节 */
    rpl->txbuf[rpl->len++] = (uint8_t)((crc >> 8) & 0xFF);/* 高字节 */
}

void rs485_reply_exception(rs485_reply_t *rpl, uint8_t addr, uint8_t func, modbus_exception_t ex)
{
    rs485_reply_begin(rpl, addr, (uint8_t)(func | 0x80));
    rs485_reply_push_u8(rpl, (uint8_t)ex);
    rs485_reply_finalize(rpl);
}

/* ================= 对外：分发 ================= */

void rs485_proto_init(rs485_proto_ctx_t *ctx, uint8_t slave_addr)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->slave_addr = slave_addr;
}

void rs485_proto_register(rs485_proto_ctx_t *ctx, uint8_t func, rs485_handler_fn fn)
{
    ctx->table[func] = fn;
}

bool rs485_proto_dispatch(rs485_proto_ctx_t *ctx, const uint8_t *rx, uint16_t rx_len, rs485_reply_t *rpl)
{
    if (rpl) { 
        rpl->len = 0; 
        rpl->has_reply = false; 
    }

    rs485_frame_t frm;
    parse_status_t st = parse_and_classify(ctx, rx, rx_len, &frm);

    if (st != PARSE_OK) {
        return false;
    }

    rs485_handler_fn fn = ctx->table[frm.func];
    if (fn) {
        bool need_send = fn(&frm, rpl);
        if (rpl) rpl->has_reply = need_send;
    } else {
        if (rpl) {
            rs485_reply_exception(rpl, frm.addr, frm.func, MODBUS_EX_ILLEGAL_FUNCTION);
            rpl->has_reply = true;
        }
    }
    return true;
}
