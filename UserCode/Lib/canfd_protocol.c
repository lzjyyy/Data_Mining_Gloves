#include "canfd_protocol.h"
#include <string.h>

void canfd_proto_init(canfd_proto_ctx_t *ctx)
{
    memset(ctx, 0, sizeof(*ctx));
}

void canfd_proto_register(canfd_proto_ctx_t *ctx, uint8_t func, canfd_handler_fn fn)
{
    ctx->table[func] = fn;
}

void canfd_reply_begin(canfd_reply_t *rpl, uint8_t func)
{
    rpl->len = 0;
    rpl->txbuf[rpl->len++] = func;
}

void canfd_reply_push(canfd_reply_t *rpl, const void *data, uint16_t n)
{
    if (n == 0) return;
    memcpy(&rpl->txbuf[rpl->len], data, n);
    rpl->len += n;
}

void canfd_reply_push_u8(canfd_reply_t *rpl, uint8_t v)
{
    rpl->txbuf[rpl->len++] = v;
}

void canfd_reply_push_u16_be(canfd_reply_t *rpl, uint16_t v)
{
    rpl->txbuf[rpl->len++] = (uint8_t)((v >> 8) & 0xFF);
    rpl->txbuf[rpl->len++] = (uint8_t)(v & 0xFF);
}

void canfd_reply_exception(canfd_reply_t *rpl, uint8_t func, uint8_t ex)
{
    rpl->len = 0;
    rpl->txbuf[rpl->len++] = (uint8_t)(func | 0x80);
    rpl->txbuf[rpl->len++] = ex;
    rpl->has_reply = true;
}

bool canfd_proto_dispatch(canfd_proto_ctx_t *ctx, const uint8_t *rx, uint16_t rx_len, canfd_reply_t *rpl)
{
    if (rpl) {
        rpl->len = 0;
        rpl->has_reply = false;
        rpl->is_OTA_reply = false;
        rpl->is_reset_reply = false;
    }

    if (!rx || rx_len < 1) {
        return false;
    }

    canfd_frame_t frm;
    frm.func = rx[0];
    frm.raw = rx;
    frm.raw_len = rx_len;

    canfd_handler_fn fn = ctx->table[frm.func];
    if (fn) {
        bool need_send = fn(&frm, rpl);
        if (rpl) rpl->has_reply = need_send;
    } else {
        if (rpl) {
            canfd_reply_exception(rpl, frm.func, 0x01); // 非法功能码
        }
    }

    return true;
}



