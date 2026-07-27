#ifndef __CANFD_PROTOCOL_H__
#define __CANFD_PROTOCOL_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t  func;        // 功能码
    const uint8_t *raw;   // 指向 func 开始
    uint16_t raw_len;     // 原始长度（不含CRC）
} canfd_frame_t;

typedef struct {
    uint8_t  txbuf[64];
    uint16_t len;
    bool     has_reply;
    bool     is_OTA_reply;
    bool     is_reset_reply;
} canfd_reply_t;

typedef bool (*canfd_handler_fn)(const canfd_frame_t *frm, canfd_reply_t *rpl);

typedef struct {
    canfd_handler_fn table[256];
} canfd_proto_ctx_t;

void canfd_proto_init(canfd_proto_ctx_t *ctx);
void canfd_proto_register(canfd_proto_ctx_t *ctx, uint8_t func, canfd_handler_fn fn);
bool canfd_proto_dispatch(canfd_proto_ctx_t *ctx, const uint8_t *rx, uint16_t rx_len, canfd_reply_t *rpl);

/* 回包工具 */
void canfd_reply_begin(canfd_reply_t *rpl, uint8_t func);
void canfd_reply_push(canfd_reply_t *rpl, const void *data, uint16_t n);
void canfd_reply_push_u8(canfd_reply_t *rpl, uint8_t v);
void canfd_reply_push_u16_be(canfd_reply_t *rpl, uint16_t v);
void canfd_reply_exception(canfd_reply_t *rpl, uint8_t func, uint8_t ex);

#ifdef __cplusplus
}
#endif

#endif

