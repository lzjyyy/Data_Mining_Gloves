#include "canfd_protocol.h"
#include "canfd_handlers.h"
#include "bsp_fdcan.h"
#include "cmsis_os2.h"
#include <stdio.h>

extern uint8_t rx_data1[64];
extern uint16_t rec_id1;
extern volatile uint8_t fdcan1_rx_len;
extern FDCAN_HandleTypeDef hfdcan1;

static canfd_proto_ctx_t g_canfd_ctx;
static uint8_t g_canfd_inited = 0;

void StartCANFDRecvTask(void *argument)
{
    bsp_can_init();

    if (!g_canfd_inited) {
        canfd_proto_init(&g_canfd_ctx);
        canfd_handlers_init(&g_canfd_ctx);
        g_canfd_inited = 1;
        printf("CANFD protocol init done\r\n");
    }

    for (;;)
    {
        if (fdcan1_rx_len > 0)
        {
            uint8_t len = fdcan1_rx_len;
            fdcan1_rx_len = 0;

            printf("CANFD RX ID=0x%03X LEN=%d\r\n", rec_id1, len);

            if (rec_id1 == 0x100)
            {
                canfd_reply_t rpl;
                canfd_proto_dispatch(&g_canfd_ctx, rx_data1, len, &rpl);

                if (rpl.has_reply)
                {
                    if (fdcanx_send_data(&hfdcan1, 0x180, rpl.txbuf, rpl.len) == 0)
                    {
                        printf("CANFD REPLY TX OK\r\n");
                    }
                    else
                    {
                        printf("CANFD REPLY TX FAIL\r\n");
                    }
                }
            }
        }

        osDelay(10);
    }
}


