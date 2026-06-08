#ifndef __RS485_UASRT_H__
#define __RS485_UASRT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define RS485_RX_BUFFER_SIZE 1024U
#define RS485_TX_BUFFER_SIZE 1024U
#define RS485_RX_GPIO_LEVEL GPIO_PIN_RESET
#define RS485_TX_GPIO_LEVEL GPIO_PIN_SET

typedef struct
{
  uint32_t init_calls;
  uint32_t rx_start_ok;
  uint32_t tx_requests;
  uint32_t tx_from_init;
  uint32_t tx_from_echo_task;
  uint32_t tx_dma_irq;
  uint32_t tx_cplt_callback;
  uint32_t rx_events;
  uint32_t rx_bytes;
  uint32_t rx_overwrite;
  uint32_t tx_done;
  uint32_t errors;
  uint8_t tx_busy;
} RS485_StatusTypeDef;

HAL_StatusTypeDef RS485_Init(void);
HAL_StatusTypeDef RS485_StartReceive(void);
HAL_StatusTypeDef RS485_SendDMA(const uint8_t *data, uint16_t size);
HAL_StatusTypeDef RS485_Send(const uint8_t *data, uint16_t size);
uint8_t RS485_TakeRxFrame(uint8_t *data, uint16_t *size, uint16_t max_size);
uint8_t RS485_IsTxBusy(void);
void RS485_PollEcho(void);
void RS485_GetStatus(RS485_StatusTypeDef *status);
void RS485_OnTxDmaIrq(void);

#ifdef __cplusplus
}
#endif

#endif
