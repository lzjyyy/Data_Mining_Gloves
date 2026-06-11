#include "../inc/rs485_task.h"

#include "cmsis_os2.h"
#include "RS485_uasrt.h"

#define RS485_TASK_EVT_RX_FRAME (1UL << 0)
#define RS485_TASK_EVT_TX_EVENT (1UL << 1)

static osThreadId_t rs485_task_id = NULL;

/* Task-level counters: RE counts RX frame notifications, TE counts UART TC notifications. */
static volatile uint32_t rs485_task_rx_events = 0U;
static volatile uint32_t rs485_task_tx_events = 0U;

static void RS485_TaskNotify(uint32_t flags)
{
  if (rs485_task_id != NULL)
  {
    (void)osThreadFlagsSet(rs485_task_id, flags);
  }
}

void RS485_TaskNotifyRxFrame(void)
{
  rs485_task_rx_events++;
  RS485_TaskNotify(RS485_TASK_EVT_RX_FRAME);
}

void RS485_TaskNotifyTxComplete(void)
{
  /* TE means UART transmission complete, not DMA transfer complete. */
  rs485_task_tx_events++;
  RS485_TaskNotify(RS485_TASK_EVT_TX_EVENT);
}

void RS485_TaskNotifyTxError(void)
{
  RS485_TaskNotify(RS485_TASK_EVT_TX_EVENT);
}

void RS485_TaskGetEventCounts(uint32_t *rx_event_count, uint32_t *tx_event_count)
{
  if (rx_event_count != NULL)
  {
    *rx_event_count = rs485_task_rx_events;
  }

  if (tx_event_count != NULL)
  {
    *tx_event_count = rs485_task_tx_events;
  }
}

void StartRs485Task(void *argument)
{
  uint32_t flags;

  (void)argument;
  rs485_task_id = osThreadGetId();

  for (;;)
  {
    /* No RX/TX event means the RS485 task stays blocked here. */
    flags = osThreadFlagsWait(RS485_TASK_EVT_RX_FRAME | RS485_TASK_EVT_TX_EVENT,
                              osFlagsWaitAny,
                              osWaitForever);
    if ((flags & osFlagsError) == 0U)
    {
      if ((flags & RS485_TASK_EVT_TX_EVENT) != 0U)
      {
        RS485_ProcessTxEvent();
      }

      (void)(flags & RS485_TASK_EVT_RX_FRAME);
    }
  }
}
