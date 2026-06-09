#ifndef __RS485_TASK_H__
#define __RS485_TASK_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void StartRs485Task(void *argument);

/* Called by UART RX callbacks after a complete ReceiveToIdle frame is copied. */
void RS485_TaskNotifyRxFrame(void);

/* Called only after UART TC completion; this is the LCD TE counter source. */
void RS485_TaskNotifyTxComplete(void);

/* Wake the task for TX recovery without incrementing the TC-complete counter. */
void RS485_TaskNotifyTxError(void);

/* Read RS485 task-level event counters for LCD/debug output. */
void RS485_TaskGetEventCounts(uint32_t *rx_event_count, uint32_t *tx_event_count);

#ifdef __cplusplus
}
#endif

#endif /* __RS485_TASK_H__ */
