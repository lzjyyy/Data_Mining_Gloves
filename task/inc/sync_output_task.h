#ifndef __SYNC_OUTPUT_TASK_H__
#define __SYNC_OUTPUT_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void StartSyncOutputTask(void *argument);
void SyncOutput_OnButtonExti(uint16_t gpio_pin);
uint8_t SyncOutput_IsRunning(void);
uint32_t SyncOutput_GetToggleCount(void);
uint32_t SyncOutput_GetPulseCount(void);

#ifdef __cplusplus
}
#endif

#endif /* __SYNC_OUTPUT_TASK_H__ */
