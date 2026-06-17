#include "../inc/sd_task.h"

#include "cmsis_os2.h"
#include "sd_log.h"

#define SD_TASK_RETRY_DELAY_MS    1000U

volatile uint32_t sd_task_entry_count = 0U;
volatile uint32_t sd_task_loop_count = 0U;
volatile int32_t sd_task_last_result = 0;

void StartSdTask(void *argument)
{
  SdLogStatusSnapshot_t status;
  uint8_t log_initialized = 0U;
  int result;

  (void)argument;

  sd_task_entry_count++;

  for (;;)
  {
    sd_task_loop_count++;

    if (log_initialized == 0U)
    {
      result = SDLog_Init();
      sd_task_last_result = result;
      if (result == SDLOG_OK)
      {
        log_initialized = 1U;
      }
    }

    if (log_initialized != 0U)
    {
      result = SDLog_GetStatusSnapshot(&status);
      sd_task_last_result = result;

      if ((result == SDLOG_OK) &&
          (status.fs_status != SD_LOG_FS_MOUNTED) &&
          (status.log_status != SD_LOG_RECORD_RECORDING) &&
          (status.log_status != SD_LOG_RECORD_STOPPING))
      {
        sd_task_last_result = SDLog_Mount();
      }
    }

    osDelay(SD_TASK_RETRY_DELAY_MS);
  }
}
