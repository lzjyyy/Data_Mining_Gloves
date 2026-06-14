#ifndef __SD_LOG_H__
#define __SD_LOG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define SD_LOG_BLOCK_SIZE          1024U
#define SD_LOG_WRITE_BLOCKS        2U
#define SD_LOG_WRITE_SIZE          (SD_LOG_BLOCK_SIZE * SD_LOG_WRITE_BLOCKS)
#define SD_LOG_FILENAME_BYTES      32U

typedef enum
{
  SD_LOG_FS_NOT_MOUNTED = 0U,
  SD_LOG_FS_MOUNTED = 1U
} SdLogFsStatus_t;

typedef enum
{
  SD_LOG_RECORD_IDLE = 0U,
  SD_LOG_RECORD_RECORDING = 1U,
  SD_LOG_RECORD_STOPPING = 2U,
  SD_LOG_RECORD_ERROR = 0x8000U
} SdLogRecordStatus_t;

typedef struct
{
  uint16_t fs_status;
  uint16_t log_status;
  uint16_t error_code;
  uint16_t current_file_id;
  uint64_t current_file_size;
  uint32_t current_write_count;
  uint32_t total_size_mb;
  uint32_t free_size_mb;
  uint32_t used_size_mb;
  char current_filename[SD_LOG_FILENAME_BYTES];
  char last_filename[SD_LOG_FILENAME_BYTES];
} SdLogStatusSnapshot_t;

void SdLog_Init(void);
void SdLog_RequestCreateFile(void);
void SdLog_RequestStart(void);
void SdLog_RequestStop(void);
void SdLog_RequestReset(void);
void SdLog_OnTimPeriodElapsed(TIM_HandleTypeDef *htim);
void SdLog_GetStatus(SdLogStatusSnapshot_t *status);

#ifdef __cplusplus
}
#endif

#endif /* __SD_LOG_H__ */
