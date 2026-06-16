#ifndef __SD_LOG_H__
#define __SD_LOG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define SDLOG_FRAME_SIZE            512U
#define SDLOG_WRITE_BUFFER_SIZE     (16U * 1024U)
#define SDLOG_QUEUE_LENGTH          128U
#define SDLOG_FILE_LIST_MAX         32U
#define SDLOG_NAME_BYTES            32U

typedef enum
{
  SDLOG_OK = 0,
  SDLOG_ERR_PARAM = -1,
  SDLOG_ERR_NOT_INITIALIZED = -2,
  SDLOG_ERR_MOUNT = -3,
  SDLOG_ERR_OPEN = -4,
  SDLOG_ERR_WRITE = -5,
  SDLOG_ERR_SYNC = -6,
  SDLOG_ERR_CLOSE = -7,
  SDLOG_ERR_NOT_RECORDING = -8,
  SDLOG_ERR_NO_SPACE = -9,
  SDLOG_ERR_BUSY = -10,
  SDLOG_ERR_QUEUE_FULL = -11,
  SDLOG_ERR_TIMEOUT = -12,
  SDLOG_ERR_INTERNAL = -13
} SdLogResult_t;

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
  char current_filename[SDLOG_NAME_BYTES];
  char last_filename[SDLOG_NAME_BYTES];
  uint16_t file_list_count;
  char file_list_names[SDLOG_FILE_LIST_MAX][SDLOG_NAME_BYTES];
  uint64_t file_list_sizes[SDLOG_FILE_LIST_MAX];
} SdLogStatusSnapshot_t;

int SDLog_Init(void);
int SDLog_Mount(void);
int SDLog_Start(void);
int SDLog_Stop(void);
int SDLog_CreateNewFile(void);
int SDLog_WriteFrame(const uint8_t *data, uint32_t len);
int SDLog_Sync(void);
int SDLog_GetCurrentFileSize(uint64_t *size);
int SDLog_ListFiles(void);
int SDLog_GetFileInfo(const char *filename, uint64_t *size);
int SDLog_ReadFile(const char *filename, uint64_t offset, uint8_t *buf, uint32_t len, uint32_t *read_len);
int SDLog_GetStatusSnapshot(SdLogStatusSnapshot_t *status);
int SDLog_GetLastFResult(void);

void SdLog_Init(void);
void SdLog_RequestCreateFile(void);
void SdLog_RequestStart(void);
void SdLog_RequestStop(void);
void SdLog_RequestReset(void);
void SdLog_RequestScanLog(void);
void SdLog_OnTimPeriodElapsed(TIM_HandleTypeDef *htim);
void SdLog_GetStatus(SdLogStatusSnapshot_t *status);

#ifdef __cplusplus
}
#endif

#endif /* __SD_LOG_H__ */