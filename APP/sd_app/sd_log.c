#include "sd_log.h"

#include "cmsis_os2.h"
#include "ff.h"
#include "modbus_registers.h"
#include "modbus_time_sync.h"
#include "sdmmc.h"
#include "tim.h"
#include <stdio.h>
#include <string.h>

#define SD_LOG_EVT_CREATE          (1UL << 0)
#define SD_LOG_EVT_START           (1UL << 1)
#define SD_LOG_EVT_STOP            (1UL << 2)
#define SD_LOG_EVT_TICK            (1UL << 3)
#define SD_LOG_EVT_RESET           (1UL << 4)
#define SD_LOG_EVT_SCAN            (1UL << 5)

#define SD_LOG_CONTENT_SIZE        1021U
#define SD_LOG_FRAME_HEAD          0xA5U
#define SD_LOG_FRAME_TAIL          0x5AU
#define SD_LOG_SEPARATOR           0x00U
#define SD_LOG_HAND_RIGHT          0x00U
#define SD_LOG_HAND_LEFT           0x80U
#define SD_LOG_DATA_TYPE_MASK      0x7FU
#define SD_LOG_HAND_FLAG_MASK      0x80U
#define SD_LOG_DATA_TYPE_IMU       0x01U
#define SD_LOG_DATA_TYPE_JOINT     0x02U
#define SD_LOG_DATA_TYPE_TACTILE   0x03U
#define SD_LOG_PENDING_TICK_LIMIT  256U
#define SD_LOG_TICK_BATCH_LIMIT    16U
#define SD_LOG_MOUNT_RETRY_COUNT   5U
#define SD_LOG_MOUNT_RETRY_DELAY_MS 100U
#define SD_LOG_SYNC_INTERVAL_BYTES (64U * 1024U)

static osThreadId_t sd_log_task_id;
static FATFS sd_log_fs;
static FIL sd_log_file;
static volatile uint32_t sd_log_pending_ticks;
static uint32_t sd_log_unsynced_bytes;

static SdLogStatusSnapshot_t sd_log_status = {
  .fs_status = SD_LOG_FS_NOT_MOUNTED,
  .log_status = SD_LOG_RECORD_IDLE,
  .error_code = SD_ERROR_NONE
};

static __ALIGNED(4) uint8_t sd_log_frame[SD_LOG_BLOCK_SIZE];
static __ALIGNED(4) uint8_t sd_log_write_buffer[SD_LOG_WRITE_SIZE];
static uint8_t sd_log_write_frames;
static float sd_log_imu[160];
static float sd_log_joint[21];
static uint16_t sd_log_adc[132];

static FRESULT SdLog_Mount(void);

static uint16_t SdLog_Crc16(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0xFFFFU;

  for (uint16_t index = 0U; index < len; index++)
  {
    crc ^= data[index];
    for (uint8_t bit = 0U; bit < 8U; bit++)
    {
      if ((crc & 0x0001U) != 0U)
      {
        crc = (uint16_t)((crc >> 1) ^ 0xA001U);
      }
      else
      {
        crc >>= 1;
      }
    }
  }

  return crc;
}

static uint8_t SdLog_MakeDataId(uint8_t hand_flag, uint8_t data_type)
{
  return (uint8_t)((hand_flag & SD_LOG_HAND_FLAG_MASK) |
                   (data_type & SD_LOG_DATA_TYPE_MASK));
}

static uint64_t SdLog_GetTimestampUs(void)
{
  uint64_t timestamp = ModbusTimeSync_GetUtcTimestampUs();

  if (timestamp == 0U)
  {
    timestamp = ModbusTimeSync_GetLocalUptimeUs();
  }

  return timestamp;
}

static void SdLog_WriteU64Le(uint8_t *data, uint64_t value)
{
  for (uint8_t index = 0U; index < 8U; index++)
  {
    data[index] = (uint8_t)(value >> (index * 8U));
  }
}

static void SdLog_UpdateTestData(void)
{
  sd_log_imu[0] += 1.0f;
  for (uint16_t index = 1U; index < 160U; index++)
  {
    sd_log_imu[index] = sd_log_imu[index - 1U] + 1.0f;
  }

  sd_log_joint[0] += 1.0f;
  for (uint16_t index = 1U; index < 21U; index++)
  {
    sd_log_joint[index] = sd_log_joint[index - 1U] + 1.0f;
  }

  sd_log_adc[0]++;
  for (uint16_t index = 1U; index < 132U; index++)
  {
    sd_log_adc[index] = (uint16_t)(sd_log_adc[index - 1U] + 1U);
  }
}

static void SdLog_BuildFrame(uint8_t block[SD_LOG_BLOCK_SIZE])
{
  uint16_t offset = 0U;
  uint16_t crc;
  uint64_t timestamp_us = SdLog_GetTimestampUs();

  SdLog_UpdateTestData();

  block[offset++] = SD_LOG_FRAME_HEAD;
  block[offset++] = SdLog_MakeDataId(SD_LOG_HAND_RIGHT, SD_LOG_DATA_TYPE_IMU);
  memcpy(&block[offset], sd_log_imu, sizeof(sd_log_imu));
  offset = (uint16_t)(offset + sizeof(sd_log_imu));
  SdLog_WriteU64Le(&block[offset], timestamp_us);
  offset = (uint16_t)(offset + 8U);
  block[offset++] = SD_LOG_FRAME_TAIL;

  block[offset++] = SD_LOG_FRAME_HEAD;
  block[offset++] = SdLog_MakeDataId(SD_LOG_HAND_RIGHT, SD_LOG_DATA_TYPE_JOINT);
  memcpy(&block[offset], sd_log_joint, sizeof(sd_log_joint));
  offset = (uint16_t)(offset + sizeof(sd_log_joint));
  SdLog_WriteU64Le(&block[offset], timestamp_us);
  offset = (uint16_t)(offset + 8U);
  block[offset++] = SD_LOG_FRAME_TAIL;

  block[offset++] = SD_LOG_FRAME_HEAD;
  block[offset++] = SdLog_MakeDataId(SD_LOG_HAND_RIGHT, SD_LOG_DATA_TYPE_TACTILE);
  memcpy(&block[offset], sd_log_adc, sizeof(sd_log_adc));
  offset = (uint16_t)(offset + sizeof(sd_log_adc));
  SdLog_WriteU64Le(&block[offset], timestamp_us);
  offset = (uint16_t)(offset + 8U);
  block[offset++] = SD_LOG_FRAME_TAIL;

  crc = SdLog_Crc16(block, SD_LOG_CONTENT_SIZE);
  block[offset++] = (uint8_t)(crc & 0xFFU);
  block[offset++] = (uint8_t)(crc >> 8);
  block[offset++] = SD_LOG_SEPARATOR;
}

static void SdLog_SetError(uint16_t error_code)
{
  sd_log_status.error_code = error_code;
  sd_log_status.log_status = SD_LOG_RECORD_ERROR;
}

static void SdLog_UpdateCapacity(void)
{
  FATFS *fs;
  DWORD free_clusters;

  if (f_getfree("0:", &free_clusters, &fs) == FR_OK)
  {
    uint64_t total_sectors = (uint64_t)(fs->n_fatent - 2U) * fs->csize;
    uint64_t free_sectors = (uint64_t)free_clusters * fs->csize;
    uint64_t total_mb = total_sectors / 2048U;
    uint64_t free_mb = free_sectors / 2048U;

    sd_log_status.total_size_mb = (uint32_t)total_mb;
    sd_log_status.free_size_mb = (uint32_t)free_mb;
    sd_log_status.used_size_mb = (uint32_t)(total_mb - free_mb);
  }
}

static uint16_t SdLog_ParseLogFileId(const char *filename)
{
  uint16_t file_id = 0U;

  if ((filename == NULL) ||
      (strncmp(filename, "LOG", 3U) != 0) ||
      (strlen(filename) != 11U) ||
      (strcmp(&filename[7], ".BIN") != 0))
  {
    return 0U;
  }

  for (uint8_t index = 3U; index < 7U; index++)
  {
    if ((filename[index] < '0') || (filename[index] > '9'))
    {
      return 0U;
    }
    file_id = (uint16_t)((file_id * 10U) + (uint16_t)(filename[index] - '0'));
  }

  return file_id;
}

static void SdLog_UpdateNextFileId(void)
{
  DIR dir;
  FILINFO info;
  FRESULT result;
  uint16_t file_id;
  uint16_t max_file_id = 0U;

  result = f_opendir(&dir, "0:/");
  if (result != FR_OK)
  {
    return;
  }

  for (;;)
  {
    result = f_readdir(&dir, &info);
    if ((result != FR_OK) || (info.fname[0] == '\0'))
    {
      break;
    }

    file_id = SdLog_ParseLogFileId(info.fname);
    if (file_id > max_file_id)
    {
      max_file_id = file_id;
    }
  }

  (void)f_closedir(&dir);
  sd_log_status.current_file_id = max_file_id;
}

static FRESULT SdLog_ScanLatestFile(void)
{
  DIR dir;
  FILINFO info;
  FILINFO latest_info;
  FRESULT result;
  uint16_t file_id;
  uint16_t max_file_id = 0U;
  uint16_t list_count = 0U;

  if (sd_log_status.log_status == SD_LOG_RECORD_RECORDING)
  {
    return FR_DENIED;
  }

  if (sd_log_status.fs_status != SD_LOG_FS_MOUNTED)
  {
    result = SdLog_Mount();
    if (result != FR_OK)
    {
      return result;
    }
  }

  memset(&latest_info, 0, sizeof(latest_info));
  sd_log_status.file_list_count = 0U;
  memset(sd_log_status.file_list_names, 0, sizeof(sd_log_status.file_list_names));
  memset(sd_log_status.file_list_sizes, 0, sizeof(sd_log_status.file_list_sizes));

  result = f_opendir(&dir, "0:/");
  if (result != FR_OK)
  {
    SdLog_SetError((uint16_t)result);
    return result;
  }

  for (;;)
  {
    result = f_readdir(&dir, &info);
    if ((result != FR_OK) || (info.fname[0] == '\0'))
    {
      break;
    }

    file_id = SdLog_ParseLogFileId(info.fname);
    if ((file_id != 0U) && (list_count < SD_LOG_FILE_LIST_MAX))
    {
      snprintf(sd_log_status.file_list_names[list_count],
               sizeof(sd_log_status.file_list_names[list_count]),
               "0:/%s",
               info.fname);
      sd_log_status.file_list_sizes[list_count] = info.fsize;
      list_count++;
    }

    if (file_id > max_file_id)
    {
      max_file_id = file_id;
      latest_info = info;
    }
  }

  (void)f_closedir(&dir);

  if (result != FR_OK)
  {
    SdLog_SetError((uint16_t)result);
    return result;
  }

  if (max_file_id == 0U)
  {
    sd_log_status.current_file_id = 0U;
    sd_log_status.current_file_size = 0U;
    sd_log_status.current_write_count = 0U;
    sd_log_status.current_filename[0] = '\0';
    sd_log_status.file_list_count = list_count;
    sd_log_status.error_code = SD_ERROR_NONE;
    return FR_OK;
  }

  sd_log_status.current_file_id = max_file_id;
  sd_log_status.current_file_size = latest_info.fsize;
  sd_log_status.current_write_count = (uint32_t)(latest_info.fsize / SD_LOG_BLOCK_SIZE);
  snprintf(sd_log_status.current_filename,
           sizeof(sd_log_status.current_filename),
           "0:/%s",
           latest_info.fname);
  strncpy(sd_log_status.last_filename,
          sd_log_status.current_filename,
          sizeof(sd_log_status.last_filename));
  sd_log_status.file_list_count = list_count;
  sd_log_status.error_code = SD_ERROR_NONE;

  return FR_OK;
}

static FRESULT SdLog_Mount(void)
{
  FRESULT result = f_mount(&sd_log_fs, "0:", 1U);

  if (result == FR_OK)
  {
    sd_log_status.fs_status = SD_LOG_FS_MOUNTED;
    sd_log_status.error_code = SD_ERROR_NONE;
    sd_log_status.log_status = SD_LOG_RECORD_IDLE;
  }
  else
  {
    sd_log_status.fs_status = SD_LOG_FS_NOT_MOUNTED;
    SdLog_SetError((uint16_t)result);
  }

  return result;
}

static FRESULT SdLog_CreateFile(void)
{
  FRESULT result;

  if (sd_log_status.fs_status != SD_LOG_FS_MOUNTED)
  {
    if (SdLog_Mount() != FR_OK)
    {
      return FR_NOT_READY;
    }
  }

  if (sd_log_status.log_status == SD_LOG_RECORD_RECORDING)
  {
    return FR_DENIED;
  }

  SdLog_UpdateNextFileId();

  if (sd_log_file.obj.fs != NULL)
  {
    (void)f_close(&sd_log_file);
    memset(&sd_log_file, 0, sizeof(sd_log_file));
  }

  sd_log_status.current_file_id++;
  snprintf(sd_log_status.current_filename,
           sizeof(sd_log_status.current_filename),
           "0:/LOG%04u.BIN",
           sd_log_status.current_file_id);

  result = f_open(&sd_log_file,
                  sd_log_status.current_filename,
                  FA_CREATE_ALWAYS | FA_WRITE);
  if (result == FR_OK)
  {
    sd_log_status.current_file_size = 0U;
    sd_log_status.current_write_count = 0U;
    sd_log_status.error_code = SD_ERROR_NONE;
    sd_log_status.log_status = SD_LOG_RECORD_IDLE;
    sd_log_write_frames = 0U;
    sd_log_unsynced_bytes = 0U;
  }
  else
  {
    SdLog_SetError((uint16_t)result);
  }

  return result;
}

static void SdLog_FlushWriteBuffer(uint8_t force)
{
  UINT written = 0U;
  UINT bytes_to_write;
  FRESULT result;

  if (sd_log_write_frames == 0U)
  {
    return;
  }

  if ((force == 0U) && (sd_log_write_frames < SD_LOG_WRITE_BLOCKS))
  {
    return;
  }

  bytes_to_write = (UINT)sd_log_write_frames * SD_LOG_BLOCK_SIZE;
  result = f_write(&sd_log_file, sd_log_write_buffer, bytes_to_write, &written);
  if ((result != FR_OK) || (written != bytes_to_write))
  {
    SdLog_SetError((uint16_t)result);
    sd_log_write_frames = 0U;
    return;
  }

  sd_log_status.current_file_size += written;
  sd_log_status.current_write_count += sd_log_write_frames;
  sd_log_unsynced_bytes += written;
  sd_log_write_frames = 0U;

  if ((force != 0U) || (sd_log_unsynced_bytes >= SD_LOG_SYNC_INTERVAL_BYTES))
  {
    result = f_sync(&sd_log_file);
    if (result != FR_OK)
    {
      SdLog_SetError((uint16_t)result);
    }
    else
    {
      sd_log_unsynced_bytes = 0U;
    }
  }
}

static void SdLog_RecordOneFrame(void)
{
  if (sd_log_status.log_status != SD_LOG_RECORD_RECORDING)
  {
    return;
  }

  SdLog_BuildFrame(sd_log_frame);
  memcpy(&sd_log_write_buffer[sd_log_write_frames * SD_LOG_BLOCK_SIZE],
         sd_log_frame,
         SD_LOG_BLOCK_SIZE);
  sd_log_write_frames++;

  SdLog_FlushWriteBuffer(0U);
}

static void SdLog_StartRecording(void)
{
  if (sd_log_status.log_status == SD_LOG_RECORD_RECORDING)
  {
    return;
  }

  if (sd_log_status.log_status == SD_LOG_RECORD_ERROR)
  {
    if (sd_log_file.obj.fs != NULL)
    {
      (void)f_close(&sd_log_file);
      memset(&sd_log_file, 0, sizeof(sd_log_file));
    }
    sd_log_status.log_status = SD_LOG_RECORD_IDLE;
  }

  if (sd_log_file.obj.fs == NULL)
  {
    if (SdLog_CreateFile() != FR_OK)
    {
      return;
    }
  }

  sd_log_pending_ticks = 0U;
  sd_log_unsynced_bytes = 0U;
  sd_log_status.log_status = SD_LOG_RECORD_RECORDING;
  sd_log_status.error_code = SD_ERROR_NONE;
  if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
  {
    SdLog_SetError(CMD_ERROR_START_FAILED);
  }
}

static void SdLog_StopRecording(void)
{
  if (sd_log_status.log_status != SD_LOG_RECORD_RECORDING)
  {
    return;
  }

  (void)HAL_TIM_Base_Stop_IT(&htim3);
  sd_log_status.log_status = SD_LOG_RECORD_STOPPING;

  SdLog_FlushWriteBuffer(1U);
  if (sd_log_status.log_status != SD_LOG_RECORD_ERROR)
  {
    if ((sd_log_unsynced_bytes != 0U) && (f_sync(&sd_log_file) != FR_OK))
    {
      SdLog_SetError(CMD_ERROR_STOP_FAILED);
    }
    else
    {
      sd_log_unsynced_bytes = 0U;
    }
  }

  if (sd_log_status.log_status != SD_LOG_RECORD_ERROR)
  {
    if (f_close(&sd_log_file) != FR_OK)
    {
      SdLog_SetError(CMD_ERROR_STOP_FAILED);
    }
    else
    {
      memset(&sd_log_file, 0, sizeof(sd_log_file));
    }
  }

  if (sd_log_status.log_status != SD_LOG_RECORD_ERROR)
  {
    strncpy(sd_log_status.last_filename,
            sd_log_status.current_filename,
            sizeof(sd_log_status.last_filename));
    sd_log_status.log_status = SD_LOG_RECORD_IDLE;
    SdLog_UpdateCapacity();
  }
}

static void SdLog_ResetDriver(void)
{
  (void)HAL_TIM_Base_Stop_IT(&htim3);
  sd_log_pending_ticks = 0U;
  sd_log_write_frames = 0U;
  sd_log_unsynced_bytes = 0U;

  if (sd_log_file.obj.fs != NULL)
  {
    (void)f_close(&sd_log_file);
    memset(&sd_log_file, 0, sizeof(sd_log_file));
  }

  (void)f_mount(NULL, "0:", 0U);
  (void)HAL_SD_DeInit(&hsd1);
  osDelay(SD_LOG_MOUNT_RETRY_DELAY_MS);
  MX_SDMMC1_SD_Init();
  osDelay(SD_LOG_MOUNT_RETRY_DELAY_MS);

  sd_log_status.fs_status = SD_LOG_FS_NOT_MOUNTED;
  sd_log_status.log_status = SD_LOG_RECORD_IDLE;
  sd_log_status.error_code = SD_ERROR_NONE;
  sd_log_status.current_file_size = 0U;
  sd_log_status.current_write_count = 0U;
  sd_log_status.current_filename[0] = '\0';

  (void)SdLog_Mount();
}

void SdLog_Init(void)
{
  sd_log_task_id = osThreadGetId();
  (void)SdLog_Mount();
}

void SdLog_RequestCreateFile(void)
{
  if (sd_log_task_id != NULL)
  {
    (void)osThreadFlagsSet(sd_log_task_id, SD_LOG_EVT_CREATE);
  }
}

void SdLog_RequestStart(void)
{
  if (sd_log_task_id != NULL)
  {
    (void)osThreadFlagsSet(sd_log_task_id, SD_LOG_EVT_START);
  }
}

void SdLog_RequestStop(void)
{
  if (sd_log_task_id != NULL)
  {
    (void)osThreadFlagsSet(sd_log_task_id, SD_LOG_EVT_STOP);
  }
}

void SdLog_RequestReset(void)
{
  if (sd_log_task_id != NULL)
  {
    (void)osThreadFlagsSet(sd_log_task_id, SD_LOG_EVT_RESET);
  }
}

void SdLog_RequestScanLog(void)
{
  if (sd_log_task_id != NULL)
  {
    (void)osThreadFlagsSet(sd_log_task_id, SD_LOG_EVT_SCAN);
  }
}

void SdLog_OnTimPeriodElapsed(TIM_HandleTypeDef *htim)
{
  if ((htim != NULL) && (htim->Instance == TIM3))
  {
    if ((sd_log_status.log_status == SD_LOG_RECORD_RECORDING) &&
        (sd_log_pending_ticks < SD_LOG_PENDING_TICK_LIMIT))
    {
      sd_log_pending_ticks++;
    }
    if ((sd_log_status.log_status == SD_LOG_RECORD_RECORDING) &&
        (sd_log_task_id != NULL))
    {
      (void)osThreadFlagsSet(sd_log_task_id, SD_LOG_EVT_TICK);
    }
  }
}

void SdLog_GetStatus(SdLogStatusSnapshot_t *status)
{
  if (status != NULL)
  {
    __disable_irq();
    *status = sd_log_status;
    __enable_irq();
  }
}

void StartSdTask(void *argument)
{
  (void)argument;
  osDelay(500U);
  SdLog_Init();

  for (;;)
  {
    uint32_t flags = osThreadFlagsWait(SD_LOG_EVT_CREATE |
                                       SD_LOG_EVT_START |
                                       SD_LOG_EVT_STOP |
                                       SD_LOG_EVT_TICK |
                                       SD_LOG_EVT_RESET |
                                       SD_LOG_EVT_SCAN,
                                       osFlagsWaitAny,
                                       osWaitForever);
    if ((flags & osFlagsError) != 0U)
    {
      continue;
    }

    if ((flags & SD_LOG_EVT_RESET) != 0U)
    {
      SdLog_ResetDriver();
      continue;
    }

    if ((flags & SD_LOG_EVT_SCAN) != 0U)
    {
      (void)SdLog_ScanLatestFile();
    }

    if ((flags & SD_LOG_EVT_CREATE) != 0U)
    {
      (void)SdLog_CreateFile();
    }

    if ((flags & SD_LOG_EVT_START) != 0U)
    {
      SdLog_StartRecording();
    }

    if ((flags & SD_LOG_EVT_STOP) != 0U)
    {
      SdLog_StopRecording();
    }

    if ((flags & SD_LOG_EVT_TICK) != 0U)
    {
      uint32_t processed_ticks = 0U;

      for (;;)
      {
        uint32_t pending_flags = osThreadFlagsGet();

        if ((pending_flags & SD_LOG_EVT_STOP) != 0U)
        {
          (void)osThreadFlagsClear(SD_LOG_EVT_STOP);
          SdLog_StopRecording();
          break;
        }

        __disable_irq();
        if (sd_log_pending_ticks == 0U)
        {
          __enable_irq();
          break;
        }
        sd_log_pending_ticks--;
        __enable_irq();

        SdLog_RecordOneFrame();
        processed_ticks++;
        if (processed_ticks >= SD_LOG_TICK_BATCH_LIMIT)
        {
          if ((sd_log_pending_ticks != 0U) && (sd_log_task_id != NULL))
          {
            (void)osThreadFlagsSet(sd_log_task_id, SD_LOG_EVT_TICK);
          }
          break;
        }
      }
    }
  }
}
