#include "sd_log.h"

#include "cmsis_os2.h"
#include "ff.h"
#include "sdmmc.h"

#include <stdio.h>
#include <string.h>

#define SDLOG_VOLUME_PATH          "0:"
#define SDLOG_ROOT_PATH            "0:/"
#define SDLOG_FILE_PREFIX          "LOG"
#define SDLOG_FILE_SUFFIX          ".BIN"
#define SDLOG_FILE_ID_DIGITS       4U
#define SDLOG_MAX_PATH_BYTES       260U
#define SDLOG_FLUSH_THRESHOLD      SDLOG_WRITE_BUFFER_SIZE

#define SDLOG_EVT_WAKE             (1UL << 0)
#define SDLOG_EVT_STOP_DONE        (1UL << 1)
#define SDLOG_EVT_SYNC_DONE        (1UL << 2)

typedef struct
{
  uint32_t len;
  uint8_t data[SDLOG_FRAME_SIZE];
} SdLogFrameMsg_t;

typedef struct
{
  uint64_t committed_size;
  uint32_t write_count;
  uint16_t current_file_id;
  char current_filename[SDLOG_NAME_BYTES];
  char last_filename[SDLOG_NAME_BYTES];
} SdLogRuntime_t;

static osThreadId_t sdlog_task_id;
static osMessageQueueId_t sdlog_queue_id;
static osMutexId_t sdlog_mutex_id;
static osEventFlagsId_t sdlog_event_id;

static FATFS sdlog_fs;
static FIL sdlog_file;

static volatile uint8_t sdlog_initialized;
static volatile uint8_t sdlog_fs_mounted;
static volatile uint8_t sdlog_recording;
static volatile uint8_t sdlog_stopping;
static volatile uint8_t sdlog_sync_pending;
static volatile uint8_t sdlog_stop_pending;
static volatile uint8_t sdlog_file_open;
static volatile SdLogResult_t sdlog_last_error = SDLOG_OK;
static volatile uint16_t sdlog_last_fresult = FR_OK;

static SdLogRuntime_t sdlog_rt;
static SdLogStatusSnapshot_t sdlog_snapshot;

static __ALIGNED(4) uint8_t sdlog_write_buffer[SDLOG_WRITE_BUFFER_SIZE];
static uint32_t sdlog_write_buffer_used;

static const osThreadAttr_t sdlog_task_attr =
{
  .name = "sdlog_task",
  .priority = osPriorityLow,
  .stack_size = 4096U
};

static const osMessageQueueAttr_t sdlog_queue_attr =
{
  .name = "sdlog_queue"
};

static const osMutexAttr_t sdlog_mutex_attr =
{
  .name = "sdlog_mutex"
};

static const osEventFlagsAttr_t sdlog_event_attr =
{
  .name = "sdlog_event"
};

static void SDLog_Task(void *argument);
static int SDLog_Lock(uint32_t timeout_ms);
static void SDLog_Unlock(void);
static void SDLog_SetError(SdLogResult_t error_code, FRESULT fr);
static void SDLog_ClearError(void);
static int SDLog_EnsureInitialized(void);
static int SDLog_EnsureMounted(void);
static int SDLog_BuildPath(char *path, size_t path_size, const char *filename);
static int SDLog_ParseLogFileId(const char *filename, uint16_t *file_id);
static int SDLog_ScanMaxFileIdLocked(uint16_t *max_file_id);
static int SDLog_OpenNextFileLocked(void);
static int SDLog_FlushBufferLocked(int force_sync);
static int SDLog_CloseFileLocked(int sync_first);
static int SDLog_RefreshCapacityLocked(void);
static uint64_t SDLog_CurrentLogicalSizeLocked(void);
static int SDLog_RefreshSnapshotLocked(void);
static int SDLog_ListFilesLocked(void);

static void SDLog_SetError(SdLogResult_t error_code, FRESULT fr)
{
  sdlog_last_error = error_code;
  sdlog_last_fresult = (uint16_t)fr;
  sdlog_snapshot.error_code = (uint16_t)((error_code < 0) ? (uint16_t)(-error_code) : (uint16_t)error_code);
  sdlog_snapshot.log_status = SD_LOG_RECORD_ERROR;
}

static void SDLog_ClearError(void)
{
  sdlog_last_error = SDLOG_OK;
  sdlog_last_fresult = FR_OK;
  sdlog_snapshot.error_code = 0U;
  if (sdlog_snapshot.log_status == SD_LOG_RECORD_ERROR)
  {
    sdlog_snapshot.log_status = sdlog_recording ? SD_LOG_RECORD_RECORDING : SD_LOG_RECORD_IDLE;
  }
}

static int SDLog_Lock(uint32_t timeout_ms)
{
  if (sdlog_mutex_id == NULL)
  {
    return SDLOG_ERR_NOT_INITIALIZED;
  }

  if (osMutexAcquire(sdlog_mutex_id, timeout_ms) != osOK)
  {
    return SDLOG_ERR_BUSY;
  }

  return SDLOG_OK;
}

static void SDLog_Unlock(void)
{
  if (sdlog_mutex_id != NULL)
  {
    (void)osMutexRelease(sdlog_mutex_id);
  }
}

static int SDLog_EnsureInitialized(void)
{
  return (sdlog_initialized != 0U) ? SDLOG_OK : SDLOG_ERR_NOT_INITIALIZED;
}

static int SDLog_BuildPath(char *path, size_t path_size, const char *filename)
{
  size_t len;

  if ((path == NULL) || (filename == NULL) || (path_size < 4U))
  {
    return SDLOG_ERR_PARAM;
  }

  if ((filename[0] == '0') && (filename[1] == ':'))
  {
    len = strlen(filename);
    if ((len + 1U) > path_size)
    {
      return SDLOG_ERR_PARAM;
    }
    memcpy(path, filename, len + 1U);
    return SDLOG_OK;
  }

  if (snprintf(path, path_size, SDLOG_VOLUME_PATH "/%s", filename) >= (int)path_size)
  {
    return SDLOG_ERR_PARAM;
  }

  return SDLOG_OK;
}

static int SDLog_ParseLogFileId(const char *filename, uint16_t *file_id)
{
  const char *name;
  uint32_t value = 0U;
  uint32_t index;

  if ((filename == NULL) || (file_id == NULL))
  {
    return SDLOG_ERR_PARAM;
  }

  name = filename;
  if ((name[0] == '0') && (name[1] == ':'))
  {
    name += 2;
    if (name[0] == '/')
    {
      name++;
    }
  }

  if ((strncmp(name, SDLOG_FILE_PREFIX, 3U) != 0) && (strncmp(name, "log", 3U) != 0))
  {
    return SDLOG_ERR_PARAM;
  }

  for (index = 3U; index < (3U + SDLOG_FILE_ID_DIGITS); index++)
  {
    char c = name[index];
    if ((c < '0') || (c > '9'))
    {
      return SDLOG_ERR_PARAM;
    }
    value = (value * 10U) + (uint32_t)(c - '0');
  }

  if (!(((name[7] == '.') || (name[7] == '.')) &&
        ((name[8] == 'B') || (name[8] == 'b')) &&
        ((name[9] == 'I') || (name[9] == 'i')) &&
        ((name[10] == 'N') || (name[10] == 'n')) &&
        (name[11] == '\0')))
  {
    return SDLOG_ERR_PARAM;
  }

  if (value > 0xFFFFU)
  {
    return SDLOG_ERR_NO_SPACE;
  }

  *file_id = (uint16_t)value;
  return SDLOG_OK;
}

static uint64_t SDLog_CurrentLogicalSizeLocked(void)
{
  uint64_t queued_bytes = (uint64_t)osMessageQueueGetCount(sdlog_queue_id) * (uint64_t)SDLOG_FRAME_SIZE;
  return sdlog_rt.committed_size + (uint64_t)sdlog_write_buffer_used + queued_bytes;
}

static int SDLog_RefreshCapacityLocked(void)
{
  FATFS *fs;
  DWORD free_clusters;
  FRESULT fr;
  uint64_t total_sectors;
  uint64_t free_sectors;

  fr = f_getfree(SDLOG_VOLUME_PATH, &free_clusters, &fs);
  if (fr != FR_OK)
  {
    SDLog_SetError(SDLOG_ERR_MOUNT, fr);
    return SDLOG_ERR_MOUNT;
  }

  total_sectors = (uint64_t)(fs->n_fatent - 2U) * (uint64_t)fs->csize;
  free_sectors = (uint64_t)free_clusters * (uint64_t)fs->csize;
  sdlog_snapshot.total_size_mb = (uint32_t)(total_sectors / 2048U);
  sdlog_snapshot.free_size_mb = (uint32_t)(free_sectors / 2048U);
  sdlog_snapshot.used_size_mb = sdlog_snapshot.total_size_mb - sdlog_snapshot.free_size_mb;
  return SDLOG_OK;
}

static int SDLog_RefreshSnapshotLocked(void)
{
  sdlog_snapshot.fs_status = sdlog_fs_mounted ? SD_LOG_FS_MOUNTED : SD_LOG_FS_NOT_MOUNTED;
  sdlog_snapshot.log_status = sdlog_recording ? SD_LOG_RECORD_RECORDING : SD_LOG_RECORD_IDLE;
  sdlog_snapshot.current_file_id = sdlog_rt.current_file_id;
  sdlog_snapshot.current_file_size = SDLog_CurrentLogicalSizeLocked();
  sdlog_snapshot.current_write_count = (uint32_t)(sdlog_snapshot.current_file_size / SDLOG_FRAME_SIZE);
  strncpy(sdlog_snapshot.current_filename, sdlog_rt.current_filename, sizeof(sdlog_snapshot.current_filename) - 1U);
  sdlog_snapshot.current_filename[sizeof(sdlog_snapshot.current_filename) - 1U] = '\0';
  strncpy(sdlog_snapshot.last_filename, sdlog_rt.last_filename, sizeof(sdlog_snapshot.last_filename) - 1U);
  sdlog_snapshot.last_filename[sizeof(sdlog_snapshot.last_filename) - 1U] = '\0';
  return SDLOG_OK;
}

static int SDLog_ScanMaxFileIdLocked(uint16_t *max_file_id)
{
  DIR dir;
  FILINFO info;
  FRESULT fr;
  uint16_t file_id;
  uint16_t max_id = 0U;

  if (max_file_id == NULL)
  {
    return SDLOG_ERR_PARAM;
  }

  fr = f_opendir(&dir, SDLOG_ROOT_PATH);
  if (fr != FR_OK)
  {
    SDLog_SetError(SDLOG_ERR_MOUNT, fr);
    return SDLOG_ERR_MOUNT;
  }

  for (;;)
  {
    fr = f_readdir(&dir, &info);
    if (fr != FR_OK)
    {
      (void)f_closedir(&dir);
      SDLog_SetError(SDLOG_ERR_MOUNT, fr);
      return SDLOG_ERR_MOUNT;
    }

    if (info.fname[0] == '\0')
    {
      break;
    }

    if (SDLog_ParseLogFileId(info.fname, &file_id) == SDLOG_OK)
    {
      if (file_id > max_id)
      {
        max_id = file_id;
      }
    }
  }

  (void)f_closedir(&dir);
  *max_file_id = max_id;
  return SDLOG_OK;
}

static int SDLog_ListFilesLocked(void)
{
  DIR dir;
  FILINFO info;
  FRESULT fr;
  uint16_t file_id;
  uint32_t count = 0U;

  memset(sdlog_snapshot.file_list_names, 0, sizeof(sdlog_snapshot.file_list_names));
  memset(sdlog_snapshot.file_list_sizes, 0, sizeof(sdlog_snapshot.file_list_sizes));
  sdlog_snapshot.file_list_count = 0U;

  fr = f_opendir(&dir, SDLOG_ROOT_PATH);
  if (fr != FR_OK)
  {
    SDLog_SetError(SDLOG_ERR_MOUNT, fr);
    return SDLOG_ERR_MOUNT;
  }

  for (;;)
  {
    fr = f_readdir(&dir, &info);
    if (fr != FR_OK)
    {
      (void)f_closedir(&dir);
      SDLog_SetError(SDLOG_ERR_MOUNT, fr);
      return SDLOG_ERR_MOUNT;
    }

    if (info.fname[0] == '\0')
    {
      break;
    }

    if ((SDLog_ParseLogFileId(info.fname, &file_id) == SDLOG_OK) && (count < SDLOG_FILE_LIST_MAX))
    {
      strncpy(sdlog_snapshot.file_list_names[count], info.fname, sizeof(sdlog_snapshot.file_list_names[count]) - 1U);
      sdlog_snapshot.file_list_names[count][sizeof(sdlog_snapshot.file_list_names[count]) - 1U] = '\0';
      if ((sdlog_recording != 0U) && (strcmp(info.fname, sdlog_rt.current_filename) == 0))
      {
        sdlog_snapshot.file_list_sizes[count] = SDLog_CurrentLogicalSizeLocked();
      }
      else
      {
        sdlog_snapshot.file_list_sizes[count] = (uint64_t)info.fsize;
      }
      count++;
    }
  }

  (void)f_closedir(&dir);
  sdlog_snapshot.file_list_count = (uint16_t)count;
  return SDLOG_OK;
}

static int SDLog_EnsureMounted(void)
{
  FRESULT fr;

  if (sdlog_fs_mounted != 0U)
  {
    return SDLOG_OK;
  }

  fr = f_mount(&sdlog_fs, SDLOG_VOLUME_PATH, 1U);
  if (fr != FR_OK)
  {
    sdlog_fs_mounted = 0U;
    SDLog_SetError(SDLOG_ERR_MOUNT, fr);
    return SDLOG_ERR_MOUNT;
  }

  sdlog_fs_mounted = 1U;
  sdlog_snapshot.fs_status = SD_LOG_FS_MOUNTED;
  SDLog_ClearError();
  return SDLOG_OK;
}

static int SDLog_OpenNextFileLocked(void)
{
  uint16_t max_file_id;
  FRESULT fr;
  char path[SDLOG_MAX_PATH_BYTES];

  if (SDLog_ScanMaxFileIdLocked(&max_file_id) != SDLOG_OK)
  {
    return sdlog_last_error;
  }

  if (max_file_id >= 9999U)
  {
    SDLog_SetError(SDLOG_ERR_NO_SPACE, FR_DENIED);
    return SDLOG_ERR_NO_SPACE;
  }

  sdlog_rt.current_file_id = (uint16_t)(max_file_id + 1U);
  if (snprintf(sdlog_rt.current_filename,
               sizeof(sdlog_rt.current_filename),
               SDLOG_FILE_PREFIX "%04u" SDLOG_FILE_SUFFIX,
               sdlog_rt.current_file_id) >= (int)sizeof(sdlog_rt.current_filename))
  {
    SDLog_SetError(SDLOG_ERR_INTERNAL, FR_INVALID_NAME);
    return SDLOG_ERR_INTERNAL;
  }

  if (SDLog_BuildPath(path, sizeof(path), sdlog_rt.current_filename) != SDLOG_OK)
  {
    SDLog_SetError(SDLOG_ERR_INTERNAL, FR_INVALID_NAME);
    return SDLOG_ERR_INTERNAL;
  }

  fr = f_open(&sdlog_file, path, FA_CREATE_ALWAYS | FA_WRITE);
  if (fr != FR_OK)
  {
    SDLog_SetError(SDLOG_ERR_OPEN, fr);
    return SDLOG_ERR_OPEN;
  }

  sdlog_file_open = 1U;
  sdlog_rt.committed_size = 0U;
  sdlog_rt.write_count = 0U;
  sdlog_write_buffer_used = 0U;
  sdlog_snapshot.current_file_id = sdlog_rt.current_file_id;
  strncpy(sdlog_snapshot.current_filename, sdlog_rt.current_filename, sizeof(sdlog_snapshot.current_filename) - 1U);
  sdlog_snapshot.current_filename[sizeof(sdlog_snapshot.current_filename) - 1U] = '\0';
  sdlog_snapshot.current_file_size = 0U;
  sdlog_snapshot.current_write_count = 0U;
  SDLog_ClearError();
  return SDLOG_OK;
}

static int SDLog_FlushBufferLocked(int force_sync)
{
  FRESULT fr;
  UINT written = 0U;
  UINT to_write;

  if ((sdlog_file_open == 0U) || (sdlog_write_buffer_used == 0U))
  {
    if ((force_sync != 0) && (sdlog_file_open != 0U))
    {
      fr = f_sync(&sdlog_file);
      if (fr != FR_OK)
      {
        SDLog_SetError(SDLOG_ERR_SYNC, fr);
        return SDLOG_ERR_SYNC;
      }
    }
    return SDLOG_OK;
  }

  to_write = (UINT)sdlog_write_buffer_used;
  fr = f_write(&sdlog_file, sdlog_write_buffer, to_write, &written);
  if ((fr != FR_OK) || (written != to_write))
  {
    SDLog_SetError(SDLOG_ERR_WRITE, (fr != FR_OK) ? fr : FR_DISK_ERR);
    return SDLOG_ERR_WRITE;
  }

  sdlog_rt.committed_size += (uint64_t)written;
  sdlog_rt.write_count += (uint32_t)(written / SDLOG_FRAME_SIZE);
  sdlog_write_buffer_used = 0U;
  sdlog_snapshot.current_file_size = sdlog_rt.committed_size;
  sdlog_snapshot.current_write_count = sdlog_rt.write_count;

  if (force_sync != 0)
  {
    fr = f_sync(&sdlog_file);
    if (fr != FR_OK)
    {
      SDLog_SetError(SDLOG_ERR_SYNC, fr);
      return SDLOG_ERR_SYNC;
    }
  }

  SDLog_ClearError();
  return SDLOG_OK;
}

static int SDLog_CloseFileLocked(int sync_first)
{
  FRESULT fr;

  if (sdlog_file_open == 0U)
  {
    return SDLOG_OK;
  }

  if (SDLog_FlushBufferLocked(sync_first) != SDLOG_OK)
  {
    return sdlog_last_error;
  }

  fr = f_close(&sdlog_file);
  if (fr != FR_OK)
  {
    SDLog_SetError(SDLOG_ERR_CLOSE, fr);
    return SDLOG_ERR_CLOSE;
  }

  sdlog_file_open = 0U;
  sdlog_rt.committed_size = 0U;
  sdlog_write_buffer_used = 0U;
  SDLog_ClearError();
  return SDLOG_OK;
}

static void SDLog_Task(void *argument)
{
  SdLogFrameMsg_t frame_msg;
  uint32_t flags;

  (void)argument;

  for (;;)
  {
    flags = osThreadFlagsWait(SDLOG_EVT_WAKE, osFlagsWaitAny, osWaitForever);
    if ((flags & osFlagsError) != 0U)
    {
      continue;
    }

    if (SDLog_Lock(osWaitForever) != SDLOG_OK)
    {
      continue;
    }

    while (osMessageQueueGet(sdlog_queue_id, &frame_msg, NULL, 0U) == osOK)
    {
      if ((sdlog_recording == 0U) || (sdlog_file_open == 0U))
      {
        continue;
      }

      if ((sdlog_write_buffer_used + frame_msg.len) > SDLOG_WRITE_BUFFER_SIZE)
      {
        if (SDLog_FlushBufferLocked(0) != SDLOG_OK)
        {
          break;
        }
      }

      if ((sdlog_write_buffer_used + frame_msg.len) > SDLOG_WRITE_BUFFER_SIZE)
      {
        SDLog_SetError(SDLOG_ERR_INTERNAL, FR_INT_ERR);
        break;
      }

      memcpy(&sdlog_write_buffer[sdlog_write_buffer_used], frame_msg.data, frame_msg.len);
      sdlog_write_buffer_used += frame_msg.len;

      if (sdlog_write_buffer_used >= SDLOG_FLUSH_THRESHOLD)
      {
        if (SDLog_FlushBufferLocked(0) != SDLOG_OK)
        {
          break;
        }
      }
    }

    if (sdlog_recording != 0U)
    {
      if ((sdlog_sync_pending != 0U) && (osMessageQueueGetCount(sdlog_queue_id) == 0U))
      {
        if (SDLog_FlushBufferLocked(1) == SDLOG_OK)
        {
          sdlog_sync_pending = 0U;
          (void)osEventFlagsSet(sdlog_event_id, SDLOG_EVT_SYNC_DONE);
        }
      }

      if ((sdlog_stop_pending != 0U) && (osMessageQueueGetCount(sdlog_queue_id) == 0U))
      {
        if (SDLog_CloseFileLocked(1) == SDLOG_OK)
        {
          sdlog_recording = 0U;
          sdlog_stopping = 0U;
          sdlog_stop_pending = 0U;
          strncpy(sdlog_rt.last_filename, sdlog_rt.current_filename, sizeof(sdlog_rt.last_filename) - 1U);
          sdlog_rt.last_filename[sizeof(sdlog_rt.last_filename) - 1U] = '\0';
          strncpy(sdlog_snapshot.last_filename, sdlog_rt.last_filename, sizeof(sdlog_snapshot.last_filename) - 1U);
          sdlog_snapshot.last_filename[sizeof(sdlog_snapshot.last_filename) - 1U] = '\0';
          sdlog_snapshot.log_status = SD_LOG_RECORD_IDLE;
          (void)osEventFlagsSet(sdlog_event_id, SDLOG_EVT_STOP_DONE);
        }
      }
    }

    SDLog_Unlock();
  }
}

int SDLog_Init(void)
{
  if (sdlog_initialized != 0U)
  {
    return SDLOG_OK;
  }

  memset(&sdlog_fs, 0, sizeof(sdlog_fs));
  memset(&sdlog_file, 0, sizeof(sdlog_file));
  memset(&sdlog_rt, 0, sizeof(sdlog_rt));
  memset(&sdlog_snapshot, 0, sizeof(sdlog_snapshot));
  sdlog_snapshot.fs_status = SD_LOG_FS_NOT_MOUNTED;
  sdlog_snapshot.log_status = SD_LOG_RECORD_IDLE;

  sdlog_mutex_id = osMutexNew(&sdlog_mutex_attr);
  sdlog_queue_id = osMessageQueueNew(SDLOG_QUEUE_LENGTH, sizeof(SdLogFrameMsg_t), &sdlog_queue_attr);
  sdlog_event_id = osEventFlagsNew(&sdlog_event_attr);
  sdlog_task_id = osThreadNew(SDLog_Task, NULL, &sdlog_task_attr);

  if ((sdlog_mutex_id == NULL) || (sdlog_queue_id == NULL) || (sdlog_event_id == NULL) || (sdlog_task_id == NULL))
  {
    return SDLOG_ERR_INTERNAL;
  }

  sdlog_initialized = 1U;
  SDLog_ClearError();
  return SDLOG_OK;
}

int SDLog_Mount(void)
{
  FRESULT fr;
  int result;

  result = SDLog_EnsureInitialized();
  if (result != SDLOG_OK)
  {
    return result;
  }

  result = SDLog_Lock(osWaitForever);
  if (result != SDLOG_OK)
  {
    return result;
  }

  if (sdlog_recording != 0U)
  {
    SDLog_Unlock();
    return SDLOG_ERR_BUSY;
  }

  fr = f_mount(&sdlog_fs, SDLOG_VOLUME_PATH, 1U);
  if (fr != FR_OK)
  {
    sdlog_fs_mounted = 0U;
    SDLog_SetError(SDLOG_ERR_MOUNT, fr);
    SDLog_Unlock();
    return SDLOG_ERR_MOUNT;
  }

  sdlog_fs_mounted = 1U;
  sdlog_snapshot.fs_status = SD_LOG_FS_MOUNTED;
  (void)SDLog_RefreshCapacityLocked();
  SDLog_ClearError();
  SDLog_Unlock();
  return SDLOG_OK;
}

int SDLog_CreateNewFile(void)
{
  int result;

  result = SDLog_EnsureInitialized();
  if (result != SDLOG_OK)
  {
    return result;
  }

  result = SDLog_Lock(osWaitForever);
  if (result != SDLOG_OK)
  {
    return result;
  }

  if (sdlog_recording != 0U)
  {
    SDLog_Unlock();
    return SDLOG_ERR_BUSY;
  }

  result = SDLog_EnsureMounted();
  if (result != SDLOG_OK)
  {
    SDLog_Unlock();
    return result;
  }

  (void)SDLog_CloseFileLocked(1);
  sdlog_write_buffer_used = 0U;

  result = SDLog_OpenNextFileLocked();
  if (result != SDLOG_OK)
  {
    SDLog_Unlock();
    return result;
  }

  sdlog_snapshot.log_status = SD_LOG_RECORD_IDLE;
  (void)SDLog_RefreshCapacityLocked();
  SDLog_Unlock();
  return SDLOG_OK;
}

int SDLog_Start(void)
{
  int result;

  result = SDLog_CreateNewFile();
  if (result != SDLOG_OK)
  {
    return result;
  }

  result = SDLog_Lock(osWaitForever);
  if (result != SDLOG_OK)
  {
    return result;
  }

  sdlog_recording = 1U;
  sdlog_stopping = 0U;
  sdlog_sync_pending = 0U;
  sdlog_stop_pending = 0U;
  sdlog_snapshot.log_status = SD_LOG_RECORD_RECORDING;
  sdlog_snapshot.error_code = 0U;
  (void)osEventFlagsClear(sdlog_event_id, SDLOG_EVT_STOP_DONE | SDLOG_EVT_SYNC_DONE);
  SDLog_Unlock();
  return SDLOG_OK;
}

int SDLog_Stop(void)
{
  uint32_t flags;
  int result;

  result = SDLog_EnsureInitialized();
  if (result != SDLOG_OK)
  {
    return result;
  }

  result = SDLog_Lock(osWaitForever);
  if (result != SDLOG_OK)
  {
    return result;
  }

  if (sdlog_recording == 0U)
  {
    SDLog_Unlock();
    return SDLOG_ERR_NOT_RECORDING;
  }

  sdlog_stopping = 1U;
  sdlog_stop_pending = 1U;
  sdlog_snapshot.log_status = SD_LOG_RECORD_STOPPING;
  SDLog_Unlock();

  (void)osThreadFlagsSet(sdlog_task_id, SDLOG_EVT_WAKE);
  flags = osEventFlagsWait(sdlog_event_id, SDLOG_EVT_STOP_DONE, osFlagsWaitAny, 5000U);
  if ((flags & osFlagsError) != 0U)
  {
    return SDLOG_ERR_TIMEOUT;
  }

  return (sdlog_last_error == SDLOG_OK) ? SDLOG_OK : sdlog_last_error;
}

int SDLog_WriteFrame(const uint8_t *data, uint32_t len)
{
  SdLogFrameMsg_t msg;
  int result;

  if ((data == NULL) || (len != SDLOG_FRAME_SIZE))
  {
    return SDLOG_ERR_PARAM;
  }

  result = SDLog_EnsureInitialized();
  if (result != SDLOG_OK)
  {
    return result;
  }

  result = SDLog_Lock(osWaitForever);
  if (result != SDLOG_OK)
  {
    return result;
  }

  if ((sdlog_recording == 0U) || (sdlog_stopping != 0U))
  {
    SDLog_Unlock();
    return SDLOG_ERR_NOT_RECORDING;
  }

  msg.len = len;
  memcpy(msg.data, data, len);
  SDLog_Unlock();

  if (osMessageQueuePut(sdlog_queue_id, &msg, 0U, 0U) != osOK)
  {
    return SDLOG_ERR_QUEUE_FULL;
  }

  (void)osThreadFlagsSet(sdlog_task_id, SDLOG_EVT_WAKE);
  return SDLOG_OK;
}

int SDLog_Sync(void)
{
  uint32_t flags;
  int result;

  result = SDLog_EnsureInitialized();
  if (result != SDLOG_OK)
  {
    return result;
  }

  result = SDLog_Lock(osWaitForever);
  if (result != SDLOG_OK)
  {
    return result;
  }

  if ((sdlog_recording == 0U) || (sdlog_file_open == 0U))
  {
    SDLog_Unlock();
    return SDLOG_ERR_NOT_RECORDING;
  }

  sdlog_sync_pending = 1U;
  SDLog_Unlock();

  (void)osThreadFlagsSet(sdlog_task_id, SDLOG_EVT_WAKE);
  flags = osEventFlagsWait(sdlog_event_id, SDLOG_EVT_SYNC_DONE, osFlagsWaitAny, 5000U);
  if ((flags & osFlagsError) != 0U)
  {
    return SDLOG_ERR_TIMEOUT;
  }

  return (sdlog_last_error == SDLOG_OK) ? SDLOG_OK : sdlog_last_error;
}

int SDLog_GetCurrentFileSize(uint64_t *size)
{
  int result;

  if (size == NULL)
  {
    return SDLOG_ERR_PARAM;
  }

  result = SDLog_EnsureInitialized();
  if (result != SDLOG_OK)
  {
    return result;
  }

  result = SDLog_Lock(osWaitForever);
  if (result != SDLOG_OK)
  {
    return result;
  }

  *size = SDLog_CurrentLogicalSizeLocked();
  SDLog_Unlock();
  return SDLOG_OK;
}

int SDLog_ListFiles(void)
{
  int result;

  result = SDLog_EnsureInitialized();
  if (result != SDLOG_OK)
  {
    return result;
  }

  result = SDLog_Lock(osWaitForever);
  if (result != SDLOG_OK)
  {
    return result;
  }

  result = SDLog_EnsureMounted();
  if (result == SDLOG_OK)
  {
    result = SDLog_ListFilesLocked();
    if (result == SDLOG_OK)
    {
      (void)SDLog_RefreshCapacityLocked();
      (void)SDLog_RefreshSnapshotLocked();
    }
  }

  SDLog_Unlock();
  return result;
}

int SDLog_GetFileInfo(const char *filename, uint64_t *size)
{
  FILINFO info;
  char path[SDLOG_MAX_PATH_BYTES];
  int result;

  if ((filename == NULL) || (size == NULL))
  {
    return SDLOG_ERR_PARAM;
  }

  result = SDLog_EnsureInitialized();
  if (result != SDLOG_OK)
  {
    return result;
  }

  result = SDLog_Lock(osWaitForever);
  if (result != SDLOG_OK)
  {
    return result;
  }

  result = SDLog_EnsureMounted();
  if (result != SDLOG_OK)
  {
    SDLog_Unlock();
    return result;
  }

  result = SDLog_BuildPath(path, sizeof(path), filename);
  if (result != SDLOG_OK)
  {
    SDLog_Unlock();
    return result;
  }

  if ((sdlog_recording != 0U) && (strcmp(filename, sdlog_rt.current_filename) == 0))
  {
    *size = SDLog_CurrentLogicalSizeLocked();
    SDLog_Unlock();
    return SDLOG_OK;
  }

  if (f_stat(path, &info) != FR_OK)
  {
    SDLog_Unlock();
    return SDLOG_ERR_OPEN;
  }

  *size = (uint64_t)info.fsize;
  SDLog_Unlock();
  return SDLOG_OK;
}

int SDLog_ReadFile(const char *filename, uint64_t offset, uint8_t *buf, uint32_t len, uint32_t *read_len)
{
  FIL fil;
  char path[SDLOG_MAX_PATH_BYTES];
  FRESULT fr;
  FSIZE_t seek_pos;
  FSIZE_t file_size;
  UINT total_read = 0U;
  UINT chunk;
  int result;

  if ((filename == NULL) || (buf == NULL) || (read_len == NULL))
  {
    return SDLOG_ERR_PARAM;
  }

  *read_len = 0U;

  result = SDLog_EnsureInitialized();
  if (result != SDLOG_OK)
  {
    return result;
  }

  result = SDLog_Lock(osWaitForever);
  if (result != SDLOG_OK)
  {
    return result;
  }

  result = SDLog_EnsureMounted();
  if (result != SDLOG_OK)
  {
    SDLog_Unlock();
    return result;
  }

  result = SDLog_BuildPath(path, sizeof(path), filename);
  if (result != SDLOG_OK)
  {
    SDLog_Unlock();
    return result;
  }

  if ((sdlog_recording != 0U) && (strcmp(filename, sdlog_rt.current_filename) == 0))
  {
    SDLog_Unlock();
    return SDLOG_ERR_BUSY;
  }

  fr = f_open(&fil, path, FA_READ);
  if (fr != FR_OK)
  {
    SDLog_Unlock();
    return SDLOG_ERR_OPEN;
  }

  file_size = f_size(&fil);
  if (offset >= (uint64_t)file_size)
  {
    (void)f_close(&fil);
    SDLog_Unlock();
    return SDLOG_OK;
  }

  seek_pos = (FSIZE_t)offset;
  if (f_lseek(&fil, seek_pos) != FR_OK)
  {
    (void)f_close(&fil);
    SDLog_Unlock();
    return SDLOG_ERR_OPEN;
  }

  while ((len > 0U) && (fr == FR_OK))
  {
    chunk = len;
    fr = f_read(&fil, buf + total_read, chunk, &chunk);
    if (fr != FR_OK)
    {
      (void)f_close(&fil);
      SDLog_Unlock();
      return SDLOG_ERR_WRITE;
    }

    total_read += chunk;
    if (chunk == 0U)
    {
      break;
    }
    len -= chunk;
  }

  (void)f_close(&fil);
  *read_len = total_read;
  SDLog_Unlock();
  return SDLOG_OK;
}

int SDLog_GetStatusSnapshot(SdLogStatusSnapshot_t *status)
{
  int result;

  if (status == NULL)
  {
    return SDLOG_ERR_PARAM;
  }

  result = SDLog_EnsureInitialized();
  if (result != SDLOG_OK)
  {
    return result;
  }

  result = SDLog_Lock(osWaitForever);
  if (result != SDLOG_OK)
  {
    return result;
  }

  sdlog_snapshot.current_file_size = SDLog_CurrentLogicalSizeLocked();
  sdlog_snapshot.current_write_count = (uint32_t)(sdlog_snapshot.current_file_size / SDLOG_FRAME_SIZE);
  sdlog_snapshot.fs_status = sdlog_fs_mounted ? SD_LOG_FS_MOUNTED : SD_LOG_FS_NOT_MOUNTED;
  sdlog_snapshot.log_status = sdlog_recording ? SD_LOG_RECORD_RECORDING : SD_LOG_RECORD_IDLE;
  *status = sdlog_snapshot;
  SDLog_Unlock();
  return SDLOG_OK;
}

int SDLog_GetLastFResult(void)
{
  return (int)sdlog_last_fresult;
}

void SdLog_Init(void)
{
  (void)SDLog_Init();
}

void SdLog_RequestCreateFile(void)
{
  (void)SDLog_CreateNewFile();
}

void SdLog_RequestStart(void)
{
  (void)SDLog_Start();
}

void SdLog_RequestStop(void)
{
  (void)SDLog_Stop();
}

void SdLog_RequestReset(void)
{
  int result;

  result = SDLog_EnsureInitialized();
  if (result != SDLOG_OK)
  {
    return;
  }

  result = SDLog_Lock(osWaitForever);
  if (result != SDLOG_OK)
  {
    return;
  }

  (void)SDLog_CloseFileLocked(1);
  (void)f_mount(NULL, SDLOG_VOLUME_PATH, 0U);
  sdlog_fs_mounted = 0U;
  sdlog_recording = 0U;
  sdlog_stopping = 0U;
  sdlog_sync_pending = 0U;
  sdlog_stop_pending = 0U;
  sdlog_snapshot.fs_status = SD_LOG_FS_NOT_MOUNTED;
  sdlog_snapshot.log_status = SD_LOG_RECORD_IDLE;
  sdlog_rt.committed_size = 0U;
  sdlog_rt.write_count = 0U;
  sdlog_rt.current_file_id = 0U;
  sdlog_rt.current_filename[0] = '\0';
  sdlog_rt.last_filename[0] = '\0';
  sdlog_snapshot.current_filename[0] = '\0';
  sdlog_snapshot.last_filename[0] = '\0';
  sdlog_snapshot.current_file_id = 0U;
  sdlog_snapshot.current_file_size = 0U;
  sdlog_snapshot.current_write_count = 0U;
  sdlog_write_buffer_used = 0U;
  (void)osMessageQueueReset(sdlog_queue_id);
  SDLog_ClearError();
  SDLog_Unlock();
}

void SdLog_RequestScanLog(void)
{
  (void)SDLog_ListFiles();
}

void SdLog_OnTimPeriodElapsed(TIM_HandleTypeDef *htim)
{
  (void)htim;
}

void SdLog_GetStatus(SdLogStatusSnapshot_t *status)
{
  (void)SDLog_GetStatusSnapshot(status);
}
