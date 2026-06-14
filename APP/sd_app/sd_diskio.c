#include "ff.h"
#include "diskio.h"

#include "cmsis_os2.h"
#include "sdmmc.h"
#include <stdint.h>
#include <string.h>

#define SD_DISK_PDRV              0U
#define SD_DISK_BLOCK_SIZE        512U
#define SD_DISK_TIMEOUT_MS        5000U

static volatile DSTATUS sd_status = STA_NOINIT;
volatile uint32_t sd_disk_last_hal_status = 0U;
volatile uint32_t sd_disk_last_hal_error = 0U;
volatile uint32_t sd_disk_last_result = RES_OK;
static osSemaphoreId_t sd_dma_sem;
static __ALIGNED(4) uint8_t sd_scratch[SD_DISK_BLOCK_SIZE];

static DRESULT SdDisk_SetResult(DRESULT result, HAL_StatusTypeDef hal_status)
{
  sd_disk_last_result = result;
  sd_disk_last_hal_status = (uint32_t)hal_status;
  sd_disk_last_hal_error = HAL_SD_GetError(&hsd1);

  return result;
}

static void SdDisk_EnsureSemaphore(void)
{
  if (sd_dma_sem == NULL)
  {
    const osSemaphoreAttr_t attr = {
      .name = "sdDmaSem"
    };

    sd_dma_sem = osSemaphoreNew(1U, 0U, &attr);
  }
}

static DRESULT SdDisk_WaitReady(uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();

  while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER)
  {
    if ((HAL_GetTick() - start) > timeout_ms)
    {
      return RES_ERROR;
    }
    osDelay(1U);
  }

  return RES_OK;
}

static DRESULT SdDisk_WaitDma(void)
{
  SdDisk_EnsureSemaphore();
  if (sd_dma_sem == NULL)
  {
    return RES_ERROR;
  }

  if (osSemaphoreAcquire(sd_dma_sem, SD_DISK_TIMEOUT_MS) != osOK)
  {
    return RES_ERROR;
  }

  return SdDisk_WaitReady(SD_DISK_TIMEOUT_MS);
}

DSTATUS disk_initialize(BYTE pdrv)
{
  if (pdrv != SD_DISK_PDRV)
  {
    return STA_NOINIT;
  }

  if (HAL_SD_GetCardState(&hsd1) == HAL_SD_CARD_TRANSFER)
  {
    sd_status = 0U;
  }
  else if (HAL_SD_InitCard(&hsd1) == HAL_OK)
  {
    sd_status = 0U;
  }
  else
  {
    sd_status = STA_NOINIT;
  }

  SdDisk_EnsureSemaphore();

  return sd_status;
}

DSTATUS disk_status(BYTE pdrv)
{
  if (pdrv != SD_DISK_PDRV)
  {
    return STA_NOINIT;
  }

  return sd_status;
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count)
{
  HAL_StatusTypeDef status;

  if ((pdrv != SD_DISK_PDRV) || (buff == NULL) || (count == 0U))
  {
    return RES_PARERR;
  }

  if ((sd_status & STA_NOINIT) != 0U)
  {
    return RES_NOTRDY;
  }

  if ((((uintptr_t)buff) & 0x03U) == 0U)
  {
    status = HAL_SD_ReadBlocks_DMA(&hsd1, buff, (uint32_t)sector, count);
    if (status != HAL_OK)
    {
      return SdDisk_SetResult(RES_ERROR, status);
    }

    return SdDisk_SetResult(SdDisk_WaitDma(), status);
  }

  for (UINT index = 0U; index < count; index++)
  {
    status = HAL_SD_ReadBlocks_DMA(&hsd1, sd_scratch, (uint32_t)sector + index, 1U);
    if (status != HAL_OK)
    {
      return SdDisk_SetResult(RES_ERROR, status);
    }
    if (SdDisk_WaitDma() != RES_OK)
    {
      return SdDisk_SetResult(RES_ERROR, status);
    }
    memcpy(&buff[index * SD_DISK_BLOCK_SIZE], sd_scratch, SD_DISK_BLOCK_SIZE);
  }

  return SdDisk_SetResult(RES_OK, HAL_OK);
}

#if FF_FS_READONLY == 0
DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count)
{
  HAL_StatusTypeDef status;

  if ((pdrv != SD_DISK_PDRV) || (buff == NULL) || (count == 0U))
  {
    return RES_PARERR;
  }

  if ((sd_status & STA_NOINIT) != 0U)
  {
    return RES_NOTRDY;
  }

  if ((((uintptr_t)buff) & 0x03U) == 0U)
  {
    status = HAL_SD_WriteBlocks_DMA(&hsd1, buff, (uint32_t)sector, count);
    if (status != HAL_OK)
    {
      return SdDisk_SetResult(RES_ERROR, status);
    }

    return SdDisk_SetResult(SdDisk_WaitDma(), status);
  }

  for (UINT index = 0U; index < count; index++)
  {
    memcpy(sd_scratch, &buff[index * SD_DISK_BLOCK_SIZE], SD_DISK_BLOCK_SIZE);
    status = HAL_SD_WriteBlocks_DMA(&hsd1, sd_scratch, (uint32_t)sector + index, 1U);
    if (status != HAL_OK)
    {
      return SdDisk_SetResult(RES_ERROR, status);
    }
    if (SdDisk_WaitDma() != RES_OK)
    {
      return SdDisk_SetResult(RES_ERROR, status);
    }
  }

  return SdDisk_SetResult(RES_OK, HAL_OK);
}
#endif

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
  HAL_SD_CardInfoTypeDef card_info;

  if (pdrv != SD_DISK_PDRV)
  {
    return RES_PARERR;
  }

  if ((sd_status & STA_NOINIT) != 0U)
  {
    return RES_NOTRDY;
  }

  switch (cmd)
  {
    case CTRL_SYNC:
      return SdDisk_WaitReady(SD_DISK_TIMEOUT_MS);

    case GET_SECTOR_COUNT:
      if (buff == NULL)
      {
        return RES_PARERR;
      }
      if (HAL_SD_GetCardInfo(&hsd1, &card_info) != HAL_OK)
      {
        return RES_ERROR;
      }
      *(LBA_t *)buff = (LBA_t)card_info.LogBlockNbr;
      return RES_OK;

    case GET_SECTOR_SIZE:
      if (buff == NULL)
      {
        return RES_PARERR;
      }
      *(WORD *)buff = SD_DISK_BLOCK_SIZE;
      return RES_OK;

    case GET_BLOCK_SIZE:
      if (buff == NULL)
      {
        return RES_PARERR;
      }
      *(DWORD *)buff = 1U;
      return RES_OK;

    default:
      return RES_PARERR;
  }
}

DWORD get_fattime(void)
{
  return ((DWORD)(2026U - 1980U) << 25) |
         ((DWORD)1U << 21) |
         ((DWORD)1U << 16);
}

void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd)
{
  (void)hsd;
  if (sd_dma_sem != NULL)
  {
    (void)osSemaphoreRelease(sd_dma_sem);
  }
}

void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd)
{
  (void)hsd;
  if (sd_dma_sem != NULL)
  {
    (void)osSemaphoreRelease(sd_dma_sem);
  }
}

void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd)
{
  (void)hsd;
  if (sd_dma_sem != NULL)
  {
    (void)osSemaphoreRelease(sd_dma_sem);
  }
}
