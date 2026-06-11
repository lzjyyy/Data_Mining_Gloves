#include "modbus_time_sync.h"

static volatile uint32_t time_sync_local_base_ms = 0U;
static volatile uint32_t time_sync_utc_base_ms = 0U;
static volatile uint64_t time_sync_last_sync_utc_us = 0U;
static volatile uint8_t time_sync_synced = 0U;

static uint64_t ModbusTimeSync_GetElapsedUsFromBase(uint32_t base_ms)
{
  return (uint64_t)((uint32_t)(HAL_GetTick() - base_ms)) * 1000ULL;
}

HAL_StatusTypeDef ModbusTimeSync_Init(void)
{
  uint32_t now_ms = HAL_GetTick();

  time_sync_local_base_ms = now_ms;
  time_sync_utc_base_ms = now_ms;
  time_sync_last_sync_utc_us = 0U;
  time_sync_synced = 0U;

  return HAL_OK;
}

void ModbusTimeSync_OnTimPeriodElapsed(TIM_HandleTypeDef *htim)
{
  (void)htim;
}

void ModbusTimeSync_OnGpioFalling(uint16_t gpio_pin)
{
  (void)gpio_pin;
}

uint64_t ModbusTimeSync_GetLocalUptimeUs(void)
{
  uint32_t local_base_ms;

  __disable_irq();
  local_base_ms = time_sync_local_base_ms;
  __enable_irq();

  return ModbusTimeSync_GetElapsedUsFromBase(local_base_ms);
}

uint64_t ModbusTimeSync_GetUtcTimestampUs(void)
{
  uint32_t utc_base_ms;
  uint64_t utc_base_us;
  uint8_t synced;

  __disable_irq();
  utc_base_ms = time_sync_utc_base_ms;
  utc_base_us = time_sync_last_sync_utc_us;
  synced = time_sync_synced;
  __enable_irq();

  if (synced == 0U)
  {
    return 0U;
  }

  return utc_base_us + ModbusTimeSync_GetElapsedUsFromBase(utc_base_ms);
}

uint64_t ModbusTimeSync_GetLastSyncUtcUs(void)
{
  uint64_t utc_us;

  __disable_irq();
  utc_us = time_sync_last_sync_utc_us;
  __enable_irq();

  return utc_us;
}

uint64_t ModbusTimeSync_GetLastSyncEdgeLocalUs(void)
{
  return 0U;
}

uint64_t ModbusTimeSync_GetLastLocalIntervalUs(void)
{
  return 0U;
}

uint64_t ModbusTimeSync_GetPredictedEdgeUtcUs(void)
{
  return 0U;
}

int64_t ModbusTimeSync_GetLastSyncErrorUs(void)
{
  return 0;
}

int32_t ModbusTimeSync_GetFreqCorrPpb(void)
{
  return 0;
}

uint8_t ModbusTimeSync_IsSynced(void)
{
  return time_sync_synced;
}

uint8_t ModbusTimeSync_IsWaitingUtc(void)
{
  return 0U;
}

void ModbusTimeSync_SetUtcFromMaster(uint64_t utc_us)
{
  __disable_irq();
  time_sync_last_sync_utc_us = utc_us;
  time_sync_utc_base_ms = HAL_GetTick();
  time_sync_synced = 1U;
  __enable_irq();
}
