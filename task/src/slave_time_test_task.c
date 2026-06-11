#include "../inc/slave_time_test_task.h"

#include "cmsis_os2.h"
#include "modbus_master.h"
#include "modbus_registers.h"
#include "timers_APP.h"
#include "../inc/sync_output_task.h"

#define SLAVE_TIME_TEST_ADDR                 MODBUS_SLAVE_ADDR_DEFAULT
#define SLAVE_TIME_TEST_INITIAL_UTC_US       1710000000000000ULL
#define SLAVE_TIME_TEST_INTERVAL_US          100000ULL
#define SLAVE_TIME_TEST_INTERVAL_TOL_US      10000ULL
#define SLAVE_TIME_TEST_MODBUS_TIMEOUT_MS    30U

volatile uint32_t slave_time_test_sync_ok = 0U;
volatile uint32_t slave_time_test_sync_fail = 0U;
volatile uint32_t slave_time_test_read_ok = 0U;
volatile uint32_t slave_time_test_read_fail = 0U;
volatile uint32_t slave_time_test_interval_ok = 0U;
volatile uint32_t slave_time_test_interval_fail = 0U;
volatile uint64_t slave_time_test_last_utc_us = 0U;
volatile uint64_t slave_time_test_last_delta_us = 0U;

static uint8_t SlaveTimeTest_IsIntervalOk(uint64_t delta_us)
{
  uint64_t diff_us;

  if (delta_us >= SLAVE_TIME_TEST_INTERVAL_US)
  {
    diff_us = delta_us - SLAVE_TIME_TEST_INTERVAL_US;
  }
  else
  {
    diff_us = SLAVE_TIME_TEST_INTERVAL_US - delta_us;
  }

  return (diff_us <= SLAVE_TIME_TEST_INTERVAL_TOL_US) ? 1U : 0U;
}

void StartSlaveTimeTestTask(void *argument)
{
  uint64_t slave_utc_us = 0U;
  uint64_t last_utc_us = 0U;
  uint64_t delta_us;
  uint8_t has_last = 0U;
  uint8_t was_running = 0U;
  uint8_t sync_sent = 0U;

  (void)argument;

  osDelay(300);

  for (;;)
  {
    if (SyncOutput_IsRunning() == 0U)
    {
      was_running = 0U;
      sync_sent = 0U;
      has_last = 0U;
      osDelay(20);
      continue;
    }

    if (was_running == 0U)
    {
      was_running = 1U;
      has_last = 0U;
      sync_sent = 0U;
    }

    if (sync_sent == 0U)
    {
      if (ModbusMaster_WriteU64(SLAVE_TIME_TEST_ADDR,
                                REG_TIME_SYNC_UTC_US,
                                SLAVE_TIME_TEST_INITIAL_UTC_US,
                                SLAVE_TIME_TEST_MODBUS_TIMEOUT_MS) == HAL_OK)
      {
        slave_time_test_sync_ok++;
        sync_sent = 1U;
      }
      else
      {
        slave_time_test_sync_fail++;
        osDelay(100);
        continue;
      }
    }

    if (Timers_APP_TakeSlaveTimeCheckEvent() == 0U)
    {
      osDelay(5);
      continue;
    }

    if (ModbusMaster_ReadU64(SLAVE_TIME_TEST_ADDR,
                             REG_UTC_TIMESTAMP_US,
                             &slave_utc_us,
                             SLAVE_TIME_TEST_MODBUS_TIMEOUT_MS) == HAL_OK)
    {
      slave_time_test_read_ok++;
      slave_time_test_last_utc_us = slave_utc_us;

      if (has_last != 0U)
      {
        delta_us = slave_utc_us - last_utc_us;
        slave_time_test_last_delta_us = delta_us;

        if ((slave_utc_us >= last_utc_us) && (SlaveTimeTest_IsIntervalOk(delta_us) != 0U))
        {
          slave_time_test_interval_ok++;
        }
        else
        {
          slave_time_test_interval_fail++;
        }
      }

      last_utc_us = slave_utc_us;
      has_last = 1U;
    }
    else
    {
      slave_time_test_read_fail++;
    }
  }
}
