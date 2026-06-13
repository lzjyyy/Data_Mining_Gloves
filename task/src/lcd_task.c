#include "../inc/lcd_task.h"

#include "cmsis_os2.h"
#include "lcd.h"
#include "rtc.h"
#include "RS485_uasrt.h"
#include "modbus_master.h"
#include "../inc/rs485_task.h"
#include "timers_APP.h"
#include "../inc/sync_output_task.h"

extern lcd lcd_desc;
extern RTC_HandleTypeDef hrtc;
extern volatile uint32_t slave_time_test_sync_ok;
extern volatile uint32_t slave_time_test_sync_fail;
extern volatile uint32_t slave_time_test_read_ok;
extern volatile uint32_t slave_time_test_read_fail;
extern volatile uint32_t slave_time_test_interval_ok;
extern volatile uint32_t slave_time_test_interval_fail;
extern volatile uint64_t slave_time_test_last_delta_us;

volatile uint32_t lcd_task_entry_count = 0U;
volatile uint32_t lcd_task_loop_count = 0U;

void StartLcdTask(void *argument)
{
  RTC_TimeTypeDef RTC_TimeStruct;
  RTC_DateTypeDef RTC_DateStruct;
  RS485_StatusTypeDef rs485_status;
  ModbusMaster_DebugTypeDef modbus_debug;
  uint32_t rs485_task_rx_events = 0U;
  uint32_t rs485_task_tx_events = 0U;
  uint32_t lcd_refresh_count = 0U;
  uint32_t lcd_wait_ms = 0U;

  (void)argument;

  lcd_task_entry_count++;
  lcd_print(&lcd_desc, 8, 85, "LCD TASK RUN");

  for (;;)
  {
    lcd_task_loop_count++;

    if (Timers_APP_TakeLcdRefreshEvent() == 0U)
    {
      osDelay(10);
      lcd_wait_ms += 10U;

      if (lcd_wait_ms < 200U)
      {
        continue;
      }
    }

    lcd_wait_ms = 0U;
    lcd_refresh_count++;

    HAL_RTC_GetTime(&hrtc, &RTC_TimeStruct, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &RTC_DateStruct, RTC_FORMAT_BIN);
    RS485_GetStatus(&rs485_status);
    ModbusMaster_GetDebug(&modbus_debug);
    RS485_TaskGetEventCounts(&rs485_task_rx_events, &rs485_task_tx_events);

    lcd_print(&lcd_desc, 8, 85, "STATE:%s OP:%lu MS:%lu ML:%lu ",
              (SyncOutput_IsRunning() != 0U) ? "RUN" : "STOP",
              modbus_debug.op,
              modbus_debug.status,
              modbus_debug.rx_len);
    lcd_print(&lcd_desc, 8, 105, "SYNC O:%lu F:%lu RD O:%lu F:%lu   ",
              slave_time_test_sync_ok,
              slave_time_test_sync_fail,
              slave_time_test_read_ok,
              slave_time_test_read_fail);
    lcd_print(&lcd_desc, 8, 125, "INTV O:%lu F:%lu D:%lu us       ",
              slave_time_test_interval_ok,
              slave_time_test_interval_fail,
              (uint32_t)slave_time_test_last_delta_us);
    lcd_print(&lcd_desc, 8, 145, "KEY:%lu P:%lu A:%lu F:%lu    ",
              SyncOutput_GetToggleCount(),
              SyncOutput_GetPulseCount(),
              modbus_debug.rx_addr,
              modbus_debug.rx_func);
  }
}
