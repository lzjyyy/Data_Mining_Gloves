#include "../inc/lcd_task.h"

#include "cmsis_os2.h"
#include "lcd.h"
#include "rtc.h"
#include "RS485_uasrt.h"
#include "../inc/rs485_task.h"
#include "timers_APP.h"

extern lcd lcd_desc;
extern RTC_HandleTypeDef hrtc;

volatile uint32_t lcd_task_entry_count = 0U;
volatile uint32_t lcd_task_loop_count = 0U;

void StartLcdTask(void *argument)
{
  RTC_TimeTypeDef RTC_TimeStruct;
  RTC_DateTypeDef RTC_DateStruct;
  RS485_StatusTypeDef rs485_status;
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
    RS485_TaskGetEventCounts(&rs485_task_rx_events, &rs485_task_tx_events);

    lcd_print(&lcd_desc, 8, 85, "STATE:%s IRQ:%lu CB:%lu      ",
              (rs485_status.tx_busy != 0U) ? "TX" : "RX",
              rs485_status.tx_dma_irq,
              rs485_status.tx_cplt_callback);
    lcd_print(&lcd_desc, 8, 105, "REQ:%lu RX:%lu TD:%lu OV:%lu      ",
              rs485_status.tx_requests,
              rs485_status.rx_events,
              rs485_status.tx_done,
              rs485_status.rx_overwrite);
    lcd_print(&lcd_desc, 8, 125, "RE:%lu TE:%lu MR:%lu MN:%lu MF:%lu   ",
              rs485_task_rx_events,
              rs485_task_tx_events,
              rs485_status.modbus_response_ready,
              rs485_status.modbus_no_response,
              rs485_status.modbus_frame_error);
    lcd_print(&lcd_desc, 8, 145, "RTC:20%02d-%02d-%02d %02d:%02d:%02d",
              RTC_DateStruct.Year,
              RTC_DateStruct.Month,
              RTC_DateStruct.Date,
              RTC_TimeStruct.Hours,
              RTC_TimeStruct.Minutes,
              RTC_TimeStruct.Seconds);
  }
}
