#include "../inc/lcd_task.h"

#include "cmsis_os2.h"
#include "lcd.h"
#include "rtc.h"
#include "RS485_uasrt.h"
#include "timers_APP.h"

extern lcd lcd_desc;
extern RTC_HandleTypeDef hrtc;

void StartLcdTask(void *argument)
{
  RTC_TimeTypeDef RTC_TimeStruct;
  RTC_DateTypeDef RTC_DateStruct;
  RS485_StatusTypeDef rs485_status;
  uint32_t lcd_refresh_count = 0U;
  uint32_t run_tick = 0U;

  (void)argument;

  lcd_fill(&lcd_desc, 0, 118, 319, 171, BLACK);

  for (;;)
  {
    if (Timers_APP_TakeLcdRefreshEvent() == 0U)
    {
      osDelay(10);
      continue;
    }

    lcd_refresh_count++;

    HAL_RTC_GetTime(&hrtc, &RTC_TimeStruct, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &RTC_DateStruct, RTC_FORMAT_BIN);
    RS485_GetStatus(&rs485_status);
    run_tick = osKernelGetTickCount();

    lcd_print(&lcd_desc, 8, 120, "STATE:%s IRQ:%lu CB:%lu      ",
              (rs485_status.tx_busy != 0U) ? "TX" : "RX",
              rs485_status.tx_dma_irq,
              rs485_status.tx_cplt_callback);
    lcd_print(&lcd_desc, 8, 135, "REQ:%lu RX:%lu TX:%lu OV:%lu      ",
              rs485_status.tx_requests,
              rs485_status.rx_events,
              rs485_status.tx_done,
              rs485_status.rx_overwrite);
    lcd_print(&lcd_desc, 8, 150, "RTC:20%02d-%02d-%02d %02d:%02d:%02d",
              RTC_DateStruct.Year,
              RTC_DateStruct.Month,
              RTC_DateStruct.Date,
              RTC_TimeStruct.Hours,
              RTC_TimeStruct.Minutes,
              RTC_TimeStruct.Seconds);
    lcd_print(&lcd_desc, 8, 165, "RUN:%lums LCD:%lu I:%lu T:%lu      ",
              run_tick,
              lcd_refresh_count,
              rs485_status.tx_from_init,
              rs485_status.tx_from_echo_task);
  }
}
