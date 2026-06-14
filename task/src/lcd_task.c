#include "../inc/lcd_task.h"

#include "cmsis_os2.h"
#include "lcd.h"
#include "rtc.h"
#include "RS485_uasrt.h"
#include "sd_log.h"
#include "../inc/rs485_task.h"
#include "timers_APP.h"
#include <stdio.h>

extern lcd lcd_desc;
extern RTC_HandleTypeDef hrtc;

volatile uint32_t lcd_task_entry_count = 0U;
volatile uint32_t lcd_task_loop_count = 0U;

void StartLcdTask(void *argument)
{
  RTC_TimeTypeDef RTC_TimeStruct;
  RTC_DateTypeDef RTC_DateStruct;
  RS485_StatusTypeDef rs485_status;
  SdLogStatusSnapshot_t sd_status;
  char sd_status_text[18];
  uint32_t sd_written_kb = 0U;
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
    SdLog_GetStatus(&sd_status);
    sd_written_kb = (uint32_t)(sd_status.current_file_size / 1024U);

    if (sd_status.log_status == SD_LOG_RECORD_ERROR)
    {
      snprintf(sd_status_text, sizeof(sd_status_text), "SD:ERR:%02u     ", sd_status.error_code);
    }
    else if (sd_status.fs_status != SD_LOG_FS_MOUNTED)
    {
      snprintf(sd_status_text, sizeof(sd_status_text), "SD:NO CARD     ");
    }
    else if (sd_status.log_status == SD_LOG_RECORD_RECORDING)
    {
      snprintf(sd_status_text, sizeof(sd_status_text), "SD:OK LOG:REC ");
    }
    else
    {
      snprintf(sd_status_text, sizeof(sd_status_text), "SD:OK LOG:IDLE");
    }

    lcd_print(&lcd_desc, 8, 85, "STATE:%s IRQ:%lu      ",
              (rs485_status.tx_busy != 0U) ? "TX" : "RX",
              rs485_status.tx_dma_irq);
    lcd_print(&lcd_desc, 100, 20, "WRITE:%lu KB        ", sd_written_kb);
    lcd_print(&lcd_desc, 200, 85, "%s", sd_status_text);
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
