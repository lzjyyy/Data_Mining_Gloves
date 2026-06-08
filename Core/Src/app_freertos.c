/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : FreeRTOS applicative file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_freertos.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "rtc.h"
#include "lcd.h"
#include "RS485_uasrt.h"
#include "timers_APP.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern lcd lcd_desc;
extern RTC_HandleTypeDef hrtc;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for rs485Task */
osThreadId_t rs485TaskHandle;
const osThreadAttr_t rs485Task_attributes = {
  .name = "rs485Task",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 512 * 4
};
/* Definitions for lcdTask */
osThreadId_t lcdTaskHandle;
const osThreadAttr_t lcdTask_attributes = {
  .name = "lcdTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 512 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of rs485Task */
  rs485TaskHandle = osThreadNew(StartRs485Task, NULL, &rs485Task_attributes);

  /* creation of lcdTask */
  lcdTaskHandle = osThreadNew(StartLcdTask, NULL, &lcdTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}
/* USER CODE BEGIN Header_StartDefaultTask */
/**
* @brief Function implementing the defaultTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN defaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END defaultTask */
}

/* USER CODE BEGIN Header_StartRs485Task */
/**
* @brief Function implementing the rs485Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRs485Task */
void StartRs485Task(void *argument)
{
  /* USER CODE BEGIN rs485Task */
  /* Infinite loop */
  for(;;)
  {
    RS485_PollEcho();
    osDelay(1);
  }
  /* USER CODE END rs485Task */
}

/* USER CODE BEGIN Header_StartLcdTask */
/**
* @brief Function implementing the lcdTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLcdTask */
void StartLcdTask(void *argument)
{
  /* USER CODE BEGIN lcdTask */
  RTC_TimeTypeDef RTC_TimeStruct;
  RTC_DateTypeDef RTC_DateStruct;
  RS485_StatusTypeDef rs485_status;
  uint32_t lcd_refresh_count = 0U;
  uint32_t run_tick = 0U;

  lcd_fill(&lcd_desc, 0, 118, 319, 171, BLACK);

  /* Infinite loop */
  for(;;)
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
  /* USER CODE END lcdTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

