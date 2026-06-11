#include "../inc/app_task.h"

#include "cmsis_os2.h"
#include "../inc/lcd_task.h"
#include "../inc/rs485_task.h"
#include "../inc/slave_time_test_task.h"

static osThreadId_t rs485TaskHandle;
static const osThreadAttr_t rs485Task_attributes = {
  .name = "rs485Task",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 512 * 4
};

static osThreadId_t lcdTaskHandle;
static const osThreadAttr_t lcdTask_attributes = {
  .name = "lcdTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 512 * 4
};

static osThreadId_t slaveTimeTestTaskHandle;
static const osThreadAttr_t slaveTimeTestTask_attributes = {
  .name = "slaveTimeTest",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 512 * 4
};

static uint8_t rs485TaskCreated = 0U;
static uint8_t lcdTaskCreated = 0U;

volatile uint32_t app_task_init_count = 0U;
volatile uint32_t app_task_rs485_create_ok = 0U;
volatile uint32_t app_task_lcd_create_ok = 0U;

void AppTask_Init(void)
{
  app_task_init_count++;

  rs485TaskHandle = osThreadNew(StartRs485Task, NULL, &rs485Task_attributes);
  rs485TaskCreated = (rs485TaskHandle != NULL) ? 1U : 0U;
  app_task_rs485_create_ok = rs485TaskCreated;

  lcdTaskHandle = osThreadNew(StartLcdTask, NULL, &lcdTask_attributes);
  lcdTaskCreated = (lcdTaskHandle != NULL) ? 1U : 0U;
  app_task_lcd_create_ok = lcdTaskCreated;

  slaveTimeTestTaskHandle = osThreadNew(StartSlaveTimeTestTask, NULL, &slaveTimeTestTask_attributes);
}

void AppTask_GetCreateStatus(uint8_t *rs485_created, uint8_t *lcd_created)
{
  if (rs485_created != NULL)
  {
    *rs485_created = rs485TaskCreated;
  }

  if (lcd_created != NULL)
  {
    *lcd_created = lcdTaskCreated;
  }
}
