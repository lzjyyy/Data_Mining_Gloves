#include "../inc/sync_output_task.h"

#include "cmsis_os2.h"
#include "main.h"

#define SYNC_OUTPUT_EVT_BUTTON       (1UL << 0)
#define SYNC_OUTPUT_DEBOUNCE_MS      80U
#define SYNC_OUTPUT_PULSE_LOW_MS     2U
#define SYNC_OUTPUT_PERIOD_MS        100U

static osThreadId_t sync_output_task_id = NULL;
static volatile uint8_t sync_output_running = 0U;
static volatile uint32_t sync_output_toggle_count = 0U;
static volatile uint32_t sync_output_pulse_count = 0U;
static volatile uint32_t sync_output_last_button_tick = 0U;

static void SyncOutput_SetIdleLevel(void)
{
  HAL_GPIO_WritePin(time_tongbu_GPIO_Port, time_tongbu_Pin, GPIO_PIN_SET);
}

static void SyncOutput_PulseFallingEdge(void)
{
  HAL_GPIO_WritePin(time_tongbu_GPIO_Port, time_tongbu_Pin, GPIO_PIN_SET);
  osDelay(1);
  HAL_GPIO_WritePin(time_tongbu_GPIO_Port, time_tongbu_Pin, GPIO_PIN_RESET);
  osDelay(SYNC_OUTPUT_PULSE_LOW_MS);
  HAL_GPIO_WritePin(time_tongbu_GPIO_Port, time_tongbu_Pin, GPIO_PIN_SET);
  sync_output_pulse_count++;
}

void SyncOutput_OnButtonExti(uint16_t gpio_pin)
{
  uint32_t now_tick;

  if (gpio_pin != syctime_key_Pin)
  {
    return;
  }

  now_tick = HAL_GetTick();
  if ((uint32_t)(now_tick - sync_output_last_button_tick) < SYNC_OUTPUT_DEBOUNCE_MS)
  {
    return;
  }

  sync_output_last_button_tick = now_tick;
  sync_output_running = (sync_output_running == 0U) ? 1U : 0U;
  sync_output_toggle_count++;

  if (sync_output_task_id != NULL)
  {
    (void)osThreadFlagsSet(sync_output_task_id, SYNC_OUTPUT_EVT_BUTTON);
  }
}

uint8_t SyncOutput_IsRunning(void)
{
  return sync_output_running;
}

uint32_t SyncOutput_GetToggleCount(void)
{
  return sync_output_toggle_count;
}

uint32_t SyncOutput_GetPulseCount(void)
{
  return sync_output_pulse_count;
}

void StartSyncOutputTask(void *argument)
{
  uint32_t flags;

  (void)argument;

  sync_output_task_id = osThreadGetId();
  SyncOutput_SetIdleLevel();

  for (;;)
  {
    if (sync_output_running == 0U)
    {
      SyncOutput_SetIdleLevel();
      (void)osThreadFlagsWait(SYNC_OUTPUT_EVT_BUTTON, osFlagsWaitAny, osWaitForever);
      continue;
    }

    SyncOutput_PulseFallingEdge();
    flags = osThreadFlagsWait(SYNC_OUTPUT_EVT_BUTTON,
                              osFlagsWaitAny,
                              SYNC_OUTPUT_PERIOD_MS);
    (void)flags;
  }
}
