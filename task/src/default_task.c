#include "../inc/default_task.h"

#include "cmsis_os2.h"

void StartDefaultTask(void *argument)
{
  (void)argument;

  for (;;)
  {
    osDelay(1);
  }
}
