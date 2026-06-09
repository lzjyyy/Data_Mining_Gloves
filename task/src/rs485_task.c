#include "../inc/rs485_task.h"

#include "cmsis_os2.h"
#include "RS485_uasrt.h"

void StartRs485Task(void *argument)
{
  (void)argument;

  for (;;)
  {
    RS485_PollEcho();
    osDelay(1);
  }
}
