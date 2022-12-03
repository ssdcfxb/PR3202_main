#include "heartbeat_task.h"

void StartHeartBeatTask(void const * argument)
{
  for(;;)
  {
    osDelay(1);
  }
}
