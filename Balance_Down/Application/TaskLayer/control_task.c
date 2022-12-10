#include "control_task.h"

void StartControlTask(void const * argument)
{
  for(;;)
  {
		//chassic.ctrl();
		
		chassic.stop();
		
    osDelay(1);
  }
}
