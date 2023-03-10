#include "control_task.h"

void StartControlTask(void const * argument)
{
	gimbal.init();
	launcher.init();
  for(;;)
  {
		if(module.state == MODULE_STATE_NORMAL) 
		{
			gimbal.ctrl();
			launcher.ctrl();
		} 
		else 
		{
			gimbal.self_protect();
			launcher.self_protect();
		}
		
		slave.tx(&slave);
		CAN_SendAll();
		
    osDelay(1);
  }
}
