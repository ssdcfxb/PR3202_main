#include "control_task.h"

void StartControlTask(void const * argument)
{
	gimbal.init();
	launcher.init();
  for(;;)
  {
		if (symbal.rc_update == 1)
		{
			keyboard_update_interrupt(rc_sensor.info);
		}
		else
		{
			keyboard_update(rc_sensor.info);
		}
		
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
