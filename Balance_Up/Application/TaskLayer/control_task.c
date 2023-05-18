#include "control_task.h"


void StartControlTask(void const * argument)
{
	gimbal.init();
	launcher.init();
  for(;;)
  {
		if((module.state == MODULE_STATE_NORMAL) && (symbal.slave_reset != 1))
		{
			if (module.remote_mode == KEY)
			{
				if (symbal.rc_update == 1)
				{
					keyboard_update_interrupt(rc_sensor.info);
					symbal.rc_update = 0;
				}
				else
				{
					keyboard_update(rc_sensor.info);
				}
			}
			gimbal.ctrl();
			launcher.ctrl();
		} 
		else 
		{
			gimbal.self_protect();
			launcher.self_protect();
		}
		
		CAN_SendAll();
		
    osDelay(1);
  }
}
