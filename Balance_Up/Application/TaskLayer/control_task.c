#include "control_task.h"

void StartControlTask(void const * argument)
{
	gimbal.init();
	launcher.init();
  for(;;)
  {
		if(module.state == MODULE_STATE_NORMAL) 
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
		
		slave.tx(&slave);
		vision_send_data();
		CAN_SendAll();
		
    osDelay(1);
  }
}
