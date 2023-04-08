#include "control_task.h"

uint16_t fps = 5;

void StartControlTask(void const * argument)
{
	uint16_t i = 0;
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
		CAN_SendAll();
		
		if (++i == 60000)
		{
			i = 0;
		}
		if (i % fps == 0)
		{
			vision_send_data();
		}
		
    osDelay(1);
  }
}
