#include "heartbeat_task.h"

uint8_t  super_cnt;
void StartHeartBeatTask(void const * argument)
{
  for(;;)
  {
		
		#ifdef LINK_TO_RECEIVE	
		{
			rc_sensor.heart_beat(&rc_sensor);
		}
		#endif	
		
		#ifdef NO_LINK_TO_RECEIVE	
		{
			Master.heart_beat(&Master);	
		}
		#endif	
		
		All_motor_heartbeat();
		
		judge_sensor.heart_beat(&judge_sensor);
		
		super_cnt ++;
		if(super_cnt == 5)
		{
			Super_2023.heart_beat(&Super_2023);
			super_cnt = 0;
		}
		
		ALL_TOF_Sensor_HeartBeat();
		
    osDelay(1);
  }
}
