#include "heartbeat_task.h"

void StartHeartBeatTask(void const * argument)
{
  for(;;)
  {
		imu_sensor.heart_beat(&imu_sensor.work_state);
		rc_sensor.heart_beat(&rc_sensor);
		for (int16_t i = 0; i < MOTOR_LIST; i++)
		{
			RM_motor[i].heartbeat(&RM_motor[i]);
		}
		module.heartbeat(&module);
    osDelay(1);
  }
}
