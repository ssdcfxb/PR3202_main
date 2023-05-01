#include "heartbeat_task.h"

void StartHeartBeatTask(void const * argument)
{
  for(;;)
  {
		HAL_IWDG_Refresh(&hiwdg);
		imu_sensor.heart_beat(&imu_sensor.work_state);
		rc_sensor.heart_beat(&rc_sensor);
		vision_sensor.heart_beat(&vision_sensor);
		judge.heart_beat(&judge);
		slave.heart_beat(&slave);
		for (int16_t i = 0; i < MOTOR_LIST; i++)
		{
			RM_motor[i].heartbeat(&RM_motor[i]);
		}
		module.heartbeat(&module);
    osDelay(1);
  }
}
