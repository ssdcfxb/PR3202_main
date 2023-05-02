#include "heartbeat_task.h"

uint32_t err_code = 0;
float imu_temp[3];

void StartHeartBeatTask(void const * argument)
{
	uint16_t i = 0;
  for(;;)
  {
//		HAL_IWDG_Refresh(&hiwdg);
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
		
		if (++i == 60000)
			i = 0;
		if (imu_sensor.work_state.cali_end == 1)
		{
			
			imu_sensor.work_state.cali_end = 0;
			imu_temp[0] = imu_sensor.info->offset_info.gx_offset;
			imu_temp[1] = imu_sensor.info->offset_info.gy_offset;
			imu_temp[2] = imu_sensor.info->offset_info.gz_offset;
			err_code = Flash_EraseSector11();
			
			if (++err_code == 0)
				Flash_WriteWordData(IMU_DATA_ADDR, (uint32_t*)imu_temp, 3);
		}
		
    osDelay(1);
  }
}
