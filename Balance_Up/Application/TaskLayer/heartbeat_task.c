#include "heartbeat_task.h"

uint16_t fps = 4;
uint32_t err_code = 0;
float imu_temp[3];

void StartHeartBeatTask(void const * argument)
{
	uint16_t i = 0;
  for(;;)
  {
		if (++i == 60000)
			i = 0;
		
		if (*(imu_sensor.info->kp) != 0.1f)
		{
			if (i % 1000 == 0)
				*(imu_sensor.info->kp) = 0.1f;
		}
		
#if defined	(DEBUG_MODE) && (DEBUG_MODE == 0)
		HAL_IWDG_Refresh(&hiwdg);
#endif
		imu_sensor.heart_beat(&imu_sensor.work_state);
		rc_sensor.heart_beat(&rc_sensor);
		vision_sensor.heart_beat(&vision_sensor);
		judge.heart_beat(&judge);
		slave.heart_beat(&slave);
		for (int16_t j = 0; j < MOTOR_LIST; j++)
		{
			RM_motor[j].heartbeat(&RM_motor[j]);
		}
		module.heartbeat(&module);
		slave.tx(&slave);
		if (i % fps == 0)
			vision_send_data();
		
//		if (imu_sensor.work_state.cali_end == 1)
//		{
//			
//			imu_sensor.work_state.cali_end = 0;
//			imu_temp[0] = imu_sensor.info->offset_info.gx_offset;
//			imu_temp[1] = imu_sensor.info->offset_info.gy_offset;
//			imu_temp[2] = imu_sensor.info->offset_info.gz_offset;
//			err_code = Flash_EraseSector11();
//			
//			if (++err_code == 0)
//				Flash_WriteWordData(IMU_DATA_ADDR, (uint32_t*)imu_temp, 3);
//		}
		
    osDelay(1);
  }
}
