#include "led_task.h"

void StartLedTask(void const * argument)
{
	static	uint16_t i = 0;
	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
  for(;;)
  {
		if (++i == 60000)
			i = 0;
		
		/*  陀螺仪指示灯  */
//		if (imu_sensor.work_state.err_code == IMU_NONE_ERR)
//		{
//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
//		}
//		else
//		{
//			if (i % 200 == 0)
//				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_1);
//		}
//		
//		/*  云台电机指示灯  */
//		if ((RM_motor[GIM_Y].state.work_state == M_ONLINE) && (RM_motor[GIM_P].state.work_state == M_ONLINE))
//		{
//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
//		}
//		else
//		{
//			if (i % 500 == 0)
//				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);
//		}
//		
//		/*  下主控&遥控器指示灯  */
//		if ((slave.work_state == DEV_ONLINE) && (rc_sensor.work_state == DEV_ONLINE))
//		{
//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
//		}
//		else
//		{
//			if ((slave.work_state == DEV_OFFLINE) && (rc_sensor.work_state == DEV_OFFLINE))
//			{
//				if (i % 150 == 0)
//					HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_2);
//			}
//			else
//			{
//				if (slave.work_state == DEV_OFFLINE)
//				{
//					if (i % 1000 == 0)
//						HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_2);
//				}
//				if (rc_sensor.work_state == DEV_OFFLINE)
//				{
//					if (i % 500 == 0)
//						HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_2);
//				}
//			}
//		}
		
		if (slave.work_state == DEV_ONLINE)
		{
			HAL_GPIO_WritePin(GPIOD, RBG_G_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, RBG_B_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, RBG_R_Pin, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOD, RBG_G_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, RBG_B_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, RBG_R_Pin, GPIO_PIN_SET);
		}
		
    osDelay(1);
  }
}
