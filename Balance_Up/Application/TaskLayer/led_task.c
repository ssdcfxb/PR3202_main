#include "led_task.h"

void StartLedTask(void const * argument)
{
	static	uint16_t i = 0;
	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
  for(;;)
  {
		if (++i == 60000)
			i = 0;
		
		/*  陀螺仪指示灯  */
		if (imu_sensor.work_state.err_code == IMU_NONE_ERR)
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
		}
		else
		{
			if (i % 200 == 0)
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_1);
		}
		
		/*  云台电机指示灯  */
		if ((RM_motor[GIM_Y].state.work_state == M_ONLINE) && (RM_motor[GIM_P].state.work_state == M_ONLINE))
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
		}
		else
		{
			if (i % 500 == 0)
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_2);
		}
		
		/*  下主控指示灯  */
		if (slave.work_state == DEV_ONLINE)
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
		}
		else
		{
			if (i % 1000 == 0)
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);
		}
		
    osDelay(1);
  }
}
