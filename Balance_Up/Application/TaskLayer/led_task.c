#include "led_task.h"

void StartLedTask(void const * argument)
{
  for(;;)
  {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
		if (imu_sensor.work_state.dev_state == DEV_ONLINE)
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
		}
		if (rc_sensor.work_state == DEV_ONLINE)
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
		}
		if (slave.work_state == DEV_ONLINE)
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
		}
    osDelay(1);
  }
}
