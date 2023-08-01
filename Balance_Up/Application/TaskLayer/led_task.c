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
