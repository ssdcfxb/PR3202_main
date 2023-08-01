#ifndef __LED_TASK
#define __LED_TASK

#include "cmsis_os.h"
#include "main.h"
#include "module.h"

#define LED_RED_ON()		(HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET))
#define LED_RED_OFF()		(HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET))
#define LED_RED_TOGGLE()	(HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin))

#define LED_GREEN_ON()		(HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET))
#define LED_GREEN_OFF()		(HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET))
#define LED_GREEN_TOGGLE()	(HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin))

#define LED_BLUE_ON()		(HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET))
#define LED_BLUE_OFF()		(HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET))
#define LED_BLUE_TOGGLE()	(HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin))

#define BLACK_ON()          LED_RED_OFF(); LED_GREEN_OFF();LED_BLUE_OFF()
#define RED_ON()            LED_RED_ON(); LED_GREEN_OFF();LED_BLUE_OFF()
#define GREEN_ON()          LED_RED_OFF(); LED_GREEN_ON();LED_BLUE_OFF()
#define BLUE_ON()           LED_RED_OFF(); LED_GREEN_OFF();LED_BLUE_ON()
#define YELLOW_ON()         LED_RED_ON(); LED_GREEN_ON();LED_BLUE_OFF()
#define PINK_ON()           LED_RED_ON(); LED_GREEN_OFF();LED_BLUE_ON()
#define LIGHT_ON()          LED_RED_OFF(); LED_GREEN_ON();LED_BLUE_ON()
#define WHITE_ON()          LED_RED_ON(); LED_GREEN_ON();LED_BLUE_ON()

void StartLedTask(void const * argument);

#endif
