/* Includes ------------------------------------------------------------------*/
#include "drv_tim.h"

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint16_t val_open = 30;
uint16_t val_clse = 75;
/* Exported variables --------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void TIM3_Init(void)
{
	HAL_TIM_Base_Init(&htim3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
}

void TIM4_Init(void)
{
	HAL_TIM_Base_Init(&htim4);
	HAL_TIM_Base_Start_IT(&htim4);
}

/* Servo functions */
void Magazine_Open(void)
{
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, val_open);
}

void Magazine_Close(void)
{
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, val_clse);
}

