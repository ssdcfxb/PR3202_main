/* Includes ------------------------------------------------------------------*/
#include "drv_tim.h"

extern TIM_HandleTypeDef htim4;

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void TIM4_Init(void)
{
	HAL_TIM_Base_Init(&htim4);
	HAL_TIM_Base_Start_IT(&htim4);
}

