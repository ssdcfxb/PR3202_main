/* Includes ------------------------------------------------------------------*/
#include "drv_tim.h"
#include "rp_config.h"


//extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//uint16_t val_open = 70;
//uint16_t val_clse = 106;
#if defined (CAR) && (CAR == 1)
uint16_t val_open = 50;
uint16_t val_clse = 120;
#elif defined (CAR) && (CAR == 2)
uint16_t val_open = 120;
uint16_t val_clse = 50;
#endif

uint16_t val_slep = 0;
/* Exported variables --------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void TIM3_Init(void)
{
	HAL_TIM_Base_Init(&htim3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

//¾É
//void TIM5_Init(void)
//{
//	HAL_TIM_Base_Init(&htim5);
//	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
//}

void TIM4_Init(void)
{
	HAL_TIM_Base_Init(&htim4);
	HAL_TIM_Base_Start_IT(&htim4);
}

/* Servo functions */
void Magazine_Open(void)
{
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, val_open);
}

void Magazine_Close(void)
{
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, val_clse);
}

void Magazine_Sleep(void)
{
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, val_slep);
}

//¾É
//void Magazine_Open(void)
//{
//	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, val_open);
//}

//void Magazine_Close(void)
//{
//	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, val_clse);
//}

//void Magazine_Sleep(void)
//{
//	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, val_slep);
//}

