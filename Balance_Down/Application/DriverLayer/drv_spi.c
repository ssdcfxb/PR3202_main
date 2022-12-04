/* Includes ------------------------------------------------------------------*/
#include "drv_spi.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void SPI2_Init(void)
{
	HAL_SPI_Init(&hspi2);
	HAL_GPIO_WritePin(BMI_CS_GPIO_Port, BMI_CS_Pin, GPIO_PIN_SET);
}


//extern uint8_t rev[12];
//void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
//{
//	SPI2_rxDataHandler(rev);
//}

