#ifndef __DRV_SPI_H
#define __DRV_SPI_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "imu_sensor.h"

#include "main.h"
/* Exported types ------------------------------------------------------------*/
extern SPI_HandleTypeDef hspi2;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
//void DRV_UART_IRQHandler(UART_HandleTypeDef *huart);
void SPI2_Init(void);

#endif
