#ifndef __DRV_CAN_H
#define __DRV_CAN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#include "rp_driver_config.h"
#include "rm_protocol.h"
#include "device.h"

#define CHASSIS_CAN_ID_LF	RM3508_CAN_ID_201
#define CHASSIS_CAN_ID_RF	RM3508_CAN_ID_202
#define CHASSIS_CAN_ID_LB	RM3508_CAN_ID_203
#define CHASSIS_CAN_ID_RB	RM3508_CAN_ID_204

typedef struct 
{
  CAN_RxHeaderTypeDef Header;
  uint8_t Data[8];
}can_rx_info_t;


extern uint8_t can1_tx_buf[16];
extern uint8_t can2_tx_buf[16];

void CAN_Filter_Init(void);
HAL_StatusTypeDef CAN_SendData(CAN_HandleTypeDef *hcan, uint32_t stdId, uint8_t *dat);
void CAN_Tx_Cmd(CAN_HandleTypeDef *hcan, uint32_t identifier, int16_t data_1,
								int16_t data_2, int16_t data_3, int16_t data_4);


#endif
