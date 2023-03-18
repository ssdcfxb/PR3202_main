 
/* Includes ------------------------------------------------------------------*/
#include "slave_protocol.h"

#include "crc.h"
#include "slave.h"
#include "rc_sensor.h"
#include "judge.h"
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
extern void judge_update(judge_t *self, slave_info_t *info);
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t slave_txBuf[30];
/* Exported variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart3;
extern slv_tx_info_t slave_tx_info;
extern slv_rx_info_t slave_rx_info;

/* Private functions ---------------------------------------------------------*/

void USART3_rxDataHandler(uint8_t *rxBuf)
{
	slave.rx(&slave, rxBuf);
}

/* Exported functions --------------------------------------------------------*/

bool slave_send_data(slave_t *slef)
{
	slave_tx_info.rc_work_state = 0;
	if(rc_sensor.work_state == DEV_ONLINE)
		slave_tx_info.rc_work_state = 1;
	
	slave_tx_info.data_length = sizeof(slv_tx_info_t);
	memcpy(slave_txBuf, &slave_tx_info, sizeof(slv_tx_info_t));
	Append_CRC8_Check_Sum(slave_txBuf, 3);
	slave_tx_info.CRC8 = slave_txBuf[2];
	Append_CRC16_Check_Sum(slave_txBuf, sizeof(slv_tx_info_t));
	slave_tx_info.CRC16 = (uint16_t)((slave_txBuf[sizeof(slv_tx_info_t)-2] << 8)
                                   | slave_txBuf[sizeof(slv_tx_info_t)-1]);
	
	memset(&slave_tx_info.data_length, 0, (sizeof(slv_tx_info_t) - 1));
	
	if(HAL_UART_Transmit_DMA(&huart3,slave_txBuf,sizeof(slv_tx_info_t)) == HAL_OK)
	{
			return true;
	}
	return false;
	
}


void slave_receive_data(slave_t *slef, uint8_t *rxBuf)
{
	slv_rx_info_t *rx_info = slef->info->rx_info;
	slave_info_t *info = slef->info;
	
	info->offline_cnt = 0;
	
	slef->info->rx_flag = 0;
	if(rxBuf[0] == 0xA5)
	{
		if(Verify_CRC8_Check_Sum(rxBuf, 3) == true)
		{
			if(Verify_CRC16_Check_Sum(rxBuf, sizeof(slv_rx_info_t)) == true)
			{
				memcpy(rx_info, rxBuf, sizeof(slv_rx_info_t));
				slef->info->rx_flag = 1;
			}
		}
	}
	
	if (info->rx_flag == 1)
	{
		memcpy(&info->shooter_cooling_limit, (const void*)&rx_info->shooter_cooling_limit, 2);
		memcpy(&info->shooter_speed_limit, (const void*)&rx_info->shooter_speed_limit, 2);
		memcpy(&info->shooter_cooling_heat, (const void*)&rx_info->shooter_cooling_heat, 2);
		memcpy(&info->bullet_speed, (void*)&rx_info->bullet_speed, 4);
		
		judge_update(&judge, info);
	}
	
	
	if(rxBuf[sizeof(slv_rx_info_t)] == 0xA5)
	{
		slave_receive_data(&slave, &rxBuf[sizeof(slv_rx_info_t)]);
	}
}

