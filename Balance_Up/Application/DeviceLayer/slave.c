
/* Includes ------------------------------------------------------------------*/
#include "slave.h"
#include "string.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
extern bool slave_send_data(slave_t *slef);
extern void slave_receive_data(slave_t *slef, uint8_t *rxBuf);
void slave_init(slave_t *slef);
void slave_heart_beat(slave_t *slef);
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
drv_uart_t slave_driver = {
	.id = DRV_UART3,
	.tx_byte = NULL,
};

slv_tx_info_t slave_tx_info = {
	.SOF = 0xA5,
};

slv_rx_info_t slave_rx_info;

slave_info_t info = {
	.tx_info = &slave_tx_info,
	.rx_info = &slave_rx_info,
	.offline_max_cnt = 50,
	.rx_flag = 0,
};
/* Exported variables --------------------------------------------------------*/
slave_t slave = {
	.info = &info,
	.tx = slave_send_data,
	.rx = slave_receive_data,
	.init = slave_init,
	.heart_beat = slave_heart_beat,
};

/* Private functions ---------------------------------------------------------*/
void slave_init(slave_t *slef)
{
	slv_tx_info_t *tx_info = slef->info->tx_info;
	slv_rx_info_t *rx_info = slef->info->rx_info;
	
	tx_info->data_length = 0;
	tx_info->rc_work_state = 0;
	tx_info->rc_ch_ws_val = 0;
	tx_info->rc_ch_ad_val = 0;
	tx_info->magz_mode = 0;
	tx_info->imu_angle = 0.f;
	tx_info->motor_angle = 0;
	
	rx_info->data_length = 0;
	rx_info->shooter_cooling_limit = 0;
	rx_info->shooter_speed_limit = 0;
	rx_info->shooter_cooling_heat = 0;
	rx_info->bullet_speed = 0.f;
	
}


void slave_heart_beat(slave_t *slef)
{
	slave_info_t *slv_info = slef->info;

	slv_info->offline_cnt++;
	if(slv_info->offline_cnt > slv_info->offline_max_cnt) {
		slv_info->offline_cnt = slv_info->offline_max_cnt;
		slef->work_state = DEV_OFFLINE;
	} 
	else {
		/* ÀëÏß->ÔÚÏß */
		if(slef->work_state == DEV_OFFLINE)
		{
			slef->work_state = DEV_ONLINE;
		}
	}
}


/* Exported functions --------------------------------------------------------*/
