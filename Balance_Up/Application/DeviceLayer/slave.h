
#ifndef __SLAVE_H
#define __SLAVE_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef __packed struct slv_tx_info_struct {

	uint8_t SOF;
	uint8_t data_length;
	uint8_t CRC8;
	
	uint8_t  rc_info[16];
	uint8_t  magz_mode;  //0:¹Ø 1:¿ª
	int16_t  angle;
	uint16_t CRC16;

} slv_tx_info_t;

typedef __packed struct slv_rx_info_struct {

	uint8_t SOF;
	uint8_t data_length;
	uint8_t CRC8;
	
	uint16_t shooter_cooling_limit;
	uint16_t shooter_speed_limit;
	uint16_t shooter_cooling_heat;
	float    bullet_speed;
	uint16_t CRC16;

} slv_rx_info_t;

typedef struct slave_info_struct {

	slv_tx_info_t *tx_info;
	slv_rx_info_t *rx_info;
	
	uint16_t shooter_cooling_limit;
	uint16_t shooter_speed_limit;
	uint16_t shooter_cooling_heat;
	float    bullet_speed;
	
	uint8_t  rx_flag;
	int16_t	 offline_cnt;
	int16_t	 offline_max_cnt;

} slave_info_t;


typedef struct slv_struct {
	
	slave_info_t *info;
	drv_uart_t	 *driver;
	
	bool				(*tx)(struct slv_struct *self);
	void				(*rx)(struct slv_struct *self, uint8_t *rxBuf);
	void				(*init)(struct slv_struct *self);
	void				(*heart_beat)(struct slv_struct *self);
	
	dev_work_state_t	work_state;
} slave_t;

extern slave_t slave;

/* Exported functions --------------------------------------------------------*/

#endif
