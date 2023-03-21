
#ifndef __SLAVE_H
#define __SLAVE_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

// 小陀螺状态枚举
typedef enum
{
	On_Gyro,        // 开启
	Off_Gyro,       // 关闭
	WaitCommond_Gyro // 等待指令
} gyro_status_e;

typedef __packed struct slv_tx_info_struct {

	uint8_t SOF;
	uint8_t data_length;
	uint8_t CRC8;
	
	int16_t  rc_ch_ws_val;
	int16_t  rc_ch_ad_val;
	uint8_t  status;	//1:遥控器(0:关 1:开) 2:弹仓(0:关 1:开) 3:小陀螺(0:关 1:开) 4:换头(0:7191 1:3095)
	float    imu_angle;
	int16_t  motor_angle;
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
	
	uint8_t  			rx_flag;
	gyro_status_e	gyro_status;
	
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
