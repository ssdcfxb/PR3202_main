
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
	
	int16_t  rc_ch_ws_val;
	int16_t  rc_ch_ad_val;
	uint16_t status;	//1:遥控器(0:关 1:开) 2:弹仓(0:关 1:开) 3:小陀螺(0:关 1:开) 4:换头(0:前5110 1:后1014) 5:换头标志位(0:完成换头 1:开始换头) 6:自瞄(0:关 1:开) 7:复位(0:关 1:开) 8:侧身(0:关 1:开)
	//9:加速(0:关 1:开) 10:打符(0:关 1:开)	11:陀螺仪状态(0:正常 1:有问题) 12:左右标志位(0:左 1:右) 13:卡墙处理(0:关 1:开) 14:打符模式(0:小符 1:大符)
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
	
	__packed union
	{
		uint8_t  info;
		__packed struct 
		{
			uint8_t  my_color : 1; //红:0 蓝:1
			uint8_t  balance_info : 7; // 敌方平衡步编号和
		} bit;
	} car_info;
	
	uint8_t	 blood_1;
	uint8_t	 blood_2;
	uint8_t	 blood_3;
	uint8_t	 blood_4;
	uint8_t	 blood_5;
	uint8_t	 blood_6;
	uint8_t	 blood_7;
	uint8_t	 blood_8;
	
	uint16_t CRC16;

} slv_rx_info_t;

typedef struct slave_info_struct {

	slv_tx_info_t *tx_info;
	slv_rx_info_t *rx_info;
	
	uint16_t shooter_cooling_limit;
	uint16_t shooter_speed_limit;
	uint16_t shooter_cooling_heat;
	float    bullet_speed;
	
	union
	{
		uint8_t  info;
		struct
		{
			uint8_t  my_color : 1; //红:0 蓝:1
			uint8_t  balance_info : 7; // 敌方平衡步编号和
		} bit;
	} car_info;
	
	uint8_t	 blood_1;
	uint8_t	 blood_2;
	uint8_t	 blood_3;
	uint8_t	 blood_4;
	uint8_t	 blood_5;
	uint8_t	 blood_6;
	uint8_t	 blood_7;
	uint8_t	 blood_8;
	uint8_t	 size_3;
	uint8_t	 size_4;
	uint8_t	 size_5;
	
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
