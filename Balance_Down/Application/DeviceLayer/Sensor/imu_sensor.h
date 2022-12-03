#ifndef __IMU_H
#define __IMU_H

#include "rp_config.h"
#include "BMI.h"
#include "drv_tick.h"

typedef enum {
	IMU_NONE_ERR,
  IMU_TYPE_ERR,
  IMU_ID_ERR,
  IMU_INIT_ERR,
  IMU_DATA_ERR,
} imu_err_e;


typedef enum{
	DR_SPI1,
	DR_SPI2,
	DR_SPI3,
	
	DR_IIC,
	
}drv_type_e;

typedef struct drive_str{
	drv_type_e tpye;
	
	int8_t (*send)(struct drive_str *self, uint8_t *Txbuff, uint16_t len);
	int8_t (*read)(struct drive_str *self, uint8_t *Rxbuff, uint16_t len);
	int8_t (*sendread)(struct drive_str *self, uint8_t *Txbuff, uint8_t *Rxbuff, uint16_t len);
}driver_t;


typedef struct work_state_struct {
	dev_work_state_t dev_state;
	imu_err_e	       err_code;
	
	uint8_t   offline_cnt;
	uint8_t   offline_max_cnt;
	
} work_state_t;



typedef struct{
	float acc_x;
	float acc_y;
	float acc_z;
	
	float gyro_x;
	float gyro_y;
	float gyro_z;	
	
//	float rate_yaw_offset;
//	float rate_pitch_offset;	
//	float rate_roll_offset;

}	raw_info_t;

typedef struct{
	float yaw;
	float pitch;
	float roll;
	
	float rate_yaw;
	float rate_pitch;
	float rate_roll;	
	
	float ave_rate_yaw;
	float ave_rate_pitch;	
	float ave_rate_roll;

}	base_info_t;

typedef struct imu_info_struct {

	raw_info_t  raw_info;

	base_info_t base_info;
	
	uint8_t		  init_flag;

} imu_info_t;



typedef struct imu_struct {
	
  bmi_t       *bmi;	
	imu_info_t	info;
	driver_t	  driver;
	
	void				(*init)(struct imu_struct *self);
	void				(*update)(struct imu_struct *self, uint8_t *rxBuf);
	void				(*heart_beat)(struct work_state_struct *self);
	
	work_state_t  work_state;
	dev_id_t		   id;	
} imu_sensor_t;

void Get_IMU_Data(void);
void TIM4_Init_Handle(uint16_t timeout);
void BMI_DATA(void);

extern imu_sensor_t imu_sensor;

#endif


