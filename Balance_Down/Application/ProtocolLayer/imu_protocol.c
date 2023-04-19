/* Includes ------------------------------------------------------------------*/
#include "imu_sensor.h"
#include "imu_protocol.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ave_filter_t imu_pitch_dif_speed_ave_filter;
ave_filter_t imu_roll_dif_speed_ave_filter;
ave_filter_t imu_yaw_dif_speed_ave_filter;

float rate_pitch, rate_yaw, rate_roll, pitch, yaw, roll;

short gyrox, gyroy, gyroz;
short accx, accy, accz;
float gyrox_, gyroy_, gyroz_;
float accx_, accy_, accz_;
float pitch_, roll_, yaw_;
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void imu_update(imu_sensor_t *imu_sen)
{
  imu_info_t *imu_info = imu_sen->info;
	short gyro_tp, acc_tp;
	
	/* 获取陀螺仪数据 */
  BMI_Get_RawData(&gyrox, &gyroy, &gyroz, &accx, &accy, &accz);
	
	imu_info->raw_info.acc_x = accx;
	imu_info->raw_info.acc_y = accy;
	imu_info->raw_info.acc_z = accz;
	imu_info->raw_info.gyro_x = gyrox;
	imu_info->raw_info.gyro_y = gyroy;
	imu_info->raw_info.gyro_z = gyroz;

	/* 坐标系变换 */
	acc_tp = accx;
	accx   = -accy;
	accy   = acc_tp;
	gyro_tp = gyrox;
	gyrox   = -gyroy;
	gyroy   = gyro_tp;
	
	
	/* 原始数据低通滤波 */
	gyrox_ = lowpass(gyrox_, gyrox, 0.3);
	gyroy_ = lowpass(gyroy_, gyroy, 0.3);
	gyroz_ = lowpass(gyroz_, gyroz, 0.3);
	
	accx_ = lowpass(accx_, accx, 1);
	accy_ = lowpass(accy_, accy, 1);
	accz_ = lowpass(accz_, accz, 1);
	
	
	/* 解算陀螺仪数据 */
  BMI_Get_EulerAngle(&imu_info->base_info.pitch, &imu_info->base_info.roll, &imu_info->base_info.yaw,\
										 &pitch_, &roll_, &yaw_, \
										 &gyrox_, &gyroy_, &gyroz_, \
										 &accx_, &accy_, &accz_);
    
	/* 计算陀螺仪数据 */
	//pitch
	imu_info->base_info.rate_pitch = pitch_;
	if (abs_(imu_info->base_info.rate_pitch) > 180.0f)
	{
		imu_info->base_info.rate_pitch -= one(imu_info->base_info.rate_pitch) * 360.0f;
	}
	imu_info->base_info.rate_pitch *= 2000.0f;
	imu_info->base_info.ave_rate_pitch = ave_fil_update(&imu_pitch_dif_speed_ave_filter, imu_info->base_info.rate_pitch, 3);
	
	rate_pitch = imu_info->base_info.ave_rate_pitch;
	pitch = imu_info->base_info.pitch;
	//roll
	imu_info->base_info.rate_roll = roll_;
	if (abs_(imu_info->base_info.rate_roll) > 180.0f)
	{
		imu_info->base_info.rate_roll -= one(imu_info->base_info.rate_roll) * 360.0f;
	}
	imu_info->base_info.rate_roll *= 2000.0f;
	imu_info->base_info.ave_rate_roll = ave_fil_update(&imu_roll_dif_speed_ave_filter, imu_info->base_info.rate_roll, 3);
	
	rate_roll = imu_info->base_info.ave_rate_roll;
	roll = imu_info->base_info.roll;
	//yaw
	imu_info->base_info.rate_yaw = yaw_;
	if (abs_(imu_info->base_info.rate_yaw) > 180.0f)
	{
		imu_info->base_info.rate_yaw -= one(imu_info->base_info.rate_yaw) * 360.0f;
	}
	imu_info->base_info.rate_yaw *= 2000.0f;
	imu_info->base_info.ave_rate_yaw = ave_fil_update(&imu_yaw_dif_speed_ave_filter, imu_info->base_info.rate_yaw, 3);
	
	rate_yaw = imu_info->base_info.ave_rate_yaw;
	yaw = imu_info->base_info.yaw;
	
	imu_sen->work_state.offline_cnt = 0;
}
