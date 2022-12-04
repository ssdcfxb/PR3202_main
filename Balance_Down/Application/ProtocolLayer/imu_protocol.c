/* Includes ------------------------------------------------------------------*/
#include "imu_sensor.h"

#include "imu_protocol.h"

//#include "BMI.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ave_filter_t imu_pitch_dif_speed_ave_filter;
ave_filter_t imu_roll_dif_speed_ave_filter;
ave_filter_t imu_yaw_dif_speed_ave_filter;
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
short gyrox, gyroy, gyroz;
short accx, accy, accz;
float gyrox_, gyroy_, gyroz_;
float accx_, accy_, accz_;
float pitch_, roll_, yaw_;
void imu_update(imu_sensor_t *imu_sen)
{
  imu_info_t *imu_info = imu_sen->info;
	
	/* 获取陀螺仪数据 */
  BMI_Get_RawData(&gyrox, &gyroy, &gyroz, &accx, &accy, &accz);
	
	
	
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
	if (abs(imu_info->base_info.rate_pitch) > 180.0f)
	{
		imu_info->base_info.rate_pitch -= one(imu_info->base_info.rate_pitch) * 360.0f;
	}
	imu_info->base_info.rate_pitch *= 1000.0f;
	imu_info->base_info.ave_rate_pitch = ave_fil_update(&imu_pitch_dif_speed_ave_filter, imu_info->base_info.rate_pitch, 3);
	
	//roll
	imu_info->base_info.rate_roll = roll_;
	if (abs(imu_info->base_info.rate_roll) > 180.0f)
	{
		imu_info->base_info.rate_roll -= one(imu_info->base_info.rate_roll) * 360.0f;
	}
	imu_info->base_info.rate_roll *= 1000.0f;
	imu_info->base_info.ave_rate_roll = ave_fil_update(&imu_roll_dif_speed_ave_filter, imu_info->base_info.rate_roll, 3);
	
	//yaw
	imu_info->base_info.rate_yaw = yaw_;
	if (abs(imu_info->base_info.rate_yaw) > 180.0f)
	{
		imu_info->base_info.rate_yaw -= one(imu_info->base_info.rate_yaw) * 360.0f;
	}
	imu_info->base_info.rate_yaw *= 1000.0f;
	imu_info->base_info.ave_rate_yaw = ave_fil_update(&imu_yaw_dif_speed_ave_filter, imu_info->base_info.rate_yaw, 3);
	
	
	imu_sen->work_state.offline_cnt = 0;
}
