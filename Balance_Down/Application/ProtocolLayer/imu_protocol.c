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

short ggx, ggy, ggz;
short aax, aay, aaz;
float gyrox, gyroy, gyroz;
float accx, accy, accz;
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
	if(imu_sen->driver.tpye == DR_SPI1)
	{
		EX_BMI_Get_RawData(&ggx, &ggy, &ggz, &aax, &aay, &aaz);
	}
	else if(imu_sen->driver.tpye == DR_SPI2)
	{
		BMI_Get_RawData(&ggx, &ggy, &ggz, &aax, &aay, &aaz);
	}
	
	imu_info->raw_info.acc_x = aax;
	imu_info->raw_info.acc_y = aay;
	imu_info->raw_info.acc_z = aaz;
	imu_info->raw_info.gyro_x = ggx;
	imu_info->raw_info.gyro_y = ggy;
	imu_info->raw_info.gyro_z = ggz;
	
	/* 坐标系变换 */
//	Vector_Transform(ggx, ggy, ggz, aax, aay, aaz,\
//	                 &gyrox, &gyroy, &gyroz, &accx, &accy, &accz);
	
	accx = aax;
	accy = aay;
	gyrox = ggx;
	gyroy = ggy;
	
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
	imu_info->base_info.ave_rate_pitch = ave_fil_update(&imu_pitch_dif_speed_ave_filter, imu_info->base_info.rate_pitch, 3);
	
	//roll
	imu_info->base_info.rate_roll = roll_;
	imu_info->base_info.ave_rate_roll = ave_fil_update(&imu_roll_dif_speed_ave_filter, imu_info->base_info.rate_roll, 3);
	
	//yaw
	imu_info->base_info.rate_yaw = yaw_;
	imu_info->base_info.ave_rate_yaw = ave_fil_update(&imu_yaw_dif_speed_ave_filter, imu_info->base_info.rate_yaw, 3);
	
	imu_sen->work_state.offline_cnt = 0;
}
