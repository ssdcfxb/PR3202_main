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

//uint16_t i = 0;

short ggx, ggy, ggz;
short aax, aay, aaz;

//临时变量
short ggx_, ggy_, ggz_;
short aax_, aay_, aaz_;
float pitch_ex, roll_ex, yaw_ex;
float pitch, roll, yaw;

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
	
	/* 获取陀螺仪数据 */
	if(imu_sen->driver.tpye == DR_SPI1)
	{
		EX_BMI_Get_RawData(&ggx, &ggy, &ggz, &aax, &aay, &aaz);
	}
	else if(imu_sen->driver.tpye == DR_SPI2)
	{
		BMI_Get_RawData(&ggx, &ggy, &ggz, &aax, &aay, &aaz);
	}
//	if (++i == 60000)
//		i = 0;
//	if (i % 2 == 1)
//	{
//		BMI_Get_RawData(&ggx, &ggy, &ggz, &aax, &aay, &aaz);
//		ggx_=ggx,ggy_=ggy,ggz_=ggz,aax_=aax,aay_=aay,aaz_=aaz;
//		
		/* 原始数据低通滤波 */
//		gyrox_ = lowpass(gyrox_, ggx_, 0.3);
//		gyroy_ = lowpass(gyroy_, ggy_, 0.3);
//		gyroz_ = lowpass(gyroz_, ggz_, 0.3);
//		accx_ = lowpass(accx_, aax_, 1);
//		accy_ = lowpass(accy_, aay_, 1);
//		accz_ = lowpass(accz_, aaz_, 1);
		
		/* 解算陀螺仪数据 */
//		BMI_Get_EulerAngle(&pitch_ex, &roll_ex, &yaw_ex,\
//											 &pitch_, &roll_, &yaw_, \
//											 &gyrox_, &gyroy_, &gyroz_, \
//											 &accx_, &accy_, &accz_);
//	}
//	else
//	{
//		EX_BMI_Get_RawData(&ggx, &ggy, &ggz, &aax, &aay, &aaz);
	imu_info->raw_info.acc_x = aax;
	imu_info->raw_info.acc_y = aay;
	imu_info->raw_info.acc_z = aaz;
	imu_info->raw_info.gyro_x = ggx;
	imu_info->raw_info.gyro_y = ggy;
	imu_info->raw_info.gyro_z = ggz;
	
	/* 坐标系变换 */
	Vector_Transform(ggx, ggy, ggz, aax, aay, aaz,\
	                 &gyrox, &gyroy, &gyroz, &accx, &accy, &accz);
	
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
										 
	pitch = imu_info->base_info.pitch, roll = imu_info->base_info.roll, yaw = imu_info->base_info.yaw;
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
//	}
}
