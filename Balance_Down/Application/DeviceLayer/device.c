/**
 * @file        device.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        15-September-2020
 * @brief       Devices' Manager.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "device.h"


/* Private macro -------------------------------------------------------------*/
void Judge_Imu_Is_Correct_OK(void);
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/


dev_list_t dev_list = {
	.imu_sen    = &imu_sensor,
	.rc_sen     = &rc_sensor,
//	.kt_motor   = KT_motor,
//	.rm_motor   = MOM_motor,
};

/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void DEVICE_Init(void)
{
	//�����ǵĳ�ʼ��������driver.c�У��ӿ��ʼ�����
	
	dev_list.rc_sen->init(dev_list.rc_sen);
	
	KT_motor_init();
	
	RM_motor_init();
	
	//��ʱ,ʹ�������ǽ������
	HAL_Delay(2000);

}
