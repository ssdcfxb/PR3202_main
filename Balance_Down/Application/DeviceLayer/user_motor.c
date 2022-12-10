/**
 * @file        user_motor.c
 * @author      SSDCFXB
 * @Version     V1.0
 * @date        18-November-2022
 * @brief       User Motor Devives Config
 * @update
 */

/* Includes ------------------------------------------------------------------*/
#include "user_motor.h"

/*
* ʹ�õ��ǰ�������úõ�������͡�id�š�������ʽ
*/
KT_motor_t KT_motor[MOTOR_LIST] =
{
	[LEG_L] = {
		
		.KT_motor_info.id.tx_id = 0x141,
		.KT_motor_info.id.rx_id = 0x141,
		.KT_motor_info.id.drive_type = M_CAN1,
		.KT_motor_info.id.motor_type = KT9025,
		
		.init = KT_motor_class_init,
	},
	
	[LEG_R] = {
		
		.KT_motor_info.id.tx_id = 0x142,
		.KT_motor_info.id.rx_id = 0x142,
		.KT_motor_info.id.drive_type = M_CAN1,
  	.KT_motor_info.id.motor_type = KT9025,
		
		.init = KT_motor_class_init,
	},
};


/* Exported variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/


/**
 *	@brief	KT����ܳ�ʼ��
 */
void KT_motor_all_init(void)
{
	//�����ʼ��
	KT_motor[LEG_L].init(&KT_motor[LEG_L]);
	KT_motor[LEG_R].init(&KT_motor[LEG_R]);
	
	
}




