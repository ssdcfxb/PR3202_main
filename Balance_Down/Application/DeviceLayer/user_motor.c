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
KT_motor_t KT_motor[KT_MOTOR_LIST] =
{
	[LEG_L] = {
		
		.KT_motor_info.id.tx_id = KT9025_CAN_ID_141,
		.KT_motor_info.id.rx_id = KT9025_CAN_ID_141,
		.KT_motor_info.id.drive_type = M_CAN1,
		.KT_motor_info.id.motor_type = KT9025,
		
		.init = KT_motor_class_init,
	},
	
	[LEG_R] = {
		
		.KT_motor_info.id.tx_id = KT9025_CAN_ID_142,
		.KT_motor_info.id.rx_id = KT9025_CAN_ID_142,
		.KT_motor_info.id.drive_type = M_CAN1,
  	.KT_motor_info.id.motor_type = KT9025,
		
		.init = KT_motor_class_init,
	},
};

//����ID���ڵ���init�������Զ���ȡ
RM_motor_t MOM_motor[MOM_MOTOR_LIST] = 
{
	[MOMENTUM_L] = {
	
		.id.rx_id      		 = GM6020_CAN_ID_209,
		.id.drive_type		 = M_CAN2,
		.id.motor_type		 = GM6020,
		.mec_info.mec_mid  = MOMENTUM_L_B_POS - MOMENTUM_L_POS_OFFSET,
		
		.init          = motor_class_init,
	},
	
	[MOMENTUM_R] = {
	
		.id.rx_id      		 = GM6020_CAN_ID_20A,
		.id.drive_type 		 = M_CAN2,
		.id.motor_type 		 = GM6020,
		.mec_info.mec_mid  = MOMENTUM_R_B_POS - MOMENTUM_R_POS_OFFSET,
		
		.init          = motor_class_init,
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
void KT_motor_init(void)
{
	//�����ʼ��
	KT_motor[LEG_L].init(&KT_motor[LEG_L]);
	KT_motor[LEG_R].init(&KT_motor[LEG_R]);
	
}

/**
 *	@brief	RM����ܳ�ʼ��
 */
void RM_motor_init(void)
{
	
	MOM_motor[MOMENTUM_L].init(&MOM_motor[MOMENTUM_L]);
	MOM_motor[MOMENTUM_R].init(&MOM_motor[MOMENTUM_R]);
	
}

/**
 *	@brief  ������е��������
 */
void All_motor_heartbeat(void)
{
	MOM_motor[MOMENTUM_L].heartbeat(&MOM_motor[MOMENTUM_L]);
	MOM_motor[MOMENTUM_R].heartbeat(&MOM_motor[MOMENTUM_R]);
	
	
	KT_motor[LEG_L].heartbeat(&KT_motor[LEG_L]);
	KT_motor[LEG_R].heartbeat(&KT_motor[LEG_R]);
	
}

