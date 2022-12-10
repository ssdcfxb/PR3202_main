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

RM_motor_t RM_motor[MOM_MOTOR_LIST] = 
{
	[MOMENTUM_L] = {
	
		.id.rx_id      = GM6020_CAN_ID_205,
		.id.drive_type = M_CAN2,
		.id.motor_type = GM6020,
		
		.init          = motor_class_init,
	},
	
	[MOMENTUM_R] = {
	
		.id.rx_id      = GM6020_CAN_ID_206,
		.id.drive_type = M_CAN2,
		.id.motor_type = GM6020,
		
		.init          = motor_class_init,
	},
};


/*
	float	  kp;
	float 	ki;
	float 	kd;
	
	float   blind_err;	
	float 	integral_max;	
	float   iout_max;
	float 	out_max;
*/
float momentum_L_angle_pid_param[7]    = {0,0,0,0,0,0,0};
float momentum_L_angle_in_pid_param[7] = {0,0,0,0,0,0,0};

float momentum_R_angle_pid_param[7]    = {0,0,0,0,0,0,0};
float momentum_R_angle_in_pid_param[7] = {0,0,0,0,0,0,0};

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
	
	RM_motor[MOMENTUM_L].init(&RM_motor[MOMENTUM_L]);
	RM_motor[MOMENTUM_R].init(&RM_motor[MOMENTUM_R]);
	
	
	//PID��ʼ��
	RM_motor[MOMENTUM_L].pid_init(&RM_motor[MOMENTUM_L].pid.angle, momentum_L_angle_pid_param);
	RM_motor[MOMENTUM_L].pid_init(&RM_motor[MOMENTUM_L].pid.angle_in, momentum_L_angle_in_pid_param);
	
	RM_motor[MOMENTUM_R].pid_init(&RM_motor[MOMENTUM_R].pid.angle, momentum_R_angle_pid_param);
	RM_motor[MOMENTUM_R].pid_init(&RM_motor[MOMENTUM_R].pid.angle_in, momentum_R_angle_in_pid_param);
	
}



