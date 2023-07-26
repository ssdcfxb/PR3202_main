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

//����ID���ڵ���init�������Զ���ȡ
RM_motor_t RM_motor[MOTOR_LIST] = 
{
	[FRIC_R] = {
	
		.id.drive_type = M_CAN1,
		.id.motor_type = RM3508,
		.id.rx_id = 0x201,
		
		.init = motor_class_init,
	},
	[FRIC_L] = {
	
		.id.drive_type = M_CAN1,
		.id.motor_type = RM3508,
		.id.rx_id = 0x202,
		
		.init = motor_class_init,
	},
	[DIAL] = {
	
		.id.drive_type = M_CAN1,
		.id.motor_type = RM2006,
		.id.rx_id = 0x203,
		
		.init = motor_class_init,
	},
#if defined (CAR)	&& (CAR == 0)
	[GIM_P] = {
	
		.id.drive_type = M_CAN2,
		.id.motor_type = GM6020,
		.id.rx_id = 0x205,
		
		.init = motor_class_init,
	},
	[GIM_Y] = {
	
		.id.drive_type = M_CAN2,
		.id.motor_type = GM6020,
		.id.rx_id = 0x206,
		
		.init = motor_class_init,
	},
#elif defined (CAR) && ((CAR == 1) || (CAR == 2))
	
	[GIM_P] = {
	
		.id.drive_type = M_CAN2,
		.id.motor_type = GM6020,
		.id.rx_id = 0x206,
		
		.init = motor_class_init,
	},
	[GIM_Y] = {
	
		.id.drive_type = M_CAN2,
		.id.motor_type = GM6020,
		.id.rx_id = 0x205,
		
		.init = motor_class_init,
	},
#endif
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
float fric_r_speed_pid_param[7] = {20.f, 0.01f, 0.f, 0.f, 6000.f, 200.f, 12000.f};
float fric_l_speed_pid_param[7] = {20.f, 0.01f, 0.f, 0.f, 6000.f, 200.f, 12000.f};
float dial_position_in_pid_param[7] = {17.f, 1.2f, 0.f, 0.f, 6000.f, 6000.f, 12000.f};
float dial_position_pid_param[7] = {0.24414f, 0.f, 0.f, 0.f, 0.f, 0.f, 10000.f};
float gim_y_position_in_pid_param[7] = {200.f, 2.f, 0.f, 0.f, 10000.f, 15000.f, 29000.f};
float gim_y_position_pid_param[7] = {21.f, 0.f, 30.f, 0.f, 10000.f, 10000.f, 30000.f};

#if defined (CAR)	&& (CAR == 1)
float gim_p_position_in_pid_param[7] = {200.f, 2.f, 0.f, 0.f, 9000.f, 18000.f, 29000.f};
float gim_p_position_pid_param[7] = {15.f, 0.f, 0.f, 0.f, 10000.f, 10000.f, 30000.f};
#elif defined (CAR) && (CAR == 2)
float gim_p_position_in_pid_param[7] = {200.f, 2.f, 0.f, 0.f, 9000.f, 18000.f, 29000.f};
float gim_p_position_pid_param[7] = {15.f, 0.f, 0.f, 0.f, 10000.f, 10000.f, 30000.f};
#endif

//float yaw_gyro_position[7] = {20.f, 0.f, 0.f, 0.f, 10000.f, 10000.f, 25000.f};
//float yaw_gyro_position_in[7] = {200.f, 2.f, 0.f, 0.f, 10000.f, 15000.f, 25000.f};
//float pitch_gyro_position[7] = {60.f, 0.f, 0.f, 0.f, 10000.f, 10000.f, 25000.f};
//float pitch_gyro_position_in[7] = {80.f, 2.f, 0.f, 0.f, 10000.f, 15000.f, 25000.f};
/* Exported variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
 *	@brief	RM����ܳ�ʼ��
 */
void RM_motor_init(void)
{
	//�����ʼ��
	RM_motor[FRIC_R].init(&RM_motor[FRIC_R]);
	RM_motor[FRIC_L].init(&RM_motor[FRIC_L]);
	RM_motor[DIAL].init(&RM_motor[DIAL]);
	RM_motor[GIM_P].init(&RM_motor[GIM_P]);
	RM_motor[GIM_Y].init(&RM_motor[GIM_Y]);
	//����ٶ�pid��ʼ��
	RM_motor[FRIC_R].pid_init(&RM_motor[FRIC_R].pid.speed, fric_r_speed_pid_param);
	RM_motor[FRIC_L].pid_init(&RM_motor[FRIC_L].pid.speed, fric_l_speed_pid_param);
	RM_motor[DIAL].pid_init(&RM_motor[DIAL].pid.position_in, dial_position_in_pid_param);
	RM_motor[DIAL].pid_init(&RM_motor[DIAL].pid.position, dial_position_pid_param);
	RM_motor[GIM_P].pid_init(&RM_motor[GIM_P].pid.position_in, gim_p_position_in_pid_param);
	RM_motor[GIM_P].pid_init(&RM_motor[GIM_P].pid.position, gim_p_position_pid_param);
	RM_motor[GIM_Y].pid_init(&RM_motor[GIM_Y].pid.position_in, gim_y_position_in_pid_param);
	RM_motor[GIM_Y].pid_init(&RM_motor[GIM_Y].pid.position, gim_y_position_pid_param);
}



