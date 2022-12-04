/**
 * @file        user_motor.c
 * @author      SSDCFXB
 * @Version     V1.0
 * @date        18-November-2022
 * @brief       User Motor Devives
 * @update
 */

/* Includes ------------------------------------------------------------------*/
#include "user_motor.h"
#include "string.h"
#include "drv_can.h"
extern CAN_HandleTypeDef hcan1;
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/*
* ʹ�õ��ǰ�������úõ�������͡�id�š�������ʽ
*/
motor_9025_info_t motor_9025_info;

motor_t motor[MOTOR_LIST] =
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
	
	[LEG] = {
		
		.id.drive_type = M_CAN1,
		.id.motor_type = KT9025,
		.id.rx_id = 0x141,
		.id.tx_id = 0x141,
		
		.kt9025_info = &motor_9025_info,
		
		.init = motor_class_init,
	}
};


/*
	����һ��������Ϊ�����������ʹ�ýṹ������Ϊ�����ʼ��չ��Ϊһά�Ŀ������Ƚϼ�Լ������Ҫ��ס���ǵ�λ�úͺ���
	float	  kp;
	float 	ki;
	float 	kd;
	
	float   blind_err;	
	float 	integral_max;	
	float   iout_max;
	float 	out_max;
*/
float fric_r_speed_pid_param[7] = {15.f, 0.5f, 0.f, 0.f, 6000.f, 6000.f, 12000.f};
float fric_l_speed_pid_param[7] = {15.f, 0.5f, 0.f, 0.f, 6000.f, 6000.f, 12000.f};
float dial_position_in_pid_param[7] = {17.f, 1.2f, 0.f, 0.f, 6000.f, 6000.f, 12000.f};
float dial_position_pid_param[7] = {0.24414f, 0.f, 0.f, 0.f, 0.f, 0.f, 10000.f};

/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/


/**
 *	@brief	����ܳ�ʼ��
 */
void motor_all_init(void)
{
	//�����ʼ��
	motor[FRIC_R].init(&motor[FRIC_R]);
	motor[FRIC_L].init(&motor[FRIC_L]);
	motor[DIAL].init(&motor[DIAL]);
	//����ٶ�pid��ʼ��
	motor[FRIC_R].pid_init(&motor[FRIC_R].pid.speed,fric_r_speed_pid_param);
	motor[FRIC_L].pid_init(&motor[FRIC_L].pid.speed,fric_l_speed_pid_param);
	motor[DIAL].pid_init(&motor[DIAL].pid.position_in,dial_position_in_pid_param);
	motor[DIAL].pid_init(&motor[DIAL].pid.position,dial_position_pid_param);
	
	
	
}

void kt9025_send(motor_t *motor, uint8_t cmd)
{
	uint8_t data[8];
	
	memset(data, 0, 8);
	data[0] = cmd;
	
	CAN_SendData(&hcan1,motor->id.tx_id,data);
	
}


