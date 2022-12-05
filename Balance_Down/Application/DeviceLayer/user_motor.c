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
#include "string.h"
#include "drv_can.h"
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/*
* ʹ�õ��ǰ�������úõ�������͡�id�š�������ʽ
*/
motor_9025_info_t motor_9025_info_l;
motor_9025_info_t motor_9025_info_r;

motor_t motor[MOTOR_LIST] =
{
	[LEG_L] = {
		
		.id.drive_type = M_CAN1,
		.id.motor_type = KT9025,
		.id.rx_id = 0x141,
		.id.tx_id = 0x141,
		
		.kt9025_info = &motor_9025_info_l,
		
		.init = motor_class_init,
	},
	
	[LEG_R] = {
		
		.id.drive_type = M_CAN1,
		.id.motor_type = KT9025,
		.id.rx_id = 0x142,
		.id.tx_id = 0x142,
		
		.kt9025_info = &motor_9025_info_r,
		
		.init = motor_class_init,
	},
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
//float fric_r_speed_pid_param[7] = {15.f, 0.5f, 0.f, 0.f, 6000.f, 6000.f, 12000.f};
//float fric_l_speed_pid_param[7] = {15.f, 0.5f, 0.f, 0.f, 6000.f, 6000.f, 12000.f};
//float dial_position_in_pid_param[7] = {17.f, 1.2f, 0.f, 0.f, 6000.f, 6000.f, 12000.f};
//float dial_position_pid_param[7] = {0.24414f, 0.f, 0.f, 0.f, 0.f, 0.f, 10000.f};

/* Exported variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/


/**
 *	@brief	����ܳ�ʼ��
 */
void motor_all_init(void)
{
	//�����ʼ��
	motor[LEG_L].init(&motor[LEG_L]);
	motor[LEG_R].init(&motor[LEG_R]);
	
}

/**
 *	@brief	kt��������
 */
void kt9025_cmd_send(motor_t *motor, uint8_t cmd)
{
	uint8_t data[8];
	
	memset(data, 0, 8);
	data[0] = cmd;
	
	if(motor->id.drive_type == M_CAN1)
	{
		CAN_SendData(&hcan1, motor->id.tx_id, data);
	}
	else if(motor->id.drive_type == M_CAN2)
	{
		CAN_SendData(&hcan2, motor->id.tx_id, data);
	}
	
}

/**
 *	@brief	kt�������+���ݷ���
 */
void kt9025_data_send(motor_t *motor, int16_t *dat)
{
	uint8_t data[8];
	
	data[0] = (uint8_t)((int16_t)dat[0] >> 8);
	data[1] = (uint8_t)((int16_t)dat[0]);
	data[2] = (uint8_t)((int16_t)dat[1] >> 8);
	data[3] = (uint8_t)((int16_t)dat[1]);
	data[4] = (uint8_t)((int16_t)dat[2] >> 8);
	data[5] = (uint8_t)((int16_t)dat[2]);
	data[6] = (uint8_t)((int16_t)dat[3] >> 8);
	data[7] = (uint8_t)((int16_t)dat[3]);			
	
	if(motor->id.drive_type == M_CAN1)
	{
		CAN_SendData(&hcan1, motor->id.tx_id, data);
	}
	else if(motor->id.drive_type == M_CAN2)
	{
		CAN_SendData(&hcan2, motor->id.tx_id, data);
	}
	
}


