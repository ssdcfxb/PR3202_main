/**
 * @file        user_motor.c/.h
 * @author      SSDCFXB
 * @Version     V1.0
 * @date        18-November-2022
 * @brief       User Motor Devives
 * @update
 */

#ifndef __USER_MOTOR_H
#define __USER_MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "motor.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef enum
{
	LEG_L = 0,
	LEG_R,
	
	MOTOR_LIST,
}motor_list_e;

extern motor_t motor[MOTOR_LIST];

/* Exported functions --------------------------------------------------------*/
void motor_all_init(void);
void kt9025_cmd_send(motor_t *motor, uint8_t cmd);
void kt9025_data_send(motor_t *motor, int16_t *dat);

#endif

