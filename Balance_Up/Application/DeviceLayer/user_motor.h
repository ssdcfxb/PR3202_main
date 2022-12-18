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
#include "RM_motor.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef enum
{
	FRIC_R = 0,
	FRIC_L,
	DIAL,
	
	GIM_P,
	GIM_Y,
	
	MOTOR_LIST,
	
}motor_list_e;

extern RM_motor_t RM_motor[MOTOR_LIST];


///* Exported functions --------------------------------------------------------*/
void RM_motor_init(void);

#endif

