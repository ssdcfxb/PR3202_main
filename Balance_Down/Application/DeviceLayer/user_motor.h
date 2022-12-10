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
#include "KT_motor.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef enum
{
	LEG_L = 0,
	LEG_R,
	
	MOTOR_LIST,
	
}KT_motor_list_e;

extern KT_motor_t KT_motor[MOTOR_LIST];



///* Exported functions --------------------------------------------------------*/
void KT_motor_all_init(void);


#endif

