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
#include "RM_motor.h"

#define MOMENTUM_L_INIT      RM_motor[MOMENTUM_L].state.init_flag == M_INIT
#define MOMENTUM_L_DEINIT    RM_motor[MOMENTUM_L].state.init_flag == M_DEINIT
#define MOMENTUM_L_ONLINE    RM_motor[MOMENTUM_L].state.work_state == M_ONLINE
#define MOMENTUM_L_OFFLINE	 RM_motor[MOMENTUM_L].state.work_state == M_OFFLINE

#define MOMENTUM_R_INIT      RM_motor[MOMENTUM_R].state.init_flag == M_INIT
#define MOMENTUM_R_DEINIT    RM_motor[MOMENTUM_R].state.init_flag == M_DEINIT
#define MOMENTUM_R_ONLINE    RM_motor[MOMENTUM_R].state.work_state == M_ONLINE
#define MOMENTUM_R_OFFLINE	 RM_motor[MOMENTUM_R].state.work_state == M_OFFLINE


#define LEG_L_INIT        KT_motor[LEG_L].KT_motor_info.state_info.init_flag  == M_INIT
#define LEG_L_DEINIT      KT_motor[LEG_L].KT_motor_info.state_info.init_flag  == M_DEINIT
#define LEG_L_ONLINE      KT_motor[LEG_L].KT_motor_info.state_info.work_state == M_ONLINE
#define LEG_L_OFFLINE     KT_motor[LEG_L].KT_motor_info.state_info.work_state == M_OFFLINE
#define LEG_L_PROTECT_ON  KT_motor[LEG_L].KT_motor_info.state_info.selfprotect_flag == M_PROTECT_ON
#define LEG_L_PROTECT_OFF KT_motor[LEG_L].KT_motor_info.state_info.selfprotect_flag == M_PROTECT_OFF


#define LEG_R_INIT        KT_motor[LEG_R].KT_motor_info.state_info.init_flag  == M_INIT
#define LEG_R_DEINIT      KT_motor[LEG_R].KT_motor_info.state_info.init_flag  == M_DEINIT
#define LEG_R_ONLINE      KT_motor[LEG_R].KT_motor_info.state_info.work_state == M_ONLINE
#define LEG_R_OFFLINE     KT_motor[LEG_R].KT_motor_info.state_info.work_state == M_OFFLINE
#define LEG_R_PROTECT_ON  KT_motor[LEG_R].KT_motor_info.state_info.selfprotect_flag == M_PROTECT_ON
#define LEG_R_PROTECT_OFF KT_motor[LEG_R].KT_motor_info.state_info.selfprotect_flag == M_PROTECT_OFF
/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef enum
{
	LEG_L = 0,
	LEG_R ,
	
	KT_MOTOR_LIST ,
	
}KT_motor_list_e;

typedef enum
{
	MOMENTUM_L = 0,
	MOMENTUM_R ,
	
	MOM_MOTOR_LIST ,
	
}MOM_motor_list_e;

extern KT_motor_t KT_motor[KT_MOTOR_LIST];
extern RM_motor_t RM_motor[MOM_MOTOR_LIST];


///* Exported functions --------------------------------------------------------*/
void KT_motor_init(void);
void RM_motor_init(void);

#endif

