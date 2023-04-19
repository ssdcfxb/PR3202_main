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


#define MOMENTUM_L_POS_OFFSET         5000							//���������Ƕ�ƫ��
#define MOMENTUM_R_POS_OFFSET         5000							//���������Ƕ�ƫ��

#define MOMENTUM_L_F_POS       		  160             	//��������ǰ��ĵ��λ�� 
#define MOMENTUM_L_M_POS   	    		4915				  		//���������м�ĵ��λ��
#define MOMENTUM_L_B_POS      			1730							//�������ں���ĵ��λ��
#define MOMENTUM_L_F_POS_OFFSET    	1263           
#define MOMENTUM_L_M_POS_OFFSET     4580			 		
#define MOMENTUM_L_B_POS_OFFSET   	7840							


#define MOMENTUM_R_F_POS       		  2195              //��������ǰ��ĵ��λ�� 
#define MOMENTUM_R_M_POS   	    		5614 				  		//���������м�ĵ��λ��
#define MOMENTUM_R_B_POS      			596							//�������ں���ĵ��λ��
#define MOMENTUM_R_F_POS_OFFSET    	7948            
#define MOMENTUM_R_M_POS_OFFSET   	4520				  	
#define MOMENTUM_R_B_POS_OFFSET   	1376	

#define YAW_MOTOR_F_ANGLE						5110
#define YAW_MOTOR_B_ANGLE						1014


#define MOMENTUM_L_INIT      MOM_motor[MOMENTUM_L].state.init_flag == M_INIT
#define MOMENTUM_L_DEINIT    MOM_motor[MOMENTUM_L].state.init_flag == M_DEINIT
#define MOMENTUM_L_ONLINE    MOM_motor[MOMENTUM_L].state.work_state == M_ONLINE
#define MOMENTUM_L_OFFLINE	 MOM_motor[MOMENTUM_L].state.work_state == M_OFFLINE

#define MOMENTUM_R_INIT      MOM_motor[MOMENTUM_R].state.init_flag == M_INIT
#define MOMENTUM_R_DEINIT    MOM_motor[MOMENTUM_R].state.init_flag == M_DEINIT
#define MOMENTUM_R_ONLINE    MOM_motor[MOMENTUM_R].state.work_state == M_ONLINE
#define MOMENTUM_R_OFFLINE	 MOM_motor[MOMENTUM_R].state.work_state == M_OFFLINE


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




///* Exported functions --------------------------------------------------------*/
void KT_motor_init(void);
void RM_motor_init(void);
void All_motor_heartbeat(void);
#endif

