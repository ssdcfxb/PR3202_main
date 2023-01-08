/**
 * @file        module.c/.h
 * @author      SSDCFXB
 * @Version     V1.0
 * @date        5-January-2023
 * @brief       Module Status Control
 * @update
 */

/* Includes ------------------------------------------------------------------*/
#include "module.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void module_info_update(module_t *mod);
void RC_StateCheck(void);
void Motor_StateCheck(void);
/* Private typedef -----------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
module_t module = {
	.heartbeat = module_info_update,
};

flag_t flag = {
	.gimbal_flag.reset_start = 1,
	.gimbal_flag.reset_ok = 0,
	.gimbal_flag.turn_ok = 2,
	.chassis_flag.stop_start = 0,
	.chassis_flag.stop_ok = 0,
	.rc_init = 0,
};

/* Private functions ---------------------------------------------------------*/
void module_info_update(module_t *mod)
{
	if(module.state != MODULE_STATE_NORMAL) 
	{
		module.mode = MODULE_MODE_RESET;
	}
	else 
	{
		if (rc_sensor.info->s1 == RC_SW_DOWN)
		{
			module.remote_mode = RC;
			module.mode = MODULE_MODE_MACHINE;
			gimbal.info->gimbal_mode = gim_machine;
		}
		else if (rc_sensor.info->s1 == RC_SW_MID)
		{
			module.remote_mode = RC;
			module.mode = MODULE_MODE_GYRO;
			gimbal.info->gimbal_mode = gim_gyro;
			
			if (rc_sensor.info->s2 == RC_SW_DOWN)
			{
				gimbal.info->gimbal_mode = gim_vision;
			}
		}
		else if (rc_sensor.info->s1 == RC_SW_UP)
		{
//			if (flag.rc_init == 1)
//			{
//				module.remote_mode = KEY;
////				key_update_info();
//			}
//			else
//			{
//				gimbal.info->gimbal_mode = gim_machine;
//			}
		}
		else 
		{
			module.mode = MODULE_MODE_RESET;
			gimbal.info->gimbal_mode = gim_keep;
		}
	}
	
	if (module.remote_mode == RC)
	{
		gimbal.info->remote_mode = RC;
	}
	else if (module.remote_mode == KEY)
	{
		gimbal.info->remote_mode = KEY;
	}
	
	RC_StateCheck();
}

void RC_StateCheck(void)
{
	if(rc_sensor.work_state == DEV_OFFLINE) 
	{
		module.state = MODULE_STATE_RCLOST;
		RC_ResetData(&rc_sensor);
	} 
	/* 遥控在线 */
	else if(rc_sensor.work_state == DEV_ONLINE)
	{
		/* 遥控正常 */
		if(rc_sensor.errno == NONE_ERR) 
		{
			/* 失联恢复 */
			if(module.state == SYS_STATE_RCLOST) 
			{
				// 可在此处同步云台复位标志位					
				// 系统参数复位
				module.remote_mode = RC;
				module.state = MODULE_STATE_NORMAL;
				module.mode = MODULE_MODE_RESET;
			}
		}
	}
}

void Motor_StateCheck(void)
{
	uint16_t i = 0;
	for ( ; i < DIAL; i++)
	{
		if (RM_motor[i].state.work_state == M_OFFLINE)
		{
			module.state = MODULE_STATE_LCHLOST;
			break;
		}
		else
		{
			module.state = MODULE_STATE_NORMAL;
		}
	}
	
	for (i = DIAL; i < MOTOR_LIST; i++)
	{
		if (RM_motor[i].state.work_state == M_OFFLINE)
		{
			module.state = MODULE_STATE_GIMLOST;
			break;
		}
		else
		{
			module.state = MODULE_STATE_NORMAL;
		}
	}
}
