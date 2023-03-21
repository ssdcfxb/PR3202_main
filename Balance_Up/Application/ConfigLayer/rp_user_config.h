/**
 * @file        rp_user_config.h
 * @author      RobotPilots
 * @Version     v1.0
 * @brief       RobotPilots Robots' User Configuration.
 * @update
 *              v1.0(7-November-2021)  
 */
#ifndef __RP_USER_CONFIG_H
#define __RP_USER_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stdbool.h"

/* Exported macro ------------------------------------------------------------*/
#define EVENT_SEND_CAN1_MAILBOX (0x1UL << 0)
#define EVENT_SEND_CAN2_MAILBOX (0x1UL << 1)
#define USE_MACHINE_MODE (0)
/* Exported types ------------------------------------------------------------*/
/* 应用层 --------------------------------------------------------------------*/
/* Remote Mode Enum */
typedef enum {
	RC = 0,
	KEY = 1,
	REMOTE_MODE_CNT = 2,
} remote_mode_t;

typedef enum {
	SYS_STATE_NORMAL,	// 系统正常
	SYS_STATE_RCLOST,	// 遥控失联
	SYS_STATE_RCERR,	// 遥控出错
	SYS_STATE_WRONG,	// 其它系统错误
} sys_state_t;

typedef enum {
	SYS_MODE_NORMAL,	// 常规模式
	SYS_MODE_CNT,
} sys_mode_t;

typedef struct {
	uint8_t stop_start;
	uint8_t stop_ok;
	uint8_t forward; // 1:头为正 0:尾为正
} chassis_flag_t;

typedef struct {
	uint8_t reset_start;
	uint8_t reset_ok;
	uint8_t turn_start;
	uint8_t turn_ok;
} gimbal_flag_t;

typedef struct __flag
{
	gimbal_flag_t   gimbal_flag;
	chassis_flag_t  chassis_flag;
	uint8_t rc_update;
} flag_t;

typedef struct {
	remote_mode_t		remote_mode;	// 控制方式
	sys_state_t			state;			// 系统状态
	sys_mode_t			mode;			// 系统模式
} system_t;

extern flag_t	flag;
extern system_t sys;

#endif
