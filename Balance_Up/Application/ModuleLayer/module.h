/**
 * @file        module.c/.h
 * @author      SSDCFXB
 * @Version     V1.0
 * @date        5-January-2023
 * @brief       Module Status Control
 * @update
 */

#ifndef __MODULE_H
#define __MODULE_H

/* Includes ------------------------------------------------------------------*/
#include "launcher.h"
#include "gimbal.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef enum {
	MODULE_STATE_RCLOST,	// 遥控失联
	MODULE_STATE_NORMAL,	// 组件正常
	MODULE_STATE_RCINIT,	// 遥控初始化
	MODULE_STATE_WRONG,	  // 组件错误
} module_state_t;

typedef enum {
	MODULE_MODE_RESET,		// 模式复位
	MODULE_MODE_MACHINE,	// 机械模式
	MODULE_MODE_GYRO,			// 陀螺仪模式
	MODULE_MODE_GYRO2,		// 陀螺仪模式2
//	MODULE_MODE_KEY,			// 键盘模式
	MODULE_MODE_CNT,
} module_mode_t;

typedef struct __module_t
{
	remote_mode_t		remote_mode;	// 控制方式
	module_state_t	state;	// 组件状态
	module_mode_t		mode;	// 组件模式
	
	void (*heartbeat)(struct __module_t *module);
} module_t;

/* Exported functions --------------------------------------------------------*/

extern module_t module;

#endif
