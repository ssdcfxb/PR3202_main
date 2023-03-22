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
	MODULE_STATE_RCLOST,	// ң��ʧ��
	MODULE_STATE_NORMAL,	// �������
	MODULE_STATE_RCINIT,	// ң�س�ʼ��
	MODULE_STATE_WRONG,	  // �������
} module_state_t;

typedef enum {
	MODULE_MODE_RESET,		// ģʽ��λ
	MODULE_MODE_MACHINE,	// ��еģʽ
	MODULE_MODE_GYRO,			// ������ģʽ
	MODULE_MODE_GYRO2,		// ������ģʽ2
//	MODULE_MODE_KEY,			// ����ģʽ
	MODULE_MODE_CNT,
} module_mode_t;

typedef struct __module_t
{
	remote_mode_t		remote_mode;	// ���Ʒ�ʽ
	module_state_t	state;	// ���״̬
	module_mode_t		mode;	// ���ģʽ
	
	void (*heartbeat)(struct __module_t *module);
} module_t;

/* Exported functions --------------------------------------------------------*/

extern module_t module;

#endif
