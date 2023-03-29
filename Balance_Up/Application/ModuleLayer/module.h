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
#include "device.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef enum {
	MODULE_STATE_RCLOST,	// ң��ʧ��
	MODULE_STATE_RCINIT,	// ң�س�ʼ��
	MODULE_STATE_NORMAL,	// �������
	MODULE_STATE_WRONG,	  // �������
} module_state_t;

typedef enum {
	MODULE_MODE_RESET,		// ģʽ��λ
	MODULE_MODE_MACHINE,	// ��еģʽ
	MODULE_MODE_GYRO,			// ������ģʽ
	MODULE_MODE_GYRO2,		// ������ģʽ2
	MODULE_MODE_CNT,
} module_mode_t;

/*  �������״̬  */
typedef struct launcher_state_struct {
	fric_cmd_e		fric_state;
	magz_cmd_e		magz_state;
	dial_cmd_e		shoot_state;
} launcher_state_t;

/*  С����״̬  */
typedef enum
{
	gyro_reset, 			// ���ܸ�λ
	gyro_on,        	// ����
	gyro_off,       	// �ر�
} chas_cmd_e;

typedef struct {
	launcher_state_t	lch_state;
	gim_cmd_e					gim_state;
	chas_cmd_e				chas_state;
	lch_cmd_t					lch_cmd;
	gim_cmd_e					gim_cmd;
	gim_mode_e				gim_mode;
	uint8_t						tw_last_state;
} status_t;

typedef struct __module_t
{
	remote_mode_t		remote_mode;	// ���Ʒ�ʽ
	module_state_t	state;	// ���״̬
	module_mode_t		mode;	// ���ģʽ
	symbal_t				*symbal;
	status_t				*status;
	
	void (*heartbeat)(struct __module_t *module);
} module_t;

extern status_t status;
extern module_t module;

/* Exported functions --------------------------------------------------------*/

#endif
