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
void Slave_StateCheck(void);
/* Private typedef -----------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
symbal_t symbal = {
	.gim_sym.reset_start = 1,
	.gim_sym.reset_ok = 0,
	.gim_sym.turn_start = 0,
	.gim_sym.turn_ok = 1,
	.rc_update = 0,
};

module_t module = {
	.heartbeat = module_info_update,
};

/* Private functions ---------------------------------------------------------*/
void module_info_update(module_t *mod)
{
	RC_StateCheck();
	Slave_StateCheck();
	
	if(module.state != MODULE_STATE_NORMAL) 
	{
		module.remote_mode = RC;
		module.mode = MODULE_MODE_RESET;
		gimbal.info->gimbal_mode = gim_keep;
		launcher.info->launcher_mode = lch_keep;
		symbal.gim_sym.reset_start = 1;
		symbal.gim_sym.reset_ok = 0;
	}
	else 
	{
		/*  ģʽ�л�  */
		if (rc_sensor.info->s1 == RC_SW_DOWN)
		{
			module.remote_mode = RC;
#if USE_MACHINE_MODE == 1
			module.mode = MODULE_MODE_MACHINE;
			gimbal.info->gimbal_mode = gim_machine;
			launcher.info->launcher_mode = lch_machine;
#else
			module.mode = MODULE_MODE_GYRO2;
			gimbal.info->gimbal_mode = gim_gyro2;
			launcher.info->launcher_mode = lch_gyro2;
#endif
		}
		else if (rc_sensor.info->s1 == RC_SW_MID)
		{
			module.remote_mode = RC;
			module.mode = MODULE_MODE_GYRO;
			gimbal.info->gimbal_mode = gim_gyro;
			launcher.info->launcher_mode = lch_gyro;
		}
		else if ((rc_sensor.info->s1 == RC_SW_UP) && (rc_sensor.info->last_s1 == RC_SW_MID))
		{
			module.remote_mode = KEY;
			module.mode = MODULE_MODE_GYRO;
			gimbal.info->gimbal_mode = gim_gyro;
			launcher.info->launcher_mode = lch_gyro;
		}
		else 
		{
			module.mode = MODULE_MODE_RESET;
			gimbal.info->gimbal_mode = gim_keep;
			launcher.info->launcher_mode = lch_keep;
		}
	}
	
	/*  ���Ʒ�ʽ�л�  */
	if (module.remote_mode == RC)
	{
		gimbal.info->remote_mode = RC;
		launcher.info->remote_mode = RC;
	}
	else if (module.remote_mode == KEY)
	{
		gimbal.info->remote_mode = KEY;
		launcher.info->remote_mode = KEY;
		Key_StateCheck();
	}
	
}

void RC_StateCheck(void)
{
	if(rc_sensor.work_state == DEV_OFFLINE) 
	{
		module.state = MODULE_STATE_RCLOST;
		RC_ResetData(&rc_sensor);
	} 
	/* ң������ */
	else if(rc_sensor.work_state == DEV_ONLINE)
	{
		/* ң������ */
		if(rc_sensor.errno == NONE_ERR) 
		{
			/* ʧ���ָ� */
			if(module.state == MODULE_STATE_RCLOST) 
			{
				// ���ڴ˴�ͬ����̨��λ��־λ					
				// ϵͳ������λ
				module.remote_mode = RC;
				module.state = MODULE_STATE_RCINIT;
			}
			
			/* ʧ���ָ���ʼ�� */
			if(module.state == MODULE_STATE_RCINIT) 
			{
				if(RC_IsChannelReset())
				{
					module.state = MODULE_STATE_NORMAL;
					module.mode = MODULE_MODE_RESET;
				}
			}
		}
	}
}

void Slave_StateCheck(void)
{
	int16_t front, right;
	static uint8_t last_status = 0;
		
	/*  1:ң����״̬��־λ  */
	slave.info->tx_info->status &= 0xFE;
	if (rc_sensor.work_state == DEV_ONLINE)
		slave.info->tx_info->status |= 0x01;
	
	/*  2:����״̬��־λ  */
	slave.info->tx_info->status &= 0xFD;
	if (launcher.work_info->launcher_commond == Magz_Open)
		slave.info->tx_info->status |= 0x02;
	
	/*  3:С����״̬��־λ  */
	if (rc_sensor.work_state == DEV_OFFLINE)
		slave.info->gyro_status = WaitCommond_Gyro;
	if ((rc_sensor.info->thumbwheel.status == RC_TB_DOWN) && (last_status != RC_TB_DOWN))
	{
		if (slave.info->gyro_status == WaitCommond_Gyro)
		{
			slave.info->gyro_status = On_Gyro;
		}
		else if (slave.info->gyro_status == On_Gyro)
		{
			slave.info->gyro_status = Off_Gyro;
		}
		else if (slave.info->gyro_status == Off_Gyro)
		{
			slave.info->gyro_status = On_Gyro;
		}
	}
	slave.info->tx_info->status &= 0xFB;
	if (slave.info->gyro_status == On_Gyro)
		slave.info->tx_info->status |= 0x04;
	
	/*  4:ͷ�ĳ���  */
	slave.info->tx_info->status &= 0xF7;
	if (gimbal.conf->restart_yaw_motor_angle != gimbal.conf->MID_VALUE)
		slave.info->tx_info->status |= 0x08;
	
	// ң������ͷ
	if ((rc_sensor.info->thumbwheel.status == RC_TB_MID) && (last_status == RC_TB_UP))
	{
		symbal.gim_sym.turn_start = 1;
		symbal.gim_sym.turn_ok = 0;
	}
	
	/*  5:��ͷ״̬��־λ  */
	slave.info->tx_info->status &= 0xEF;
	if ((symbal.gim_sym.reset_ok == 0) || (symbal.gim_sym.turn_ok == 0))
		slave.info->tx_info->status |= 0x10;
	
	/*  yaw�����Ƕ�����  */
	slave.info->tx_info->motor_angle = RM_motor[GIM_Y].rx_info.angle;
	/*  yaw�������ǽǶ�����  */
	slave.info->tx_info->imu_angle = imu_sensor.info->base_info.yaw;
	
	/*  �����ƶ�ͨ��ֵ  */
	if (module.remote_mode == RC)
	{
		slave.info->tx_info->rc_ch_ws_val = rc_sensor.info->ch3;
		slave.info->tx_info->rc_ch_ad_val = rc_sensor.info->ch2;
	}
	else if(module.remote_mode == KEY)
	{
		front = 0;
		right = 0;
		front += (float)rc_sensor.info->W.cnt / (float)KEY_W_CNT_MAX * 660.f;
		front -= (float)rc_sensor.info->S.cnt / (float)KEY_S_CNT_MAX * 660.f;
		right += (float)rc_sensor.info->D.cnt / (float)KEY_D_CNT_MAX * 660.f;
		right -= (float)rc_sensor.info->A.cnt / (float)KEY_A_CNT_MAX * 660.f;		
	}
	last_status = rc_sensor.info->thumbwheel.status;
}

