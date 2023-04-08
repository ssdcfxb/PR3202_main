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

#include "launcher.h"
#include "gimbal.h"
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void module_info_update(module_t *mod);
void RC_StateCheck(void);
void Slave_TxInfoUpdate(void);
void Vision_TxInfoUpdate(void);
void Rc_RxInfoCheck(void);
void Key_RxInfoCheck(void);
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

status_t status = {
	.lch_state.fric_state = fric_reset,
	.lch_state.magz_state = magz_reset,
	.lch_state.shoot_state = shoot_reset,
	.gim_state = gim_reset,
	.chas_state = gyro_reset,
	.gim_mode = gyro,
	.tw_last_state = 0,
};

module_t module = {
	.symbal = &symbal,
	.status = &status,
	.heartbeat = module_info_update,
};

/* Private functions ---------------------------------------------------------*/
void module_info_update(module_t *mod)
{
	RC_StateCheck();
	Slave_TxInfoUpdate();
	Vision_TxInfoUpdate();
	
	if(module.state != MODULE_STATE_NORMAL) 
	{
		module.remote_mode = RC;
		module.mode = MODULE_MODE_RESET;
		gimbal.info->gimbal_mode = gim_keep;
		launcher.info->launcher_mode = lch_keep;
		symbal.gim_sym.reset_start = 1;
		symbal.gim_sym.reset_ok = 0;
		status.gim_mode = gyro;
	}
	else 
	{
		/*  模式切换  */
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
		else if (rc_sensor.info->s1 == RC_SW_UP)
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
		
		if (status.gim_mode == vision)
		{
			gimbal.info->gimbal_mode = gim_vision;
		}
		
		// 遥控器视觉
		if ((rc_sensor.info->thumbwheel.status == RC_TB_MID) && (status.tw_last_state == RC_TB_UP))
		{
			if (vision_sensor.work_state == DEV_ONLINE)
			{
					status.gim_mode = vision;
					gimbal.info->gimbal_mode = gim_vision;
			}
			else
			{
				status.gim_mode = gyro;
				gimbal.info->gimbal_mode = gim_gyro;
			}
		}
		if ((rc_sensor.info->thumbwheel.status == RC_TB_MID) && (status.tw_last_state == RC_TB_DOWN))
		{
			status.gim_mode = gyro;
			gimbal.info->gimbal_mode = gim_gyro;
		}
	
	}
	
	/*  控制方式切换  */
	if (module.remote_mode == RC)
	{
		gimbal.info->remote_mode = RC;
		launcher.info->remote_mode = RC;
		Rc_RxInfoCheck();
	}
	else if (module.remote_mode == KEY)
	{
		gimbal.info->remote_mode = KEY;
		launcher.info->remote_mode = KEY;
		Key_RxInfoCheck();
	}
	
	status.tw_last_state = rc_sensor.info->thumbwheel.status;
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
			if(module.state == MODULE_STATE_RCLOST) 
			{
				// 可在此处同步云台复位标志位					
				// 系统参数复位
				module.remote_mode = RC;
				module.state = MODULE_STATE_RCINIT;
				module.mode = MODULE_MODE_RESET;
			}
			
			if(module.state == MODULE_STATE_RCINIT)
			{
				if (RC_IsChannelReset())
				{
					module.state = MODULE_STATE_NORMAL;
				}
			}
			
		}
	}
}

void Slave_TxInfoUpdate(void)
{
	int16_t front, right;
		
	/*  1:遥控器状态标志位  */
	slave.info->tx_info->status &= 0xFE;
	if (rc_sensor.work_state == DEV_ONLINE)
		slave.info->tx_info->status |= 0x01;
	
	/*  2:弹仓状态标志位  */
	slave.info->tx_info->status &= 0xFD;
	if (launcher.work_info->launcher_commond == Magz_Open)
		slave.info->tx_info->status |= 0x02;
	
	/*  3:小陀螺状态标志位  */
	if (rc_sensor.work_state == DEV_OFFLINE)
		status.chas_state = gyro_reset;
//	if ((rc_sensor.info->thumbwheel.status == RC_TB_MID) && (status.tw_last_state == RC_TB_DOWN))
//	{
//		if (status.chas_state == gyro_on)
//		{
//			status.chas_state = gyro_off;
//		}
//		else
//		{
//			status.chas_state = gyro_on;
//		}
//	}
	if (status.lch_state.magz_state == magz_open)
	{
		status.chas_state = gyro_off;
	}
	slave.info->tx_info->status &= 0xFB;
	if (status.chas_state == gyro_on)
		slave.info->tx_info->status |= 0x04;
	
	/*  4:头的朝向  */
	slave.info->tx_info->status &= 0xF7;
	if (gimbal.conf->restart_yaw_motor_angle != gimbal.conf->MID_VALUE)
		slave.info->tx_info->status |= 0x08;
	
	/*  5:换头状态标志位  */
	slave.info->tx_info->status &= 0xEF;
	if ((symbal.gim_sym.reset_ok == 0) || (symbal.gim_sym.turn_ok == 0))
		slave.info->tx_info->status |= 0x10;
	
	/*  yaw轴电机角度数据  */
	slave.info->tx_info->motor_angle = RM_motor[GIM_Y].rx_info.angle;
	/*  yaw轴陀螺仪角度数据  */
	slave.info->tx_info->imu_angle = imu_sensor.info->base_info.yaw;
	
	/*  底盘移动通道值  */
	if (module.state == MODULE_STATE_NORMAL)
	{
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
	}
}

void Vision_TxInfoUpdate(void)
{
	vision_sensor.info->measure_pitch_angle = imu_sensor.info->base_info.pitch * ANGLE_TO_ECD + (float)HALF_ECD_RANGE;
	vision_sensor.info->measure_yaw_angle = (imu_sensor.info->base_info.yaw + 180.f) * ANGLE_TO_ECD;
	vision_sensor.info->measure_shoot_speed = launcher.conf->fric_mode;
	
	vision_sensor.info->tx_info->pitch_angle = vision_sensor.info->measure_pitch_angle;
	vision_sensor.info->tx_info->yaw_angle = vision_sensor.info->measure_yaw_angle;
	vision_sensor.info->tx_info->shoot_speed = vision_sensor.info->measure_shoot_speed;
	vision_sensor.info->tx_info->mode = AIM_ON;
	vision_sensor.info->cmd_mode = vision_sensor.info->tx_info->mode;
	vision_sensor.info->tx_info->my_color = slave.info->my_color;
}

void Rc_RxInfoCheck(void)
{
	/*  遥控器换头  */
//	if (vision_sensor.work_state == DEV_OFFLINE)
//	{
//		if ((rc_sensor.info->thumbwheel.status == RC_TB_MID) && (status.tw_last_state == RC_TB_UP))
//		{
//			if (status.lch_state.magz_state == magz_close)
//			{
//				symbal.gim_sym.turn_start = 1;
//				symbal.gim_sym.turn_ok = 0;
//				status.gim_cmd = gim_turn;
//			}
//		}
//	}
	/*  关弹仓  */
	if (status.chas_state == gyro_on)
	{
		status.lch_cmd.magz_cmd = magz_close;
	}
}

void Key_RxInfoCheck(void)
{
	Key_StateUpdate();
	
	status.lch_cmd.fric_cmd = fric_reset;
//	keyboard.lch_cmd.magz_cmd = lch_reset;
	status.lch_cmd.shoot_cmd = shoot_reset;
	/*  Ctrl(优先级顺序):关弹仓 关摩擦轮 关小陀螺  */
	if (keyboard.state.Ctrl == down_K)
	{
		if (status.lch_state.magz_state == magz_open)
		{
			status.lch_cmd.magz_cmd = magz_close;
		}
		else if (status.lch_state.fric_state == fric_on)
		{
			status.lch_cmd.fric_cmd = fric_off;
		}
		if (status.chas_state == gyro_on) 
		{
			status.chas_state = gyro_off;
		}
	}
	/*  R:开摩擦轮  */
	if (keyboard.state.R == down_K)
	{
		status.lch_cmd.fric_cmd = fric_on;
	}
	/*  G:开弹仓  */
	if (keyboard.state.G == down_K)
	{
		status.lch_cmd.magz_cmd = magz_open;
	}
	/*  B:关弹仓  */
	if (keyboard.state.B == down_K)
	{
		status.lch_cmd.magz_cmd = magz_close;
	}
	if (status.chas_state == gyro_on)
	{
		status.lch_cmd.magz_cmd = magz_close;
	}
	
	/*  mouse_btn_l  */
	if (keyboard.state.mouse_btn_l == up_K)
	{
		status.lch_cmd.fric_cmd = fric_on;
	}
	if ((keyboard.state.mouse_btn_l == short_press_K) && (status.lch_state.fric_state == fric_on))
	{
		status.lch_cmd.shoot_cmd = single_shoot;
	}
	if (keyboard.state.mouse_btn_l == long_press_K)
	{
		status.lch_cmd.shoot_cmd = keep_shoot;
	}
	
	
	status.gim_cmd = gim_reset;
	/*  V:掉头  */
	if ((keyboard.state.V == down_K) && (status.lch_state.magz_state == magz_close))
	{
		symbal.gim_sym.turn_start = 1;
		symbal.gim_sym.turn_ok = 0;
		status.gim_cmd = gim_turn;
	}
	
	
	status.gim_mode = gyro;
	/*  mouse_btn_r:自瞄  */
	if ((keyboard.state.mouse_btn_r == short_press_K) || (keyboard.state.mouse_btn_r == long_press_K))
	{
		status.gim_mode = vision;
	}
}

