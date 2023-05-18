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

//#include "drv_tim.h"
//#include "can_protocol.h"
#include "launcher.h"
#include "gimbal.h"
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void module_info_update(module_t *mod);
void Module_StateCheck(void);
void Slave_TxInfoUpdate(void);
void Vision_TxInfoUpdate(void);
void Rc_RxInfoCheck(void);
void Key_RxInfoCheck(void);
/* Private typedef -----------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
//extern int16_t can1_send_buf[8], can2_send_buf[8];
/* Exported functions --------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
symbal_t symbal = {
	.gim_sym.reset_start = 1,
	.gim_sym.reset_ok = 0,
	.gim_sym.turn_start = 0,
	.gim_sym.turn_ok = 1,
	.gim_sym.turn_right = 0,
	.gim_sym.turn_left = 0,
	.rc_update = 0,
	.slave_reset = 0,
};

status_t status = {
	.lch_state.fric_state = fric_reset,
	.lch_state.magz_state = magz_reset,
	.lch_state.shoot_state = shoot_reset,
	.gim_state = gim_reset,
	.chas_cmd = chas_reset,
	.chas_state = chas_reset,
	.gim_mode = gyro,
	.heat_mode = heat_limit_on,
	.speed_cmd = speed_reset,
	.auto_cmd = auto_shoot_off,
	.autobuff_cmd = auto_buff_off,
	.buff_cmd = buff_reset,
	.block_cmd = block_reset,
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
	Module_StateCheck();
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
		symbal.slave_reset = 0;
		status.gim_mode = gyro;
		status.lch_cmd.fric_cmd = fric_reset;
		status.lch_cmd.shoot_cmd = shoot_reset;
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
//		if (module.remote_mode == RC)
//		{
//			if ((rc_sensor.info->thumbwheel.status == RC_TB_MID) && (status.tw_last_state == RC_TB_UP))
//			{
//				if (vision_sensor.work_state == DEV_ONLINE)
//				{
//					if (status.gim_mode == gyro)
//					{
//						status.gim_mode = vision;
//						gimbal.info->gimbal_mode = gim_vision;
//					}
//					else
//					{
//						status.gim_mode = gyro;
//						gimbal.info->gimbal_mode = gim_gyro;
//					}
//				}
//				else
//				{
//					status.gim_mode = gyro;
//					gimbal.info->gimbal_mode = gim_gyro;
//				}
//			}
//		}
//		if ((rc_sensor.info->thumbwheel.status == RC_TB_MID) && (status.tw_last_state == RC_TB_DOWN))
//		{
//			status.gim_mode = gyro;
//			gimbal.info->gimbal_mode = gim_gyro;
//		}
	
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

void Module_StateCheck(void)
{
	/*  遥控器状态检测  */
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
					status.lch_cmd.magz_cmd = magz_close;
				}
			}
			
		}
	}
	
	/*  陀螺仪状态检测  */
	if ((imu_sensor.work_state.err_code != IMU_NONE_ERR) && \
			(imu_sensor.work_state.err_code != IMU_DATA_CALI))
	{
		module.state = MODULE_STATE_IMUERR;
	}
}

void Slave_TxInfoUpdate(void)
{
	int16_t front, right;
		
	slave.info->tx_info->rc_ch_ws_val = 0;
	slave.info->tx_info->rc_ch_ad_val = 0;
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
			slave.info->tx_info->rc_ch_ws_val = front;
			slave.info->tx_info->rc_ch_ad_val = right;
		}
	}
	
	/*  1:遥控器状态标志位  */
	slave.info->tx_info->status &= 0xFFFE;
	if (module.state == MODULE_STATE_NORMAL)
		slave.info->tx_info->status |= 0x0001;
	
	/*  2:弹仓状态标志位  */
	slave.info->tx_info->status &= 0xFFFD;
	if (status.lch_state.magz_state == magz_open)
		slave.info->tx_info->status |= 0x0002;
	
	/*  3:小陀螺状态标志位  */
		/*  遥控器断电关闭  */
	if (rc_sensor.work_state == DEV_OFFLINE)
		status.chas_cmd = chas_reset;
		/*  遥控器开关小陀螺  */
	if ((module.remote_mode == RC) && (module.mode == MODULE_MODE_GYRO))
	{
		if ((rc_sensor.info->thumbwheel.status == RC_TB_MID) && (status.tw_last_state == RC_TB_DOWN))
		{
			if (status.chas_state == gyro_on)
			{
				status.chas_cmd = chas_reset;
			}
			else
			{
				status.chas_cmd = gyro_on;
			}
		}
	}
		/*  开弹仓关闭  */
	if (status.lch_state.magz_state == magz_open)
	{
		status.chas_cmd = chas_reset;
	}
		/*  设置开关标志位  */
	if (status.chas_cmd == gyro_on)
		status.chas_state = gyro_on;
	else if (status.chas_cmd == chas_reset)
		status.chas_state = chas_reset;
		/*  移动时临时关闭  */
	if ((status.chas_state == gyro_on) && (slave.info->tx_info->rc_ch_ws_val != 0))
	{
		status.chas_state = chas_reset;
	}
	slave.info->tx_info->status &= 0xFFFB;
	if (status.chas_state == gyro_on)
		slave.info->tx_info->status |= 0x0004;
	
	/*  4:头的朝向(开启侧身时为1)  */
	slave.info->tx_info->status &= 0xFFF7;
	if (gimbal.conf->restart_yaw_motor_angle != gimbal.conf->MID_VALUE)
		slave.info->tx_info->status |= 0x0008;
	
	/*  5:换头状态标志位  */
	slave.info->tx_info->status &= 0xFFEF;
	if ((symbal.gim_sym.reset_ok == 0) || (symbal.gim_sym.turn_ok == 0))
		slave.info->tx_info->status |= 0x0010;
	
	/*  6:自瞄标志位  */
	slave.info->tx_info->status &= 0xFFDF;
	if (((vision_sensor.info->is_find_target == 1) || (vision_sensor.info->is_find_buff == 1))\
			&& (status.gim_mode == vision))
		slave.info->tx_info->status |= 0x0020;
	
	/*  7:复位标志位  */
	slave.info->tx_info->status &= 0xFFBF;
	if (symbal.slave_reset == 1)
		slave.info->tx_info->status |= 0x0040;
	
	/*  8:侧身状态标志位  */
		/*  遥控器断电关闭  */
	if (rc_sensor.work_state == DEV_OFFLINE)
		status.chas_cmd = chas_reset;
		/*  遥控器开关侧身  */
	if ((module.remote_mode == RC) && (module.mode == MODULE_MODE_GYRO2))
	{
		if ((rc_sensor.info->thumbwheel.status == RC_TB_MID) && (status.tw_last_state == RC_TB_DOWN))
		{
			if (status.chas_state == lean_on)
			{
				status.chas_cmd = chas_reset;
			}
			else
			{
				status.chas_cmd = lean_on;
			}
		}
	}
		/*  开弹仓关闭  */
	if (status.lch_state.magz_state == magz_open)
	{
		status.chas_cmd = chas_reset;
	}
		/*  设置开关标志位  */
	if (status.chas_cmd == lean_on)
		status.chas_state = lean_on;
	else if (status.chas_cmd == chas_reset)
		status.chas_state = chas_reset;
		/*  移动时临时关闭  */
	if ((status.chas_state == lean_on) && (slave.info->tx_info->rc_ch_ws_val != 0))
	{
		status.chas_state = chas_reset;
	}
	
	slave.info->tx_info->status &= 0xFF7F;
	if (status.chas_state == lean_on)
		slave.info->tx_info->status |= 0x0080;
	
	/*  9:加速标志位  */
	slave.info->tx_info->status &= 0xFEFF;
	if (status.speed_cmd == rapid_on)
		slave.info->tx_info->status |= 0x0100;
	
	/*  10:打符标志位  */
	slave.info->tx_info->status &= 0xFDFF;
	if (status.buff_cmd != buff_reset)
		slave.info->tx_info->status |= 0x0200;
	
	/*  11:陀螺仪状态标志位  */
	slave.info->tx_info->status &= 0xFBFF;
	if (imu_sensor.work_state.err_code != IMU_NONE_ERR)
		slave.info->tx_info->status |= 0x0400;
	
	/*  12:左右标志位(关闭侧身时为0)  */
	slave.info->tx_info->status &= 0xF7FF;
	if ((gimbal.info->measure_yaw_motor_angle < gimbal.conf->MID_VALUE) &&\
			(gimbal.info->measure_yaw_motor_angle > (gimbal.conf->MID_VALUE - HALF_ECD_RANGE)))
		slave.info->tx_info->status |= 0x0800;	
	
	/*  13:卡墙处理标志位  */
	slave.info->tx_info->status &= 0xEFFF;
	if (status.block_cmd == block_on)
		slave.info->tx_info->status |= 0x1000;
	
	/*  yaw轴电机角度数据  */
	slave.info->tx_info->motor_angle = RM_motor[GIM_Y].rx_info.angle;
	/*  yaw轴陀螺仪角度数据  */
	slave.info->tx_info->imu_angle = imu_sensor.info->base_info.yaw;
	
}

void Vision_TxInfoUpdate(void)
{
	vision_sensor.info->measure_pitch_angle = imu_sensor.info->base_info.pitch * ANGLE_TO_ECD + (float)HALF_ECD_RANGE;
	vision_sensor.info->measure_yaw_angle = (imu_sensor.info->base_info.yaw + 180.f) * ANGLE_TO_ECD;
	vision_sensor.info->measure_shoot_speed = launcher.conf->fric_mode;
	
	vision_sensor.info->tx_info->pitch_angle = vision_sensor.info->measure_pitch_angle;
	vision_sensor.info->tx_info->yaw_angle = vision_sensor.info->measure_yaw_angle;
	vision_sensor.info->tx_info->shoot_speed = vision_sensor.info->measure_shoot_speed;
	vision_sensor.info->tx_info->my_color = slave.info->my_color;
	
	vision_sensor.info->tx_info->mode = AIM_ON;
	if (status.buff_cmd == small_buff_on)
	{
		vision_sensor.info->tx_info->mode = AIM_SMALL_BUFF;
	}
	if (status.buff_cmd == big_buff_on)
	{
		vision_sensor.info->tx_info->mode = AIM_BIG_BUFF;
	}
	vision_sensor.info->cmd_mode = vision_sensor.info->tx_info->mode;
	
}

void Rc_RxInfoCheck(void)
{
	/*  遥控器清空热量  */
	if (module.mode == MODULE_MODE_GYRO)
	{
		status.lch_cmd.shoot_cmd = shoot_reset;
		if ((rc_sensor.info->thumbwheel.status == RC_TB_MID) && (status.tw_last_state == RC_TB_UP))
		{
			if (status.lch_state.magz_state == magz_close)
			{
				status.lch_cmd.shoot_cmd = swift_shoot;
			}
		}
	}
	/*  遥控器加速  */
	status.speed_cmd = rapid_off;
	if (module.mode == MODULE_MODE_GYRO2)
	{
		status.speed_cmd = rapid_on;
	}
	// 遥控器视觉
	if (module.mode == MODULE_MODE_GYRO2)
	{
		if ((rc_sensor.info->thumbwheel.status == RC_TB_MID) && (status.tw_last_state == RC_TB_UP))
		{
			if (vision_sensor.work_state == DEV_ONLINE)
			{
				if (status.gim_mode == gyro)
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
			else
			{
				status.gim_mode = gyro;
				gimbal.info->gimbal_mode = gim_gyro;
			}
		}
	}
//		if ((rc_sensor.info->thumbwheel.status == RC_TB_MID) && (status.tw_last_state == RC_TB_DOWN))
//		{
//			status.gim_mode = gyro;
//			gimbal.info->gimbal_mode = gim_gyro;
//		}
	/*  遥控器陀螺仪校正  */
	if (module.state == MODULE_STATE_IMUTMP)
	{
		if ((rc_sensor.info->s2 == RC_SW_DOWN) && (rc_sensor.info->thumbwheel.status == RC_TB_MID) && (status.tw_last_state == RC_TB_UP))
		{
			imu_sensor.work_state.err_code = IMU_DATA_CALI;
		}
		if ((rc_sensor.info->s2 != RC_SW_DOWN) && (imu_sensor.work_state.err_code != IMU_DATA_CALI))
		{
			module.state = MODULE_STATE_NORMAL;
		}
	}
	/*  关弹仓  */
	if (status.chas_state == gyro_on)
	{
		status.lch_cmd.magz_cmd = magz_close;
	}
}

uint8_t rst_cnt = 0;
void Key_RxInfoCheck(void)
{
	Key_StateUpdate();
	
	symbal.slave_reset = 0;
	status.lch_cmd.fric_cmd = fric_reset;
//	keyboard.lch_cmd.magz_cmd = lch_reset;
	status.lch_cmd.shoot_cmd = shoot_reset;
	status.heat_mode = heat_limit_on;
	status.buff_cmd = buff_reset;
	status.gim_mode = gyro;
	/*  Ctrl(同时):关弹仓 关侧身 关小陀螺  */
	if (keyboard.state.Ctrl == down_K)
	{
		if (status.lch_state.magz_state == magz_open)
		{
			status.lch_cmd.magz_cmd = magz_close;
		}
//		if (status.lch_state.fric_state == fric_on)
//		{
//			status.lch_cmd.fric_cmd = fric_off;
//		}
		if (status.chas_state == gyro_on) 
		{
			status.chas_cmd = chas_reset;
		}
		else if (status.chas_state == lean_on) 
		{
			status.chas_cmd = chas_reset;
		}
	}
	/*  Q/E:转头90°  */
	if ((keyboard.state.Q == down_K) && (status.lch_state.magz_state == magz_close))
	{
		symbal.gim_sym.turn_left = 1;
	}
	if ((keyboard.state.E == down_K) && (status.lch_state.magz_state == magz_close))
	{
		symbal.gim_sym.turn_right = 1;
	}
	/*  R:速射  */
	if (keyboard.state.R == down_K)
	{
		status.lch_cmd.fric_cmd = fric_on;
		status.lch_cmd.magz_cmd = magz_close;
	}
	if ((keyboard.state.R == down_K) && (status.lch_state.fric_state == fric_on))
	{
		status.lch_cmd.shoot_cmd = swift_shoot;
	}
	/*  A/D:侧身  */
	if ((keyboard.state.A == down_K) || (keyboard.state.D == down_K))
	{
		status.chas_cmd = lean_on;
	}
	/*  F:小陀螺  */
	if (keyboard.state.F == down_K)
	{
		status.chas_cmd = gyro_on;
	}
	/*  G:热量解锁  */
	if ((keyboard.state.G == short_press_K) || (keyboard.state.G == long_press_K))
	{
		status.heat_mode = heat_limit_off;
	}
	/*  X:小符  */
	if ((keyboard.state.X == short_press_K) || (keyboard.state.X == long_press_K))
	{
		status.chas_cmd = lean_on;
		status.buff_cmd = small_buff_on;
		status.gim_mode = vision;
	}
	/*  Z:大符  */
	if ((keyboard.state.Z == short_press_K) || (keyboard.state.Z == long_press_K))
	{
		status.chas_cmd = lean_on;
		status.buff_cmd = big_buff_on;
		status.gim_mode = vision;
	}
	/*  自动打符  */
	if ((keyboard.state.X == down_K) || (keyboard.state.Z == down_K))
	{
		status.autobuff_cmd = auto_buff_on;
		status.lch_cmd.fric_cmd = fric_on;
		status.lch_cmd.magz_cmd = magz_close;
	}
	
	if ((keyboard.state.X == up_K) || (keyboard.state.Z == up_K))
	{
		status.chas_cmd = chas_reset;
		status.autobuff_cmd = auto_buff_off;
	}
	if ((keyboard.state.X == relax_K) && (keyboard.state.Z == relax_K))
	{
		status.autobuff_cmd = auto_buff_off;
	}
	/*  ZX:反符  */
//	if ((keyboard.state.X != relax_K) && (keyboard.state.Z != relax_K))
//	{
//		status.buff_cmd = debuff_on;
//	}
	
	/*  V:切换目标  */
	if (keyboard.state.V == down_K)
	{
		vision_sensor.info->tx_info->is_change_target ++;
	}
	
	
	if (status.lch_state.fric_state == fric_on)
	{
		status.lch_cmd.magz_cmd = magz_close;
	}
	/*  B:开弹仓  */
	if ((keyboard.state.B == down_K) && (symbal.gim_sym.turn_ok == 1))
	{
		status.lch_cmd.magz_cmd = magz_open;
		status.lch_cmd.fric_cmd = fric_off;
	}
	/*  长按B:关弹仓  */
	if (keyboard.state.B == long_press_K)
	{
		status.lch_cmd.magz_cmd = magz_close;
	}
	if (status.chas_state == gyro_on)
	{
		status.lch_cmd.magz_cmd = magz_close;
	}
	if (status.chas_state == lean_on)
	{
		status.lch_cmd.magz_cmd = magz_close;
	}
	
	/*  mouse_btn_l  */
	if (keyboard.state.mouse_btn_l == down_K)
	{
		status.lch_cmd.fric_cmd = fric_on;
		status.lch_cmd.magz_cmd = magz_close;
	}
	if ((keyboard.state.mouse_btn_l == down_K) && (status.lch_state.fric_state == fric_on))
	{
		status.lch_cmd.shoot_cmd = single_shoot;
	}
	if ((keyboard.state.mouse_btn_l == long_press_K) && (status.lch_state.fric_state == fric_on))
	{
		status.lch_cmd.shoot_cmd = keep_shoot;
	}
	
	
	/*  长按Shift:加速  */
	status.speed_cmd = speed_reset;
	if (keyboard.state.Shift != relax_K)
		status.speed_cmd = rapid_on;
	
	/*  长按Ctrl:卡墙处理  */
	status.block_cmd = block_reset;
	if (keyboard.state.Ctrl == long_press_K)
	{
		status.block_cmd = block_on;
		status.chas_cmd = chas_reset;
	}
	if (keyboard.state.Ctrl == up_K)
	{
		status.lch_cmd.magz_cmd = magz_close;
	}
	
	status.gim_cmd = gim_reset;
	/*  C:掉头  */
	if ((keyboard.state.C == down_K) && (status.lch_state.magz_state == magz_close))
	{
		symbal.gim_sym.turn_start = 1;
		symbal.gim_sym.turn_ok = 0;
		status.gim_cmd = gim_turn;
	}
	
	/*  mouse_btn_r:自动打弹  */
	if (keyboard.state.mouse_btn_r == down_K)
	{
//		status.auto_cmd = auto_shoot_on;
	}
	if (keyboard.state.mouse_btn_r == relax_K)
	{
		status.auto_cmd = auto_shoot_off;
	}
	if (keyboard.state.mouse_btn_l != relax_K)
	{
		status.auto_cmd = auto_shoot_off;
		status.autobuff_cmd = auto_buff_off;
	}
	
	/*  mouse_btn_r:自瞄  */
	if ((keyboard.state.mouse_btn_r == short_press_K) || (keyboard.state.mouse_btn_r == long_press_K))
	{
		status.gim_mode = vision;
	}
	
	/*  ZXV:上下主控复位  */
	if ((keyboard.state.Z != relax_K) && (keyboard.state.X != relax_K) && (keyboard.state.V != relax_K))
	{
		symbal.slave_reset = 1;
		if (++rst_cnt == 20)
		{
			__set_FAULTMASK(1); 
			NVIC_SystemReset();
		}
	}
	else
	{
		rst_cnt = 0;
	}
//	if ((keyboard.state.Z == long_press_K) && (keyboard.state.X == long_press_K) && (keyboard.state.V == long_press_K))
//	{
//		memset(can1_send_buf, 0, 16);
//		memset(can2_send_buf, 0, 16);
//		CAN_SendAll();
//		Magazine_Sleep();
//		
//		__set_FAULTMASK(1); 
//		NVIC_SystemReset();
//	}
	
	
	
	
	
	/*  遥控器复位  */
	if ((rc_sensor.info->s2 == RC_SW_UP) && (rc_sensor.info->thumbwheel.status == RC_TB_MID) && (status.tw_last_state == RC_TB_UP))
	{
		static uint8_t cnt = 0;
		symbal.slave_reset = 1;
		if(cnt++ == 5)
		{
			__set_FAULTMASK(1);
			NVIC_SystemReset();
		}
	}
	
	/*  遥控器卡墙处理  */
//	if ((rc_sensor.info->s2 == RC_SW_UP) && (abs(rc_sensor.info->ch1) == 660))
//	{
//		status.block_cmd = block_on;
//	}
	
	
	/*  遥控器陀螺仪校正  */
	if ((rc_sensor.info->s2 == RC_SW_DOWN) && (rc_sensor.info->thumbwheel.status == RC_TB_MID) && (status.tw_last_state == RC_TB_DOWN))
	{
		module.state = MODULE_STATE_IMUTMP;
		imu_sensor.info->offset_info.gx_offset = 0.f;
		imu_sensor.info->offset_info.gy_offset = 0.f;
		imu_sensor.info->offset_info.gz_offset = 0.f;
	}
	
}

