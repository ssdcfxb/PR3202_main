/**
 * @file        launcher.c
 * @author      SSDCFXB
 * @Version     V1.0
 * @date        20-November-2022
 * @brief       Launcher Control Center
 */

/* Includes ------------------------------------------------------------------*/
#include "launcher.h"

#include "rp_math.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void Launcher_Init(void);
void Launcher_Ctrl(void);
void Launcher_SelfProtect(void);

void Launcher_GetInfo(void);
void Launcher_GetBaseInfo(void);
void Judge_GetInfo(void);
void Judge_GetSpeedInfo(void);
void Judge_AdaptFricSpeed(void);
void Judge_AdaptDialSpeed(void);
void Launcher_GetCtrlInfo(void);
void Launcher_GetRcState(void);
void Launcher_GetKeyState(void);
void Get_LauncherStatus(void);
void Fric_StatusCheck(void);
void Dial_StatusCheck(void);

void Launcher_MotorCtrl(void);
void Fric_Ctrl(void);
void Dial_Ctrl(void);

void Launcher_SendOut(void);

void Launcher_Stop(void);

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
int16_t    launcher_out[3];

// 发射机构设备
launcher_dev_t		launcher_dev = {
	.fricl_motor = &RM_motor[FRIC_L],
	.fricr_motor = &RM_motor[FRIC_R],
	.dial_motor = &RM_motor[DIAL],
	.rc_sensor = &rc_sensor,
};

// 发射机构信息
launcher_info_t 	launcher_info = {
	.remote_mode = RC,
};

launcher_work_info_t  launcher_work_info = {
	.fric_status = WaitCommond_Fric,
	.dial_status = WaitCommond_Dial,
	.launcher_commond.Fric_cmd = Fric_Reset,
	.launcher_commond.Magz_cmd = Magz_Reset,
	.launcher_commond.Dial_cmd = Shoot_Reset,
	.lock_cnt = 0,
	.shoot_cnt = 0,
};

launcher_conf_t   launcher_conf = {
	.Fric_15 = Fric_15_init,
	.Fric_18 = Fric_18_init,
	.Fric_20 = Fric_20_init,
	.Fric_22 = Fric_22_init,
	.Fric_30 = Fric_30_init,
	.dial_speed = -4000.0f,
	.dial_swiftspeed = -6000.0f,
	.dial_torque_limit = 2000.0f,
	.lock_angle_check = 1.5f,
	.lock_cnt = 50,
	.Back_Angle = 45.0f,
	.Load_Angle = -45.0f,
//	.wait_time = 1000,  //发射间隔时间，单位ms
};

/* Exported variables --------------------------------------------------------*/
extern int16_t can2_send_buf[8];

launcher_t launcher = {
	.dev = &launcher_dev,
	.info = &launcher_info,
	.work_info = &launcher_work_info,
	.conf = &launcher_conf,
	.init = Launcher_Init,
	.ctrl = Launcher_Ctrl,
	.self_protect = Launcher_SelfProtect,
};

/* Exported functions --------------------------------------------------------*/
/**
  * @brief  发射机构初始化
  * @param  
  * @retval 
  */
void Launcher_Init(void)
{
	if (rc_sensor.work_state == DEV_ONLINE)
	{
		launcher.info->init_s2 = rc_sensor.info->s2;
		launcher.info->last_s2 = rc_sensor.info->s2;
	}
	
	launcher.conf->fric_speed = launcher.conf->Fric_15;
	launcher.conf->fric_mode = 15;
	
	launcher.info->rc_work_state = rc_sensor.work_state;
	
}


/**
  * @brief  发射机构控制
  * @param  
  * @retval 
  */
void Launcher_Ctrl(void)
{
	Launcher_GetInfo();
	
	Launcher_MotorCtrl();
	
	Launcher_SendOut();
}


/**
  * @brief  发射机构控制离线保护
  * @param  
  * @retval 
  */
void Launcher_SelfProtect(void)
{
	Launcher_Stop();
	Launcher_GetInfo();
}


/* Private functions ---------------------------------------------------------*/
/**
  * @brief  读取发射机构相关信息
  * @param  
  * @retval 
  */
void Launcher_GetInfo(void)
{
	Launcher_GetBaseInfo();
	
	Launcher_GetCtrlInfo();
	
	Get_LauncherStatus();
}


/**
  * @brief  读取发射机构电机信息
  * @param  
  * @retval 
  */
void Launcher_GetBaseInfo(void)
{
	launcher.info->measure_left_speed = RM_motor[FRIC_L].rx_info.speed;
	launcher.info->measure_right_speed = RM_motor[FRIC_R].rx_info.speed;
	launcher.info->measure_dial_speed = RM_motor[DIAL].rx_info.speed;
	launcher.info->measure_dial_angle = RM_motor[DIAL].rx_info.angle_sum * M2006_ECD_TO_ANGLE;
	
	Judge_GetInfo();
}


/**
  * @brief  读取裁判系统信息
  * @param  
  * @retval 
  */
void Judge_GetInfo(void)
{
	Judge_GetSpeedInfo();
	
	Judge_AdaptFricSpeed();
	
	Judge_AdaptDialSpeed();
	
}


/**
  * @brief  读取限制弹速与测量弹速
  * @param  
  * @retval 
  */
void Judge_GetSpeedInfo(void)
{
	launcher.info->limit_speed = (float)judge.info->game_robot_status.shooter_id1_17mm_speed_limit;
	launcher.info->measure_launcher_speed = judge.info->shoot_data.bullet_speed;
	
	if (judge.work_state == DEV_OFFLINE)
	{
		launcher.conf->fric_speed = launcher.conf->Fric_30;
		launcher.conf->fric_mode = 30;
	}
	else
	{
		switch (judge.info->game_robot_status.shooter_id1_17mm_speed_limit)
		{
			case 0:
				launcher.conf->fric_speed = 0;
				launcher.conf->fric_mode = 0;
				break;
			case 15:
				launcher.conf->fric_speed = launcher.conf->Fric_15;
				launcher.conf->fric_mode = 15;
				break;
			case 18:
				launcher.conf->fric_speed = launcher.conf->Fric_18;
				launcher.conf->fric_mode = 18;
				break;
			case 20:
				launcher.conf->fric_speed = launcher.conf->Fric_20;
				launcher.conf->fric_mode = 20;
				break;
			case 22:
				launcher.conf->fric_speed = launcher.conf->Fric_22;
				launcher.conf->fric_mode = 22;
				break;
			case 30:
				launcher.conf->fric_speed = launcher.conf->Fric_30;
				launcher.conf->fric_mode = 30;
				break;
			default:
				launcher.conf->fric_speed = 0;
				launcher.conf->fric_mode = 0;
				break;
		}
	}
}


/**
  * @brief  调整摩擦轮转速
  * @param  
  * @retval 
  */
void Judge_AdaptFricSpeed(void)
{
	static int8_t cnt = 0, speed_adapt = 0, adapt_k = 5;
	
	if (judge.work_state == DEV_ONLINE)
	{
		if (launcher.info->measure_launcher_speed > (launcher.info->limit_speed - 0.95f))
		{
			cnt++;
			if (cnt > 2)
			{
				cnt = 0;
				speed_adapt = -1;
			}
		}
		else if (launcher.info->measure_launcher_speed < (launcher.info->limit_speed - 1.5f))
		{
			cnt--;
			if (cnt < -2)
			{
				cnt = 0;
				speed_adapt = 1;
			}
		}
		else
		{
			cnt = 0;
			speed_adapt = 0;
		}
		launcher.conf->fric_speed = launcher.conf->fric_speed + speed_adapt * adapt_k;
	}
	
}


/**
  * @brief  调整拨盘转速
  * @param  
  * @retval 
  */
void Judge_AdaptDialSpeed(void)
{
	static uint8_t heat_low = 0, heat_high = 0;
	
	launcher.info->limit_heat = judge.info->game_robot_status.shooter_id1_17mm_cooling_limit;
	launcher.info->measure_launcher_heat = judge.info->power_heat_data.shooter_id1_17mm_cooling_heat;
	
	if ((launcher.info->limit_heat - launcher.info->measure_launcher_heat) > 50)
	{
		heat_low = 1;
	}
	else 
	{
		heat_low = 0;
	}
	if ((launcher.info->limit_heat - launcher.info->measure_launcher_heat) < 20)
	{
		heat_high = 1;
	}
	else
	{
		heat_high = 0;
	}
	
	
	if (judge.work_state == DEV_OFFLINE)
	{
		launcher.conf->dial_speed = -4000.0f;
		launcher.conf->dial_swiftspeed = -6000.f;
	}
	else
	{
		if (status.heat_mode == heat_limit_on)
		{
			if (heat_low == 1)
			{
				launcher.conf->dial_speed = -5000.0f;
			}
			else
			{
				launcher.conf->dial_speed = -4000.0f;
			}
			if (heat_high == 1)
			{
				launcher.conf->dial_speed = -0.0f;
				launcher.conf->dial_swiftspeed = -0.f;
			}
			else
			{
				launcher.conf->dial_speed = -4000.0f;
				launcher.conf->dial_swiftspeed = -6000.f;
			}
		}
		else
		{
			launcher.conf->dial_speed = -4000.0f;
			launcher.conf->dial_swiftspeed = -6000.f;
		}
	}
}

/**
  * @brief  读取遥控器控制信息
  * @param  
  * @retval 
  */
void Launcher_GetCtrlInfo(void)
{
	if (launcher.info->remote_mode == RC)
	{
		Launcher_GetRcState();
	}
	else if(launcher.info->remote_mode == KEY) {
		Launcher_GetKeyState();
	}
}


/**
  * @brief  读取遥控器信息
  * @param  
  * @retval 
  */
void Launcher_GetRcState(void)
{
	/**    读取遥控器上电时刻信息    **/
	if ((launcher.info->rc_work_state == DEV_OFFLINE) \
		&& (rc_sensor.work_state == DEV_ONLINE))
	{
		launcher.info->init_s2 = rc_sensor.info->s2;
		launcher.info->last_s2 = rc_sensor.info->s2;
	}
	
	
	if (rc_sensor.work_state == DEV_ONLINE)
	{
		/**    模式切换保护    **/
		if (launcher.info->last_s1 != rc_sensor.info->s1)
		{
			launcher.info->init_s2 = rc_sensor.info->s2;
		}
		
		/**    遥控器上电保护    **/
		if (launcher.info->init_s2 == rc_sensor.info->s2)
		{
			launcher.work_info->launcher_commond.Fric_cmd = Fric_Reset;
			launcher.work_info->launcher_commond.Magz_cmd = Magz_Reset;
			launcher.work_info->launcher_commond.Dial_cmd = Shoot_Reset;
		}
		else
		{
			launcher.info->init_s2 = 0;
			
			/**    读取遥控器拨杆跳变信息    **/
			if (launcher.info->launcher_mode == lch_gyro)
			{
				
				if (rc_sensor.info->s2 == RC_SW_MID)
				{
					launcher.work_info->launcher_commond.Fric_cmd = Fric_Reset;
					launcher.work_info->launcher_commond.Dial_cmd = Shoot_Reset;
					if (launcher.info->last_s2 == RC_SW_DOWN)
						launcher.work_info->launcher_commond.Fric_cmd = Fric_Toggle;
				}
				else if (rc_sensor.info->s2 == RC_SW_UP)
				{
					launcher.work_info->launcher_commond.Dial_cmd = Keep_Shoot;
					status.lch_state.shoot_state = keep_shoot;
					if (launcher.info->last_s2 != rc_sensor.info->s2)
					{
						launcher.work_info->dial_status = Reload_Dial;
					}
				}
				else if (rc_sensor.info->s2 == RC_SW_DOWN)
				{
					launcher.work_info->launcher_commond.Fric_cmd = Fric_Reset;
				}
				
			}
			else if ((launcher.info->launcher_mode == lch_machine) || (launcher.info->launcher_mode == lch_gyro2))
			{
				
				if (rc_sensor.info->s2 == RC_SW_MID)
				{
					launcher.work_info->launcher_commond.Magz_cmd = Magz_Reset;
					launcher.work_info->launcher_commond.Dial_cmd = Shoot_Reset;
				}
				else if (rc_sensor.info->s2 == RC_SW_UP)
				{
					launcher.work_info->launcher_commond.Dial_cmd = Single_Shoot;
					status.lch_state.shoot_state = single_shoot;
					if (launcher.info->last_s2 != rc_sensor.info->s2)
					{
						launcher.work_info->dial_status = Reload_Dial;
						launcher.info->target_dial_angle = launcher.conf->Load_Angle + launcher.info->measure_dial_angle;
					}
				}
				else if (rc_sensor.info->s2 == RC_SW_DOWN)
				{
					launcher.work_info->launcher_commond.Magz_cmd = Magz_Open;
				}
				
			}
			else 
			{
				/**    遥控器离线信息    **/
				launcher.work_info->launcher_commond.Fric_cmd = Fric_Reset;
				launcher.work_info->launcher_commond.Magz_cmd = Magz_Reset;
				launcher.work_info->launcher_commond.Dial_cmd = Shoot_Reset;
				launcher.work_info->fric_status = WaitCommond_Fric;
				launcher.work_info->dial_status = WaitCommond_Dial;
			}
		}
		
//		if (status.lch_cmd.magz_cmd == magz_close)
//		{
//			launcher.work_info->launcher_commond = Magz_Close;
//		}
	}
	else if(launcher.info->rc_work_state == DEV_ONLINE)
	{
		/**    遥控器离线跳变信息    **/
		launcher.work_info->launcher_commond.Fric_cmd = Fric_Reset;
		launcher.work_info->launcher_commond.Magz_cmd = Magz_Reset;
		launcher.work_info->launcher_commond.Dial_cmd = Shoot_Reset;
		launcher.work_info->fric_status = Off_Fric;
		launcher.work_info->dial_status = WaitCommond_Dial;
	}
	else 
	{
		/**    遥控器离线信息    **/
		launcher.work_info->launcher_commond.Fric_cmd = Fric_Reset;
		launcher.work_info->launcher_commond.Magz_cmd = Magz_Reset;
		launcher.work_info->launcher_commond.Dial_cmd = Shoot_Reset;
		launcher.work_info->fric_status = WaitCommond_Fric;
		launcher.work_info->dial_status = WaitCommond_Dial;
	}
	
}


/**
  * @brief  读取键盘信息
  * @param  
  * @retval 
  */
void Launcher_GetKeyState(void)
{
	launcher.work_info->launcher_commond.Fric_cmd = Fric_Reset;
	launcher.work_info->launcher_commond.Magz_cmd = Magz_Reset;
//	launcher.work_info->launcher_commond.Dial_cmd = Shoot_Reset;
	/*  拨盘指令  */
	if (status.lch_cmd.shoot_cmd == keep_shoot)
	{
		launcher.work_info->launcher_commond.Dial_cmd = Keep_Shoot;
		if (status.lch_state.shoot_state != keep_shoot)
		{
			status.lch_state.shoot_state = keep_shoot;
			launcher.work_info->dial_status = Reload_Dial;
		}
	}
	if (status.lch_cmd.shoot_cmd == single_shoot)
	{
		launcher.work_info->launcher_commond.Dial_cmd = Single_Shoot;
		if (status.lch_state.shoot_state != single_shoot)
		{
			status.lch_state.shoot_state = single_shoot;
			launcher.work_info->dial_status = Reload_Dial;
			launcher.info->target_dial_angle = launcher.conf->Load_Angle + launcher.info->measure_dial_angle;
		}
	}
	if (status.lch_cmd.shoot_cmd == swift_shoot)
	{
		launcher.work_info->launcher_commond.Dial_cmd = Swift_Shoot;
		if (status.lch_state.shoot_state != swift_shoot)
		{
			status.lch_state.shoot_state = swift_shoot;
			launcher.work_info->dial_status = Reload_Dial;
		}
	}
	if (status.lch_cmd.shoot_cmd == shoot_reset)
	{
		if (status.lch_state.shoot_state != single_shoot)
		{
			status.lch_state.shoot_state = shoot_reset;
			launcher.work_info->launcher_commond.Dial_cmd = Shoot_Reset;
		}
	}
	
	/*  弹仓指令  */
	if (status.lch_cmd.magz_cmd == magz_open)
	{
		launcher.work_info->launcher_commond.Magz_cmd = Magz_Open;
	}
	else if (status.lch_cmd.magz_cmd == magz_close)
	{
		launcher.work_info->launcher_commond.Magz_cmd = Magz_Close;
	}
	
	/*  摩擦轮指令  */
	if (status.lch_cmd.fric_cmd == fric_on)
	{
		launcher.work_info->launcher_commond.Fric_cmd = Fric_Open;
	}
	else if (status.lch_cmd.fric_cmd == fric_off)
	{
		launcher.work_info->launcher_commond.Fric_cmd = Fric_Close;
	}
	
}


/**
  * @brief  读取发射机构当前状态
  * @param  
  * @retval 
  */
void Get_LauncherStatus(void)
{
	Fric_StatusCheck();
	
	Dial_StatusCheck();
	
	launcher.info->rc_work_state = rc_sensor.work_state;
	launcher.info->last_s1 = rc_sensor.info->s1;
	launcher.info->last_s2 = rc_sensor.info->s2;
}



/**
  * @brief  摩擦轮开启跳变检测与弹仓盖开关
  * @param  
  * @retval 
  */
void Fric_StatusCheck(void)
{
	/*  遥控器开关摩擦轮  */
	if ((launcher.work_info->launcher_commond.Fric_cmd == Fric_Toggle) \
		&& (launcher.info->last_s2 != rc_sensor.info->s2))
	{
		if (launcher.work_info->fric_status == On_Fric)
		{
			launcher.work_info->fric_status = Off_Fric;
			status.lch_state.fric_state = fric_off;
		}
		else
		{
			launcher.work_info->fric_status = On_Fric;
		}
	}
	
	/*  键盘开关摩擦轮  */
	if (launcher.work_info->launcher_commond.Fric_cmd == Fric_Open)
	{
		launcher.work_info->fric_status = On_Fric;
	}
	if (launcher.work_info->launcher_commond.Fric_cmd == Fric_Close)
	{
		launcher.work_info->fric_status = Off_Fric;
		status.lch_state.fric_state = fric_off;
	}
	
	if (launcher.work_info->fric_status == On_Fric)
	{
		if(((uint16_t)launcher.conf->fric_speed - launcher.info->measure_right_speed) < 200)
		{
			status.lch_state.fric_state = fric_on;
		}
	}
	
	
	/*  开关弹仓  */
	if (launcher.work_info->launcher_commond.Magz_cmd == Magz_Open)
	{
		Magazine_Open();
		launcher.work_info->fric_status = Off_Fric;
		status.lch_state.magz_state = magz_open;
		status.lch_cmd.fric_cmd = fric_off;
	}
	else
	{
		Magazine_Close();
		status.lch_state.magz_state = magz_close;
	}
	
	/*  弹仓卸力  */
	if (rc_sensor.work_state == DEV_OFFLINE)
	{
		Magazine_Sleep();
		status.lch_state.magz_state = magz_reset;
	}
}


/**
  * @brief  更新拨盘控制信息
  * @param  
  * @retval 
  */
void Dial_StatusCheck(void)
{
	/**    拨盘功能复位    **/
	if (launcher.work_info->launcher_commond.Dial_cmd == Shoot_Reset)
	{
		launcher.work_info->dial_status = WaitCommond_Dial;
	}
	
	/**    摩擦轮状态检测    **/
	if (launcher.work_info->fric_status != On_Fric)
	{
		launcher.work_info->dial_status = WaitCommond_Dial;
	}
	
	/**    更新拨盘控制信息    **/
	if (launcher.work_info->dial_status == Reload_Dial)
	{
		if (launcher.info->measure_dial_angle < launcher.info->target_dial_angle + launcher.conf->lock_angle_check)
		{
			if (launcher.work_info->launcher_commond.Dial_cmd == Single_Shoot)
			{
				/**    单发    **/
				launcher.work_info->dial_status = WaitCommond_Dial;
				launcher.work_info->launcher_commond.Dial_cmd = Shoot_Reset;
				status.lch_state.shoot_state = shoot_reset;
			}
			else if (launcher.work_info->launcher_commond.Dial_cmd == Keep_Shoot)
			{
				/**    连发    **/
				launcher.work_info->dial_status = SpeedKeep_Dial;
			}
			else if (launcher.work_info->launcher_commond.Dial_cmd == Swift_Shoot)
			{
				/**    速射    **/
				launcher.work_info->dial_status = SwiftKeep_Dial;
			}
		}
		else 
		{
			/**    拨盘角度堵转检测    **/
			if (abs(launcher.info->measure_dial_speed) < 5)
			{
				launcher.work_info->lock_cnt ++;
				if (launcher.work_info->lock_cnt > launcher.conf->lock_cnt)
				{
					launcher.work_info->dial_status = F_Lock_Dial;
					launcher.info->target_dial_angle = launcher.conf->Back_Angle + launcher.info->measure_dial_angle;
					launcher.work_info->lock_cnt = 0;
				}
			}
			else 
			{
				launcher.work_info->lock_cnt = 0;
			}
		}
	}
	else if (launcher.work_info->dial_status == SpeedKeep_Dial)
	{
		/**    拨盘速度堵转检测    **/
		if (RM_motor[DIAL].ctr_stuck_flag(&RM_motor[DIAL], launcher.conf->dial_torque_limit))
		{
			launcher.work_info->dial_status = F_Lock_Dial;
			launcher.info->target_dial_angle = launcher.conf->Back_Angle + launcher.info->measure_dial_angle;
		}
		else
		{
			launcher.info->target_dial_speed = launcher.conf->dial_speed;
		}
	}
	else if (launcher.work_info->dial_status == SwiftKeep_Dial)
	{
		/**    拨盘速度堵转检测    **/
		if (RM_motor[DIAL].ctr_stuck_flag(&RM_motor[DIAL], launcher.conf->dial_torque_limit))
		{
			launcher.work_info->dial_status = F_Lock_Dial;
			launcher.info->target_dial_angle = launcher.conf->Back_Angle + launcher.info->measure_dial_angle;
		}
		else
		{
			launcher.info->target_dial_speed = launcher.conf->dial_swiftspeed;
		}
	}
	else if (launcher.work_info->dial_status == F_Lock_Dial)
	{
		/**    拨盘堵转处理    **/
		if (launcher.info->measure_dial_angle > launcher.info->target_dial_angle - launcher.conf->lock_angle_check)
		{
			launcher.info->target_dial_angle = launcher.conf->Load_Angle + launcher.info->measure_dial_angle;
			launcher.work_info->dial_status = Reload_Dial;
		}
		else 
		{
			if (abs(launcher.info->measure_dial_speed) < 5)
			{
				launcher.work_info->lock_cnt ++;
				if (launcher.work_info->lock_cnt > launcher.conf->lock_cnt)
				{
					launcher.work_info->dial_status = Reload_Dial;
					launcher.info->target_dial_angle = -launcher.conf->Back_Angle + launcher.info->measure_dial_angle;
					launcher.work_info->lock_cnt = 0;
				}
			}
			else 
			{
				launcher.work_info->lock_cnt = 0;
			}
		}
	}
	else
	{
		launcher.work_info->dial_status = WaitCommond_Dial;
//		launcher.info->target_dial_angle = launcher.info->measure_dial_angle;
	}
}



/**
  * @brief  发射机构电机控制
  * @param  
  * @retval 
  */
void Launcher_MotorCtrl(void)
{
	Fric_Ctrl();
	Dial_Ctrl();
}


/**
  * @brief  摩擦轮电机控制
  * @param  
  * @retval 
  */
void Fric_Ctrl(void)
{
	if (launcher.work_info->fric_status == On_Fric)
	{
		launcher.info->target_left_speed = -launcher.conf->fric_speed;
		launcher.info->target_right_speed = launcher.conf->fric_speed;
	
		launcher_out[RM_motor[FRIC_L].id.buff_p] = RM_motor[FRIC_L].ctr_speed(&RM_motor[FRIC_L], 
		                                        launcher.info->target_left_speed);
		
		launcher_out[RM_motor[FRIC_R].id.buff_p] = RM_motor[FRIC_R].ctr_speed(&RM_motor[FRIC_R], 
		                                        launcher.info->target_right_speed);
		
	}
	else if (launcher.work_info->fric_status == Off_Fric || launcher.work_info->fric_status == WaitCommond_Fric)
	{
		launcher.info->target_left_speed = 0.0f;
		launcher.info->target_right_speed = 0.0f;
		
		launcher_out[RM_motor[FRIC_L].id.buff_p] = RM_motor[FRIC_L].ctr_speed(&RM_motor[FRIC_L], 
		                                        launcher.info->target_left_speed);
		
		launcher_out[RM_motor[FRIC_R].id.buff_p] = RM_motor[FRIC_R].ctr_speed(&RM_motor[FRIC_R], 
		                                        launcher.info->target_right_speed);
		
	}
}

/**
  * @brief  拨盘电机控制
  * @param  
  * @retval 
  */
void Dial_Ctrl(void)
{
	if ((launcher.work_info->dial_status == SpeedKeep_Dial) || (launcher.work_info->dial_status == SwiftKeep_Dial))
	{
		
		launcher_out[RM_motor[DIAL].id.buff_p] = RM_motor[DIAL].ctr_pid1(&RM_motor[DIAL].pid.position_in, 
		                                                         RM_motor[DIAL].rx_info.speed,
                                                      		   launcher.info->target_dial_speed);
		
	}
	else if (launcher.work_info->dial_status != WaitCommond_Dial)
	{
		// 过零点处理
		if (launcher.info->target_dial_angle < -360.0f)
		{
			launcher.info->target_dial_angle += 360.0f;
		}
		else if (launcher.info->target_dial_angle > 360.0f)
		{
			launcher.info->target_dial_angle -= 360.0f;
		}
		
		if (launcher.info->target_dial_angle - launcher.info->measure_dial_angle > 180.0f)
		{
			launcher.info->target_dial_angle -= 360.0f;
		}
		else if (launcher.info->target_dial_angle - launcher.info->measure_dial_angle < -180.0f)
		{
			launcher.info->target_dial_angle += 360.0f;
		}
		
		launcher_out[RM_motor[DIAL].id.buff_p] = RM_motor[DIAL].ctr_posit(&RM_motor[DIAL], 
		                                      launcher.info->target_dial_angle / M2006_ECD_TO_ANGLE);
		
	}
	else 
	{
		launcher_out[RM_motor[DIAL].id.buff_p] = RM_motor[DIAL].ctr_posit(&RM_motor[DIAL], 
		                                      launcher.info->measure_dial_angle / M2006_ECD_TO_ANGLE);
		
	}
	
}




/**
  * @brief  发送发射机构电机控制数据
  * @param  
  * @retval 
  */
void Launcher_SendOut(void)
{
	if(RM_motor[FRIC_L].state.work_state == M_ONLINE)
		can2_send_buf[RM_motor[FRIC_L].id.buff_p] = launcher_out[RM_motor[FRIC_L].id.buff_p];
	else
		can2_send_buf[RM_motor[FRIC_L].id.buff_p] = 0;
	
	if(RM_motor[FRIC_R].state.work_state == M_ONLINE)
		can2_send_buf[RM_motor[FRIC_R].id.buff_p] = launcher_out[RM_motor[FRIC_R].id.buff_p];
	else
		can2_send_buf[RM_motor[FRIC_R].id.buff_p] = 0;
	
	if(RM_motor[DIAL].state.work_state == M_ONLINE)
		can2_send_buf[RM_motor[DIAL].id.buff_p] = launcher_out[RM_motor[DIAL].id.buff_p];
	else
		can2_send_buf[RM_motor[DIAL].id.buff_p] = 0;
}



/**
  * @brief  发射机构卸力
  * @param  
  * @retval 
  */
void Launcher_Stop(void)
{
	launcher_out[0] = 0;
	launcher_out[1] = 0;
	launcher_out[2] = 0;
	
	can2_send_buf[0] = launcher_out[0];
	can2_send_buf[1] = launcher_out[1];
	can2_send_buf[2] = launcher_out[2];
}
