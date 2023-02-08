
/* Includes ------------------------------------------------------------------*/
#include "gimbal.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void Gimbal_Init(void);
void Gimbal_Ctrl(void);
void Gimbal_SelfProtect(void);

void Gimbal_GetInfo(void);
void Gimbal_GetBaseInfo(void);
void Gimbal_GetRcInfo(void);
void Gimbal_GetKeyInfo(void);
void Gimbal_Reset(void);
void PID_ParamsInit(void);
void Gimbal_MotoReset(void);
void Gimbal_GyroReset(void);
void Gimbal_MotorCtrl(void);
void Gimbal_Yaw_Angle_PidCalc(void);
void Gimbal_Pitch_Angle_PidCalc(void);
void Gimbal_SendOut(void);
void Gimbal_Stop(void);

void Gimbal_Test(void);

/* Private typedef -----------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

//  0:yaw轴  1:pitch轴
int16_t  gim_out[2];

/*
	float	  kp;
	float 	ki;
	float 	kd;
	
	float   blind_err;	
	float 	integral_max;	
	float   iout_max;
	float 	out_max;
*/
float yaw_gyro_position[7] = {20.f, 0.f, 0.f, 0.f, 10000.f, 10000.f, 25000.f};
float yaw_gyro_position_in[7] = {200.f, 2.f, 0.f, 0.f, 10000.f, 15000.f, 25000.f};
float pitch_gyro_position[7] = {40.f, 0.f, 0.f, 0.f, 10000.f, 10000.f, 25000.f};
float pitch_gyro_position_in[7] = {80.f, 2.f, 0.f, 0.f, 10000.f, 15000.f, 25000.f};

float yaw_mach_position[7] = {0.638f, 0.f, 0.f, 0.f, 10000.f, 10000.f, 25000.f};
float yaw_mach_position_in[7] = {350.f, 1.f, 0.f, 0.f, 10000.f, 10000.f, 25000.f};
float pitch_mach_position[7] = {1.f, 0.f, 0.f, 0.f, 10000.f, 10000.f, 25000.f};
float pitch_mach_position_in[7] = {200.f, 7.f, 0.f, 0.f, 10000.f, 10000.f, 25000.f};

// 云台设备
gimbal_dev_t		gim_dev = {
	.yaw_motor = &RM_motor[GIM_Y],
	.pitch_motor = &RM_motor[GIM_P],
	.imu_sensor = &imu_sensor,
	.rc_sensor = &rc_sensor,
};

// 云台模块信息
gimbal_info_t 	gim_info = {
	.remote_mode = RC,
};

gimbal_conf_t   gim_conf = {
	.restart_yaw_imu_angle = 0.0f,
	.restart_pitch_imu_angle = 0.0f,
	.MID_VALUE = 300,
	.restart_yaw_motor_angle = 300, // 双枪 2100  麦轮 4777
	.restart_pitch_motor_angle = 5280, // 双枪 6600  麦轮 6900
	.rc_pitch_motor_offset = 110,
	.rc_yaw_imu_offset = 1100.0f,//3300
	.rc_pitch_imu_offset = 1100.0f,
	.max_pitch_imu_angle = 18.0f,
	.min_pitch_imu_angle = -35.0f,
	.max_pitch_motor_angle = 6100, // 双枪、麦轮 7200  
	.min_pitch_motor_angle = 5000, // 双枪、麦轮 6100
};

/* Exported variables --------------------------------------------------------*/
extern int16_t can2_send_buf[8];

gimbal_t gimbal = {
	.dev = &gim_dev,
	.info = &gim_info,
	.conf = &gim_conf,
	.init = Gimbal_Init,
	.ctrl = Gimbal_Ctrl,
	.self_protect = Gimbal_SelfProtect,
};

/* Exported functions --------------------------------------------------------*/
void Gimbal_Init(void)
{
	
}

/**
  * @brief  云台控制
  * @param  
  * @retval 
  */
void Gimbal_Ctrl(void)
{
	Gimbal_GetInfo();
	
	Gimbal_Reset();
	
//	Gimbal_Test();
	
	Gimbal_MotorCtrl();
	
	Gimbal_SendOut();
}


/**
  * @brief  云台控制离线保护
  * @param  
  * @retval 
  */
void Gimbal_SelfProtect(void)
{
	Gimbal_Stop();
	Gimbal_GetInfo();
	flag.gimbal_flag.reset_start = 1;
	flag.gimbal_flag.reset_ok = 0;
}

/* Private functions ---------------------------------------------------------*/
void Gimbal_Test(void)
{
	static uint16_t cnt = 0;
	gimbal.info->target_pitch_imu_angle = 0.f;
	if (abs(gimbal.info->target_yaw_imu_angle - gimbal.info->measure_yaw_imu_angle) < 0.5f)
	{
		cnt++;
		if (cnt == 500)
		{
			if (gimbal.info->target_yaw_imu_angle > 0)
			{
				gimbal.info->target_yaw_imu_angle = -30.f;
			}
			else
			{
				gimbal.info->target_yaw_imu_angle = 30.f;
			}
		}
	}
	else
	{
		cnt = 0;
	}
}


/**
  * @brief  云台数据读取
  * @param  
  * @retval 
  */
void Gimbal_GetInfo(void)
{
	Gimbal_GetBaseInfo();
	
	PID_ParamsInit();
	
	if (flag.gimbal_flag.reset_ok == 0)
	{
		if (RC_IsChannelReset() && flag.gimbal_flag.reset_start == 0)
		{
			flag.gimbal_flag.reset_ok = 1;
		}
	}
	else
	{
		if(gim_info.remote_mode == RC) {
			Gimbal_GetRcInfo();
		}
		else if(gim_info.remote_mode == KEY) {
			Gimbal_GetKeyInfo();
		}
	}
}

void Gimbal_GetBaseInfo(void)
{
	gimbal.info->measure_yaw_motor_angle = RM_motor[GIM_Y].rx_info.angle;
	gimbal.info->measure_yaw_motor_speed = RM_motor[GIM_Y].rx_info.speed;
	gimbal.info->measure_pitch_motor_angle = RM_motor[GIM_P].rx_info.angle;
	gimbal.info->measure_pitch_motor_speed = RM_motor[GIM_P].rx_info.speed;
	
	gimbal.info->measure_yaw_imu_speed = imu_sensor.info->base_info.ave_rate_yaw;
	gimbal.info->measure_yaw_imu_angle = imu_sensor.info->base_info.yaw;
	gimbal.info->measure_pitch_imu_speed = imu_sensor.info->base_info.ave_rate_pitch;
	gimbal.info->measure_pitch_imu_angle = imu_sensor.info->base_info.pitch;
	gimbal.info->measure_roll_imu_speed = imu_sensor.info->base_info.ave_rate_roll;
	gimbal.info->measure_roll_imu_angle = imu_sensor.info->base_info.roll;
	
}

void PID_ParamsInit(void)
{
	if (gimbal.info->gimbal_mode == gim_machine)
	{
		RM_motor[GIM_P].pid_init(&RM_motor[GIM_P].pid.position_in, pitch_mach_position_in);
		RM_motor[GIM_P].pid_init(&RM_motor[GIM_P].pid.position, pitch_mach_position);
		RM_motor[GIM_Y].pid_init(&RM_motor[GIM_Y].pid.position_in, yaw_mach_position_in);
		RM_motor[GIM_Y].pid_init(&RM_motor[GIM_Y].pid.position, yaw_mach_position);
	}
	else if (gimbal.info->gimbal_mode == gim_gyro)
	{
		RM_motor[GIM_P].pid_init(&RM_motor[GIM_P].pid.position_in, pitch_gyro_position_in);
		RM_motor[GIM_P].pid_init(&RM_motor[GIM_P].pid.position, pitch_gyro_position);
		RM_motor[GIM_Y].pid_init(&RM_motor[GIM_Y].pid.position_in, yaw_gyro_position_in);
		RM_motor[GIM_Y].pid_init(&RM_motor[GIM_Y].pid.position, yaw_gyro_position);
	}
}

void Gimbal_GetRcInfo(void)
{
	if (gimbal.info->gimbal_mode == gim_machine)
	{
		gimbal.info->target_pitch_motor_deltaangle = rc_sensor.info->ch1 / gim_conf.rc_pitch_motor_offset;
		gimbal.info->target_pitch_imu_angle = gimbal.info->measure_pitch_imu_angle;
		gimbal.info->target_yaw_imu_angle = gimbal.info->measure_yaw_imu_angle;
	}
	else if (gimbal.info->gimbal_mode == gim_gyro)
	{
		gimbal.info->target_pitch_motor_angle = gimbal.info->measure_pitch_motor_angle;
		gimbal.info->target_pitch_imu_deltaangle = (float)rc_sensor.info->ch1 / gim_conf.rc_pitch_imu_offset;
		gimbal.info->target_yaw_imu_deltaangle = -(float)rc_sensor.info->ch0 / gim_conf.rc_yaw_imu_offset;
		
	}
	else if (gimbal.info->gimbal_mode == gim_vision)
	{
//		gimbal.info->target_pitch_motor_angle = gimbal.info->measure_pitch_motor_angle;
//		gimbal.info->target_pitch_imu_angle = vision_sensor.info->rx_info->pitch_angle * ECD_TO_ANGLE - 180.0f;
//		gimbal.info->target_yaw_imu_angle = vision_sensor.info->rx_info->yaw_angle * ECD_TO_ANGLE;
	}
	else
	{
		gimbal.info->target_pitch_imu_angle = gimbal.info->measure_pitch_imu_angle;
		gimbal.info->target_yaw_imu_angle = gimbal.info->measure_yaw_imu_angle;
	}
}

void Gimbal_GetKeyInfo(void)
{
	Gimbal_GetRcInfo();
}



/**
  * @brief  云台复位
  * @param  
  * @retval 
  */
void Gimbal_Reset(void)
{
	if (flag.gimbal_flag.reset_start == 1 && flag.gimbal_flag.reset_ok == 0)
	{
		if (gimbal.info->gimbal_mode == gim_machine)
		{
			Gimbal_MotoReset();
		}
		else if (gimbal.info->gimbal_mode == gim_gyro)
		{
			Gimbal_GyroReset();
		}
	}
}

void Gimbal_MotoReset(void)
{
	gimbal.info->target_pitch_motor_angle = gim_conf.restart_pitch_motor_angle;
	gimbal.info->target_yaw_motor_angle = gim_conf.restart_yaw_motor_angle;
	
	flag.gimbal_flag.reset_start = 0;
}

void Gimbal_GyroReset(void)
{
	gimbal.info->target_pitch_imu_angle = gim_conf.restart_pitch_imu_angle;
	gimbal.info->target_yaw_imu_angle = gimbal.info->measure_yaw_imu_angle;
	
	flag.gimbal_flag.reset_start = 0;
}



/**
  * @brief  云台电机控制
  * @param  
  * @retval 
  */
void Gimbal_MotorCtrl(void)
{
	// 机械模式
	if (gimbal.info->gimbal_mode == gim_machine)
	{
			gimbal.info->target_pitch_motor_angle += gimbal.info->target_pitch_motor_deltaangle;
			
			// pitch轴电机限位
			if (gimbal.info->target_pitch_motor_angle > gim_conf.max_pitch_motor_angle)
			{
				gimbal.info->target_pitch_motor_angle = gim_conf.max_pitch_motor_angle;
			}
			else if (gimbal.info->target_pitch_motor_angle < gim_conf.min_pitch_motor_angle)
			{
				gimbal.info->target_pitch_motor_angle = gim_conf.min_pitch_motor_angle;
			}
		
			
			// yaw轴电机error过零点处理
		  gimbal.info->yaw_motor_angle_err = gim_conf.restart_yaw_motor_angle - gimbal.info->measure_yaw_motor_angle;
			if (gimbal.info->yaw_motor_angle_err > HALF_ECD_RANGE)
			{
				gimbal.info->yaw_motor_angle_err -= ECD_RANGE;
			}
			else if (gimbal.info->yaw_motor_angle_err < -HALF_ECD_RANGE)
			{
				gimbal.info->yaw_motor_angle_err += ECD_RANGE;
			}
			// yaw轴电机就近归中
			if (abs(gimbal.info->yaw_motor_angle_err) > (HALF_ECD_RANGE / 2))
			{
				if (gimbal.conf->restart_yaw_motor_angle > HALF_ECD_RANGE)
				{
					gimbal.conf->restart_yaw_motor_angle = gimbal.conf->MID_VALUE - HALF_ECD_RANGE;
					flag.chassis_flag.forward = 0;
				}
				else
				{
					gimbal.conf->restart_yaw_motor_angle = gimbal.conf->MID_VALUE;
					flag.chassis_flag.forward = 1;
				}
			}
			gim_info.target_yaw_motor_angle = gim_conf.restart_yaw_motor_angle;
			
	}
	// 陀螺仪模式
	else if (gimbal.info->gimbal_mode == gim_gyro)
	{
			gimbal.info->target_pitch_imu_angle += gimbal.info->target_pitch_imu_deltaangle;
			gimbal.info->target_yaw_imu_angle += gimbal.info->target_yaw_imu_deltaangle;
			
			
			// pitch轴电机限位（陀螺仪模式下需要修改）
			if (gimbal.info->target_pitch_imu_angle > gim_conf.max_pitch_imu_angle)
			{
				gimbal.info->target_pitch_imu_angle = gim_conf.max_pitch_imu_angle;
			}
			else if (gimbal.info->target_pitch_imu_angle < gim_conf.min_pitch_imu_angle)
			{
				gimbal.info->target_pitch_imu_angle = gim_conf.min_pitch_imu_angle;
			}
			
			
			// yaw轴电机target过零点处理
			if (gimbal.info->target_yaw_imu_angle > 180.0f)
			{
				gimbal.info->target_yaw_imu_angle -= 360.0f;
			}
			else if (gimbal.info->target_yaw_imu_angle < -180.0f)
			{
				gimbal.info->target_yaw_imu_angle += 360.0f;
			}
	}
	else if (gimbal.info->gimbal_mode == gim_vision)
	{
		
			// pitch轴电机限位（陀螺仪模式下需要修改）
			if (gimbal.info->target_pitch_imu_angle > gim_conf.max_pitch_imu_angle)
			{
				gimbal.info->target_pitch_imu_angle = gim_conf.max_pitch_imu_angle;
			}
			else if (gimbal.info->target_pitch_imu_angle < gim_conf.min_pitch_imu_angle)
			{
				gimbal.info->target_pitch_imu_angle = gim_conf.min_pitch_imu_angle;
			}
			
			
			// yaw轴电机target过零点处理
			if (gimbal.info->target_yaw_imu_angle > 180.0f)
			{
				gimbal.info->target_yaw_imu_angle -= 360.0f;
			}
			else if (gimbal.info->target_yaw_imu_angle < -180.0f)
			{
				gimbal.info->target_yaw_imu_angle += 360.0f;
			}
	}
	else
	{
			Gimbal_Stop();
	}
	
	Gimbal_Yaw_Angle_PidCalc();
	Gimbal_Pitch_Angle_PidCalc();
}




void Gimbal_Yaw_Angle_PidCalc(void)
{
	if (gimbal.info->gimbal_mode == gim_machine)
	{
		// yaw轴电机error过零点处理
		if (gimbal.info->target_yaw_motor_angle - gimbal.info->measure_yaw_motor_angle > HALF_ECD_RANGE)
		{
			gimbal.info->measure_yaw_motor_angle += ECD_RANGE;
		}
		else if (gimbal.info->target_yaw_motor_angle - gimbal.info->measure_yaw_motor_angle < -HALF_ECD_RANGE)
		{
			gimbal.info->measure_yaw_motor_angle -= ECD_RANGE;
		}
		
		RM_motor[GIM_Y].base_info.motor_out = RM_motor[GIM_Y].ctr_pid2(&RM_motor[GIM_Y].pid.position, 
																																	&RM_motor[GIM_Y].pid.position_in,
																																	(float)gimbal.info->measure_yaw_motor_angle,
																																	gimbal.info->measure_yaw_imu_speed,
																																	(float)gimbal.info->target_yaw_motor_angle,
																																	0);
		gim_out[RM_motor[GIM_Y].id.buff_p] = RM_motor[GIM_Y].base_info.motor_out;
	}
	else if (gimbal.info->gimbal_mode == gim_gyro)
	{
		// yaw轴电机error过零点处理
		if (gimbal.info->target_yaw_imu_angle - gimbal.info->measure_yaw_imu_angle > 180.0f)
		{
			gimbal.info->measure_yaw_imu_angle += 360.0f;
		}
		else if (gimbal.info->target_yaw_imu_angle - gimbal.info->measure_yaw_imu_angle < -180.0f)
		{
			gimbal.info->measure_yaw_imu_angle -= 360.0f;
		}
		
		RM_motor[GIM_Y].base_info.motor_out = RM_motor[GIM_Y].ctr_pid2(&RM_motor[GIM_Y].pid.position, 
																																	&RM_motor[GIM_Y].pid.position_in,
																																	gimbal.info->measure_yaw_imu_angle,
																																	gimbal.info->measure_yaw_imu_speed,
																																	gimbal.info->target_yaw_imu_angle,
																																	0);
		gim_out[RM_motor[GIM_Y].id.buff_p] = RM_motor[GIM_Y].base_info.motor_out;
	}
}

void Gimbal_Pitch_Angle_PidCalc(void)
{
	if (gimbal.info->gimbal_mode == gim_machine)
	{
		RM_motor[GIM_P].base_info.motor_out = RM_motor[GIM_P].ctr_pid2(&RM_motor[GIM_P].pid.position, 
																																	&RM_motor[GIM_P].pid.position_in,
																																	(float)gimbal.info->measure_pitch_motor_angle,
																																	gimbal.info->measure_pitch_imu_speed,
																																	(float)gimbal.info->target_pitch_motor_angle,
																																	0);
		gim_out[RM_motor[GIM_P].id.buff_p] = RM_motor[GIM_P].base_info.motor_out;
	}
	else if (gimbal.info->gimbal_mode == gim_gyro)
	{
		// pitch轴增量与电机增量方向不同
		RM_motor[GIM_P].base_info.motor_out = -RM_motor[GIM_P].ctr_pid2(&RM_motor[GIM_P].pid.position, 
																																	&RM_motor[GIM_P].pid.position_in,
																																	gimbal.info->measure_pitch_imu_angle,
																																	gimbal.info->measure_pitch_imu_speed,
																																	gimbal.info->target_pitch_imu_angle,
																																	0);
		gim_out[RM_motor[GIM_P].id.buff_p] = RM_motor[GIM_P].base_info.motor_out\
																				- 47 * gimbal.info->target_pitch_imu_angle - 6200;//前馈
	}
	
}



/**
  * @brief  云台控制输出
  * @param  
  * @retval 
  */
void Gimbal_SendOut(void)
{
	// yaw轴电机离线保护
	if (RM_motor[GIM_Y].state.work_state == M_ONLINE)
	{
		can2_send_buf[RM_motor[GIM_Y].id.buff_p] = gim_out[RM_motor[GIM_Y].id.buff_p];
	}
	else
	{
		can2_send_buf[RM_motor[GIM_Y].id.buff_p] = 0;
	}
	
	// pitch轴电机离线保护
	if (RM_motor[GIM_P].state.work_state == M_ONLINE)
	{
		can2_send_buf[RM_motor[GIM_P].id.buff_p] = gim_out[RM_motor[GIM_P].id.buff_p];
	}
	else
	{
		can2_send_buf[RM_motor[GIM_P].id.buff_p] = 0;
	}
}



void Gimbal_Stop()
{
	RM_motor[GIM_P].pid.position_in.info.out = 0.0f;
	RM_motor[GIM_Y].pid.position_in.info.out = 0.0f;
	gim_out[0] = (int16_t)RM_motor[GIM_P].pid.position_in.info.out;
	gim_out[1] = (int16_t)RM_motor[GIM_Y].pid.position_in.info.out;
	
	can2_send_buf[0] = gim_out[0];
	can2_send_buf[1] = gim_out[1];
}


