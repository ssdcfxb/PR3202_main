#include "chassic.h"
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void chassic_status_var_update(void);
void chassic_LQR_calc(void);
void chassic_LQR_out(void);
void chassic_LQR_Ctrl(void);
void chassic_get_info(void);
void chassic_stop(void);
void chassic_selfprotect(void);
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

extern imu_info_t imu_info;
extern KT_motor_t KT_motor[MOTOR_LIST];

chassis_info_t chassic_info = 
{
	//指针赋值
	.chassic_motor = KT_motor,
	
	
	//LQR参数
	.LQR_param.k1  = 0,
	.LQR_param.k2  = 0,
	.LQR_param.k3  = 0,
	.LQR_param.k4  = 0,
	.LQR_param.k5  = 0,
	.LQR_param.k6  = 0,
	
	
	
	//三个扭矩分量的限幅------------------试出来的
	.torq_data.Max_Speed_Torq 	= 2.8f,
	.torq_data.Max_Balance_Torq = 20.0f,
	.torq_data.Max_Spin_Torq    = 2.0f,
	
	
	
	//是否开启LQR的计算
	.balance_mode.speed = ON,
	.balance_mode.stand = ON,
	.balance_mode.turn = ON,
	
};

chassic_class_t chassic = 
{
  .info = &chassic_info,
	
	.get_info          = chassic_get_info,
	.stop              = chassic_stop,
	.selfprotect       = chassic_selfprotect,
	.update_status_var = chassic_status_var_update,
	.LQR_ctrl          = chassic_LQR_Ctrl,
};
	




/**
 *	@brief	底盘更新状态变量

		问题：θ角度是不是pitch？？
 */
void chassic_status_var_update(void)
{
	
	KT_motor_rx_info_t  *LEG_L_info  =  &KT_motor[LEG_L].KT_motor_info.rx_info;
	KT_motor_rx_info_t  *LEG_R_info  =  &KT_motor[LEG_R].KT_motor_info.rx_info;
	status_variables_t  *status      =  &chassic_info.status_var;
	
	status->along_X_distance  = 0; //不反馈，作为0
	status->along_X_speed     = (LEG_L_info->speed + LEG_R_info->speed) / 2 * //整车速度  m/s
															 ANGLE_CONVERSION_RADIAN	* WHEEL_RADIUS ;  
	
	status->pitch             = imu_info.base_info.pitch;
	status->rate_pitch        = imu_info.base_info.rate_pitch;
	
	status->yaw               = imu_info.base_info.yaw;
	status->rate_yaw          = imu_info.base_info.rate_yaw;
	
}

/**
 *	@brief	底盘扭矩计算
						注意：matlab仿真时，规定系统两个输入，根据这两个输入的表达式可列出左右两轮的力
									进而得到力矩，再去得到扭矩电流

	问题：各部分扭矩限幅值哪来的
				左边和右边的电流正负？

 */
void chassic_LQR_calc(void)
{
	status_variables_t  *status       = &chassic_info.status_var;
	LQRData_t           *LQRdata      = &chassic_info.LQR_param;
	TorqData_t          *Torqdata  	  = &chassic_info.torq_data;
	balance_mode_t      *balance_mode = &chassic_info.balance_mode;
	
	//速度扭矩
	switch(balance_mode->speed)
	{
		case ON:
			Torqdata->Speed_Torq = ( LQRdata->k1 * status->along_X_distance +
														   LQRdata->k2 * status->along_X_speed ) * WHEEL_RADIUS;

			Torqdata->Speed_Torq = 0.2f * Torqdata->Speed_Torq + 0.8f * Torqdata->Last_Speed_Torq;

			Torqdata->Speed_Torq  = constrain(Torqdata->Speed_Torq, -Torqdata->Max_Speed_Torq, Torqdata->Max_Speed_Torq);
			
			Torqdata->Last_Speed_Torq = Torqdata->Speed_Torq;
		break;
		
		case OFF:
			Torqdata->Speed_Torq = Torqdata->Last_Speed_Torq = 0;
		break;	
	}
	
	
	
	//平衡扭矩
	switch(balance_mode->stand)
	{
		case ON:
			Torqdata->Balance_Torq = ( LQRdata->k3 * status->pitch +
																 LQRdata->k4 * status->rate_pitch ) * WHEEL_RADIUS;
	
			Torqdata->Balance_Torq  = constrain(Torqdata->Speed_Torq, -Torqdata->Max_Balance_Torq, Torqdata->Max_Balance_Torq);
		break;
		
		case OFF:
			Torqdata->Balance_Torq = 0;
		break;
		
	}
	
	
	//转向扭矩
	switch(balance_mode->turn)
	{
		case ON:
			Torqdata->Spin_Torq = ( LQRdata->k5 * status->yaw +
												  LQRdata->k6 * status->rate_yaw ) * WHEEL_RADIUS;
												
			Torqdata->Spin_Torq = 0.2f * Torqdata->Spin_Torq + 0.8f * Torqdata->Last_Spin_Torq;
	
			Torqdata->Spin_Torq  = constrain(Torqdata->Speed_Torq, -Torqdata->Max_Spin_Torq, Torqdata->Max_Spin_Torq);
	
			Torqdata->Last_Spin_Torq = Torqdata->Spin_Torq;	
		break;
		
		case OFF:
			Torqdata->Spin_Torq = Torqdata->Last_Spin_Torq = 0;
		break;
	}

	


	//计算两个轮子各自扭矩和
	Torqdata->LEG_L_Sum_Torq = ( Torqdata->Speed_Torq +
															 Torqdata->Balance_Torq -
															 Torqdata->Spin_Torq ) * 0.5f;
							
	Torqdata->LEG_L_Sum_Torq = constrain(Torqdata->LEG_L_Sum_Torq, -MAX_TORQUE, MAX_TORQUE);

	Torqdata->LEG_R_Sum_Torq = ( Torqdata->Speed_Torq +
															 Torqdata->Balance_Torq +
															 Torqdata->Spin_Torq ) * 0.5f;		

	Torqdata->LEG_R_Sum_Torq = constrain(Torqdata->LEG_R_Sum_Torq, -MAX_TORQUE, MAX_TORQUE);
	
	
	
	
	
	//转换成扭矩电流，写到电机发送结构体的扭矩电流中，内部有限幅
	Torqdata->LEG_L_Current = -K_CURRENT_TURN * (Torqdata->LEG_L_Sum_Torq / TORQUE_CONSTANT);
	
	KT_motor[LEG_L].W_iqControl(&KT_motor[LEG_L], (int16_t)Torqdata->LEG_L_Current);
	
	Torqdata->LEG_R_Current =  K_CURRENT_TURN * (Torqdata->LEG_R_Sum_Torq / TORQUE_CONSTANT);
	
	KT_motor[LEG_R].W_iqControl(&KT_motor[LEG_R], (int16_t)Torqdata->LEG_R_Current);
	
}

/**
 *	@brief	底盘CAN发送
 */

void chassic_LQR_out(void)
{
	
	if(KT_motor[LEG_L].KT_motor_info.state_info.work_state == M_ONLINE && 
		 KT_motor[LEG_R].KT_motor_info.state_info.work_state == M_ONLINE)
	{
		KT_motor[LEG_L].tx_W_cmd( &KT_motor[LEG_L], TORQUE_CLOSE_LOOP_ID);
		
		KT_motor[LEG_R].tx_W_cmd( &KT_motor[LEG_R], TORQUE_CLOSE_LOOP_ID);
	}
	
	//失联，卸力
	else		
		chassic_stop();
	
}
/**
 *	@brief	底盘LQR总控制
 */

void chassic_LQR_Ctrl(void)
{
	chassic_LQR_calc();
	
	chassic_LQR_out();
}

/**
 *	@brief	KT电机读取有用的数据
 */
void chassic_get_info(void)
{
	
	KT_motor[LEG_L].tx_R_cmd( &KT_motor[LEG_L], STATE1_ID);
	KT_motor[LEG_L].tx_R_cmd( &KT_motor[LEG_L], STATE2_ID);
	
	KT_motor[LEG_R].tx_R_cmd( &KT_motor[LEG_R], STATE1_ID);
	KT_motor[LEG_R].tx_R_cmd( &KT_motor[LEG_R], STATE2_ID);
}

/**
 *	@brief	KT电机卸力，无需手动情况发送数组，只需把发送结构体的电流置零
						用的是扭矩控制的卸力方式
 */
void chassic_stop(void)
{
	
	KT_motor[LEG_L].KT_motor_info.tx_info.iqControl = 0;
	KT_motor[LEG_L].tx_W_cmd( &KT_motor[LEG_L], TORQUE_CLOSE_LOOP_ID);
	
	KT_motor[LEG_R].KT_motor_info.tx_info.iqControl = 0;
	KT_motor[LEG_R].tx_W_cmd( &KT_motor[LEG_R], TORQUE_CLOSE_LOOP_ID);
}
	

/**
 *	@brief	底盘自我保护----针对电机温度或者电压异常情况
						卸力  +  停止LQR计算
*/
void chassic_selfprotect(void)
{
	
	if(KT_motor[LEG_L].KT_motor_info.state_info.selfprotect_flag == True || 
		 KT_motor[LEG_R].KT_motor_info.state_info.selfprotect_flag == True  )
	{
		
	}	
	
}	
