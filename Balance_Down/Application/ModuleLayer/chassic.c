#include "chassic.h"
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void chassic_ctrl(void);   //总控制
	
void chassic_status_var_update(void);
void chassic_LQR_calc(void);
void chassic_LQR_out(void);
void chassic_LQR_Ctrl(void);
void BalanceModeON(void);	
void BalanceModeOFF(void);
void chassic_get_info(void);
void chassic_stop(void);
void chassic_examine_status(void);
void chassic_selfprotect(void);   
void momentum_current_out(float L_tar, float R_tar);
void momentum_ctrl(void);
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

extern imu_info_t imu_info;
extern KT_motor_t KT_motor[KT_MOTOR_LIST];
extern RM_motor_t RM_motor[MOM_MOTOR_LIST];


int16_t momentum_current_txbuf[4];

chassis_info_t chassic_info =  //---------------待修改
{
	//指针赋值
	.chassic_motor  = KT_motor,
	.momentum_motor = RM_motor,
	
	//整车机械角度----------------------------待修改
	.status_var.pitch_tar = MEC_ANGLE,
	.status_var.X_speed_max_tar = X_MAX_SPEED,
	.status_var.rate_yaw_max_tar = Z_MAX_SPEED,
	
	
	//LQR参数-------------------------------------------待修改
	.LQR_param.k1  = 0,
	.LQR_param.k2  = 0,
	.LQR_param.k3  = 0,
	.LQR_param.k4  = 0,
	.LQR_param.k5  = 0,
	.LQR_param.k6  = 0,
	
	
	
	//三个扭矩分量的限幅--------------------------------------------待修改
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
	
	.ctrl							 = chassic_ctrl,
	
	.get_info          = chassic_get_info,
	.stop              = chassic_stop,
	.selfprotect       = chassic_selfprotect,
	.exam_status       = chassic_examine_status,
	.update_status_var = chassic_status_var_update,
	.LQR_ctrl          = chassic_LQR_Ctrl,
	.momentum_out      = momentum_current_out,
};
	




/**
 *	@brief	底盘总的控制，内部已经有对电机状态的判断

 */
void chassic_ctrl(void)
{
	//检查电机是否初始化
	if(LEG_L_DEINIT || LEG_R_DEINIT || MOMENTUM_L_DEINIT || MOMENTUM_R_DEINIT)
		return;
	
	
	//不管是否失联，都要调用该函数
	chassic.get_info();
	
	
	//检查电机是否在线
	if(LEG_L_ONLINE && LEG_R_ONLINE && MOMENTUM_L_ONLINE && MOMENTUM_R_ONLINE)
	{
		
		chassic.exam_status();
		
		//检查电机是否要自保
		if(LEG_L_PROTECT_OFF && LEG_R_PROTECT_OFF)
		{
			
		//	momentum_ctrl();
			
			BalanceModeON();	//必须放在最前面
			
			chassic.update_status_var();
			
			chassic.LQR_ctrl();
			
		}
		else
		{
			chassic.selfprotect();	
		}		
		
	}
		
	else
		chassic.selfprotect();	
	
}


/**
 *	@brief	底盘更新状态变量，内部做好了对目标值的限幅

		问题：θ角度是不是pitch？？目标值的遥控器映射，待修改
					这里的误差计算是测量-误差？？
 */
void chassic_status_var_update(void)
{
	
	KT_motor_rx_info_t  *LEG_L_info  =  &KT_motor[LEG_L].KT_motor_info.rx_info;
	KT_motor_rx_info_t  *LEG_R_info  =  &KT_motor[LEG_R].KT_motor_info.rx_info;
	status_variables_t  *status      =  &chassic_info.status_var;
	
//测量值更新	
	status->along_X_distance_meas  = 0; //不反馈，作为0
	status->along_X_speed_meas     = (LEG_L_info->speed + LEG_R_info->speed) / 2 * //整车速度  m/s
																		ANGLE_CONVERSION_RADIAN	* WHEEL_RADIUS ;  
	
	status->pitch_meas             = imu_info.base_info.pitch;
	status->rate_pitch_meas        = imu_info.base_info.ave_rate_pitch;
	
	status->yaw_meas               = imu_info.base_info.yaw;
	status->rate_yaw_meas          = imu_info.base_info.ave_rate_yaw;
	
	
//目标值更新
	status->X_speed_tar  = 0;
	status->X_speed_tar = constrain(status->X_speed_tar, -status->X_speed_max_tar, status->X_speed_max_tar);
	
	status->rate_yaw_tar = 0;
	status->rate_yaw_tar = constrain(status->rate_yaw_tar, -status->rate_yaw_max_tar, status->rate_yaw_max_tar);
	

//误差值更新
	status->X_speed_err   = status->along_X_speed_meas - status->X_speed_tar;
	
	status->rate_yaw_err  = status->rate_yaw_meas - status->rate_yaw_tar;

}

/**
 *	@brief	底盘扭矩计算
						注意：matlab仿真时，规定系统两个输入，根据这两个输入的表达式可列出左右两轮的力
									进而得到力矩，再去得到扭矩电流

	问题：各部分扭矩限幅值哪来的
				左边和右边的电流正负？，待修改
				单电机的峰值扭矩做了修改，为了测试而已
				单电机的发送电流值做了修改，方便测试
				
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
			Torqdata->Speed_Torq = ( LQRdata->k1 * status->along_X_distance_meas +
														   LQRdata->k2 * status->X_speed_err ) * WHEEL_RADIUS;

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
			Torqdata->Balance_Torq = ( LQRdata->k3 * status->pitch_meas +
																 LQRdata->k4 * status->rate_pitch_meas ) * WHEEL_RADIUS;
	
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
			Torqdata->Spin_Torq = ( LQRdata->k5 * status->yaw_meas +
															LQRdata->k6 * status->rate_yaw_err ) * WHEEL_RADIUS;
												
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
	
	KT_motor[LEG_L].tx_W_cmd( &KT_motor[LEG_L], TORQUE_CLOSE_LOOP_ID);
		
	KT_motor[LEG_R].tx_W_cmd( &KT_motor[LEG_R], TORQUE_CLOSE_LOOP_ID);


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
 *	@brief	平衡三环打开
 *  @brief  
 */
void BalanceModeON(void)
{
	balance_mode_t *mode = &chassic_info.balance_mode;
	
	mode->speed = ON;
	mode->stand = ON;
	mode->turn  = ON;
}
/**
 *	@brief	平衡三环关闭
 *  @brief  
 */
void BalanceModeOFF(void)
{
	balance_mode_t *mode = &chassic_info.balance_mode;
	
	mode->speed = OFF;
	mode->stand = OFF;
	mode->turn  = OFF;
}
/**
 *	@brief	KT电机读取有用的数据，RM电机不需要发送命令
 */
void chassic_get_info(void)
{
	
	KT_motor[LEG_L].tx_R_cmd( &KT_motor[LEG_L], STATE1_ID);
	
	KT_motor[LEG_R].tx_R_cmd( &KT_motor[LEG_R], STATE1_ID);

}

/**
 *	@brief	KT电机卸力，无需手动清空发送数组，只需把发送结构体的电流置零,用的是扭矩控制的卸力方式
						RM电机卸力
						
 */
void chassic_stop(void)
{
	
	KT_motor[LEG_L].KT_motor_info.tx_info.iqControl = 0;
	KT_motor[LEG_L].tx_W_cmd( &KT_motor[LEG_L], TORQUE_CLOSE_LOOP_ID);
	
	KT_motor[LEG_R].KT_motor_info.tx_info.iqControl = 0;
	KT_motor[LEG_R].tx_W_cmd( &KT_motor[LEG_R], TORQUE_CLOSE_LOOP_ID);
	
	memset(momentum_current_txbuf, 0, 4);
	CAN2_Send_With_int16_to_uint8( RM_motor[MOMENTUM_L].id.tx_id, momentum_current_txbuf);
	
}
	
/**
 *	@brief	检测底盘的各个参数是否正常，如果异常，就认定为要自我保护，但是只是设置保护这个标志位
						检测底盘电机的返回的速度（X轴和yaw平面）、电流、电压、温度
*/
void chassic_examine_status(void)
{
	status_variables_t     *status       =  &chassic_info.status_var;
	KT_motor_rx_info_t     *LEG_L_info   =  &KT_motor[LEG_L].KT_motor_info.rx_info;
	KT_motor_state_info_t  *LEG_L_state  =  &KT_motor[LEG_L].KT_motor_info.state_info; 
	KT_motor_rx_info_t     *LEG_R_info   =  &KT_motor[LEG_R].KT_motor_info.rx_info;
	KT_motor_state_info_t  *LEG_R_state  =  &KT_motor[LEG_R].KT_motor_info.state_info; 
	

	if( abs(status->along_X_speed_meas) < X_MAX_SPEED 				 && 
		  abs(status->rate_pitch_meas)    < Z_MAX_SPEED 				 &&
	
			abs(LEG_L_info->current)        < KT_TX_IQ_CONTROL_MAX &&
	    abs(LEG_R_info->current)        < KT_TX_IQ_CONTROL_MAX && 
	
			LEG_L_info->errorState          == 0x00                &&
	    LEG_R_info->errorState          == 0x00    ) 
		LEG_L_state->selfprotect_flag = LEG_R_state->selfprotect_flag = M_PROTECT_OFF;
	
	else
		LEG_L_state->selfprotect_flag = LEG_R_state->selfprotect_flag = M_PROTECT_ON;
}

/**
 *	@brief	底盘自我保护----针对电机温度或者电压异常情况或者遥控器失联或者电机失联
						卸力  +  停止LQR计算
*/
void chassic_selfprotect(void)
{

	BalanceModeOFF();
	
	chassic_stop();
}	
/**
 *	@brief	动量部分电机的电流发送
*/
void momentum_current_out(float L_tar, float R_tar)
{
	uint8_t L_buff_p = RM_motor[MOMENTUM_L].id.buff_p;
	uint8_t R_buff_p = RM_motor[MOMENTUM_R].id.buff_p;
	
	momentum_current_txbuf[L_buff_p] = RM_motor[MOMENTUM_L].ctr_angle(&RM_motor[MOMENTUM_L], L_tar);
	
	momentum_current_txbuf[R_buff_p] = RM_motor[MOMENTUM_R].ctr_angle(&RM_motor[MOMENTUM_R], R_tar);
	
	CAN2_Send_With_int16_to_uint8( RM_motor[MOMENTUM_L].id.tx_id, momentum_current_txbuf);
}

/**
 *	@brief	动量电机控制，目前先配合KT电机达到自站效果，无其他控制，--------------还需修改
*/
void momentum_ctrl(void)
{
	float L_tar = 0, R_tar = 0;
	
//	//判断倒向哪边
//	if( imu_info.base_info.pitch > 30 )
//		L_tar = R_tar = MOMENTUM_F_POS;
//	
//	else if( imu_info.base_info.pitch < -30 )
//		L_tar = R_tar = MOMENTUM_B_POS;
//	
//	else if( abs(imu_info.base_info.pitch) < 5 )	 
//		L_tar = R_tar = MOMENTUM_M_POS;
//	
//	chassic.momentum_out(L_tar, R_tar);
//	
}

