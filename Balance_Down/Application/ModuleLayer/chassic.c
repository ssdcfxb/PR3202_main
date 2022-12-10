#include "chassic.h"
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void chassic_ctrl(void);   //�ܿ���
	
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

chassis_info_t chassic_info =  //---------------���޸�
{
	//ָ�븳ֵ
	.chassic_motor  = KT_motor,
	.momentum_motor = RM_motor,
	
	//������е�Ƕ�----------------------------���޸�
	.status_var.pitch_tar = MEC_ANGLE,
	.status_var.X_speed_max_tar = X_MAX_SPEED,
	.status_var.rate_yaw_max_tar = Z_MAX_SPEED,
	
	
	//LQR����-------------------------------------------���޸�
	.LQR_param.k1  = 0,
	.LQR_param.k2  = 0,
	.LQR_param.k3  = 0,
	.LQR_param.k4  = 0,
	.LQR_param.k5  = 0,
	.LQR_param.k6  = 0,
	
	
	
	//����Ť�ط������޷�--------------------------------------------���޸�
	.torq_data.Max_Speed_Torq 	= 2.8f,
	.torq_data.Max_Balance_Torq = 20.0f,
	.torq_data.Max_Spin_Torq    = 2.0f,
	
	
	
	//�Ƿ���LQR�ļ���
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
 *	@brief	�����ܵĿ��ƣ��ڲ��Ѿ��жԵ��״̬���ж�

 */
void chassic_ctrl(void)
{
	//������Ƿ��ʼ��
	if(LEG_L_DEINIT || LEG_R_DEINIT || MOMENTUM_L_DEINIT || MOMENTUM_R_DEINIT)
		return;
	
	
	//�����Ƿ�ʧ������Ҫ���øú���
	chassic.get_info();
	
	
	//������Ƿ�����
	if(LEG_L_ONLINE && LEG_R_ONLINE && MOMENTUM_L_ONLINE && MOMENTUM_R_ONLINE)
	{
		
		chassic.exam_status();
		
		//������Ƿ�Ҫ�Ա�
		if(LEG_L_PROTECT_OFF && LEG_R_PROTECT_OFF)
		{
			
		//	momentum_ctrl();
			
			BalanceModeON();	//���������ǰ��
			
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
 *	@brief	���̸���״̬�������ڲ������˶�Ŀ��ֵ���޷�

		���⣺�ȽǶ��ǲ���pitch����Ŀ��ֵ��ң����ӳ�䣬���޸�
					������������ǲ���-����
 */
void chassic_status_var_update(void)
{
	
	KT_motor_rx_info_t  *LEG_L_info  =  &KT_motor[LEG_L].KT_motor_info.rx_info;
	KT_motor_rx_info_t  *LEG_R_info  =  &KT_motor[LEG_R].KT_motor_info.rx_info;
	status_variables_t  *status      =  &chassic_info.status_var;
	
//����ֵ����	
	status->along_X_distance_meas  = 0; //����������Ϊ0
	status->along_X_speed_meas     = (LEG_L_info->speed + LEG_R_info->speed) / 2 * //�����ٶ�  m/s
																		ANGLE_CONVERSION_RADIAN	* WHEEL_RADIUS ;  
	
	status->pitch_meas             = imu_info.base_info.pitch;
	status->rate_pitch_meas        = imu_info.base_info.ave_rate_pitch;
	
	status->yaw_meas               = imu_info.base_info.yaw;
	status->rate_yaw_meas          = imu_info.base_info.ave_rate_yaw;
	
	
//Ŀ��ֵ����
	status->X_speed_tar  = 0;
	status->X_speed_tar = constrain(status->X_speed_tar, -status->X_speed_max_tar, status->X_speed_max_tar);
	
	status->rate_yaw_tar = 0;
	status->rate_yaw_tar = constrain(status->rate_yaw_tar, -status->rate_yaw_max_tar, status->rate_yaw_max_tar);
	

//���ֵ����
	status->X_speed_err   = status->along_X_speed_meas - status->X_speed_tar;
	
	status->rate_yaw_err  = status->rate_yaw_meas - status->rate_yaw_tar;

}

/**
 *	@brief	����Ť�ؼ���
						ע�⣺matlab����ʱ���涨ϵͳ�������룬��������������ı��ʽ���г��������ֵ���
									�����õ����أ���ȥ�õ�Ť�ص���

	���⣺������Ť���޷�ֵ������
				��ߺ��ұߵĵ��������������޸�
				������ķ�ֵŤ�������޸ģ�Ϊ�˲��Զ���
				������ķ��͵���ֵ�����޸ģ��������
				
 */
void chassic_LQR_calc(void)
{
	status_variables_t  *status       = &chassic_info.status_var;
	LQRData_t           *LQRdata      = &chassic_info.LQR_param;
	TorqData_t          *Torqdata  	  = &chassic_info.torq_data;
	balance_mode_t      *balance_mode = &chassic_info.balance_mode;
	
	//�ٶ�Ť��
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
	
	
	
	//ƽ��Ť��
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
	
	
	//ת��Ť��
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

	


	//�����������Ӹ���Ť�غ�
	Torqdata->LEG_L_Sum_Torq = ( Torqdata->Speed_Torq +
															 Torqdata->Balance_Torq -
															 Torqdata->Spin_Torq ) * 0.5f;
							
	Torqdata->LEG_L_Sum_Torq = constrain(Torqdata->LEG_L_Sum_Torq, -MAX_TORQUE, MAX_TORQUE);

	Torqdata->LEG_R_Sum_Torq = ( Torqdata->Speed_Torq +
															 Torqdata->Balance_Torq +
															 Torqdata->Spin_Torq ) * 0.5f;		

	Torqdata->LEG_R_Sum_Torq = constrain(Torqdata->LEG_R_Sum_Torq, -MAX_TORQUE, MAX_TORQUE);
	
	
	
	
	
	//ת����Ť�ص�����д��������ͽṹ���Ť�ص����У��ڲ����޷�
	Torqdata->LEG_L_Current = -K_CURRENT_TURN * (Torqdata->LEG_L_Sum_Torq / TORQUE_CONSTANT);
	
	KT_motor[LEG_L].W_iqControl(&KT_motor[LEG_L], (int16_t)Torqdata->LEG_L_Current);
	
	Torqdata->LEG_R_Current =  K_CURRENT_TURN * (Torqdata->LEG_R_Sum_Torq / TORQUE_CONSTANT);
	
	KT_motor[LEG_R].W_iqControl(&KT_motor[LEG_R], (int16_t)Torqdata->LEG_R_Current);
	
}




/**
 *	@brief	����CAN����
 */

void chassic_LQR_out(void)
{
	
	KT_motor[LEG_L].tx_W_cmd( &KT_motor[LEG_L], TORQUE_CLOSE_LOOP_ID);
		
	KT_motor[LEG_R].tx_W_cmd( &KT_motor[LEG_R], TORQUE_CLOSE_LOOP_ID);


}
/**
 *	@brief	����LQR�ܿ���
 */

void chassic_LQR_Ctrl(void)
{
	chassic_LQR_calc();
	
	chassic_LQR_out();
}

/**
 *	@brief	ƽ��������
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
 *	@brief	ƽ�������ر�
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
 *	@brief	KT�����ȡ���õ����ݣ�RM�������Ҫ��������
 */
void chassic_get_info(void)
{
	
	KT_motor[LEG_L].tx_R_cmd( &KT_motor[LEG_L], STATE1_ID);
	
	KT_motor[LEG_R].tx_R_cmd( &KT_motor[LEG_R], STATE1_ID);

}

/**
 *	@brief	KT���ж���������ֶ���շ������飬ֻ��ѷ��ͽṹ��ĵ�������,�õ���Ť�ؿ��Ƶ�ж����ʽ
						RM���ж��
						
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
 *	@brief	�����̵ĸ��������Ƿ�����������쳣�����϶�ΪҪ���ұ���������ֻ�����ñ��������־λ
						�����̵���ķ��ص��ٶȣ�X���yawƽ�棩����������ѹ���¶�
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
 *	@brief	�������ұ���----��Ե���¶Ȼ��ߵ�ѹ�쳣�������ң����ʧ�����ߵ��ʧ��
						ж��  +  ֹͣLQR����
*/
void chassic_selfprotect(void)
{

	BalanceModeOFF();
	
	chassic_stop();
}	
/**
 *	@brief	�������ֵ���ĵ�������
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
 *	@brief	����������ƣ�Ŀǰ�����KT����ﵽ��վЧ�������������ƣ�--------------�����޸�
*/
void momentum_ctrl(void)
{
	float L_tar = 0, R_tar = 0;
	
//	//�жϵ����ı�
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

