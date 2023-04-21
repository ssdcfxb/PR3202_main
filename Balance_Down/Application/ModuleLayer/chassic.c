#include "chassic.h"
//* Private macro -------------------------------------------------------------*/
//* Private function prototypes -----------------------------------------------*/
//----------上层函数
void Chassic_Ctrl(void);
void LQR_Param_UP(float* C_LQR, float* M_LQR);
void Chassic_Status_UP(void);
void Chassic_Sys_Input_Tx(void);
void Chassic_Stop(void);
void Chassic_In_RC_OFFLINE_Handler(void);
void Chassic_Unnormal_State_Handler(void);


//-----------虚函数
__weak void Chassic_Flag_Set(const char flag);
__weak void Chas_Show_UI_Stuck(const char flag);


//----------基础函数
void Chassic_Status_Meas_UP(Sys_Status_t *status);
void Chassic_Status_Tar_UP(Sys_Status_t *status);
void Chassic_Status_Err_UP(Sys_Status_t *status);
void Chassic_Gesture_TOF_Meas_UP(void);
void Chassic_Judge_Is_In_Air(void);


void Chassic_Sys_Input_Cal(void);
void Motor_Rx_Torque_UP(Sys_Input_t *input);
void Cal_Motor_Sys_Input(Sys_Input_t *input);
void Motor_Err_Torque_UP(Sys_Input_t *input);
void Cal_Motor_Iq_Input(Sys_Input_t *input);
void Tx_KT_Motor_Iq_Input(void);
void Tx_KT_Motor_SpeedControl_Input(void);
void Tx_RM_Motor_Iq_Input(void);



//----------辅助函数
float PosMeas_2_XMeas(uint16_t pos_meas, uint16_t mid_pos_meas, MOM_motor_list_e name);
void Chassic_Sim(float WS, float AD);
void Chassic_LQR_Reset(float* c_lqr_tmp, float* m_lqr_tmp);
void Chassic_LQR_Copy(float* c_lqr_reserve, float* m_lqr_reserve);
void Chassic_Yaw_Motor_Angle_Standard(Sys_Status_t *status, 
																			const uint16_t now_angle,
																			const uint16_t bias_angle);
bool Judge_LQR_Is_Replace(float *former, float* now);//相同为1,不同为0
void Momentum_At_Limited_Pos_Protect(void);													
void Chassic_X_Speed_Tar_Dynamic_Changes(float WS);																			
void Chassic_Improve_Fixed_Point_Effects(void);
void Chassic_Brake_Over_Dynamic_Ctrl(void);
void Check_KT_Motor_State(void);
void Chas_Clear_X_Info(void);
																			
//* Private variables ---------------------------------------------------------*/
Chassic_t chassic = 
{
	.ctrl = Chassic_Ctrl,
	
};

//依次是位移,速度,动量块位移,动量快速度,角度,角速度,yaw,yaw角度

//负号依次:(车体的)- -  (动量快的)- -   (角度的)+ +  (yaw)- -
float  C_LQR_Param[STATUS_VAL_NUM] = {-8, -15, -50, -24, 100, 16, -40, -6};
//符号依次:(车体的)+ +  (动量快的)+ +   (角度的)- -  (yaw)0 0
float  M_LQR_Param[STATUS_VAL_NUM] = {0.2, 0.5, 0.8, 0.5, -0.5, -0.1, 0, 0};
//空中的动量块LQR参数,为了防止debug改错符号,在Chassic_In_Air_Handler里进行逻辑强制符号
float  M_Air_LQR_Param[STATUS_VAL_NUM] = {0, 0, 2, 0.5, -1, -0.3, 0, 0};

float C_LQR_Reserve[STATUS_VAL_NUM] = {0};
float M_LQR_Reserve[STATUS_VAL_NUM] = {0};

extern RM_motor_t MOM_motor[MOM_MOTOR_LIST];
extern KT_motor_t KT_motor[KT_MOTOR_LIST];

//4个不可修改的全局指针指向4个电机,但是9025不加const,因为在遥控失联要把电机算的总电机数清除
KT_motor_rx_info_t*  Leg_L_info = &KT_motor[LEG_L].KT_motor_info.rx_info;
KT_motor_rx_info_t*	 Leg_R_info = &KT_motor[LEG_R].KT_motor_info.rx_info;

const RM_motor_rx_info_t*  Mom_L_info = &MOM_motor[MOMENTUM_L].rx_info;
const RM_motor_rx_info_t*	 Mom_R_info = &MOM_motor[MOMENTUM_R].rx_info;

//LQR共16个参数的正负号表格,强制把动量块关于yaw方向的增益变为0
const int LQR_Sign_Buff[2][STATUS_VAL_NUM] = \
{
	//先底盘,后动量块
	{-1, -1, //车体位移速度
	 -1, -1, //动量块位移速度
	  1,  1, //车体倾角
	 -1, -1},//yaw
	
	{ 1,  1, //车体位移速度
	  1,  1, //动量块位移速度
	 -1, -1, //车体倾角
	  0,  0} //yaw
};




void Chassic_Ctrl(void)
{
	LQR_Param_UP(C_LQR_Param, M_LQR_Param);
	
	if(RC_ONLINE)
	{
		//Chassic_Unnormal_State_Handler();		
		
		Chassic_Status_UP();
		
		Chassic_Sys_Input_Tx();
	}
	
	else
	{
		Chassic_Stop();
		
		Chassic_In_RC_OFFLINE_Handler();
	}
}



//-----------------------------------------系统更新


 


//只需修改上面两个LQR数组就可保存到结构体的两个数组中
//内置符号逻辑设置,防止调参手误疯车
void LQR_Param_UP(float* C_LQR, float* M_LQR)
{
	if(C_LQR == NULL || M_LQR == NULL)
		return;
	
	uint8_t i = 0, j = 0;
	//先判断底盘符号
	for(i = 0; i < 2; i ++)
	{
		for(j = 0; j < STATUS_VAL_NUM; j ++)
		{
			//对于底盘,判断C_LQR数组
			if( i == 0 && LQR_Sign_Buff[i][j] != one(C_LQR[j]) )
			{
				if(LQR_Sign_Buff[i][j] == 0)	
					C_LQR[j] = 0;
					
				else
					C_LQR[j] *= -1;					
			}
			//对于动量块,判断M_LQR数组
			else if( i == 1 && LQR_Sign_Buff[i][j] != one(M_LQR[j]) )
			{
				if(LQR_Sign_Buff[i][j] == 0)	
					M_LQR[j] = 0;
					
				else
					M_LQR[j] *= -1;
			}		
		}
		//一轮结束修改j值
		j = 0;
	}
	i = 0;
	
	memcpy( chassic.info.C_LQR_Gain, C_LQR, STATUS_VAL_NUM * sizeof(C_LQR[0]) );
	memcpy( chassic.info.M_LQR_Gain, M_LQR, STATUS_VAL_NUM * sizeof(M_LQR[0]) );
	
	//判断空中动量块的LQR参数
	for(i = 0; i < STATUS_VAL_NUM; i ++)
	{
		if(i < M_X) //前2个参数必须是0	
			M_Air_LQR_Param[i] = 0;
		
		else if(i >= M_X && i < Y)//后4个参数的正负必须跟地上的参数一致
		{
			if( one(M_Air_LQR_Param[i]) != one(M_LQR_Param[i]) )
				M_Air_LQR_Param[i] *= -1;
		}
		
		else
			M_Air_LQR_Param[i] = 0;
		
	}
	
	
		//底盘LQR参数恢复,动量块LQR参数恢复
	Chassic_LQR_Copy(C_LQR_Reserve, M_LQR_Reserve);

}

void Chassic_Status_UP(void)
{
	
	Chassic_Status_Meas_UP(&chassic.info.Status);
	Chassic_Status_Tar_UP(&chassic.info.Status);
	Chassic_Status_Err_UP(&chassic.info.Status);
	
	Chassic_Gesture_TOF_Meas_UP();
	//Chassic_Improve_Fixed_Point_Effects();
}

//右边系统的轮子电机位移、速度加一个负号、动量块速度加一个负号跟左系统对应好

void Chassic_Status_Meas_UP(Sys_Status_t *status)
{
	if(status == NULL)
		return;

	//用总弧度 / 2pai * (2pai*R)
	status->meas.C_X_L = Leg_L_info->radian_sum * WHEEL_RADIUS;
	status->meas.C_X_R = -Leg_R_info->radian_sum * WHEEL_RADIUS;
	
	//用rpm--->m/s，首先÷60变成1s有N转，再乘以(2*pai*R)
	status->meas.C_V_L = ((float)Leg_L_info->speed) / 360.f * WHEEL_GIRTH;
	status->meas.C_V_R = -((float)Leg_R_info->speed) / 360.f * WHEEL_GIRTH;

	
	//各自的角度位置
	status->meas.L_M_Pos = Mom_L_info->angle_offset;
	status->meas.R_M_Pos = Mom_R_info->angle_offset;	
	
	//各自的位移
	status->meas.M_X_L = PosMeas_2_XMeas(status->meas.L_M_Pos,
																		   MOMENTUM_L_M_POS_OFFSET,
																		   MOMENTUM_L);
	status->meas.M_X_R = PosMeas_2_XMeas(status->meas.R_M_Pos,
																		   MOMENTUM_R_M_POS_OFFSET,
																		   MOMENTUM_R);
	
	//各自的速度
	status->meas.M_V_L = ((float)(Mom_L_info->speed)) / 60.f * MOMENTUM_GIRTH;
	status->meas.M_V_R = -((float)(Mom_R_info->speed)) / 60.f * MOMENTUM_GIRTH;		
	
	//Yaw电机角度
	Chassic_Yaw_Motor_Angle_Standard(status, 
																	 Master.rx_pack.std_frame_rx_data_t.motor_angle,
																	 YAW_MOTOR_F_ANGLE);
	
	
	//轮子平均数据
	status->meas.C_X = (status->meas.C_X_L + status->meas.C_X_R) / 2.f; 
	status->meas.C_V = (status->meas.C_V_L + status->meas.C_V_R) / 2.f; 
	
	//动量块平均数据
	status->meas.M_X = (status->meas.M_X_L + status->meas.M_X_R) / 2.f;
	status->meas.M_V = (status->meas.M_V_L + status->meas.M_V_R) / 2.f;
	
	//转弧度值
	status->meas.Pitch = imu_sensor.info->base_info.pitch * ANGLE_CONVERSION_RADIAN;
	status->meas.Rate_Pitch = imu_sensor.info->base_info.ave_rate_pitch * ANGLE_CONVERSION_RADIAN;

	
	//如果接了接收器,采用自身yaw
	#ifdef LINK_TO_RECEIVE
	{
		status->meas.Yaw = imu_sensor.info->base_info.yaw * ANGLE_CONVERSION_RADIAN;		
	}
	#endif

		
	//用上主控发的电机角度作为yaw参考值
	#ifdef NO_LINK_TO_RECEIVE
	{
		//要么是起立过程
		//要么是对齐中调头过程
		//要么起立后卡墙
		if( State.chassic_flag.now_state == FALL || \
		  ( State.chassic_flag.now_state == ALINE && \
					State.chassic_flag.gimb_motion_mode == GIMB_TURING_AROUND )	)
		{
			status->meas.Yaw = imu_sensor.info->base_info.yaw * ANGLE_CONVERSION_RADIAN;	
		}

		//只处于对齐
		//自旋中调头或不调头
		//自旋关闭过程
		else
		{
			//得到0~-360范围
			status->meas.Yaw = -status->meas.Y_Standard_Pos / RM_ANGEL_TO_IMU_ANGLE * ANGLE_CONVERSION_RADIAN;
			//规定化到-180~180
			if(status->meas.Yaw <= -VALUE_PI)
				status->meas.Yaw += VALUE_PI * 2.f;
			
	  }
			
	}
	#endif
	
	status->meas.Rate_Yaw = imu_sensor.info->base_info.ave_rate_yaw * ANGLE_CONVERSION_RADIAN;
	
}


//目标值更新中
//如果接了接收机,进行了速度目标不为0清除位移、失联保持yaw的目标值是测量值

void Chassic_Status_Tar_UP(Sys_Status_t *status)
{
	if(status == NULL)
		return;
	
	#ifdef LINK_TO_RECEIVE
	{
		Chassic_Sim(RC_LEFT_CH_UD_VALUE, RC_RIGH_CH_LR_VALUE);
	}
	#endif	
	
	#ifdef NO_LINK_TO_RECEIVE
	{

		if( State.rc_flag.ch_wise == CLOSE )	
		{
			Chassic_Sim(0, 0);
		}
		/*-------------------------------卡墙检测开始-----------------------------------------*/
		if(abs_(judge_info.power_heat_data.chassis_power_buffer)<20.f)
		{
			Chassic_Flag_Set(ALINE_STUCK);
			Chassic_Flag_Set(H_SPEED_CTRL);	
			Chas_Clear_X_Info();
			/*显示卡墙大图标*/
			Chas_Show_UI_Stuck(ALINE_STUCK);
			
			/*旋转卡墙*/
			if( abs_(chassic.info.Status.meas.Yaw) >= VALUE_PI*1.5f && \
					State.chassic_flag.chas_motion_mode == SMALL_TOP_ON )
			{
				Chassic_Flag_Set(SMALL_TOP_OFF);
				Chassic_Flag_Set(TORQUE_CTRL);
				/*显示卡墙大图标*/
				Chas_Show_UI_Stuck(SPIN_STUCK);
			}
		}	
		else
			Chas_Show_UI_Stuck(NOT_STUCK);
	
		
		/*--------------------------------状态机开始工作---------------------------------*/
		
		
		
		
		//初始状态
		if(State.chassic_flag.now_state == FALL)
		{
			//初始复位底盘用自己的yaw,等待云台复位成功再跟随
			if(State.chassic_flag.gimb_motion_mode == GIMB_TURING_AROUND)
				status->tar.Yaw = status->meas.Yaw;

			//云台复位成功
			else
			{		
				Chassic_Flag_Set(ALINE);
				Chassic_Flag_Set(TORQUE_CTRL);

			}
				
		}
		
		//直线卡墙
		else if( State.chassic_flag.now_state == ALINE_STUCK )
		{
			if(abs_(judge_info.power_heat_data.chassis_power_buffer)>40.f)
			{
				
				Chassic_Flag_Set(TORQUE_CTRL);
				Chassic_Flag_Set(ALINE);
				//在前面,目标角度为0
				if(State.chassic_flag.chas_dir == ALINE_TO_F)
				{
					status->tar.Yaw = 0.f;
				}
				//在后面,目标角度为180°,即Π弧度
				else if(State.chassic_flag.chas_dir == ALINE_TO_B)
				{
					status->tar.Yaw = VALUE_PI;
				}
			}

				
		}

		//底盘跟随
		else if(State.chassic_flag.now_state == ALINE)
  	{
			//开启小陀螺,切换底盘状态为SPIN,但要求倾角小于20°内
			if(State.chassic_flag.chas_motion_mode == SMALL_TOP_ON)
			{
			
				if(abs_(imu_sensor.info->base_info.pitch) <= 13.f && \
					 abs_(chassic.info.Status.meas.C_V_L) <= 1.f && \
					 abs_(chassic.info.Status.meas.C_V_R) <= 1.f && \
					 abs_(chassic.info.Status.err.Yaw) <= 3.f * ANGLE_CONVERSION_RADIAN)
					Chassic_Flag_Set(SPIN);

			}			
				
			
			//如果调头了
			if(State.chassic_flag.gimb_motion_mode == GIMB_TURING_AROUND)
			{
				status->tar.Yaw = status->meas.Yaw;
				Chassic_Sim(0, 0);
			}
			
			//如果头固定,要么头在前,要么头在后
			else
			{
				//在前面,目标角度为0
				if(State.chassic_flag.chas_dir == ALINE_TO_F)
				{
					if(State.rc_flag.magz_state == OPEN)
						Chassic_Sim(RC_LEFT_CH_UD_VALUE / 5.f, 0);
					
					else
						Chassic_Sim(RC_LEFT_CH_UD_VALUE, 0);

					status->tar.Yaw = 0.f;
				}
				//在后面,目标角度为180°,即Π弧度
				else if(State.chassic_flag.chas_dir == ALINE_TO_B)
				{
					if(State.rc_flag.magz_state == OPEN)
						Chassic_Sim(-RC_LEFT_CH_UD_VALUE / 5.f, 0);
					
					else
						Chassic_Sim(-RC_LEFT_CH_UD_VALUE, 0);
				
					status->tar.Yaw = VALUE_PI;
				}
				
			}
		}
		//底盘旋转
		else if(State.chassic_flag.now_state == SPIN)
		{
			//开启自旋,不允许前进后退,动量块在一前一后
			//允许云台自主移动
			//允许调头
			if(State.chassic_flag.chas_motion_mode == SMALL_TOP_ON)
			{	
				Chassic_Sim(0, 0);
				status->tar.Yaw = status->meas.Yaw + VALUE_PI / 2.f;
				status->tar.Yaw = Float_ZeroManage( status->tar.Yaw, VALUE_PI );
				
				status->tar.M_X_L = -MOMENTUM_MAX_X_MEAS;
				status->tar.M_X_R = MOMENTUM_MAX_X_MEAS;
				
				chassic.info.M_LQR_Gain[M_X] = 10 * M_LQR_Param[M_X];
				chassic.info.M_LQR_Gain[C_X] = chassic.info.M_LQR_Gain[C_V] = 0;
				
				Chas_Clear_X_Info();
			}
		
			//关闭小陀螺,根据头的位置就近回位,误差小到一定程度然后设置底盘状态为对齐
			else if(State.chassic_flag.chas_motion_mode == SMALL_TOP_OFF)
			{		

				
				status->tar.M_X_L = status->tar.M_X_R = 0;
			
				chassic.info.M_LQR_Gain[M_X] = M_LQR_Param[M_X];
				chassic.info.M_LQR_Gain[C_X] = M_LQR_Param[C_X];
				chassic.info.M_LQR_Gain[C_V] = M_LQR_Param[C_V];
				
				if(State.chassic_flag.chas_dir == ALINE_TO_F)	
					status->tar.Yaw = 0.f;
			
				else		
					status->tar.Yaw = VALUE_PI;
			
//				if( abs_(status->err.Yaw) <= 3 * ANGLE_CONVERSION_RADIAN ) 
//					Chassic_Flag_Set(ALINE);
//				else
//					Chassic_Sim(0, 0);
			}		
		}
		
	}
	
	#endif
	
	
	if( status->tar.C_V != 0 )
			 
	{
		Chas_Clear_X_Info();
	}
		
	
	
}

//采用指针+for循环算误差
void Chassic_Status_Err_UP(Sys_Status_t *status)
{
	if(status == NULL)
		return;
	
	float*  meas_p = &status->meas.C_X;
	float*  tar_p  = &status->tar.C_X;
	float*	err_p  = &status->err.C_X;
	//浮点型数据的个数
	uint8_t i_max = ( sizeof(status_t) - sizeof(status->meas.L_M_Pos) * STRUCT_STATUS_UNNOT_UP_VAL_NUM ) \
									/ sizeof(status->meas.C_X);
	
	//这里对除了动量块的机械角度以外的状态变量误差计算
	for(int i = 0; i < i_max; i ++)
	{
		(*err_p) = (*meas_p) - (*tar_p);
		
		//如果对yaw的误差进行计算，要进行过零点
		if(i == Y)
			*err_p = Float_ZeroManage( *err_p, VALUE_PI );
		
		if(i != i_max - 1)
			{	err_p ++, meas_p ++, tar_p ++; }
		 
	}
	
	//以防后续还用到,先变为最开始的值
	meas_p = &status->meas.C_X;
	tar_p  = &status->tar.C_X;
	err_p  = &status->err.C_X;


}





//----------------------------------------系统输入计算








//发送给电机计算好的系统输入

void Chassic_Sys_Input_Tx(void)
{
	Chassic_Brake_Over_Dynamic_Ctrl();
	
	Chassic_Sys_Input_Cal();
	
	Momentum_At_Limited_Pos_Protect();
	
	Tx_RM_Motor_Iq_Input();
	
	if(State.chassic_flag.chas_ctrl_mode == TORQUE_CTRL)
		Tx_KT_Motor_Iq_Input();
	
	else
		Tx_KT_Motor_SpeedControl_Input();
	
	
}
void Chassic_Sys_Input_Cal(void)
{
	//各个电机的系统输入,扭矩
	Motor_Rx_Torque_UP(&chassic.info.C_L_M);
	Cal_Motor_Sys_Input(&chassic.info.C_L_M);
	Motor_Err_Torque_UP(&chassic.info.C_L_M);
	
	Motor_Rx_Torque_UP(&chassic.info.C_R_M);
	Cal_Motor_Sys_Input(&chassic.info.C_R_M);
	Motor_Err_Torque_UP(&chassic.info.C_R_M);
	
	Cal_Motor_Sys_Input(&chassic.info.M_L_M);
	Cal_Motor_Sys_Input(&chassic.info.M_R_M);
	
	
	//各个电机的iq控制量
	Cal_Motor_Iq_Input(&chassic.info.C_L_M);
	Cal_Motor_Iq_Input(&chassic.info.C_R_M);
	Cal_Motor_Iq_Input(&chassic.info.M_L_M);
	Cal_Motor_Iq_Input(&chassic.info.M_R_M);
	
}



void Motor_Rx_Torque_UP(Sys_Input_t *input)
{
	if(input == NULL)
		return;
	
	//加一个负号
	if(input == &chassic.info.C_L_M)
		input->Rx_Torque = -Leg_L_info->torque;
	
	else if(input == &chassic.info.C_R_M)
		input->Rx_Torque = Leg_R_info->torque;
	
}
//计算单个电机的系统输入,也就是扭矩，其中左边系统轮子扭矩加负号
void Cal_Motor_Sys_Input(Sys_Input_t *input)
{
	if(input == NULL)
		return;
	
	//某一侧系统的状态变量误差，指向这个系统误差的第一个元素
	float* status_err_p = &chassic.info.Status.err.C_X;
	float* LQR_p = NULL;
	float  buffer = CHASSIC_MAX_POWER_BUFFER;
	
	//由于左右轮涉及到旋转,所以定义两个变量,即左+右,左-右;动量块不涉及
	float total_spd_t = 0, total_bal_t = 0, total_gravity_t = 0, total_yaw_t = 0;

	if(input == &chassic.info.M_L_M || input == &chassic.info.M_R_M)
		LQR_p = chassic.info.M_LQR_Gain;
	else
		LQR_p = chassic.info.C_LQR_Gain;
	if(LQR_p == NULL)
		return;
	
	
	
	
	//计算2个电机的分量扭矩和
	for(int i = 0; i < STATUS_VAL_NUM; i ++)
	{
		//速度扭矩
		if(i >= C_X && i <= C_V)
			total_spd_t += *(status_err_p + i) * LQR_p[i];
		//重心扭矩	
		else if(i >= M_X && i <= M_V)
		{
			if(input == &chassic.info.M_L_M)
				input->Gravity_Torque = chassic.info.Status.err.M_X_L * LQR_p[M_X] + \
																chassic.info.Status.err.M_V_L * LQR_p[M_V];	
			
			else if(input == &chassic.info.M_R_M)
				input->Gravity_Torque = chassic.info.Status.err.M_X_R * LQR_p[M_X] + \
															  chassic.info.Status.err.M_V_R * LQR_p[M_V];	
			
			else
				total_gravity_t += *(status_err_p + i) * LQR_p[i];
		}
			
		//平衡扭矩	
		else if(i >= P && i <= P1)
			total_bal_t += *(status_err_p + i) * LQR_p[i];
		//转向扭矩
		else if(i >= Y && i <= Y1)
			total_yaw_t += *(status_err_p + i) * LQR_p[i];
	}

	
	//左腿电机或右腿电机,用动量块的平均测量
	if(input == &chassic.info.C_L_M || input == &chassic.info.C_R_M)
	{
		//单电机扭矩分量
		input->Spd_Torque = constrain(total_spd_t / 2, -MAX_SPEED_TORQUE, MAX_SPEED_TORQUE);
		Chassis_Motor_Power_Limit(&input->Spd_Torque, buffer);

		input->Gravity_Torque = total_gravity_t / 2;
		
		input->Balance_Torque = total_bal_t / 2;
		
		if(input == &chassic.info.C_L_M)
			input->Spin_Torque = total_yaw_t / 2;
		else
			input->Spin_Torque = -total_yaw_t / 2;
		Chassis_Motor_Power_Limit(&input->Spin_Torque, buffer);

		
		
		//单电机总扭矩
		input->Tx_Torque = input->Spd_Torque + input->Gravity_Torque + \
											 input->Balance_Torque + input->Spin_Torque;
		
		//扭矩限幅
		input->Tx_Torque = constrain(input->Tx_Torque, -KT_9025_MAX_TORQUE, KT_9025_MAX_TORQUE );
		
	}

	
	//左动量块或右动量块,用各自的位移速度测量
	else if(input == &chassic.info.M_L_M || input == &chassic.info.M_R_M)
	{
		//单电机扭矩分量
		input->Spd_Torque = total_spd_t;
		input->Balance_Torque = total_bal_t;
		input->Spin_Torque = 0;
		
		//单电机总扭矩
		input->Tx_Torque = input->Spd_Torque + input->Gravity_Torque + \
											 input->Balance_Torque + input->Spin_Torque;
		
		
		//扭矩限幅
		input->Tx_Torque = constrain(input->Tx_Torque, -RM_6020_MAX_TORQUE, RM_6020_MAX_TORQUE );	
	}
	
	
	
}

void Motor_Err_Torque_UP(Sys_Input_t *input)
{
	if(input == NULL)
		return;
	
	if(input == &chassic.info.C_L_M || input == &chassic.info.C_R_M)
		input->Err_Torque = input->Rx_Torque - input->Tx_Torque;
	
}

//将系统输入变为电机的控制输入量
void Cal_Motor_Iq_Input(Sys_Input_t *input)
{
	if(input == NULL)
		return;
	
	//左腿电机或右腿电机---->iq是电流,所以扭矩--->电流
	if(input == &chassic.info.C_L_M || input == &chassic.info.C_R_M)
	{
		input->Current = input->Tx_Torque * KT_TORQUE_TO_CURRENT;
		
		//限幅iq控制量
		input->Current = constrain(input->Current, -KT_9025_MAX_CURRENT , KT_9025_MAX_CURRENT);
	
		//写到KT电机的发送结构体中
		if(input == &chassic.info.C_L_M)
			KT_motor[LEG_L].W_iqControl(&KT_motor[LEG_L], -(int16_t)input->Current);
		
		else
			KT_motor[LEG_R].W_iqControl(&KT_motor[LEG_R], (int16_t)input->Current);
	}
	
	//左动量块或右动量块---->iq是电压,所以扭矩--->电压
	else if(input == &chassic.info.M_L_M || input == &chassic.info.M_R_M)
	{
		input->Voltage = input->Tx_Torque * RM_6020_TORQUE_TO_VOLTAGE;
		
		//限幅iq控制量
		input->Voltage = constrain(input->Voltage, -RM_6020_MAX_VOLTAGE , RM_6020_MAX_VOLTAGE);
		
		//左边电压×负号
		if(input == &chassic.info.M_L_M)
			input->Voltage *= -1.f;
	}
}

//KT电机发送自己的iq控制量
void Tx_KT_Motor_Iq_Input(void)
{
	KT_motor[LEG_L].W_speedControl(&KT_motor[LEG_L], 0);
	KT_motor[LEG_R].W_speedControl(&KT_motor[LEG_R], 0);
	
	KT_motor[LEG_L].tx_W_cmd( &KT_motor[LEG_L], TORQUE_CLOSE_LOOP_ID );
  KT_motor[LEG_R].tx_W_cmd( &KT_motor[LEG_R], TORQUE_CLOSE_LOOP_ID );
}
//KT电机发送自己的速度目标值
void Tx_KT_Motor_SpeedControl_Input(void)
{				
	float speed_ctrl; 	
	
	Chas_Clear_X_Info();
	
	chassic.info.Status.tar.C_V = 0;
	RC_LEFT_CH_UD_VALUE = 0;
	memset(&chassic.info.C_L_M, 0, sizeof(Sys_Input_t));
	KT_motor[LEG_L].W_iqControl(&KT_motor[LEG_L], 0);
	memset(&chassic.info.C_R_M, 0, sizeof(Sys_Input_t));
	KT_motor[LEG_R].W_iqControl(&KT_motor[LEG_R], 0);
	
	
	if(State.chassic_flag.chas_ctrl_mode == H_SPEED_CTRL)
		speed_ctrl = KT_SPEED_TO_SPEED_CONTROL(H_SPEED_CONTROL);
	else if(State.chassic_flag.chas_ctrl_mode == L_SPEED_CTRL)
		speed_ctrl = KT_SPEED_TO_SPEED_CONTROL(L_SPEED_CONTROL);

	
	
	/*离地时,速度控制的方向看车头的朝向*/
	if(State.chassic_flag.chas_dir == ALINE_TO_F)
	{
		KT_motor[LEG_L].W_speedControl(&KT_motor[LEG_L], speed_ctrl );
		KT_motor[LEG_R].W_speedControl(&KT_motor[LEG_R], -speed_ctrl);
	}
	else if(State.chassic_flag.chas_dir == ALINE_TO_B)
	{
		KT_motor[LEG_L].W_speedControl(&KT_motor[LEG_L], -speed_ctrl );
		KT_motor[LEG_R].W_speedControl(&KT_motor[LEG_R], speed_ctrl);
	}
	else
	{
		KT_motor[LEG_L].W_speedControl(&KT_motor[LEG_L], 0);
		KT_motor[LEG_R].W_speedControl(&KT_motor[LEG_R], 0);
	}
	
	/*卡墙时,速度控制的方向要看此时的倾角倾向哪边*/	
	if(State.chassic_flag.now_state == ALINE_STUCK) 
	{
		//车后卡墙
		if( one(imu_sensor.info->base_info.pitch) == 1 )
		{
			KT_motor[LEG_L].W_speedControl(&KT_motor[LEG_L], speed_ctrl*7.f );
			KT_motor[LEG_R].W_speedControl(&KT_motor[LEG_R], -speed_ctrl*7.f );
		}
		
		//车前卡墙
		else
		{
			KT_motor[LEG_L].W_speedControl(&KT_motor[LEG_L], -speed_ctrl*7.f );
			KT_motor[LEG_R].W_speedControl(&KT_motor[LEG_R], speed_ctrl*7.f );
		}
		
	}

	
	KT_motor[LEG_L].tx_W_cmd( &KT_motor[LEG_L], SPEED_CLOSE_LOOP_ID );
	KT_motor[LEG_R].tx_W_cmd( &KT_motor[LEG_R], SPEED_CLOSE_LOOP_ID );
	
}

//RM电机发送自己的电压控制
void Tx_RM_Motor_Iq_Input(void)
{
	//对于动量快系统，有一个电压数组，两个数组序号，一个发送的ID
	int16_t M_txbuf[4] = {0};
	uint8_t M_L_Pos = MOM_motor[MOMENTUM_L].id.buff_p, \
					M_R_Pos = MOM_motor[MOMENTUM_R].id.buff_p;
	
	uint32_t Tx_ID  = MOM_motor[MOMENTUM_L].id.tx_id;
	
	M_txbuf[M_L_Pos] = (int16_t)chassic.info.M_L_M.Voltage;
	M_txbuf[M_R_Pos] = (int16_t)chassic.info.M_R_M.Voltage;
	
	CAN2_Send_With_int16_to_uint8(Tx_ID, M_txbuf);
	
}
//-----------------------------------------处理措施







//清除LQR计算的扭矩、电流、电压,允许系统更新,恢复原始LQR,清除窗口数组

void Chassic_Stop(void)
{
	Chassic_Flag_Set(TORQUE_CTRL);
	
	//清除4个电机的系统输入
	memset( &chassic.info.C_L_M, 0, sizeof(Sys_Input_t) );
	memset( &chassic.info.C_R_M, 0, sizeof(Sys_Input_t) );
	memset( &chassic.info.M_L_M, 0, sizeof(Sys_Input_t) );
	memset( &chassic.info.M_R_M, 0, sizeof(Sys_Input_t) );
	
	
	//对于9025系统
	KT_motor[LEG_L].KT_motor_info.tx_info.iqControl = 0;
	KT_motor[LEG_L].KT_motor_info.tx_info.speedControl = 0;
	KT_motor[LEG_L].tx_W_cmd( &KT_motor[LEG_L], TORQUE_CLOSE_LOOP_ID);
	
	KT_motor[LEG_R].KT_motor_info.tx_info.iqControl = 0;
	KT_motor[LEG_L].KT_motor_info.tx_info.speedControl = 0;
	KT_motor[LEG_R].tx_W_cmd( &KT_motor[LEG_R], TORQUE_CLOSE_LOOP_ID);
	
	

	//对于动量快系统
	int16_t M_txbuf[4] = {0}; 
	uint32_t Tx_ID  = MOM_motor[MOMENTUM_L].id.tx_id;
	CAN2_Send_With_int16_to_uint8( Tx_ID, M_txbuf );
	
}

//遥控器失联底盘措施,不用虚函数了,在这里写较方便
void Chassic_In_RC_OFFLINE_Handler(void)
{
	memset(&chassic.users, 0 , sizeof(User_typedef_t));
	
	//测量值处理,把9025算出来的总弧度清除
	Chassic_Status_Meas_UP(&chassic.info.Status);
	Chassic_Gesture_TOF_Meas_UP();
	Chas_Clear_X_Info();
	
	
	//目标值处理
	float yaw_meas = imu_sensor.info->base_info.yaw * ANGLE_CONVERSION_RADIAN;
	memset( &chassic.info.Status.tar.C_X, 0, sizeof(status_t) );
	chassic.info.Status.tar.Yaw = yaw_meas;
	
	//误差值依旧更新
	Chassic_Status_Err_UP(&chassic.info.Status);

	//恢复LQR
	Chassic_LQR_Reset(C_LQR_Reserve, M_LQR_Reserve);

}


//-----------------------------------------检测异常状态函数



void Chassic_Unnormal_State_Handler(void)
{
	//Chassic_Judge_Is_In_Air();
	
	Check_KT_Motor_State();
}


__weak void Chassic_Flag_Set(const char flag)
{
	
}
__weak void Chas_Show_UI_Stuck(const char flag)
{
	
}

uint32_t time_cnt;//主要是用来判断是否弹起
void Chassic_Judge_Is_In_Air(void)
{
	float L_spd = abs_(chassic.info.Status.meas.C_V_L), \
				R_spd = abs_(chassic.info.Status.meas.C_V_R), \
				pitch = abs_(imu_sensor.info->base_info.pitch), \
				F_tof_raw = chassic.info.TOF_Meas.F_Raw_X, \
				B_tof_raw = chassic.info.TOF_Meas.B_Raw_X, \
				horizon_vert = chassic.info.TOF_Meas.Horizon_Vert, \
				tof_vert_sum = chassic.info.TOF_Meas.Vert_Sum;	
	
	float tmp_buf[STATUS_VAL_NUM]; //不直接用M_Air_LQR_Param数组
	memcpy(tmp_buf, M_Air_LQR_Param, STATUS_VAL_NUM*sizeof(M_Air_LQR_Param[0]));
	
	
	
	/*多类方法判断是否离地*/
	if( (L_spd>IN_Air_SPEED) || (R_spd>IN_Air_SPEED) || \
			(F_tof_raw>CHAS_F_TOF_MAX_RAW_X+8.f) || (B_tof_raw>CHAS_B_TOF_MAX_RAW_X+8.f) || \
			(tof_vert_sum>horizon_vert+8.f) )
			Chassic_Flag_Set(Fly_Out_Of_Ground); //飞出离地类型

	/*飞出离地了*/
	if( State.chassic_flag.is_in_air == Fly_Out_Of_Ground )
	{
		//保证飞出较远距离再开启低速度控制
		if((tof_vert_sum-horizon_vert)>=15.f && pitch<5.f) //水平悬空才开启较高速度
			Chassic_Flag_Set(H_SPEED_CTRL);
		else 
			Chassic_Flag_Set(L_SPEED_CTRL);
		
		#ifdef LINK_TO_RECEIVE
		{
			if(chassic.info.Status.meas.C_V > 0)
				Chassic_Flag_Set(ALINE_TO_F);
			else
				Chassic_Flag_Set(ALINE_TO_B);
		}
		#endif

		tmp_buf[P] = tmp_buf[P1] = 0;
		
		memset(C_LQR_Param, 0, STATUS_VAL_NUM);
					
		memcpy(M_LQR_Param, tmp_buf, STATUS_VAL_NUM);
													 		
	}
		



	
	/*为了防止落地一瞬间弹起但那时候已经采取扭矩控制导致弹起后姿态有明显变化,
		不能一落地就立刻采取扭矩控制,
		并且弹起离地后动量块用角度控制*/
	if( State.chassic_flag.is_in_air != ON_GROUND )
	{
		/*判断是否落地*/
		if( ((L_spd<0.15f) || (R_spd<0.15f) || \
				(F_tof_raw<CHAS_F_TOF_MAX_RAW_X+3.f) || (B_tof_raw<CHAS_B_TOF_MAX_RAW_X+3.f)) && \
				(State.chassic_flag.is_in_air == Fly_Out_Of_Ground) )
			Chassic_Flag_Set(LAND);
		
		
		
		
		
		/*判断是否落地后弹起*/
		if( State.chassic_flag.is_in_air == LAND )
		{
			if(time_cnt == 0)
				time_cnt = HAL_GetTick();
			
			/*策略:通过等一小段时间来判断距离*/
			if( (HAL_GetTick() - time_cnt) >= 500 )
			{
				/*是弹起*/
				if(	(F_tof_raw>CHAS_F_TOF_MAX_RAW_X+4.f) || (B_tof_raw>CHAS_B_TOF_MAX_RAW_X+4.f) || \
						(tof_vert_sum>horizon_vert+3.f) )
				{
					Chassic_Flag_Set(Bounce_Up_Of_Ground);
					
					tmp_buf[M_X] = tmp_buf[M_V] = 0;
			
					memset(C_LQR_Param, 0, STATUS_VAL_NUM);
					
					memcpy(M_LQR_Param, tmp_buf, STATUS_VAL_NUM);
										 
					//控制模式为速度
					Chassic_Flag_Set(L_SPEED_CTRL);				
				}
				/*没有弹起*/
				else
					Chassic_Flag_Set(ON_GROUND);		
			}	
		}
		

		
	}
	else
	{	
		time_cnt = 0;
		
		Chassic_LQR_Reset(C_LQR_Reserve, M_LQR_Reserve);

		//控制模式为扭矩
		Chassic_Flag_Set(TORQUE_CTRL);	
	}
}





//-----------------------------------------辅助函数






/**
 *	@brief	动量块当前位置转变为位移
 */
float PosMeas_2_XMeas(uint16_t pos_meas, uint16_t mid_pos_meas, MOM_motor_list_e name)
{
  float scale = 0;//刻度，记录电机走1m要走多少机械角度
	
	float x_meas = 0;
	
	//确保左动量快在前面的位移是正的
	if( name == MOMENTUM_L )
	{	
		scale = ((float)L_MOMENTUM_B_2_F_ANGLE_SUM) / CHASSIC_LENGTH ; 
		x_meas = ((float)(mid_pos_meas - pos_meas)) / scale ;
	}
	else if( name == MOMENTUM_R )
	{	
		scale = ((float)R_MOMENTUM_B_2_F_ANGLE_SUM) / CHASSIC_LENGTH ; 
		x_meas = ((float)(pos_meas - mid_pos_meas)) / scale ;	
	}
	
	x_meas = constrain(x_meas, -MOMENTUM_MAX_X_MEAS, MOMENTUM_MAX_X_MEAS);

	return x_meas;
}

/**
 *	@brief	底盘映射目标值
 */
void Chassic_Sim(float WS, float AD)
{
	//左右系统的速度目标一致
	//Chassic_X_Speed_Tar_Dynamic_Changes(WS);
	if(JUDGE_ONLINE)
	{
		chassic.info.Status.tar.C_V = WS / CH_MAX * X_MAX_START_SPEED;
	}
	else
	{
		chassic.info.Status.tar.C_V = WS / CH_MAX * Judge_Offline_X_Speed;
	}
	
	
	float v_tar = chassic.info.Status.tar.C_V, \
				v_err = chassic.info.Status.err.C_V;	
	
	float yaw_tar = chassic.info.Status.tar.Yaw, \
				yaw_err = chassic.info.Status.err.Yaw;

	
	if( abs_(yaw_err) < MAX_YAW_ANGLE_ERR * ANGLE_CONVERSION_RADIAN )
	{
		//如果只是原地旋转
		if( v_tar == 0 && abs_(v_err) < SPIN_STATE_X_SPEED )
			yaw_tar += AD / -CH_MAX * ONLY_SPIN_RATE;  
		
		//如果是还没刹车完就旋转
		if( v_tar == 0 && abs_(v_err) > SPIN_STATE_X_SPEED )
			yaw_tar += AD / -CH_MAX * SPIN_AS_BRAKING_RATE;  
		
		//如果是前进的时候要旋转
		if( v_tar != 0 )
			yaw_tar += AD / -CH_MAX * SPIN_AS_FORWARD_RATE;  
	
	}

	yaw_tar = Float_ZeroManage( yaw_tar, VALUE_PI );
	
	chassic.info.Status.tar.Yaw = yaw_tar; 
	
}


//相同为1,不同为0
bool Judge_LQR_Is_Replace(float *former, float* now)
{
	if(former == NULL || now == NULL)
		return 1;
	
	uint8_t same_cnt = 0;
	
	for(int i = 0; i < STATUS_VAL_NUM; i ++)
	{
		if(former[i] == now[i])
			same_cnt ++;
	}	
	
	if(same_cnt == STATUS_VAL_NUM)
		return 1;
	else
		return 0;
			
}

void Chassic_LQR_Copy(float* c_lqr_reserve, float* m_lqr_reserve)
{
	if(c_lqr_reserve == NULL || m_lqr_reserve == NULL)
		return;
	
	//底盘LQR参数拷贝
	float zero_buf[STATUS_VAL_NUM] = {0};
	if(Judge_LQR_Is_Replace(C_LQR_Param, zero_buf) == Flase)
		memcpy(c_lqr_reserve, C_LQR_Param, STATUS_VAL_NUM * sizeof(C_LQR_Param[0]));
	
	//动量块LQR参数拷贝
	if(Judge_LQR_Is_Replace(M_LQR_Param, M_Air_LQR_Param) == Flase)
		memcpy(m_lqr_reserve, M_LQR_Param, STATUS_VAL_NUM * sizeof(M_LQR_Param[0]));
	
}

//内置防止debug的时候修改两个备份数组
void Chassic_LQR_Reset(float* c_lqr_tmp, float* m_lqr_tmp)
{
	if(c_lqr_tmp == NULL || m_lqr_tmp == NULL)
		return;
	
	memcpy(C_LQR_Param, c_lqr_tmp, STATUS_VAL_NUM * sizeof(C_LQR_Param[0]));
		
	memcpy(M_LQR_Param, m_lqr_tmp, STATUS_VAL_NUM * sizeof(M_LQR_Param[0]));
}

//bias_angle是想要变为0的原机械角度
void Chassic_Yaw_Motor_Angle_Standard(Sys_Status_t *status, 
																			const uint16_t now_angle,
																			const uint16_t bias_angle)
{
	status->meas.Y_Pos = now_angle;
	
	uint16_t err = RM_TOTAL_ANGLE - bias_angle;
	
	status->meas.Y_Standard_Pos = status->meas.Y_Pos + err;
	
	if(status->meas.Y_Standard_Pos >= RM_TOTAL_ANGLE)
		status->meas.Y_Standard_Pos -= RM_TOTAL_ANGLE;
	
}

void Momentum_At_Limited_Pos_Protect(void)
{
	float M_X_L = chassic.info.Status.meas.M_X_L, \
		    M_X_R = chassic.info.Status.meas.M_X_R;
	
	float M_L_T = chassic.info.M_L_M.Tx_Torque, \
		    M_R_T = chassic.info.M_R_M.Tx_Torque;
	
	//左动量块在最前或在最后
	if( abs_(M_X_L) >= MOMENTUM_MAX_X_MEAS - 0.01f ) //0.17m
	{
		//动量块在前(即正的)且扭矩一直往前(即负的)
		//动量块在后(即负的)且扭矩一直往后(即正的)
		if( M_X_L * M_L_T < 0 )
			chassic.info.M_L_M.Tx_Torque = chassic.info.M_L_M.Voltage = 0;
	}
	
	if( abs_(M_X_R) >= MOMENTUM_MAX_X_MEAS - 0.01f )
	{
		//动量块在前(即正的)且扭矩一直往前(即负的)
		//动量块在后(即负的)且扭矩一直往后(即正的)
		if( M_X_R * M_R_T < 0 )
			chassic.info.M_R_M.Tx_Torque = chassic.info.M_R_M.Voltage = 0;
	}
	
}


//如果底盘速度在起步最大速度附近且此时遥控器通道值一直都拉满
//就让动态提高目标速度
void Chassic_X_Speed_Tar_Dynamic_Changes(float WS)
{
	float spd_meas = chassic.info.Status.meas.C_V;
	float spd_err  = chassic.info.Status.err.C_V;
	
	//第一阶段,先到达最大起步
	if(abs_(spd_meas) <= X_MAX_START_SPEED - 0.2f) //当前速度小于1.8m/s
		chassic.info.Status.tar.C_V = WS / CH_MAX * X_MAX_START_SPEED;
		
	//第二阶段,提高目标速度
	else  //当前速度大于1.8m/s
	{
		//想要正向再加速
		if(WS ==  CH_MAX)
		{
			if(abs_(spd_err) <= 0.2f)
				chassic.info.Status.tar.C_V += 0.5f;
		}
		
		//想要反向再加速
		else if(WS ==  CH_MIN)
		{
			if(abs_(spd_err) <= 0.2f)
				chassic.info.Status.tar.C_V -= 0.5f;
		}
		
		else
		{
			chassic.info.Status.tar.C_V = WS / CH_MAX * abs_(spd_meas);
		}
		
		chassic.info.Status.tar.C_V = constrain(chassic.info.Status.tar.C_V,
																						-X_MAX_LIMITED_SPEED,
																						X_MAX_LIMITED_SPEED );
	}
}



//刹车调整位移目标0点
int16_t Last_WS_Val, This_WS_Val;
float kp,ki;
void Chassic_Brake_Over_Dynamic_Ctrl(void)
{
	float cnt;
	This_WS_Val = RC_LEFT_CH_UD_VALUE;
	
	
	if(Last_WS_Val != This_WS_Val && \
		 abs_(chassic.info.Status.meas.C_V) > 0.5f && \
		 This_WS_Val == 0)
	{	
		Chassic_Flag_Set(OPEN);
		
		chassic.users.brake_over_dir = one(chassic.info.Status.meas.C_V);
	}

		
	//如果刹车一瞬间突然要开启小陀螺了
	if(State.chassic_flag.brake_ctrl_switch == OPEN && \
		 State.chassic_flag.chas_motion_mode == SMALL_TOP_ON)
	{
		Chassic_Flag_Set(CLOSE);
		chassic.info.C_LQR_Gain[C_X] = C_LQR_Param[C_X];
		Chas_Clear_X_Info();
	}
		
	
	//开启回拉时突然拨遥控器
	if(State.chassic_flag.brake_ctrl_switch == OPEN && \
		 This_WS_Val != 0)
	{
		Chassic_Flag_Set(CLOSE);
		chassic.info.C_LQR_Gain[C_X] = C_LQR_Param[C_X];
		Chas_Clear_X_Info();
	}
		
	
	if(State.chassic_flag.brake_ctrl_switch == OPEN)
	{					
		chassic.info.C_LQR_Gain[C_X] = C_LQR_Param[C_X];
		cnt = abs_(chassic.info.Status.meas.C_X)/0.1f;
			
		if( abs_(chassic.info.Status.meas.C_V) <= 0.02f )
		{
			Chas_Clear_X_Info();
			Chassic_Flag_Set(CLOSE);	
		}
		
		/*刹车速度还没到0*/
		else
		{
			if(abs_(chassic.info.Status.meas.Pitch) < 15.f * ANGLE_CONVERSION_RADIAN)
				chassic.info.C_LQR_Gain[C_X] -= cnt*10.f;
			else
				chassic.info.C_LQR_Gain[C_X] -= cnt*5.f;
		}
	}


	Last_WS_Val = This_WS_Val;
}

//姿态距离结算
void Chassic_Gesture_TOF_Meas_UP(void)
{
	//原始测距数据
	chassic.info.TOF_Meas.F_Raw_X = TOF_Sensor[POS_F].info.raw_dis_meas;
	chassic.info.TOF_Meas.B_Raw_X = TOF_Sensor[POS_B].info.raw_dis_meas;
	
	//计算得到的垂直距离
	chassic.info.TOF_Meas.F_Vert_X = arm_cos_f32(abs_(chassic.info.Status.meas.Pitch)) * \
																							 chassic.info.TOF_Meas.F_Raw_X;
	chassic.info.TOF_Meas.B_Vert_X = arm_cos_f32(abs_(chassic.info.Status.meas.Pitch)) * \
																							 chassic.info.TOF_Meas.B_Raw_X;
	chassic.info.TOF_Meas.Vert_Sum = chassic.info.TOF_Meas.F_Vert_X + \
																	 chassic.info.TOF_Meas.B_Vert_X;
	chassic.info.TOF_Meas.Horizon_Vert = CHAS_Horizontal_Height * 2.f;
}





/*检查轮子是否持续高电流*/
void Check_KT_Motor_State(void)
{
	if( abs_(Leg_L_info->temperature) > 60.f )
	{
		if(chassic.users.L_M_Over_I_Time == 0)
			chassic.users.L_M_Over_I_Time = HAL_GetTick();
		
		if(HAL_GetTick() >= (chassic.users.L_M_Over_I_Time + 8000))
			Chassic_Stop();
	}
	else
		chassic.users.L_M_Over_I_Time = 0;
	
	if( abs_(Leg_R_info->temperature) > 60.f )
	{
		if(chassic.users.R_M_Over_I_Time == 0)
			chassic.users.R_M_Over_I_Time = HAL_GetTick();
		
		if(HAL_GetTick() >= (chassic.users.R_M_Over_I_Time + 8000))
			Chassic_Stop();
	}
	else
		chassic.users.R_M_Over_I_Time = 0;	
	
}

/*清除位移信息*/
void Chas_Clear_X_Info(void)
{
	Leg_L_info->encoder_prev = Leg_R_info->encoder_prev = 0;
	Leg_L_info->encoder_sum = Leg_R_info->encoder_sum = 0;
	Leg_L_info->radian_sum = Leg_R_info->radian_sum = 0;
  chassic.info.Status.meas.C_X = 0;	
}
