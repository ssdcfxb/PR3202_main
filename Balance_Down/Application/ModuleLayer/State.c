#include "State.h"
//---------------------------------------上层函数
void State_Ctrl(void);//总执行
void State_Judge_RC_Is_Boot(void);
void Sys_Judge_Is_Reset(void);

//---------------------------------------重写回调函数
void Comm_Judge_Is_Data_Err_Handler(char flag);//Master_Rx_Info_UP调用
void Comm_RC_CH_Not_True_Handler(void);//Not_Link_RC_CH_UP调用
void Comm_UP_RC_Flag_Handler(const uint8_t status);//Not_Link_RC_Flag_UP调用
void Comm_UP_Chassic_Flag_Handler(const uint8_t status);//Not_Link_RC_Flag_UP调用
void Comm_RC_Offline_Handler(void);//在State_Ctrl被调用
void Comm_Fail_Handler(void);//在State_Ctrl被调用


void Chassic_Flag_Set(const char flag);





void Link_RC_CH_Is_Reset_Handler(char flag);
bool Link_RC_Judge_Is_Wise_Enable_Handler(void);
void Link_RC_Offline_Handler(void);//在State_Ctrl被调用
void Link_RC_State_UP_Handler(char state);


State_t State =
{
	//任务中调用
	.ctrl = State_Ctrl,	
	
	
	.chassic_flag.now_state = FALL,
	.chassic_flag.chas_ctrl_mode = TORQUE_CTRL,
	.chassic_flag.is_in_air = ON_GROUND,
	
};



void State_Ctrl(void)
{
	//接了接收机,毫无相干
	#ifdef LINK_TO_RECEIVE
	{
		if( RC_OFFLINE )
			Link_RC_Offline_Handler();
		
		if( FAIL_COMM )
			Comm_Fail_Handler();
	}	
	#endif
	
	//要么通讯成功但是遥控器失联,要么通讯失败遥控器失联
	#ifdef NO_LINK_TO_RECEIVE
	{
		State_Judge_RC_Is_Boot();
		Sys_Judge_Is_Reset();
		
		if( RC_OFFLINE && SUCCESS_COMM )
			Comm_RC_Offline_Handler();//这里只清除遥控器和底盘标志位
		
		else if( RC_OFFLINE && FAIL_COMM )
			Comm_Fail_Handler();//这里是清除所有标志位
	}	
	#endif

		
}

/*新增遥控开关机模式*/
uint8_t last_work_state, this_work_state;
void State_Judge_RC_Is_Boot(void)
{
	this_work_state = State.rc_flag.work_state;
	
	if(this_work_state == DEV_ONLINE && last_work_state == DEV_OFFLINE)
		State.rc_flag.boot_state = SWITCH_UP;
	
	else if(this_work_state == DEV_OFFLINE && last_work_state == DEV_ONLINE)
		State.rc_flag.boot_state = SWITCH_DOWN;
	
	last_work_state = this_work_state;
	
}

/*系统是否复位*/
void Sys_Judge_Is_Reset(void)
{
	if(State.chassic_flag.sys_reset_switch == OPEN)
	{
		__set_FAULTMASK(1); 
		NVIC_SystemReset();
	}
}


//--------------------------------------双板通讯的虚函数重写
void Comm_Judge_Is_Data_Err_Handler(char flag)
{
	State.comm_flag.state_flag = (comm_state_flg_e)flag;
	
}

void Comm_RC_CH_Not_True_Handler(void)
{
	State.rc_flag.ch_wise = CLOSE;
}

void Comm_UP_RC_Flag_Handler(const uint8_t status)
{

	//0关1开低1位
	State.rc_flag.work_state = (dev_work_state_e)( (status >> RC_WORK_STATE_BIT) & 0x01 );
	
	//0关1开低2位
	State.rc_flag.magz_state = (switch_e)( (status >> MAGZ_FLAG_BIT) & 0x01 );
	
}

void Comm_UP_Chassic_Flag_Handler(const uint8_t status)
{
	//0关1开低3位
	if( (status >> SMALL_TOP_FLAG_BIT) & 0x01 )
		State.chassic_flag.chas_motion_mode= SMALL_TOP_ON;
	else
		State.chassic_flag.chas_motion_mode = SMALL_TOP_OFF;
	
	
	//0前1后低4位
	if( (status >> GIMBAL_HEADER_BIT) & 0x01 )
		State.chassic_flag.chas_dir = ALINE_TO_B;	
	else
		State.chassic_flag.chas_dir = ALINE_TO_F;
	
	//0头锁住1正在调头低5位
	if( ((status >> GIMBAL_TURN_BIT) & 0x01) )
		State.chassic_flag.gimb_motion_mode = GIMB_TURING_AROUND;
	else
		State.chassic_flag.gimb_motion_mode = GIMB_TURING_OK;
	
	//0关1开自瞄
	if( ((status >> GIMBAL_AUTO_BIT) & 0x01) )
		State.chassic_flag.gimb_auto_mode = AUTO_OPEN;
	else
		State.chassic_flag.gimb_auto_mode = AUTO_CLOSE;

	//0关1复位系统
	if( ((status >> SYS_RESET_BIT) & 0x01) )
		State.chassic_flag.sys_reset_switch = OPEN;
	else
		State.chassic_flag.sys_reset_switch = CLOSE;
	
}



void Comm_RC_Offline_Handler(void)
{
	//底盘
	State.chassic_flag.is_in_air = ON_GROUND;
	State.chassic_flag.now_state = FALL;
	State.chassic_flag.brake_ctrl_switch = CLOSE;
	State.chassic_flag.chas_motion_mode = SMALL_TOP_OFF;
	State.chassic_flag.gimb_motion_mode = GIMB_TURING_AROUND;
	State.chassic_flag.chas_ctrl_mode = TORQUE_CTRL;
	
	//遥控器
	State.rc_flag.work_state = DEV_OFFLINE;
	
	State.rc_flag.ch_wise    = OPEN;
	
	State.rc_flag.magz_state = OPEN;
}

void Comm_Fail_Handler(void)
{
	//底盘
	State.chassic_flag.is_in_air = ON_GROUND;
	State.chassic_flag.now_state = FALL;
	State.chassic_flag.chas_motion_mode = SMALL_TOP_OFF;
	State.chassic_flag.gimb_motion_mode = GIMB_TURING_AROUND;
	
	//遥控器,接了接收机,就改变遥控器的状态标志
	#ifdef NO_LINK_TO_RECEIVE
	{
		State.rc_flag.work_state = DEV_OFFLINE;
	}
	#endif
	State.rc_flag.ch_wise    = CLOSE;
	
	State.rc_flag.magz_state = OPEN;
	
	//通讯
	State.comm_flag.state_flag = COMM_FAIL;
}


















//-------------------------------------底盘的虚函数重写
void Chassic_Flag_Set(const char flag)
{
	chas_flag_list_e chas_flag = (chas_flag_list_e)flag;
	
	if(flag == CLOSE || flag == OPEN) 
		State.chassic_flag.brake_ctrl_switch = (switch_e)flag;
	
	else if(chas_flag == Fly_Out_Of_Ground || chas_flag == Bounce_Up_Of_Ground || \
					chas_flag == ON_GROUND || chas_flag == LAND)
		State.chassic_flag.is_in_air = chas_flag;
	
	else if(chas_flag == SMALL_TOP_OFF || chas_flag == SMALL_TOP_ON )
		State.chassic_flag.chas_motion_mode = chas_flag;	
	
	else if(chas_flag == ALINE || chas_flag == FALL || chas_flag == SPIN || \
					chas_flag == ALINE_STUCK || chas_flag == SPIN_STUCK)
		State.chassic_flag.now_state = chas_flag;	
	
	else if(chas_flag == GIMB_TURING_OK || chas_flag == GIMB_TURING_AROUND)
		State.chassic_flag.gimb_motion_mode = chas_flag;
	
	else if(chas_flag == ALINE_TO_F || chas_flag == ALINE_TO_B)
		State.chassic_flag.chas_dir = chas_flag;
	
	else if(chas_flag == TORQUE_CTRL || chas_flag == H_SPEED_CTRL || chas_flag == L_SPEED_CTRL)
		State.chassic_flag.chas_ctrl_mode = chas_flag;
	
}

























//--------------------------------------遥控器的虚函数重写

//接了接收机模式,第一次上电遥控器通道正常
void Link_RC_CH_Is_Reset_Handler(char flag)
{
	State.rc_flag.ch_wise = (switch_e)flag;
}

//接了接收机模式,返回
bool Link_RC_Judge_Is_Wise_Enable_Handler(void)
{
	return State.rc_flag.ch_wise;
}

//接了遥控器后重置遥控器所有标志位和底盘所有标志位
void Link_RC_Offline_Handler(void)
{
	//底盘
	State.chassic_flag.is_in_air = ON_GROUND;
	State.chassic_flag.brake_ctrl_switch = CLOSE;
	State.chassic_flag.chas_ctrl_mode = TORQUE_CTRL;
	//遥控器
	State.rc_flag.work_state = DEV_OFFLINE;
	
	State.rc_flag.ch_wise    = CLOSE;
	
	State.rc_flag.magz_state = OPEN;
	
}
//接了接收机，更新State的标志位
void Link_RC_State_UP_Handler(char state)
{
	State.rc_flag.work_state = (dev_work_state_e)state;
}
