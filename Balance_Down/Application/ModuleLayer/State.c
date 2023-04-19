#include "State.h"
//---------------------------------------�ϲ㺯��
void State_Ctrl(void);//��ִ��
void State_Judge_RC_Is_Boot(void);
void Sys_Judge_Is_Reset(void);

//---------------------------------------��д�ص�����
void Comm_Judge_Is_Data_Err_Handler(char flag);//Master_Rx_Info_UP����
void Comm_RC_CH_Not_True_Handler(void);//Not_Link_RC_CH_UP����
void Comm_UP_RC_Flag_Handler(const uint8_t status);//Not_Link_RC_Flag_UP����
void Comm_UP_Chassic_Flag_Handler(const uint8_t status);//Not_Link_RC_Flag_UP����
void Comm_RC_Offline_Handler(void);//��State_Ctrl������
void Comm_Fail_Handler(void);//��State_Ctrl������


void Chassic_Flag_Set(const char flag);





void Link_RC_CH_Is_Reset_Handler(char flag);
bool Link_RC_Judge_Is_Wise_Enable_Handler(void);
void Link_RC_Offline_Handler(void);//��State_Ctrl������
void Link_RC_State_UP_Handler(char state);


State_t State =
{
	//�����е���
	.ctrl = State_Ctrl,	
	
	
	.chassic_flag.now_state = FALL,
	.chassic_flag.chas_ctrl_mode = TORQUE_CTRL,
	.chassic_flag.is_in_air = ON_GROUND,
	
};



void State_Ctrl(void)
{
	//���˽��ջ�,�������
	#ifdef LINK_TO_RECEIVE
	{
		if( RC_OFFLINE )
			Link_RC_Offline_Handler();
		
		if( FAIL_COMM )
			Comm_Fail_Handler();
	}	
	#endif
	
	//ҪôͨѶ�ɹ�����ң����ʧ��,ҪôͨѶʧ��ң����ʧ��
	#ifdef NO_LINK_TO_RECEIVE
	{
		State_Judge_RC_Is_Boot();
		Sys_Judge_Is_Reset();
		
		if( RC_OFFLINE && SUCCESS_COMM )
			Comm_RC_Offline_Handler();//����ֻ���ң�����͵��̱�־λ
		
		else if( RC_OFFLINE && FAIL_COMM )
			Comm_Fail_Handler();//������������б�־λ
	}	
	#endif

		
}

/*����ң�ؿ��ػ�ģʽ*/
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

/*ϵͳ�Ƿ�λ*/
void Sys_Judge_Is_Reset(void)
{
	if(State.chassic_flag.sys_reset_switch == OPEN)
	{
		__set_FAULTMASK(1); 
		NVIC_SystemReset();
	}
}


//--------------------------------------˫��ͨѶ���麯����д
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

	//0��1����1λ
	State.rc_flag.work_state = (dev_work_state_e)( (status >> RC_WORK_STATE_BIT) & 0x01 );
	
	//0��1����2λ
	State.rc_flag.magz_state = (switch_e)( (status >> MAGZ_FLAG_BIT) & 0x01 );
	
}

void Comm_UP_Chassic_Flag_Handler(const uint8_t status)
{
	//0��1����3λ
	if( (status >> SMALL_TOP_FLAG_BIT) & 0x01 )
		State.chassic_flag.chas_motion_mode= SMALL_TOP_ON;
	else
		State.chassic_flag.chas_motion_mode = SMALL_TOP_OFF;
	
	
	//0ǰ1���4λ
	if( (status >> GIMBAL_HEADER_BIT) & 0x01 )
		State.chassic_flag.chas_dir = ALINE_TO_B;	
	else
		State.chassic_flag.chas_dir = ALINE_TO_F;
	
	//0ͷ��ס1���ڵ�ͷ��5λ
	if( ((status >> GIMBAL_TURN_BIT) & 0x01) )
		State.chassic_flag.gimb_motion_mode = GIMB_TURING_AROUND;
	else
		State.chassic_flag.gimb_motion_mode = GIMB_TURING_OK;
	
	//0��1������
	if( ((status >> GIMBAL_AUTO_BIT) & 0x01) )
		State.chassic_flag.gimb_auto_mode = AUTO_OPEN;
	else
		State.chassic_flag.gimb_auto_mode = AUTO_CLOSE;

	//0��1��λϵͳ
	if( ((status >> SYS_RESET_BIT) & 0x01) )
		State.chassic_flag.sys_reset_switch = OPEN;
	else
		State.chassic_flag.sys_reset_switch = CLOSE;
	
}



void Comm_RC_Offline_Handler(void)
{
	//����
	State.chassic_flag.is_in_air = ON_GROUND;
	State.chassic_flag.now_state = FALL;
	State.chassic_flag.brake_ctrl_switch = CLOSE;
	State.chassic_flag.chas_motion_mode = SMALL_TOP_OFF;
	State.chassic_flag.gimb_motion_mode = GIMB_TURING_AROUND;
	State.chassic_flag.chas_ctrl_mode = TORQUE_CTRL;
	
	//ң����
	State.rc_flag.work_state = DEV_OFFLINE;
	
	State.rc_flag.ch_wise    = OPEN;
	
	State.rc_flag.magz_state = OPEN;
}

void Comm_Fail_Handler(void)
{
	//����
	State.chassic_flag.is_in_air = ON_GROUND;
	State.chassic_flag.now_state = FALL;
	State.chassic_flag.chas_motion_mode = SMALL_TOP_OFF;
	State.chassic_flag.gimb_motion_mode = GIMB_TURING_AROUND;
	
	//ң����,���˽��ջ�,�͸ı�ң������״̬��־
	#ifdef NO_LINK_TO_RECEIVE
	{
		State.rc_flag.work_state = DEV_OFFLINE;
	}
	#endif
	State.rc_flag.ch_wise    = CLOSE;
	
	State.rc_flag.magz_state = OPEN;
	
	//ͨѶ
	State.comm_flag.state_flag = COMM_FAIL;
}


















//-------------------------------------���̵��麯����д
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

























//--------------------------------------ң�������麯����д

//���˽��ջ�ģʽ,��һ���ϵ�ң����ͨ������
void Link_RC_CH_Is_Reset_Handler(char flag)
{
	State.rc_flag.ch_wise = (switch_e)flag;
}

//���˽��ջ�ģʽ,����
bool Link_RC_Judge_Is_Wise_Enable_Handler(void)
{
	return State.rc_flag.ch_wise;
}

//����ң����������ң�������б�־λ�͵������б�־λ
void Link_RC_Offline_Handler(void)
{
	//����
	State.chassic_flag.is_in_air = ON_GROUND;
	State.chassic_flag.brake_ctrl_switch = CLOSE;
	State.chassic_flag.chas_ctrl_mode = TORQUE_CTRL;
	//ң����
	State.rc_flag.work_state = DEV_OFFLINE;
	
	State.rc_flag.ch_wise    = CLOSE;
	
	State.rc_flag.magz_state = OPEN;
	
}
//���˽��ջ�������State�ı�־λ
void Link_RC_State_UP_Handler(char state)
{
	State.rc_flag.work_state = (dev_work_state_e)state;
}
