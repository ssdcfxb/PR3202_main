#ifndef __State_H
#define __State_H


#include "stm32f4xx.h"                  // Device header
#include "rp_config.h"
#include "rp_math.h"



#define    NO_LINK_TO_RECEIVE		
//#define    LINK_TO_RECEIVE

//��������ؽ��˽��ջ�,��ô��Ҫ�������ط����й�ң������־,ң����ͨ��ֵ���½��йر�
//���ǻ���Ҫ����ͨѶ



#define    RC_ONLINE             (State.rc_flag.work_state == DEV_ONLINE)
#define    RC_OFFLINE            (State.rc_flag.work_state == DEV_OFFLINE)
#define    FAIL_COMM						 (State.comm_flag.state_flag == COMM_FAIL)
#define    SUCCESS_COMM					 (State.comm_flag.state_flag == COMM_OK)

//---------------------------------------------ö������
typedef enum
{
	RC_WORK_STATE_BIT = 0,	//rc_work_state, 0��1��
	
	MAGZ_FLAG_BIT, 					//magazine_state,0��1��
	
	SMALL_TOP_FLAG_BIT,    //С����,0��1��
	
	GIMBAL_HEADER_BIT,     //ͷ��λ��,0:7191(ǰ��);   1:3095(����)
	
	GIMBAL_TURN_BIT,       //��ͷ�Ƿ����,1�ڻ�(�����ϵ�ʱͷ��λ�������);0������
	
	GIMBAL_AUTO_BIT,       //����0��1��
	
	SYS_RESET_BIT,         //ϵͳ��λ0��1��

}comm_rx_status_bit_e;


typedef enum
{
	COMM_OK = 0, //������������
	
	COMM_FAIL,//û����
	
	
	//�����ݵ�����������
	//���ڴ���1У�������
	FRAME_HEADER_ERR,
	CRC8_ERR,
	CRC16_ERR,
	
}comm_state_flg_e;



typedef enum
{
	CLOSE,
	
	OPEN,
	
	SWITCH_UP,//����
	SWITCH_DOWN,//�ػ�
	NORMAL_WORK,//��������,ֻ�е�������У����ɲ����øñ�־λ
	
}switch_e;

typedef enum
{
	//is_in_air_flag
	Fly_Out_Of_Ground = 2,//�ɳ����
	Bounce_Up_Of_Ground,//�������
	ON_GROUND,
	LAND,   					 //���һ˲��
	
	//now_state
	ALINE,//����
	FALL,//����
	SPIN,//��ת	
	NOT_STUCK,
	ALINE_STUCK, 
	SPIN_STUCK,
	
	//chas_motion_mode
	SMALL_TOP_OFF,
	SMALL_TOP_ON,

	//gimb_motion_mode
	GIMB_TURING_OK,//��ͷ���,Ҳ������ͷ
	GIMB_TURING_AROUND,//���ڻ�ͷ
	
	//chas_dir
	ALINE_TO_F,//������ǰ
	ALINE_TO_B,//�����ں�
	
	//chas_ctrl_mode
	TORQUE_CTRL,
	H_SPEED_CTRL,
	L_SPEED_CTRL,

	
	
}chas_flag_list_e;

typedef enum
{
	AUTO_CLOSE,
	AUTO_OPEN, 
	
	
}gimb_flag_list_e;


//-------------------------------------------�ṹ������

typedef struct Chas_Flag
{
	chas_flag_list_e  				is_in_air; //�Ƿ����	
	switch_e                  brake_ctrl_switch;  //ɲ����������
	chas_flag_list_e       		now_state; //��ǰ״̬ģʽ
	chas_flag_list_e				  chas_motion_mode; //���̵�ǰ�˶�ģʽ	
	chas_flag_list_e	        gimb_motion_mode; //��̨��ǰ�˶�ģʽ
	chas_flag_list_e 					chas_dir; //���̷���
	chas_flag_list_e					chas_ctrl_mode; //���̿���ģʽ
	gimb_flag_list_e          gimb_auto_mode; //�����Ƿ���          
	switch_e                  sys_reset_switch; //ϵͳ�Ƿ�λ
	
}Chas_Flag_t;

typedef struct Gimb_Flag
{
	gimb_flag_list_e          auto_state;
	
}Gimb_Flag_t;

typedef struct Master_Comm_Flag
{
	comm_state_flg_e  	state_flag;		//master_comm_heart_beat����
			
}Master_Comm_Flag_t;

typedef struct RC_Flag
{
	dev_work_state_e  	work_state;		
	switch_e         		ch_wise; 			//Link_RC_CH_Is_Reset_Handler�������޸�
	switch_e  					magz_state;		
	switch_e            boot_state;   //ֻ�Ǽ����һ˲���Ƿ񿪻��ػ�,������
	
}RC_Flag_t;
	
typedef struct
{
	Chas_Flag_t  				chassic_flag;
	Master_Comm_Flag_t  comm_flag;
	RC_Flag_t       		rc_flag;
	Gimb_Flag_t         gimbal_flag;

	void    (*ctrl)(void);

}State_t;

void Comm_Judge_Is_Data_Err_Handler(char flag);
void Comm_RC_CH_Not_True_Handler(void);
void Comm_UP_RC_Flag_Handler(const uint8_t status);
void Comm_UP_Chassic_Flag_Handler(const uint8_t status);
void Comm_RC_Offline_Handler(void);//��State_Ctrl������
void Comm_Fail_Handler(void);//��State_Ctrl������


void Chassic_Flag_Set(const char flag);



void Link_RC_CH_Is_Reset_Handler(char flag);
bool Link_RC_Judge_Is_Wise_Enable_Handler(void);
void Link_RC_Offline_Handler(void);//��State_Ctrl������
void Link_RC_State_UP_Handler(char state);

extern State_t State;

#endif
