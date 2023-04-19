#ifndef __RC_SENSOR_H
#define __RC_SENSOR_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

/* Exported macro ------------------------------------------------------------*/
/* ----------------------- RC Channel Definition------------------------------*/




#define    RC_CH_VALUE_MIN       ((uint16_t)364 )
#define    RC_CH_VALUE_OFFSET    ((uint16_t)1024)
#define	   RC_CH_VALUE_MAX       ((uint16_t)1684)
#define	   RC_CH_VALUE_SIDE_WIDTH	((RC_CH_VALUE_MAX-RC_CH_VALUE_MIN)/2)

/* ----------------------- RC Switch Definition-------------------------------*/

#define    RC_SW_UP              ((uint16_t)1)
#define    RC_SW_MID             ((uint16_t)3)
#define    RC_SW_DOWN            ((uint16_t)2)

/* ----------------------- PC Key Definition-------------------------------- */

#define    KEY_PRESSED_OFFSET_W        ((uint16_t)0x01<<0)
#define    KEY_PRESSED_OFFSET_S        ((uint16_t)0x01<<1)
#define    KEY_PRESSED_OFFSET_A        ((uint16_t)0x01<<2)
#define    KEY_PRESSED_OFFSET_D        ((uint16_t)0x01<<3)
#define    KEY_PRESSED_OFFSET_SHIFT    ((uint16_t)0x01<<4)
#define    KEY_PRESSED_OFFSET_CTRL     ((uint16_t)0x01<<5)
#define    KEY_PRESSED_OFFSET_Q        ((uint16_t)0x01<<6)
#define    KEY_PRESSED_OFFSET_E        ((uint16_t)0x01<<7)
#define    KEY_PRESSED_OFFSET_R        ((uint16_t)0x01<<8)
#define    KEY_PRESSED_OFFSET_F        ((uint16_t)0x01<<9)
#define    KEY_PRESSED_OFFSET_G        ((uint16_t)0x01<<10)
#define    KEY_PRESSED_OFFSET_Z        ((uint16_t)0x01<<11)
#define    KEY_PRESSED_OFFSET_X        ((uint16_t)0x01<<12)
#define    KEY_PRESSED_OFFSET_C        ((uint16_t)0x01<<13)
#define    KEY_PRESSED_OFFSET_V        ((uint16_t)0x01<<14)
#define    KEY_PRESSED_OFFSET_B        ((uint16_t)0x01<<15)

/* ----------------------- Function Definition-------------------------------- */
/* ң��ҡ��ͨ��ƫ��ֵ */
#define		RC_SW1_VALUE				(rc_sensor_info.s1)
#define		RC_SW2_VALUE				(rc_sensor_info.s2)
#define		RC_LEFT_CH_LR_VALUE			(rc_sensor_info.ch2)
#define		RC_LEFT_CH_UD_VALUE			(rc_sensor_info.ch3)
#define		RC_RIGH_CH_LR_VALUE			(rc_sensor_info.ch0)
#define		RC_RIGH_CH_UD_VALUE			(rc_sensor_info.ch1)
#define		RC_THUMB_WHEEL_VALUE		(rc_sensor_info.thumbwheel)
/* ����WSADͨ��ƫ��ֵ */
#define   KEY_WS_CH_VALUE		 (rc_sensor_info.Ch[3])
#define   KEY_AD_CH_VALUE		 (rc_sensor_info.Ch[2])



/* ���ң��������״̬ */
#define    IF_RC_SW1_UP      (rc_sensor_info.s1 == RC_SW_UP)
#define    IF_RC_SW1_MID     (rc_sensor_info.s1 == RC_SW_MID)
#define    IF_RC_SW1_DOWN    (rc_sensor_info.s1 == RC_SW_DOWN)
#define    IF_RC_SW2_UP      (rc_sensor_info.s2 == RC_SW_UP)
#define    IF_RC_SW2_MID     (rc_sensor_info.s2 == RC_SW_MID)
#define    IF_RC_SW2_DOWN    (rc_sensor_info.s2 == RC_SW_DOWN)



/*  ÿ����λ��bitֵ��1����0  */
#define KEY_W                rc_sensor.info->key_u.bit.W		
#define KEY_S                rc_sensor.info->key_u.bit.S		
#define KEY_A                rc_sensor.info->key_u.bit.A		
#define KEY_D                rc_sensor.info->key_u.bit.D	
#define KEY_SHIFT            rc_sensor.info->key_u.bit.SHIFT	
#define KEY_CTRL             rc_sensor.info->key_u.bit.CTRL		
#define KEY_Q                rc_sensor.info->key_u.bit.Q		
#define KEY_E                rc_sensor.info->key_u.bit.E		
#define KEY_R                rc_sensor.info->key_u.bit.R		
#define KEY_F                rc_sensor.info->key_u.bit.F		
#define KEY_G                rc_sensor.info->key_u.bit.G		
#define KEY_Z                rc_sensor.info->key_u.bit.Z		
#define KEY_X                rc_sensor.info->key_u.bit.X		
#define KEY_C                rc_sensor.info->key_u.bit.C		
#define KEY_V                rc_sensor.info->key_u.bit.V		
#define KEY_B                rc_sensor.info->key_u.bit.B		
#define KEY_ALL_CODE         rc_sensor.info->key_u.key_code_val




/*����ʱ�� ���̰����ж�*/
#define CHANGE_TIM                      5    //ms
#define LONG_CHANGE_TIM_W               1000   //ms
#define LONG_CHANGE_TIM_S               1000   //ms
#define LONG_CHANGE_TIM_A               1000   //ms
#define LONG_CHANGE_TIM_D               1000   //ms
#define LONG_CHANGE_TIM_Q               500    //ms
#define LONG_CHANGE_TIM_E               500    //ms
#define LONG_CHANGE_TIM_R               500    //ms
#define LONG_CHANGE_TIM_F               500    //ms
#define LONG_CHANGE_TIM_G               500    //ms
#define LONG_CHANGE_TIM_Z               500    //ms
#define LONG_CHANGE_TIM_X               500    //ms
#define LONG_CHANGE_TIM_C               500    //ms
#define LONG_CHANGE_TIM_V               500    //ms
#define LONG_CHANGE_TIM_B               500    //ms
#define LONG_CHANGE_TIM_CTRL            500   //ms
#define LONG_CHANGE_TIM_SHIFT           500    //ms
#define LONG_CHANGE_TIM_MOUSE_L         230    //ms
#define LONG_CHANGE_TIM_MOUSE_R         500    //ms

/* �����̰���״̬ */
#define KEY_UP                    0x00
#define KEY_DOWN                  0x01

#define    CH_MIN       ((int16_t)-660)
#define    CH_OFFSET    ((int16_t)   0)
#define	   CH_MAX       ((int16_t) 660)


/*����ٶ����ֵ����*/
#define Xmax    300
#define Ymax    300
/*��껬���˲�����*/
#define SF_LENGTH 15

/*���������ƶ��ٶ�*/
#define    MOUSE_X_MOVE_SPEED    (rc_sensor.info->mouse_t.mouse_vx)
#define    MOUSE_Y_MOVE_SPEED    (rc_sensor.info->mouse_t.mouse_vy)
#define    MOUSE_Z_MOVE_SPEED    (rc_sensor.info->mouse_t.mouse_vz)

/* �����갴��״̬ */
#define    MOUSE_LEFT    (rc_sensor.info->mouse_t.mouse_press_l )
#define    MOUSE_RIGH    (rc_sensor.info->mouse_t.mouse_press_r )

#define MOUSE_X_RATE 15.f//6.46f  //���ת��Ϊͨ��ֵ�ı���
#define MOUSE_Y_RATE 10.f//8.49f  //���ת��Ϊͨ��ֵ�ı���


/* Exported types ------------------------------------------------------------*/

//����״̬���ж�˼·��ͨ������͹������˼·���������������λ��bitֵ���Ա���һ�ε�bitֵ
//bitֵΪ0��ʾû���£�Ϊ1��ʾ���£�����������bitֵ֮���ʱ��ĳ������жϾ�����ʲô״̬
typedef enum KEY_Press_State_e
{
	//�Ӱ��µ��ɿ���2������
	//���ɿ���������3������
  UP = 0,    //0
  PRESS,     //0->1
  SHORT_DOWN,//1�̰�
  DOWN,      //1����
  RELAX,     //1->0
}KEY_State_e;

typedef struct  KEY_Info_struct //����һ����λ����Ϣ
{
	uint8_t     	prev_KEY_PRESS;//��һ�������λ��bitֵ
  uint8_t     	KEY_PRESS;		 //��һ�������λ��bitֵ
	
  int         	state_cnt ;    //��¼�Ӱ��µ��ɿ����ɿ������µ�ʱ��
	
  KEY_State_e 	this_State;	
  KEY_State_e 	prev_State;

}KEY_Info_t;

typedef struct
{
  float SFX[SF_LENGTH]; 
  float SFY[SF_LENGTH]; 
  float SFZ[SF_LENGTH];	
}Mouse_SF_t;

typedef struct Keyboard_Info_struct //���̵���Ϣ
{
  KEY_Info_t     W;
  KEY_Info_t     S;
  KEY_Info_t     A;
  KEY_Info_t     D;
  KEY_Info_t     Q;       
  KEY_Info_t     E;       
  KEY_Info_t     R;       
  KEY_Info_t     F;       
  KEY_Info_t     G;      
  KEY_Info_t     Z;
  KEY_Info_t     X;
  KEY_Info_t     C;       
  KEY_Info_t     V;       
  KEY_Info_t     B;       
  KEY_Info_t     CTRL;    
  KEY_Info_t     SHIFT;   
  KEY_Info_t     Mouse_L;
  KEY_Info_t     Mouse_R; 

	Mouse_SF_t     MouseSF; //���ÿ������ٶ��˲��Ŀ������
}Keyboard_Info_t;


//����˵�����֡˳��ṹ����
typedef struct rc_sensor_info_struct {
	int16_t 	ch0;
	int16_t 	ch1;
	int16_t 	ch2;
	int16_t 	ch3;
	
	uint8_t  	s1;
	uint8_t  	s2;

	//�����Ϣ
  struct mouse_info_struct {	
		int16_t		mouse_vx;
		int16_t 	mouse_vy;
		int16_t 	mouse_vz;
		uint8_t 	mouse_press_l;
		uint8_t 	mouse_press_r;
	} mouse_t;
	
	//2���ֽ�16��λ��16��λ��ʾ16����λ�����µĻ���λ��Ӧ���Ǹ�λΪ1
	union 
	{
		uint16_t key_code_val;
		struct 
		{
			uint16_t W : 1;//ð�ű�ʾλ��
			uint16_t S : 1;
			uint16_t A : 1;
			uint16_t D : 1;
			uint16_t SHIFT : 1;
			uint16_t CTRL : 1;
			uint16_t Q : 1;
			uint16_t E : 1;
			uint16_t R : 1;
			uint16_t F : 1;
			uint16_t G : 1;
			uint16_t Z : 1;
			uint16_t X : 1;
			uint16_t C : 1;
			uint16_t V : 1;
			uint16_t B : 1;
		} bit;
	} key_u;	
	
	int16_t 	thumbwheel;	
	
	float	Mouse_ROLL; //Z����걾���˲�����ٶ�
	
	float Ch[4];	    /*���+����ģ��ͨ�������ǰ������WSAD������*/
	
	int16_t		offline_cnt;
	int16_t		offline_max_cnt;
	
} rc_sensor_info_t;

typedef struct rc_sensor_struct {
	rc_sensor_info_t	*info;
	
	
	
	drv_uart_t		  	*driver;
	void				(*init)(struct rc_sensor_struct *self);
	void				(*update)(struct rc_sensor_struct *self, uint8_t *rxBuf);
	void				(*check)(struct rc_sensor_struct *self);	
	void				(*heart_beat)(struct rc_sensor_struct *self);
	dev_work_state_e	work_state;
	dev_errno_t			errno;
	dev_id_t			id;
} rc_sensor_t;

extern rc_sensor_info_t rc_sensor_info;
extern rc_sensor_t 		rc_sensor;
extern Keyboard_Info_t keyboard;


/* Exported functions --------------------------------------------------------*/
__weak void Link_RC_CH_Is_Reset_Handler(char flag);
__weak bool Link_RC_Judge_Is_Wise_Enable_Handler(void);
__weak void Link_RC_Offline_Handler(void);
__weak void Link_RC_State_UP_Handler(char state);





bool RC_IsChannelReset(void);
void RC_ResetData(rc_sensor_t *rc);
void KEY_State_Judge(KEY_Info_t *str , uint8_t KEY_PRESS , int change_tim ,int long_change_tim);
void Key_Channel_Update(void);
void Mouse_FS(void);
#endif
