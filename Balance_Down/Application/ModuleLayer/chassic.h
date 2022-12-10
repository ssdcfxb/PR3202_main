#ifndef __CHASSIC_H
#define __CHASSIC_H

#include "device.h"
#include "algo.h"
/* Exported macro ------------------------------------------------------------*/

//����Ϊ��������
#define MEC_ANGLE          -2.0f		   					//�����Ļ�е��ֵ�Ƕȣ�����������ֵ
#define WHEEL_RADIUS        0.1f          			//���Ӱ뾶      m��λ

#define MOMENTUM_L_F_POS    1400              //��������ǰ��ĵ��λ�� //-----------������
#define MOMENTUM_L_M_POS   	7840  						//���������м�ĵ��λ��
#define MOMENTUM_L_B_POS   	3280							//�������ں���ĵ��λ��

#define MOMENTUM_R_F_POS    6890              //��������ǰ��ĵ��λ�� //-----------������
#define MOMENTUM_R_M_POS   	1110  						//���������м�ĵ��λ��
#define MOMENTUM_R_B_POS   	5278							//�������ں���ĵ��λ��




//����Ϊ�������ƵĲ���
#define X_MAX_SPEED      2.8f          //�����������ٶ�
#define Z_MAX_SPEED      2.1f          //����������ת�ٶ�


/* Exported types ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
 *	@brief	LQR
 */
 
 /*	@note  k1* ����λ��		 k2* �������ٶ�	  
					 k3* ����Ƕ�    k4* ���Ľ��ٶ�       
           k5* ƫ����	     k6* ƫ�����ٶ�			*/
typedef struct 
{
	float k1;	
	float k2;
	float k3;
	float k4;
	float k5;
	float k6;
} LQRData_t;


/**
 *	@brief	����ƽ����������
 *	@class	balance_mode
 */
typedef enum{
	OFF,
	ON,
}off_on_t;

typedef struct {
	off_on_t speed;
	off_on_t stand;
	off_on_t turn;
} balance_mode_t;


/**
 *	@brief	����״̬��������ֵ��Ŀ��ֵ�����ֵ
 */
typedef struct 
{
//����ֵ	
	float  along_X_distance_meas;       //����X���߹���λ��
	float  along_X_speed_meas;		   //����X������ٶ�
	
	float  pitch_meas;            
	float  rate_pitch_meas;   
	
	float  yaw_meas;
	float  rate_yaw_meas;
	
	
//Ŀ��ֵ��Ŀǰֻ��Ҫ���ٶ�Ŀ��ֵ��yaw��Ľ��ٶ�Ŀ��ֵ
	float X_speed_tar;           //����X������ٶ�Ŀ��ֵ
	float X_speed_max_tar;       
	
	float pitch_tar;             //
	float rate_pitch_tar;        
	
	float yaw_tar;               //
	float rate_yaw_tar;
	float rate_yaw_max_tar;
	
	
//���ֵ��Ŀǰֻ������
	float X_speed_err;
	
	float rate_yaw_err;
	
} status_variables_t;

/**
 *	@brief	Ť������
 *	@class	LQR
 */
typedef struct {
	float Speed_Torq;						//�ٶ�����	
	float Balance_Torq;					//���ƽ������	
	float Spin_Torq;						//���ת������

	float Last_Speed_Torq;      //��һ�ε��ٶ�����
	float Last_Spin_Torq;       //��һ�ε�ת��Ť��
		
	float Max_Speed_Torq;       //�����ٶ�����
	float Max_Balance_Torq;     //����ƽ������
	float Max_Spin_Torq;        //����ת������
	
	float LEG_L_Sum_Torq;       //��������֮��
	float LEG_R_Sum_Torq;       //��������֮��
		
	float LEG_L_Current;        //���ֵ�Ť�ص���
	float LEG_R_Current;        //���ֵ�Ť�ص���
}TorqData_t;



typedef struct
{
	KT_motor_t           *chassic_motor;
	RM_motor_t           *momentum_motor;
	
	status_variables_t   status_var;
	LQRData_t            LQR_param;
	TorqData_t           torq_data;
	balance_mode_t       balance_mode;
	
}chassis_info_t;

typedef struct
{
	chassis_info_t       *info;
	
	void                (*ctrl)(void);		//�ܿ���
	
	void                (*get_info)(void);
	void			   			  (*stop)(void);	
	void                (*selfprotect)(void);
	void                (*exam_status)(void);
	void                (*update_status_var)(void);
	void                (*LQR_ctrl)(void);
	void								(*momentum_out)(float L_tar, float R_tar);
	
}chassic_class_t;


extern chassic_class_t chassic;
#endif
