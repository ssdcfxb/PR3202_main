#ifndef __CHASSIC_H
#define __CHASSIC_H

#include "device.h"

/* Exported macro ------------------------------------------------------------*/


#define WHEEL_RADIUS     0.1f          //���Ӱ뾶 m��λ
/* Exported types ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
 *	@brief	LQR
 */
 
 /*	@note  k1* ����λ��		 k2* �������ٶ�	  
		   k3* ����Ƕ�      k4* ���Ľ��ٶ�       
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
 *	@brief	����״̬����
 */
typedef struct 
{
	float  along_X_distance;       //����X���߹���λ��
	float  along_X_speed;			     //����X������ٶ�
	
	float  pitch;            //
	float  rate_pitch;   
	
	float  yaw;
	float  rate_yaw;
	
} status_variables_t;

/**
 *	@brief	Ť������
 *	@class	LQR
 */
typedef struct {
	float Speed_Torq;			//�ٶ�����	
	float Balance_Torq;			//���ƽ������	
	float Spin_Torq;			//���ת������

	float Last_Speed_Torq;      //��һ�ε��ٶ�����
	float Last_Spin_Torq;       //��һ�ε�ת��Ť��
	
	float LEG_L_Current;        //���ֵ�Ť�ص���
	float LEG_R_Current;        //���ֵ�Ť�ص���
	
	float Max_Speed_Torq;       //�����ٶ�����
	float Max_Balance_Torq;     //����ƽ������
	float Max_Spin_Torq;        //����ת������
	
	float LEG_L_Sum_Torq;       //��������֮��
	float LEG_R_Sum_Torq;       //��������֮��
}TorqData_t;



typedef struct
{
	KT_motor_t           *chassic_motor;
	status_variables_t   status_var;
	LQRData_t            LQR_param;
	TorqData_t           torq_data;
	balance_mode_t       balance_mode;
	
}chassis_info_t;

typedef struct
{
	chassis_info_t       *info;
	
	void                (*get_info)(void);
	void			    (*stop)(void);	
	void                (*selfprotect)(void);
	void                (*update_status_var)(void);
	void                (*LQR_ctrl)(void);
	
	
}chassic_class_t;

#endif
