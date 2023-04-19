#ifndef __Under_Layer_H
#define __Under_Layer_H

/*����
1.�豸���ṩ��������,���浽device_info_t����
2.ϵͳĿ��ֵ����
3.ϵͳ����ֵ����
4.ϵͳ���ֵ����

*/



/*---------------------------Pravited  Include-----------------------------*/
#include "device.h"
#include "stdarg.h"

/*---------------------------Pravited  Define---------------------------*/



/*---------------------------Pravited  Typedef--------------------------*/

/*״̬����*/
typedef struct status
{
	//ƽ��
	float  C_X;       
	float  C_V;		 
	
	//ƽ��
	float  M_X;       
	float  M_V;	
		
	float  Pitch;            
	float  Pitch_1;   

	float  Yaw;
	float  Yaw_1;		
	
	float  Roll;
	float  Roll_1;
	
}	sys_status_t;

/*�û�������µ����ݿ����������*/
typedef struct
{
	//----�������Ӹ���λ���ٶȼ��ٶ�
	float  C_X_L;
	float  C_V_L;
	
	float  C_X_R;
	float  C_V_R;

	float  M_X_L;
	float  M_V_L;
	
	float  M_X_R;
	float  M_V_R;
	
	//----�������������λ��λ���ٶ�
	uint16_t L_M_Pos; //ֻ�ò���ֵ��Ŀ������	
	uint16_t R_M_Pos; //ֻ�ò���ֵ��Ŀ������	
	
	//----YAW���ԭʼ�Ƕ�,�涨����ĽǶ�
	uint16_t Y_Raw_Pos;
	uint16_t Y_Standard_Pos;
	
} device_info_t;


typedef struct
{
	sys_status_t sys_tar;
	sys_status_t sys_meas;
	sys_status_t sys_err;
	
} sys_status_pack_info_t;

#endif
