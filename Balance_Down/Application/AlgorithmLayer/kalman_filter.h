#ifndef __kalman_filter_H
#define __kalman_filter_H

#include "stm32f4xx.h"                  // Device header
#include "imu_sensor.h"


#define OBSERVE_STATE_NUM 2						//״̬��������
#define dT                1.f 			  //������B����͸����������ֵ���õ�
#define Q_MATRIX_ELEMENT  0.0025f			//�����Ƿ������Ϊ��
#define R_MATRIX_ELEMENT  0.0001f				


#define Kalman_Filter_DeInit 		Kal_Filter.Init_Flag == DeInit


//��ʼ�������ñ�־	
typedef enum Filter_Init_e
{
	Init = 0,
	DeInit,
	
}Filter_Init_e;
		
//����ֵ
typedef struct Measure_Data
{
	
	short gyro_x;
	short gyro_y;
	
	short acc_x;
	short acc_y;
		
}Measure_Data;

//�������ֵ������Э�������
typedef struct Prior_Hat
{
	float gyro_x;
	float gyro_y;
	
	float Prior_Hat_Err_Cov[OBSERVE_STATE_NUM][OBSERVE_STATE_NUM];
	
}Prior_Hat;

//ϵͳ����
typedef struct System_Input
{
	short acc_x;
	short acc_y;
	
}System_Input;

//�������ֵ������Э�������
typedef struct Posterior_Hat
{
	float gyro_x;
	float gyro_y;
	
	float Posterior_Hat_Err_Cov[OBSERVE_STATE_NUM][OBSERVE_STATE_NUM];
	
}Posterior_Hat;

//����
typedef struct Matrix
{
	uint8_t State_Tran_A[OBSERVE_STATE_NUM][OBSERVE_STATE_NUM];
	
	uint8_t Ctrl_B[OBSERVE_STATE_NUM][OBSERVE_STATE_NUM];
	
	uint8_t Observed_H[OBSERVE_STATE_NUM][OBSERVE_STATE_NUM];
	
	uint8_t Unit_Matrix_I[OBSERVE_STATE_NUM][OBSERVE_STATE_NUM];
	
	float Process_Noise_Cov_Q[OBSERVE_STATE_NUM][OBSERVE_STATE_NUM];

	float Measure_Noise_Cov_R[OBSERVE_STATE_NUM][OBSERVE_STATE_NUM];
	
	float Kalman_Gain[OBSERVE_STATE_NUM][OBSERVE_STATE_NUM];
	
}Matrix;
//ʱ���¼��
typedef struct Time_Cnt
{
	uint32_t Time_Last;
	
	uint32_t Time_This;
	
	uint32_t Time_dT;
	
}Time_Cnt;

//�Ѻ������ֵ���浽����
typedef struct Kal_Filter_Pass_Data
{
	float gyro_x;
	float gyro_y;
	
}Kal_Filter_Pass_Data;
//�������˲���
typedef struct Kalman_Filter
{
	Measure_Data   				Meas_Data; //ʵ���в�õ�
	Prior_Hat 						Pri_Hat;
	System_Input					Sys_Input;
	Posterior_Hat   			Post_Hat;
	Matrix								All_Matrix;
	Time_Cnt        			Time;
	Filter_Init_e   			Init_Flag; 
	Kal_Filter_Pass_Data  KalPass_Data;
	
	
	
	void						(*Set_Initial)( struct Kalman_Filter *Kal_Filter );
	void						(*Data_Update)( struct Kalman_Filter *Kal_Filter );
	void						(*Predict)( struct Kalman_Filter *Kal_Filter );
	void						(*Correct)( struct Kalman_Filter *Kal_Filter );
	void            (*KalPass_Data_Update)( struct Kalman_Filter *Kal_Filter );
	
}Kalman_Filter;

extern Kalman_Filter Kal_Filter;

void Start_Kalman_Filter(void);

#endif
