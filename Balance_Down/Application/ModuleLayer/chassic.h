#ifndef __CHASSIC_H
#define __CHASSIC_H

#include "device.h"
#include "algo.h"
/* Exported macro ------------------------------------------------------------*/

//以下为底盘属性
#define MEC_ANGLE          -2.0f		   					//整车的机械中值角度，现在随便给的值
#define WHEEL_RADIUS        0.1f          			//轮子半径      m单位

#define MOMENTUM_L_F_POS    1400              //动量块在前面的电机位置 //-----------随便给的
#define MOMENTUM_L_M_POS   	7840  						//动量块在中间的电机位置
#define MOMENTUM_L_B_POS   	3280							//动量块在后面的电机位置

#define MOMENTUM_R_F_POS    6890              //动量块在前面的电机位置 //-----------随便给的
#define MOMENTUM_R_M_POS   	1110  						//动量块在中间的电机位置
#define MOMENTUM_R_B_POS   	5278							//动量块在后面的电机位置




//以下为底盘限制的参数
#define X_MAX_SPEED      2.8f          //整车最大的线速度
#define Z_MAX_SPEED      2.1f          //整车最大的旋转速度


/* Exported types ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
 *	@brief	LQR
 */
 
 /*	@note  k1* 车的位移		 k2* 车的线速度	  
					 k3* 车身角度    k4* 车的角速度       
           k5* 偏航角	     k6* 偏航角速度			*/
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
 *	@brief	底盘平衡三环开关
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
 *	@brief	底盘状态变量测量值，目标值，误差值
 */
typedef struct 
{
//测量值	
	float  along_X_distance_meas;       //沿着X轴走过的位移
	float  along_X_speed_meas;		   //沿着X轴的线速度
	
	float  pitch_meas;            
	float  rate_pitch_meas;   
	
	float  yaw_meas;
	float  rate_yaw_meas;
	
	
//目标值，目前只需要线速度目标值和yaw轴的角速度目标值
	float X_speed_tar;           //沿着X轴的线速度目标值
	float X_speed_max_tar;       
	
	float pitch_tar;             //
	float rate_pitch_tar;        
	
	float yaw_tar;               //
	float rate_yaw_tar;
	float rate_yaw_max_tar;
	
	
//误差值，目前只需两个
	float X_speed_err;
	
	float rate_yaw_err;
	
} status_variables_t;

/**
 *	@brief	扭矩数据
 *	@class	LQR
 */
typedef struct {
	float Speed_Torq;						//速度力矩	
	float Balance_Torq;					//输出平衡力矩	
	float Spin_Torq;						//输出转向力矩

	float Last_Speed_Torq;      //上一次的速度力矩
	float Last_Spin_Torq;       //上一次的转向扭矩
		
	float Max_Speed_Torq;       //最大的速度力矩
	float Max_Balance_Torq;     //最大的平衡力矩
	float Max_Spin_Torq;        //最大的转向力矩
	
	float LEG_L_Sum_Torq;       //左轮力矩之和
	float LEG_R_Sum_Torq;       //右轮力矩之和
		
	float LEG_L_Current;        //左轮的扭矩电流
	float LEG_R_Current;        //右轮的扭矩电流
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
	
	void                (*ctrl)(void);		//总控制
	
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
