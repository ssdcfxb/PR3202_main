#ifndef __CHASSIC_H
#define __CHASSIC_H

#include "device.h"

/* Exported macro ------------------------------------------------------------*/


#define WHEEL_RADIUS     0.1f          //轮子半径 m单位
/* Exported types ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
 *	@brief	LQR
 */
 
 /*	@note  k1* 车的位移		 k2* 车的线速度	  
		   k3* 车身角度      k4* 车的角速度       
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
 *	@brief	底盘状态变量
 */
typedef struct 
{
	float  along_X_distance;       //沿着X轴走过的位移
	float  along_X_speed;			     //沿着X轴的线速度
	
	float  pitch;            //
	float  rate_pitch;   
	
	float  yaw;
	float  rate_yaw;
	
} status_variables_t;

/**
 *	@brief	扭矩数据
 *	@class	LQR
 */
typedef struct {
	float Speed_Torq;			//速度力矩	
	float Balance_Torq;			//输出平衡力矩	
	float Spin_Torq;			//输出转向力矩

	float Last_Speed_Torq;      //上一次的速度力矩
	float Last_Spin_Torq;       //上一次的转向扭矩
	
	float LEG_L_Current;        //左轮的扭矩电流
	float LEG_R_Current;        //右轮的扭矩电流
	
	float Max_Speed_Torq;       //最大的速度力矩
	float Max_Balance_Torq;     //最大的平衡力矩
	float Max_Spin_Torq;        //最大的转向力矩
	
	float LEG_L_Sum_Torq;       //左轮力矩之和
	float LEG_R_Sum_Torq;       //右轮力矩之和
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
