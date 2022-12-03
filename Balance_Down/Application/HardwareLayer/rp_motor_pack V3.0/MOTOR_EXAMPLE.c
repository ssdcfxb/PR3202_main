/*
* EXAMPLE
*	RP MOTOR_BAG
*	2022.11.6 
* AUTHOR CCB HXZP
*
*使用电机前必须配置好电机的类型、id号、驱动方式，以及init指针函数初始化
*使用pid前也需要进行pid初始化，否则无法使用
*可以直接使用内置函数进行pid控制 也可以直接使用库内定义函数或者自己定义的函数
*kt系列电机：使用kt系列电机前需要先实例化motor_9025_info_t，再与motor内的指针联系，其余步骤无异
*/

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#include "MOTOR_EXAMPLE.h"
#include "motor.h"
#include "drv_can.h"

void CAN1_rxDataHandler(uint32_t canId, uint8_t *rxBuf);
void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf);


/*--------------------------------------------------------------------------*/

/*该部分can驱动实际放在can中定义*/
/*该部分can驱动实际放在can中定义*/
/*该部分can驱动实际放在can中定义*/
/*该部分can驱动实际放在can中定义*/

/*
	接收数据帧定义 帧头+数据
*/
typedef struct {
	CAN_RxHeaderTypeDef header;
	uint8_t				data[8];
} CAN_RxFrameTypeDef;

/*
	can1 can2实例
*/
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

CAN_RxFrameTypeDef hcan1RxFrame;
CAN_RxFrameTypeDef hcan2RxFrame;

/*
	can使用8位发送，需要将16位转为8位
*/
void CAN1_Send_With_int16_to_uint8(uint32_t stdId, int16_t *dat)
{
	uint8_t data[8];
	
	data[0] = (uint8_t)((int16_t)dat[0] >> 8);
	data[1] = (uint8_t)((int16_t)dat[0]);
	data[2] = (uint8_t)((int16_t)dat[1] >> 8);
	data[3] = (uint8_t)((int16_t)dat[1]);
	data[4] = (uint8_t)((int16_t)dat[2] >> 8);
	data[5] = (uint8_t)((int16_t)dat[2]);
	data[6] = (uint8_t)((int16_t)dat[3] >> 8);
	data[7] = (uint8_t)((int16_t)dat[3]);			
	
	CAN_SendData(&hcan1,stdId,data);
	
}



/**
  * @brief  can接受中断，在stm32f4xx_hal_can.c内弱定义
  * @param  
  * @retval 
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  
  if(hcan == &hcan1)
  {
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hcan1RxFrame.header, hcan1RxFrame.data);
		
		CAN1_rxDataHandler(hcan1RxFrame.header.StdId, hcan1RxFrame.data);
  }
  else if(hcan == &hcan2)
  {
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hcan2RxFrame.header, hcan2RxFrame.data);
		
		CAN2_rxDataHandler(hcan2RxFrame.header.StdId, hcan2RxFrame.data);
  }
  else 
  {
    return;
  }
}

/*--------------------------------------------------------------------------*/




/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/
/*---------------------------以下为正文-------------------------------------*/


typedef enum
{
	CHAS_FL = 0,
	CHAS_FR,
	CHAS_BL,
	CHAS_BR,
	
	GIMB_Y,
	GIMB_P,
	GIMB_R,
	
	MOTOR_LIST,
}motor_list_e;


/*
* 使用电机前必须配置好电机的类型、id号、驱动方式
*/
motor_t motor[MOTOR_LIST] =
{
	[GIMB_Y] = {
	
		.id.drive_type = M_CAN1,
		.id.motor_type = GM6020,
		.id.rx_id = 0x205,
		
		.init = motor_class_init,
	}
};







/*--------------------------------------------------------------------------*/






/*
	需要定义一个数组作为传入参数，不使用结构体是因为数组初始化展开为一维的看起来比较简约，但是要记住他们的位置和含义
	float	  kp;
	float 	ki;
	float 	kd;
	
	float   blind_err;	
	float 	integral_max;	
	float   iout_max;
	float 	out_max;
*/
float gimb_y_speed_pid_param[7] = {5,0,0,0,0,15000,20000};

/*	
	发送数组
*/
int16_t send_buff[4];
float tar;





void StartControlTask(void const * argument)
{
	
	//电机初始化
	motor[GIMB_Y].init(&motor[GIMB_Y]);
	//电机速度pid初始化
	motor[GIMB_Y].pid_init(&motor[GIMB_Y].pid.speed,gimb_y_speed_pid_param);
	
  for(;;)
  {
		//电机心跳，用于判断是否失联
		motor[GIMB_Y].heartbeat(&motor[GIMB_Y]);

//		//pid计算
		send_buff[motor[GIMB_Y].id.buff_p] = motor[GIMB_Y].c_speed(&motor[GIMB_Y],tar);


//		//控制数据的发送
		CAN1_Send_With_int16_to_uint8(motor[GIMB_Y].id.tx_id,send_buff);
//		
//		
		
		
    osDelay(1);
  }

}





//外部单独文件使用 用于can的总中断接收入口
void CAN1_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
	//GIMB_Y电机，数据8个字节长度（实际此处字节长度没有作用，冗余参数）
	motor[GIMB_Y].rx(&motor[GIMB_Y],rxBuf,8);

}

void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{



}









