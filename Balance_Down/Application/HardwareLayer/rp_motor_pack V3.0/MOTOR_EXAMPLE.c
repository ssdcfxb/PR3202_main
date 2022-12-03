/*
* EXAMPLE
*	RP MOTOR_BAG
*	2022.11.6 
* AUTHOR CCB HXZP
*
*ʹ�õ��ǰ�������úõ�������͡�id�š�������ʽ���Լ�initָ�뺯����ʼ��
*ʹ��pidǰҲ��Ҫ����pid��ʼ���������޷�ʹ��
*����ֱ��ʹ�����ú�������pid���� Ҳ����ֱ��ʹ�ÿ��ڶ��庯�������Լ�����ĺ���
*ktϵ�е����ʹ��ktϵ�е��ǰ��Ҫ��ʵ����motor_9025_info_t������motor�ڵ�ָ����ϵ�����ಽ������
*/

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#include "MOTOR_EXAMPLE.h"
#include "motor.h"
#include "drv_can.h"

void CAN1_rxDataHandler(uint32_t canId, uint8_t *rxBuf);
void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf);


/*--------------------------------------------------------------------------*/

/*�ò���can����ʵ�ʷ���can�ж���*/
/*�ò���can����ʵ�ʷ���can�ж���*/
/*�ò���can����ʵ�ʷ���can�ж���*/
/*�ò���can����ʵ�ʷ���can�ж���*/

/*
	��������֡���� ֡ͷ+����
*/
typedef struct {
	CAN_RxHeaderTypeDef header;
	uint8_t				data[8];
} CAN_RxFrameTypeDef;

/*
	can1 can2ʵ��
*/
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

CAN_RxFrameTypeDef hcan1RxFrame;
CAN_RxFrameTypeDef hcan2RxFrame;

/*
	canʹ��8λ���ͣ���Ҫ��16λתΪ8λ
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
  * @brief  can�����жϣ���stm32f4xx_hal_can.c��������
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
/*---------------------------����Ϊ����-------------------------------------*/


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
* ʹ�õ��ǰ�������úõ�������͡�id�š�������ʽ
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
	��Ҫ����һ��������Ϊ�����������ʹ�ýṹ������Ϊ�����ʼ��չ��Ϊһά�Ŀ������Ƚϼ�Լ������Ҫ��ס���ǵ�λ�úͺ���
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
	��������
*/
int16_t send_buff[4];
float tar;





void StartControlTask(void const * argument)
{
	
	//�����ʼ��
	motor[GIMB_Y].init(&motor[GIMB_Y]);
	//����ٶ�pid��ʼ��
	motor[GIMB_Y].pid_init(&motor[GIMB_Y].pid.speed,gimb_y_speed_pid_param);
	
  for(;;)
  {
		//��������������ж��Ƿ�ʧ��
		motor[GIMB_Y].heartbeat(&motor[GIMB_Y]);

//		//pid����
		send_buff[motor[GIMB_Y].id.buff_p] = motor[GIMB_Y].c_speed(&motor[GIMB_Y],tar);


//		//�������ݵķ���
		CAN1_Send_With_int16_to_uint8(motor[GIMB_Y].id.tx_id,send_buff);
//		
//		
		
		
    osDelay(1);
  }

}





//�ⲿ�����ļ�ʹ�� ����can�����жϽ������
void CAN1_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
	//GIMB_Y���������8���ֽڳ��ȣ�ʵ�ʴ˴��ֽڳ���û�����ã����������
	motor[GIMB_Y].rx(&motor[GIMB_Y],rxBuf,8);

}

void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{



}









