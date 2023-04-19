#include "Master_Comm.h"

//----------------------------------�ϲ㺯��
void Master_Ctrl(void);
static void Master_Comm_Heart_Beat(Master_Comm_Info_t *self);
void Master_Rx_Info_Buff(uint8_t *rxBuf); //�ڴ���1�жϻص�������

//-----------------------------------�麯���ӿ�
__weak void Comm_Judge_Is_Data_Err_Handler(char flag);
__weak void Comm_RC_CH_Not_True_Handler(void);
__weak void Comm_UP_RC_Flag_Handler(const uint8_t status);
__weak void Comm_UP_Chassic_Flag_Handler(const uint8_t status);
__weak void Comm_RC_Offline_Handler(void);
__weak void Comm_Fail_Handler(void);

//------------------------------------��������
void Master_Info_UP(void);
void Master_Tx_Info_Buff(void);
void Not_Link_RC_CH_UP(void); 
void Not_Link_RC_Flag_UP(void);



//------------------------------------��������
void Master_Rx_Info_UP(uint8_t *rxbuf);
void Master_Tx_Info_UP(void);




//* Private variables ---------------------------------------------------------*/
extern UART_HandleTypeDef huart1;

Master_Comm_Info_t  Master = 
{
	.comm_way = UART1_COMM,
	
	.offline_max_cnt = 100,
	
	
	.ctrl     	= Master_Ctrl,
	.rx         = Master_Rx_Info_Buff,
	.heart_beat = Master_Comm_Heart_Beat,
};






void Master_Ctrl(void)
{
	
	Master_Info_UP();
	
	
}

//����Ҫ���͵�ԭʼ����,����ң����ͨ��,���������ط����ı�־λ
void Master_Info_UP(void)
{
	Master_Tx_Info_Buff();
	
	//��������ؽ���ң����,��ô�����ط�����ң����ͨ������رո���
	#ifdef NO_LINK_TO_RECEIVE
	{
		Not_Link_RC_CH_UP();
	}
	#endif
	

	Not_Link_RC_Flag_UP();
	
}


//---------------------------------------------ͨѶ�շ�����

//����ԭʼ����,д���ṹ����
void Master_Rx_Info_Buff(uint8_t *rxBuf)
{
	if( rxBuf == NULL )
		return;
	
	Master_Rx_Info_UP(rxBuf);
	
	
}

//���ʹ���
void Master_Tx_Info_Buff(void)
{
	Master_Tx_Info_UP();
	
	if( __HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) )
	{
		__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_TC);
		HAL_UART_Transmit_DMA(&huart1, Master.tx_buf, MASTER_TX_PACK_LENGTH);
	}
		
}

//У��+д�뵽���սṹ����
void Master_Rx_Info_UP(uint8_t *rxBuf)
{
	uint8_t  frame_length_sum = 0;							 //ԭʼ���ݰ����ܳ���
	uint8_t  frame_head_length = LEN_FRAME_HEAD; //ԭʼ���ݰ���֡ͷ����
	uint8_t  data_length; 
	uint8_t  frame_end_length  = LEN_FRAME_END;  //ԭʼ���ݰ���֡β����
		
	/* SOF���� */
	if(rxBuf[RX_SOF] == MASTER_COMM_FRAME_HEADER) 
	{
		
		//ԭʼ���ݰ������ݶγ���
		data_length	= rxBuf[RX_DATA_LEN];

		/* ֡ͷCRC8У�� */
		if(Verify_CRC8_Check_Sum(rxBuf, LEN_FRAME_HEAD) == true) 
		{
			/* ͳ��һ֡�������ݳ��ȣ�����CRC16У�� */    
			frame_length_sum = frame_head_length + data_length + frame_end_length;

			//CRC16У��
			if(Verify_CRC16_Check_Sum(rxBuf, frame_length_sum) == true)	
			{
				//���浽���սṹ����,frame_length_sum�൱���������ݰ����ֽڸ�����
				memcpy(&Master.rx_pack, rxBuf, frame_length_sum );
				
				//���ʧ������ֵ
				Master.offline_cnt = 0;		

				Comm_Judge_Is_Data_Err_Handler(COMM_OK);
			}
			//CRC16����
			else	
				Comm_Judge_Is_Data_Err_Handler(CRC16_ERR);	
		}		
		
		//CRC8����
		else
			Comm_Judge_Is_Data_Err_Handler(CRC8_ERR);
		
	}
	//֡ͷ����
	else
		Comm_Judge_Is_Data_Err_Handler(FRAME_HEADER_ERR);
	
	
	/* ֡βCRC16��һ�ֽ��Ƿ�Ϊ0xA5 */
	if(rxBuf[frame_length_sum] == MASTER_COMM_FRAME_HEADER)
	{
		/* ���һ�����ݰ������˶�֡���ݾ��ٴζ�ȡ */
		Master_Rx_Info_UP( &rxBuf[frame_length_sum] );
		
	}
	
	
}

//���ݰ���д�뵽�ṹ��,��д��������
void Master_Tx_Info_UP(void)
{
	uint8_t robot_id = judge_info.game_robot_status.robot_id;
	
	//----------------------------֡ͷ����-------------------------------------
	Master.tx_pack.std_frame_tx_header_t.SOF = MASTER_COMM_FRAME_HEADER;
	Master.tx_pack.std_frame_tx_header_t.data_length = MASTER_TX_PACK_LENGTH;

	
	
	//-----------------------------���ݶ�-----------------------------------------
	if(JUDGE_ONLINE)
	{
		Master.tx_pack.std_frame_tx_data_t.shooter_cooling_limit = judge_info.game_robot_status.shooter_id1_17mm_cooling_limit;
		Master.tx_pack.std_frame_tx_data_t.shooter_speed_limit = judge_info.game_robot_status.shooter_id1_17mm_speed_limit;
		Master.tx_pack.std_frame_tx_data_t.shooter_cooling_heat = judge_info.power_heat_data.shooter_id1_17mm_cooling_heat;
		Master.tx_pack.std_frame_tx_data_t.bullet_speed = judge_info.shoot_data.bullet_speed;
	}
	else
	{
		Master.tx_pack.std_frame_tx_data_t.shooter_cooling_limit = 120;
		Master.tx_pack.std_frame_tx_data_t.shooter_speed_limit = 0;
		Master.tx_pack.std_frame_tx_data_t.shooter_cooling_heat = 0;
		Master.tx_pack.std_frame_tx_data_t.bullet_speed = -1.f;
	}
	
	if(robot_id >=3 && robot_id <= 5) //��ɫ����3,4,5
		Master.tx_pack.std_frame_tx_data_t.robot_id = 0;
	else if(robot_id >=103 && robot_id <=105) //��ɫ����3,4,5
		Master.tx_pack.std_frame_tx_data_t.robot_id = 1;
	else //���������,Ĭ��Ϊ��ɫ
		Master.tx_pack.std_frame_tx_data_t.robot_id = 0;
	
	//----------------------------CRC����--------------------------------------
			//Ϊ�˼���CRC
	memcpy(Master.tx_buf, &Master.tx_pack, MASTER_TX_PACK_LENGTH );
	
			//CRC8��֡ͷ
	Append_CRC8_Check_Sum(Master.tx_buf, LEN_FRAME_HEAD);
																															  
			//CRC16���ܳ���
	Append_CRC16_Check_Sum(Master.tx_buf, MASTER_TX_PACK_LENGTH);	
																														    																									  
	memcpy(&Master.tx_pack, Master.tx_buf, MASTER_TX_PACK_LENGTH);
}

//---------------------------------------------���յ������ݽ���ϸ������




//���ݰ��ͨѶ������������State���ң����,���̱�־λ
void Not_Link_RC_Flag_UP(void)
{
	Comm_UP_RC_Flag_Handler(Master.rx_pack.std_frame_rx_data_t.status);
	
	if(RC_ONLINE)
		Comm_UP_Chassic_Flag_Handler(Master.rx_pack.std_frame_rx_data_t.status);	
	
}
//���ݰ��ͨѶ������������RC_sensor���ͨ��ֵ
void Not_Link_RC_CH_UP(void)
{
	//RC_LEFT_CH_LR_VALUE
	rc_sensor_info.ch3 = Master.rx_pack.std_frame_rx_data_t.rc_ch_ws_val;
	
	//RC_LEFT_CH_UD_VALUE
	rc_sensor_info.ch2 = Master.rx_pack.std_frame_rx_data_t.rc_ch_ad_val;
	
	if(State.rc_flag.ch_wise == CLOSE)
		rc_sensor_info.ch2 = rc_sensor_info.ch3 = 0;
	
	//ң����ͨ��ֵ������
	if( abs(rc_sensor_info.ch2) > CH_MAX || abs(rc_sensor_info.ch3) > CH_MAX )
	{
		rc_sensor_info.ch2 = rc_sensor_info.ch3 = 0;
		
		Comm_RC_CH_Not_True_Handler();
	}
		

}








//-----------------------------------------------��⺯��


/**
 *	@brief	ͨѶ�������������ͨѶ�Ƿ�ɹ�
 */
static void Master_Comm_Heart_Beat(struct Master_Comm_Info *self)
{
	self->offline_cnt ++;
	if(self->offline_cnt > self->offline_max_cnt) {
		self->offline_cnt = self->offline_max_cnt + 1;
		
		self->work_state = DEV_OFFLINE;
		
		State.comm_flag.state_flag = COMM_FAIL;
	} 
	else {
		/* ����->���� */
		if(self->work_state == DEV_OFFLINE)
		{
			self->work_state = DEV_ONLINE;
		}
	}
	
	
}

//------------------------------------------------����State����ر�־λ



//�������ط������ݲ����м��
__weak void Comm_Judge_Is_Data_Err_Handler(char flag)
{
	
}

//�����ط���ң����ͨ��ֵ����Ҫ������,�����˱�־λ
__weak void Comm_RC_CH_Not_True_Handler(void)
{
	
}
//����ң�����ı�־λ
__weak void Comm_UP_RC_Flag_Handler(const uint8_t status)
{

}
//���µ��̵��˶���־λ
__weak void Comm_UP_Chassic_Flag_Handler(const uint8_t status)
{
	
}

//ң����ʧ�������ң�����ı�־λ�͵��̵ı�־λ
__weak void Comm_RC_Offline_Handler(void)
{

}

//�����ͨѶʧ��,���������ز��ӽ��ջ�,��������б�־λ;����,ֻ�ı����ң��������״̬�ı�־λ
__weak void Comm_Fail_Handler(void)
{
	
}

