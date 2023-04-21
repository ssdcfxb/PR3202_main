#include "super.h"

void super_ctrl(void);
void super_heart_beat(struct CAP_RP_struct *self);


void set_message(void);
void cap_send_0x2E(char can);
void cap_send_0x2F(char can);


/******************************************************************************/
int16_t cap_data_0x2E[4];
int16_t cap_data_0x2F[4];


Super_t Super_2023 = {

  .work_state = DEV_OFFLINE,	
  .offline_max_cnt = 5000,
	
  .ctrl       = super_ctrl, 
  .heart_beat = super_heart_beat,

};

/******************************************************************************/

void super_ctrl(void)
{
	set_message();
	
	cap_send_0x2E(2);
	cap_send_0x2F(2);	

}

void super_heart_beat(struct CAP_RP_struct *self)
{

	self->offline_cnt++;
	if(self->offline_cnt > self->offline_max_cnt) 
	{
		self->offline_cnt = self->offline_max_cnt+1;
		self->work_state = DEV_OFFLINE;
	}
	else 
	{
		if(self->work_state == DEV_OFFLINE)
			 self->work_state =  DEV_ONLINE;
	}
}






/*----------------------------�볬�����ص�ͨѶ--------------------------------------------*/
void set_message(void)
{
	Super_2023.TX.chassis_power_buffer = 10;//judge_sensor.info->power_heat_data.chassis_power_buffer;
	Super_2023.TX.chassis_power_limit = judge_sensor.info->game_robot_status.chassis_power_limit;
	Super_2023.TX.chassis_volt = judge_sensor.info->power_heat_data.chassis_volt;	//**
	Super_2023.TX.chassis_current = judge_sensor.info->power_heat_data.chassis_current;	//**
	Super_2023.TX.output_power_limit = CAP_OUTPUT_LIMIT;
	Super_2023.TX.input_power_limit = CAP_INPUT_LIMIT;

	Super_2023.TX.cap_control.bit.cap_switch = 1;
	Super_2023.TX.cap_control.bit.cap_record = 1;

	if(judge_sensor.info->game_status.game_progress == 4)	//***
	{
		Super_2023.TX.cap_control.bit.gamegoing = 1;
	}
	else
	{
		Super_2023.TX.cap_control.bit.gamegoing = 0;
	}	
}

//����ϵͳ���͹��ʻ����Ƶ��Ϊ50Hz���ú�����Ҫÿ20ms����һ�Ρ�
void cap_send_0x2E(char can)
{
	cap_data_0x2E[0] = Super_2023.TX.chassis_power_buffer;    //���̹��ʻ��壬0~60J
	cap_data_0x2E[1] = Super_2023.TX.chassis_volt;    	//���������ѹ ��λ ���� **
	cap_data_0x2E[2] = Super_2023.TX.chassis_current;    //����������� ��λ ���� **

	if(can == 1)
		CAN1_Send_With_int16_to_uint8((uint32_t)(0x2E), cap_data_0x2E);
	
	else if(can == 2)
		CAN2_Send_With_int16_to_uint8((uint32_t)(0x2E), cap_data_0x2E);
}

void cap_send_0x2F(char can)
{
	cap_data_0x2F[0] = Super_2023.TX.chassis_power_limit;   //���̹����������ޣ�0~120W
	cap_data_0x2F[1] = Super_2023.TX.output_power_limit;    //���ݷŵ繦�����ƣ�-120~300W
	cap_data_0x2F[2] = Super_2023.TX.input_power_limit;    //���ݳ�繦�����ƣ�0~150W
	cap_data_0x2F[3] = Super_2023.TX.cap_control.all;            //���ݿ��أ�0���رգ���1��������

	if(can == 1)
		CAN1_Send_With_int16_to_uint8((uint32_t)(0x2F), cap_data_0x2F);
	
	else if(can == 2)
		CAN2_Send_With_int16_to_uint8((uint32_t)(0x2F), cap_data_0x2F);
}



