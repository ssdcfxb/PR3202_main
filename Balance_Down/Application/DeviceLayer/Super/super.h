#ifndef __SUPER_H
#define __SUPER_H
#include "rp_config.h"
#include "algo.h"
#include "driver.h"
#include "judge_sensor.h"


/***************************************************************************/

#define  CAP_RX_ID         (0x30)
#define  CAP_OUTPUT_LIMIT  150
#define  CAP_INPUT_LIMIT   150
#define  CAP_MAX_VOLTAGE   (24.5f)


typedef struct
{
	//��0x2E��0x2F�������ݰ�����
	uint16_t chassis_power_buffer;  //���̹��ʻ��壬0~60J
	uint16_t chassis_power_limit;   //�����˵��̹����������ޣ�0~120W
	int16_t  output_power_limit;     //���ݷŵ繦�����ƣ�-120~300W
	uint16_t input_power_limit;     //���ݳ�繦�����ƣ�0~150W
	uint16_t chassis_volt;			//���������ѹ ��λ ���� **
	uint16_t chassis_current;		//����������� ��λ ���� **

	union{
				uint16_t all;
				struct
				{
						uint16_t cap_switch : 1;    //���ݿ���
						uint16_t cap_record : 1;    //��¼���ܿ���
						uint16_t gamegoing : 1;     //����������Ϊ1������Ϊ0 ***						
				}bit;
		}cap_control;
		
}cap_send_data_t;

typedef struct
{
	float cap_Ucr;    //�������˵�ѹUcr��0~30V
	float cap_I;    //���ݵ���I��-20~20A
	union
	{
			uint16_t state;     //����״̬
			struct
			{
					uint16_t none : 1;   //��
					uint16_t cap_U_over : 1;    //���ݹ�ѹ
					uint16_t cap_I_over : 1;    //���ݹ���
					uint16_t cap_U_low : 1;     //����Ƿѹ
					uint16_t bat_I_low : 1;     //����ϵͳǷѹ
					uint16_t can_receive_miss : 1;    //����δ���յ�CANͨ������
			}bit;
	}cap_state;
}cap_receive_data_t;

typedef struct CAP_RP_struct 
{
  cap_send_data_t      TX;
  cap_receive_data_t   RX;
	
  uint16_t		       offline_cnt;
  uint16_t		       offline_max_cnt;	
  dev_work_state_e	 work_state; 


  void    (*ctrl)(void);	
  void		(*heart_beat)(struct CAP_RP_struct *self);	

} Super_t;


void set_message(void);
void cap_send_0x2E(char can);
void cap_send_0x2F(char can);



extern Super_t Super_2023;


#endif


