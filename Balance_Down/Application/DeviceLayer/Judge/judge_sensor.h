#ifndef __JUDGE_SENSOR_H
#define __JUDGE_SENSOR_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"                  // Device header
#include "rp_config.h"

#define JUDGE_ONLINE   judge_sensor.work_state == DEV_ONLINE
#define JUDGE_OFFLINE  judge_sensor.work_state == DEV_OFFLINE

#define CHAS_MAX_POWER_BUFFER  60

/*
����ϵͳ�������ǵ����ݣ�

1��֡ͷ------------std_frame_header_t(5���ֽ�)

-----֡ͷ��ƫ��---->judge_frame_header_offset_t

2��������------------CMD_ID��2���ֽڣ�



3�����ݶ�-----------�ӵ�7����ʼ�����Ȳ�����ȫ��ͬ
��ͬ��ָ���Ӧ��ͬ�����ݰ�



4��֡β------2���ֽ�





������ƫ��λ�ã�
judge_frame_offset_t--------------->���������ݰ���ƫ�Ʒ���

judge_frame_header_offset_t--------->��֡ͷ�ĵ���ƫ�Ʒ���
*/



typedef __packed struct
{
	uint8_t  sof;
	uint16_t data_length;
	uint8_t  seq;
	uint8_t  crc8;
} std_frame_header_t;

//֡�ֽڣ���λ/�ֽڣ�
typedef enum {//֡ͷռ����ֽڣ�������ռ�����ֽڣ��ӵ��߸���ʼΪ����֡
	FRAME_HEADER	= 0,
	CMD_ID			= 5,
	DATA_SEG		= 7
}judge_frame_offset_t;
//֡ͷ�ֽڣ���λ/bit��
typedef enum  {//֡ͷ����ϸλ��
	SOF			= 0,
	DATA_LENGTH	= 1,
	SEQ			= 3,
	CRC8		= 4
}judge_frame_header_offset_t;//FRAME_HEADER

//LEN_CMD_ID
//LEN_FRAME_TAIL
/* ID: 0x0001	Byte: 	11	����״̬���� */
typedef __packed struct 
{ 
	uint8_t game_type     : 4;		// ��������
	uint8_t game_progress : 4;		// �����׶�
	uint16_t stage_remain_time;		// ��ǰ�׶�ʣ��ʱ��(��λ:s)
	uint64_t SyncTimeStamp;       //�����˽��յ���ָ��ľ�ȷ Unix ʱ�䣬�����ض��յ���Ч�� NTP ��������ʱ����Ч
} ext_game_status_t; //LEN_GAME_STATUS   ��3-1
/* ID: 0x0002	Byte:	1	����������� */
typedef __packed struct
{
	uint8_t winner;
} ext_game_result_t; //LEN_GAME_RESULT   ��3-2
/* ID: 0x0003	Byte:	32	������Ѫ���������� */
typedef __packed struct 
{ 
	uint16_t red_1_robot_HP;	// ��1Ӣ�ۻ�����Ѫ��(δ�ϳ�������Ѫ��Ϊ0)
	uint16_t red_2_robot_HP;	// ��2���̻�����Ѫ��
	uint16_t red_3_robot_HP;	// ��3����������Ѫ��
	uint16_t red_4_robot_HP;	// ��4����������Ѫ��
	uint16_t red_5_robot_HP;	// ��5����������Ѫ��
	uint16_t red_7_robot_HP;	// ��7�ڱ�������Ѫ��
	uint16_t red_outpost_HP;	// �췽ǰ��վѪ��
	uint16_t red_base_HP;			// �췽����Ѫ��
	uint16_t blue_1_robot_HP;	// ��1Ӣ�ۻ�����Ѫ��
	uint16_t blue_2_robot_HP;	// ��2���̻�����Ѫ��
	uint16_t blue_3_robot_HP;	// ��3����������Ѫ��
	uint16_t blue_4_robot_HP;	// ��4����������Ѫ��
	uint16_t blue_5_robot_HP;	// ��5����������Ѫ��
	uint16_t blue_7_robot_HP;	// ��7�ڱ�������Ѫ��
	uint16_t blue_outpost_HP;	// ����ǰ��վѪ��
	uint16_t blue_base_HP;		// ��������Ѫ��	
} ext_game_robot_HP_t; //LEN_GAME_ROBOT_HP  ��3-3
///* ID: 0x0004 	Byte:	3	���ڷ���״̬ */
//typedef __packed struct
//{
//	uint8_t dart_belong;
//	uint16_t stage_remaining_time;
//} ext_dart_status_t;//LEN_DART_STATUS   ��3-4
/* ID: 0x0005 	Byte:	11	�˹�������ս���ӳ���ͷ���״̬ */
typedef __packed struct
{
	uint8_t F1_zone_status:1;            //����״̬
	uint8_t F1_zone_buff_debuff_status:3;//�췽��Ѫ��
	uint8_t F2_zone_status:1;
	uint8_t F2_zone_buff_debuff_status:3;//�췽��ҩ������
	uint8_t F3_zone_status:1;
	uint8_t F3_zone_buff_debuff_status:3;//������Ѫ��
	uint8_t F4_zone_status:1;
	uint8_t F4_zone_buff_debuff_status:3;//������ҩ������
	uint8_t F5_zone_status:1;
	uint8_t F5_zone_buff_debuff_status:3;//��ֹ�����
	uint8_t F6_zone_status:1;
	uint8_t F6_zone_buff_debuff_status:3;//��ֹ�ƶ���
	uint16_t red1_bullet_left;           //�췽 1 ��ʣ�൯��
	uint16_t red2_bullet_left;           //�췽 2 ��ʣ�൯��
	uint16_t blue1_bullet_left;          //���� 1 ��ʣ�൯��
	uint16_t blue2_bullet_left;          //���� 2 ��ʣ�൯��
} ext_ICRA_buff_debuff_zone_status_t;//LEN_ICRA_BUFF_DEBUFF_ZONE_STATUS  ��3-5
/* ID: 0x0101  Byte:  4    �����¼����� */
typedef __packed struct 
{ 
	uint32_t event_type;//��������վ��Ѫ��ռ��״̬,��������״̬, ���θߵ�ռ��״̬,�������ػ���״̬,����ǰ��ս״̬
} ext_event_data_t; //LEN_EVENT_DATA   ��3-6
/* ID: 0x0102  Byte:  4    ����վ������ʶ���� */
typedef __packed struct 
{ 
	uint8_t supply_projectile_id;  //����վ�� ID
	uint8_t supply_robot_id;       //���������� ID
	uint8_t supply_projectile_step;//�����ڿ���״̬
	uint8_t supply_projectile_num; //��������
} ext_supply_projectile_action_t;//LEN_SUPPLY_PROJECTILE_ACTION  ��3-7
//LEN_SUPPLY_PROJECTILE_BOOKING
/* ID: 0X0104  Byte:  2	   ���о�����Ϣ */
typedef __packed struct
{
	uint8_t level;        //����ȼ�
	uint8_t foul_robot_id;//��������� ID
} ext_referee_warning_t;//LEN_REFEREE_WARNING   ��3-8
/* ID: 0X0105  Byte:  1	   ���ڷ���ڵ���ʱ */
typedef __packed struct
{
	uint8_t dart_remaining_time;//15s ����ʱ
} ext_dart_remaining_time_t;//LEN_DART_REMAINING_TIME
/* ID: 0X0201  Byte: 27    ������״̬���� */
typedef __packed struct 
{ 
	uint8_t robot_id;                       //�������� ID
	uint8_t robot_level;                    //�����˵ȼ�
	uint16_t remain_HP;                     //������ʣ��Ѫ��
	uint16_t max_HP;                        //����������Ѫ��
	uint16_t shooter_id1_17mm_cooling_rate; //������ 1 �� 17mm ǹ��ÿ����ȴֵ
	uint16_t shooter_id1_17mm_cooling_limit;//������ 1 �� 17mm ǹ����������
	uint16_t shooter_id1_17mm_speed_limit;  //������ 1 �� 17mm ǹ�������ٶ� ��λ m/s
	uint16_t shooter_id2_17mm_cooling_rate; //������ 2 �� 17mm ǹ��ÿ����ȴֵ
	uint16_t shooter_id2_17mm_cooling_limit;//������ 2 �� 17mm ǹ����������
	uint16_t shooter_id2_17mm_speed_limit;  //������ 2 �� 17mm ǹ�������ٶ� ��λ m/s
	uint16_t shooter_id1_42mm_cooling_rate; //������ 42mm ǹ��ÿ����ȴֵ
	uint16_t shooter_id1_42mm_cooling_limit;//������ 42mm ǹ����������
	uint16_t shooter_id1_42mm_speed_limit;  //������ 42mm ǹ�������ٶ� ��λ m/s
	uint16_t chassis_power_limit;           //�����˵��̹�����������
	//���ص�Դ������
	uint8_t mains_power_gimbal_output  : 1; //gimbal ������� 1 Ϊ�� 24V �����0 Ϊ�� 24v ���
	uint8_t mains_power_chassis_output : 1; //chassis �������1 Ϊ�� 24V �����0 Ϊ�� 24v ���
	uint8_t mains_power_shooter_output : 1; //shooter �������1 Ϊ�� 24V �����0 Ϊ�� 24v ���
} ext_game_robot_status_t; //LEN_GAME_ROBOT_STATUS  ��3-10
/* ID: 0X0202  Byte: 16    ʵʱ������������ */
typedef __packed struct 
{ 
	uint16_t chassis_volt;   				// ���������ѹ����λ��mV
	uint16_t chassis_current;				// ���������������λ��mA
	float chassis_power;   					// ����˲ʱ���ʣ���λ��W
	uint16_t chassis_power_buffer;	// ���̹��ʻ��壬��λ��60J������������(���¸��ݹ���������250J)
	uint16_t shooter_id1_17mm_cooling_heat; //1 �� 17mm ǹ������
	uint16_t shooter_id2_17mm_cooling_heat; //2 �� 17mm ǹ������
	uint16_t shooter_id1_42mm_cooling_heat;	//42mm ǹ������
} ext_power_heat_data_t; //LEN_POWER_HEAT_DATA   ��3-11
/* ID: 0x0203  Byte: 16    ������λ������ */
typedef __packed struct 
{   
	float x;   
	float y;   
	float z;   
	float yaw; //λ��ǹ��,��λ����
} ext_game_robot_pos_t; //LEN_GAME_ROBOT_POS   ��3-12
/* ID: 0x0204  Byte:  1    �������������� */
typedef __packed struct 
{ 
	uint8_t power_rune_buff; 
} ext_buff_t; //LEN_BUFF_MASK  ��3-13
/* ID: 0x0205  Byte:  1    ���л���������״̬���� */
typedef __packed struct 
{ 
	uint8_t attack_time;//�ɹ���ʱ�� ��λ s��30s �ݼ��� 0
} ext_aerial_robot_energy_t; //LEN_AERIAL_ROBOT_ENERGY  ��3-14
/* ID: 0x0206  Byte:  1    �˺�״̬���� */
typedef __packed struct 
{ 
	uint8_t armor_id : 4; 	// װ���˺�ʱ����װ��ID
	uint8_t hurt_type : 4; 	// 0x0װ���˺� 0x1ģ����� 0x2������ 0x3������ 0x4������ 0x5ײ��
} ext_robot_hurt_t; //LEN_ROBOT_HURT  ��3-15
/* ID: 0x0207  Byte:  7    ʵʱ������� */
typedef __packed struct 
{ 
	uint8_t bullet_type; 	// �ӵ�����(1-17mm, 2-42mm)
	uint8_t shooter_id;   // ������� ID��1��1 �� 17mm ���������2��2 �� 17mm ���������3��42mm ���������
	uint8_t bullet_freq;  // �ӵ���Ƶ(Hz)
	float bullet_speed;		// �ӵ�����(m/s)
} ext_shoot_data_t; //LEN_SHOOT_DATA  ��3-16
/* ID: 0x0208  Byte:  6    �ӵ�ʣ�෢�������� */
typedef __packed struct
{
	uint16_t bullet_remaining_num_17mm;//17mm �ӵ�ʣ�෢����Ŀ
	uint16_t bullet_remaining_num_42mm;//42mm �ӵ�ʣ�෢����Ŀ
	uint16_t coin_remaining_num;       //ʣ��������
} ext_bullet_remaining_t; //LEN_BULLET_REMAINING   ��3-17
/* ID: 0x0209  Byte:  4 	������RFID״̬ */
typedef __packed struct
{
	uint32_t rfid_status;
} ext_rfid_status_t;//LEN_RFID_STATUS  ��3-18
/*ID��0x020A   Byte��6  ���ڻ����˿ͻ���ָ������*/
typedef struct{
	uint8_t dart_launch_opening_status;//��ǰ���ڷ���ڵ�״̬
	uint8_t dart_attack_target;        //���ڵĴ��Ŀ�꣬Ĭ��Ϊǰ��վ��1��ǰ��վ��2�����أ�
	uint16_t target_change_time;       //�л����Ŀ��ʱ�ı���ʣ��ʱ��
	uint16_t operate_launch_cmd_time;  //���һ�β�����ȷ������ָ��ʱ�ı���ʣ��ʱ�䣬��λ��
} ext_dart_client_cmd_t; //LEN_DART_CLIENT_DIRECTIVE  ��3-19

/*
	������ ID��
	1��Ӣ��(��)��
	2������(��)��
	3/4/5������(��)��
	6������(��)��
	7���ڱ�(��)��
	9���״�죩
	101��Ӣ��(��)��
	102������(��)��
	103/104/105������(��)��
	106������(��)��
	107���ڱ�(��)��
	109���״����

	�ͻ��� ID�� 
	0x0101 ΪӢ�۲����ֿͻ���(��) ��
	0x0102 Ϊ���̲����ֿͻ���( �� )��
	0x0103/0x0104/0x0105 Ϊ���������ֿͻ���(��)��
	0x0106 Ϊ���в����ֿͻ���((��)�� 

	0x0165��Ӣ�۲����ֿͻ���(��)��
	0x0166�����̲����ֿͻ���(��)��
	0x0167/0x0168/0x0169�����������ֿͻ���(��)��
	0x016A�����в����ֿͻ���(��)�� 
*/
enum judge_robot_ID{
	hero_red       = 1,
	engineer_red   = 2,
	infantry3_red  = 3,
	infantry4_red  = 4,
	infantry5_red  = 5,
	plane_red      = 6,
	
	hero_blue      = 101,
	engineer_blue  = 102,
	infantry3_blue = 103,
	infantry4_blue = 104,
	infantry5_blue = 105,
	plane_blue     = 106,
};
typedef struct{
	uint16_t teammate_hero;
	uint16_t teammate_engineer;
	uint16_t teammate_infantry3;
	uint16_t teammate_infantry4;
	uint16_t teammate_infantry5;
	uint16_t teammate_plane;
	uint16_t teammate_sentry;
	
	uint16_t client_hero;
	uint16_t client_engineer;
	uint16_t client_infantry3;
	uint16_t client_infantry4;
	uint16_t client_infantry5;
	uint16_t client_plane;
} ext_interact_id_t;

typedef __packed struct
{
	uint8_t find_enemy   :1;
	uint8_t find_hero    :1;
	uint8_t fine_infantry:2;//������Լӷ��ּ�������
	uint8_t stealed_tower:2;//��͵��
	uint8_t meaningless  :2;//����ӹ���
}ext_sentry_t;

typedef struct judge_info_struct {
	std_frame_header_t							fream_header;				// ֡ͷ��Ϣ
	
	ext_game_status_t 							game_status;				// 0x0001
	ext_game_result_t 							game_result;				// 0x0002
	ext_game_robot_HP_t 						game_robot_HP;			// 0x0003
//	ext_dart_status_t								dart_status;				// 0x0004
	ext_ICRA_buff_debuff_zone_status_t	ICRA_buff;
	
	ext_event_data_t								event_data;					// 0x0101
	ext_supply_projectile_action_t	supply_projectile_action;		// 0x0102
	//ext_supply_projectile_booking_t supply_projectile_booking;// 0x0103
	ext_referee_warning_t						referee_warning;		// 0x0104
	ext_dart_remaining_time_t				dart_remaining_time;// 0x0105
	
	ext_game_robot_status_t					game_robot_status;	// 0x0201
	ext_power_heat_data_t						power_heat_data;		// 0x0202
	ext_game_robot_pos_t						game_robot_pos;			// 0x0203
	ext_buff_t											buff;								// 0x0204
	ext_aerial_robot_energy_t				aerial_robot_energy;// 0x0205
	ext_robot_hurt_t								robot_hurt;					// 0x0206
	ext_shoot_data_t								shoot_data;					// 0x0207
	ext_bullet_remaining_t					bullet_remaining;		// 0x0208	
	ext_rfid_status_t								rfid_status;				// 0x0209	
	ext_dart_client_cmd_t           dart_client;        // 0x020A
	
	ext_interact_id_t								ids;								//�뱾�������Ļ�����id
	
	uint16_t                        self_client;        //�����ͻ���
	//????????????????????????????????????
	ext_sentry_t                    sentry_Info_rescue; //�ڱ��ṩ����Ϣ
	
	bool	 		data_valid;	// ������Ч��
	bool			err_cnt;
	uint16_t		offline_cnt;
	uint16_t		offline_max_cnt;	
} judge_info_t;



typedef struct judge_sensor_struct {
	judge_info_t		*info;
	drv_uart_t			*driver;
	void		(*init)(struct judge_sensor_struct *self);
	void		(*update)(struct judge_sensor_struct *self, uint8_t *rxBuf);
	void		(*check)(struct judge_sensor_struct *self);	
	void		(*heart_beat)(struct judge_sensor_struct *self);
	void		(*sendclient)(struct judge_sensor_struct *self);
	void		(*sendteammate)(struct judge_sensor_struct *self);
	dev_work_state_e	work_state;
	dev_errno_t				errno;
	dev_id_t					id;
} judge_sensor_t;




extern judge_info_t judge_info;
extern judge_sensor_t judge_sensor;

/* Exported functions --------------------------------------------------------*/

#endif
