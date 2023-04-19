#ifndef __UI_H
#define __UI_H

#include "stm32f4xx.h"                  // Device header
#include "device.h"
#include "judge_potocol.h"
#include "chassic.h"

#define INTERACT_DATA_MAX_LEN			113 //���ݽ�����,���ݶ������ֽڸ���Ϊ113
#define INTERACT_DATA_FIXED_LEN		13
/*  UI��������, 1920*1080  */
#define UI_mid_position_x 	960   //������������
#define UI_mid_position_y 	540   //������������

/*��ʱ�����*/
#define UI_Task_Os_Time   1
#define Draw_Total_Num    5
#define Fixed_Photo_Num   3
#define Not_Fixed_Photo_Num 	(Draw_Total_Num - Fixed_Photo_Num)
#define UI_Data_Current_Interval  200 //ms

typedef enum  //���ݶ�����ǰ6���ֽڵ�data_cmd_idö�ٶ���
{
	//0x200-0x02ff 	�����Զ������� ��ʽ  INTERACT_ID_XXXX
	INTERACT_ID_delete_graphic 			= 0x0100,	/*�ͻ���ɾ��ͼ��*/
	INTERACT_ID_draw_one_graphic 		= 0x0101,	/*�ͻ��˻���һ��ͼ��*/
	INTERACT_ID_draw_two_graphic 		= 0x0102,	/*�ͻ��˻���2��ͼ��*/
	INTERACT_ID_draw_five_graphic 	= 0x0103,	/*�ͻ��˻���5��ͼ��*/
	INTERACT_ID_draw_seven_graphic 	= 0x0104,	/*�ͻ��˻���7��ͼ��*/
	INTERACT_ID_draw_char_graphic 	= 0x0110,	/*�ͻ��˻����ַ�ͼ��*/
	INTERACT_ID_bigbome_num					= 0x02ff,
	
}ext_student_interactive_header_data_cmd_id_e;


typedef enum  //�ͻ���ɾ��ͼ��ָ��Ĳ�������
{
	NONE_delete    = 0,   /*�ղ���*/
	GRAPHIC_delete = 1,   /*ɾ��ͼ��*/
	ALL_delete     = 2,		/*ɾ������*/
	
}ext_client_custom_graphic_delete_type_e;


typedef enum   //ͼ����Ϣ���а�����������Ϣ
{
	NONE   = 0,		/*�ղ���*/
	ADD    = 1,		/*����ͼ��*/
	MODIFY = 2,		/*�޸�ͼ��*/
	DELETE = 3,		/*ɾ��ͼ��*/
	
}graphic_data_struct_operate_type_e;

typedef enum   //ͼ����Ϣ���а�����״����Ϣ
{
	LINE      = 0,	//ֱ��
	RECTANGLE = 1,	//����
	CIRCLE    = 2,	//��Բ
	OVAL      = 3,	//��Բ
	ARC       = 4,	//Բ��
	INT    	  = 5,	//������
	FLOAT     = 6,	//������
	CHAR      = 7, 	//�ַ�
	
}graphic_data_struct_graphic_type_e;


typedef enum    //ͼ����Ϣ���а�����ɫ����Ϣ
{
	RED_BLUE  = 0,	//������ɫ	
	YELLOW    = 1,
	GREEN     = 2,
	ORANGE    = 3,
	FUCHSIA   = 4,	/*�Ϻ�ɫ*/
	PINK      = 5,
	CYAN_BLUE = 6,	/*��ɫ*/
	BLACK     = 7,
	WHITE     = 8,
	
}graphic_data_struct_color_e;








typedef __packed struct //���ݶ�����ǰ6���ֽ�
{ 
	uint16_t data_cmd_id;    
	uint16_t send_ID;    
	uint16_t receiver_ID; 
	
}ext_student_interactive_data_header_t; 

typedef __packed struct //�ͻ���ɾ��ͼ��ָ��,CMD_ID��0x0100
{
	uint8_t operate_type; 
	uint8_t layer;//ͼ������0~9
}ext_client_custom_graphic_delete_t;

typedef __packed struct		//ͼ�����ݻ�����Ԫ
{                          
	uint8_t  graphic_name[3]; 
	uint32_t operate_type:3; 
	uint32_t graphic_type:3; //ֱ��  ����  ��Բ  ��Բ  Բ��  	����  							  ����  �ַ�
	uint32_t layer:4;        
	uint32_t color:4;        
	uint32_t start_angle:9;  //��    ��    ��    ��    �Ƕ�  	��С  								��С  ��С
	uint32_t end_angle:9;    //��    ��    ��    ��          	С������Ч����λ��    ��   ����
	uint32_t width:10;       
	uint32_t start_x:11;     //���  ���  Բ��  Բ��  Բ��  	���  								���  ���
	uint32_t start_y:11;     //-----------------(ͬ��һ��)---------------------------------
	uint32_t radius:10;      //��    ��    �뾶  ��    ��    	��    									��    ��
	uint32_t end_x:11;       //�յ�  �Զ�  ��    ����  ����  	��    									��    ��
	uint32_t end_y:11;       //-------(ͬ��һ��)-----------  	��    									��    ��
}graphic_data_struct_t;


typedef __packed struct    //�ַ����ݻ�����Ԫ
{
	graphic_data_struct_t 	grapic_info;
	uint8_t 								buf[30];
}ext_client_custom_character_t;


typedef __packed struct		//���������߸�������(ֻ����ͼ�����ݻ�����Ԫĩβ�����޸�)	
{                          
	uint8_t 	graphic_name[3]; 
	uint32_t  operate_type:3; 
	uint32_t 	graphic_type:3; 
	uint32_t 	layer:4;        
	uint32_t 	color:4;        
	uint32_t 	start_angle:9;  
	uint32_t 	end_angle:9;    
	uint32_t 	width:10;       
	uint32_t 	start_x:11;    
	uint32_t 	start_y:11;     
	
  int32_t 	number;       
} int32_or_float_data_struct_t;


//-------------------------------�����ÿ����Ϣ���ṹ-----------------------
/*������֮���ͨѶ���ݰ�*/
typedef __packed struct
{
	std_frame_header_t 				 							Tx_FrameHeader;			
	uint16_t  								 							CmdID;						
	
	ext_student_interactive_data_header_t   Data_FrameHeader;
	
	//���ݶξ������ݻ�û����!
	
	uint16_t															  CRC16;								
}ext_robot_interactive_data_t;

/*ɾ��ͼ�ε����ݰ�*/
typedef __packed struct
{
	std_frame_header_t 				 							Tx_FrameHeader;			
	uint16_t  								 							CmdID;						
	
	ext_student_interactive_data_header_t   Data_FrameHeader;
	ext_client_custom_graphic_delete_t 		  Client_Data;		
	
	uint16_t															  CRC16;				
}ext_DeleteLayer_Data_t;

/*����1��ͼ�ε����ݰ�*/
typedef __packed struct
{
	std_frame_header_t 											Tx_FrameHeader;			
	uint16_t  															CmdID;		
	
	ext_student_interactive_data_header_t   Data_FrameHeader;
	graphic_data_struct_t 									Client_Data;		
	
	uint16_t																CRC16;					
}ext_graphic_one_photo_t;

/*����2��ͼ�ε����ݰ�*/
typedef __packed struct
{
	std_frame_header_t 											Tx_FrameHeader;			
	uint16_t  															CmdID;		
	
	ext_student_interactive_data_header_t   Data_FrameHeader;
	graphic_data_struct_t 									Client_Data[2];		
	
	uint16_t																CRC16;					
}ext_graphic_two_photo_t;

/*����5��ͼ�ε����ݰ�*/
typedef __packed struct
{
	std_frame_header_t 											Tx_FrameHeader;			
	uint16_t  															CmdID;		
	
	ext_student_interactive_data_header_t   Data_FrameHeader;
	graphic_data_struct_t 									Client_Data[5];		
	
	uint16_t																CRC16;					
}ext_graphic_five_photo_t;

/*����7��ͼ�ε����ݰ�*/
typedef __packed struct
{
	std_frame_header_t 											Tx_FrameHeader;			
	uint16_t  															CmdID;		
	
	ext_student_interactive_data_header_t   Data_FrameHeader;
	graphic_data_struct_t 									Client_Data[7];		
	
	uint16_t																CRC16;					
}ext_graphic_seven_photo_t;

/*����1���ַ����򵥸��ַ������ݰ�*/
typedef __packed struct
{
	std_frame_header_t 											Tx_FrameHeader;			
	uint16_t  															CmdID;				
	
	ext_student_interactive_data_header_t   Data_FrameHeader;
	ext_client_custom_character_t 					Client_Data;
	
	uint16_t																CRC16;							
	
}ext_char_or_string_data_t;


/*����1���������ݻ��߸��������ݵ����ݰ�*/
typedef __packed struct
{
	std_frame_header_t 											Tx_FrameHeader;			
	uint16_t  															CmdID;				
	
	ext_student_interactive_data_header_t   Data_FrameHeader;
	int32_or_float_data_struct_t   					Client_Data;
	
	uint16_t																CRC16;							
	
}ext_int32_or_float_one_data_t;

/*����2���������ݻ��߸��������ݵ����ݰ�*/
typedef __packed struct
{
	std_frame_header_t 											Tx_FrameHeader;			
	uint16_t  															CmdID;				
	
	ext_student_interactive_data_header_t   Data_FrameHeader;
	int32_or_float_data_struct_t   					Client_Data[2];
	
	uint16_t																CRC16;							
	
}ext_int32_or_float_two_data_t;

/*����5���������ݻ��߸��������ݵ����ݰ�*/
typedef __packed struct
{
	std_frame_header_t 											Tx_FrameHeader;			
	uint16_t  															CmdID;				
	
	ext_student_interactive_data_header_t   Data_FrameHeader;
	int32_or_float_data_struct_t   					Client_Data[5];
	
	uint16_t																CRC16;							
	
}ext_int32_or_float_five_data_t;

/*����7���������ݻ��߸��������ݵ����ݰ�*/
typedef __packed struct
{
	std_frame_header_t 											Tx_FrameHeader;			
	uint16_t  															CmdID;				
	
	ext_student_interactive_data_header_t   Data_FrameHeader;
	int32_or_float_data_struct_t   					Client_Data[7];
	
	uint16_t																CRC16;							
	
}ext_int32_or_float_seven_data_t;



//------------------------------�û���˽�ж�������---------------------------------
typedef enum   //�û���ʾ���ݵ����������Ǵ�С
{
	/*ģʽ�Ƿ���=*/
	Close = 0,
	Open = 1,
	
	Not_Stuck = 0,
	Stuck = 1,
	
	/*������������ʱ����Ϣ*/
	Cap_Energy_Bar_Layer = 3,	//������������ͼ�β�
	Cap_Name_Size        = 20,
	Cap_Name_Width       = 2,
	Cap_Name_Color       = YELLOW,
	Cap_Name_X_Start 		 = 1800,
	Cap_Name_Y_Start 		 = 750,
	Cap_Energy_Bar_Color = RED_BLUE,
	Cap_Energy_Bar_Warn_Color = FUCHSIA,
	Cap_Energy_Bar_Width = 20, //�����Ŀ��
	Cap_Energy_Bar_X_Start = 1650, 
	Cap_Energy_Bar_X_End	 = 1900, 
	Cap_Energy_Bar_Y_Start = 700, 
	Cap_Energy_Bar_Y_End	 = 700, 
	
	/*����Բ�ε���Ϣ*/
	Chas_Circle_Layer   = 2,
	Chas_Circle_Width   = 3,	
	Chas_Circle_Color   = YELLOW,
	Chas_Circle_X       = 200, //Բ��
	Chas_Circle_Y       = 750,
	Chas_Circle_Radius  = 80,  
	
	/*���̳������Ϣ*/
	Chas_Dir_Angle_Layer = Chas_Circle_Layer,
	Chas_Dir_Angle_Width = 8,
	Chas_Dir_Color       = FUCHSIA,
	Chas_Dir_Angle_X     = Chas_Circle_X, //Բ��
	Chas_Dir_Angle_Y     = Chas_Circle_Y,
	Chas_Dir_Angle_End_X = Chas_Circle_Radius, 					//x����ĳ���
	Chas_Dir_Angle_End_Y = Chas_Circle_Radius,
	
	/*��̨�������Ϣ*/
	Gimb_Dir_Layer       = Chas_Circle_Layer,
	Gimb_Dir_Width       = 10,
	Gimb_Dir_Color       = FUCHSIA,
	Gimb_Dir_X_Start     = Chas_Circle_X,
	Gimb_Dir_Y_Start     = Chas_Circle_Y,
	Gimb_Dir_X_End       = Gimb_Dir_X_Start,
	Gimb_Dir_Y_End       = Gimb_Dir_Y_Start+Chas_Circle_Radius,
	
	
	/*������̬����Ϣ*/
	Chas_Data_Layer   	= 6,
	Chas_Data_Width   	= 2,
	Chas_Pitch_X_Mid    = 980,
	Chas_Pitch_Y_Mid    = 80,
	Chas_Pitch_Color    = YELLOW,
	Chas_Line_Half_Len  = 100, //������̬��һ��ֱ�߱�ʾ,�е㵽һ�˵ľ���
	Chas_Header_Size    = 20,
	Chas_Header_Width   = 5,
	Chas_Header_Color   = FUCHSIA,
	Chas_Header_Len     = 30, 	
	
	/*������λ�õ���Ϣ*/
	Mometum_Data_Layer   	= 4,
	Mometum_Data_Width   	= 2,
	Mometum_Data_Color    = YELLOW,
	Mometum_Circle_R      = 20, //������Բ��ͼ���İ뾶

	/*2����־λ�Ĺ̶��ַ����ƺͱ�־���1��0*/
	Fixed_Flag_Str_Layer   = 5,
	Fixed_Flag_Str_Size    = 20,
	Fixed_Flag_Str_Width   = 2,
	Fixed_Flag_Str_Color   = YELLOW,
	Fixed_Flag_Str_X_Start = 870,
	Fixed_Flag_Str_Y_Start = 800,
	Flag_X_Start           = Fixed_Flag_Str_X_Start,
	Flag_Y_Start           = Fixed_Flag_Str_Y_Start-50,
	
	/*�Ƿ���ֿ�ǽ����Ϣ*/
	Stuck_Data_Layer   = 0,
	Stuck_Data_Size    = 60,
	Stuck_Data_Width   = 5,
	Stuck_Data_Color   = GREEN,
	Stuck_Data_X_Start = 820,
	Stuck_Data_Y_Start = 800,
	

	/*������Ұ��ȵ���Ϣ*/
	Chas_Vision_Wide_Layer = 2,
	Chas_Vision_Wide_Color = PINK,
	Chas_Vision_Wide_Width = 3,
	//������������
	Chas_Vision_Wide_L_X_Start = 600,
	Chas_Vision_Wide_L_X_End = Chas_Vision_Wide_L_X_Start+200,
	Chas_Vision_Wide_L_Y_Start = 0,
	Chas_Vision_Wide_L_Y_End = 300,
	//�ҿ����������
	Chas_Vision_Wide_R_X_Start = Chas_Vision_Wide_L_X_Start+720,
	Chas_Vision_Wide_R_X_End = Chas_Vision_Wide_R_X_Start-200,
	Chas_Vision_Wide_R_Y_Start = Chas_Vision_Wide_L_Y_Start,
	Chas_Vision_Wide_R_Y_End = Chas_Vision_Wide_L_Y_End,
	//�Ͽ����������
	Chas_Vision_Length_H_X_Start = Chas_Vision_Wide_L_X_End,
	Chas_Vision_Length_H_X_End = Chas_Vision_Wide_R_X_End,
	Chas_Vision_Length_H_Y_Start = Chas_Vision_Wide_L_Y_End,
	Chas_Vision_Length_H_Y_End = Chas_Vision_Wide_L_Y_End,
	//�¿����������
	Chas_Vision_Length_L_X_Start = (Chas_Vision_Wide_L_X_Start+Chas_Vision_Wide_L_X_End)/2+10,
	Chas_Vision_Length_L_X_End = (Chas_Vision_Wide_R_X_Start+Chas_Vision_Wide_R_X_End)/2-10,
	Chas_Vision_Length_L_Y_Start = Chas_Vision_Wide_L_Y_End-130,
	Chas_Vision_Length_L_Y_End = Chas_Vision_Length_L_Y_Start,
	
	/*�ϲ�׼��*/
	Aim_Layer = 1,
	Aim_Color = GREEN,
	Aim_Width = 5,
	Aim_Short_Line_X_Start = 980,  //�̺��ߵ�X�������
	Aim_Short_Line_Length = 40, //�̺��ߵĳ���
	Aim_Top_Y_Base_Start = 600, //���ϲ��׼�ĺ���
	Aim_Y_Interval = 30, 						//����֮��ļ��
	
	
	
}User_Typedef_e;




typedef struct UI_Data_Info //Ҫ��ʾ�ľ�������
{
	float  cap_power_percent; //����ʣ�������ٷֱ�,����������24.5VʱΪ�������,����������ʽ��ʾ������
	
	float  chas_pitch; 	 //�������,��Ϊ��λ
	float  momentum_pos; //������λ��,��Ϊ��λ
	
	User_Typedef_e  small_top_state;  //С�����Ƿ���
	User_Typedef_e  magz_state;       //�����Ƿ���

	char  stuck_state;      //�Ƿ�ǽ
	
	float  chas_power_buffer_percent;    //����ʣ�๦�ʻ���ٷֱ�
	
	float  chas_dir_angle;  //���̳���ķ�λ��
	
	
}UI_Data_Info_t;

typedef struct UI_Operate   //ÿ�����ݵĲ�������
{
	/*�ַ������ݰ�*/	
	graphic_data_struct_operate_type_e    fixed_flag_str;
	graphic_data_struct_operate_type_e    flag; 
	graphic_data_struct_operate_type_e    stuck_str;
	
	/*ͼƬ���ݰ�*/
	graphic_data_struct_operate_type_e    fixed_photo;	
	graphic_data_struct_operate_type_e    not_fixed_photo;
	graphic_data_struct_operate_type_e    aim;
	
}UI_Operate_t;

typedef struct UI_UART_Tx_Pack //Ҫͨ�����ڷ��ͳ�ȥ�����ݰ�
{
	/*�ַ������ݰ�*/
	ext_char_or_string_data_t  fixed_flag_str; //���ΰ���small_top_data, magz_data, magz_data
	ext_char_or_string_data_t  flag;  //���ΰ���ʣ���ӵ���Ŀ,small_top_data, magz_data, magz_data
	
	ext_char_or_string_data_t  stuck_str; 
	
	/*ͼ�����ݰ�*/	
	ext_graphic_seven_photo_t  fixed_photo; //���ΰ���chas_circle,gimb_dir,chas_vision_wide,chas_vision_length��6��
	ext_graphic_five_photo_t   not_fixed_photo; //���ΰ���chas_gesture,momentum_gesture,chas_header,chas_dir,cap_data��5��	  
	ext_graphic_five_photo_t   aim; 
	
}UI_UART_Tx_Pack_t;

typedef struct UI_Data
{
	
	UI_Data_Info_t  	 data;
	UI_Operate_t			 operate;
	UI_UART_Tx_Pack_t  ext;
	
	uint8_t         robot_id;
	uint16_t        client_id;
	
	
	
}UI_Data_t;

void UI_Ctrl(void);

#endif
