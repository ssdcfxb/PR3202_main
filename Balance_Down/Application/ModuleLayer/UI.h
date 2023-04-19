#ifndef __UI_H
#define __UI_H

#include "stm32f4xx.h"                  // Device header
#include "device.h"
#include "judge_potocol.h"
#include "chassic.h"

#define INTERACT_DATA_MAX_LEN			113 //数据交互中,数据段最大的字节个数为113
#define INTERACT_DATA_FIXED_LEN		13
/*  UI界面中心, 1920*1080  */
#define UI_mid_position_x 	960   //从左往右增加
#define UI_mid_position_y 	540   //从下往上增加

/*跟时间相关*/
#define UI_Task_Os_Time   1
#define Draw_Total_Num    5
#define Fixed_Photo_Num   3
#define Not_Fixed_Photo_Num 	(Draw_Total_Num - Fixed_Photo_Num)
#define UI_Data_Current_Interval  200 //ms

typedef enum  //数据段内容前6个字节的data_cmd_id枚举定义
{
	//0x200-0x02ff 	队伍自定义命令 格式  INTERACT_ID_XXXX
	INTERACT_ID_delete_graphic 			= 0x0100,	/*客户端删除图形*/
	INTERACT_ID_draw_one_graphic 		= 0x0101,	/*客户端绘制一个图形*/
	INTERACT_ID_draw_two_graphic 		= 0x0102,	/*客户端绘制2个图形*/
	INTERACT_ID_draw_five_graphic 	= 0x0103,	/*客户端绘制5个图形*/
	INTERACT_ID_draw_seven_graphic 	= 0x0104,	/*客户端绘制7个图形*/
	INTERACT_ID_draw_char_graphic 	= 0x0110,	/*客户端绘制字符图形*/
	INTERACT_ID_bigbome_num					= 0x02ff,
	
}ext_student_interactive_header_data_cmd_id_e;


typedef enum  //客户端删除图形指令的操作类型
{
	NONE_delete    = 0,   /*空操作*/
	GRAPHIC_delete = 1,   /*删除图层*/
	ALL_delete     = 2,		/*删除所有*/
	
}ext_client_custom_graphic_delete_type_e;


typedef enum   //图形信息中有包含操作的信息
{
	NONE   = 0,		/*空操作*/
	ADD    = 1,		/*增加图层*/
	MODIFY = 2,		/*修改图层*/
	DELETE = 3,		/*删除图层*/
	
}graphic_data_struct_operate_type_e;

typedef enum   //图形信息中有包含形状的信息
{
	LINE      = 0,	//直线
	RECTANGLE = 1,	//矩形
	CIRCLE    = 2,	//正圆
	OVAL      = 3,	//椭圆
	ARC       = 4,	//圆弧
	INT    	  = 5,	//浮点数
	FLOAT     = 6,	//整型数
	CHAR      = 7, 	//字符
	
}graphic_data_struct_graphic_type_e;


typedef enum    //图形信息中有包含颜色的信息
{
	RED_BLUE  = 0,	//红蓝主色	
	YELLOW    = 1,
	GREEN     = 2,
	ORANGE    = 3,
	FUCHSIA   = 4,	/*紫红色*/
	PINK      = 5,
	CYAN_BLUE = 6,	/*青色*/
	BLACK     = 7,
	WHITE     = 8,
	
}graphic_data_struct_color_e;








typedef __packed struct //数据段内容前6个字节
{ 
	uint16_t data_cmd_id;    
	uint16_t send_ID;    
	uint16_t receiver_ID; 
	
}ext_student_interactive_data_header_t; 

typedef __packed struct //客户端删除图形指令,CMD_ID是0x0100
{
	uint8_t operate_type; 
	uint8_t layer;//图层数：0~9
}ext_client_custom_graphic_delete_t;

typedef __packed struct		//图形数据基础单元
{                          
	uint8_t  graphic_name[3]; 
	uint32_t operate_type:3; 
	uint32_t graphic_type:3; //直线  矩形  正圆  椭圆  圆弧  	浮点  							  整型  字符
	uint32_t layer:4;        
	uint32_t color:4;        
	uint32_t start_angle:9;  //空    空    空    空    角度  	大小  								大小  大小
	uint32_t end_angle:9;    //空    空    空    空          	小数点有效数字位数    空   长度
	uint32_t width:10;       
	uint32_t start_x:11;     //起点  起点  圆心  圆心  圆心  	起点  								起点  起点
	uint32_t start_y:11;     //-----------------(同上一行)---------------------------------
	uint32_t radius:10;      //空    空    半径  空    空    	、    									、    空
	uint32_t end_x:11;       //终点  对顶  空    半轴  半轴  	、    									、    空
	uint32_t end_y:11;       //-------(同上一行)-----------  	数    									数    空
}graphic_data_struct_t;


typedef __packed struct    //字符数据基础单元
{
	graphic_data_struct_t 	grapic_info;
	uint8_t 								buf[30];
}ext_client_custom_character_t;


typedef __packed struct		//整型数或者浮点型数(只是在图形数据基础单元末尾进行修改)	
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


//-------------------------------具体的每个信息包结构-----------------------
/*机器人之间的通讯数据包*/
typedef __packed struct
{
	std_frame_header_t 				 							Tx_FrameHeader;			
	uint16_t  								 							CmdID;						
	
	ext_student_interactive_data_header_t   Data_FrameHeader;
	
	//数据段具体内容还没补充!
	
	uint16_t															  CRC16;								
}ext_robot_interactive_data_t;

/*删除图形的数据包*/
typedef __packed struct
{
	std_frame_header_t 				 							Tx_FrameHeader;			
	uint16_t  								 							CmdID;						
	
	ext_student_interactive_data_header_t   Data_FrameHeader;
	ext_client_custom_graphic_delete_t 		  Client_Data;		
	
	uint16_t															  CRC16;				
}ext_DeleteLayer_Data_t;

/*绘制1个图形的数据包*/
typedef __packed struct
{
	std_frame_header_t 											Tx_FrameHeader;			
	uint16_t  															CmdID;		
	
	ext_student_interactive_data_header_t   Data_FrameHeader;
	graphic_data_struct_t 									Client_Data;		
	
	uint16_t																CRC16;					
}ext_graphic_one_photo_t;

/*绘制2个图形的数据包*/
typedef __packed struct
{
	std_frame_header_t 											Tx_FrameHeader;			
	uint16_t  															CmdID;		
	
	ext_student_interactive_data_header_t   Data_FrameHeader;
	graphic_data_struct_t 									Client_Data[2];		
	
	uint16_t																CRC16;					
}ext_graphic_two_photo_t;

/*绘制5个图形的数据包*/
typedef __packed struct
{
	std_frame_header_t 											Tx_FrameHeader;			
	uint16_t  															CmdID;		
	
	ext_student_interactive_data_header_t   Data_FrameHeader;
	graphic_data_struct_t 									Client_Data[5];		
	
	uint16_t																CRC16;					
}ext_graphic_five_photo_t;

/*绘制7个图形的数据包*/
typedef __packed struct
{
	std_frame_header_t 											Tx_FrameHeader;			
	uint16_t  															CmdID;		
	
	ext_student_interactive_data_header_t   Data_FrameHeader;
	graphic_data_struct_t 									Client_Data[7];		
	
	uint16_t																CRC16;					
}ext_graphic_seven_photo_t;

/*绘制1个字符串或单个字符的数据包*/
typedef __packed struct
{
	std_frame_header_t 											Tx_FrameHeader;			
	uint16_t  															CmdID;				
	
	ext_student_interactive_data_header_t   Data_FrameHeader;
	ext_client_custom_character_t 					Client_Data;
	
	uint16_t																CRC16;							
	
}ext_char_or_string_data_t;


/*绘制1个整形数据或者浮点型数据的数据包*/
typedef __packed struct
{
	std_frame_header_t 											Tx_FrameHeader;			
	uint16_t  															CmdID;				
	
	ext_student_interactive_data_header_t   Data_FrameHeader;
	int32_or_float_data_struct_t   					Client_Data;
	
	uint16_t																CRC16;							
	
}ext_int32_or_float_one_data_t;

/*绘制2个整形数据或者浮点型数据的数据包*/
typedef __packed struct
{
	std_frame_header_t 											Tx_FrameHeader;			
	uint16_t  															CmdID;				
	
	ext_student_interactive_data_header_t   Data_FrameHeader;
	int32_or_float_data_struct_t   					Client_Data[2];
	
	uint16_t																CRC16;							
	
}ext_int32_or_float_two_data_t;

/*绘制5个整形数据或者浮点型数据的数据包*/
typedef __packed struct
{
	std_frame_header_t 											Tx_FrameHeader;			
	uint16_t  															CmdID;				
	
	ext_student_interactive_data_header_t   Data_FrameHeader;
	int32_or_float_data_struct_t   					Client_Data[5];
	
	uint16_t																CRC16;							
	
}ext_int32_or_float_five_data_t;

/*绘制7个整形数据或者浮点型数据的数据包*/
typedef __packed struct
{
	std_frame_header_t 											Tx_FrameHeader;			
	uint16_t  															CmdID;				
	
	ext_student_interactive_data_header_t   Data_FrameHeader;
	int32_or_float_data_struct_t   					Client_Data[7];
	
	uint16_t																CRC16;							
	
}ext_int32_or_float_seven_data_t;



//------------------------------用户的私有定义类型---------------------------------
typedef enum   //用户显示数据的坐标点或者是大小
{
	/*模式是否开启=*/
	Close = 0,
	Open = 1,
	
	Not_Stuck = 0,
	Stuck = 1,
	
	/*电容满能量条时的信息*/
	Cap_Energy_Bar_Layer = 3,	//电容能量条的图形层
	Cap_Name_Size        = 20,
	Cap_Name_Width       = 2,
	Cap_Name_Color       = YELLOW,
	Cap_Name_X_Start 		 = 1800,
	Cap_Name_Y_Start 		 = 750,
	Cap_Energy_Bar_Color = RED_BLUE,
	Cap_Energy_Bar_Warn_Color = FUCHSIA,
	Cap_Energy_Bar_Width = 20, //线条的宽度
	Cap_Energy_Bar_X_Start = 1650, 
	Cap_Energy_Bar_X_End	 = 1900, 
	Cap_Energy_Bar_Y_Start = 700, 
	Cap_Energy_Bar_Y_End	 = 700, 
	
	/*底盘圆形的信息*/
	Chas_Circle_Layer   = 2,
	Chas_Circle_Width   = 3,	
	Chas_Circle_Color   = YELLOW,
	Chas_Circle_X       = 200, //圆心
	Chas_Circle_Y       = 750,
	Chas_Circle_Radius  = 80,  
	
	/*底盘朝向的信息*/
	Chas_Dir_Angle_Layer = Chas_Circle_Layer,
	Chas_Dir_Angle_Width = 8,
	Chas_Dir_Color       = FUCHSIA,
	Chas_Dir_Angle_X     = Chas_Circle_X, //圆心
	Chas_Dir_Angle_Y     = Chas_Circle_Y,
	Chas_Dir_Angle_End_X = Chas_Circle_Radius, 					//x半轴的长度
	Chas_Dir_Angle_End_Y = Chas_Circle_Radius,
	
	/*云台朝向的信息*/
	Gimb_Dir_Layer       = Chas_Circle_Layer,
	Gimb_Dir_Width       = 10,
	Gimb_Dir_Color       = FUCHSIA,
	Gimb_Dir_X_Start     = Chas_Circle_X,
	Gimb_Dir_Y_Start     = Chas_Circle_Y,
	Gimb_Dir_X_End       = Gimb_Dir_X_Start,
	Gimb_Dir_Y_End       = Gimb_Dir_Y_Start+Chas_Circle_Radius,
	
	
	/*底盘姿态的信息*/
	Chas_Data_Layer   	= 6,
	Chas_Data_Width   	= 2,
	Chas_Pitch_X_Mid    = 980,
	Chas_Pitch_Y_Mid    = 80,
	Chas_Pitch_Color    = YELLOW,
	Chas_Line_Half_Len  = 100, //底盘姿态以一条直线表示,中点到一端的距离
	Chas_Header_Size    = 20,
	Chas_Header_Width   = 5,
	Chas_Header_Color   = FUCHSIA,
	Chas_Header_Len     = 30, 	
	
	/*动量块位置的信息*/
	Mometum_Data_Layer   	= 4,
	Mometum_Data_Width   	= 2,
	Mometum_Data_Color    = YELLOW,
	Mometum_Circle_R      = 20, //动量块圆形图案的半径

	/*2个标志位的固定字符名称和标志结果1或0*/
	Fixed_Flag_Str_Layer   = 5,
	Fixed_Flag_Str_Size    = 20,
	Fixed_Flag_Str_Width   = 2,
	Fixed_Flag_Str_Color   = YELLOW,
	Fixed_Flag_Str_X_Start = 870,
	Fixed_Flag_Str_Y_Start = 800,
	Flag_X_Start           = Fixed_Flag_Str_X_Start,
	Flag_Y_Start           = Fixed_Flag_Str_Y_Start-50,
	
	/*是否出现卡墙的信息*/
	Stuck_Data_Layer   = 0,
	Stuck_Data_Size    = 60,
	Stuck_Data_Width   = 5,
	Stuck_Data_Color   = GREEN,
	Stuck_Data_X_Start = 820,
	Stuck_Data_Y_Start = 800,
	

	/*底盘视野宽度的信息*/
	Chas_Vision_Wide_Layer = 2,
	Chas_Vision_Wide_Color = PINK,
	Chas_Vision_Wide_Width = 3,
	//左宽度线条坐标
	Chas_Vision_Wide_L_X_Start = 600,
	Chas_Vision_Wide_L_X_End = Chas_Vision_Wide_L_X_Start+200,
	Chas_Vision_Wide_L_Y_Start = 0,
	Chas_Vision_Wide_L_Y_End = 300,
	//右宽度线条坐标
	Chas_Vision_Wide_R_X_Start = Chas_Vision_Wide_L_X_Start+720,
	Chas_Vision_Wide_R_X_End = Chas_Vision_Wide_R_X_Start-200,
	Chas_Vision_Wide_R_Y_Start = Chas_Vision_Wide_L_Y_Start,
	Chas_Vision_Wide_R_Y_End = Chas_Vision_Wide_L_Y_End,
	//上宽度线条坐标
	Chas_Vision_Length_H_X_Start = Chas_Vision_Wide_L_X_End,
	Chas_Vision_Length_H_X_End = Chas_Vision_Wide_R_X_End,
	Chas_Vision_Length_H_Y_Start = Chas_Vision_Wide_L_Y_End,
	Chas_Vision_Length_H_Y_End = Chas_Vision_Wide_L_Y_End,
	//下宽度线条坐标
	Chas_Vision_Length_L_X_Start = (Chas_Vision_Wide_L_X_Start+Chas_Vision_Wide_L_X_End)/2+10,
	Chas_Vision_Length_L_X_End = (Chas_Vision_Wide_R_X_Start+Chas_Vision_Wide_R_X_End)/2-10,
	Chas_Vision_Length_L_Y_Start = Chas_Vision_Wide_L_Y_End-130,
	Chas_Vision_Length_L_Y_End = Chas_Vision_Length_L_Y_Start,
	
	/*上层准心*/
	Aim_Layer = 1,
	Aim_Color = GREEN,
	Aim_Width = 5,
	Aim_Short_Line_X_Start = 980,  //短横线的X起点坐标
	Aim_Short_Line_Length = 40, //短横线的长度
	Aim_Top_Y_Base_Start = 600, //最上层的准心横线
	Aim_Y_Interval = 30, 						//横线之间的间距
	
	
	
}User_Typedef_e;




typedef struct UI_Data_Info //要显示的具体数据
{
	float  cap_power_percent; //超电剩余能量百分比,电容组两端24.5V时为最高能量,以能量条形式表示该数据
	
	float  chas_pitch; 	 //底盘倾角,度为单位
	float  momentum_pos; //动量块位置,米为单位
	
	User_Typedef_e  small_top_state;  //小陀螺是否开启
	User_Typedef_e  magz_state;       //弹仓是否开启

	char  stuck_state;      //是否卡墙
	
	float  chas_power_buffer_percent;    //底盘剩余功率缓冲百分比
	
	float  chas_dir_angle;  //底盘朝向的方位角
	
	
}UI_Data_Info_t;

typedef struct UI_Operate   //每个数据的操作类型
{
	/*字符串数据包*/	
	graphic_data_struct_operate_type_e    fixed_flag_str;
	graphic_data_struct_operate_type_e    flag; 
	graphic_data_struct_operate_type_e    stuck_str;
	
	/*图片数据包*/
	graphic_data_struct_operate_type_e    fixed_photo;	
	graphic_data_struct_operate_type_e    not_fixed_photo;
	graphic_data_struct_operate_type_e    aim;
	
}UI_Operate_t;

typedef struct UI_UART_Tx_Pack //要通过串口发送出去的数据包
{
	/*字符串数据包*/
	ext_char_or_string_data_t  fixed_flag_str; //依次包括small_top_data, magz_data, magz_data
	ext_char_or_string_data_t  flag;  //依次包括剩余子弹数目,small_top_data, magz_data, magz_data
	
	ext_char_or_string_data_t  stuck_str; 
	
	/*图形数据包*/	
	ext_graphic_seven_photo_t  fixed_photo; //依次包括chas_circle,gimb_dir,chas_vision_wide,chas_vision_length共6个
	ext_graphic_five_photo_t   not_fixed_photo; //依次包括chas_gesture,momentum_gesture,chas_header,chas_dir,cap_data共5个	  
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
