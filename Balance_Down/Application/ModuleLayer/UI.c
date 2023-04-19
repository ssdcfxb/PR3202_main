#include "UI.h"

/*----------私有变量声明----------*/
extern UART_HandleTypeDef huart4;
UI_Data_t 	UI_Data;
uint8_t  Tx_Client_Buf[200];

//-----------------------上层
void UI_Ctrl(void);



//------------------------虚函数,来自chassic文件
void Chas_Show_UI_Stuck(const char flag);


//-----------------------应用2
void UI_Traverse_Show_Ext(const uint16_t i);
void UI_Change_Operate_Type(const graphic_data_struct_operate_type_e  type);
void Save_Tx_Pack_Fixed_Info(void);
void UI_Prepare(void);

//-----------------------应用1
void UI_TX_ext_fixed_photo(void);
void UI_TX_ext_fixed_flag_str(void);
void UI_TX_ext_data(void);
void UI_TX_ext_not_fixed_photo(void);
void UI_TX_ext_stuck_str(void);
void UI_TX_ext_aim(void);



//-----------------------中间
void UI_Save_Fixed_Photo(const uint8_t operate_type);
void UI_Save_Chas_Circle(const uint8_t operate_type);
void UI_Save_Gimb_Dir(const uint8_t operate_type);
void UI_Save_Chas_Vision_Wide(const uint8_t operate_type);
void UI_Save_Chas_Vision_Length(const uint8_t operate_type);

void UI_Save_Fixed_Flag_Str(const uint8_t operate_type);

void UI_Save_Data(const char top, const char magz, const uint8_t operate_type);
void UI_Save_Stuck_Str(const uint8_t mode, const uint8_t operate_type);

void UI_Save_Not_Fixed_Photo(const uint8_t operate_type);
void UI_Save_Chas_Gesture(const float pitch, const uint8_t operate_type);
void UI_Save_Mom_Gesture(const float momentum_pos ,const uint8_t operate_type);
void UI_Save_Chas_Header(const uint8_t operate_type);
void UI_Save_Chas_Dir(const float chas_dir_angle,const uint8_t operate_type);
void UI_Save_Cap_Data(const float voltage, const uint8_t operate_type);

void UI_Save_Aim(const uint8_t operate_type);



//-----------------------底层
void UI_Send_Data_Pack(const uint8_t* txbuf, const uint8_t len);
void UI_Data_ID_UP(void);



/**	
	*@brief  Tx_Client_Tmp_Buf的大小是下面三个的长度,存放发给裁判系统数据的前固定数据
	std_frame_header_t 											Tx_FrameHeader;	 		 5字节
	uint16_t  															CmdID;							 2字节
		
	ext_student_interactive_data_header_t   Data_FrameHeader;		 6字节
*/
uint8_t  Tx_Client_Tmp_Buf[INTERACT_DATA_FIXED_LEN];


/*--------------------------------通过串口发送一帧数据-----------------------------------*/
void UI_Send_Data_Pack(const uint8_t* txbuf, const uint8_t len)
{
	if(txbuf == NULL)
		return;

	HAL_UART_Transmit_DMA(&huart4, txbuf, len);
	
}

/*-----------------------------------显示指定的类型,底层函数----------------------------------------*/




void Draw_String_Or_Char(ext_client_custom_character_t*  graphic,  //讲图形的特点信息放到这里
												 
												 const char* name,     //作为客户端的索引
													 
									       uint32_t operate_type, //图形操作
												 uint32_t layer,				//图形层数
												 uint32_t color,				//图形颜色
												 uint32_t size,					//字符大小						 
												 uint32_t length,				//字符长度 
												 uint32_t width,				//线宽
												 uint32_t start_x,			//X坐标
												 uint32_t start_y,			//Y坐标						 
												 
												 const char *string)		//要显示的字符
{

	if(graphic == NULL || name == NULL || string == NULL)
		return;
	
	graphic_data_struct_t 	*grapic_info = &graphic->grapic_info;
	
	for(char i = 0 ; i < 3; i ++)
		grapic_info->graphic_name[i] = name[i];	
	grapic_info->operate_type = operate_type;
	grapic_info->graphic_type = CHAR;        //Char类型直接赋值
	grapic_info->layer = layer;
	grapic_info->color = color;
	grapic_info->start_angle = size;
	grapic_info->end_angle = length;	
	grapic_info->width = width;
	grapic_info->start_x = start_x;
	grapic_info->start_y = start_y;	
	
	grapic_info->radius = 0;
	grapic_info->end_x = 0;
	grapic_info->end_y = 0;

	memcpy(graphic->buf, string, length);
	
}

void Draw_Firgue(graphic_data_struct_t* graphic,
										
								 const char* name,
									
								 uint32_t operate_type,    
								 uint32_t graphic_type,   
								 uint32_t layer,					 
								 uint32_t color,          
								 uint32_t start_angle,     
								 uint32_t end_angle,
								 uint32_t width,
								 uint32_t start_x,
								 uint32_t start_y,
								 uint32_t radius,
								 uint32_t end_x,
								 uint32_t end_y)							
{
	if(graphic == NULL || name == NULL)
		return;
	
	for(char i = 0; i < 3; i ++)
		graphic->graphic_name[i] = name[i];	//字符索引
	
	graphic->operate_type = operate_type; 
	graphic->graphic_type = graphic_type;        
	graphic->layer        = layer;		
	graphic->color        = color;
	graphic->start_angle  = start_angle;
	graphic->end_angle    = end_angle;	
	graphic->width        = width;
	graphic->start_x      = start_x;
	graphic->start_y      = start_y;	
	graphic->radius       = radius;
	graphic->end_x        = end_x;
	graphic->end_y        = end_y;
	
}

void Draw_Float_Or_Int32(int32_or_float_data_struct_t* graphic,
	
												 const char* name,
													 
												 uint32_t operate_type,
												 uint32_t graphic_type,
												 uint32_t layer,
												 uint32_t color,
												 uint32_t size,
												 uint32_t decimal,	//小数点后的有效位数, 0即显示int32_t类型
												 uint32_t width,
												 uint32_t start_x,
												 uint32_t start_y,
												 
												 int32_t number)							
{
	if(graphic == NULL || name == NULL)
		return;
	
	for(char i = 0; i < 3; i ++)
		graphic->graphic_name[i] = name[i];	
	
	graphic->operate_type = operate_type; 
	graphic->graphic_type = graphic_type;  
	graphic->layer        = layer;
	graphic->color        = color;
	graphic->start_angle  = size;
	graphic->end_angle    = decimal;
	graphic->width        = width;
	graphic->start_x      = start_x;
	graphic->start_y      = start_y;	
	graphic->number       = number;
}




/*-------------------------------总控制---------------------------------------------*/
uint16_t ext_i, operate_i;
void UI_Ctrl(void)
{
	UI_Prepare();
	
	/*运行完2周后将图层操作变为ADD*/
	if( operate_i < 2 )
	{
		UI_Change_Operate_Type(ADD);
		UI_Traverse_Show_Ext( ext_i / UI_Data_Current_Interval );/*相邻两个数据流的时间间隔: 150ms*/

		if(ext_i == UI_Data_Current_Interval * Draw_Total_Num + 1)
		{
			operate_i ++;
			ext_i = 0;
		}
	}
	else
	{
		UI_Change_Operate_Type(MODIFY);
		
		if(ext_i % (UI_Data_Current_Interval+100) == 0)
			UI_Traverse_Show_Ext(ext_i / (UI_Data_Current_Interval+100) + Fixed_Photo_Num);

		if(ext_i == (UI_Data_Current_Interval+100) * Not_Fixed_Photo_Num + 1)
		{
			operate_i ++;
			ext_i = 0;
		}
		
	}
	
	ext_i++;
	operate_i %= 10;
}




//--------------------------------应用层2--------------------------------------------------------//

void UI_Prepare(void)
{
	Save_Tx_Pack_Fixed_Info();
	UI_Data_ID_UP();
}


/*底盘虚函数重写*/
void Chas_Show_UI_Stuck(const char flag)
{
	UI_Data.data.stuck_state = flag;
}
/*修改图层的操作方式*/
void UI_Change_Operate_Type(const graphic_data_struct_operate_type_e type)
{
	graphic_data_struct_operate_type_e* p  =  &UI_Data.operate.fixed_flag_str;

	for(uint8_t j = 0; j < sizeof(UI_Operate_t) / sizeof(graphic_data_struct_operate_type_e); j ++)
		p[j] = type;
	
}

/*遍历显示,前2个固定*/
void UI_Traverse_Show_Ext(const uint16_t i)
{
	switch(i)
	{
		case 1:
			UI_TX_ext_fixed_photo();
		break;
		
		case 2:
			UI_TX_ext_fixed_flag_str();
		break;
		
		case 3:
			UI_TX_ext_aim();
		break;
			
		case 4:
			UI_TX_ext_not_fixed_photo();
		break;
		
		case 5:
			UI_TX_ext_data();
		break;
		
		default:
		break;
	}
}
//--------------------------------应用层1--------------------------------------------------------//





/*发送给裁判系统的数据,在前面一端数据基本是固定的*/
//需更新数据长度,CRC8,数据段头结构的数据命令码
void Save_Tx_Pack_Fixed_Info(void)
{

	//帧头
	Tx_Client_Tmp_Buf[0] = JUDGE_FRAME_HEADER;
	//数据长度
	Tx_Client_Tmp_Buf[1] = Tx_Client_Tmp_Buf[2] = 0;
	//包序号
	Tx_Client_Tmp_Buf[3] = 0;
	//CRC8
	Tx_Client_Tmp_Buf[4] = 0;
	
	
	
	//命令码
	Tx_Client_Tmp_Buf[5] = (uint8_t)ID_robot_interactive_header_data;
	Tx_Client_Tmp_Buf[6] = (uint8_t)(ID_robot_interactive_header_data>>8);
	
	
	
	
	
	//数据段头结构的数据命令码
	Tx_Client_Tmp_Buf[7] = 0;
	Tx_Client_Tmp_Buf[8] = 0;
	//数据段头结构的机器人ID
	Tx_Client_Tmp_Buf[9] =  (uint8_t)UI_Data.robot_id;
	Tx_Client_Tmp_Buf[10] = (uint8_t)(UI_Data.robot_id>>8);
	//数据段头结构的客户端ID
	Tx_Client_Tmp_Buf[11] = (uint8_t)UI_Data.client_id;
	Tx_Client_Tmp_Buf[12] = (uint8_t)(UI_Data.client_id>>8);
	
	
}


/*发送给客户端关于固定图片组*/
void UI_TX_ext_fixed_photo(void)
{
	UI_Save_Fixed_Photo(UI_Data.operate.fixed_photo);
	
	//固定头数据,前13个字节
	memcpy(&UI_Data.ext.fixed_photo.Tx_FrameHeader.sof, \
				 Tx_Client_Tmp_Buf, \
				 INTERACT_DATA_FIXED_LEN);
	
	UI_Data.ext.fixed_photo.Tx_FrameHeader.data_length = sizeof(ext_student_interactive_data_header_t) + \
																											 sizeof(graphic_data_struct_t)*7;
	
	UI_Data.ext.fixed_photo.Data_FrameHeader.data_cmd_id = INTERACT_ID_draw_seven_graphic;	
	
	//头校验CRC8
	memcpy(Tx_Client_Buf, &UI_Data.ext.fixed_photo.Tx_FrameHeader.sof, sizeof(ext_graphic_seven_photo_t));

	Append_CRC8_Check_Sum(Tx_Client_Buf, sizeof(std_frame_header_t));
	
	Append_CRC16_Check_Sum(Tx_Client_Buf,sizeof(ext_graphic_seven_photo_t));
		
	UI_Send_Data_Pack(Tx_Client_Buf, sizeof(ext_graphic_seven_photo_t));
	
	
}

/*发送给客户端关于固定的标志符号字符串的信息*/
void UI_TX_ext_fixed_flag_str(void)
{
	memset(UI_Data.ext.fixed_flag_str.Client_Data.buf, 0, 30);
	
	UI_Save_Fixed_Flag_Str(UI_Data.operate.fixed_flag_str);
	
	//固定头数据,前13个字节
	memcpy(&UI_Data.ext.fixed_flag_str.Tx_FrameHeader.sof, \
				 Tx_Client_Tmp_Buf, \
				 INTERACT_DATA_FIXED_LEN);
	
	UI_Data.ext.fixed_flag_str.Tx_FrameHeader.data_length = sizeof(ext_student_interactive_data_header_t) + \
																													sizeof(ext_client_custom_character_t);
	
	UI_Data.ext.fixed_flag_str.Data_FrameHeader.data_cmd_id = INTERACT_ID_draw_char_graphic;	
	
	//头校验CRC8
	memcpy(Tx_Client_Buf, &UI_Data.ext.fixed_flag_str.Tx_FrameHeader.sof, sizeof(ext_char_or_string_data_t));

	Append_CRC8_Check_Sum(Tx_Client_Buf, sizeof(std_frame_header_t));
	
	Append_CRC16_Check_Sum(Tx_Client_Buf,sizeof(ext_char_or_string_data_t));
		
	UI_Send_Data_Pack(Tx_Client_Buf, sizeof(ext_char_or_string_data_t));
	
	
}
/*发送给客户端关于固定的标志位的具体值*/
void UI_TX_ext_data(void)
{
	char top, magz;
	
	memset(UI_Data.ext.flag.Client_Data.buf, 0, 30);
	
	if(State.chassic_flag.chas_motion_mode == SMALL_TOP_ON)
		top = 1;
	if(State.rc_flag.magz_state == OPEN)
		magz = 1;

	UI_Save_Data(top, magz,UI_Data.operate.flag);
							 
	
	//固定头数据,前13个字节
	memcpy(&UI_Data.ext.flag.Tx_FrameHeader.sof, \
				 Tx_Client_Tmp_Buf, \
				 INTERACT_DATA_FIXED_LEN);
	
	UI_Data.ext.flag.Tx_FrameHeader.data_length = sizeof(ext_student_interactive_data_header_t) + \
																								sizeof(ext_client_custom_character_t);
	
	UI_Data.ext.flag.Data_FrameHeader.data_cmd_id = INTERACT_ID_draw_char_graphic;	
	
	//头校验CRC8
	memcpy(Tx_Client_Buf, &UI_Data.ext.flag.Tx_FrameHeader.sof, sizeof(ext_char_or_string_data_t));

	Append_CRC8_Check_Sum(Tx_Client_Buf, sizeof(std_frame_header_t));
	
	Append_CRC16_Check_Sum(Tx_Client_Buf,sizeof(ext_char_or_string_data_t));
		
	UI_Send_Data_Pack(Tx_Client_Buf, sizeof(ext_char_or_string_data_t));
	
	
}


/*发送给客户端关于不固定图片组*/
void UI_TX_ext_not_fixed_photo(void)
{
	UI_Save_Not_Fixed_Photo(UI_Data.operate.not_fixed_photo);
	
	//固定头数据,前13个字节
	memcpy(&UI_Data.ext.not_fixed_photo.Tx_FrameHeader.sof, \
				 Tx_Client_Tmp_Buf, \
				 INTERACT_DATA_FIXED_LEN);
	
	UI_Data.ext.not_fixed_photo.Tx_FrameHeader.data_length = sizeof(ext_student_interactive_data_header_t) + \
																											 sizeof(graphic_data_struct_t)*5;
	
	UI_Data.ext.not_fixed_photo.Data_FrameHeader.data_cmd_id = INTERACT_ID_draw_five_graphic;	
	
	//头校验CRC8
	memcpy(Tx_Client_Buf, &UI_Data.ext.not_fixed_photo.Tx_FrameHeader.sof, sizeof(ext_graphic_five_photo_t));

	Append_CRC8_Check_Sum(Tx_Client_Buf, sizeof(std_frame_header_t));
	
	Append_CRC16_Check_Sum(Tx_Client_Buf,sizeof(ext_graphic_five_photo_t));
		
	UI_Send_Data_Pack(Tx_Client_Buf, sizeof(ext_graphic_five_photo_t));
	
	
}
/*发送给客户端关于是否卡墙的信息*/
void UI_TX_ext_stuck_str(void)
{
	memset(UI_Data.ext.stuck_str.Client_Data.buf, 0, 30);
	
	UI_Save_Stuck_Str(UI_Data.data.stuck_state, UI_Data.operate.stuck_str);
	
	//固定头数据,前13个字节
	memcpy(&UI_Data.ext.stuck_str.Tx_FrameHeader.sof, \
				 Tx_Client_Tmp_Buf, \
				 INTERACT_DATA_FIXED_LEN);
	
	//更新数据长度,CRC8,数据段头结构的数据命令码
	UI_Data.ext.stuck_str.Tx_FrameHeader.data_length = sizeof(ext_student_interactive_data_header_t) + \
																										 sizeof(ext_client_custom_character_t);
	UI_Data.ext.stuck_str.Data_FrameHeader.data_cmd_id = INTERACT_ID_draw_char_graphic;
	
	//放在数组里
	memcpy(Tx_Client_Buf, &UI_Data.ext.stuck_str.Tx_FrameHeader.sof, sizeof(ext_char_or_string_data_t));
	
	//头校验CRC8
	Append_CRC8_Check_Sum(Tx_Client_Buf, sizeof(std_frame_header_t));
	//尾校验CRC16
	Append_CRC16_Check_Sum(Tx_Client_Buf, sizeof(ext_char_or_string_data_t));
	
	UI_Send_Data_Pack(Tx_Client_Buf, sizeof(ext_char_or_string_data_t));
}
/*发送给客户端关于准心图组*/
void UI_TX_ext_aim(void)
{
	UI_Save_Aim(UI_Data.operate.aim);
	
	//固定头数据,前13个字节
	memcpy(&UI_Data.ext.aim.Tx_FrameHeader.sof, \
				 Tx_Client_Tmp_Buf, \
				 INTERACT_DATA_FIXED_LEN);
	
	UI_Data.ext.aim.Tx_FrameHeader.data_length = sizeof(ext_student_interactive_data_header_t) + \
																											 sizeof(graphic_data_struct_t)*5;
	
	UI_Data.ext.aim.Data_FrameHeader.data_cmd_id = INTERACT_ID_draw_five_graphic;	
	
	//头校验CRC8
	memcpy(Tx_Client_Buf, &UI_Data.ext.aim.Tx_FrameHeader.sof, sizeof(ext_graphic_five_photo_t));

	Append_CRC8_Check_Sum(Tx_Client_Buf, sizeof(std_frame_header_t));
	
	Append_CRC16_Check_Sum(Tx_Client_Buf,sizeof(ext_graphic_five_photo_t));
		
	UI_Send_Data_Pack(Tx_Client_Buf, sizeof(ext_graphic_five_photo_t));
	
	
}


//------------------------------保存要显示的具体数据到结构体------------------------------------//
/*固定图片依次顺序*/



void UI_Save_Fixed_Photo(const uint8_t operate_type)
{
	UI_Save_Chas_Circle(operate_type);
	UI_Save_Gimb_Dir(operate_type);
	UI_Save_Chas_Vision_Wide(operate_type);
	UI_Save_Chas_Vision_Length(operate_type);
}
/*以圆形形式显示底盘圆形*/
void UI_Save_Chas_Circle(const uint8_t operate_type)
{

	Draw_Firgue(&UI_Data.ext.fixed_photo.Client_Data[0], 
							"CC1", 
							operate_type, 
							CIRCLE,
							Chas_Circle_Layer,
							Chas_Circle_Color,
							0,
							0,
							Chas_Circle_Width,
							Chas_Circle_X,
							Chas_Circle_Y,
							Chas_Circle_Radius,
							0,
							0);	
	
}

/*以直线形式显示云台朝向*/
void UI_Save_Gimb_Dir(const uint8_t operate_type)
{

	Draw_Firgue(&UI_Data.ext.fixed_photo.Client_Data[1], 
							"GB1", 
							operate_type, 
							LINE,
							Gimb_Dir_Layer,
							Gimb_Dir_Color,
							0,
							0,
							Gimb_Dir_Width,
							Gimb_Dir_X_Start,
							Gimb_Dir_Y_Start,
							0,
							Gimb_Dir_X_End,
							Gimb_Dir_Y_End);	
	
}

/*以直线形式显示底盘视野的宽*/
void UI_Save_Chas_Vision_Wide(const uint8_t operate_type)
{
	/*左宽度*/
	Draw_Firgue(&UI_Data.ext.fixed_photo.Client_Data[2], 
							"CW1", 
							operate_type, 
							LINE,
							Chas_Vision_Wide_Layer,
							Chas_Vision_Wide_Color,
							0,
							0,
							Chas_Vision_Wide_Width,
							Chas_Vision_Wide_L_X_Start,
							Chas_Vision_Wide_L_Y_Start,
							0,
							Chas_Vision_Wide_L_X_End,
							Chas_Vision_Wide_L_Y_End);	

	/*右宽度*/
	Draw_Firgue(&UI_Data.ext.fixed_photo.Client_Data[3], 
							"CW2", 
							operate_type, 
							LINE,
							Chas_Vision_Wide_Layer,
							Chas_Vision_Wide_Color,
							0,
							0,
							Chas_Vision_Wide_Width,
							Chas_Vision_Wide_R_X_Start,
							Chas_Vision_Wide_R_Y_Start,
							0,
							Chas_Vision_Wide_R_X_End,
							Chas_Vision_Wide_R_Y_End);	
}

/*以直线形式显示底盘视野的长*/
void UI_Save_Chas_Vision_Length(const uint8_t operate_type)
{
	/*上长度*/
	Draw_Firgue(&UI_Data.ext.fixed_photo.Client_Data[4], 
							"CL1", 
							operate_type, 
							LINE,
							Chas_Vision_Wide_Layer,
							Chas_Vision_Wide_Color,
							0,
							0,
							Chas_Vision_Wide_Width,
							Chas_Vision_Length_H_X_Start,
							Chas_Vision_Length_H_Y_Start,
							0,
							Chas_Vision_Length_H_X_End,
							Chas_Vision_Length_H_Y_End);	

	/*下长度*/
	Draw_Firgue(&UI_Data.ext.fixed_photo.Client_Data[5], 
							"CL2", 
							operate_type, 
							LINE,
							Chas_Vision_Wide_Layer,
							Chas_Vision_Wide_Color,
							0,
							0,
							Chas_Vision_Wide_Width,
							Chas_Vision_Length_L_X_Start,
							Chas_Vision_Length_L_Y_Start,
							0,
							Chas_Vision_Length_L_X_End,
							Chas_Vision_Length_L_Y_End);	
}

/*固定字符串依次顺序*/




void UI_Save_Fixed_Flag_Str(const uint8_t operate_type)
{
	strcpy((char*)UI_Data.ext.fixed_flag_str.Client_Data.buf,
				 "TOP  MAGZ");
		
	Draw_String_Or_Char(&UI_Data.ext.fixed_flag_str.Client_Data,
											"FL1",
											operate_type,
											Fixed_Flag_Str_Layer,
											Fixed_Flag_Str_Color,
											Fixed_Flag_Str_Size,
											strlen((char*)UI_Data.ext.fixed_flag_str.Client_Data.buf),
											Fixed_Flag_Str_Width,
											Fixed_Flag_Str_X_Start,
	                    Fixed_Flag_Str_Y_Start,
											(char*)UI_Data.ext.fixed_flag_str.Client_Data.buf);
}
/*不固定字符串*/



/*标志位用字符1或者0表示,1开0关*/
void UI_Save_Data(const char top, const char magz, const uint8_t operate_type)                
{
	sprintf((char*)UI_Data.ext.flag.Client_Data.buf, " %d    %d",top, magz);
																																						
		
	Draw_String_Or_Char(&UI_Data.ext.flag.Client_Data,
											"FL2",
											operate_type,
											Fixed_Flag_Str_Layer,
											Fixed_Flag_Str_Color,
											Fixed_Flag_Str_Size,
											strlen((char*)UI_Data.ext.flag.Client_Data.buf),
											Fixed_Flag_Str_Width,
											Flag_X_Start,
	                    Flag_Y_Start,
											(char*)UI_Data.ext.flag.Client_Data.buf);
}






/*以字符串的形式去显示是否有卡墙*/
void UI_Save_Stuck_Str(const uint8_t mode, const uint8_t operate_type)
{
	if(mode == ALINE_STUCK || mode == SPIN_STUCK)	
	{
		UI_Data.data.stuck_state = Stuck;
		strcpy((char*)UI_Data.ext.stuck_str.Client_Data.buf,
						"STUCK!");
	}
	else //不卡墙
	{
		UI_Data.data.stuck_state = Not_Stuck;
		memset(UI_Data.ext.stuck_str.Client_Data.buf, \
					 ' ', \
					 30);
	}
		
	Draw_String_Or_Char(&UI_Data.ext.stuck_str.Client_Data,
											"ST1",
											operate_type,
											Stuck_Data_Layer,
											Stuck_Data_Color,
											Stuck_Data_Size,
											strlen((char*)UI_Data.ext.stuck_str.Client_Data.buf),
											Stuck_Data_Width,
											Stuck_Data_X_Start,
											Stuck_Data_Y_Start,
											(char*)UI_Data.ext.stuck_str.Client_Data.buf);
	
}
/*不固定图片依次顺序*/


void UI_Save_Not_Fixed_Photo(const uint8_t operate_type)
{
	UI_Save_Chas_Gesture(chassic.info.Status.meas.Pitch, operate_type);
	UI_Save_Mom_Gesture(chassic.info.Status.meas.M_X, operate_type);
	UI_Save_Chas_Header(operate_type);
	UI_Save_Chas_Dir(chassic.info.Status.meas.Yaw*RADIAN_CONVERSION_ANGLE, operate_type);
	UI_Save_Cap_Data(Super_2023.RX.cap_Ucr, operate_type);
}
/*以直线形式显示底盘倾角,参数为弧度*/
void UI_Save_Chas_Gesture(const float pitch, const uint8_t operate_type)
{
	UI_Data.data.chas_pitch = pitch;
	
	//底盘倾角,且在现实中底盘往前倾斜的pitch角度是负的
	Draw_Firgue(&UI_Data.ext.not_fixed_photo.Client_Data[0],
							"CH1",
							operate_type,
							LINE,
							Chas_Data_Layer,
							Chas_Pitch_Color,
							0,
							0,
							Chas_Data_Width,
							Chas_Pitch_X_Mid - Chas_Line_Half_Len, //起始X,直线左端
							Chas_Pitch_Y_Mid - Chas_Line_Half_Len * arm_sin_f32(pitch),  //起始Y,直线左端
	            0,
							Chas_Pitch_X_Mid + Chas_Line_Half_Len, //终点X,直线右端
							Chas_Pitch_Y_Mid + Chas_Line_Half_Len * arm_sin_f32(pitch)); //终点Y,直线右端

							
}
/*以圆形形式显示动量块位置*/
void UI_Save_Mom_Gesture(const float momentum_pos ,const uint8_t operate_type)	 
{
	UI_Data.data.momentum_pos = momentum_pos;
	
	float len = momentum_pos / MOMENTUM_MAX_X_MEAS * Chas_Line_Half_Len;
	
	//动量块平均位置,且在现实中动量块往前位移是正的
	Draw_Firgue(&UI_Data.ext.not_fixed_photo.Client_Data[1],
							"MO1",
							operate_type,
							CIRCLE,
							Mometum_Data_Layer,
							Mometum_Data_Color,
							0,
							0,
							Mometum_Data_Width,
							Chas_Pitch_X_Mid + len, //直线终点X,直线右端
							Chas_Pitch_Y_Mid + len * arm_sin_f32(chassic.info.Status.meas.Pitch), //直线终点Y,直线右端
							Mometum_Circle_R,
							0,
							0);
							
}
/*以竖直直线的形式去显示底盘倾角的正方向*/
void UI_Save_Chas_Header(const uint8_t operate_type)
{
	float x_start, y_start, pitch;
	
	//获取底盘姿态图案坐标
	x_start = UI_Data.ext.not_fixed_photo.Client_Data[0].end_x;
	y_start = UI_Data.ext.not_fixed_photo.Client_Data[0].end_y;
	pitch = chassic.info.Status.meas.Pitch;
	
	
	Draw_Firgue(&UI_Data.ext.not_fixed_photo.Client_Data[2],
							"CE1",
							operate_type,
							LINE,
							Chas_Data_Layer,
							Chas_Header_Color,
							0,
							0,
							Chas_Header_Width,
							x_start,    //起点X
							y_start,		//起点Y
	            0,
							x_start - Chas_Header_Len*arm_sin_f32(pitch), 				//终点X
							y_start + Chas_Header_Len*arm_cos_f32(pitch)); //终点Y

}
/*以圆弧形式显示底盘此时方位,参数是角度*/
void UI_Save_Chas_Dir(const float chas_dir_angle,const uint8_t operate_type)
{
	UI_Data.data.chas_dir_angle = chas_dir_angle;

	float chas_dir_start_angle, chas_dir_end_angle;
	if(chas_dir_angle > 0)
		chas_dir_start_angle = 360.f - chas_dir_angle;
	else
		chas_dir_start_angle = -chas_dir_angle;
	chas_dir_end_angle = chas_dir_start_angle + 30.f;
	
	
	
	/*以圆弧的形式画底盘此时的朝向*/
	Draw_Firgue(&UI_Data.ext.not_fixed_photo.Client_Data[3], 
							"CC2", 
							operate_type, 
							ARC,
							Chas_Dir_Angle_Layer,
							Chas_Dir_Color,
							chas_dir_start_angle,
							chas_dir_end_angle,
							Chas_Dir_Angle_Width,
							Chas_Circle_X,
							Chas_Circle_Y,
							0,
							Chas_Dir_Angle_End_X,
							Chas_Dir_Angle_End_Y);		
}

/*以粗直线能量条的形式去显示超电的剩余能量*/
void UI_Save_Cap_Data(const float voltage, const uint8_t operate_type)
{
	float X_End;
	uint8_t  color;
	
	UI_Data.data.cap_power_percent = voltage / CAP_MAX_VOLTAGE;
	
	X_End = Cap_Energy_Bar_X_Start+voltage*voltage/2.f;
	
	if(voltage<18)
		color = Cap_Energy_Bar_Warn_Color;
	else
		color = Cap_Energy_Bar_Color;
	
	Draw_Firgue(&UI_Data.ext.not_fixed_photo.Client_Data[4], 
							"SR1", 
							operate_type, 
							LINE,
							Cap_Energy_Bar_Layer,
							color,
							0,
							0,
							Cap_Energy_Bar_Width,
							Cap_Energy_Bar_X_Start,
							Cap_Energy_Bar_Y_Start,
							0,
							(uint32_t)X_End,
							Cap_Energy_Bar_Y_End);
			
}




/*准心*/
void UI_Save_Aim(const uint8_t operate_type)
{
	/*竖直线*/
	Draw_Firgue(&UI_Data.ext.aim.Client_Data[0], "AI0", operate_type, LINE, Aim_Layer, \
							Aim_Color, 0,	0, Aim_Width, 960, 650, 0, 960, 350);
	
	/*第1条横线*/
	Draw_Firgue(&UI_Data.ext.aim.Client_Data[1], "AI1", operate_type, LINE, Aim_Layer, \
							Aim_Color, 0,	0, Aim_Width, 800, 520, 0, 960-800+960, 520);
	
	/*第2条横线*/
	Draw_Firgue(&UI_Data.ext.aim.Client_Data[2], "AI2", operate_type, LINE, Aim_Layer, \
							Aim_Color, 0,	0, Aim_Width, 850, 500, 0, 960-850+960, 500);
	
	/*第3条横线*/
	Draw_Firgue(&UI_Data.ext.aim.Client_Data[3], "AI3", operate_type, LINE, Aim_Layer, \
							Aim_Color, 0,	0, Aim_Width, 850, 480, 0, 960-850+960, 480);
	
}



/*id更新*/
void UI_Data_ID_UP(void)
{
	UI_Data.robot_id = judge_info.game_robot_status.robot_id;
	UI_Data.client_id = judge_info.self_client;
	
}
