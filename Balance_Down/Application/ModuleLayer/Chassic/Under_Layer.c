#include "Under_Layer.h"
/*Private variables*/
device_info_t  					 Device_Info;
sys_status_pack_info_t   Chas_Sys_Status;

/*Exported variables*/

/*Privated Functions*/
void Save_Device_Info(void);


/**
	*@brief  �����û������Ŀ��������ݰ�������3��
	*@note   Ĭ��user_status_t��ÿ�����ݶ���float����
	*@param[in] statusΪ�������û������Ŀ��������ݰ���������ָ��
	*@return NULL
*/
//void Sys_Info_UP(user_status_pack_info_t* status)									 
//{


//}


///**
//	*@brief 	�����û���������ݰ�,��ΪĿ�����������������
//	*@note  	Ĭ��user_status_t��ÿ��������ռ�ڴ�Ĵ�Сһ��
//	*@param[in] 	statusΪָ���û���������ݰ���ָ��
//	*@param[in] 	param_numΪ�ɱ�����б�Ĳ�������
//	*@return 	NULL
// */
//void User_Status_Info_UP(user_status_t* status, const uint8_t param_num, ...)
//{
//	if(status == NULL)	return;
//	
//	va_list valist;
//	va_start(valist, param_num);
//	float* status_p = &status->C_X_L;
//	
//	for(uint8_t i = 0; i < param_num; i ++)
//	{
//		status_p[i] = va_arg(valist, float);
//	}	
//	
//	va_end(valist);
//	
//}
/**
	*@brief 	�����豸�ṩ������
	*@return 	NULL
 */
void Save_Device_Info(void)
{
	
}
