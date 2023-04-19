#include "power_limit.h"


void Chassis_Motor_Power_Limit(float *data,float buffer)
{
	if(data == NULL)
		return;
	
	float Limit_k = 0;
	
	if(buffer > CHAS_MAX_POWER_BUFFER)//������ɺ�,buffer���Զ���߲�ͣ������
	{
		buffer = CHAS_MAX_POWER_BUFFER;
		Limit_k = 1;
	}
	else if(buffer >= CHAS_MID_POWER_BUFFER)//һ������
	{
		Limit_k = buffer / CHAS_MAX_POWER_BUFFER;
	}
	else //��������
	{
		Limit_k = buffer / CHAS_MAX_POWER_BUFFER;
		Limit_k *= Limit_k;
	}
	
	(*data) *= Limit_k;
	
}

