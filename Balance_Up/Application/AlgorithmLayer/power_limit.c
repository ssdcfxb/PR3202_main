#include "Power_Limit.h"


/*-�洫����-*/
void Chassis_Motor_Power_Limit(int16_t *data)
{
	float buffer = judge.info->power_heat_data.chassis_power_buffer;
	float heat_rate,Limit_k, CHAS_LimitOutput, CHAS_TotalOutput;
	
	uint16_t OUT_MAX = 0;
	
	OUT_MAX = CHAS_SP_MAX_OUT * 4;
	
	if(buffer > 60)buffer = 60;//��ֹ����֮�󻺳�250J��Ϊ������ϵ��
	
	Limit_k = buffer / 60;
	
	if(buffer < 25)
		Limit_k = Limit_k * Limit_k ;// * Limit_k; //3��
	else
		Limit_k = Limit_k;// * str->Limit_k; //ƽ��
	
	if(buffer < 60)
		CHAS_LimitOutput = Limit_k * OUT_MAX;
	else 
		CHAS_LimitOutput = OUT_MAX;    
	
	CHAS_TotalOutput = abs(data[0]) + abs(data[1]) + abs(data[2]) + abs(data[3]) ;
	
	heat_rate = CHAS_LimitOutput / CHAS_TotalOutput;
	
  if(CHAS_TotalOutput >= CHAS_LimitOutput)
  {
		for(char i = 0 ; i < 4 ; i++)
		{	
			data[i] = (int16_t)(data[i] * heat_rate);	
		}
	}
}

