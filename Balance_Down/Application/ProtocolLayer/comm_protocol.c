#include "comm_protocol.h"


//ң�����������������ط��͸������أ����Դ�2������

void USART1_rxDataHandler(uint8_t *rxBuf)
{	
	if(rxBuf == NULL)
		return;
	
	Master.rx(rxBuf);

}

