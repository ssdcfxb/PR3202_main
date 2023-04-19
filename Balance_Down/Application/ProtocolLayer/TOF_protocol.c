#include "TOF_protocol.h"

uint8_t rx[16];

void USART3_rxDataHandler(uint8_t *rxBuf)
{
	if(rxBuf == NULL)
		return;

	memcpy(rx, rxBuf, 16);
	uint8_t device_id = rxBuf[TOF_Rx_DATA_ID_POS];
	
	switch(device_id)
	{
		case POS_F:
			TOF_Sensor[POS_F].rx(&TOF_Sensor[POS_F], rxBuf);
		break;
		
		case POS_B:
			TOF_Sensor[POS_B].rx(&TOF_Sensor[POS_B], rxBuf);
		break;
			
	}
	
}

