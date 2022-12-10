#include "can_protocol.h"


void CAN1_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
	switch(canId)
	{
		case CHASSIS_CAN_ID_LEG_L:
			KT_motor[LEG_L].get_info( &KT_motor[LEG_L], rxBuf );
		
		case CHASSIS_CAN_ID_LEG_R:
			KT_motor[LEG_L].get_info( &KT_motor[LEG_L], rxBuf );
	}

}

void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{



}
