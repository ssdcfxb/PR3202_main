#include "can_protocol.h"


void CAN1_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
	switch(canId)
	{
		case CHASSIS_CAN_ID_LEG_L:
			KT_motor[LEG_L].get_info( &KT_motor[LEG_L], rxBuf );
			break;
		
		case CHASSIS_CAN_ID_LEG_R:
			KT_motor[LEG_R].get_info( &KT_motor[LEG_R], rxBuf );
			break;
		
		default:
			break;
	}

}

void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
	switch(canId)
	{
		case MOMENTUM_CAN_ID_L:
			RM_motor[MOMENTUM_L].rx( &RM_motor[MOMENTUM_L], rxBuf );
			break;
		
		case MOMENTUM_CAN_ID_R:
			RM_motor[MOMENTUM_R].rx( &RM_motor[MOMENTUM_R], rxBuf );
			break;
		
		default:
			break;
	}

}
