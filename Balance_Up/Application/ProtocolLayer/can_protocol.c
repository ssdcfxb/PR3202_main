#include "can_protocol.h"


void CAN1_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
	switch(canId)
	{
		case 0x201:
			RM_motor[FRIC_R].rx(&RM_motor[FRIC_R],rxBuf);
			break;
		case 0x202:
			RM_motor[FRIC_L].rx(&RM_motor[FRIC_L],rxBuf);
			break;
		case 0x203:
			RM_motor[DIAL].rx(&RM_motor[DIAL],rxBuf);
			break;
		default:
			break;
	}

}
void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
	switch(canId)
	{
		case 0x205:
			RM_motor[GIM_P].rx(&RM_motor[GIM_P],rxBuf);
			break;
		case 0x206:
			RM_motor[GIM_Y].rx(&RM_motor[GIM_Y],rxBuf);
			break;
		default:
			break;
	}

}
