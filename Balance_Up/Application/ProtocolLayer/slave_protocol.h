#ifndef __SLAVE_PROTOCOL_H
#define __SLAVE_PROTOCOL_H

#include "rp_config.h"
#include "string.h"

#include "crc.h"
#include "slave.h"

void USART3_rxDataHandler(uint8_t *rxBuf);
	
#endif
