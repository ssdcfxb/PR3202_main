#include "imu_protocol.h"

#include "imu_sensor.h"

/**
 *	@brief	��SPI2�н�������������Э��
 */
void SPI2_rxDataHandler(uint8_t *rxBuf)
{
	// ��������������
	imu_sensor.update(&imu_sensor, rxBuf);
}
