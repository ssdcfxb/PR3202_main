#include "imu_protocol.h"

#include "imu_sensor.h"

/**
 *	@brief	在SPI2中解析陀螺仪数据协议
 */
void SPI2_rxDataHandler(uint8_t *rxBuf)
{
	// 更新陀螺仪数据
	imu_sensor.update(&imu_sensor, rxBuf);
}
