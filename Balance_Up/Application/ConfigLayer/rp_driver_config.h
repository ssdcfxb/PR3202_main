/**
 * @file        rp_driver_config.h
 * @author      RobotPilots
 * @Version     v1.0
 * @brief       RobotPilots Robots' Driver Configuration.
 * @update
 *              v1.0(7-November-2021)
 */
#ifndef __RP_DRIVER_CONFIG_H
#define __RP_DRIVER_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stdbool.h"

/* Exported macro ------------------------------------------------------------*/
#define configDRV_CAN_USE_MAIL  1

/* Exported types ------------------------------------------------------------*/
/* 驱动层 --------------------------------------------------------------------*/
/**
 *	@brief	驱动类型
 *	@class	driver
 */
typedef enum drv_type{
		DRV_CAN,
		DRV_PWM,
		DRV_SPI,
		DRV_IIC,
		DRV_UART,
} drv_type_t;

/**
 *	@brief	can驱动 id
 *	@class	driver
 */
typedef enum {
    DRV_CAN1,
    DRV_CAN2
} can_id_t;

/**
 *	@brief	iic驱动 id
 *	@class	driver
 */
typedef enum {
    DRV_IIC1
} iic_id_t;

/**
 *	@brief	spi驱动 id
 *	@class	driver
 */
typedef enum {
    DRV_SPI1
} spi_id_t;

/**
 *	@brief	uart驱动 id
 *	@class	driver
 */
typedef enum {
    DRV_UART1,
    DRV_UART2,
    DRV_UART3,
    DRV_UART4,
    DRV_UART5
} uart_id_t;

/**
 *	@brief	iic驱动
 *	@class	driver
 */
typedef struct drv_iic {
    drv_type_t 	type;
		iic_id_t 	id;
} drv_iic_t;

/**
 *	@brief	can驱动
 *	@class	driver
 */
typedef struct 
{
	CAN_HandleTypeDef *hcan;
	drv_type_t	type;
	can_id_t    id;
	uint32_t 		rx_id;
} drv_can_t;

/**
 *	@brief	pwm驱动
 *	@class	driver
 */
typedef struct drv_pwm {
		drv_type_t	type;
		void				(*output)(struct drv_pwm *self, int16_t pwm);
} drv_pwm_t;

/**
 *	@brief	uart驱动
 *	@class	driver
 */
typedef struct drv_uart {
		drv_type_t	type;
    uart_id_t   id;
		void				(*tx_byte)(struct drv_uart *self, uint8_t byte);
} drv_uart_t;

#endif
