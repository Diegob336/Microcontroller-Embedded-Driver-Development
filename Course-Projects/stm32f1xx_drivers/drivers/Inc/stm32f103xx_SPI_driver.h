/*
 * stm32f103xx_SPI_driver.h
 *
 *  Created on: Jul 21, 2025
 *      Author: diego
 */

#ifndef INC_STM32F103XX_SPI_DRIVER_H_
#define INC_STM32F103XX_SPI_DRIVER_H_

#include "stm32f103xx.h"

typedef struct {
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOl;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;

typedef struct{
	SPI_RegDef *pSPIx;
	SPI_Config_t SPI_Config;
}SPI_Handle_t;
#endif /* INC_STM32F103XX_SPI_DRIVER_H_ */

/*
 * Peripheral Clock setup
 */
void SPI_clk_control(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Init and De-init
 */

void SPI_Init(SPI_RegDef_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data send and receive
 */

void SPI_SendData(SPI_RegDef_t *pSPIHandle);
void SPI_ReceiveData(SPI_RegDef_t *pSPIHandle);
