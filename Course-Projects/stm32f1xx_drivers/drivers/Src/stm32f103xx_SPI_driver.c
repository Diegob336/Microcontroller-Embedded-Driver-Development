/*
 * stm32f103xx_SPI_driver.c
 *
 *  Created on: Jul 21, 2025
 *      Author: diego
 */

#include "stm32f103xx_SPI_driver.h"
/*
 * Peripheral Clock setup
 */
void SPI_clk_control(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if (EnorDi == ENABLE){
		if (pSPIx == SPI1){
			SPI1_CLk_EN();
		}
		else if (pSPIx == SPI2){
			SPI2_CLk_EN();
		}
		else if (pSPIx == SPI3){
			SPI3_CLk_EN();
		}
	}
	else {

	}

}

/*
 * Init and De-init
 */

void SPI_Init(SPI_Handle_t *pSPIHandle){
	//configure the device mode
	// peripheral will be configured in slave mode by default
	uint8_t temp = 0;
	temp |= (pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR);


	//configure the bus
	if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		temp &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		temp |= (1 << SPI_CR1_BIDIMODE);
	}
	else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RX){
		temp &= ~(1 << SPI_CR1_BIDIMODE);
		temp |= (1 << SPI_CR1_RXONLY);
	}

	//configure the clock speed
	temp |= (pSPIHandle->SPI_Config.SPI_SclkSpeed << SPI_CR1_BR);

	//configure the data frame rate
	temp |= (pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF);

	// configure CPOL
	temp |= (pSPIHandle->SPI_Config.SPI_CPOl << SPI_CR1_CPOL);

	// configure CPOL
	temp |= (pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA);


	pSPIHandle->pSPIx->CR1 = temp;

}

void SPI_DeInit(SPI_RegDef_t *pSPIx){

}

/*
 * Data send and receive
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len){

	while(len > 0){
		//wait until tx buffer is empty
		while(!(pSPIx->SR & SPI_SR_TXE));

		uint8_t dff = (pSPIx->CR1 >> SPI_CR1_DFF);
		if ((dff & 1) == SPI_DFF_16bits) {
			if (len >= 2){
				pSPIx->DR = *((uint16_t *)pTxBuffer);
				len -= 2;
				pTxBuffer += 2;

			}
			else {
				pSPIx->DR = *(pTxBuffer);
				len--;
				pTxBuffer++;
			}

		}
		else {
			pSPIx->DR = *(pTxBuffer);
			len--;
			pTxBuffer++;
			}


	}
}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer){

}

/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){

}
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t priority){

}
void SPi_IRQHandling(SPI_Handle_t *pSPIHandle){

}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t Flag){

}
