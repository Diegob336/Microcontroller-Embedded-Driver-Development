/*
 * stm32f103xx_SPI_driver.c
 *
 *  Created on: Jul 21, 2025
 *      Author: diego
 */

#include "stm32f103xx_SPI_driver.h"

void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

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
 * Data send and receive polling based
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

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len){
	while (len > 0){
		while (!(pSPIx->SR & SPI_SR_RXNE));

		if (pSPIx->CR1 & ( 1 << SPI_CR1_DFF)){
			*((uint16_t *)pRxBuffer) = pSPIx->DR;
			pRxBuffer += 2;
			len -= 2;
		}
		else {
			*pRxBuffer = pSPIx->DR;
			pRxBuffer++;
			len--;
		}
	}
}

/*
 * Data send and receive interrupt based
 */

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len){

	uint8_t state = pSPIHandle->TxState;
	if (state != SPI_BUSY_IN_TX){
		// save the tx buffer address and len information
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = len;

		/*
		 * make the SPI state as busy in transmission so that
		 * no other code takes the same peripheral until transmission is over
		 */

		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//enable the TXEIE control bit to get interrupts
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);


	}

	return state;

}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len){

	uint8_t state = pSPIHandle->RxState;
	if (state != SPI_BUSY_IN_RX){
		// save the Rx buffer address and len information
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = len;

		/*
		 * make the SPI state as busy in reception so that
		 * no other code takes the same peripheral until reception is over
		 */

		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//enable the RXNEIE control bit to get interrupts
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);


	}

	return state;

}

/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){
	if (EnorDi == 1){
		if (IRQNumber <= 31){
			//program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber < 64){
			//program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber >= 64 && IRQNumber < 96){
			//program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 32));
		}
	}
	else {
		if (IRQNumber <= 31){
			//program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber < 64){
			//program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber >= 64 && IRQNumber < 96){
			//program ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber % 32));
		}
	}
}
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t priority){
	volatile uint32_t *PR_Register = NVIC_PR_BASEADDR;
	uint8_t ipr_index = IRQNumber / 4;
	uint8_t section_offset = (IRQNumber % 4) * 8;
	*(PR_Register+ (ipr_index)) &= ~(0xFF << section_offset);
	*(PR_Register+ (ipr_index)) |= (priority << (section_offset + (8 - NO_PR_BITS_IMPLEMENTED)));
}
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle){
	uint8_t status1, status2;

	//Check for TXE
	status1 = pSPIHandle->pSPIx->SR & SPI_SR_TXE;
	//
	status2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if (status1 && status2){
		spi_txe_interrupt_handle(pSPIHandle);
	}

	//Check for RXNE
	status1 = pSPIHandle->pSPIx->SR & SPI_SR_RXNE;
	//
	status2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if (status1 && status2){
		spi_rxne_interrupt_handle(pSPIHandle);
	}

	//Check for OVR Flag
	status1 = pSPIHandle->pSPIx->SR & SPI_SR_OVR;
	//
	status2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if (status1 && status2){
		spi_ovr_err_interrupt_handle(pSPIHandle);
	}
}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDI){
	if (EnorDI == ENABLE){
		pSPIx->CR1 |= (1 << 6);
	}
	else {
		pSPIx->CR1 &= ~(1 << 6);
	}
}

/*
 * Helper functionns
 */

void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle){

	uint8_t dff = (pSPIHandle->pSPIx->CR1 >> SPI_CR1_DFF);
	if ((dff & 1) == SPI_DFF_16bits) {
		if (pSPIHandle->TxLen >= 2){
			pSPIHandle->pSPIx->DR = *((uint16_t *)pSPIHandle->pTxBuffer);
			pSPIHandle->TxLen -= 2;
			pSPIHandle->pTxBuffer += 2;

		}
		else {
			pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
			pSPIHandle->TxLen--;
			pSPIHandle->pTxBuffer++;
		}

	}
	else {
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
		}

	if (pSPIHandle->TxLen <= 0 ){
		// close the spi transmission
		// turn off the tx interrupt

		pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
		pSPIHandle->pTxBuffer = NULL;
		pSPIHandle->TxLen = 0;
		pSPIHandle->TxState = SPI_READY;

		SPI_ApplicationEventCallBack(pSPIHandle, SPI_EVENT_TX_CMPLT);

	}

}

void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle){

	if (pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF)){
		*((uint16_t *)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->pRxBuffer += 2;
		pSPIHandle->RxLen -= 2;
	}
	else {
		*(pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->pRxBuffer++;
		pSPIHandle->RxLen--;
	}

	if (pSPIHandle->RxLen <= 0){
		pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
		pSPIHandle->pRxBuffer = NULL;
		pSPIHandle->RxLen = 0;
		pSPIHandle->RxState = SPI_READY;

		SPI_ApplicationEventCallBack(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}

}

void spi_ovr_err_interrupt_handle(SPI_Handle_t *SPIHandle){

}
