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
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPI_Config;
	uint8_t      *pTxBuffer;
	uint8_t      *pRxBuffer;
	uint32_t     TxLen;
	uint32_t     RxLen;
	uint8_t      TxState;
	uint8_t      RxState;

}SPI_Handle_t;

/*
 * SPI applicaiton states
 */

#define SPI_READY                     0
#define SPI_BUSY_IN_RX                1
#define SPI_BUSY_IN_TX                2

/*
 * SPI Application events
 */

#define SPI_EVENT_TX_CMPLT            1
#define SPI_EVENT_RX_CMPLT            2
#define SPI_EVENT_OVR_ERR             3


/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER        1
#define SPI_DEVICE_MODE_SLAVE         0

/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD             1
#define SPI_BUS_CONFIG_HD             2
#define SPI_BUS_CONFIG_SIMPLEX_RX     3

/*
 * @SPI_SclkSpeed
 */

#define SPI_SCLK_SPEED_DIV2           0
#define SPI_SCLK_SPEED_DIV4           1
#define SPI_SCLK_SPEED_DIV8           2
#define SPI_SCLK_SPEED_DIV16          3
#define SPI_SCLK_SPEED_DIV32          4
#define SPI_SCLK_SPEED_DIV64          5
#define SPI_SCLK_SPEED_DIV128         6
#define SPI_SCLK_SPEED_DIV256         7

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS                 0
#define SPI_DFF_16bits                1

/*
 * @SPI_CPOL
 */

#define SPI_CPOL_LOW                  0
#define SPI_CPOL_HIGH                 1

/*
 * @SPI_CPHA
 */

#define SPI_CPHA_LOW                  0
#define SPI_CPHA_HIGH                 1

/*
 * @SPI_SSM
 */

#define SPI_SSM_DI                    0
#define SPI_SSM_EN                    1

/*
 * Macros for Status register flags
 */

// SPI_SR bit positions as masks
#define SPI_SR_RXNE     (1 << 0)   // Receive buffer not empty
#define SPI_SR_TXE      (1 << 1)   // Transmit buffer empty
#define SPI_SR_CHSIDE   (1 << 2)   // Channel side (only meaningful in full duplex)
#define SPI_SR_UDR      (1 << 3)   // Underrun flag
#define SPI_SR_CRCERR   (1 << 4)   // CRC error flag
#define SPI_SR_MODF     (1 << 5)   // Mode fault
#define SPI_SR_OVR      (1 << 6)   // Overrun flag
#define SPI_SR_BSY      (1 << 7)   // Busy flag

// SPI_CR1 bit positions
#define SPI_CR1_CPHA           0
#define SPI_CR1_CPOL           1
#define SPI_CR1_MSTR           2
#define SPI_CR1_BR             3  // BR[2:0] spans 3 bits: [5:3]
#define SPI_CR1_SPE            6
#define SPI_CR1_LSBFIRST       7
#define SPI_CR1_SSI            8
#define SPI_CR1_SSM            9
#define SPI_CR1_RXONLY        10
#define SPI_CR1_DFF           11
#define SPI_CR1_CRCNEXT       12
#define SPI_CR1_CRCEN         13
#define SPI_CR1_BIDIOE        14
#define SPI_CR1_BIDIMODE      15

// SPI_CR2 bit positions
#define SPI_CR2_RXDMAEN        0   // Rx buffer DMA enable
#define SPI_CR2_TXDMAEN        1   // Tx buffer DMA enable
#define SPI_CR2_SSOE           2   // SS output enable
#define SPI_CR2_ERRIE          5   // Error interrupt enable
#define SPI_CR2_RXNEIE         6   // RX buffer not empty interrupt enable
#define SPI_CR2_TXEIE          7   // Tx buffer empty interrupt enable

/*
 * SPI peripheral clock control
 */

void SPI_clk_control(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Init and De-init
 */

void SPI_Init(SPI_Handle_t *pSPIHandle);

void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data send and receive
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len);

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len);

/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t priority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

//other APIs
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDI);
void SPI_CLearOVRFLag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHadnle);
void SPI_CloseReception(SPI_Handle_t *pSPIHadnle);

// Applicaition callback

void SPI_ApplicationEventCallBack(SPI_Handle_t *pSPIHadnle, uint8_t AppEvent);


#endif /* INC_STM32F103XX_SPI_DRIVER_H_ */

/*
 * Peripheral Clock setup
 */

