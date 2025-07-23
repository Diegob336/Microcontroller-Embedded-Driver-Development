/*
 * main.c
 *
 *  Created on: Jul 15, 2025
 *      Author: diego
 */

#include <string.h>
#include "stm32f103xx.h"

//PB12 -> NSS
//PB13 -> SCK
//PB14 -> MISO
//PB15 -> MOSI

void SPI2_GPIO_init(){
	GPIO_Handle_t spiPins;

	spiPins.pGPIOx = GPIOB;

	spiPins.GPIO_PinConfig.GPIO_Mode_CNF = GPIO_MODE_AF_PP;
	spiPins.GPIO_PinConfig.GPIO_Mode_Speed = GPIO_SPEED_2MHZ;

	GPIO_clk_control(spiPins.pGPIOx, ENABLE);

	//CLK
	spiPins.GPIO_PinConfig.GPIO_PinNumber = 13;

	GPIO_Init(&spiPins);

	//MOSI
	spiPins.GPIO_PinConfig.GPIO_PinNumber = 15;

	GPIO_Init(&spiPins);



}

void SPI2_init(){
	SPI_Handle_t spiHandle;

	spiHandle.pSPIx = SPI2;

	spiHandle.SPI_Config.SPI_DeviceMode = SPI_CR1_MSTR;
	spiHandle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	spiHandle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	spiHandle.SPI_Config.SPI_DFF =SPI_DFF_8BITS;
	spiHandle.SPI_Config.SPI_CPOl = SPI_CPOL_LOW;
	spiHandle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	spiHandle.SPI_Config.SPI_SSM = SPI_SSM_EN;


	SPI_clk_control(spiHandle.pSPIx, ENABLE);
	SPI_Init(&spiHandle);



}


int main(void){

	char userData[] = "Hi mom";

	SPI2_GPIO_init();
	SPI2_init();

	//before sending the data, SPI peripheral must be enabled
	SPI_PeripheralControl(SPI2, ENABLE);

	SPI_SendData(SPI2, (uint8_t *)&userData, strlen(userData));

	while(1);


}
