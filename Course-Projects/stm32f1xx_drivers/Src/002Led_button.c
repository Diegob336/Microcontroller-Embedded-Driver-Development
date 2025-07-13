/*
 * 002Led_button.c
 *
 *  Created on: Jul 13, 2025
 *      Author: diego
 */


#include "stm32f103xx.h"
#include "stm32f103xx_gpio_driver.h"

void delay(){
	for (uint32_t i = 0; i < 300000; i ++);
}


int main(void){
	GPIO_Handle_t GPIOLedButton;
	GPIO_Handle_t GPIOLed;
	GPIOLedButton.GPIO_PinConfig.GPIO_PinNumber = 13;
	GPIOLedButton.pGPIOx = GPIOC;
	GPIOLedButton.GPIO_PinConfig.GPIO_Mode_CNF = GPIO_MODE_IN_FLOATING;
	GPIOLedButton.GPIO_PinConfig.GPIO_Mode_Speed = 0;

	GPIOLed.pGPIOx = GPIOA;
	GPIOLed.GPIO_PinConfig.GPIO_PinNumber = 5;
	GPIOLed.GPIO_PinConfig.GPIO_Mode_CNF = GPIO_MODE_OUT_PP;
	GPIOLed.GPIO_PinConfig.GPIO_Mode_Speed = GPIO_SPEED_2MHZ;

	GPIO_clk_control(GPIOA, 1);
	GPIO_clk_control(GPIOC, 1);
	GPIO_Init(&GPIOLedButton);
	GPIO_Init(&GPIOLed);



	while (1){
		if (!GPIO_ReadFromInputPin(GPIOC, 13)) {
			GPIO_WriteToOutputPin(GPIOA, 5, 1);
		}
		else {
			GPIO_WriteToOutputPin(GPIOA, 5, 0);
		}
		delay();
	}
}
