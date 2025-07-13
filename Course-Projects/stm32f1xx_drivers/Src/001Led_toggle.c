/*
 * 001Led_toggle.c
 *
 *  Created on: Jul 12, 2025
 *      Author: diego
 */

#include "../drivers/Inc/stm32f103xx.h"
#include "../drivers/Inc/stm32f103xx_gpio_driver.h"


int main(void){
	GPIO_Handle_t GPIOLed;
	GPIOLed.pGPIOx = GPIOA;
	GPIOLed.GPIO_PinConfig.GPIO_PinNumber = 5;
	GPIOLed.GPIO_PinConfig.GPIO_Mode_CNF = GPIO_MODE_OUT_PP;
	GPIOLed.GPIO_PinConfig.GPIO_Mode_Speed = GPIO_SPEED_2MHZ;
	GPIO_clk_control(GPIOA, 1);
	GPIO_Init(&GPIOLed);
	while (1) {
	        GPIO_ToggleOutputPin(GPIOA, 5);
	        for (volatile int i = 0; i < 100000; i++);  // delay
	    }
}
