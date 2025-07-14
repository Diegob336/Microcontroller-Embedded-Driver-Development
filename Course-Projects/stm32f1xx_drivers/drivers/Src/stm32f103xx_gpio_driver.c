#include "../../drivers/Inc/stm32f103xx_gpio_driver.h"

#include "../../drivers/Inc/stm32f103xx.h"

uint8_t GPIO_BASEADDR_TO_CODE(GPIO_RegDef_t *pGPIOx);


void GPIO_clk_control(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE){
		if (pGPIOx == AFIO){
			AFIO_CLK_EN();
		}
		else if(pGPIOx == GPIOA){
			IOA_CLK_EN();
		}
		else if (pGPIOx == GPIOB){
			IOB_CLK_EN();
		}
		else if (pGPIOx == GPIOC){
			IOC_CLK_EN();
		}
		else if (pGPIOx == GPIOD){
			IOD_CLK_EN();
		}
	}
	else {
		if (pGPIOx == AFIO){
			AFIO_CLK_DI();
		}
		else if (pGPIOx == GPIOA){
			IOA_CLK_DI();
		}
		else if (pGPIOx == GPIOB){
			IOB_CLK_DI();
		}
		else if (pGPIOx == GPIOC){
			IOC_CLK_DI();
		}
		else if (pGPIOx == GPIOD){
			IOD_CLK_DI();
		}
	}
}

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;
	uint8_t pin = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	if(pGPIOHandle->GPIO_PinConfig.GPIO_Mode_CNF <= 0x0C) {
	// configure the mode/speed of the pin
		if (pin < 8) {
			temp = pGPIOHandle->GPIO_PinConfig.GPIO_Mode_CNF | pGPIOHandle->GPIO_PinConfig.GPIO_Mode_Speed;
			temp = temp << (4 * pin);
			pGPIOHandle->pGPIOx->CRL &= ~(0xF << (4 * pin));
			pGPIOHandle->pGPIOx->CRL |= temp;

		}
		else if (pin >= 8) {
			temp = pGPIOHandle->GPIO_PinConfig.GPIO_Mode_CNF | pGPIOHandle->GPIO_PinConfig.GPIO_Mode_Speed;
			temp = temp << (4 * (pin % 8));
			pGPIOHandle->pGPIOx->CRH &= ~(0xF << (4 * (pin % 8)));
			pGPIOHandle->pGPIOx->CRH |= temp;
		}
	}
	// interrupt mode
	else {
		if(pGPIOHandle->GPIO_PinConfig.GPIO_Mode_CNF == 0x0D){
			// configure the FTSR
			EXTI->FTSR |= (1 << pin);
			// clear the bit corresponding to RSTR
			EXTI->RTSR &= ~(1 << pin);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_Mode_CNF == 0x0E){
			// configure the RTSR
			EXTI->RTSR |= (1 << pin);
			// clear the bit corresponding to FSTR
			EXTI->FTSR &= ~(1 << pin);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_Mode_CNF == 0x0E){
			// configure the FTSR
			EXTI->FTSR |= (1 << pin);
			// clear the bit
			EXTI->RTSR |= (1 << pin);
		}

		AFIO_CLK_EN();
		uint8_t portCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);

		//enable the exti interrupt delivery using IMR
		EXTI->IMR |= (1 << pin);

		//configure the GPIO port selection in SYSCFG_EXTICR
		AFIO->EXTICR[pin / 4] = (portCode << ((pin % 4) * 4));

	}
}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if (pGPIOx == AFIO){
		RCC->APB2RSTR |= (1 << 0);
		RCC->APB2RSTR &= ~(1 << 0);
	}
	else if(pGPIOx == GPIOA){
		RCC->APB2RSTR |= (1 << 2);
		RCC->APB2RSTR &= ~(1 << 2);

	}
	else if (pGPIOx == GPIOB){
		RCC->APB2RSTR |= (1 << 3);
		RCC->APB2RSTR &= ~(1 << 3);
	}
	else if (pGPIOx == GPIOC){
		RCC->APB2RSTR |= (1 << 4);
		RCC->APB2RSTR &= ~(1 << 4);
	}
	else if (pGPIOx == GPIOD){
		RCC->APB2RSTR |= (1 << 5);
		RCC->APB2RSTR &= ~(1 << 5);
	}
}

/*
 * Read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)(pGPIOx->IDR >> PinNumber) & 1;
	return value;

}
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value)
{
	if(value == 1){
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else {
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t value)
{
	pGPIOx->ODR = value;
}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi)
{
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
void GPIO_IRQHandling(uint8_t PinNumber)
{

}

uint8_t GPIO_BASEADDR_TO_CODE(GPIO_RegDef_t *pGPIOx){
	if(pGPIOx == GPIOA){
		return 0;
	}
	else if (pGPIOx == GPIOB){
		return 1;
	}
	else if (pGPIOx == GPIOC){
		return 2;
	}
	else if (pGPIOx == GPIOD){
		return 3;
	}
	return -1;
}
