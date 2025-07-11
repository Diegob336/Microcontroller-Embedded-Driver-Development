#include "stm32f103xx_gpio_driver.h"
#include "stm32f103xx.h"


void GPIO_clk_control(GPIO_RefDef_t *pGPIOx, uint8_t EnorDi)
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
void GPIO_DeInit(GPIO_RefDef_t *pGPIOx)
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
uint8_t GPIO_ReadFromInputPin(GPIO_RefDef_t *pGPIOx, uint8_t PinNumber)
{

}
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef *pGPIOx)
{

}
void GPIO_WriteToOutputPin(GPIO_RefDef_t *pGPIOx, uint8_t PinNumber, uint8_t value)
{

}
void GPIO_WriteToOutputPort(GPIO_RegDef *pGPIOx, uint8_t value)
{

}
void GPIO_ToggleOutputPin(GPIO_RefDef_t *pGPIOx, uint8_t PinNumber)
{

}

/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi)
{

}
void GPIO_IRQHandling(uint8_t PinNumber)
{

}
