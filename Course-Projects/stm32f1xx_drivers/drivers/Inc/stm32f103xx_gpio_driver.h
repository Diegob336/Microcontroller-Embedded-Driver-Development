/*
 * stm32f103xx_gpio_driver.h
 *
 *  Created on: Jul 7, 2025
 *      Author: diego
 */

#ifndef STM32F103XX_GPIO_DRIVER_H_
#define STM32F103XX_GPIO_DRIVER_H_

#include "../../drivers/Inc/stm32f103xx.h"



typedef struct
{
    uint8_t GPIO_PinNumber;      // Pin number (0 to 15)
    uint8_t GPIO_Mode_CNF;           // Input, Output, Alternate, Analog
    uint8_t GPIO_Mode_Speed;          // Output speed: 2MHz, 10MHz, 50MHz
} GPIO_PinConfig_t;


/*
 * Handle structure for a GPIO pin
 */

typedef struct {
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;

}GPIO_Handle_t;


#define GPIO_MODE_IN_ANALOG        0x00  // CNF=00, MODE=00
#define GPIO_MODE_IN_FLOATING      0x04  // CNF=01, MODE=00
#define GPIO_MODE_IN_PU_PD         0x08  // CNF=10, MODE=00

#define GPIO_MODE_OUT_PP           0x00  // CNF=00
#define GPIO_MODE_OUT_OD           0x04  // CNF=01
#define GPIO_MODE_AF_PP            0x08  // CNF=10
#define GPIO_MODE_AF_OD            0x0C  // CNF=11

#define GPIO_SPEED_2MHZ            0x02
#define GPIO_SPEED_10MHZ           0x01
#define GPIO_SPEED_50MHZ           0x03


/*
 * Peripheral Clock setup
 */
void GPIO_clk_control(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* STM32F103XX_GPIO_DRIVER_H_ */
