#ifndef INC_STM32F103XX_H_
#define INC_STM32F103XX_H_

#include <stdint.h>

/*
 * base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR         0x08000000U
#define SRAM_BASEADDR          0x20000000U
#define ROM_BASEADDR           0x1FFFF000U

/*
 * AHBx and APBx Bus Peripheral base address
 */

#define PERIPH_BASE            0X40000000U
#define APB1PERIPH_BASE        PERIPH_BASE
#define APB2PERIPH_BASE        0x40010000U
#define AHBPERIPH_BASE         0x40018000U

/*
 * GPIOs Peripheral base addresses
 */
#define GPIOA_BASE             (APB2PERIPH_BASE + 0x0800)
#define GPIOB_BASE             (APB2PERIPH_BASE + 0x0C00)
#define GPIOC_BASE             (APB2PERIPH_BASE + 0x1000)
#define GPIOD_BASE             (APB2PERIPH_BASE + 0x1400)
#define GPIOE_BASE             (APB2PERIPH_BASE + 0x1800)
#define GPIOF_BASE             (APB2PERIPH_BASE + 0x1C00)
#define GPIOG_BASE             (APB2PERIPH_BASE + 0x2000)

/*
 * Peripherals on the APB1 bus, base addresses
 */

#define I2C1_BASE              (APB1PERIPH_BASE + 0x5400)
#define I2C2_BASE              (APB1PERIPH_BASE + 0x5800)
#define SPI2_BASE              (APB1PERIPH_BASE + 0x3800)
#define SPI3_BASE              (APB1PERIPH_BASE + 0x3C00)
#define USART2_BASE            (APB1PERIPH_BASE + 0x4400)
#define USART3_BASE            (APB1PERIPH_BASE + 0x4800)
#define UART4_BASE             (APB1PERIPH_BASE + 0x4C00)
#define UART5_BASE             (APB1PERIPH_BASE + 0x5000)

/*
 * Peripherals on the APB2 bus, base addresses
 */

#define SPI1_BASE              (APB2PERIPH_BASE + 0x3000)
#define USART1_BASE            (APB2PERIPH_BASE + 0x3800)
#define EXTI_BASE              (APB2PERIPH_BASE + 0x0400)
#define AFIO_BASE                  (APB2PERIPH_BASE + 0x0000)

/*
 * Peripherals on the AHB bus, base addresses
 */
#define RCC_BASE               0x40021000U

/*
 * Peripherals definition
 */
#define GPIOA                  ((GPIO_RegDef_t *)GPIOA_BASE)
#define GPIOB                  ((GPIO_RegDef_t *)GPIOB_BASE)
#define GPIOC                  ((GPIO_RegDef_t *)GPIOC_BASE)
#define GPIOD                  ((GPIO_RegDef_t *)GPIOD_BASE)
#define RCC                    ((RCC_RegDef *)RCC_BASE)
#define AFIO                   ((AFIO_RegDef_t *)AFIO_BASE)
#define I2C1                   ((I2C_RegDef_t *)I2C1_BASE)
#define SPI1                   ((SPI_RegDef *)SPI1_BASE)
#define SPI2                   ((SPI_RegDef *)SPI2_BASE)

/*
 * GPIOx peripheral clock enable
 */

#define IOA_CLK_EN()       (RCC->APB2ENR |= (1 << 2))
#define IOB_CLK_EN()       (RCC->APB2ENR |= (1 << 3))
#define IOC_CLK_EN()       (RCC->APB2ENR |= (1 << 4))
#define IOD_CLK_EN()       (RCC->APB2ENR |= (1 << 5))
#define AFIO_CLK_EN()          (RCC->APB2ENR |= (1 << 0))

/* GPIOx peripheral clock disable macros */
#define IOA_CLK_DI()           (RCC->APB2ENR &= ~(1 << 2))
#define IOB_CLK_DI()           (RCC->APB2ENR &= ~(1 << 3))
#define IOC_CLK_DI()           (RCC->APB2ENR &= ~(1 << 4))
#define IOD_CLK_DI()           (RCC->APB2ENR &= ~(1 << 5))
#define AFIO_CLK_DI()          (RCC->APB2ENR &= ~(1 << 0))




typedef struct {
    volatile uint32_t CR;           // 0x00: Clock control register
    volatile uint32_t CFGR;         // 0x04: Clock configuration register
    volatile uint32_t CIR;          // 0x08: Clock interrupt register
    volatile uint32_t APB2RSTR;     // 0x0C: APB2 peripheral reset register
    volatile uint32_t APB1RSTR;     // 0x10: APB1 peripheral reset register
    volatile uint32_t AHBENR;       // 0x14: AHB peripheral clock enable register
    volatile uint32_t APB2ENR;      // 0x18: APB2 peripheral clock enable register
    volatile uint32_t APB1ENR;      // 0x1C: APB1 peripheral clock enable register
    volatile uint32_t BDCR;         // 0x20: Backup domain control register
    volatile uint32_t CSR;          // 0x24: Control/status register
} RCC_RegDef;


typedef struct {
	volatile uint32_t CRL;
	volatile uint32_t CRH;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t BRR;
	volatile uint32_t LCKR;
} GPIO_RegDef_t;

typedef struct {
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t OAR1;
	volatile uint32_t OAR2;
	volatile uint32_t DR;
	volatile uint32_t SR1;
	volatile uint32_t SR2;
	volatile uint32_t CCR;
	volatile uint32_t TRISE;
}I2C_RegDef_t;

typedef struct {
	volatile uint32_t CR1;        // 0x00: Control Register 1
	volatile uint32_t CR2;        // 0x04: Control Register 2
	volatile uint32_t SR;         // 0x08: Status Register
	volatile uint32_t DR;         // 0x0C: Data Register
	volatile uint32_t CRCPR;      // 0x10: CRC Polynomial Register
	volatile uint32_t RXCRCR;     // 0x14: RX CRC Register
	volatile uint32_t TXCRCR;     // 0x18: TX CRC Register
	volatile uint32_t I2SCFGR;    // 0x1C: I2S Configuration Register
	volatile uint32_t I2SPR;      // 0x20: I2S Prescaler Register
} SPI_RegDef;

/*
 * Generic Macros
 */
#define ENABLE		1
#define DISABLE		0
#define SET			ENABLE
#define RESET		DISABLE

#endif /* INC_STM32F103XX_H_ */
