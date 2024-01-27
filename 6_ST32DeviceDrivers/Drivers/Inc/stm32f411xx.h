/*
 * stm32f411xx.h
 *
 *  Created on: Jan 26, 2024
 *      Author: Ninad T.
 */

#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_

#include <stdint.h>

#define __vo volatile
/*
 *  base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDRESS			0x08000000U   /* Embedded Flash memory in STM32F411xC/E */
#define SRAM1_BASEADDRESS 			0x20000000U	  /* Physical remap in STM32F411xC/E from RM */
#define ROM_BASEADDRESS				0x1FFF0000U	  /* System memory from Embedded Flash memory section in RM */
#define SRAM_BASEADDRESS			SRAM1_BASEADDRESS

/*
 * AHBx and APBx Bus Peripheral base addresses
 * Memory Mapping from Reference Manual
 */

#define PERIPH_BASE					0x40000000U
#define APB1PERIPH_BASE				PERIPH_BASE
#define APB2PERIPH_BASE				0x40010000U
#define AHB1PERIPH_BASE				0x40020000U
#define AHB2PERIPH_BASE				0x50000000U

/*
 * GPIO peripheral base address which are located on AHB1 bus
 */

#define GPIOA_BASEADDR			(AHB1PERIPH_BASE)
#define GPIOB_BASEADDR			(AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR			(AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR			(AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR			(AHB1PERIPH_BASE + 0x1000)
#define GPIOH_BASEADDR			(AHB1PERIPH_BASE + 0x1C00)

/*
 * Base address for peripherals located on APB1 bus from Memory Map section
 */

#define I2C1_BASEADDR			(APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR			(APB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR			(APB1PERIPH_BASE + 0x5C00)

#define SPI2_BASEADDR			(APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR			(APB1PERIPH_BASE + 0x3C00)

#define USART2_BASEADDR			(APB1PERIPH_BASE + 0x4400)

/*
 * Base address for peripherals located on APB2 bus from Memory Map section
 */

#define USART1_BASEADDR			(APB2PERIPH_BASE + 0x1000)
#define USART6_BASEADDR			(APB2PERIPH_BASE + 0x1400)

#define SPI1_BASEADDR			(APB2PERIPH_BASE + 0x3000)
#define SPI4_BASEADDR			(APB2PERIPH_BASE + 0x3400)
#define SPI5_BASEADDR			(APB2PERIPH_BASE + 0x5000)

#define EXTI_BASEADDR			(APB2PERIPH_BASE + 0x3C00)

/*
 * Peripheral register address mapping
 */

typedef struct
{
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint16_t BSRRL;
	__vo uint16_t BSRRH;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];			/* AFR[0]: GPIO function register low register, AFR[1]: GPIO function high register */
}GPIO_RegDef_t;

typedef struct {
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	uint32_t RESERVED0[2];
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	uint32_t RESERVED1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	uint32_t RESERVED2[2];
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	uint32_t RESERVED3[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	uint32_t RESERVED4[2];
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	uint32_t RESERVED5[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	uint32_t RESERVED6[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	uint32_t RESERVED7;
	__vo uint32_t DCKCFGR;
}RCC_RegDef_t;

/*
 * Peripheral definations typecasted to xx_RegDef
 */

#define GPIOA 					((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB 					((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 					((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 					((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE 					((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF 					((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG 					((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH 					((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI 					((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC_BASEADDR			(AHB1PERIPH_BASE + 0x3800)
#define RCC						((RCC_RegDef_t*)RCC_BASEADDR)

/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN() 	(RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN() 	(RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN() 	(RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN() 	(RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN() 	(RCC->AHB1ENR |= (1<<4))
#define GPIOH_PCLK_EN() 	(RCC->AHB1ENR |= (1<<7))

/*
 * Clock Enable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN() 	(RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN() 	(RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN() 	(RCC->APB1ENR |= (1<<23))


/*
 * Clock Enable Macros for SPIx peripherals
 */

#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN() (RCC->APB1ENR |= (1<<15))
#define SPI4_PCLK_EN() (RCC->APB2ENR |= (1<<13))
#define SPI5_PCLK_EN() (RCC->APB2ENR |= (1<<20))


/*
 * Clock Enable Macros for UARTx peripherals
 */

#define USART1_PCLK_EN() (RCC->APB2ENR |= (1<<4))
#define USART2_PCLK_EN() (RCC->APB1ENR |= (1<<17))
#define USART6_PCLK_EN() (RCC->APB2ENR |= (1<<5))


/*
 * Clock Disable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DIS() 	(RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DIS() 	(RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DIS() 	(RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DIS() 	(RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DIS() 	(RCC->AHB1ENR &= ~(1<<4))
#define GPIOH_PCLK_DIS() 	(RCC->AHB1ENR &= ~(1<<7))

/*
 * Clock Disable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_DIS() 	(RCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DIS() 	(RCC->APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DIS() 	(RCC->APB1ENR &= ~(1<<23))


/*
 * Clock Disable Macros for SPIx peripherals
 */

#define SPI1_PCLK_DIS() (RCC->APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DIS() (RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DIS() (RCC->APB1ENR &= ~(1<<15))
#define SPI4_PCLK_DIS() (RCC->APB2ENR &= ~(1<<13))
#define SPI5_PCLK_DIS() (RCC->APB2ENR &= ~(1<<20))


/*
 * Clock Disable Macros for UARTx peripherals
 */

#define USART1_PCLK_DIS() (RCC->APB2ENR &= ~(1<<4))
#define USART2_PCLK_DIS() (RCC->APB1ENR &= ~(1<<17))
#define USART6_PCLK_DIS() (RCC->APB2ENR &= ~(1<<5))


//some generic macros

#define ENABLE 			1
#define DISABLE 		0
#define SET 			ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET  RESET

#endif /* INC_STM32F411XX_H_ */
