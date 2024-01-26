/*
 * stm32f411xx.h
 *
 *  Created on: Jan 26, 2024
 *      Author: Ninad T.
 */

#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_

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
#define GPIOF_BASEADDR			(AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR			(AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR			(AHB1PERIPH_BASE + 0x1C00)
#define GPIOI_BASEADDR			(AHB1PERIPH_BASE + 0x2000)

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

/*
 * Peripheral definations typecasted to xx_RegDef
 */

#define GPIOA 			((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB 			((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 			((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 			((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE 			((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF 			((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG 			((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH 			((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI 			((GPIO_RegDef_t*)GPIOI_BASEADDR)



#endif /* INC_STM32F411XX_H_ */
