/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdint.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#define RCC_BASE_ADDRESS 0x40023800UL
#define RCC_CFGR_OFFSET 0x08UL
#define RCC_CFGR_ADDRESS (RCC_BASE_ADDRESS + RCC_CFGR_OFFSET)
#define GPIOA_BASE_ADDRESS 0x40020000

int main(void)
{
	uint32_t *pRCCCfgrReg = (uint32_t*) RCC_CFGR_ADDRESS;
	*pRCCCfgrReg &= ~(0x3 << 21); //clear 21 and 22 bit positions

	//configure MCO1 prescaler
	*pRCCCfgrReg |= ( 1 << 26);
	*pRCCCfgrReg |= ( 1 << 25);

	uint32_t *pRCCAhb1Enr = (uint32_t*)(RCC_BASE_ADDRESS + 0x30);
	*pRCCAhb1Enr |= (1 << 0 );//Enable GPIOA peripheral clock

	//Configure mode of GPIOA pin 8 as alternate function mode

	uint32_t *pGPIOAModeReg = (uint32_t*)(GPIOA_BASE_ADDRESS + 00);
	*pGPIOAModeReg &= ~( 0x3 << 16); //clear
	*pGPIOAModeReg |= ( 0x2 << 16); //set

	//Configure the alternation function register to set the mode 0 for PAB

	uint32_t *pGPIOAAltFunHighReg = (uint32_t*)(GPIOA_BASE_ADDRESS + 0x24);
	*pGPIOAAltFunHighReg &= ~(0xf << 0);

	/* Loop forever */
	for(;;);
}

void I2C1_EV_IRQHandler(void)
{

}
