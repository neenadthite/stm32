/*
 * stm32f411xx_gpio_driver.c
 *
 *  Created on: Jan 27, 2024
 *      Author: 141584
 */

#include "stm32f411xx_gpio_driver.h"

/*********************************************************************************************************
 * @fn				- GPIO_PeriClockControl
 *
 * @brief			- This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]		- base address of the gpio peripheral
 * @param[in]		- ENABLE or DISABLE macros
 *
 * @return			- none
 *
 * @Note			- none
 *
 *********************************************************************************************************/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}else if (EnorDi == DISABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DIS();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DIS();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DIS();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DIS();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DIS();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DIS();
		}
	}
}


/*********************************************************************************************************
 * @fn				- GPIO_Init
 *
 * @brief			- This function enables port
 *
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none
 *
 *********************************************************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;

	//Configure the mode of GPIO pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// the non interrupt mode
		temp = (pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode << ( 2 * pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
		pGPIOHandle->pGPIOx->MODER |= temp;
	}else
	{
		//TODO: interrupt mode
	}
	temp = 0;

	//Configure the Speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

	pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	//Configure of PuPd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	//Configure the Output type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	//Configure the Alternate functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		//configure the alt function registers.
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << ( 4 * temp2 ));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * temp2 ));
	}
}

/*********************************************************************************************************
 * @fn				- GPIO_DeInit
 *
 * @brief			- This function enables port
 *
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none
 *
 *********************************************************************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}else if (pGPIOx == GPIOB)
	{
		GPIOA_REG_RESET();
	}
	else if (pGPIOx == GPIOC)
	{
		GPIOA_REG_RESET();
	}
	else if (pGPIOx == GPIOD)
	{
		GPIOA_REG_RESET();
	}
	else if (pGPIOx == GPIOE)
	{
		GPIOA_REG_RESET();
	}
	else if (pGPIOx == GPIOH)
	{
		GPIOA_REG_RESET();
	}
}

/*********************************************************************************************************
 * @fn				- GPIO_ReadFromInputPin
 *
 * @brief			- This function enables port
 *
 * @param[in]		-
 *
 * @return			- 0 or 1
 *
 * @Note			- none
 *
 *********************************************************************************************************/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x000000001);
	return value;
}

/*********************************************************************************************************
 * @fn				- GPIO_ReadFromInputPort
 *
 * @brief			- This function enables port
 *
 * @param[in]		-
 *
 * @return			- 0 or 1
 *
 * @Note			- none
 *
 *********************************************************************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}

/*********************************************************************************************************
 * @fn				- GPIO_WriteToOutputPin
 *
 * @brief			- This function enables port
 *
 * @param[in]		-
 *
 * @return			- 0 or 1
 *
 * @Note			- none
 *
 *********************************************************************************************************/
uint16_t GPIO_WriteToOutpuPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		//write 1 to the output data register at the bit field corresponding pin number
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/*********************************************************************************************************
 * @fn				- GPIO_WriteToOutputPort
 *
 * @brief			- This function enables port
 *
 * @param[in]		-
 *
 * @return			- 0 or 1
 *
 * @Note			- none
 *
 *********************************************************************************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/*********************************************************************************************************
 * @fn				- GPIO_ToggleOutputPin
 *
 * @brief			-
 *
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			- none
 *
 *********************************************************************************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= ( 1 << PinNumber);
}


