/*
 * stm32f407xx.c
 *
 *  Created on: Jan 27, 2025
 *      Author: fabx
 */


#include "stm32f407xx_gpio_driver.h"

/*
 * @Brief: Allows to enable/disable the clock for the selected GPIO port.
 *
 * @Parameter: pointer to GPIO register structure
 *
 * @Parameter: ENABLE/DISABLE macros for toggle.
 *
 * @return: void
 *
 * @Note: none
 */
void GPIO_CLK(GPIO_Handle_t *pGPIOHandle, uint8_t value)
{
	if (value == ENABLE) {
		if (pGPIOHandle->pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		} else if (pGPIOHandle->pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		} else if (pGPIOHandle->pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		} else if (pGPIOHandle->pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		} else if (pGPIOHandle->pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		} else if (pGPIOHandle->pGPIOx == GPIOF) {
			GPIOF_PCLK_EN();
		} else if (pGPIOHandle->pGPIOx == GPIOG) {
			GPIOG_PCLK_EN();
		} else if (pGPIOHandle->pGPIOx == GPIOH) {
			GPIOH_PCLK_EN();
		} else if (pGPIOHandle->pGPIOx == GPIOI) {
			GPIOI_PCLK_EN();
		}
	} else {
		if (pGPIOHandle->pGPIOx == GPIOA) {
			GPIOA_PCLK_DI();
		} else if (pGPIOHandle->pGPIOx == GPIOB) {
			GPIOB_PCLK_DI();
		} else if (pGPIOHandle->pGPIOx == GPIOB) {
			GPIOB_PCLK_DI();
		} else if (pGPIOHandle->pGPIOx == GPIOC) {
			GPIOC_PCLK_DI();
		} else if (pGPIOHandle->pGPIOx == GPIOD) {
			GPIOD_PCLK_DI();
		} else if (pGPIOHandle->pGPIOx == GPIOE) {
			GPIOE_PCLK_DI();
		} else if (pGPIOHandle->pGPIOx == GPIOF) {
			GPIOF_PCLK_DI();
		} else if (pGPIOHandle->pGPIOx == GPIOG) {
			GPIOG_PCLK_DI();
		} else if (pGPIOHandle->pGPIOx == GPIOH) {
			GPIOH_PCLK_DI();
		} else if (pGPIOHandle->pGPIOx == GPIOI) {
			GPIOI_PCLK_DI();
		}
	}
}


/*
 * @Brief: Allows to configure the following parameters:
 *
 * 			- Pin mode
 * 			- Pin Speed
 * 			- Pin Pull-up/down resistor
 * 			- Alternate Function Mode
 *
 * @Parameter: pointer to GPIO register structure
 *
 * @Parameter: SET/RESET macros for toggle.
 *
 * @return: void
 *
 * @Note: Pins that are represented by 2 or more bits require a shifted value multiplied by the number
 *        of the bits.
 * 		  i.e MODER uses 2 bits to represent a pin, so (2 * PinNumber)
 *
 * 		  If a single bit is required no multiplication is needed.
 *
 * 		  In case of AFRH, the pins are from 8-15 but the bits start at 0,
 * 		  that's why the - 8 operation on the pin number. Those are always from
 * 		  0-7 position in the new register and we don't have to exceed the 32 bits.
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	// clear the bits of selected pin before use.
	pGPIOHandle->pGPIOx->MODER   &= ~(0x3 << pGPIOHandle->PinConfig.PinNumber);
	pGPIOHandle->pGPIOx->OTYPER  &= ~(0x1 << pGPIOHandle->PinConfig.PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->PinConfig.PinNumber);
	pGPIOHandle->pGPIOx->PUPDR   &= ~(0x3 << pGPIOHandle->PinConfig.PinNumber);
	pGPIOHandle->pGPIOx->AFRL    &= ~(0xF << pGPIOHandle->PinConfig.PinNumber);
	pGPIOHandle->pGPIOx->AFRH    &= ~(0xF << pGPIOHandle->PinConfig.PinNumber);

	if (pGPIOHandle->PinConfig.PinMode <= GPIO_MODER_ANALOG) {
		pGPIOHandle->pGPIOx->MODER |= (pGPIOHandle->PinConfig.PinMode << (0x2 * pGPIOHandle->PinConfig.PinNumber));
	} else {
		// TODO: Handle interrupt mode
	}

	pGPIOHandle->pGPIOx->OTYPER  |= (pGPIOHandle->PinConfig.PinOPType      << pGPIOHandle->PinConfig.PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= (pGPIOHandle->PinConfig.PinSpeed       << (0x2 * pGPIOHandle->PinConfig.PinNumber));
	pGPIOHandle->pGPIOx->PUPDR   |= (pGPIOHandle->PinConfig.PinPuPdControl << (0x2 * pGPIOHandle->PinConfig.PinNumber));

	// If pin is in Alternate Function mode, configure the selected mode.
	if (pGPIOHandle->PinConfig.PinMode == GPIO_MODER_ALTFN) {
		if (pGPIOHandle->PinConfig.PinNumber <= GPIO_PIN_7) {
			pGPIOHandle->pGPIOx->AFRL |=
					(pGPIOHandle->PinConfig.PinAltFunMode << (0x4 * pGPIOHandle->PinConfig.PinNumber));
		} else {
			pGPIOHandle->pGPIOx->AFRH |=
					(pGPIOHandle->PinConfig.PinAltFunMode << (0x4 * (pGPIOHandle->PinConfig.PinNumber - 8)));
		}
	}
}

/*
 * @Brief: Reset GPIO registers for a specific port.
 *
 * @Parameter: pointer to GPIO register structure
 *
 * @return: void
 *
 * @Note: none.
 */
void GPIO_DeInit(GPIO_Handle_t *pGPIOHandle)
{
	if (pGPIOHandle->pGPIOx == GPIOA) {
		GPIOA_REG_RESET();
	} else if (pGPIOHandle->pGPIOx == GPIOB) {
		GPIOB_REG_RESET();
	} else if (pGPIOHandle->pGPIOx == GPIOC) {
		GPIOC_REG_RESET();
	} else if (pGPIOHandle->pGPIOx == GPIOD) {
		GPIOD_REG_RESET();
	} else if (pGPIOHandle->pGPIOx == GPIOE) {
		GPIOE_REG_RESET();
	} else if (pGPIOHandle->pGPIOx == GPIOF) {
		GPIOF_REG_RESET();
	} else if (pGPIOHandle->pGPIOx == GPIOG) {
		GPIOG_REG_RESET();
	} else if (pGPIOHandle->pGPIOx == GPIOH) {
		GPIOH_REG_RESET();
	} else if (pGPIOHandle->pGPIOx == GPIOI) {
		GPIOI_REG_RESET();
	}
}


/*
 * @Brief: Read value from Input Data Register pin.
 *         Applies bit extraction technique and masks only the least significant bit LSB.
 *
 * @Parameter: pointer to GPIO handler
 *
 * @Parameter: pin number
 *
 * @return: 0 or 1 at 8 bits
 *
 * @Note: none.
 */
volatile uint8_t GPIO_ReadInputPin(GPIO_Handle_t *pGPIOHandle)
{
	uint8_t value = (uint8_t)(pGPIOHandle->pGPIOx->IDR >> pGPIOHandle->PinConfig.PinNumber) & 0x00000001;
	return value;
}

/*
 * @Brief: Read from whole Input Data Register.
 *         Applies bit extraction technique and masks only
 *         the least significant bit LSB.
 *
 * @Parameter: pointer to GPIO handler
 *
 * @Parameter: pin number
 *
 * @return: 0 or 1 at 16 bit because bit 15-31 are reserved.
 *
 * @Note: none.
 */
volatile uint16_t GPIO_ReadInputPort(GPIO_Handle_t *pGPIOHandle)
{
	uint16_t value = (uint16_t)pGPIOHandle->pGPIOx->IDR;
	return value;
}


/*
 * @Brief: Write bit to output pin
 *
 * @Parameter: pointer to GPIO handler
 *
 * @Parameter: value to indicate if to enable or disable pin.
 *
 * @return: none.
 *
 * @Note: none.
 */
void GPIO_WriteOutputPin(GPIO_Handle_t *pGPIOHandle, uint8_t value)
{
	if (value == ENABLE) {
		pGPIOHandle->pGPIOx->ODR |= (0x1 << pGPIOHandle->PinConfig.PinNumber);
	} else {
		pGPIOHandle->pGPIOx->ODR &= ~(0x1 << pGPIOHandle->PinConfig.PinNumber);
	}
}

/*
 * @Brief: Write to whole output data register
 *
 * @Parameter: pointer to GPIO handler
 *
 * @Parameter: value to write to port.
 *
 * @return: none.
 *
 * @Note: none.
 */
void GPIO_WriteOutputPort(GPIO_Handle_t *pGPIOHandle, uint16_t value)
{
	if (value == ENABLE) {
		pGPIOHandle->pGPIOx->ODR = value;
	}
}

/*
 * @Brief: Toggle a pin with bitwise XOR. (if bits are different returns 1, if equal, 0)
 *
 * @Parameter: pointer to GPIO handler
 *
 * @Parameter: value to toggle a specific pin.
 *
 * @return: none.
 *
 * @Note: none.
 */
void GPIO_TogglePin(GPIO_Handle_t *pGPIOHandler)
{
	pGPIOHandler->pGPIOx->ODR ^= (0x1 << pGPIOHandler->PinConfig.PinNumber);
}


// Interrupt IQR ISR configuration for the NVIC.
void GPIO_IRQConfig(uint8_t IRQNUmber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t pinNumber);
