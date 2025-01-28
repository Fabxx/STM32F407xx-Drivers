/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Jan 27, 2025
 *      Author: fabx
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

/*
 * Structure to hold pin configurations.
 */
typedef struct
{
	uint8_t PinNumber;
	uint8_t PinMode;
	uint8_t PinSpeed;
	uint8_t PinPuPdControl;
	uint8_t PinOPType;
	uint8_t PinAltFunMode;
}Pincfg_t;

/*
 * Handle structure for GPIO, to handle pin configurations or GPIO registries.
 */

typedef struct
{
	GPIO_REGS_t *pGPIOx;
	Pincfg_t PinConfig;
}GPIO_Handle_t;


// SECTION DEDICATED TO ALL MODES FOR EACH REGISTER

// @GPIO_PIN_NUMBERS

#define GPIO_PIN_0		  0
#define GPIO_PIN_1		  1
#define GPIO_PIN_2		  2
#define GPIO_PIN_3		  3
#define GPIO_PIN_4		  4
#define GPIO_PIN_5		  5
#define GPIO_PIN_6		  6
#define GPIO_PIN_7		  7
#define GPIO_PIN_8		  8
#define GPIO_PIN_9		  9
#define GPIO_PIN_10		  10
#define GPIO_PIN_11		  11
#define GPIO_PIN_12		  12
#define GPIO_PIN_13		  13
#define GPIO_PIN_14		  14
#define GPIO_PIN_15		  15


// @GPIO_PIN_MODES.

#define GPIO_MODER_IN     0
#define GPIO_MODER_OUT    1
#define GPIO_MODER_ALTFN  2
#define GPIO_MODER_ANALOG 3

// Alternate Function Values (AF0-AF15)

#define GPIO_AF0		  0
#define GPIO_AF1		  1
#define GPIO_AF2		  2
#define GPIO_AF3		  3
#define GPIO_AF4		  4
#define GPIO_AF5		  5
#define GPIO_AF6		  6
#define GPIO_AF7		  7
#define GPIO_AF8		  8
#define GPIO_AF9		  9
#define GPIO_AF10		  10
#define GPIO_AF11		  11
#define GPIO_AF12		  12
#define GPIO_AF13		  13
#define GPIO_AF14		  14
#define GPIO_AF15		  15

// Pin modes for interrupt handling. Falling/Rising edge.

#define GPIO_MODER_IT_FT  4
#define GPIO_MODER_IT_RT  5
#define GPIO_MODER_IT_RFT 6


// @GPIO_OUTPUT_TYPES

#define GPIO_OTYPER_PP   0
#define GPIO_OTYPER_OD   1

// @GPIO_SPEED_MODES

#define GPIO_SPEEDR_LOW    0
#define GPIO_SPEEDR_MEDIUM 1
#define GPIO_SPEEDR_HIGH   2
#define GPIO_SPEEDR_VHIGH  3

// @GPIO_PULLUPDOWN_MODES

#define GPIO_PUPDR_NONE   0
#define GPIO_PUPDR_PUP    1
#define GPIO_PUPDR_PDOWN  2

/*
 * Function prototypes to work with GPIO
 */

void GPIO_CLK(GPIO_Handle_t *pGPIOHandle, uint8_t value);
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_Handle_t *pGPIOHandle);

uint8_t GPIO_ReadInputPin(GPIO_Handle_t *PGPIOHandle);
uint16_t GPIO_ReadInputPort(GPIO_Handle_t *PGPIOHandle);

void GPIO_WriteOutputPin(GPIO_Handle_t *PGPIOHandle, uint8_t value);
void GPIO_WriteOutputPort(GPIO_Handle_t *PGPIOHandle, uint16_t value);
uint8_t GPIO_ReadOutputPin(GPIO_Handle_t *pGPIOHandle);
void GPIO_TogglePin(GPIO_Handle_t *PGPIOHandle);
void GPIO_IRQConfig(uint8_t IRQNUmber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t pinNumber);

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
