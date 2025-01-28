/*
 * stm32f407xx.h
 *
 *  Created on: Jan 27, 2025
 *      Author: fabx
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

typedef struct
{
	volatile uint32_t MODER;			 // GPIO PORT MODE REGISTER Offset 0x00
	volatile uint32_t OTYPER;			 // GPIO OUTPUT TYPE REGISTER Offset 0x04
	volatile uint32_t OSPEEDR;			 // GPIO OUTPUT SPEED REGISTER Offset 0x08
	volatile uint32_t PUPDR;			 // GPIO PULL-UP/DOWN REGISTER Offset 0x0C
	volatile uint32_t IDR;   // GPIO INPUT DATA REGISTER Offset 0x10 (READ ONLY)
	volatile uint32_t ODR;				 // GPIO OUTPUT DATA REGISTER Offset 0x14
	volatile uint32_t BSSR;				 // GPIO BIT SET/RESET REGISTER Offset 0x18
	volatile uint32_t LCKR;              // GPIO PORT CONFIG LOCK REGISTER Offset 0x1C
	volatile uint32_t AFRL;				 // ALTERNATE FUNCTION LOW REGISTER Offset  0x20
	volatile uint32_t AFRH;              // ALTERNATE FUNCTION HIGH REGISTER Offset  0x24

}GPIO_REGS_t;

/*
 * Structure for RCC Registers
 */

typedef struct
{
	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t AHB3RSTR;
	volatile uint32_t RESERVED_1;
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	volatile uint32_t RESERVED_2;
	volatile uint32_t RESERVED_3;
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
}RCC_REGS_t;


/*
 * Base addresses of FLASH and SRAM
 *
 * Note: Addresses can be found in either reference manual or data sheet, "Memory map" section
 *
 * the ROM address is the "System Memory" address in the flash interface memory.
 */

#define DRV_FLASH_BASEADDR              0x08000000U
#define DRV_ROM                         0x1FFF0000U // System Memory Address
#define DRV_SRAM                        DRV_SRAM1_BASEADDR
#define DRV_SRAM1_BASEADDR              0x20000000U
#define DRV_SRAM2_BASEADDR              0x2001C000U

#define DRV_AHB1PERIPH_BASE             0x40020000U

/*
 * Base addresses of AHB1 Peripherals
*/

#define DRV_RCC_BASEADDR                (DRV_AHB1PERIPH_BASE + 0x3800U)

#define DRV_GPIOA_BASEADDR               DRV_AHB1PERIPH_BASE
#define DRV_GPIOB_BASEADDR              (DRV_AHB1PERIPH_BASE + 0x0400U)
#define DRV_GPIOC_BASEADDR              (DRV_AHB1PERIPH_BASE + 0x0800U)
#define DRV_GPIOD_BASEADDR              (DRV_AHB1PERIPH_BASE + 0x0C00U)
#define DRV_GPIOE_BASEADDR              (DRV_AHB1PERIPH_BASE + 0x1000U)
#define DRV_GPIOF_BASEADDR              (DRV_AHB1PERIPH_BASE + 0x1400U)
#define DRV_GPIOG_BASEADDR              (DRV_AHB1PERIPH_BASE + 0x1800U)
#define DRV_GPIOH_BASEADDR              (DRV_AHB1PERIPH_BASE + 0x1C00U)
#define DRV_GPIOI_BASEADDR              (DRV_AHB1PERIPH_BASE + 0x2000U)


/*
 * Type casted macros to initialize struct. These basically say:
 * initialize GPIO registers on that specific port.
*/

#define GPIOA							((GPIO_REGS_t*)DRV_GPIOA_BASEADDR)
#define GPIOB							((GPIO_REGS_t*)DRV_GPIOB_BASEADDR)
#define GPIOC							((GPIO_REGS_t*)DRV_GPIOC_BASEADDR)
#define GPIOD							((GPIO_REGS_t*)DRV_GPIOD_BASEADDR)
#define GPIOE							((GPIO_REGS_t*)DRV_GPIOE_BASEADDR)
#define GPIOF							((GPIO_REGS_t*)DRV_GPIOF_BASEADDR)
#define GPIOG							((GPIO_REGS_t*)DRV_GPIOG_BASEADDR)
#define GPIOH							((GPIO_REGS_t*)DRV_GPIOH_BASEADDR)
#define GPIOI							((GPIO_REGS_t*)DRV_GPIOI_BASEADDR)
#define GPIOJ							((GPIO_REGS_t*)DRV_GPIOJ_BASEADDR)
#define GPIOK							((GPIO_REGS_t*)DRV_GPIOK_BASEADDR)

#define RCC								((RCC_REGS_t*)DRV_RCC_BASEADDR)

/*
 * Macro to enable RCC Clock for GPIO Ports.
 */

#define GPIOA_PCLK_EN()				   (RCC->AHB1ENR |= (0x1 << 0))
#define GPIOB_PCLK_EN()				   (RCC->AHB1ENR |= (0x1 << 1))
#define GPIOC_PCLK_EN()				   (RCC->AHB1ENR |= (0x1 << 2))
#define GPIOD_PCLK_EN()				   (RCC->AHB1ENR |= (0x1 << 3))
#define GPIOE_PCLK_EN()				   (RCC->AHB1ENR |= (0x1 << 4))
#define GPIOF_PCLK_EN()				   (RCC->AHB1ENR |= (0x1 << 5))
#define GPIOG_PCLK_EN()				   (RCC->AHB1ENR |= (0x1 << 6))
#define GPIOH_PCLK_EN()				   (RCC->AHB1ENR |= (0x1 << 7))
#define GPIOI_PCLK_EN()				   (RCC->AHB1ENR |= (0x1 << 8))

/*
 * Macro to disable RCC CLock for GPIO Ports
 */

#define GPIOA_PCLK_DI()				   (RCC->AHB1ENR &= ~(0x1 << 0))
#define GPIOB_PCLK_DI()				   (RCC->AHB1ENR &= ~(0x1 << 1))
#define GPIOC_PCLK_DI()				   (RCC->AHB1ENR &= ~(0x1 << 2))
#define GPIOD_PCLK_DI()				   (RCC->AHB1ENR &= ~(0x1 << 3))
#define GPIOE_PCLK_DI()				   (RCC->AHB1ENR &= ~(0x1 << 4))
#define GPIOF_PCLK_DI()				   (RCC->AHB1ENR &= ~(0x1 << 5))
#define GPIOG_PCLK_DI()				   (RCC->AHB1ENR &= ~(0x1 << 6))
#define GPIOH_PCLK_DI()				   (RCC->AHB1ENR &= ~(0x1 << 7))
#define GPIOI_PCLK_DI()				   (RCC->AHB1ENR &= ~(0x1 << 8))

/*
 * Macro to reset GPIO Registers. Set bit to 1 for reset, then to 0 to not reset anymore.
 * do-while loop used to execute multiple statements in a macro.
 */

#define GPIOA_REG_RESET() do { RCC->AHB1RSTR |= (0x1 << 0); RCC->AHB1RSTR &= ~(0x1 << 0); } while(0)
#define GPIOB_REG_RESET() do { RCC->AHB1RSTR |= (0x1 << 1); RCC->AHB1RSTR &= ~(0x1 << 1); } while(0)
#define GPIOC_REG_RESET() do { RCC->AHB1RSTR |= (0x1 << 2); RCC->AHB1RSTR &= ~(0x1 << 2); } while(0)
#define GPIOD_REG_RESET() do { RCC->AHB1RSTR |= (0x1 << 3); RCC->AHB1RSTR &= ~(0x1 << 3); } while(0)
#define GPIOE_REG_RESET() do { RCC->AHB1RSTR |= (0x1 << 4); RCC->AHB1RSTR &= ~(0x1 << 4); } while(0)
#define GPIOF_REG_RESET() do { RCC->AHB1RSTR |= (0x1 << 5); RCC->AHB1RSTR &= ~(0x1 << 5); } while(0)
#define GPIOG_REG_RESET() do { RCC->AHB1RSTR |= (0x1 << 6); RCC->AHB1RSTR &= ~(0x1 << 6); } while(0)
#define GPIOH_REG_RESET() do { RCC->AHB1RSTR |= (0x1 << 7); RCC->AHB1RSTR &= ~(0x1 << 7); } while(0)
#define GPIOI_REG_RESET() do { RCC->AHB1RSTR |= (0x1 << 8); RCC->AHB1RSTR &= ~(0x1 << 8); } while(0)



// Generic macro

#define ENABLE 1
#define DISABLE 0

#endif /* INC_STM32F407XX_H_ */
