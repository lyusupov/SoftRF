/*
 * jump2bootloder.c
 *
 *  Created on: 1 Feb 2017
 *      Author: sid
 */

#include <Arduino.h>
//#include <stm32/l1/rcc.h>
//#include <stm32/l1/scb.h>
//#include <stm32/l1/systick.h>

#include "../fanet.h"
#include "serial.h"
#include "jump2dfu.h"

/**
 * @brief  Set the Main Stack Pointer
 *
 * @param  topOfMainStack  Main Stack Pointer
 *
 * Assign the value mainStackPointer to the MSP
 * (main stack pointer) Cortex processor register
 */
//void __set_MSP(uint32_t topOfMainStack) __attribute__( ( naked ) );
//void __set_MSP(uint32_t topOfMainStack)
//{
//	__asm volatile ("MSR msp, %0\n\t"
//			"BX  lr     \n\t" : : "r" (topOfMainStack) );
//}

/**
 * @brief  Set the Priority Mask value
 *
 * @param  priMask  PriMask
 *
 * Set the priority mask bit in the priority mask register
 */
//void __set_PRIMASK(uint32_t priMask)
//{
//	__asm volatile ("MSR primask, %0" : : "r" (priMask) );
//}

void RCC_DeInit(void)
{
	/* Set MSION bit */
//	RCC_CR |= (uint32_t) 0x00000100;

	/* Reset SW[1:0], HPRE[3:0], PPRE1[2:0], PPRE2[2:0], MCOSEL[2:0] and MCOPRE[2:0] bits */
//	RCC_CFGR &= (uint32_t) 0x88FFC00C;

	/* Reset HSION, HSEON, CSSON and PLLON bits */
//	RCC_CR &= (uint32_t) 0xEEFEFFFE;

	/* Reset HSEBYP bit */
//	RCC_CR &= (uint32_t) 0xFFFBFFFF;

	/* Reset PLLSRC, PLLMUL[3:0] and PLLDIV[1:0] bits */
//	RCC_CFGR &= (uint32_t) 0xFF02FFFF;

	/* Disable all interrupts */
//	RCC_CIR = 0x00000000;
}

#define SCB_AIRCR_VECTKEY_Pos              16                                             /*!< SCB AIRCR: VECTKEY Position */
#define SCB_AIRCR_VECTKEY_Msk              (0xFFFFul << SCB_AIRCR_VECTKEY_Pos)            /*!< SCB AIRCR: VECTKEY Mask */

#define SCB_AIRCR_PRIGROUP_Pos              8                                             /*!< SCB AIRCR: PRIGROUP Position */
#define SCB_AIRCR_PRIGROUP_Msk             (7ul << SCB_AIRCR_PRIGROUP_Pos)                /*!< SCB AIRCR: PRIGROUP Mask */

#define SCB_AIRCR_SYSRESETREQ_Pos           2                                             /*!< SCB AIRCR: SYSRESETREQ Position */
#define SCB_AIRCR_SYSRESETREQ_Msk          (1ul << SCB_AIRCR_SYSRESETREQ_Pos)             /*!< SCB AIRCR: SYSRESETREQ Mask */

//static __inline void __DSB() { __asm volatile ("dsb"); }

//static __inline void NVIC_SystemReset(void)
//{
//	SCB_AIRCR = ((0x5FA << SCB_AIRCR_VECTKEY_Pos) | (SCB_AIRCR & SCB_AIRCR_PRIGROUP_Msk) |
//	SCB_AIRCR_SYSRESETREQ_Msk); /* Keep priority group unchanged */
//	__DSB(); /* Ensure completion of memory access */
//	while (1)
//		; /* wait until reset */
//}

/* Define our function pointer */
//void (*SysMemBootJump)(void);

void jump2dfu(bool hardware)
{
//	SerialFANET.end();
#ifdef FANET_USB
//		SerialFANET_USB.end();
#endif

//	if(hardware)
//	{
		/* requires a RC network connected to BOOT0 */
//		while(1)
//		{
//			pinMode(J2D_HW_PIN, OUTPUT);
//			digitalWrite(J2D_HW_PIN, HIGH);
//			delay(10);
//			pinMode(J2D_HW_PIN, INPUT_FLOATING);
//			delay(1000);
//		}

//		NVIC_SystemReset();
//	}

	// Reset the processor
//	SysMemBootJump = (void (*)(void)) (*((uint32_t *) 0x1FF00004)); // Point the PC to the System Memory reset vector (+4)

	/* set default clock source */
//	RCC_DeInit();

	/* Reset Systicks */
//	SysTick->CTRL = 0;		//Reset Systicks
//	SysTick->LOAD = 0;
//	SysTick->VAL = 0;

	/* Disable Interrupts */
//	__set_PRIMASK(1);

	/* Set the main stack point to its default value */
//	__set_MSP(0x20008000);

	/* Execute */
//	SysMemBootJump();
}
