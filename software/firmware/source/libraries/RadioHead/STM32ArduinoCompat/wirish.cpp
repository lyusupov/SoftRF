// ArduinoCompat/wirish.cpp
//
// Arduino-like API for STM32F4 Discovery and similar
// using STM32F4xx_DSP_StdPeriph_Lib_V1.3.0

#include <RadioHead.h>
#if (RH_PLATFORM == RH_PLATFORM_STM32STD)
#include <wirish.h>

SerialUSBClass SerialUSB;

// Describes all the STM32 things we need to know about a digital IO pin to
// make it input or output or to configure as an interrupt
typedef struct
{
    uint32_t         ahbperiph;
    GPIO_TypeDef*    port;
    uint16_t         pin;
    uint8_t          extiportsource;
    uint8_t          extipinsource;
} GPIOPin;

// These describe the registers and bits for each digital IO pin to allow us to 
// provide Arduino-like pin addressing, digitalRead etc.
// Indexed by pin number
GPIOPin pins[] = 
{
    { RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_0,  EXTI_PortSourceGPIOA, EXTI_PinSource0  }, // 0 = PA0
    { RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_1,  EXTI_PortSourceGPIOA, EXTI_PinSource1  },
    { RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_2,  EXTI_PortSourceGPIOA, EXTI_PinSource2  },
    { RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_3,  EXTI_PortSourceGPIOA, EXTI_PinSource3  },
    { RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_4,  EXTI_PortSourceGPIOA, EXTI_PinSource4  },
    { RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_5,  EXTI_PortSourceGPIOA, EXTI_PinSource5  },
    { RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_6,  EXTI_PortSourceGPIOA, EXTI_PinSource6  },
    { RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_7,  EXTI_PortSourceGPIOA, EXTI_PinSource7  },
    { RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_8,  EXTI_PortSourceGPIOA, EXTI_PinSource8  },
    { RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_9,  EXTI_PortSourceGPIOA, EXTI_PinSource9  },
    { RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_10, EXTI_PortSourceGPIOA, EXTI_PinSource10 },
    { RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_11, EXTI_PortSourceGPIOA, EXTI_PinSource11 },
    { RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_12, EXTI_PortSourceGPIOA, EXTI_PinSource12 },
    { RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_13, EXTI_PortSourceGPIOA, EXTI_PinSource13 },
    { RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_14, EXTI_PortSourceGPIOA, EXTI_PinSource14 },
    { RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_15, EXTI_PortSourceGPIOA, EXTI_PinSource15 }, // 15 = PA15

    { RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_0,  EXTI_PortSourceGPIOB, EXTI_PinSource0  }, // 16 = PB0
    { RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_1,  EXTI_PortSourceGPIOB, EXTI_PinSource1  },
    { RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_2,  EXTI_PortSourceGPIOB, EXTI_PinSource2  },
    { RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_3,  EXTI_PortSourceGPIOB, EXTI_PinSource3  },
    { RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_4,  EXTI_PortSourceGPIOB, EXTI_PinSource4  },
    { RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_5,  EXTI_PortSourceGPIOB, EXTI_PinSource5  },
    { RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_6,  EXTI_PortSourceGPIOB, EXTI_PinSource6  },
    { RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_7,  EXTI_PortSourceGPIOB, EXTI_PinSource7  },
    { RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_8,  EXTI_PortSourceGPIOB, EXTI_PinSource8  },
    { RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_9,  EXTI_PortSourceGPIOB, EXTI_PinSource9  },
    { RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_10, EXTI_PortSourceGPIOB, EXTI_PinSource10 },
    { RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_11, EXTI_PortSourceGPIOB, EXTI_PinSource11 },
    { RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_12, EXTI_PortSourceGPIOB, EXTI_PinSource12 },
    { RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_13, EXTI_PortSourceGPIOB, EXTI_PinSource13 },
    { RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_14, EXTI_PortSourceGPIOB, EXTI_PinSource14 },
    { RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_15, EXTI_PortSourceGPIOB, EXTI_PinSource15 }, // 31 = PB15

    { RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_0,  EXTI_PortSourceGPIOC, EXTI_PinSource0  }, // 32 = PC0
    { RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_1,  EXTI_PortSourceGPIOC, EXTI_PinSource1  },
    { RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_2,  EXTI_PortSourceGPIOC, EXTI_PinSource2  },
    { RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_3,  EXTI_PortSourceGPIOC, EXTI_PinSource3  },
    { RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_4,  EXTI_PortSourceGPIOC, EXTI_PinSource4  },
    { RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_5,  EXTI_PortSourceGPIOC, EXTI_PinSource5  },
    { RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_6,  EXTI_PortSourceGPIOC, EXTI_PinSource6  },
    { RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_7,  EXTI_PortSourceGPIOC, EXTI_PinSource7  },
    { RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_8,  EXTI_PortSourceGPIOC, EXTI_PinSource8  },
    { RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_9,  EXTI_PortSourceGPIOC, EXTI_PinSource9  },
    { RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_10, EXTI_PortSourceGPIOC, EXTI_PinSource10 },
    { RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_11, EXTI_PortSourceGPIOC, EXTI_PinSource11 },
    { RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_12, EXTI_PortSourceGPIOC, EXTI_PinSource12 },
    { RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_13, EXTI_PortSourceGPIOC, EXTI_PinSource13 },
    { RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_14, EXTI_PortSourceGPIOC, EXTI_PinSource14 },
    { RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_15, EXTI_PortSourceGPIOC, EXTI_PinSource15 }, // 47 = PC15

    { RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_0,  EXTI_PortSourceGPIOD, EXTI_PinSource0  }, // 48 = PD0
    { RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_1,  EXTI_PortSourceGPIOD, EXTI_PinSource1  },
    { RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_2,  EXTI_PortSourceGPIOD, EXTI_PinSource2  },
    { RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_3,  EXTI_PortSourceGPIOD, EXTI_PinSource3  },
    { RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_4,  EXTI_PortSourceGPIOD, EXTI_PinSource4  },
    { RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_5,  EXTI_PortSourceGPIOD, EXTI_PinSource5  },
    { RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_6,  EXTI_PortSourceGPIOD, EXTI_PinSource6  },
    { RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_7,  EXTI_PortSourceGPIOD, EXTI_PinSource7  },
    { RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_8,  EXTI_PortSourceGPIOD, EXTI_PinSource8  },
    { RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_9,  EXTI_PortSourceGPIOD, EXTI_PinSource9  },
    { RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_10, EXTI_PortSourceGPIOD, EXTI_PinSource10 },
    { RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_11, EXTI_PortSourceGPIOD, EXTI_PinSource11 },
    { RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_12, EXTI_PortSourceGPIOD, EXTI_PinSource12 },
    { RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_13, EXTI_PortSourceGPIOD, EXTI_PinSource13 },
    { RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_14, EXTI_PortSourceGPIOD, EXTI_PinSource14 },
    { RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_15, EXTI_PortSourceGPIOD, EXTI_PinSource15 }, // 63 = PD15

    { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_0,  EXTI_PortSourceGPIOE, EXTI_PinSource0  }, // 64 = PE0
    { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_1,  EXTI_PortSourceGPIOE, EXTI_PinSource1  },
    { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_2,  EXTI_PortSourceGPIOE, EXTI_PinSource2  },
    { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_3,  EXTI_PortSourceGPIOE, EXTI_PinSource3  },
    { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_4,  EXTI_PortSourceGPIOE, EXTI_PinSource4  },
    { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_5,  EXTI_PortSourceGPIOE, EXTI_PinSource5  },
    { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_6,  EXTI_PortSourceGPIOE, EXTI_PinSource6  },
    { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_7,  EXTI_PortSourceGPIOE, EXTI_PinSource7  },
    { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_8,  EXTI_PortSourceGPIOE, EXTI_PinSource8  },
    { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_9,  EXTI_PortSourceGPIOE, EXTI_PinSource9  },
    { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_10, EXTI_PortSourceGPIOE, EXTI_PinSource10 },
    { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_11, EXTI_PortSourceGPIOE, EXTI_PinSource11 },
    { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_12, EXTI_PortSourceGPIOE, EXTI_PinSource12 },
    { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_13, EXTI_PortSourceGPIOE, EXTI_PinSource13 },
    { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_14, EXTI_PortSourceGPIOE, EXTI_PinSource14 },
    { RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_15, EXTI_PortSourceGPIOE, EXTI_PinSource15 }, // 79 = PE15

};
#define NUM_PINS (sizeof(pins) / sizeof(GPIOPin))

typedef struct
{
    uint32_t  extiline;
    uint8_t   extiirqn;
    void      (*handler)(void);
} IRQLine;

// IRQ line data indexed by pin source number with its port
// and the programmable handler that will handle interrupts on that line
IRQLine irqlines[] = 
{
    { EXTI_Line0,  EXTI0_IRQn,     0 },
    { EXTI_Line1,  EXTI1_IRQn,     0 },
    { EXTI_Line2,  EXTI2_IRQn,     0 },
    { EXTI_Line3,  EXTI3_IRQn,     0 },
    { EXTI_Line4,  EXTI4_IRQn,     0 },
    { EXTI_Line5,  EXTI9_5_IRQn,   0 },
    { EXTI_Line6,  EXTI9_5_IRQn,   0 },
    { EXTI_Line7,  EXTI9_5_IRQn,   0 },
    { EXTI_Line8,  EXTI9_5_IRQn,   0 },
    { EXTI_Line9,  EXTI9_5_IRQn,   0 },
    { EXTI_Line10, EXTI15_10_IRQn, 0 },
    { EXTI_Line11, EXTI15_10_IRQn, 0 },
    { EXTI_Line12, EXTI15_10_IRQn, 0 },
    { EXTI_Line13, EXTI15_10_IRQn, 0 },
    { EXTI_Line14, EXTI15_10_IRQn, 0 },
    { EXTI_Line15, EXTI15_10_IRQn, 0 },
};

#define NUM_IRQ_LINES (sizeof(irqlines) / sizeof(IRQLine))

// Functions we expect to find in the sketch
extern void setup();
extern void loop();

volatile unsigned long systick_count = 0;

void SysTickConfig()
{
    /* Setup SysTick Timer for 1ms interrupts  */
    if (SysTick_Config(SystemCoreClock / 1000))
    {
	/* Capture error */
	while (1);
    }
    
    /* Configure the SysTick handler priority */
    NVIC_SetPriority(SysTick_IRQn, 0x0);
    // SysTick_Handler will now be called every 1 ms
}

// These interrupt handlers have to be extern C else they dont get linked in to the interrupt vectors
extern "C"
{
    // Called every 1 ms
    void SysTick_Handler(void)
    {
	systick_count++;
    }

    // Interrupt handlers for optional external GPIO interrupts
    void EXTI0_IRQHandler(void)
    {
	if (EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
	    if (irqlines[0].handler)
		irqlines[0].handler();
	    EXTI_ClearITPendingBit(EXTI_Line0);
	}
    }
    void EXTI1_IRQHandler(void)
    {
	if (EXTI_GetITStatus(EXTI_Line1) != RESET)
	{
	    if (irqlines[1].handler)
		irqlines[1].handler();
	    EXTI_ClearITPendingBit(EXTI_Line1);
	}
    }
    void EXTI2_IRQHandler(void)
    {
	if (EXTI_GetITStatus(EXTI_Line2) != RESET)
	{
	    if (irqlines[2].handler)
		irqlines[2].handler();
	    EXTI_ClearITPendingBit(EXTI_Line2);
	}
    }
    void EXTI3_IRQHandler(void)
    {
	if (EXTI_GetITStatus(EXTI_Line3) != RESET)
	{
	    if (irqlines[3].handler)
		irqlines[3].handler();
	    EXTI_ClearITPendingBit(EXTI_Line3);
	}
    }
    void EXTI4_IRQHandler(void)
    {
	if (EXTI_GetITStatus(EXTI_Line4) != RESET)
	{
	    if (irqlines[4].handler)
		irqlines[4].handler();
	    EXTI_ClearITPendingBit(EXTI_Line4);
	}
    }
    void EXTI9_5_IRQHandler(void)
    {
	if (EXTI_GetITStatus(EXTI_Line5) != RESET)
	{
	    if (irqlines[5].handler)
		irqlines[5].handler();
	    EXTI_ClearITPendingBit(EXTI_Line5);
	}
	if (EXTI_GetITStatus(EXTI_Line6) != RESET)
	{
	    if (irqlines[6].handler)
		irqlines[6].handler();
	    EXTI_ClearITPendingBit(EXTI_Line6);
	}
	if (EXTI_GetITStatus(EXTI_Line7) != RESET)
	{
	    if (irqlines[7].handler)
		irqlines[7].handler();
	    EXTI_ClearITPendingBit(EXTI_Line7);
	}
	if (EXTI_GetITStatus(EXTI_Line8) != RESET)
	{
	    if (irqlines[8].handler)
		irqlines[8].handler();
	    EXTI_ClearITPendingBit(EXTI_Line8);
	}
	if (EXTI_GetITStatus(EXTI_Line9) != RESET)
	{
	    if (irqlines[9].handler)
		irqlines[9].handler();
	    EXTI_ClearITPendingBit(EXTI_Line9);
	}
    }
    void EXTI15_10_IRQHandler(void)
    {
	if (EXTI_GetITStatus(EXTI_Line10) != RESET)
	{
	    if (irqlines[10].handler)
		irqlines[10].handler();
	    EXTI_ClearITPendingBit(EXTI_Line10);
	}
	if (EXTI_GetITStatus(EXTI_Line11) != RESET)
	{
	    if (irqlines[11].handler)
		irqlines[11].handler();
	    EXTI_ClearITPendingBit(EXTI_Line11);
	}
	if (EXTI_GetITStatus(EXTI_Line12) != RESET)
	{
	    if (irqlines[12].handler)
		irqlines[12].handler();
	    EXTI_ClearITPendingBit(EXTI_Line12);
	}
	if (EXTI_GetITStatus(EXTI_Line13) != RESET)
	{
	    if (irqlines[13].handler)
		irqlines[13].handler();
	    EXTI_ClearITPendingBit(EXTI_Line13);
	}
	if (EXTI_GetITStatus(EXTI_Line14) != RESET)
	{
	    if (irqlines[14].handler)
		irqlines[14].handler();
	    EXTI_ClearITPendingBit(EXTI_Line14);
	}
	if (EXTI_GetITStatus(EXTI_Line15) != RESET)
	{
	    if (irqlines[15].handler)
		irqlines[15].handler();
	    EXTI_ClearITPendingBit(EXTI_Line15);
	}
    }
}

// The sketch we want to run
//#include "examples/rf22/rf22_client/rf22_client.pde"

// Run the Arduino standard functions in the main loop
int main(int argc, char** argv)
{
    SysTickConfig();
    // Seed the random number generator
//    srand(getpid() ^ (unsigned) time(NULL)/2);
    setup();
    while (1)
	loop();
}

void pinMode(uint8_t pin, WiringPinMode mode)
{
    if (pin > NUM_PINS)
	return;
    // Enable the GPIO clock
    RCC_AHB1PeriphClockCmd(pins[pin].ahbperiph, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = pins[pin].pin;
    if (mode == INPUT)
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; // REVISIT
    else if (mode == OUTPUT)
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; // REVISIT
    else
	return; // Unknown so far
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(pins[pin].port, &GPIO_InitStructure);
}

// This takes about 150ns on STM32F4 Discovery
void digitalWrite(uint8_t pin, uint8_t val)
{
    if (pin > NUM_PINS)
	return;
    if (val)
	GPIO_SetBits(pins[pin].port, pins[pin].pin);
    else
	GPIO_ResetBits(pins[pin].port, pins[pin].pin);
}

uint8_t digitalRead(uint8_t pin)
{
    if (pin > NUM_PINS)
	return 0;
    return GPIO_ReadInputDataBit(pins[pin].port, pins[pin].pin);
}

void attachInterrupt(uint8_t pin, void (*handler)(void), int mode)
{
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // Record the handler to call when the interrupt occurs
    irqlines[pins[pin].extipinsource].handler = handler;

    /* Connect EXTI Line to GPIO Pin */
    SYSCFG_EXTILineConfig(pins[pin].extiportsource, pins[pin].extipinsource);

    /* Configure EXTI line */
    EXTI_InitStructure.EXTI_Line = irqlines[pins[pin].extipinsource].extiline;

    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    if (mode == RISING)
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
    else if (mode == FALLING)
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
    else if (mode == CHANGE)
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable and set EXTI Interrupt to the lowest priority */
    NVIC_InitStructure.NVIC_IRQChannel = irqlines[pins[pin].extipinsource].extiirqn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure); 

    // The relevant EXTI?_IRQHandler
    // will now be called when the pin makes the selected transition
}

void delay(unsigned long ms)
{
    unsigned long start = millis();

    while (millis() - start < ms)
	;
}

unsigned long millis()
{
    return systick_count;
}

long random(long from, long to)
{
    return from + (RNG_GetRandomNumber() % (to - from));
}

long random(long to)
{
    return random(0, to);
}

extern "C"
{
    // These need to be in C land for correct linking
    void _init() {}
    void _fini() {}
}

#endif
