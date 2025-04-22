// RH_ASK.cpp
//
// Copyright (C) 2014 Mike McCauley
// $Id: RH_ASK.cpp,v 1.32 2020/08/04 09:02:14 mikem Exp $

#include <RH_ASK.h>
#include <RHCRC.h>

#ifndef __SAMD51__

#if (RH_PLATFORM == RH_PLATFORM_STM32)
    // Maple etc
  HardwareTimer timer(MAPLE_TIMER);
#elif defined(ARDUINO_ARCH_RP2040)

#elif defined(BOARD_NAME)
  // ST's Arduino Core STM32, https://github.com/stm32duino/Arduino_Core_STM32
 #if defined(RH_HW_TIMER)
 // Can define your own timer name based on macros defs passed to compiler eg in platformio.ini
  HardwareTimer timer(RH_HW_TIMER);
 #else
  HardwareTimer timer(TIM1);
 #endif
    
#elif defined(ARDUINO_ARCH_STM32) || defined(ARDUINO_ARCH_STM32F1) || defined(ARDUINO_ARCH_STM32F3) || defined(ARDUINO_ARCH_STM32F4)
  // Roger Clark Arduino STM32, https://github.com/rogerclarkmelbourne/Arduino_STM32
  // And stm32duino    
  HardwareTimer timer(1);

#elif defined(ARDUINO_UNOR4_MINIMA) || defined(ARDUINO_UNOR4_WIFI)
  #include "FspTimer.h"
  FspTimer ask_timer;

#elif (RH_PLATFORM == RH_PLATFORM_ARDUINO) && defined(RH_CUBE_CELL_BOARD)
    static TimerEvent_t timer;
#endif


#if (RH_PLATFORM == RH_PLATFORM_ESP32)
  // Michael Cain
  DRAM_ATTR hw_timer_t * timer;
  //jPerotto Non-constant static data from ESP32 https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/general-notes.html#dram-data-ram
   #define RH_DRAM_ATTR DRAM_ATTR
#else
   #define RH_DRAM_ATTR 
#endif

// RH_ASK on Arduino uses Timer 1 to generate interrupts 8 times per bit interval
// Define RH_ASK_ARDUINO_USE_TIMER2 if you want to use Timer 2 instead of Timer 1 on Arduino
// You may need this to work around other libraries that insist on using timer 1
// Should be moved to header file
//#define RH_ASK_ARDUINO_USE_TIMER2

// RH_ASK on ATtiny8x uses Timer 0 to generate interrupts 8 times per bit interval. 
// Timer 0 is used by Arduino platform for millis()/micros() which is used by delay()
// Uncomment the define RH_ASK_ATTINY_USE_TIMER1 bellow, if you want to use Timer 1 instead of Timer 0 on ATtiny
// Timer 1 is also used by some other libraries, e.g. Servo. Alway check usage of Timer 1 before enabling this.
//  Should be moved to header file
//#define RH_ASK_ATTINY_USE_TIMER1

// Interrupt handler uses this to find the most recently initialised instance of this driver
static RH_ASK* thisASKDriver;

// 4 bit to 6 bit symbol converter table
// Used to convert the high and low nybbles of the transmitted data
// into 6 bit symbols for transmission. Each 6-bit symbol has 3 1s and 3 0s 
// with at most 3 consecutive identical bits
RH_DRAM_ATTR static uint8_t symbols[] =
{
    0xd,  0xe,  0x13, 0x15, 0x16, 0x19, 0x1a, 0x1c, 
    0x23, 0x25, 0x26, 0x29, 0x2a, 0x2c, 0x32, 0x34
};

// This is the value of the start symbol after 6-bit conversion and nybble swapping
#define RH_ASK_START_SYMBOL 0xb38

RH_ASK::RH_ASK(uint16_t speed, uint8_t rxPin, uint8_t txPin, uint8_t pttPin, bool pttInverted)
    :
    _speed(speed),
    _rxPin(rxPin),
    _txPin(txPin),
    _pttPin(pttPin),
    _rxInverted(false),
    _pttInverted(pttInverted)
{
    // Initialise the first 8 nibbles of the tx buffer to be the standard
    // preamble. We will append messages after that. 0x38, 0x2c is the start symbol before
    // 6-bit conversion to RH_ASK_START_SYMBOL
    uint8_t preamble[RH_ASK_PREAMBLE_LEN] = {0x2a, 0x2a, 0x2a, 0x2a, 0x2a, 0x2a, 0x38, 0x2c};
    memcpy(_txBuf, preamble, sizeof(preamble));
}

bool RH_ASK::init()
{
    if (!RHGenericDriver::init())
	return false;
    thisASKDriver = this;

#if (RH_PLATFORM == RH_PLATFORM_GENERIC_AVR8)
 #ifdef RH_ASK_PTT_PIN 				
    RH_ASK_PTT_DDR  |=  (1<<RH_ASK_PTT_PIN); 
    RH_ASK_TX_DDR   |=  (1<<RH_ASK_TX_PIN);
    RH_ASK_RX_DDR   &= ~(1<<RH_ASK_RX_PIN);
 #else
    RH_ASK_TX_DDR   |=  (1<<RH_ASK_TX_PIN);
    RH_ASK_RX_DDR   &= ~(1<<RH_ASK_RX_PIN);
 #endif
#else
    // Set up digital IO pins for arduino
    pinMode(_txPin, OUTPUT);
    pinMode(_rxPin, INPUT);
    pinMode(_pttPin, OUTPUT);
#endif

    // Ready to go
    setModeIdle();
    timerSetup();

    return true;
}

// Put these prescaler structs in PROGMEM, not on the stack
#if (RH_PLATFORM == RH_PLATFORM_ARDUINO) || (RH_PLATFORM == RH_PLATFORM_GENERIC_AVR8) || (RH_PLATFORM == RH_PLATFORM_ATTINY) 
 #if defined(RH_ASK_ARDUINO_USE_TIMER2)
 // Timer 2 has different prescalers
 PROGMEM static const uint16_t prescalers[] = {0, 1, 8, 32, 64, 128, 256, 3333}; 
#elif defined(RH_ASK_ATTINY_USE_TIMER1) && defined(TCCR1)
 // ATtiny85 Timer1 prescalers
 PROGMEM static const uint16_t prescalers[] = {0, 1, 2, 4, 8, 16, 32, 64, 128, 256, 1024, 2048, 4096, 8192, 16384, 33333}; 
 #else
 PROGMEM static const uint16_t prescalers[] = {0, 1, 8, 64, 256, 1024, 3333}; 
 #endif
 #define NUM_PRESCALERS (sizeof(prescalers) / sizeof( uint16_t))
#endif

// Common function for setting timer ticks @ prescaler values for speed
// Returns prescaler index into {0, 1, 8, 64, 256, 1024} array
// and sets nticks to compare-match value if lower than max_ticks
// returns 0 & nticks = 0 on fault
uint8_t RH_ASK::timerCalc(uint16_t speed, uint16_t max_ticks, uint16_t *nticks)
{
#if (RH_PLATFORM == RH_PLATFORM_ARDUINO && !defined(ARDUINO_ARCH_RP2040) && !defined(RH_CUBE_CELL_BOARD)) || (RH_PLATFORM == RH_PLATFORM_GENERIC_AVR8) || (RH_PLATFORM == RH_PLATFORM_ATTINY)
    // Clock divider (prescaler) values - 0/3333: error flag
    uint8_t prescaler;     // index into array & return bit value
    unsigned long ulticks; // calculate by ntick overflow

    // Div-by-zero protection
    if (speed == 0)
    {
        // signal fault
        *nticks = 0;
        return 0;
    }

    // test increasing prescaler (divisor), decreasing ulticks until no overflow
    // 1/Fraction of second needed to xmit one bit
    unsigned long inv_bit_time = ((unsigned long)speed) * 8;
    for (prescaler = 1; prescaler < NUM_PRESCALERS; prescaler += 1)
    {
	// Integer arithmetic courtesy Jim Remington
	// 1/Amount of time per CPU clock tick (in seconds)
	uint16_t prescalerValue;
	memcpy_P(&prescalerValue, &prescalers[prescaler], sizeof(uint16_t));
        unsigned long inv_clock_time = F_CPU / ((unsigned long)prescalerValue);
        // number of prescaled ticks needed to handle bit time @ speed
        ulticks = inv_clock_time / inv_bit_time;

        // Test if ulticks fits in nticks bitwidth (with 1-tick safety margin)
        if ((ulticks > 1) && (ulticks < max_ticks))
            break; // found prescaler

        // Won't fit, check with next prescaler value
    }


    // Check for error
    if ((prescaler == 6) || (ulticks < 2) || (ulticks > max_ticks))
    {
        // signal fault
        *nticks = 0;
        return 0;
    }

    *nticks = ulticks;
    return prescaler;
#else
    return 0; // not implemented or needed on other platforms
#endif
}

#if defined(RH_PLATFORM_ARDUINO) && defined(ARDUINO_ARCH_RP2040)
void set_pico_alarm(uint32_t speed)
{
    uint32_t period = (1000000 / 8) / speed; // In microseconds
    uint64_t target = timer_hw->timerawl + period;
    timer_hw->alarm[RH_ASK_PICO_ALARM_NUM] = (uint32_t) target;
}
#endif

// The idea here is to get 8 timer interrupts per bit period
void RH_ASK::timerSetup()
{
#if (RH_PLATFORM == RH_PLATFORM_GENERIC_AVR8)
    uint16_t nticks;
    uint8_t prescaler = timerCalc(_speed, (uint16_t)-1, &nticks);
    if (!prescaler) return;
    _COMB(TCCR,RH_ASK_TIMER_INDEX,A)= 0;					
    _COMB(TCCR,RH_ASK_TIMER_INDEX,B)= _BV(WGM12);				
    _COMB(TCCR,RH_ASK_TIMER_INDEX,B)|= prescaler;				
    _COMB(OCR,RH_ASK_TIMER_INDEX,A)= nticks;					
    _COMB(TI,MSK,RH_ASK_TIMER_INDEX)|= _BV(_COMB(OCIE,RH_ASK_TIMER_INDEX,A));

#elif (RH_PLATFORM == RH_PLATFORM_MSP430) // LaunchPad specific
    // Calculate the counter overflow count based on the required bit speed
    // and CPU clock rate
    uint16_t ocr1a = (F_CPU / 8UL) / _speed;
    
    // This code is for Energia/MSP430
    TA0CCR0 = ocr1a;				// Ticks for 62,5 us
    TA0CTL = TASSEL_2 + MC_1;       // SMCLK, up mode
    TA0CCTL0 |= CCIE;               // CCR0 interrupt enabled

#elif (RH_PLATFORM == RH_PLATFORM_STM32L0)
    Serial.println("STM32L0 RH_ASK NOT YET IMPLEMENTED ");
    
#elif (RH_PLATFORM == RH_PLATFORM_STM32) || defined(ARDUINO_ARCH_STM32) || defined(ARDUINO_ARCH_STM32F1) || defined(ARDUINO_ARCH_STM32F3) || defined(ARDUINO_ARCH_STM32F4)
    // Maple etc
    // or rogerclarkmelbourne/Arduino_STM32
    // or stm32duino
    // Pause the timer while we're configuring it
    timer.pause();

 #ifdef BOARD_NAME
    // ST's Arduino Core STM32, https://github.com/stm32duino/Arduino_Core_STM32
    // Declaration of the callback function changed in 1.9. Sigh
  #if (STM32_CORE_VERSION >= 0x01090000)
    void interrupt();
  #else
    void interrupt(HardwareTimer*); // defined below
  #endif
    uint16_t us=(1000000/8)/_speed;
    timer.setMode(1, TIMER_OUTPUT_COMPARE);
    timer.setOverflow(us, MICROSEC_FORMAT);
    timer.setCaptureCompare(1, us - 1, MICROSEC_COMPARE_FORMAT);
  #if (STM32_CORE_VERSION >= 0x01090000)
    timer.attachInterrupt(interrupt);
  #else
    timer.attachInterrupt(1, interrupt);
  #endif

 #else
    void interrupt(); // defined below
    // Roger Clark Arduino STM32, https://github.com/rogerclarkmelbourne/Arduino_STM32
    timer.setPeriod((1000000/8)/_speed);
    // Set up an interrupt on channel 1
    timer.setChannel1Mode(TIMER_OUTPUT_COMPARE);
    timer.setCompare(TIMER_CH1, 1);  // Interrupt 1 count after each update
    void interrupt(); // defined below
    timer.attachCompare1Interrupt(interrupt);
 #endif    
    // Refresh the timer's count, prescale, and overflow
    timer.refresh();
    
    // Start the timer counting
    timer.resume();

#elif (RH_PLATFORM == RH_PLATFORM_ATTINY)
    // figure out prescaler value and counter match value
    // REVISIT: does not correctly handle 1MHz clock speeds, only works with 8MHz clocks
    // At 1MHz clock, get 1/8 of the expected baud rate
    uint16_t nticks;
    uint8_t prescaler = timerCalc(_speed, (uint8_t)-1, &nticks);
    if (!prescaler)
        return; // fault
 #if defined(RH_ASK_ATTINY_USE_TIMER1)
   #if defined(TCCR1)
    // ATtiny85
    TCCR1 = 0;
    TCCR1 = _BV(CTC1); // Turn on CTC mode / Output Compare pins disconnected

    // convert prescaler index to TCCR1 prescaler bits CS10, CS11, CS12, CS13
    TCCR1 |= prescaler; // set CS10, CS11, CS12, CS13 (other bits not needed)

    // Number of ticks to count before firing interrupt
    OCR1A = uint8_t(nticks);
    //  Number of ticks to count before counter reset (CTC mode)
    OCR1C = uint8_t(nticks);
    // Synchronous mode and PLL disabled
    PLLCSR = 0;
    // Set mask to fire interrupt when OCF1A bit is set in TIFR1
    TIMSK |= _BV(OCIE1A); 
   #else //TCCR1
    // ATtiny84
    TCCR1A = 0; // Output Compare pins disconnected
    TCCR1B = _BV(WGM12); // Turn on CTC mode

    // convert prescaler index to TCCRnB prescaler bits CS10, CS11, CS12
    TCCR1B |= prescaler;
    TCCR1C = 0;

    // Caution: special procedures for setting 16 bit regs
    // is handled by the compiler
    OCR1A = nticks;
    //enable interupt
    TIMSK1 |= _BV(OCIE1A);   
   #endif //
  #else //RH_ASK_ATTINY_USE_TIMER1
    TCCR0A = 0;
    TCCR0A = _BV(WGM01); // Turn on CTC mode / Output Compare pins disconnected

    // convert prescaler index to TCCRnB prescaler bits CS00, CS01, CS02
    TCCR0B = 0;
    TCCR0B = prescaler; // set CS00, CS01, CS02 (other bits not needed)


    // Number of ticks to count before firing interrupt
    OCR0A = uint8_t(nticks);

    // Set mask to fire interrupt when OCF0A bit is set in TIFR0
   #ifdef TIMSK0
    // ATtiny84
    TIMSK0 |= _BV(OCIE0A);
   #else
    // ATtiny85
    TIMSK |= _BV(OCIE0A);
   #endif
  #endif //RH_ASK_ATTINY_USE_TIMER1
#elif (RH_PLATFORM == RH_PLATFORM_ATTINY_MEGA)
    // If your processor does not have a TCB1, you can change the timer used in RadioHead.h
    volatile TCB_t* timer = &RH_ATTINY_MEGA_ASK_TIMER;

    // Calculate compare value
    uint32_t compare_val = F_CPU / _speed / 8 - 1;
    // If compare larger than 16bits, need to prescale (will be DIV64)
    if (compare_val > 0xFFFF)
    {
        // recalculate with new prescaler
        compare_val = F_CPU / _speed / 8 / 64 - 1;
	// Prescaler needed
        timer->CTRLA = TCB_CLKSEL_CLKTCA_gc;
    }
    else
    {
	// No prescaler needed
        timer->CTRLA = TCB_CLKSEL_CLKDIV1_gc;
    }

    // Timer to Periodic interrupt mode
    // This write will also disable any active PWM outputs
    timer->CTRLB = TCB_CNTMODE_INT_gc;
    // Write compare register
    timer->CCMP = compare_val;
    // Enable interrupt
    timer->INTCTRL = TCB_CAPTEI_bm;
    // Enable Timer
    timer->CTRLA |= TCB_ENABLE_bm;

#elif (RH_PLATFORM == RH_PLATFORM_ARDUINO) && defined(RH_CUBE_CELL_BOARD)
    // Arduino CubeCell board 1.0.0 has non-standard timer support
    // Forward declaration of the callback below
    void timer_callback();
    TimerInit(&timer, timer_callback);
    // Sigh: This timer takes the timeout in milliseconds, not microseconds. That means the fastest bit rate
    // we can support is 1000 / 8 = 125 bits per second.
    uint32_t period = (1000 / 8) / _speed; // In milliseconds
    TimerSetValue(&timer, period); // mseconds
    TimerStart(&timer);
    
#elif (RH_PLATFORM == RH_PLATFORM_ARDUINO) // Arduino specific


 #if defined(__arm__) && defined(CORE_TEENSY)
    // on Teensy 3.0 (32 bit ARM), use an interval timer
    IntervalTimer *t = new IntervalTimer();
    void TIMER1_COMPA_vect(void);
    t->begin(TIMER1_COMPA_vect, 125000 / _speed);

 #elif defined (__arm__) && defined(ARDUINO_ARCH_SAMD)
    // Arduino Zero
    #define RH_ASK_ZERO_TIMER TC3
    // Clock speed is 48MHz, prescaler of 64 gives a good range of available speeds vs precision
    #define RH_ASK_ZERO_PRESCALER 64
    #define RH_ASK_ZERO_TIMER_IRQ TC3_IRQn

    // Enable clock for TC
    REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TCC2_TC3)) ;
    while ( GCLK->STATUS.bit.SYNCBUSY == 1 ); // wait for sync
    
    // The type cast must fit with the selected timer mode
    TcCount16* TC = (TcCount16*)RH_ASK_ZERO_TIMER; // get timer struct
    
    TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;   // Disable TC
    while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
    
    TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;  // Set Timer counter Mode to 16 bits
    while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
    TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ; // Set TC as Match Frequency
    while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

    // Compute the count required to achieve the requested baud (with 8 interrupts per bit)
    uint32_t rc = (VARIANT_MCK / _speed) / RH_ASK_ZERO_PRESCALER / 8;
    
    TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV64;   // Set prescaler to agree with RH_ASK_ZERO_PRESCALER
    while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
    
    TC->CC[0].reg = rc; // FIXME
    while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
    
    // Interrupts
    TC->INTENSET.reg = 0;              // disable all interrupts
    TC->INTENSET.bit.MC0 = 1;          // enable compare match to CC0
    
    // Enable InterruptVector
    NVIC_ClearPendingIRQ(RH_ASK_ZERO_TIMER_IRQ);
    NVIC_EnableIRQ(RH_ASK_ZERO_TIMER_IRQ);
    
    // Enable TC
    TC->CTRLA.reg |= TC_CTRLA_ENABLE;
    while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
    
 #elif defined(__arm__) && defined(ARDUINO_SAM_DUE)
    // Arduino Due
    // Clock speed is 84MHz
    // Due has 9 timers in 3 blocks of 3.
    // We use timer 1 TC1_IRQn on TC0 channel 1, since timers 0, 2, 3, 4, 5 are used by the Servo library
    #define RH_ASK_DUE_TIMER TC0
    #define RH_ASK_DUE_TIMER_CHANNEL 1
    #define RH_ASK_DUE_TIMER_IRQ TC1_IRQn
    pmc_set_writeprotect(false);
    pmc_enable_periph_clk(RH_ASK_DUE_TIMER_IRQ);
    
    // Clock speed 4 can handle all reasonable _speeds we might ask for. Its divisor is 128
    // and we want 8 interrupts per bit
    uint32_t rc = (VARIANT_MCK / _speed) / 128 / 8;
    TC_Configure(RH_ASK_DUE_TIMER, RH_ASK_DUE_TIMER_CHANNEL, 
		 TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);
    TC_SetRC(RH_ASK_DUE_TIMER, RH_ASK_DUE_TIMER_CHANNEL, rc);
    // Enable the RC Compare Interrupt
    RH_ASK_DUE_TIMER->TC_CHANNEL[RH_ASK_DUE_TIMER_CHANNEL].TC_IER = TC_IER_CPCS;
    NVIC_ClearPendingIRQ(RH_ASK_DUE_TIMER_IRQ);
    NVIC_EnableIRQ(RH_ASK_DUE_TIMER_IRQ);
    TC_Start(RH_ASK_DUE_TIMER, RH_ASK_DUE_TIMER_CHANNEL);
    
 #elif defined(ARDUINO_ARCH_RP2040)
    // Per https://emalliab.wordpress.com/2021/04/18/raspberry-pi-pico-arduino-core-and-timers/
    hw_set_bits(&timer_hw->inte, 1u << RH_ASK_PICO_ALARM_NUM);
    void picoInterrupt(); // Forward declaration of interrupt handler
    irq_set_exclusive_handler(RH_ASK_PICO_ALARM_IRQ, picoInterrupt);
    irq_set_enabled(RH_ASK_PICO_ALARM_IRQ, true);
    set_pico_alarm(_speed);
    
 #elif defined(ARDUINO_UNOR4_MINIMA) || defined(ARDUINO_UNOR4_WIFI)
    uint8_t timer_type = GPT_TIMER;
    int8_t tindex = FspTimer::get_available_timer(timer_type);
    if (tindex < 0)
	tindex = FspTimer::get_available_timer(timer_type, true);

    if (tindex < 0)
	return;

    FspTimer::force_use_of_pwm_reserved_timer();
    void timer_callback(timer_callback_args_t __attribute((unused)) *p_args); // Forward declaration
    if (!ask_timer.begin(TIMER_MODE_PERIODIC, timer_type, tindex, _speed * 8, 0.0f, timer_callback))
	return;

    if (!ask_timer.setup_overflow_irq())
	return;

    if (!ask_timer.open())
	return;

    if (!ask_timer.start())
	return;
    
 #else
    uint16_t nticks; // number of prescaled ticks needed
    uint8_t prescaler; // Bit values for CS0[2:0]

    // This is the path for most Arduinos
    // figure out prescaler value and counter match value
  #if defined(RH_ASK_ARDUINO_USE_TIMER2)
    prescaler = timerCalc(_speed, (uint8_t)-1, &nticks);
    if (!prescaler)
        return; // fault
    // Use timer 2
    TCCR2A = _BV(WGM21); // Turn on CTC mode)
    // convert prescaler index to TCCRnB prescaler bits CS10, CS11, CS12
    TCCR2B = prescaler;

    // Caution: special procedures for setting 16 bit regs
    // is handled by the compiler
    OCR2A = nticks;
    // Enable interrupt
   #ifdef TIMSK2
    // atmega168
    TIMSK2 |= _BV(OCIE2A);
   #else
    // others
    TIMSK |= _BV(OCIE2A);
   #endif // TIMSK2
  #else
    // Use timer 1
    prescaler = timerCalc(_speed, (uint16_t)-1, &nticks);    
    if (!prescaler)
        return; // fault
    TCCR1A = 0; // Output Compare pins disconnected
    TCCR1B = _BV(WGM12); // Turn on CTC mode

    // convert prescaler index to TCCRnB prescaler bits CS10, CS11, CS12
    TCCR1B |= prescaler;

    // Caution: special procedures for setting 16 bit regs
    // is handled by the compiler
    OCR1A = nticks;
    // Enable interrupt
   #ifdef TIMSK1
    // atmega168
    TIMSK1 |= _BV(OCIE1A);
   #else
    // others
    TIMSK |= _BV(OCIE1A);
   #endif // TIMSK1
  #endif
 #endif

#elif (RH_PLATFORM == RH_PLATFORM_STM32F2) // Photon
    // Inspired by SparkIntervalTimer
    // We use Timer 6
    void TimerInterruptHandler(); // Forward declaration for interrupt handler
    #define SYSCORECLOCK	60000000UL  // Timer clock tree uses core clock / 2
    TIM_TimeBaseInitTypeDef timerInitStructure;
    NVIC_InitTypeDef nvicStructure;
    TIM_TypeDef* TIMx;
    uint32_t period = (1000000 / 8) / _speed; // In microseconds
    uint16_t prescaler = (uint16_t)(SYSCORECLOCK / 1000000UL) - 1; //To get TIM counter clock = 1MHz

    attachSystemInterrupt(SysInterrupt_TIM6_Update, TimerInterruptHandler);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
    nvicStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
    TIMx = TIM6;
    nvicStructure.NVIC_IRQChannelPreemptionPriority = 10;
    nvicStructure.NVIC_IRQChannelSubPriority = 1;
    nvicStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicStructure);
    timerInitStructure.TIM_Prescaler = prescaler;
    timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    timerInitStructure.TIM_Period = period;
    timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    timerInitStructure.TIM_RepetitionCounter = 0;
    
    TIM_TimeBaseInit(TIMx, &timerInitStructure);
    TIM_ITConfig(TIMx, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIMx, ENABLE);

#elif (RH_PLATFORM == RH_PLATFORM_CHIPKIT_CORE)
    // UsingChipKIT Core on Arduino IDE
    uint32_t chipkit_timer_interrupt_handler(uint32_t currentTime); // Forward declaration
    attachCoreTimerService(chipkit_timer_interrupt_handler);

#elif (RH_PLATFORM == RH_PLATFORM_UNO32)
    // Under old MPIDE, which has been discontinued:
    // ON Uno32 we use timer1
    OpenTimer1(T1_ON | T1_PS_1_1 | T1_SOURCE_INT, (F_CPU / 8) / _speed);
    ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_1);

#elif (RH_PLATFORM == RH_PLATFORM_ESP8266)
    void RH_INTERRUPT_ATTR esp8266_timer_interrupt_handler(); // Forward declaration
    // The - 120 is a heuristic to correct for interrupt handling overheads
    _timerIncrement = (clockCyclesPerMicrosecond() * 1000000 / 8 / _speed) - 120;
    timer0_isr_init();
    timer0_attachInterrupt(esp8266_timer_interrupt_handler);
    timer0_write(ESP.getCycleCount() + _timerIncrement);
//    timer0_write(ESP.getCycleCount() + 41660000);
#elif (RH_PLATFORM == RH_PLATFORM_ESP32)
    void RH_INTERRUPT_ATTR esp32_timer_interrupt_handler(); // Forward declaration
 #if ESP_ARDUINO_VERSION_MAJOR >= 3
    // Sigh, this changed, apparently in version 3.
    timer = timerBegin(_speed * 8);
    timerAttachInterrupt(timer, &esp32_timer_interrupt_handler);
    timerAlarm(timer, 1, true, 0);
 #else
    // Prior to version 3
    timer = timerBegin(0, 80, true); // Alarm value will be in in us
    timerAttachInterrupt(timer, &esp32_timer_interrupt_handler, true);
    timerAlarmWrite(timer, 1000000 / _speed / 8, true);
    timerAlarmEnable(timer);
 #endif
#endif

}

void RH_INTERRUPT_ATTR RH_ASK::setModeIdle()
{
    if (_mode != RHModeIdle)
    {
	// Disable the transmitter hardware
	writePtt(LOW);
	writeTx(LOW);
	_mode = RHModeIdle;
    }
}

void RH_INTERRUPT_ATTR RH_ASK::setModeRx()
{
    if (_mode != RHModeRx)
    {
	// Disable the transmitter hardware
	writePtt(LOW);
	writeTx(LOW);
	_mode = RHModeRx;
    }
}

void RH_ASK::setModeTx()
{
    if (_mode != RHModeTx)
    {
	// PRepare state varibles for a new transmission
	_txIndex = 0;
	_txBit = 0;
	_txSample = 0;

	// Enable the transmitter hardware
	writePtt(HIGH);

	_mode = RHModeTx;
    }
}

// Call this often
bool RH_ASK::available()
{
    if (_mode == RHModeTx)
	return false;
    setModeRx();
    if (_rxBufFull)
    {
	validateRxBuf();
	_rxBufFull= false;
    }
    return _rxBufValid;
}

bool RH_INTERRUPT_ATTR RH_ASK::recv(uint8_t* buf, uint8_t* len)
{
    if (!available())
	return false;

    if (buf && len)
    {
	// Skip the length and 4 headers that are at the beginning of the rxBuf
	// and drop the trailing 2 bytes of FCS
	uint8_t message_len = _rxBufLen-RH_ASK_HEADER_LEN - 3;
	if (*len > message_len)
	    *len = message_len;
	memcpy(buf, _rxBuf+RH_ASK_HEADER_LEN+1, *len);
    }
    _rxBufValid = false; // Got the most recent message, delete it
//    printBuffer("recv:", buf, *len);
    return true;
}

// Caution: this may block
bool RH_ASK::send(const uint8_t* data, uint8_t len)
{
    uint8_t i;
    uint16_t index = 0;
    uint16_t crc = 0xffff;
    uint8_t *p = _txBuf + RH_ASK_PREAMBLE_LEN; // start of the message area
    uint8_t count = len + 3 + RH_ASK_HEADER_LEN; // Added byte count and FCS and headers to get total number of bytes

    if (len > RH_ASK_MAX_MESSAGE_LEN)
	return false;

    // Wait for transmitter to become available
    waitPacketSent();

    if (!waitCAD()) 
	return false;  // Check channel activity

    // Encode the message length
    crc = RHcrc_ccitt_update(crc, count);
    p[index++] = symbols[count >> 4];
    p[index++] = symbols[count & 0xf];

    // Encode the headers
    crc = RHcrc_ccitt_update(crc, _txHeaderTo);
    p[index++] = symbols[_txHeaderTo >> 4];
    p[index++] = symbols[_txHeaderTo & 0xf];
    crc = RHcrc_ccitt_update(crc, _txHeaderFrom);
    p[index++] = symbols[_txHeaderFrom >> 4];
    p[index++] = symbols[_txHeaderFrom & 0xf];
    crc = RHcrc_ccitt_update(crc, _txHeaderId);
    p[index++] = symbols[_txHeaderId >> 4];
    p[index++] = symbols[_txHeaderId & 0xf];
    crc = RHcrc_ccitt_update(crc, _txHeaderFlags);
    p[index++] = symbols[_txHeaderFlags >> 4];
    p[index++] = symbols[_txHeaderFlags & 0xf];

    // Encode the message into 6 bit symbols. Each byte is converted into 
    // 2 6-bit symbols, high nybble first, low nybble second
    for (i = 0; i < len; i++)
    {
	crc = RHcrc_ccitt_update(crc, data[i]);
	p[index++] = symbols[data[i] >> 4];
	p[index++] = symbols[data[i] & 0xf];
    }

    // Append the fcs, 16 bits before encoding (4 6-bit symbols after encoding)
    // Caution: VW expects the _ones_complement_ of the CCITT CRC-16 as the FCS
    // VW sends FCS as low byte then hi byte
    crc = ~crc;
    p[index++] = symbols[(crc >> 4)  & 0xf];
    p[index++] = symbols[crc & 0xf];
    p[index++] = symbols[(crc >> 12) & 0xf];
    p[index++] = symbols[(crc >> 8)  & 0xf];

    // Total number of 6-bit symbols to send
    _txBufLen = index + RH_ASK_PREAMBLE_LEN;

    // Start the low level interrupt handler sending symbols
    setModeTx();

    return true;
}

// Read the RX data input pin, taking into account platform type and inversion.
bool RH_INTERRUPT_ATTR RH_ASK::readRx()
{
    bool value;
#if (RH_PLATFORM == RH_PLATFORM_GENERIC_AVR8)
    value = ((RH_ASK_RX_PORT & (1<<RH_ASK_RX_PIN)) ? 1 : 0);
#else
    value = digitalRead(_rxPin);
#endif
    return value ^ _rxInverted;
}

// Write the TX output pin, taking into account platform type.
void RH_INTERRUPT_ATTR RH_ASK::writeTx(bool value)
{
#if (RH_PLATFORM == RH_PLATFORM_GENERIC_AVR8)
    ((value) ? (RH_ASK_TX_PORT |= (1<<RH_ASK_TX_PIN)) : (RH_ASK_TX_PORT &= ~(1<<RH_ASK_TX_PIN)));
// No longer relevant: PinStatus onlty used in old versions
//#elif (RH_PLATFORM == RH_PLATFORM_ATTINY_MEGA)
//    digitalWrite(_txPin, (PinStatus)value);
#else
    digitalWrite(_txPin, value);
#endif
}

// Write the PTT output pin, taking into account platform type and inversion.
void RH_INTERRUPT_ATTR RH_ASK::writePtt(bool value)
{
#if (RH_PLATFORM == RH_PLATFORM_GENERIC_AVR8)
 #if RH_ASK_PTT_PIN 
    ((value) ? (RH_ASK_PTT_PORT |= (1<<RH_ASK_PTT_PIN)) : (RH_ASK_PTT_PORT &= ~(1<<RH_ASK_PTT_PIN)));
 #else
    ((value) ? (RH_ASK_TX_PORT |= (1<<RH_ASK_TX_PIN)) : (RH_ASK_TX_PORT &= ~(1<<RH_ASK_TX_PIN)));
 #endif
// This no longer relevant: ater version use uint8_t
//#elif (RH_PLATFORM == RH_PLATFORM_ATTINY_MEGA)
//    digitalWrite(_txPin, (PinStatus)(value ^ _pttInverted));
#else
    digitalWrite(_pttPin, value ^ _pttInverted);
#endif
}

uint8_t RH_ASK::maxMessageLength()
{
    return RH_ASK_MAX_MESSAGE_LEN;
}

#if (RH_PLATFORM == RH_PLATFORM_ARDUINO) 
 // Assume Arduino Uno (328p or similar)
 #if defined(RH_ASK_ARDUINO_USE_TIMER2)
  #define RH_ASK_TIMER_VECTOR TIMER2_COMPA_vect
 #else
  #define RH_ASK_TIMER_VECTOR TIMER1_COMPA_vect
 #endif
#elif (RH_PLATFORM == RH_PLATFORM_ATTINY)
#if defined(RH_ASK_ATTINY_USE_TIMER1)
  #define RH_ASK_TIMER_VECTOR TIM1_COMPA_vect
 #else 
  #define RH_ASK_TIMER_VECTOR TIM0_COMPA_vect
 #endif //RH_ASK_ATTINY_USE_TIMER1
#elif (RH_PLATFORM == RH_PLATFORM_GENERIC_AVR8)
 #define __COMB(a,b,c) (a##b##c)
 #define _COMB(a,b,c) __COMB(a,b,c)
 #define RH_ASK_TIMER_VECTOR _COMB(TIMER,RH_ASK_TIMER_INDEX,_COMPA_vect)
#endif

#if (RH_PLATFORM == RH_PLATFORM_ARDUINO) && defined(__arm__) && defined(CORE_TEENSY)	
void TIMER1_COMPA_vect(void)
{
    thisASKDriver->handleTimerInterrupt();
}

#elif (RH_PLATFORM == RH_PLATFORM_ARDUINO) && defined (__arm__) && defined(ARDUINO_ARCH_SAMD)
// Arduino Zero
void TC3_Handler()
{
    // The type cast must fit with the selected timer mode
    TcCount16* TC = (TcCount16*)RH_ASK_ZERO_TIMER; // get timer struct
    TC->INTFLAG.bit.MC0 = 1;
    thisASKDriver->handleTimerInterrupt();
}

#elif (RH_PLATFORM == RH_PLATFORM_ARDUINO) && defined(__arm__) && defined(ARDUINO_SAM_DUE)
// Arduino Due
void TC1_Handler()
{
    TC_GetStatus(RH_ASK_DUE_TIMER, 1);
    thisASKDriver->handleTimerInterrupt();
}

#elif (RH_PLATFORM == RH_PLATFORM_ARDUINO) && defined(ARDUINO_ARCH_RP2040)
void picoInterrupt()
{
    hw_clear_bits(&timer_hw->intr, 1u << RH_ASK_PICO_ALARM_NUM);
    set_pico_alarm(thisASKDriver->speed());
    thisASKDriver->handleTimerInterrupt();
}

#elif defined(BOARD_NAME)
// ST's Arduino Core STM32, https://github.com/stm32duino/Arduino_Core_STM32
// Declaration of the callback function changed in 1.9
 #if (STM32_CORE_VERSION >= 0x01090000)
// This really should be callback_function_t interrupt() but some platform compilers
// warn/error, thinking there should be a return value
void interrupt()
 #else
void interrupt(HardwareTimer*)
 #endif
{
    thisASKDriver->handleTimerInterrupt();
}
#elif (RH_PLATFORM == RH_PLATFORM_ARDUINO) && (defined(ARDUINO_ARCH_STM32) || defined(ARDUINO_ARCH_STM32F1) || defined(ARDUINO_ARCH_STM32F3) || defined(ARDUINO_ARCH_STM32F4) || defined(ARDUINO_ARCH_RP2040))
// Roger Clark Arduino STM32, https://github.com/rogerclarkmelbourne/Arduino_STM32
void interrupt()
{
    thisASKDriver->handleTimerInterrupt();
}

#elif  defined(ARDUINO_UNOR4_MINIMA) || defined(ARDUINO_UNOR4_WIFI)
// callback method used by timer
void timer_callback(timer_callback_args_t __attribute((unused)) *p_args)
{
    thisASKDriver->handleTimerInterrupt();
}

#elif (RH_PLATFORM == RH_PLATFORM_ARDUINO) && defined(RH_CUBE_CELL_BOARD)
// Cube cell interrupt
void timer_callback(void)
{
    TimerStart(&timer);
    thisASKDriver->handleTimerInterrupt();
}

#elif (RH_PLATFORM == RH_PLATFORM_ARDUINO) || (RH_PLATFORM == RH_PLATFORM_GENERIC_AVR8) || (RH_PLATFORM == RH_PLATFORM_ATTINY)
// This is the interrupt service routine called when timer1 overflows
// Its job is to output the next bit from the transmitter (every 8 calls)
// and to call the PLL code if the receiver is enabled
//ISR(SIG_OUTPUT_COMPARE1A)
ISR(RH_ASK_TIMER_VECTOR)
{
    thisASKDriver->handleTimerInterrupt();
}

#elif (RH_PLATFORM == RH_PLATFORM_MSP430) || (RH_PLATFORM == RH_PLATFORM_STM32)
// LaunchPad, Maple
void interrupt()
{
    thisASKDriver->handleTimerInterrupt();
}

#elif (RH_PLATFORM == RH_PLATFORM_STM32F2) // Photon
void TimerInterruptHandler()
{
    thisASKDriver->handleTimerInterrupt();
}

#elif (RH_PLATFORM == RH_PLATFORM_MSP430) 
interrupt(TIMER0_A0_VECTOR) Timer_A_int(void) 
{
    thisASKDriver->handleTimerInterrupt();
};

#elif (RH_PLATFORM == RH_PLATFORM_CHIPKIT_CORE)
// Using ChipKIT Core on Arduino IDE
uint32_t chipkit_timer_interrupt_handler(uint32_t currentTime) 
{
    thisASKDriver->handleTimerInterrupt();
    return (currentTime + ((CORE_TICK_RATE * 1000)/8)/thisASKDriver->speed());
}

#elif (RH_PLATFORM == RH_PLATFORM_UNO32)
// Under old MPIDE, which has been discontinued:
extern "C"
{
 void __ISR(_TIMER_1_VECTOR, ipl1) timerInterrupt(void)
 {
    thisASKDriver->handleTimerInterrupt();
    mT1ClearIntFlag(); // Clear timer 1 interrupt flag
}
}
#elif (RH_PLATFORM == RH_PLATFORM_ESP8266)
void RH_INTERRUPT_ATTR esp8266_timer_interrupt_handler()
{  
//    timer0_write(ESP.getCycleCount() + 41660000);
//    timer0_write(ESP.getCycleCount() + (clockCyclesPerMicrosecond() * 100) - 120 );
    timer0_write(ESP.getCycleCount() + thisASKDriver->_timerIncrement);
//    static int toggle = 0;
//  toggle = (toggle == 1) ? 0 : 1;
//  digitalWrite(4, toggle);
    thisASKDriver->handleTimerInterrupt();
}
#elif (RH_PLATFORM == RH_PLATFORM_ESP32)
void RH_INTERRUPT_ATTR esp32_timer_interrupt_handler()
{
    thisASKDriver->handleTimerInterrupt();
}
#elif (RH_PLATFORM == RH_PLATFORM_ATTINY_MEGA)
ISR(RH_ATTINY_MEGA_ASK_TIMER_VECTOR)
{
    thisASKDriver->handleTimerInterrupt();
    RH_ATTINY_MEGA_ASK_TIMER.INTFLAGS = TCB_CAPT_bm;
}
#endif

// Convert a 6 bit encoded symbol into its 4 bit decoded equivalent
uint8_t RH_INTERRUPT_ATTR RH_ASK::symbol_6to4(uint8_t symbol)
{
    uint8_t i;
    uint8_t count;
    
    // Linear search :-( Could have a 64 byte reverse lookup table?
    // There is a little speedup here courtesy Ralph Doncaster:
    // The shortcut works because bit 5 of the symbol is 1 for the last 8
    // symbols, and it is 0 for the first 8.
    // So we only have to search half the table
    for (i = (symbol>>2) & 8, count=8; count-- ; i++)
	if (symbol == symbols[i]) return i;

    return 0; // Not found
}

// Check whether the latest received message is complete and uncorrupted
// We should always check the FCS at user level, not interrupt level
// since it is slow
void RH_ASK::validateRxBuf()
{
    uint16_t crc = 0xffff;
    // The CRC covers the byte count, headers and user data
    for (uint8_t i = 0; i < _rxBufLen; i++)
	crc = RHcrc_ccitt_update(crc, _rxBuf[i]);
    if (crc != 0xf0b8) // CRC when buffer and expected CRC are CRC'd
    {
	// Reject and drop the message
	_rxBad++;
	_rxBufValid = false;
	return;
    }

    // Extract the 4 headers that follow the message length
    _rxHeaderTo    = _rxBuf[1];
    _rxHeaderFrom  = _rxBuf[2];
    _rxHeaderId    = _rxBuf[3];
    _rxHeaderFlags = _rxBuf[4];
    if (_promiscuous ||
	_rxHeaderTo == _thisAddress ||
	_rxHeaderTo == RH_BROADCAST_ADDRESS)
    {
	_rxGood++;
	_rxBufValid = true;
    }
}

void RH_INTERRUPT_ATTR RH_ASK::receiveTimer()
{
    bool rxSample = readRx();

    // Integrate each sample
    if (rxSample)
	_rxIntegrator++;

    if (rxSample != _rxLastSample)
    {
	// Transition, advance if ramp > 80, retard if < 80
	_rxPllRamp += ((_rxPllRamp < RH_ASK_RAMP_TRANSITION) 
			   ? RH_ASK_RAMP_INC_RETARD 
			   : RH_ASK_RAMP_INC_ADVANCE);
	_rxLastSample = rxSample;
    }
    else
    {
	// No transition
	// Advance ramp by standard 20 (== 160/8 samples)
	_rxPllRamp += RH_ASK_RAMP_INC;
    }
    if (_rxPllRamp >= RH_ASK_RX_RAMP_LEN)
    {
	// Add this to the 12th bit of _rxBits, LSB first
	// The last 12 bits are kept
	_rxBits >>= 1;

	// Check the integrator to see how many samples in this cycle were high.
	// If < 5 out of 8, then its declared a 0 bit, else a 1;
	if (_rxIntegrator >= 5)
	    _rxBits |= 0x800;

	_rxPllRamp -= RH_ASK_RX_RAMP_LEN;
	_rxIntegrator = 0; // Clear the integral for the next cycle

	if (_rxActive)
	{
	    // We have the start symbol and now we are collecting message bits,
	    // 6 per symbol, each which has to be decoded to 4 bits
	    if (++_rxBitCount >= 12)
	    {
		// Have 12 bits of encoded message == 1 byte encoded
		// Decode as 2 lots of 6 bits into 2 lots of 4 bits
		// The 6 lsbits are the high nybble
		uint8_t this_byte = 
		    (symbol_6to4(_rxBits & 0x3f)) << 4 
		    | symbol_6to4(_rxBits >> 6);

		// The first decoded byte is the byte count of the following message
		// the count includes the byte count and the 2 trailing FCS bytes
		// REVISIT: may also include the ACK flag at 0x40
		if (_rxBufLen == 0)
		{
		    // The first byte is the byte count
		    // Check it for sensibility. It cant be less than 7, since it
		    // includes the byte count itself, the 4 byte header and the 2 byte FCS
		    _rxCount = this_byte;
		    if (_rxCount < 7 || _rxCount > RH_ASK_MAX_PAYLOAD_LEN)
		    {
			// Stupid message length, drop the whole thing
			_rxActive = false;
			_rxBad++;
                        return;
		    }
		}
		_rxBuf[_rxBufLen++] = this_byte;

		if (_rxBufLen >= _rxCount)
		{
		    // Got all the bytes now
		    _rxActive = false;
		    _rxBufFull = true;
		    setModeIdle();
		}
		_rxBitCount = 0;
	    }
	}
	// Not in a message, see if we have a start symbol
	else if (_rxBits == RH_ASK_START_SYMBOL)
	{
	    // Have start symbol, start collecting message
	    _rxActive = true;
	    _rxBitCount = 0;
	    _rxBufLen = 0;
	}
    }
}

void RH_INTERRUPT_ATTR RH_ASK::transmitTimer()
{
    if (_txSample++ == 0)
    {
	// Send next bit
	// Symbols are sent LSB first
	// Finished sending the whole message? (after waiting one bit period 
	// since the last bit)
	if (_txIndex >= _txBufLen)
	{
	    setModeIdle();
	    _txGood++;
	}
	else
	{
	    writeTx(_txBuf[_txIndex] & (1 << _txBit++));
	    if (_txBit >= 6)
	    {
		_txBit = 0;
		_txIndex++;
	    }
	}
    }
	
    if (_txSample > 7)
	_txSample = 0;
}

void RH_INTERRUPT_ATTR RH_ASK::handleTimerInterrupt()
{
    if (_mode == RHModeRx)
	receiveTimer(); // Receiving
    else if (_mode == RHModeTx)
        transmitTimer(); // Transmitting
}

#endif //_SAMD51__
