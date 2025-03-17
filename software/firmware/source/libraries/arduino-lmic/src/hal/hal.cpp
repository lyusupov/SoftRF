/*******************************************************************************
 * Copyright (c) 2015 Matthijs Kooijman
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * This the HAL to run LMIC on top of the Arduino environment.
 *******************************************************************************/

#if defined(ARDUINO)
#include <Arduino.h>
#include <SPI.h>
#endif /* ARDUINO */

#if defined(RASPBERRY_PI)
#include <raspi/raspi.h>
#endif /* RASPBERRY_PI */

#if defined(ENERGIA_ARCH_CC13XX) || defined(ENERGIA_ARCH_CC13X2)
#include <cc13xx/cc13xx.h>
#endif /* ENERGIA_ARCH_CC13XX || ENERGIA_ARCH_CC13X2 */

#if defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RP2350)
#undef  SPI
#define SPI SPI1
#endif /* ARDUINO_ARCH_RP2040 || ARDUINO_ARCH_RP2350 */

#if defined(ARDUINO_ARCH_RENESAS) // || defined(ARDUINO_ARCH_SILABS)
#include <SoftSPI.h>
extern  SoftSPI RadioSPI;
#undef  SPI
#define SPI RadioSPI
#endif /* ARDUINO_ARCH_RENESAS || ARDUINO_ARCH_SILABS */

#if defined(ARDUINO_ARCH_CH32)
extern  SPIClass RadioSPI;
#undef  SPI
#define SPI RadioSPI
#endif /* ARDUINO_ARCH_CH32 */

#include "../lmic.h"
#include "hal.h"
#include <stdio.h>
#include <stdarg.h>

// -----------------------------------------------------------------------------
// I/O
// in, case we have no DIO mapping to a GPIO pin, we'll need to read 
// Lora Module IRQ register
static bool check_dio = 0;

static void hal_interrupt_init(); // Fwd declaration

static void hal_io_init () {
    // NSS and DIO0 are required, DIO1 is required for LoRa, DIO2 for FSK
    ASSERT(lmic_pins.nss != LMIC_UNUSED_PIN);

    // No more needed, if dio pins are declared as unused, then LIMC will check
    // interrputs directly into Lora module register, avoiding needed GPIO line to IRQ
    //ASSERT(lmic_pins.dio[0] != LMIC_UNUSED_PIN);
    //ASSERT(lmic_pins.dio[1] != LMIC_UNUSED_PIN || lmic_pins.dio[2] != LMIC_UNUSED_PIN);

    // Write HIGH to deselect (NSS is active low). Do this before
    // setting output, to prevent a moment of OUTPUT LOW on e.g. AVR.
    digitalWrite(lmic_pins.nss, HIGH);
    pinMode(lmic_pins.nss, OUTPUT);
    // Write HIGH again after setting output, for architectures that
    // reset to LOW when setting OUTPUT (e.g. arduino-STM32L4).
    digitalWrite(lmic_pins.nss, HIGH);

    if (lmic_pins.txe != LMIC_UNUSED_PIN)
        pinMode(lmic_pins.txe, OUTPUT);
    if (lmic_pins.rxe != LMIC_UNUSED_PIN)
        pinMode(lmic_pins.rxe, OUTPUT);
    if (lmic_pins.rst != LMIC_UNUSED_PIN)
        pinMode(lmic_pins.rst, OUTPUT);
    if (lmic_pins.tcxo != LMIC_UNUSED_PIN)
#if defined(ARDUINO_NUCLEO_L073RZ)
      if (lmic_pins.tcxo == PD_7)
        hal_pin_tcxo_init();
      else
#endif /* ARDUINO_NUCLEO_L073RZ */
        pinMode(lmic_pins.tcxo, OUTPUT);

    hal_interrupt_init();
}

// rx = 0, tx = 1, off = -1
void hal_pin_rxtx (s1_t val) {
    if (lmic_pins.txe != LMIC_UNUSED_PIN)
        digitalWrite(lmic_pins.txe, val == 1 ? HIGH : LOW);
    if (lmic_pins.rxe != LMIC_UNUSED_PIN)
        digitalWrite(lmic_pins.rxe, val == 0 ? HIGH : LOW);
}

// set radio RST pin to given value (or keep floating!)
void hal_pin_rst (u1_t val) {
    if (lmic_pins.rst == LMIC_UNUSED_PIN)
        return;

    if(val == 0 || val == 1) { // drive pin
        pinMode(lmic_pins.rst, OUTPUT);
        digitalWrite(lmic_pins.rst, val == 1 ? HIGH : LOW);
    } else {
        pinMode(lmic_pins.rst, INPUT);
#if defined(RASPBERRY_PI)
        //  with a pullup
        bcm2835_gpio_set_pud(lmic_pins.rst, BCM2835_GPIO_PUD_UP);
#else
        // keep pin floating
#endif
    }
}

#if !defined(LMIC_USE_INTERRUPTS)
static void hal_interrupt_init() {
    check_dio = 0;

    // Loop to check / configure all DIO input pin
    for (uint8_t i = 0; i < NUM_DIO; ++i) {
        if (lmic_pins.dio[i] != LMIC_UNUSED_PIN) {
            // we need to check at least one DIO line 
            check_dio = 1; 
            pinMode(lmic_pins.dio[i], INPUT);

#ifdef RASPBERRY_PI
            // Enable pull down an rising edge detection on this one
            bcm2835_gpio_set_pud(lmic_pins.dio[i], BCM2835_GPIO_PUD_DOWN);
            bcm2835_gpio_ren(lmic_pins.dio[i]);
#endif

        }
    }
}

static bool dio_states[NUM_DIO] = {0};
static void hal_io_check() {
    uint8_t i;
    // At least one DIO Line to check ?
    if (check_dio) {
        for (i = 0; i < NUM_DIO; ++i) {
            if (lmic_pins.dio[i] == LMIC_UNUSED_PIN)
                continue;

#ifdef RASPBERRY_PI
            // Rising edge fired ?
            if (bcm2835_gpio_eds(lmic_pins.dio[i])) {
                // Now clear the eds flag by setting it to 1
                bcm2835_gpio_set_eds(lmic_pins.dio[i]);
                // Handle pseudo interrupt
                radio_irq_handler(i);
            }
#else
            if (dio_states[i] != digitalRead(lmic_pins.dio[i])) {
                dio_states[i] = !dio_states[i];
                if (dio_states[i]) {
                    radio_irq_handler(i);
                }
            }
#endif
        }
    } else {
        // Check IRQ flags in radio module
        if ( radio_has_irq() ) {
            radio_irq_handler(0);
        }
    }

}

#else
// Interrupt handlers
static bool interrupt_flags[NUM_DIO] = {0};

static void hal_isrPin0() {
    interrupt_flags[0] = true;
}
static void hal_isrPin1() {
    interrupt_flags[1] = true;
}
static void hal_isrPin2() {
    interrupt_flags[2] = true;
}

typedef void (*isr_t)();
static isr_t interrupt_fns[NUM_DIO] = {hal_isrPin0, hal_isrPin1, hal_isrPin2};

static void hal_interrupt_init() {
  for (uint8_t i = 0; i < NUM_DIO; ++i) {
      if (lmic_pins.dio[i] == LMIC_UNUSED_PIN)
          continue;

      attachInterrupt(digitalPinToInterrupt(lmic_pins.dio[i]), interrupt_fns[i], RISING);
  }
}

static void hal_io_check() {
    uint8_t i;
    for (i = 0; i < NUM_DIO; ++i) {
        if (lmic_pins.dio[i] == LMIC_UNUSED_PIN)
            continue;

        if (interrupt_flags[i]) {
            interrupt_flags[i] = false;
            radio_irq_handler(i);
        }
    }
}
#endif // LMIC_USE_INTERRUPTS

#if defined(ARDUINO_NUCLEO_L073RZ)
void hal_pin_tcxo_init()
{
  GPIO_InitTypeDef GPIO_Struct;

  __HAL_RCC_GPIOD_CLK_ENABLE();

  // for LoRa sx1276 TCXO OE Pin
  GPIO_Struct.Pin = GPIO_PIN_7;
  GPIO_Struct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_Struct.Pull = GPIO_NOPULL;
  GPIO_Struct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_Struct);
}
#endif /* ARDUINO_NUCLEO_L073RZ */

bool hal_pin_tcxo (u1_t val) {
    if (lmic_pins.tcxo == LMIC_UNUSED_PIN)
        return false;

#if defined(ARDUINO_NUCLEO_L073RZ)
      if (lmic_pins.tcxo == PD_7)
          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7,
                            val == 1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
      else
#endif /* ARDUINO_NUCLEO_L073RZ */

    digitalWrite(lmic_pins.tcxo, val == 1 ? HIGH : LOW);
    return true;
}

// -----------------------------------------------------------------------------
// SPI

#ifdef RASPBERRY_PI
// Raspberry Pi 2:
//    BCM2835_CORE_CLK_HZ = 250000000
//    Clock divider / 64 = 3.906 MHz
static const SPISettings settings(BCM2835_SPI_CLOCK_DIVIDER_64, BCM2835_SPI_BIT_ORDER_MSBFIRST, BCM2835_SPI_MODE0);
//#elif defined(__ASR6501__) || defined(ARDUINO_ARCH_ASR650X)
/* nothing to do */
#else
static const SPISettings settings(LMIC_SPI_FREQ, MSBFIRST, SPI_MODE0);
#endif

static void hal_spi_init () {
#if defined(ENERGIA_ARCH_CC13XX) || defined(ENERGIA_ARCH_CC13X2)
    SPI.setClockDivider(SPI_CLOCK_MAX / LMIC_SPI_FREQ);
#endif
    SPI.begin();
}

void hal_pin_nss (u1_t val) {

#if defined(SPI_HAS_TRANSACTION)
    if (!val)
        SPI.beginTransaction(settings);
    else
        SPI.endTransaction();
#endif

    //Serial.println(val?">>":"<<");
    digitalWrite(lmic_pins.nss, val ? HIGH : LOW);
}

// perform SPI transaction with radio
u1_t hal_spi (u1_t out) {
    u1_t res = SPI.transfer(out);
/*
    Serial.print(">");
    Serial.print(out, HEX);
    Serial.print("<");
    Serial.println(res, HEX);
    */
    return res;
}

static u1_t spi_buf[MAX_LEN_FRAME + 1];

u1_t hal_spi_read_reg (u1_t addr) {
    hal_pin_nss(0);
#if !defined(ENERGIA_ARCH_CC13XX) && !defined(ENERGIA_ARCH_CC13X2)
    hal_spi(addr & 0x7F);
    u1_t val = hal_spi(0x00);
#else
    spi_buf[0] = addr & 0x7F;
    spi_buf[1] = 0;
    SPI.transfer(spi_buf, 2);
    u1_t val = spi_buf[1];
#endif
    hal_pin_nss(1);
    return val;
}

void hal_spi_write_reg (u1_t addr, u1_t data) {
    hal_pin_nss(0);
#if !defined(ENERGIA_ARCH_CC13XX) && !defined(ENERGIA_ARCH_CC13X2)
    hal_spi(addr | 0x80);
    hal_spi(data);
#else
    spi_buf[0] = addr | 0x80;
    spi_buf[1] = data;
    SPI.transfer(spi_buf, 2);
#endif
    hal_pin_nss(1);
}

void hal_spi_read_buf (u1_t addr, u1_t* buf, u1_t len, u1_t inv) {
    hal_pin_nss(0);
#if !defined(ENERGIA_ARCH_CC13XX) && !defined(ENERGIA_ARCH_CC13X2)
    hal_spi(addr & 0x7F);
    u1_t i=0;
    for (i=0; i<len; i++) {
        buf[i] = (inv == 0 ? hal_spi(0x00) : ~(hal_spi(0x00)));
    }
#else
    spi_buf[0] = addr & 0x7F;
    u1_t i = 0;
    for (i=0; i<len; i++) {
        spi_buf[i+1] = 0;
    }
    SPI.transfer(spi_buf, len+1);
    for (i=0; i<len; i++) {
        buf[i] = (inv == 0 ? spi_buf[i+1] : ~spi_buf[i+1]);
    }
#endif
    hal_pin_nss(1);
}


void hal_spi_write_buf (u1_t addr, u1_t* buf, u1_t len, u1_t inv) {
    hal_pin_nss(0);
#if !defined(ENERGIA_ARCH_CC13XX) && !defined(ENERGIA_ARCH_CC13X2)
    hal_spi(addr | 0x80);
    u1_t i = 0;
    for (i=0; i<len; i++) {
        hal_spi(inv == 0 ? buf[i] : ~buf[i]);
    }
#else
    spi_buf[0] = addr | 0x80;
    u1_t i = 0;
    for (i=0; i<len; i++) {
        spi_buf[i+1] = (inv == 0 ? buf[i] : ~buf[i]);
    }
    SPI.transfer(spi_buf, len+1);
#endif
    hal_pin_nss(1);
}

// -----------------------------------------------------------------------------
// TIME

static void hal_time_init () {
    // Nothing to do
}

u4_t hal_ticks () {
    // Because micros() is scaled down in this function, micros() will
    // overflow before the tick timer should, causing the tick timer to
    // miss a significant part of its values if not corrected. To fix
    // this, the "overflow" serves as an overflow area for the micros()
    // counter. It consists of three parts:
    //  - The US_PER_OSTICK upper bits are effectively an extension for
    //    the micros() counter and are added to the result of this
    //    function.
    //  - The next bit overlaps with the most significant bit of
    //    micros(). This is used to detect micros() overflows.
    //  - The remaining bits are always zero.
    //
    // By comparing the overlapping bit with the corresponding bit in
    // the micros() return value, overflows can be detected and the
    // upper bits are incremented. This is done using some clever
    // bitwise operations, to remove the need for comparisons and a
    // jumps, which should result in efficient code. By avoiding shifts
    // other than by multiples of 8 as much as possible, this is also
    // efficient on AVR (which only has 1-bit shifts).
    static uint8_t overflow = 0;

    // Scaled down timestamp. The top US_PER_OSTICK_EXPONENT bits are 0,
    // the others will be the lower bits of our return value.
    uint32_t scaled = micros() >> US_PER_OSTICK_EXPONENT;
    // Most significant byte of scaled
    uint8_t msb = scaled >> 24;
    // Mask pointing to the overlapping bit in msb and overflow.
    const uint8_t mask = (1 << (7 - US_PER_OSTICK_EXPONENT));
    // Update overflow. If the overlapping bit is different
    // between overflow and msb, it is added to the stored value,
    // so the overlapping bit becomes equal again and, if it changed
    // from 1 to 0, the upper bits are incremented.
    overflow += (msb ^ overflow) & mask;

    // Return the scaled value with the upper bits of stored added. The
    // overlapping bit will be equal and the lower bits will be 0, so
    // bitwise or is a no-op for them.
    return scaled | ((uint32_t)overflow << 24);

    // 0 leads to correct, but overly complex code (it could just return
    // micros() unmodified), 8 leaves no room for the overlapping bit.
    static_assert(US_PER_OSTICK_EXPONENT > 0 && US_PER_OSTICK_EXPONENT < 8, "Invalid US_PER_OSTICK_EXPONENT value");
}

// Returns the number of ticks until time. Negative values indicate that
// time has already passed.
static s4_t delta_time(u4_t time) {
    return (s4_t)(time - hal_ticks());
}

void hal_waitUntil (u4_t time) {
    s4_t delta = delta_time(time);
    // From delayMicroseconds docs: Currently, the largest value that
    // will produce an accurate delay is 16383.
    while (delta > (16000 / US_PER_OSTICK)) {
        delay(16);
        delta -= (16000 / US_PER_OSTICK);
    }
    if (delta > 0)
        delayMicroseconds(delta * US_PER_OSTICK);
}

// check and rewind for target time
u1_t hal_checkTimer (u4_t time) {
    // No need to schedule wakeup, since we're not sleeping
    return delta_time(time) <= 0;
}

#if defined(ARDUINO_ARCH_STM32) || defined(ARDUINO_ARCH_SAMD) || \
    defined(ARDUINO_ARCH_RENESAS) || defined(ARDUINO_ARCH_CH32)

// Fix for STM32 HAL based cores.

// ARDUINO_ARCH_STM32 appears to be defined for these Arduino cores:
// - Arduino_Core_STM32 (aka stm32duino)
// - STM32GENERIC
// - BSFrance-stm32

// This fix solves an issue with STM32 HAL based Arduino cores where
// a call to os_init() hangs the MCU. The fix prevents LMIC-Arduino from
// disabling and re-enabling interrupts.
// While the exact cause is not known, it is assumed that disabling interrupts
// may conflict with interrupts required for the STM32 HAL core.
// (Possible side-effects on LMIC timing have not been checked.)

void hal_disableIRQs () {
}

void hal_enableIRQs () {
    hal_io_check();
}

#else /* ARDUINO_ARCH_STM32 */

static uint8_t irqlevel = 0;

void hal_disableIRQs () {
    noInterrupts();
    irqlevel++;
}

void hal_enableIRQs () {
    if(--irqlevel == 0) {
        interrupts();

        // Instead of using proper interrupts (which are a bit tricky
        // and/or not available on all pins on AVR), just poll the pin
        // values. Since os_runloop disables and re-enables interrupts,
        // putting this here makes sure we check at least once every
        // loop.
        //
        // As an additional bonus, this prevents the can of worms that
        // we would otherwise get for running SPI transfers inside ISRs
        hal_io_check();
    }
}

#endif /* ARDUINO_ARCH_STM32 */

void lmic_hal_sleep () {
    // Not implemented
}

// -----------------------------------------------------------------------------

#if defined(LMIC_PRINTF_TO)
void hal_printf(char *fmt, ... )
{
    char buf[80]; // resulting string limited to 128 chars
    va_list args;
    va_start (args, fmt );
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end (args);
    LMIC_PRINTF_TO.print(buf);
}
#endif

void lmic_hal_init (void *bootarg) {
    // configure radio I/O and interrupt handler
    hal_io_init();
    // configure radio SPI
    hal_spi_init();
    // configure timer and interrupt handler
    hal_time_init();
}

void hal_failed (const char *file, u2_t line) {
#if defined(LMIC_FAILURE_TO)
    LMIC_FAILURE_TO.println("FAILURE ");
    LMIC_FAILURE_TO.print(file);
    LMIC_FAILURE_TO.print(':');
    LMIC_FAILURE_TO.println(line);
    LMIC_FAILURE_TO.flush();
#endif
    hal_disableIRQs();
    while(1);
}
