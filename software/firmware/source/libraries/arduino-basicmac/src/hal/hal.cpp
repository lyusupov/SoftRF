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

#if defined(ARDUINO_ARCH_RENESAS)
#include <SoftSPI.h>
extern  SoftSPI RadioSPI;
#undef  SPI
#define SPI RadioSPI
#endif /* ARDUINO_ARCH_RENESAS */

#if defined(ARDUINO_ARCH_CH32)
extern  SPIClass RadioSPI;
#undef  SPI
#define SPI RadioSPI
#endif /* ARDUINO_ARCH_CH32 */

#include "../basicmac.h"
#include "hal.h"
#define _GNU_SOURCE 1 // For fopencookie
#include <stdio.h>
#undef _GNU_SOURCE

#if defined(ARDUINO_NUCLEO_L073RZ)
//#define Serial  Serial1
#endif

// Datasheet defins typical times until busy goes low. Most are < 200us,
// except when waking up from sleep, which typically takes 3500us. Since
// we cannot know here if we are in sleep, we'll have to assume we are.
// Since 3500 is typical, not maximum, wait a bit more than that.
static unsigned long MAX_BUSY_TIME = 5000;

// -----------------------------------------------------------------------------
// I/O
// in, case we have no DIO mapping to a GPIO pin, we'll need to read 
// Lora Module IRQ register
static bool check_dio = 0;

static void hal_interrupt_init(); // Fwd declaration

static void hal_io_init () {
    uint8_t i;
    ASSERT(lmic_pins.nss != LMIC_UNUSED_PIN);

#if defined(BRD_sx1272_radio) || defined(BRD_sx1276_radio)
//    ASSERT(lmic_pins.dio[0] != LMIC_UNUSED_PIN);
//    ASSERT(lmic_pins.dio[1] != LMIC_UNUSED_PIN || lmic_pins.dio[2] != LMIC_UNUSED_PIN);
#elif defined(BRD_sx1261_radio) || defined(BRD_sx1262_radio)
//    ASSERT(lmic_pins.dio[0] == LMIC_UNUSED_PIN);
    ASSERT(lmic_pins.dio[1] == LMIC_UNUSED_PIN);
    ASSERT(lmic_pins.dio[2] == LMIC_UNUSED_PIN);
#else
    #error "Unknown radio type?"
#endif

#if defined(BRD_sx1261_radio) || defined(BRD_sx1262_radio)
//    ASSERT(lmic_pins.busy != LMIC_UNUSED_PIN);
#endif

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
    if (lmic_pins.busy != LMIC_UNUSED_PIN)
        pinMode(lmic_pins.busy, INPUT);
    if (lmic_pins.tcxo != LMIC_UNUSED_PIN)
#if defined(ARDUINO_NUCLEO_L073RZ)
      if (lmic_pins.tcxo == PD_7)
        hal_pin_tcxo_init();
      else
#endif /* ARDUINO_NUCLEO_L073RZ */
        pinMode(lmic_pins.tcxo, OUTPUT);

    for (i = 0; i < NUM_DIO; ++i) {
        if (lmic_pins.dio[i] != LMIC_UNUSED_PIN)
            pinMode(lmic_pins.dio[i], INPUT);
    }

    hal_interrupt_init();
}

// rx = 0, tx = 1, off = -1
void hal_pin_rxtx (s1_t val) {
    if (lmic_pins.txe != LMIC_UNUSED_PIN)
        digitalWrite(lmic_pins.txe, val == 1 ? HIGH : LOW);
    if (lmic_pins.rxe != LMIC_UNUSED_PIN)
        digitalWrite(lmic_pins.rxe, val == 0 ? HIGH : LOW);
}

#if defined(ARDUINO_ARCH_ASR6601)
bool hal_pin_rst (u1_t val) {
    if (val) SX126xReset();

    return true;
}
#elif defined(ARDUINO_GENERIC_WLE5CCUX)
bool hal_pin_rst (u1_t val) {
#if 0
    if (val == 0)
    {
        LL_RCC_RF_EnableReset();
        LL_RCC_HSE_EnableTcxo();
        LL_RCC_HSE_Enable();
        while (!LL_RCC_HSE_IsReady());
    }
    else
        LL_RCC_RF_DisableReset();
#endif
    return true;
}
#else
// set radio RST pin to given value (or keep floating!)
bool hal_pin_rst (u1_t val) {
    if (lmic_pins.rst == LMIC_UNUSED_PIN)
        return false;

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
    return true;
}
#endif /* ARDUINO_ARCH_ASR6601 || ARDUINO_GENERIC_WLE5CCUX */

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
                radio_irq_handler(i, hal_ticks());
            }
#else
            if (dio_states[i] != digitalRead(lmic_pins.dio[i])) {
                dio_states[i] = !dio_states[i];
                if (dio_states[i]) {
                    radio_irq_handler(i, hal_ticks());
                }
            }
#endif
        }
    } else {
        // Check IRQ flags in radio module
        if ( radio_has_irq() ) {
            radio_irq_handler(0, hal_ticks());
        }
    }

}

#else /* TBD */
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
            radio_irq_handler(i, hal_ticks());
        }
    }
}
#endif // LMIC_USE_INTERRUPTS


void hal_irqmask_set (int mask) {
    // Not implemented
}

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
#if !defined(__ASR6501__)
    if (lmic_pins.tcxo == LMIC_UNUSED_PIN)
        return false;

#if defined(ARDUINO_NUCLEO_L073RZ)
      if (lmic_pins.tcxo == PD_7)
          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7,
                            val == 1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
      else
#endif /* ARDUINO_NUCLEO_L073RZ */

    digitalWrite(lmic_pins.tcxo, val == 1 ? HIGH : LOW);

#if !defined(ARDUINO_GENERIC_WLE5CCUX)
    return true;
#else
    return false;
#endif /* ARDUINO_GENERIC_WLE5CCUX */
#else
    return lmic_pins.tcxo == lmic_pins.rst ? false : true;
#endif /* __ASR6501__ */
}

#if defined(ARDUINO_ARCH_ASR6601)
void hal_pin_busy_wait (void) {
    unsigned long start = micros();

    while(((micros() - start) < MAX_BUSY_TIME) && (LORAC->SR & 0x100)) /* wait */;
}
#elif defined(ARDUINO_GENERIC_WLE5CCUX)
void hal_pin_busy_wait (void) {
    unsigned long start = micros();

    while(((micros() - start) < MAX_BUSY_TIME) && LL_PWR_IsActiveFlag_RFBUSYS()) /* wait */;
}
#else
void hal_pin_busy_wait (void) {
    if (lmic_pins.busy == LMIC_UNUSED_PIN) {
        // TODO: We could probably keep some state so we know the chip
        // is in sleep, since otherwise the delay can be much shorter.
        // Also, all delays after commands (rather than waking up from
        // sleep) are measured from the *end* of the previous SPI
        // transaction, so we could wait shorter if we remember when
        // that was.
        delayMicroseconds(MAX_BUSY_TIME);
    } else {
        unsigned long start = micros();

        while((micros() - start) < MAX_BUSY_TIME && digitalRead(lmic_pins.busy)) /* wait */;
    }
}
#endif /* ARDUINO_ARCH_ASR6601 || ARDUINO_GENERIC_WLE5CCUX */

// -----------------------------------------------------------------------------
// SPI

#if defined(RASPBERRY_PI)
// Raspberry Pi 2:
//    BCM2835_CORE_CLK_HZ = 250000000
//    Clock divider / 64 = 3.906 MHz
static const SPISettings settings(BCM2835_SPI_CLOCK_DIVIDER_64, BCM2835_SPI_BIT_ORDER_MSBFIRST, BCM2835_SPI_MODE0);
#else
static const SPISettings settings(LMIC_SPI_FREQ, MSBFIRST, SPI_MODE0);
#endif

#if defined(ARDUINO_ARCH_ASR6601)

static void hal_spi_init () {
    /* LMIC_SPI_FREQ = 1 MHz , 8 bit */
    LORAC->SSP_CR0  = (11UL /* SCR */ << 8) | 0x07 /* DSS */;
    LORAC->SSP_CPSR = 0x02; /* CPSDVR */
}

void hal_spi_select (int on) {
    LORAC->NSS_CR = (!on ? HIGH : LOW);
}

// perform SPI transaction with radio
u1_t hal_spi (u1_t out) {
    u1_t res = SpiInOut(out);
    return res;
}

#elif defined(ARDUINO_GENERIC_WLE5CCUX)
#ifdef HAL_SUBGHZ_MODULE_ENABLED

#define SUBGHZ_DEFAULT_TIMEOUT     100U    /* HAL Timeout in ms               */
/* SystemCoreClock dividers. Corresponding to time execution of while loop.   */
#define SUBGHZ_DEFAULT_LOOP_TIME   ((SystemCoreClock*28U)>>19U)

static SUBGHZ_HandleTypeDef hsubghz = {.Init = {.BaudratePrescaler =
                                               SUBGHZSPI_BAUDRATEPRESCALER_16 } };
#endif /* HAL_SUBGHZ_MODULE_ENABLED */

static void hal_spi_init () {
#ifdef HAL_SUBGHZ_MODULE_ENABLED
    HAL_SUBGHZ_Init(&hsubghz);
#endif /* HAL_SUBGHZ_MODULE_ENABLED */
}

void hal_spi_select (int on) {
    if (on)
        LL_PWR_SelectSUBGHZSPI_NSS();
    else
        LL_PWR_UnselectSUBGHZSPI_NSS();
}

// perform SPI transaction with radio
u1_t hal_spi (u1_t out) {
#ifdef HAL_SUBGHZ_MODULE_ENABLED
  HAL_StatusTypeDef status = HAL_OK;
  __IO uint32_t count;

  count = SUBGHZ_DEFAULT_TIMEOUT * SUBGHZ_DEFAULT_LOOP_TIME;

  /* Wait until TXE flag is set */
  do
  {
    if (count == 0U)
    {
      status = HAL_ERROR;
      hsubghz.ErrorCode = HAL_SUBGHZ_ERROR_TIMEOUT;
      break;
    }
    count--;
  } while (READ_BIT(SUBGHZSPI->SR, SPI_SR_TXE) != (SPI_SR_TXE));

  __IO uint8_t *spidr = ((__IO uint8_t *)&SUBGHZSPI->DR);
  *spidr = out;

  count = SUBGHZ_DEFAULT_TIMEOUT * SUBGHZ_DEFAULT_LOOP_TIME;

  /* Wait until RXNE flag is set */
  do
  {
    if (count == 0U)
    {
      status = HAL_ERROR;
      hsubghz.ErrorCode = HAL_SUBGHZ_ERROR_TIMEOUT;
      break;
    }
    count--;
  } while (READ_BIT(SUBGHZSPI->SR, SPI_SR_RXNE) != (SPI_SR_RXNE));

  return (uint8_t)(READ_REG(SUBGHZSPI->DR));
#else
  return 0;
#endif /* HAL_SUBGHZ_MODULE_ENABLED */
}

u1_t lmic_wle_rf_output = HIGH;

void hal_set_rf_output (u1_t val) {
    lmic_wle_rf_output = val;
}

#else

static void hal_spi_init () {
#if defined(ENERGIA_ARCH_CC13XX) || defined(ENERGIA_ARCH_CC13X2)
    SPI.setClockDivider(SPI_CLOCK_MAX / LMIC_SPI_FREQ);
#endif
    SPI.begin();
}

void hal_spi_select (int on) {

#if defined(SPI_HAS_TRANSACTION)
    if (on)
        SPI.beginTransaction(settings);
    else
        SPI.endTransaction();
#endif

    //Serial.println(val?">>":"<<");
    digitalWrite(lmic_pins.nss, !on ? HIGH : LOW);
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
#endif /* ARDUINO_ARCH_ASR6601 || ARDUINO_GENERIC_WLE5CCUX */

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

lmic_u8_t hal_xticks (void) {
    // TODO
    return hal_ticks();
}
/* Not actually used now
s2_t hal_subticks (void) {
    // TODO
    return 0;
}
*/

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

#if defined(ARDUINO_ARCH_STM32) || defined(ARDUINO_ARCH_NRF52) || \
    defined(__ASR6501__)  || defined(ARDUINO_ARCH_ASR650X) || \
    defined(RASPBERRY_PI) || defined(ARDUINO_ARCH_SAMD)    || \
    defined(ARDUINO_AVR_MEGA2560) || defined(ARDUINO_ARCH_ASR6601) || \
    defined(ARDUINO_ARCH_RP2040)  || defined(ARDUINO_ARCH_RP2350)  || \
    defined(ARDUINO_ARCH_RENESAS) || defined(ARDUINO_ARCH_SILABS)  || \
    defined(ARDUINO_ARCH_CH32)

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

u1_t lmic_hal_sleep (u1_t type, u4_t targettime) {
    // Actual sleeping not implemented, but jobs are only run when this
    // function returns 0, so make sure we only do that when the
    // targettime is close. When asked to sleep forever (until woken up
    // by an interrupt), just return immediately to keep polling.
    if (type == HAL_SLEEP_FOREVER)
        return 0;

    // TODO: What value should we use for "close"?
    return delta_time(targettime) < 10 ? 0 : 1;
}

void hal_watchcount (int cnt) {
    // Not implemented
}

// -----------------------------------------------------------------------------
// DEBUG

#ifdef CFG_DEBUG
static void hal_debug_init() {
    #ifdef LED_BUILTIN
    pinMode(LED_BUILTIN, OUTPUT);
    #endif
}

void hal_debug_str (const char* str) {
    Serial.print(str);
}

void hal_debug_led (int val) {
    #ifdef LED_BUILTIN
    digitalWrite(LED_BUILTIN, val);
    #endif
}
#endif

// -----------------------------------------------------------------------------

#if defined(LMIC_PRINTF_TO)
#if defined(__AVR__)
// On AVR, use the AVR-specific fdev_setup_stream to redirect stdout
static int uart_putchar (char c, FILE *)
{
    LMIC_PRINTF_TO.write(c) ;
    return 0 ;
}

void hal_printf_init() {
    // create a FILE structure to reference our UART output function
    static FILE uartout;
    memset(&uartout, 0, sizeof(uartout));

    // fill in the UART file descriptor with pointer to writer.
    fdev_setup_stream (&uartout, uart_putchar, NULL, _FDEV_SETUP_WRITE);

    // The uart is the standard output device STDOUT.
    stdout = &uartout ;
}
#else
// On non-AVR platforms, use the somewhat more complex "cookie"-based
// approach to custom streams. This is a GNU-specific extension to libc.
static ssize_t uart_putchar (void *, const char *buf, size_t len) {
    auto res = LMIC_PRINTF_TO.write(buf, len);
    // Since the C interface has no meaningful way to flush (fflush() is a
    // no-op on AVR since stdio does not introduce any buffering), just flush
    // every byte.
    LMIC_PRINTF_TO.flush();
    return res;
}

static cookie_io_functions_t functions = {
    .read = NULL,
    .write = uart_putchar,
    .seek = NULL,
    .close = NULL
};

void hal_printf_init() {
    stdout = fopencookie(NULL, "w", functions);
    // Disable buffering, so the callbacks get called right away
    setbuf(stdout, nullptr);
}
#endif // !defined(__AVR__)
#endif // defined(LMIC_PRINTF_TO)

void lmic_hal_init (void *bootarg) {
    // configure radio I/O and interrupt handler
    hal_io_init();
    // configure radio SPI
    hal_spi_init();
    // configure timer and interrupt handler
    hal_time_init();
#if defined(LMIC_PRINTF_TO)
    // printf support
    hal_printf_init();
#endif
#ifdef CFG_DEBUG
    hal_debug_init();
#endif
}

void hal_failed () {
#if defined(NRF52840_XXAA) && !defined(USE_TINYUSB)
    Serial1.flush();
#else
    Serial.flush();
#endif
    // keep IRQs enabled, to allow e.g. USB to continue to run and allow
    // firmware uploads on boards with native USB.
    while(1);
}

void hal_reboot (void) {
    // TODO
    hal_failed();
}

u1_t hal_getBattLevel (void) {
    // Not implemented
    return 0;
}

void hal_setBattLevel (u1_t level) {
    // Not implemented
}

void hal_fwinfo (hal_fwi* fwi) {
    // Not implemented
}

u1_t* hal_joineui (void) {
    return nullptr;
}

u1_t* hal_deveui (void) {
    return nullptr;
}

u1_t* hal_nwkkey (void) {
    return nullptr;
}

u1_t* hal_appkey (void) {
    return nullptr;
}

u1_t* hal_serial (void) {
    return nullptr;
}

u4_t  hal_region (void) {
    return 0;
}

u4_t  hal_hwid (void) {
    return 0;
}

u4_t  hal_unique (void) {
    return 0;
}

u4_t hal_dnonce_next (void) {
    return os_getRndU2();
}
