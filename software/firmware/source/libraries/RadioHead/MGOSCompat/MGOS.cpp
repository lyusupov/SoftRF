// RasPi.cpp
//
// Routines for implementing RadioHead on Raspberry Pi
// using BCM2835 library for GPIO
//
// Contributed by Mike Poublon and used with permission


#include <RadioHead.h>

#if (RH_PLATFORM == RH_PLATFORM_MONGOOSE_OS)

#include "mgos.h"

int pwmFreq = 1000;
float pwmDutyCycle = 0.5;

/**
 * @brief Set the direction of a GPIO pin.
 * @param pin the Pin whose direction is to be set.
 * @param mode The direction of the pin (OUTPUT or INPUT)
 **/
void pinMode(uint8_t pin, WiringPinMode mode)
{

  //SPI CS GPIO controlled by MGOS lib call so don't allow it to be set here
  if( SPI.getCSGpio() != pin ) {
      if (mode == OUTPUT)
      {
        mgos_gpio_set_mode(pin, MGOS_GPIO_MODE_OUTPUT);
      }
      else if (mode == OUTPUT_OPEN_DRAIN)
      {
          mgos_gpio_set_pull(pin, MGOS_GPIO_PULL_UP);
          mgos_gpio_set_mode(pin, MGOS_GPIO_MODE_OUTPUT);
      }
      else if (mode == INPUT || mode == INPUT_FLOATING )
      {
          mgos_gpio_set_mode(pin, MGOS_GPIO_MODE_INPUT);
      }
      else if (mode == INPUT_ANALOG)
      {
          mgos_adc_enable(pin);
      }
      else if (mode == INPUT_PULLUP)
      {
          mgos_gpio_set_pull(pin, MGOS_GPIO_PULL_UP);
          mgos_gpio_set_mode(pin, MGOS_GPIO_MODE_INPUT);
      }
      else if (mode == INPUT_PULLDOWN)
      {
          mgos_gpio_set_pull(pin, MGOS_GPIO_PULL_DOWN);
          mgos_gpio_set_mode(pin, MGOS_GPIO_MODE_INPUT);
      }
      else if (mode == PWM)
      {
          mgos_pwm_set(pin, pwmFreq, pwmDutyCycle);
      }
      else if (mode == PWM_OPEN_DRAIN) {
          mgos_pwm_set(pin, pwmFreq, pwmDutyCycle);
      }
  }
}

/**
 * @brief Set the state of a GPIO pin.
 * @param pin the Pin whose state is to be set.
 * @param value The state of the pin.
 */
void digitalWrite(unsigned char pin, unsigned char value)
{

    //SPI CS GPIO controlled by MGOS lib call so don't allow it to be set here
    if( SPI.getCSGpio() != pin ) {
        mgos_gpio_write(pin,value);
    }
}

/**
 * @brief Read the state of a GPIO pin.
 * @param pin the Pin whose state is to be set.
 * @return 1 If high, 0 if low.
 */
uint8_t digitalRead(uint8_t pin)
{
    uint8_t pinState=0;
    //SPI CS GPIO controlled by MGOS lib call so don't allow it to be set here
    if( SPI.getCSGpio() != pin ) {
        pinState = (uint8_t)mgos_gpio_read(pin);
    }
    return pinState;
}

/**
 * @brief Get the number of elapsed milliseconds since the last boot.
 */
uint32_t millis(void)
{
  return (uint32_t)mgos_uptime_micros()/1000;
}

/**
 * @brief Provide a delay in milliseconds.
 * @param ms The number of Milli Seconds to delay.
 */
void delay (unsigned long ms)
{
  mgos_msleep(ms);
}

/**
 * @brief Generate a random number between limits.
 * @param min The minimum random value to be generated.
 * @param max The maximum random value to be generated.
 */
long random(long min, long max)
{
  return mgos_rand_range( (float)min, (float)max);
}

static void mgos_gpio_int_handler(int pin, void *arg) {
    void (*handler)(void) = (void (*)())arg;
    //Note that this handler is executed in interrupt context (ISR)
    //therefore ensure that actions performed here are acceptable for the
    //platform on which the code will execute.
    //E.G
    //Use of the LOG macro to send debug data on the serial port crashes
    //esp8266 and esp32 code.
    handler();
    (void) pin;
    (void) arg;
}

void attachInterrupt(uint8_t pin, void (*handler)(void), int rh_mode)
{
    mgos_gpio_int_mode mgos_mode = MGOS_GPIO_INT_NONE;
    if( rh_mode == CHANGE ) {
        mgos_mode = MGOS_GPIO_INT_EDGE_ANY;
    } else if( rh_mode == FALLING ) {
        mgos_mode = MGOS_GPIO_INT_EDGE_NEG;
    } else if( rh_mode == RISING ) {
        mgos_mode = MGOS_GPIO_INT_EDGE_POS;
    }
    mgos_gpio_set_int_handler_isr((int)pin, mgos_mode, mgos_gpio_int_handler, (void*)handler);
}

void enableInterupt(uint8_t pin) {
    mgos_gpio_enable_int(pin);
}

/**
 * @brief Perform functions that under Mongoose OS we would normally return
 * to the RTOS. E,G flush the TX UART buffer.
 */
void mgosYield(void) {
    mgos_uart_flush(RH_SERIAL_PORT);
}

#endif
