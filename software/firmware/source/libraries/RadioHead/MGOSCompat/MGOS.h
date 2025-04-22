// MGOS.h
//
// Routines for implementing RadioHead when using Mongoose OS
// see https://mongoose-os.com/mos.html for H/W support when using Mongoose OS
// Contributed by Paul Austen

#ifndef MGOS_h
#define MGOS_h

#define PROGMEM
#define memcpy_P memcpy

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

#define HIGH 0x1
#define LOW  0x0

#define LSBFIRST 0
#define MSBFIRST 1

#define CHANGE 1
#define FALLING 2
#define RISING 3

typedef enum WiringPinMode {
    OUTPUT, /**< Basic digital output: when the pin is HIGH, the
               voltage is held at +3.3v (Vcc) and when it is LOW, it
               is pulled down to ground. */

    OUTPUT_OPEN_DRAIN, /**< In open drain mode, the pin indicates
                          "low" by accepting current flow to ground
                          and "high" by providing increased
                          impedance. An example use would be to
                          connect a pin to a bus line (which is pulled
                          up to a positive voltage by a separate
                          supply through a large resistor). When the
                          pin is high, not much current flows through
                          to ground and the line stays at positive
                          voltage; when the pin is low, the bus
                          "drains" to ground with a small amount of
                          current constantly flowing through the large
                          resistor from the external supply. In this
                          mode, no current is ever actually sourced
                          from the pin. */

    INPUT, /**< Basic digital input. The pin voltage is sampled; when
              it is closer to 3.3v (Vcc) the pin status is high, and
              when it is closer to 0v (ground) it is low. If no
              external circuit is pulling the pin voltage to high or
              low, it will tend to randomly oscillate and be very
              sensitive to noise (e.g., a breath of air across the pin
              might cause the state to flip). */

    INPUT_ANALOG, /**< This is a special mode for when the pin will be
                     used for analog (not digital) reads.  Enables ADC
                     conversion to be performed on the voltage at the
                     pin. */

    INPUT_PULLUP, /**< The state of the pin in this mode is reported
                     the same way as with INPUT, but the pin voltage
                     is gently "pulled up" towards +3.3v. This means
                     the state will be high unless an external device
                     is specifically pulling the pin down to ground,
                     in which case the "gentle" pull up will not
                     affect the state of the input. */

    INPUT_PULLDOWN, /**< The state of the pin in this mode is reported
                       the same way as with INPUT, but the pin voltage
                       is gently "pulled down" towards 0v. This means
                       the state will be low unless an external device
                       is specifically pulling the pin up to 3.3v, in
                       which case the "gentle" pull down will not
                       affect the state of the input. */

    INPUT_FLOATING, /**< Synonym for INPUT. */

    PWM, /**< This is a special mode for when the pin will be used for
            PWM output (a special case of digital output). */

    PWM_OPEN_DRAIN, /**< Like PWM, except that instead of alternating
                       cycles of LOW and HIGH, the voltage on the pin
                       consists of alternating cycles of LOW and
                       floating (disconnected). */
} WiringPinMode;

extern "C" {
  void pinMode(uint8_t pin, WiringPinMode mode);
  void digitalWrite(unsigned char pin, unsigned char value);
  uint8_t digitalRead(uint8_t pin);
  uint32_t millis(void);
  void delay (unsigned long ms);
  long random(long min, long max);
  void attachInterrupt(uint8_t pin, void (*handler)(void), int rh_mode);
  void mgosYield(void);
  void enableInterupt(uint8_t pin);
}

#endif
