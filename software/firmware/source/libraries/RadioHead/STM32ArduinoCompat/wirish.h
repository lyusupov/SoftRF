// ArduinoCompat/wirish.h

#ifndef _wirish_h
#define _wirish_h

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stm32f4xx_rng.h>

#define PROGMEM
#define memcpy_P memcpy

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

extern void pinMode(uint8_t pin, WiringPinMode mode);
extern uint32_t millis();
extern void delay(uint32_t millis);
extern void attachInterrupt(uint8_t, void (*)(void), int mode);
extern void digitalWrite(uint8_t pin, uint8_t val);
extern uint8_t digitalRead(uint8_t pin);

//extern long random(long to);
//extern long random(long from, long to);

#define HIGH 0x1
#define LOW  0x0

#define LSBFIRST 0
#define MSBFIRST 1

#define CHANGE 1
#define FALLING 2
#define RISING 3

// Equivalent to HardwareSerial in Arduino
class SerialUSBClass
{
public:
#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

    // TODO: move these from being inlined
    void begin(int baud) {}

    size_t println(const char* s)
    {
        print(s);
        printf("\n");
        return 0;
    }
    size_t print(const char* s)
    {
        printf(s);
        return 0;
    }
    size_t print(unsigned int n, int base = DEC)
    {
        if (base == DEC)
            printf("%d", n);
        else if (base == HEX)
            printf("%02x", n);
        else if (base == OCT)
	    printf("%o", n);
        // TODO: BIN
	return 0;
    }
    size_t print(char ch)
    {
       printf("%c", ch);
       return 0;
    }
    size_t println(char ch)
    {
        printf("%c\n", ch);
	return 0;
    }
    size_t print(unsigned char ch, int base = DEC)
    {
        return print((unsigned int)ch, base);
    }
    size_t println(unsigned char ch, int base = DEC)
    {
        print((unsigned int)ch, base);
        printf("\n");
	return 0;
    }

};

// Global instance of the Serial output
extern SerialUSBClass SerialUSB;

#endif

