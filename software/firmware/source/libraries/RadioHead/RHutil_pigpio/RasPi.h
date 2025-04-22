// RasPi.h
//(9/22/2019)   Contributed by Brody M. This file is based off RHutil\RasPi.h 
//              but modified for the pigpio library instead of BCM2835. Original
//              code maintained where possible. Unused code commented out and 
//              left in place. WiringPinMode enumeration declaration borrowed from 
//              STM32ArduinoCompat\wirish.h. Also some enumeration declarations 
//              "borrowed" from bcm2835.h to maintain original code and for simplicity.

// Routines for implementing RadioHead on Raspberry Pi
// using BCM2835 library for GPIO
// Contributed by Mike Poublon and used with permission

#ifndef RASPI_h
#define RASPI_h

#include <pigpio.h>

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdint.h>

typedef unsigned char byte;

#ifndef NULL
  #define NULL 0
#endif

#define HIGH 0x1
#define LOW  0x0

#define CHANGE 1
#define FALLING 2
#define RISING 3

#define memcpy_P memcpy

//#ifndef OUTPUT
//  #define OUTPUT BCM2835_GPIO_FSEL_OUTP
//#endif

class SPIClass
{
  public:
    //pigpio SPI ID
    //We need to make sure this handle can be accessed by all SPI Functions
    static byte transfer(byte _data);
    // SPI Configuration methods
    static void begin(); // Default
    //static void begin(uint32_t,uint32_t,uint32_t);
    static void begin(uint16_t, uint8_t, uint8_t);
    static void end();
    //static void setBitOrder(uint8_t);
    //static void setDataMode(uint8_t);
    //static void setClockDivider(uint16_t);
    static uint32_t convertClockDivider(uint16_t);
};

extern SPIClass SPI;

class SerialSimulator
{
  public:
    #define DEC 10
    #define HEX 16
    #define OCT 8
    #define BIN 2

    // TODO: move these from being inlined
    static void begin(int baud);
    static size_t println(const char* s);
    static size_t print(const char* s);
    static size_t print(unsigned int n, int base = DEC);
    static size_t print(char ch);
    static size_t println(char ch);
    static size_t print(unsigned char ch, int base = DEC);
    static size_t println(unsigned char ch, int base = DEC);
};

extern SerialSimulator Serial;

void RasPiSetup();

//The WiringPinMode enumeration declaration is borrowed from STM32ArduinoCompat\wirish.h

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

void pinMode(uint8_t pin, WiringPinMode mode);

//void pinMode(unsigned char pin, unsigned char mode);

void digitalWrite(unsigned char pin, unsigned char value);

unsigned long millis();

void delay (unsigned long delay);

long random(long min, long max);

void attachInterrupt(unsigned char pin, void (*handler)(void), int mode);



//The following lines are borrowed from bcm2835.h, which is part of the BCM2835 library
//(https://www.airspayce.com/mikem/bcm2835/). The original RadioHead library expects 
//BCM2835. We could eliminate this by modifying more RadioHead code, but this leaves 
//the library more "original". Another option would be to include bcm2835.h directly,
//but this method is easier.

 typedef enum
 {
     BCM2835_SPI_BIT_ORDER_LSBFIRST = 0,  
     BCM2835_SPI_BIT_ORDER_MSBFIRST = 1   
 }bcm2835SPIBitOrder;
 
 typedef enum
 {
     BCM2835_SPI_MODE0 = 0,  
     BCM2835_SPI_MODE1 = 1,  
     BCM2835_SPI_MODE2 = 2,  
     BCM2835_SPI_MODE3 = 3   
 }bcm2835SPIMode;
 
 typedef enum
 {
     BCM2835_SPI_CS0 = 0,     
     BCM2835_SPI_CS1 = 1,     
     BCM2835_SPI_CS2 = 2,     
     BCM2835_SPI_CS_NONE = 3  
 } bcm2835SPIChipSelect;
 
 typedef enum
 {
     BCM2835_SPI_CLOCK_DIVIDER_65536 = 0,       
     BCM2835_SPI_CLOCK_DIVIDER_32768 = 32768,   
     BCM2835_SPI_CLOCK_DIVIDER_16384 = 16384,   
     BCM2835_SPI_CLOCK_DIVIDER_8192  = 8192,    
     BCM2835_SPI_CLOCK_DIVIDER_4096  = 4096,    
     BCM2835_SPI_CLOCK_DIVIDER_2048  = 2048,    
     BCM2835_SPI_CLOCK_DIVIDER_1024  = 1024,    
     BCM2835_SPI_CLOCK_DIVIDER_512   = 512,     
     BCM2835_SPI_CLOCK_DIVIDER_256   = 256,     
     BCM2835_SPI_CLOCK_DIVIDER_128   = 128,     
     BCM2835_SPI_CLOCK_DIVIDER_64    = 64,      
     BCM2835_SPI_CLOCK_DIVIDER_32    = 32,      
     BCM2835_SPI_CLOCK_DIVIDER_16    = 16,      
     BCM2835_SPI_CLOCK_DIVIDER_8     = 8,       
     BCM2835_SPI_CLOCK_DIVIDER_4     = 4,       
     BCM2835_SPI_CLOCK_DIVIDER_2     = 2,       
     BCM2835_SPI_CLOCK_DIVIDER_1     = 1        
 } bcm2835SPIClockDivider;


#endif
