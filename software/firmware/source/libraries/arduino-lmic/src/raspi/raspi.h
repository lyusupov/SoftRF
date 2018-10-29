// raspi.h
//
// Routines for implementing Arduino-LIMC on Raspberry Pi
// using BCM2835 library for GPIO
// This code has been grabbed from excellent RadioHead Library

#ifdef RASPBERRY_PI

#ifndef RASPI_h
#define RASPI_h

#include <bcm2835.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <assert.h>
#include <ifaddrs.h>
#include <netpacket/packet.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <sys/stat.h> 
#include <fcntl.h>
#include <unistd.h>
#include <math.h>

#include <lmic.h>
#include "hal/hal.h"

#include <raspi/WString.h>

#ifndef NULL
  #define NULL 0
#endif

#ifndef OUTPUT
  #define OUTPUT BCM2835_GPIO_FSEL_OUTP
#endif

#ifndef INPUT
  #define INPUT BCM2835_GPIO_FSEL_INPT
#endif

#ifndef NOT_A_PIN
  #define NOT_A_PIN 0xFF
#endif

#ifndef LED_BUILTIN
  #define LED_BUILTIN NOT_A_PIN 
#endif

// We don't have IRQ on Raspberry PI
#define interrupts()   {}
#define noInterrupts() {}

// Delay macros
#define delay(x) bcm2835_delay(x)
//#define delayMicroseconds(m) bcm2835_delayMicroseconds(m)

#include <pthread.h>
#define yield() pthread_yield()

// No memcpy_P/PROGMEM on Raspberry PI
#ifndef memcpy_P
#define memcpy_P memcpy 
#endif

#ifndef PROGMEM
#define PROGMEM
#endif

// F() Macro
#define F(s)  (s)

//#define random(x) (rand() % x)
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))

// WMath prototypes
long random(long);
long random(long, long);
void randomSeed(unsigned long);

typedef unsigned char byte;
typedef bool boolean;

class SPISettings 
{
  public:
    SPISettings(uint16_t divider, uint8_t bitOrder, uint8_t dataMode) {
        init(divider, bitOrder, dataMode);
    }
    SPISettings() {
        init(BCM2835_SPI_CLOCK_DIVIDER_256, BCM2835_SPI_BIT_ORDER_MSBFIRST, BCM2835_SPI_MODE0);
    }
  private:
    void init(uint16_t divider, uint8_t bitOrder, uint8_t dataMode) {
      this->divider  = divider ; 
      this->bitOrder = bitOrder;
      this->dataMode = dataMode;
    }

    uint16_t divider  ;
    uint8_t  bitOrder ;
    uint8_t  dataMode ;
  friend class SPIClass;
};

class SPIClass {
  public:
    static byte transfer(byte _data);
    // SPI Configuration methods
    static void begin(); // Default
    static void end();
    static void beginTransaction(SPISettings settings);
    static void endTransaction();
    static void setBitOrder(uint8_t);
    static void setDataMode(uint8_t);
    static void setClockDivider(uint16_t);
};

extern SPIClass SPI; 

class SerialSimulator {
  public:
    #define DEC 10
    #define HEX 16
    #define OCT 8
    #define BIN 2

    // TODO: move these from being inlined
    static void begin(int baud);
    static size_t println(void);
    static size_t println(const char* s);
    static size_t println(String s);
    static size_t print(const char* s);
    static size_t println(u2_t n); 
    static size_t print(ostime_t n);
    static size_t print(unsigned long n);
    static size_t print(unsigned int n, int base = DEC);
    static size_t print(char ch);
    static size_t println(char ch);
    static size_t print(unsigned char ch, int base = DEC);
    static size_t println(unsigned char ch, int base = DEC);
    static size_t write(char ch);
    static size_t write(unsigned char * s, size_t len);
    static void   flush(void);

};
extern SerialSimulator Serial;

#ifdef __cplusplus
extern "C"{
#endif

void 		printConfig(const uint8_t led) ;
void 		printKey(const char * name, const uint8_t * key, uint8_t len, bool lsb); 
void 		printKeys() ;
bool 		getDevEuiFromMac(uint8_t *);
char * 	getSystemTime(char * time_buff, int len);
void 		pinMode(unsigned char, unsigned char);
void 		digitalWrite(unsigned char, unsigned char);
unsigned char digitalRead(unsigned char) ;
void          initialiseEpoch();
unsigned int  millis();
unsigned int  micros();

#ifdef __cplusplus
}
#endif

#endif // RASPI_h
#endif // RASPBERRY_PI

