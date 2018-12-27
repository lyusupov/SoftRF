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
#include <netinet/in.h>

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
#define digitalPinToInterrupt(p)             (p)
#define attachInterrupt(pin, userFunc, mode) {}

#define CHANGE 3
#define FALLING 1
#define RISING 2

// Delay macros
#define delay(x) bcm2835_delay(x)
#define delayMicroseconds(m) bcm2835_delayMicroseconds(m)

#include <pthread.h>
#define yield() pthread_yield()

// No memcpy_P/PROGMEM on Raspberry PI
#ifndef memcpy_P
#define memcpy_P memcpy 
#endif

#ifndef PROGMEM
#define PROGMEM
#endif

#ifndef snprintf_P
#define snprintf_P snprintf
#endif

#ifndef strcmp_P
#define strcmp_P strcmp
#endif

// F() Macro
#define F(s)     (s)
#define PSTR(s)  (s)

//#define random(x) (rand() % x)
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#define pgm_read_byte_near(addr) pgm_read_byte(addr)
#define pgm_read_float(addr) (*(const float *)(addr))

// WMath prototypes
long random(long);
long random(long, long);
void randomSeed(unsigned long);

#define PI 3.1415926535897932384626433832795
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)
#define sq(x) ((x)*(x))

#define _BV(bit) (1 << (bit))

typedef unsigned char byte;
typedef bool boolean;
typedef in_addr_t IPAddress;

char *dtostrf(double val, signed char width, unsigned char prec, char *sout);

// Checks for any printable character including space.
inline boolean isPrintable(int c){
  return ( isprint (c) == 0 ? false : true);
}

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
    static size_t write(const char * s);
    static size_t available(void);
    static size_t read(void);
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

