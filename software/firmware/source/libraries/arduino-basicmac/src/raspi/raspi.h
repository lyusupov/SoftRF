// raspi.h
//
// Routines for implementing Arduino-LIMC on Raspberry Pi
// using BCM2835 library for GPIO
// This code has been grabbed from excellent RadioHead Library

#if defined(RASPBERRY_PI) || defined(LUCKFOX_LYRA)

#ifndef RASPI_h
#define RASPI_h

#if defined(USE_BCMLIB)
#include <bcm2835.h>
#endif /* USE_BCMLIB */
#if defined(USE_LGPIO)
#include <lgpio.h>
#endif /* USE_LGPIO */
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

#include <basicmac.h>
#include "hal/hal.h"

#include <raspi/WString.h>

#ifndef NULL
  #define NULL 0
#endif

#if defined(USE_BCMLIB)
#ifndef OUTPUT
  #define OUTPUT BCM2835_GPIO_FSEL_OUTP
#endif

#ifndef INPUT
  #define INPUT BCM2835_GPIO_FSEL_INPT
#endif
#endif /* USE_BCMLIB */

#if defined(USE_LGPIO)
#ifndef INPUT
  #define INPUT  (0)
#endif

#ifndef OUTPUT
  #define OUTPUT (1)
#endif

#ifndef HIGH
  #define HIGH   LG_HIGH
#endif

#ifndef LOW
  #define LOW    LG_LOW
#endif

#ifndef LSBFIRST
  #define LSBFIRST  0
#endif

#ifndef INPUT_PULLUP
  #define INPUT_PULLUP LG_SET_PULL_UP
#endif
#endif /* USE_LGPIO */

#ifndef MSBFIRST
  #define MSBFIRST  1
#endif

#ifndef SPI_MODE0
  #define SPI_MODE0 0
#endif

#ifndef NOT_A_PIN
  #define NOT_A_PIN 0xFF
#endif

#ifndef NOT_AN_INTERRUPT
  #define NOT_AN_INTERRUPT 0xFF
#endif

#ifndef LED_BUILTIN
  #define LED_BUILTIN NOT_A_PIN 
#endif

// We don't have IRQ on Raspberry PI
#define interrupts()   {}
#define noInterrupts() {}
#define digitalPinToInterrupt(p)             (NOT_AN_INTERRUPT)
static inline void attachInterrupt(uint32_t irq, void (*cb)(void), uint32_t mode) {}
static inline void detachInterrupt(uint32_t irq) {}

#define CHANGE  3
#define FALLING 1
#define RISING  2

// Delay macros
#if defined(USE_BCMLIB)
#define delay(x) bcm2835_delay(x)
#define delayMicroseconds(m) bcm2835_delayMicroseconds(m)
#endif /* USE_BCMLIB */

#if defined(USE_LGPIO)
static inline void delay(unsigned long ms) {
  if(ms == 0) {
    sched_yield();
    return;
  }

  lguSleep(ms / 1000.0);
}

static inline void delayMicroseconds(unsigned long us) {
  if(us == 0) {
    sched_yield();
    return;
  }

  lguSleep(us / 1000000.0);
}
#endif

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

#ifndef strlen_P
#define strlen_P strlen
#endif

#ifndef strcat_P
#define strcat_P strcat
#endif

// F() Macro
#define F(s)     (s)
#define PSTR(s)  (s)

//#define random(x) (rand() % x)
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#define pgm_read_byte_near(addr) pgm_read_byte(addr)
#define pgm_read_float(addr) (*(const float *)(addr))
#define pgm_read_ptr(addr) (*(void * const *)(addr))

// WMath prototypes
//long random(long);
//long random(long, long);
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
#if 0
typedef in_addr_t IPAddress;
#endif

char *dtostrf(double val, signed char width, unsigned char prec, char *sout);

// Checks for any printable character including space.
#include <ctype.h>
inline boolean isPrintable(int c){
  return ( isprint (c) == 0 ? false : true);
}

#define SPI_PRI 0
#define SPI_AUX 1
#define SPI_HAS_TRANSACTION

#ifdef __cplusplus

#if defined(USE_BCMLIB)
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
    SPIClass(uint8_t spi_bus=SPI_PRI);
    byte transfer(byte _data);
    // SPI Configuration methods
    void begin(); // Default
    void end();
    void beginTransaction(SPISettings settings);
    void endTransaction();
    void setBitOrder(uint8_t);
    void setDataMode(uint8_t);
    void setClockDivider(uint16_t);
private:
    int8_t _spi_num;
};

class TwoWire {

  public:
    TwoWire();
    void begin();
    void setClock(uint32_t);
    void beginTransmission(uint8_t);
    uint8_t endTransmission(void);
    uint8_t requestFrom(uint8_t, uint8_t);
    int read(void);
    size_t write(uint8_t);
    size_t write(const uint8_t *, size_t);
};

extern TwoWire Wire;
#endif /* USE_BCMLIB */

#if defined(USE_LGPIO)
extern bool lgpio_init();
extern void lgpio_fini();

extern void tone(uint32_t, unsigned int, unsigned long);
extern void noTone(uint32_t);

class SPISettings
{
  public:
    SPISettings(uint32_t Speed, uint8_t bitOrder, uint8_t dataMode) {
        init(Speed, bitOrder, dataMode);
    }

  private:
    void init(uint32_t Speed, uint8_t bitOrder, uint8_t dataMode) {
      this->Speed    = Speed   ;
      this->bitOrder = bitOrder;
      this->dataMode = dataMode;
    }

    uint32_t Speed    ;
    uint8_t  bitOrder ;
    uint8_t  dataMode ;
  friend class SPIClass;
};

class SPIClass {
  public:
    SPIClass(uint8_t spiChannel, uint32_t spiSpeed = 2000000, uint8_t spiDevice = 0)
      :
      _spiDevice(spiDevice),
      _spiChannel(spiChannel),
      _spiSpeed(spiSpeed) {
    }

    void transfer(uint8_t* out, size_t len, uint8_t* in) {
      int result = lgSpiXfer(_spiHandle, (char *)out, (char*)in, len);
      if(result < 0) {
        fprintf(stderr, "Could not perform SPI transfer: %s\n", lguErrorText(result));
      }
    }

    byte transfer(byte _data) {
      uint8_t out = _data;
      size_t len = 1;
      uint8_t in;

      int result = lgSpiXfer(_spiHandle, (char *) &out, (char*) &in, len);
      if (result < 0) {
        fprintf(stderr, "Could not perform SPI transfer: %s\n", lguErrorText(result));
        return 0;
      } else {
        return in;
      }
    }

    void begin() {
      // now the SPI
      if(_spiHandle < 0) {
        if((_spiHandle = lgSpiOpen(_spiDevice, _spiChannel, _spiSpeed, 0)) < 0) {
          fprintf(stderr, "Could not open SPI handle on 0: %s\n", lguErrorText(_spiHandle));
        }
      }
    }

    void end() {
      // stop the SPI
      if(_spiHandle >= 0) {
        lgSpiClose(_spiHandle);
        _spiHandle = -1;
      }
    }

    void beginTransaction(SPISettings settings) {}
    void endTransaction() {}

  private:
    // the HAL can contain any additional private members
    const unsigned int _spiSpeed;
    const uint8_t _spiDevice;
    const uint8_t _spiChannel;
    int _spiHandle = -1;
};
#endif /* USE_LGPIO */

#if defined(USE_SPI1)
extern SPIClass SPI1;
#if !defined(SPI)
#define SPI SPI1
#endif
#else
extern SPIClass SPI0;
#if !defined(SPI)
#define SPI SPI0
#endif
#endif

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
    static size_t print(String s);
    static size_t println(String s);
    static size_t print(const char* s);
    static size_t println(short signed int n);
    static size_t println(short unsigned int n);
    static size_t print(ostime_t n);
    static size_t print(unsigned long n);
    static size_t println(unsigned long n);
    static size_t print(unsigned int n, int base = DEC);
    static size_t print(char ch);
    static size_t println(char ch);
    static size_t println(int8_t n);
    static size_t print(float f);
    static size_t println(float f);
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

#endif /* __cplusplus */

#ifdef __cplusplus
extern "C"{
#endif

void 		printConfig(const uint8_t led) ;
void 		printKey(const char * name, const uint8_t * key, uint8_t len, bool lsb); 
void 		printKeys() ;
bool 		getDevEuiFromMac(uint8_t *);
char * 		getSystemTime(char * time_buff, int len);
void 		pinMode(unsigned char, unsigned char);
void 		digitalWrite(unsigned char, unsigned char);
unsigned char	digitalRead(unsigned char) ;
void		initialiseEpoch();
unsigned int 	millis();
unsigned int 	micros();

#ifdef __cplusplus
}
#endif

#endif // RASPI_h
#endif // RASPBERRY_PI
