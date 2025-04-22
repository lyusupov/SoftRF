// RasPi.cpp
//(9/22/2019)   Contributed by Brody M. This file is based off RHutil\RasPi.cpp 
//              but modified for the pigpio library instead of BCM2835. Original
//              code maintained where possible. Unused code commented out and 
//              left in place.

// Routines for implementing RadioHead on Raspberry Pi
// using BCM2835 library for GPIO
//
// Contributed by Mike Poublon and used with permission


#include <RadioHead.h>

#if (RH_PLATFORM == RH_PLATFORM_RASPI)
#include <sys/time.h>
#include <time.h>
#include "RasPi.h"
#include <stdio.h>

int spiHandle;

//Initialize the values for sanity
timeval RHStartTime;

void SPIClass::begin()
{
  //Set SPI Defaults
  //Retaining BCM2835 macros for compatibility with RadioHead
  uint16_t divider = BCM2835_SPI_CLOCK_DIVIDER_256;
  uint8_t bitorder = BCM2835_SPI_BIT_ORDER_MSBFIRST;
  uint8_t datamode = BCM2835_SPI_MODE0;
  begin(divider, bitorder, datamode);
}

//void SPIClass::begin(uint32_t spiChannel, uint32_t spiBaud, uint32_t spiFlags)
void SPIClass::begin(uint16_t divider, uint8_t bitOrder, uint8_t dataMode)
{

  //Set CS pins polarity to low
  //bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, 0);

  //pigpio SPI Defailts
  //SPI Speed
  //BCM2835 divider of 256 is approx 1MHz SCLK, depending on model
  //uint32_t spiBaud = 1000000;
  //Spi Flag Settings
  //21 20 19 18 17 16 15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
  //b  b  b  b  b  b  R  T  n  n  n  n  W  A u2 u1 u0 p2 p1 p0  m  m
  //m m bits = mode
  //Mode 0 = 0 0

  uint32_t spiBaud = convertClockDivider(divider);
  //datamode is 0 to 3 on BCM2835
  uint32_t spiFlags = 0; //Zero is a good default start.
  //on pigpio, the least sig 2 bits set datamode, which will probably be zero. 
  spiFlags = 0x00000000 | (uint32_t) dataMode;
  //According to documentation, bitOrder for SPI MAIN in pigpio is always MSBFIRST. So bitOrder ignored.
  printf("\nSPI Settings:\nBaud rate=%d\nFlags=%d\n\n", spiBaud, spiFlags);
  spiHandle = spiOpen(0, spiBaud, spiFlags); //spiChannel assumed to be zero.

  //Initialize a timestamp for millis calculation
  gettimeofday(&RHStartTime, NULL);
}

void SPIClass::end()
{
  //End the SPI
  //bcm2835_spi_end();
  spiClose(spiHandle);
}

uint32_t SPIClass::convertClockDivider(uint16_t rate)
{
  //Simple divide default RPi SPI clock by divider amount.
  //Nominal clock at 250MHz for Zero.
  return 250000000/rate;
}
/*
//Thes functions aren't necessary
void SPIClass::setBitOrder(uint8_t bitOrder)
{
  //Set the SPI bit Order
  bcm2835_spi_setBitOrder(bitOrder);
}

void SPIClass::setDataMode(uint8_t mode)
{
  //Set SPI data mode
  bcm2835_spi_setDataMode(mode);
}

void SPIClass::setClockDivider(uint16_t rate)
{
  //Set SPI clock divider
  bcm2835_spi_setClockDivider(rate);
}
*/

byte SPIClass::transfer(byte _data)
{
  char txByte[1] = {(char)_data};
  char rxByte[1];
  //For RF Compatibility, just transfer 1 byte
  spiXfer(spiHandle, txByte, rxByte, 1);
  return (byte)rxByte[0];
}


//void pinMode(unsigned char pin, unsigned char mode)
void pinMode(uint8_t pin, WiringPinMode mode)
{
  if (mode == OUTPUT)
  {
    gpioSetMode(pin, PI_OUTPUT);
    //bcm2835_gpio_fsel(pin,BCM2835_GPIO_FSEL_OUTP);
  }
  else if (mode == INPUT)
  {
    gpioSetMode(pin, PI_INPUT);
    //bcm2835_gpio_fsel(pin,BCM2835_GPIO_FSEL_INPT);
  }
  else if (mode == INPUT_PULLUP)
  {
    gpioSetMode(pin, PI_INPUT);
    gpioSetPullUpDown(pin, PI_PUD_UP);
  }
  else if (mode == INPUT_PULLDOWN)
  {
    gpioSetMode(pin, PI_INPUT);
    gpioSetPullUpDown(pin, PI_PUD_DOWN);
  }
  else
  {
    //For safety
    gpioSetMode(pin, PI_INPUT);
  }
}

void digitalWrite(unsigned char pin, unsigned char value)
{
  //bcm2835_gpio_write(pin,value);
  //Could have just written gpioWrite(pin, value)
  if(value == HIGH)
  {
    gpioWrite(pin, PI_ON);
  }
  else
  {
    gpioWrite(pin, PI_OFF);
  }
}

unsigned long millis()
{
  //Declare a variable to store current time
  struct timeval RHCurrentTime;
  //Get current time
  gettimeofday(&RHCurrentTime,NULL);
  //Calculate the difference between our start time and the end time
  unsigned long difference = ((RHCurrentTime.tv_sec - RHStartTime.tv_sec) * 1000);
  difference += ((RHCurrentTime.tv_usec - RHStartTime.tv_usec)/1000);
  //Return the calculated value
  return difference;
}

void delay (unsigned long ms)
{
  //Implement Delay function
  struct timespec ts;
  ts.tv_sec=0;
  ts.tv_nsec=(ms * 1000);
  nanosleep(&ts,&ts);
}

long random(long min, long max)
{
  long diff = max - min;
  long ret = diff * rand() + min;
  return ret;
}

//******************************
//* Attach Interupt
//* Emulate Arduino Function
//******************************

void attachInterrupt(unsigned char pin, void (*handler)(void), int mode)
{
    switch(mode)
    {
        case CHANGE:
            gpioSetISRFunc(pin, EITHER_EDGE, 0, (void (*)(int,int,unsigned int))handler);
            break;
        case RISING:
            gpioSetISRFunc(pin, RISING_EDGE, 0, (void (*)(int,int,unsigned int))handler);
            break;
        case FALLING:
            gpioSetISRFunc(pin, FALLING_EDGE, 0, (void (*)(int,int,unsigned int))handler);
            break;
        default:
            break;
    }
}

void SerialSimulator::begin(int baud)
{
  //No implementation neccesary - Serial emulation on Linux = standard console
  //
  //Initialize a timestamp for millis calculation - we do this here as well in case SPI
  //isn't used for some reason
  gettimeofday(&RHStartTime, NULL);
}

size_t SerialSimulator::println(const char* s)
{
  size_t charsPrinted = 0;
  charsPrinted = print(s);
  printf("\n");
  return charsPrinted + 1;
}

size_t SerialSimulator::print(const char* s)
{
  return (size_t)printf(s);
}

size_t SerialSimulator::print(unsigned int n, int base)
{
  if (base == DEC)
    return (size_t)printf("%d", n);
  else if (base == HEX)
    return (size_t)printf("%02x", n);
  else if (base == OCT)
    return (size_t)printf("%o", n);
  // TODO: BIN
  else
    return 0;
}

size_t SerialSimulator::print(char ch)
{
  return (size_t)printf("%c", ch);
}

size_t SerialSimulator::println(char ch)
{
  return (size_t)printf("%c\n", ch);
}

size_t SerialSimulator::print(unsigned char ch, int base)
{
  return print((unsigned int)ch, base);
}

size_t SerialSimulator::println(unsigned char ch, int base)
{
  size_t charsPrinted = 0;
  charsPrinted = print((unsigned int)ch, base);
  printf("\n");
  return charsPrinted + 1;
}

#endif
