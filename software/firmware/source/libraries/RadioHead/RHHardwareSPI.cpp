// RHHardwareSPI.cpp
// Author: Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2011 Mike McCauley
// Contributed by Joanna Rutkowska
// $Id: RHHardwareSPI.cpp,v 1.29 2020/08/04 09:02:14 mikem Exp $

#include <RHHardwareSPI.h>

#ifdef RH_HAVE_HARDWARE_SPI

// Declare a single default instance of the hardware SPI interface class
RHHardwareSPI hardware_spi;


// This is very ugly and there should be a better way. Problem is that
// not all types hardware/platform dependent of SPI classes have the
// same name, so its hard to abstract it away on a per-instance basis
#if (RH_PLATFORM == RH_PLATFORM_STM32) // Maple etc
// Declare an SPI interface to use
HardwareSPI SPI(1);
#elif (RH_PLATFORM == RH_PLATFORM_STM32STD) // STM32F4 Discovery
// Declare an SPI interface to use
HardwareSPI SPI(1);
#elif (RH_PLATFORM == RH_PLATFORM_MONGOOSE_OS) // Mongoose OS platform
HardwareSPI SPI(1);
#elif (RH_PLATFORM == RH_PLATFORM_STM32L0) && (defined STM32L082xx || defined STM32L072xx)
 extern SPIClass radio_spi; // Created in RH_ABZ.cpp
 #define SPI radio_spi
#elif (RH_PLATFORM == RH_PLATFORM_ESP32 && defined(RH_ESP32_USE_HSPI))
 SPIClass SPI_HSPI(HSPI);
 #define SPI SPI_HSPI
#elif defined(RAK4630) // RAKwireless RAK4630
 extern SPIClass SPI_LORA(NRF_SPIM2, 45, 43, 44);
 #define SPI SPI_LORA
#elif defined(ARDUINO_RASPBERRY_PI_PICO)   || \
      defined(ARDUINO_RASPBERRY_PI_PICO_W) || \
      defined(ARDUINO_RASPBERRY_PI_PICO_2) || \
      defined(ARDUINO_RASPBERRY_PI_PICO_2W)
 /* Waveshare Pico-LoRa-SX1262-868M, SPI1 */
 #define SPI SPI1
#endif

// Arduino Due has default SPI pins on central SPI headers, and not on 10, 11, 12, 13
// as per other Arduinos
// http://21stdigitalhome.blogspot.com.au/2013/02/arduino-due-hardware-spi.html
#if defined (__arm__) && !defined(CORE_TEENSY) && !defined(SPI_CLOCK_DIV16) && !defined(RH_PLATFORM_NRF52)
 // Arduino Due in 1.5.5 has no definitions for SPI dividers
 // SPI clock divider is based on MCK of 84MHz  
 #define SPI_CLOCK_DIV16 (VARIANT_MCK/84000000) // 1MHz
 #define SPI_CLOCK_DIV8  (VARIANT_MCK/42000000) // 2MHz
 #define SPI_CLOCK_DIV4  (VARIANT_MCK/21000000) // 4MHz
 #define SPI_CLOCK_DIV2  (VARIANT_MCK/10500000) // 8MHz
 #define SPI_CLOCK_DIV1  (VARIANT_MCK/5250000)  // 16MHz
#endif

RHHardwareSPI::RHHardwareSPI(Frequency frequency, BitOrder bitOrder, DataMode dataMode)
    :
    RHGenericSPI(frequency, bitOrder, dataMode)
{
}

uint8_t RHHardwareSPI::transfer(uint8_t data) 
{
    return SPI.transfer(data);
}

#if (RH_PLATFORM == RH_PLATFORM_MONGOOSE_OS)
uint8_t RHHardwareSPI::transfer2B(uint8_t byte0, uint8_t byte1)
{
    return SPI.transfer2B(byte0, byte1);
}

uint8_t RHHardwareSPI::spiBurstRead(uint8_t reg, uint8_t* dest, uint8_t len)
{
    return SPI.spiBurstRead(reg, dest, len);
}

uint8_t RHHardwareSPI::spiBurstWrite(uint8_t reg, const uint8_t* src, uint8_t len)
{
    uint8_t status = SPI.spiBurstWrite(reg, src, len);
    return status;
}
#endif

void RHHardwareSPI::attachInterrupt() 
{
#ifdef RH_HAVE_SPI_ATTACH_INTERRUPT
    SPI.attachInterrupt();
#endif
}

void RHHardwareSPI::detachInterrupt() 
{
#ifdef RH_HAVE_SPI_ATTACH_INTERRUPT
    SPI.detachInterrupt();
#endif
}
    
void RHHardwareSPI::begin() 
{
#if defined(SPI_HAS_TRANSACTION)
    // Perhaps this is a uniform interface for SPI?
    // Currently Teensy and ESP32 only
   uint32_t frequency;
   if (_frequency == Frequency16MHz)
       frequency = 16000000;
   else if (_frequency == Frequency8MHz)
       frequency = 8000000;
   else if (_frequency == Frequency4MHz)
       frequency = 4000000;
   else if (_frequency == Frequency2MHz)
       frequency = 2000000;
   else
       frequency = 1000000;

#if ((RH_PLATFORM == RH_PLATFORM_ARDUINO) && defined (__arm__) && (defined(ARDUINO_SAM_DUE) || defined(ARDUINO_ARCH_SAMD))) || defined(ARDUINO_ARCH_NRF52) || defined(ARDUINO_ARCH_STM32) || defined(ARDUINO_UNOR4_MINIMA)  || defined(ARDUINO_UNOR4_WIFI) || defined(ARDUINO_ARCH_STM32L0) || defined(NRF52) || defined (ARDUINO_ARCH_RP2040)
    // Arduino Due in 1.5.5 has its own BitOrder :-(
    // So too does Arduino Zero
    // So too does rogerclarkmelbourne/Arduino_STM32
    // So too does GrumpyOldPizza/ArduinoCore-stm32l0
    // So too does RPI Pico
    ::BitOrder bitOrder;
// This no longer relevant: new versions is uint8_t
//#elif (RH_PLATFORM == RH_PLATFORM_ATTINY_MEGA)
//   ::BitOrder bitOrder;
#else
    uint8_t bitOrder;
#endif

   if (_bitOrder == BitOrderLSBFirst)
       bitOrder = LSBFIRST;
   else
       bitOrder = MSBFIRST;
   
    uint8_t dataMode;
    if (_dataMode == DataMode0)
	dataMode = SPI_MODE0;
    else if (_dataMode == DataMode1)
	dataMode = SPI_MODE1;
    else if (_dataMode == DataMode2)
	dataMode = SPI_MODE2;
    else if (_dataMode == DataMode3)
	dataMode = SPI_MODE3;
    else
	dataMode = SPI_MODE0;

    // Save the settings for use in transactions
   _settings = SPISettings(frequency, bitOrder, dataMode);
   SPI.begin();

#else // SPI_HAS_TRANSACTION
   
    // Sigh: there are no common symbols for some of these SPI options across all platforms
 #if (RH_PLATFORM == RH_PLATFORM_ARDUINO)   || (RH_PLATFORM == RH_PLATFORM_UNO32) || (RH_PLATFORM == RH_PLATFORM_CHIPKIT_CORE || RH_PLATFORM == RH_PLATFORM_NRF52)
    uint8_t dataMode;
    if (_dataMode == DataMode0)
	dataMode = SPI_MODE0;
    else if (_dataMode == DataMode1)
	dataMode = SPI_MODE1;
    else if (_dataMode == DataMode2)
	dataMode = SPI_MODE2;
    else if (_dataMode == DataMode3)
	dataMode = SPI_MODE3;
    else
	dataMode = SPI_MODE0;
  #if (RH_PLATFORM == RH_PLATFORM_ARDUINO) && defined(__arm__) && defined(CORE_TEENSY)
    // Temporary work-around due to problem where avr_emulation.h does not work properly for the setDataMode() cal
    SPCR &= ~SPI_MODE_MASK;
  #else
    #if ((RH_PLATFORM == RH_PLATFORM_ARDUINO) && defined (__arm__) && defined(ARDUINO_ARCH_SAMD)) || defined(ARDUINO_ARCH_NRF52)
    // Zero requires begin() before anything else :-)
    SPI.begin();
    #endif

    SPI.setDataMode(dataMode);
  #endif

  #if ((RH_PLATFORM == RH_PLATFORM_ARDUINO) && defined (__arm__) && (defined(ARDUINO_SAM_DUE) || defined(ARDUINO_ARCH_SAMD))) || defined(ARDUINO_ARCH_NRF52) || defined (ARDUINO_ARCH_STM32) || defined(ARDUINO_ARCH_STM32F4)  || defined(ARDUINO_ARCH_STM32F1)
    // Arduino Due in 1.5.5 has its own BitOrder :-(
    // So too does Arduino Zero
    // So too does rogerclarkmelbourne/Arduino_STM32
    // So too does stm32duino F1, F4
    ::BitOrder bitOrder;
  #else
    uint8_t bitOrder;
  #endif
    if (_bitOrder == BitOrderLSBFirst)
	bitOrder = LSBFIRST;
    else
	bitOrder = MSBFIRST;
    SPI.setBitOrder(bitOrder);
    uint8_t divider;
    switch (_frequency)
    {
	case Frequency1MHz:
	default:
  #if F_CPU == 8000000
	    divider = SPI_CLOCK_DIV8;
  #else
	    divider = SPI_CLOCK_DIV16;
  #endif
	    break;

	case Frequency2MHz:
  #if F_CPU == 8000000
	    divider = SPI_CLOCK_DIV4;
  #else
	    divider = SPI_CLOCK_DIV8;
  #endif
	    break;

	case Frequency4MHz:
  #if F_CPU == 8000000
	    divider = SPI_CLOCK_DIV2;
  #else
	    divider = SPI_CLOCK_DIV4;
  #endif
	    break;

	case Frequency8MHz:
	    divider = SPI_CLOCK_DIV2; // 4MHz on an 8MHz Arduino
	    break;

	case Frequency16MHz:
	    divider = SPI_CLOCK_DIV2; // Not really 16MHz, only 8MHz. 4MHz on an 8MHz Arduino
	    break;

    }
    
    SPI.setClockDivider(divider);

    SPI.begin();
    // Teensy requires it to be set _after_ begin()
    SPI.setClockDivider(divider);

 #elif (RH_PLATFORM == RH_PLATFORM_STM32) // Maple etc
    spi_mode dataMode;
    // Hmmm, if we do this as a switch, GCC on maple gets v confused!
    if (_dataMode == DataMode0)
	dataMode = SPI_MODE_0;
    else if (_dataMode == DataMode1)
	dataMode = SPI_MODE_1;
    else if (_dataMode == DataMode2)
	dataMode = SPI_MODE_2;
    else if (_dataMode == DataMode3)
	dataMode = SPI_MODE_3;
    else
	dataMode = SPI_MODE_0;

    uint32 bitOrder;
    if (_bitOrder == BitOrderLSBFirst)
	bitOrder = LSBFIRST;
    else
	bitOrder = MSBFIRST;

    SPIFrequency frequency; // Yes, I know these are not exact equivalents.
    switch (_frequency)
    {
	case Frequency1MHz:
	default:
	    frequency = SPI_1_125MHZ;
	    break;

	case Frequency2MHz:
	    frequency = SPI_2_25MHZ;
	    break;

	case Frequency4MHz:
	    frequency = SPI_4_5MHZ;
	    break;

	case Frequency8MHz:
	    frequency = SPI_9MHZ;
	    break;

	case Frequency16MHz:
	    frequency = SPI_18MHZ;
	    break;

    }
    SPI.begin(frequency, bitOrder, dataMode);

 #elif (RH_PLATFORM == RH_PLATFORM_STM32STD) // STM32F4 discovery
    uint8_t dataMode;
    if (_dataMode == DataMode0)
	dataMode = SPI_MODE0;
    else if (_dataMode == DataMode1)
	dataMode = SPI_MODE1;
    else if (_dataMode == DataMode2)
	dataMode = SPI_MODE2;
    else if (_dataMode == DataMode3)
	dataMode = SPI_MODE3;
    else
	dataMode = SPI_MODE0;

    uint32_t bitOrder;
    if (_bitOrder == BitOrderLSBFirst)
	bitOrder = LSBFIRST;
    else
	bitOrder = MSBFIRST;

    SPIFrequency frequency; // Yes, I know these are not exact equivalents.
    switch (_frequency)
    {
	case Frequency1MHz:
	default:
	    frequency = SPI_1_3125MHZ;
	    break;

	case Frequency2MHz:
	    frequency = SPI_2_625MHZ;
	    break;

	case Frequency4MHz:
	    frequency = SPI_5_25MHZ;
	    break;

	case Frequency8MHz:
	    frequency = SPI_10_5MHZ;
	    break;

	case Frequency16MHz:
	    frequency = SPI_21_0MHZ;
	    break;

    }
    SPI.begin(frequency, bitOrder, dataMode);

 #elif (RH_PLATFORM == RH_PLATFORM_STM32F2) // Photon
    uint8_t dataMode;
    if (_dataMode == DataMode0)
	dataMode = SPI_MODE0;
    else if (_dataMode == DataMode1)
	dataMode = SPI_MODE1;
    else if (_dataMode == DataMode2)
	dataMode = SPI_MODE2;
    else if (_dataMode == DataMode3)
	dataMode = SPI_MODE3;
    else
	dataMode = SPI_MODE0;
    SPI.setDataMode(dataMode);
    if (_bitOrder == BitOrderLSBFirst)
	SPI.setBitOrder(LSBFIRST);
    else
	SPI.setBitOrder(MSBFIRST);

    switch (_frequency)
    {
	case Frequency1MHz:
	default:
	    SPI.setClockSpeed(1, MHZ);
	    break;

	case Frequency2MHz:
	    SPI.setClockSpeed(2, MHZ);
	    break;

	case Frequency4MHz:
	    SPI.setClockSpeed(4, MHZ);
	    break;

	case Frequency8MHz:
	    SPI.setClockSpeed(8, MHZ);
	    break;

	case Frequency16MHz:
	    SPI.setClockSpeed(16, MHZ);
	    break;
    }

//      SPI.setClockDivider(SPI_CLOCK_DIV4);  // 72MHz / 4MHz = 18MHz
//      SPI.setClockSpeed(1, MHZ);
      SPI.begin();

 #elif RH_PLATFORM == (RH_PLATFORM_ESP8266) || (RH_PLATFORM == RH_PLATFORM_ESP32)
     // Requires SPI driver for ESP8266 from https://github.com/esp8266/Arduino/tree/master/libraries/SPI
     // Which ppears to be in Arduino Board Manager ESP8266 Community version 2.1.0
     // Contributed by David Skinner
     // begin comes first 
     SPI.begin();

     // datamode
     switch ( _dataMode )
     { 
	 case DataMode1:
	     SPI.setDataMode ( SPI_MODE1 );
	     break;
	 case DataMode2:
	     SPI.setDataMode ( SPI_MODE2 );
	     break;
	 case DataMode3:
	     SPI.setDataMode ( SPI_MODE3 );
	     break;
	 case DataMode0:
	 default:
	     SPI.setDataMode ( SPI_MODE0 );
	     break;
     }

     // bitorder
     SPI.setBitOrder(_bitOrder == BitOrderLSBFirst ? LSBFIRST : MSBFIRST);

     // frequency (this sets the divider)
     switch (_frequency)
     {
	 case Frequency1MHz:
	 default:
	     SPI.setFrequency(1000000);
	     break;
	 case Frequency2MHz:
	     SPI.setFrequency(2000000);
	     break;
	 case Frequency4MHz:
	     SPI.setFrequency(4000000);
	     break;
	 case Frequency8MHz:
	     SPI.setFrequency(8000000);
	     break;
	 case Frequency16MHz:
	     SPI.setFrequency(16000000);
	     break;
     }

 #elif (RH_PLATFORM == RH_PLATFORM_RASPI) // Raspberry PI
  uint8_t dataMode;
  if (_dataMode == DataMode0)
    dataMode = BCM2835_SPI_MODE0;
  else if (_dataMode == DataMode1)
    dataMode = BCM2835_SPI_MODE1;
  else if (_dataMode == DataMode2)
    dataMode = BCM2835_SPI_MODE2;
  else if (_dataMode == DataMode3)
    dataMode = BCM2835_SPI_MODE3;

  uint8_t bitOrder;
  if (_bitOrder == BitOrderLSBFirst)
    bitOrder = BCM2835_SPI_BIT_ORDER_LSBFIRST;
  else
    bitOrder = BCM2835_SPI_BIT_ORDER_MSBFIRST;

  uint32_t divider;
  switch (_frequency)
  {
    case Frequency1MHz:
    default:
      divider = BCM2835_SPI_CLOCK_DIVIDER_256;
      break;
    case Frequency2MHz:
      divider = BCM2835_SPI_CLOCK_DIVIDER_128;
      break;
    case Frequency4MHz:
      divider = BCM2835_SPI_CLOCK_DIVIDER_64;
      break;
    case Frequency8MHz:
      divider = BCM2835_SPI_CLOCK_DIVIDER_32;
      break;
    case Frequency16MHz:
      divider = BCM2835_SPI_CLOCK_DIVIDER_16;
      break;
  }
  SPI.begin(divider, bitOrder, dataMode);
 #elif (RH_PLATFORM == RH_PLATFORM_MONGOOSE_OS)
    uint8_t dataMode   = SPI_MODE0;
    uint32_t frequency = 4000000; //!!! ESP32/NRF902 works ok at 4MHz but not at 8MHz SPI clock.
    uint32_t bitOrder  = MSBFIRST;

    if (_dataMode == DataMode0) {
        dataMode = SPI_MODE0;
    } else if (_dataMode == DataMode1) {
        dataMode = SPI_MODE1;
    } else if (_dataMode == DataMode2) {
        dataMode = SPI_MODE2;
    } else if (_dataMode == DataMode3) {
        dataMode = SPI_MODE3;
    }

    if (_bitOrder == BitOrderLSBFirst) {
        bitOrder = LSBFIRST;
    }

    if (_frequency == Frequency4MHz)
        frequency = 4000000;
    else if (_frequency == Frequency2MHz)
        frequency = 2000000;
    else
        frequency = 1000000;

    SPI.begin(frequency, bitOrder, dataMode);
 #else
  #warning RHHardwareSPI does not support this platform yet. Consider adding it and contributing a patch.
 #endif

#endif // SPI_HAS_TRANSACTION
}

void RHHardwareSPI::end() 
{
    return SPI.end();
}

void RHHardwareSPI::beginTransaction()
{
#if defined(SPI_HAS_TRANSACTION)
    SPI.beginTransaction(_settings);
#endif
}

void RHHardwareSPI::endTransaction()
{
#if defined(SPI_HAS_TRANSACTION)
    SPI.endTransaction();
#endif
}

void RHHardwareSPI::usingInterrupt(uint8_t interrupt)
{
#if defined(SPI_HAS_TRANSACTION) && !defined(RH_MISSING_SPIUSINGINTERRUPT)
    SPI.usingInterrupt(interrupt);
#endif
    (void)interrupt;
}

#endif
