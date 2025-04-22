// RHSPIDriver.cpp
//
// Copyright (C) 2014 Mike McCauley
// $Id: RHSPIDriver.cpp,v 1.13 2020/08/04 09:02:14 mikem Exp $

#include <RHSPIDriver.h>

RHSPIDriver::RHSPIDriver(uint8_t slaveSelectPin, RHGenericSPI& spi)
    : 
    _spi(spi),
    _slaveSelectPin(slaveSelectPin)
{
}

bool RHSPIDriver::init()
{
    // start the SPI library with the default speeds etc:
    // On Arduino Due this defaults to SPI1 on the central group of 6 SPI pins
    _spi.begin();

#if (RH_PLATFORM == RH_PLATFORM_ARDUINO && defined(ARDUINO_LORA_E5_MINI))
    SubGhz.setResetActive(false);
//    while (SubGhz.isBusy()) ; // Wait for radio to wake up, hmm sometimes hangs forever!
#endif

    // Initialise the slave select pin
    // On Maple, this must be _after_ spi.begin

    // Sometimes we dont want to work the _slaveSelectPin here
    if (_slaveSelectPin != 0xff)
	pinMode(_slaveSelectPin, OUTPUT);

    deselectSlave();

    // This delay is needed for ATMega and maybe some others, but
    // 100ms is too long for STM32L0, and somehow can cause the USB interface to fail
    // in some versions of the core.
#if (RH_PLATFORM == RH_PLATFORM_STM32L0) && (defined STM32L082xx || defined STM32L072xx)
    delay(10);
#else
    delay(100);
#endif
    
    return true;
}

uint8_t RH_INTERRUPT_ATTR RHSPIDriver::spiRead(uint8_t reg)
{
    uint8_t val = 0;
    ATOMIC_BLOCK_START;
    beginTransaction();
    _spi.transfer(reg & ~RH_SPI_WRITE_MASK); // Send the address with the write mask off
    val = _spi.transfer(0); // The written value is ignored, reg value is read
    endTransaction();
    ATOMIC_BLOCK_END;
    return val;
}

uint8_t RH_INTERRUPT_ATTR RHSPIDriver::spiWrite(uint8_t reg, uint8_t val)
{
    uint8_t status = 0;
    ATOMIC_BLOCK_START;
    beginTransaction();
    status = _spi.transfer(reg | RH_SPI_WRITE_MASK); // Send the address with the write mask on
    _spi.transfer(val); // New value follows
    // Based on https://forum.pjrc.com/attachment.php?attachmentid=10948&d=1499109224
    // Need this delay from some processors when running fast:
    delayMicroseconds(1);
    endTransaction();
    ATOMIC_BLOCK_END;
    return status;
}

uint8_t RH_INTERRUPT_ATTR RHSPIDriver::spiBurstRead(uint8_t reg, uint8_t* dest, uint8_t len)
{
    uint8_t status = 0;
    ATOMIC_BLOCK_START;
    beginTransaction();
    status = _spi.transfer(reg & ~RH_SPI_WRITE_MASK); // Send the start address with the write mask off
    while (len--)
	*dest++ = _spi.transfer(0);
    endTransaction();
    ATOMIC_BLOCK_END;
    return status;
}

uint8_t RH_INTERRUPT_ATTR RHSPIDriver::spiBurstWrite(uint8_t reg, const uint8_t* src, uint8_t len)
{
    uint8_t status = 0;
    ATOMIC_BLOCK_START;
    beginTransaction();
    status = _spi.transfer(reg | RH_SPI_WRITE_MASK); // Send the start address with the write mask on
    while (len--)
	_spi.transfer(*src++);
    endTransaction();
    ATOMIC_BLOCK_END;
    return status;
}

void RHSPIDriver::setSlaveSelectPin(uint8_t slaveSelectPin)
{
    _slaveSelectPin = slaveSelectPin;
}

void RHSPIDriver::spiUsingInterrupt(uint8_t interruptNumber)
{
    _spi.usingInterrupt(interruptNumber);
}

void  RHSPIDriver::beginTransaction()
{
    _spi.beginTransaction();
    selectSlave();
}

void  RHSPIDriver::endTransaction()
{
    deselectSlave();
    _spi.endTransaction();
}

// Some platforms (ABZ) need to override just selectSlave and deselectSlave
void RHSPIDriver::selectSlave()
{
    if (_slaveSelectPin != 0xff)
	digitalWrite(_slaveSelectPin, LOW);
}
    
void RHSPIDriver::deselectSlave()
{
    if (_slaveSelectPin != 0xff)
	digitalWrite(_slaveSelectPin, HIGH);
}
